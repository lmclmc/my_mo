#include "dds_recorder.hpp"
#include "dds_cdr_record.hpp"
#include "dds_mcap_writer.hpp"

#include <algorithm>
#include <cerrno>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fcntl.h>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {
namespace fs = std::filesystem;
constexpr std::string_view kPrefix = "cyclonedds_record_";
constexpr std::string_view kSuffix = ".mcap";
constexpr size_t kMaxQueuedBytes = 64 * 1024 * 1024;
constexpr size_t kMaxQueuedRecords = 4096;

struct FileDescriptor {
  // 以无效描述符初始化，取得的描述符写入 value 后由本对象管理。
  FileDescriptor() = default;
  // 一个描述符只对应一个所有者，禁止复制构造。
  FileDescriptor(const FileDescriptor&) = delete;
  // 禁止用复制赋值替换已持有的描述符。
  FileDescriptor& operator=(const FileDescriptor&) = delete;
  int value = -1;
  // 析构时关闭有效描述符，释放与其关联的目录锁。
  ~FileDescriptor() { if (value >= 0) ::close(value); }
};

// 用 operation 和当前 errno 构造 I/O 异常对象。由调用方决定何时抛出。
std::runtime_error ioError(const std::string& operation) {
  return std::runtime_error(operation + "：" + std::error_code(errno, std::generic_category()).message());
}

// 从文件名 name 解析六位录制编号。命名不匹配或编号为零时返回 0。
int recordingSlot(const std::string& name) {
  if (!name.starts_with(kPrefix) || !name.ends_with(kSuffix) ||
      name.size() != kPrefix.size() + 6 + kSuffix.size()) return 0;
  const auto digits = std::string_view(name).substr(kPrefix.size(), 6);
  int slot = 0;
  for (const char digit : digits) {
    if (digit < '0' || digit > '9') return 0;
    slot = slot * 10 + digit - '0';
  }
  return slot;
}

// 将正编号 slot 格式化为带六位补零编号的 MCAP 文件名。
std::string recordingName(int slot) {
  const auto digits = std::to_string(slot);
  return std::string(kPrefix) + std::string(6 - digits.size(), '0') + digits + std::string(kSuffix);
}

// 按类型描述 type、话题 topic 和载荷 payload 估算每条排队样本占用的字节数。
size_t recordBytes(const DdsRecordingType& type, std::string_view topic, std::span<const std::byte> payload) {
  // 每条样本都计入共享类型的容量，避免类型对象与字符串的额外占用导致预算低估。
  return sizeof(DdsCdrRecord) + topic.size() + payload.size() +
         sizeof(type) + type.name.capacity() + type.idl.capacity();
}
}  // namespace

struct DdsRecorder::Impl {
  struct RecordingFile {
    int slot;
    fs::path path;
    fs::file_time_type modified;
    uintmax_t bytes;
  };
  struct Pending {
    DdsCdrRecord record;
    size_t bytes;
  };

  // 保存 configuration，接管状态回调 callback 并启动写盘线程。准备失败时抛出异常。
  Impl(const DdsRecordingSettings& configuration, Notice callback)
      : settings(configuration), notice(std::move(callback)),
        max_file_bytes(static_cast<uint64_t>(configuration.max_file_size_mb) * 1024 * 1024) {
    if (const auto error = settings.validationError(); !error.empty()) throw std::runtime_error(error);
    directory = fs::absolute(settings.directory).lexically_normal();
    fs::create_directories(directory);
    // 锁文件保留在目录中。持有描述符期间，其他录制器无法同时删除或复用同一组文件。
    const auto lock_path = directory / ".cyclonedds_recording.lock";
    directory_lock.value = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (directory_lock.value < 0) throw ioError("无法打开录制目录锁 " + lock_path.string());
    if (::flock(directory_lock.value, LOCK_EX | LOCK_NB) != 0)
      throw ioError("无法锁定录制目录（可能正被另一个录制实例使用）");
    // 只接管录制器命名的普通文件。按修改时间排列历史文件，便于轮转时从最旧的开始删除。
    // 同一修改时间用编号确定顺序，编号可复用，不能单独反映文件新旧。
    std::vector<RecordingFile> existing;
    for (const auto& entry : fs::directory_iterator(directory)) {
      const int slot = recordingSlot(entry.path().filename().string());
      if (!slot) continue;
      if (!fs::is_regular_file(entry.symlink_status()))
        throw std::runtime_error("录制文件名被非普通文件占用：" + entry.path().string());
      existing.push_back({slot, entry.path(), entry.last_write_time(), entry.file_size()});
    }
    std::sort(existing.begin(), existing.end(), [](const auto& lhs, const auto& rhs) {
      if (lhs.modified != rhs.modified) return lhs.modified < rhs.modified;
      return lhs.slot < rhs.slot;
    });
    files.assign(existing.begin(), existing.end());
    notice(false, "DDS MCAP / CDR 录制已启用：" + directory.string() + " · 单文件 " +
                  std::to_string(settings.max_file_size_mb) + " MB · 最多 " +
                  std::to_string(settings.max_files) + " 个文件（滚动覆盖）");
    worker = std::thread([this] { run(); });
  }

  // 先停止并回收写盘线程，再销毁目录锁和队列等成员。
  ~Impl() { stop(); }

  // 以 type、topic、上位机读取时间 timestamp_ns 和借用的 CDR payload 构造排队样本。
  // 成功时复制样本，无返回值。校验失败、队列超限或分配失败时停止接收并发出通知。
  void submit(std::shared_ptr<const DdsRecordingType> type, std::string_view topic,
              int64_t timestamp_ns, std::span<const std::byte> payload) {
    bool overflow = false;
    try {
      {
        std::lock_guard lock(mutex);
        if (!accepting) return;
        if (!type || payload.size() < 4) throw std::runtime_error("CDR 样本或类型描述不完整");
        // 先在队列锁内检查条数和字节预算，再复制原始载荷。超限后保留已入队的记录供写盘线程处理。
        const auto bytes = recordBytes(*type, topic, payload);
        if (queue.size() >= kMaxQueuedRecords || bytes > kMaxQueuedBytes - queued_bytes) {
          accepting = false;
          overflow = true;
        } else {
          DdsCdrRecord record{std::move(type), std::string(topic), timestamp_ns,
                              std::vector<std::byte>(payload.begin(), payload.end())};
          queue.push_back({std::move(record), bytes});
          queued_bytes += bytes;
        }
      }
      ready.notify_one();
      if (overflow)
        notice(true, "DDS 录制缓冲区已满，已停止接收新的录制数据，已有缓存继续写完。订阅和绘图继续运行。");
    } catch (const std::exception& error) {
      {
        std::lock_guard lock(mutex);
        accepting = false;
      }
      ready.notify_one();
      notice(true, std::string("DDS 录制缓存失败，已停止录制：") + error.what());
    }
  }

  // 禁止入队并唤醒写盘线程以处理剩余记录，阻塞直到线程退出。
  void stop() {
    {
      std::lock_guard lock(mutex);
      accepting = false;
    }
    ready.notify_one();
    if (worker.joinable()) worker.join();
  }

  // 写完当前文件的摘要和文件尾并释放写入器。无活动文件时直接返回，I/O 错误向上传递。
  void closeFile() {
    if (!output) return;
    output->close();
    output.reset();
  }

  // 结束当前文件，按目录的滚动策略腾出编号并创建新文件。轮转或创建失败时抛出异常。
  void openNextFile() {
    closeFile();
    // 首次写入前按本次限额清理扫描到的历史文件。用户调小限额时，旧的超限文件也会被删除。
    if (!recovered_limits_applied) {
      for (auto it = files.begin(); it != files.end();) {
        if (it->bytes > max_file_bytes) {
          removeFile(it->path);
          it = files.erase(it);
        } else ++it;
      }
      recovered_limits_applied = true;
    }
    // 优先删除最旧的文件，再从 1 起选取空闲的最小编号，新文件始终追加到队列末尾。
    while (files.size() >= static_cast<size_t>(settings.max_files)) {
      removeFile(files.front().path);
      files.pop_front();
    }
    int slot = 1;
    while (std::any_of(files.begin(), files.end(), [slot](const auto& file) { return file.slot == slot; })) ++slot;
    const auto path = directory / recordingName(slot);
    output = std::make_unique<DdsMcapWriter>(path);
    files.push_back({slot, path, {}, 0});
  }

  // 重新核对 path 指向的条目是普通文件后删除。路径被替换或文件系统操作失败时抛出异常。
  static void removeFile(const fs::path& path) {
    if (!fs::is_regular_file(fs::symlink_status(path)))
      throw std::runtime_error("录制文件已被外部修改：" + path.string());
    fs::remove(path);
  }

  // 校验并写入已拥有载荷的 record，按需要轮转。样本无效、文件容量不足或写盘失败时抛出异常。
  void writeRecord(const DdsCdrRecord& record) {
    if (record.timestamp_ns < 0) throw std::runtime_error("MCAP 接收时间不能早于 Unix epoch");
    if (!record.type || record.type->name.empty() || record.type->idl.empty() || record.payload.size() < 4)
      throw std::runtime_error("CDR 录制样本或 IDL 类型描述不完整");
    if (!output || !output->fits(record, max_file_bytes)) {
      // 先确认单条样本与文件头、类型、通道、统计和文件尾一起能放进新文件，再执行会删除旧文件的轮转。
      if (DdsMcapWriter::minimumFileSize(record) > max_file_bytes)
        throw std::runtime_error("一条样本及 MCAP 文件结构超过单文件上限，请增大单文件大小后重新连接");
      openNextFile();
    }
    output->write(record);
    ++written_records;
  }

  // 写盘线程的入口。依次消费队首记录，正常停止时写完剩余样本并关闭文件，异常时通知订阅侧。
  void run() {
    try {
      while (true) {
        Pending pending;
        {
          std::unique_lock lock(mutex);
          ready.wait(lock, [this] { return !accepting || !queue.empty(); });
          if (queue.empty()) break;
          pending = std::move(queue.front());
          queue.pop_front();
          queued_bytes -= pending.bytes;
        }
        // 解锁后执行文件操作，使提交样本的线程仅等待队列操作。
        writeRecord(pending.record);
      }
      closeFile();
      notice(false, "DDS 录制结束：本次已写入 " + std::to_string(written_records) +
                    " 条样本 · " + directory.string());
    } catch (const std::exception& error) {
      std::string reason = error.what();
      {
        std::lock_guard lock(mutex);
        accepting = false;
        queue.clear();
        queued_bytes = 0;
      }
      // 超限等样本错误发生后，尽量补齐现有文件的尾部以保留此前写好的记录。
      // 如果底层已经记录磁盘错误，关闭会报告失败，保留两段错误原因供界面显示。
      try {
        closeFile();
      } catch (const std::exception& close_error) {
        reason += std::string("；结束 MCAP 文件失败：") + close_error.what();
      }
      output.reset();
      notice(true, "DDS 录制已停止：" + reason + "。订阅和绘图继续运行。");
    }
  }

  DdsRecordingSettings settings;
  Notice notice;
  fs::path directory;
  uint64_t max_file_bytes, written_records = 0;
  FileDescriptor directory_lock;
  std::unique_ptr<DdsMcapWriter> output;
  std::deque<RecordingFile> files;
  std::mutex mutex;
  std::condition_variable ready;
  std::deque<Pending> queue;
  size_t queued_bytes = 0;
  bool accepting = true;
  bool recovered_limits_applied = false;
  std::thread worker;
};

// 将 settings 和通知回调 notice 交给录制实现，初始化失败时让构造者收到异常。
DdsRecorder::DdsRecorder(const DdsRecordingSettings& settings, Notice notice)
    : impl_(std::make_unique<Impl>(settings, std::move(notice))) {}
// 由实现对象完成停录和写盘线程回收。
DdsRecorder::~DdsRecorder() = default;

// 转交类型、话题、读取时刻和 CDR 数据，队列负责复制借用的载荷。
void DdsRecorder::submit(std::shared_ptr<const DdsRecordingType> type, std::string_view topic,
                         int64_t timestamp_ns, std::span<const std::byte> payload) {
  impl_->submit(std::move(type), topic, timestamp_ns, payload);
}
// 等待实现对象处理剩余记录并结束写盘。
void DdsRecorder::stop() { impl_->stop(); }
