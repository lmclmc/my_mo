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
  FileDescriptor() = default;
  FileDescriptor(const FileDescriptor&) = delete;
  FileDescriptor& operator=(const FileDescriptor&) = delete;
  int value = -1;
  ~FileDescriptor() { if (value >= 0) ::close(value); }
};

std::runtime_error ioError(const std::string& operation) {
  return std::runtime_error(operation + "：" + std::error_code(errno, std::generic_category()).message());
}

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

std::string recordingName(int slot) {
  const auto digits = std::to_string(slot);
  return std::string(kPrefix) + std::string(6 - digits.size(), '0') + digits + std::string(kSuffix);
}

size_t recordBytes(const DdsRecordingType& type, std::string_view topic, std::span<const std::byte> payload) {
  // Count shared schema storage conservatively for each queued reference too.
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

  Impl(const DdsRecordingSettings& configuration, Notice callback)
      : settings(configuration), notice(std::move(callback)),
        max_file_bytes(static_cast<uint64_t>(configuration.max_file_size_mb) * 1024 * 1024) {
    if (const auto error = settings.validationError(); !error.empty()) throw std::runtime_error(error);
    directory = fs::absolute(settings.directory).lexically_normal();
    fs::create_directories(directory);
    // A persistent lock file prevents two plugin instances from sharing a ring.
    const auto lock_path = directory / ".cyclonedds_recording.lock";
    directory_lock.value = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (directory_lock.value < 0) throw ioError("无法打开录制目录锁 " + lock_path.string());
    if (::flock(directory_lock.value, LOCK_EX | LOCK_NB) != 0)
      throw ioError("无法锁定录制目录（可能正被另一个录制实例使用）");
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

  ~Impl() { stop(); }

  void submit(std::shared_ptr<const DdsRecordingType> type, std::string_view topic,
              int64_t timestamp_ns, std::span<const std::byte> payload) {
    bool overflow = false;
    try {
      {
        std::lock_guard lock(mutex);
        if (!accepting) return;
        if (!type || payload.size() < 4) throw std::runtime_error("CDR 样本或类型描述不完整");
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

  void stop() {
    {
      std::lock_guard lock(mutex);
      accepting = false;
    }
    ready.notify_one();
    if (worker.joinable()) worker.join();
  }

  void closeFile() {
    if (!output) return;
    output->close();
    output.reset();
  }

  void openNextFile() {
    closeFile();
    // Recovered files participate in the same limit after reconnect/restart.
    // When a smaller size is configured, discard oversized old recordings too.
    if (!recovered_limits_applied) {
      for (auto it = files.begin(); it != files.end();) {
        if (it->bytes > max_file_bytes) {
          removeFile(it->path);
          it = files.erase(it);
        } else ++it;
      }
      recovered_limits_applied = true;
    }
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

  static void removeFile(const fs::path& path) {
    if (!fs::is_regular_file(fs::symlink_status(path)))
      throw std::runtime_error("录制文件已被外部修改：" + path.string());
    fs::remove(path);
  }

  void writeRecord(const DdsCdrRecord& record) {
    if (record.timestamp_ns < 0) throw std::runtime_error("MCAP 接收时间不能早于 Unix epoch");
    if (!record.type || record.type->name.empty() || record.type->idl.empty() || record.payload.size() < 4)
      throw std::runtime_error("CDR 录制样本或 IDL 类型描述不完整");
    if (!output || !output->fits(record, max_file_bytes)) {
      // Include the complete header, schema, channel, statistics and footer
      // before removing any old file or writing the sample.
      if (DdsMcapWriter::minimumFileSize(record) > max_file_bytes)
        throw std::runtime_error("一条样本及 MCAP 文件结构超过单文件上限，请增大单文件大小后重新连接");
      openNextFile();
    }
    output->write(record);
    ++written_records;
  }

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
      // A rejected/oversized sample must not leave previously saved samples
      // without a footer. A failed disk sink refuses further footer writes.
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

DdsRecorder::DdsRecorder(const DdsRecordingSettings& settings, Notice notice)
    : impl_(std::make_unique<Impl>(settings, std::move(notice))) {}
DdsRecorder::~DdsRecorder() = default;
void DdsRecorder::submit(std::shared_ptr<const DdsRecordingType> type, std::string_view topic,
                         int64_t timestamp_ns, std::span<const std::byte> payload) {
  impl_->submit(std::move(type), topic, timestamp_ns, payload);
}
void DdsRecorder::stop() { impl_->stop(); }
