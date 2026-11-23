#include "dds_mcap_writer.hpp"
#include "dds_cdr_record.hpp"

// 在当前编译单元引入 libmcap 的实现，并关闭 LZ4 与 Zstd 压缩依赖。
#define MCAP_IMPLEMENTATION
#define MCAP_COMPRESSION_NO_LZ4
#define MCAP_COMPRESSION_NO_ZSTD
#define MCAP_PUBLIC
#include <mcap/writer.hpp>

#include <array>
#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>

namespace {
struct RecordingTypeLess {
  // 只有类型名、编码和定义都相同的类型才共享一个 Schema。
  bool operator()(const DdsRecordingType* lhs, const DdsRecordingType* rhs) const {
    if (lhs == rhs) return false;
    if (lhs->name != rhs->name) return lhs->name < rhs->name;
    if (lhs->schema_encoding != rhs->schema_encoding) return lhs->schema_encoding < rhs->schema_encoding;
    return lhs->definition < rhs->definition;
  }
};

// 返回文件头标识及写入选项，大小估算与实际写入共用这一组设置。
mcap::McapWriterOptions writerOptions() {
  mcap::McapWriterOptions options("");
  options.library = "PlotJuggler CycloneDDS / libmcap " MCAP_LIBRARY_VERSION;
  // 直接写入未压缩的 MCAP 记录，写入前就可以精确计算封尾后的文件大小。
  options.noChunking = true;
  options.compression = mcap::Compression::None;
  return options;
}

// 将类型描述转为 MCAP Schema，并赋予调用方指定的 id。
mcap::Schema makeSchema(const DdsRecordingType& type, mcap::SchemaId id) {
  mcap::Schema schema(type.name, type.schema_encoding, type.definition);
  schema.id = id;
  return schema;
}

// 用 topic 创建 CDR 通道，返回编号为 id 并关联 schema_id 的记录。元数据记录编码和时间来源。
mcap::Channel makeChannel(std::string_view topic, const DdsRecordingType& type,
                          mcap::ChannelId id, mcap::SchemaId schema_id) {
  mcap::Channel channel(topic, "cdr", schema_id);
  if (type.schema_encoding == "omgidl") {
    channel.metadata = {{"source", "CycloneDDS"}, {"timestamp_source", "host_receive_time"},
                        {"serialization", "DDS_CDR"}, {"schema_source", "Complete_TypeObjects"}};
  } else {
    channel.metadata = {{"source", "PlotJuggler"}, {"timestamp_source", "tf_header_time"}};
  }
  channel.id = id;
  return channel;
}

class SizeSink final : public mcap::IWritable {
 public:
  // 忽略数据指针，仅累计本次写入占用的 size 个字节。
  void handleWrite(const std::byte*, uint64_t size) override { bytes += size; }

  // 返回累计的字节数。
  uint64_t size() const override { return bytes; }

  // 计数对象没有文件可关闭，结束写入时直接返回。
  void end() override {}
  uint64_t bytes = 0;
};

// 用 libmcap 的实际编码过程计算 record 的完整字节数。
template <typename T>
uint64_t recordSize(const T& record) {
  SizeSink sink;
  return mcap::McapWriter::write(sink, record);
}

// 返回文件起始魔数和 Header 记录的总字节数。
uint64_t headerSize() {
  const auto options = writerOptions();
  SizeSink sink;
  mcap::McapWriter::writeMagic(sink);
  mcap::McapWriter::write(sink, mcap::Header{options.profile, options.library});
  return sink.size();
}

// 根据载荷字节数 payload_bytes 计算完整 Message 记录大小，包含操作码及长度前缀。
uint64_t messageSize(uint64_t payload_bytes) {
  mcap::Message message{};
  message.dataSize = payload_bytes;
  return 9 + mcap::McapWriter::getRecordSize(message);
}

// 按关闭时的写入顺序计算文件尾大小，返回从 DataEnd 到末尾魔数的字节数。
// schema_record_bytes 和 channel_record_bytes 为摘要中重复写入的类型与通道大小。
// statistics 包含各通道计数，每新增一个通道都会增加这条统计记录的大小。
uint64_t tailSize(uint64_t schema_record_bytes, uint64_t channel_record_bytes,
                  const mcap::Statistics& statistics) {
  SizeSink sink;
  mcap::McapWriter::write(sink, mcap::DataEnd{});
  sink.bytes += schema_record_bytes + channel_record_bytes;
  mcap::McapWriter::write(sink, statistics);
  for (const auto opcode : {mcap::OpCode::Schema, mcap::OpCode::Channel, mcap::OpCode::Statistics})
    mcap::McapWriter::write(sink, mcap::SummaryOffset{opcode, 0, 0});
  mcap::McapWriter::write(sink, mcap::Footer{0, 0}, true);
  mcap::McapWriter::writeMagic(sink);
  return sink.size();
}

// 缓存第一次写盘、刷新或关闭错误，使发布构建也能通过 check() 向调用端报告失败。
class FileSink final : public mcap::IWritable {
 public:
  // 独占创建 path 指向的文件，为文件流设置缓冲。创建或打开失败时抛出异常。
  explicit FileSink(const std::filesystem::path& path) {
    const int descriptor = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0644);
    if (descriptor < 0) {
      latch("创建 MCAP 文件失败 " + path.string());
      check();
    }
    stream_ = ::fdopen(descriptor, "wb");
    if (!stream_) {
      latch("打开 MCAP 文件流失败");
      ::close(descriptor);
      check();
    }
    std::setvbuf(stream_, buffer_.data(), _IOFBF, buffer_.size());
  }

  // 释放尚未关闭的文件流。
  ~FileSink() override { end(); }

  // 同步处理 data 指向的 size 个字节，无返回值。写入失败记录错误供 check() 报告。
  void handleWrite(const std::byte* data, uint64_t size) override {
    if (error_.empty() && size) {
      if (!stream_) {
        error_ = "MCAP 文件已经关闭";
      } else if (std::fwrite(data, 1, static_cast<size_t>(size), stream_) != size) {
        latch("写入 MCAP 文件失败");
      }
    }
    // 错误发生后继续按请求长度累计逻辑偏移，使 libmcap 的内部记账一致。实际写入结果由 check() 检查。
    bytes_ += size;
  }

  // 返回已请求写入的累计逻辑字节数。
  uint64_t size() const override { return bytes_; }

  // 将标准库缓冲的数据提交给内核，失败时保留首个错误。
  void flush() override {
    if (stream_ && std::fflush(stream_) != 0) latch("刷新 MCAP 文件失败");
  }

  // 刷新、同步并关闭文件流，可重复调用。I/O 错误通过 check() 报告。
  void end() override {
    if (!stream_) return;
    flush();
    if (error_.empty()) {
      // 刷新成功后再请求内核同步文件，系统调用被信号中断时重试。
      int result;
      do { result = ::fsync(::fileno(stream_)); } while (result < 0 && errno == EINTR);
      if (result < 0) latch("同步 MCAP 文件失败");
    }
    if (std::fclose(stream_) != 0) latch("关闭 MCAP 文件失败");
    stream_ = nullptr;
  }

  // 此前捕获到 I/O 错误时抛出异常，否则直接返回。
  void check() const {
    if (!error_.empty()) throw std::runtime_error(error_);
  }

 private:
  // 用操作说明 operation 和当前 errno 保存第一次 I/O 错误。
  void latch(const std::string& operation) {
    if (error_.empty())
      error_ = operation + "：" + std::error_code(errno, std::generic_category()).message();
  }

  std::array<char, 64 * 1024> buffer_{};
  std::FILE* stream_ = nullptr;
  uint64_t bytes_ = 0;
  std::string error_;
};
}  // namespace

struct DdsMcapWriter::Impl {
  // 创建 path 对应的文件并写入文件头。初始化失败时终止写入器并传播异常。
  explicit Impl(const std::filesystem::path& path) : sink(path) {
    try {
      writer.open(sink, writerOptions());
      sink.check();
    } catch (...) {
      writer.terminate();
      throw;
    }
  }

  // 正常封尾由 close() 显式执行并检查。析构时终止写入器，避免错误路径再次向已失败或关闭的流封尾。
  ~Impl() { writer.terminate(); }

  // 模拟加入 channel 及其新类型所占的 new_schema_bytes，返回更新后的文件尾字节数。
  uint64_t tailWith(const mcap::Channel& channel, uint64_t new_schema_bytes) const {
    auto statistics = writer.statistics();
    statistics.channelMessageCounts.emplace(channel.id, 0);
    return tailSize(schema_bytes + new_schema_bytes, channel_bytes + recordSize(channel), statistics);
  }

  // 预估 record 写入并封尾后能否留在当前文件。limit 是字节上限，容量与 ID 都足够时返回 true。
  bool fits(const DdsCdrRecord& record, uint64_t limit) const {
    if (closed) return false;
    uint64_t extra = messageSize(record.payload.size());
    uint64_t tail = tail_bytes;
    const auto schema = schemas.find(record.type.get());
    const bool new_schema = schema == schemas.end();
    if (new_schema && schemas.size() >= std::numeric_limits<mcap::SchemaId>::max()) return false;
    const auto schema_id = new_schema ? static_cast<mcap::SchemaId>(schemas.size() + 1) : schema->second.id;
    // 新通道会增加数据区的类型或通道记录，也会扩大文件摘要。已登记的通道可继续使用缓存的文件尾大小。
    if (channels.find({record.topic, schema_id}) == channels.end()) {
      if (channels.size() >= std::numeric_limits<mcap::ChannelId>::max()) return false;
      const uint64_t added_schema_bytes = new_schema ? recordSize(makeSchema(*record.type, schema_id)) : 0;
      const auto channel = makeChannel(record.topic, *record.type,
                                       static_cast<mcap::ChannelId>(channels.size() + 1), schema_id);
      extra += recordSize(channel);
      extra += added_schema_bytes;
      tail = tailWith(channel, added_schema_bytes);
    }
    // 逐项比较剩余字节，避免无符号总和溢出。
    return sink.size() <= limit && tail <= limit - sink.size() &&
           extra <= limit - sink.size() - tail;
  }

  // 注册 record 所需的 Schema 和 Channel 并写入其原始载荷。ID 上限或 I/O 失败会抛出异常。
  void write(const DdsCdrRecord& record) {
    if (closed) throw std::runtime_error("MCAP 文件已经关闭");
    auto schema = schemas.find(record.type.get());
    const auto schema_id = schema == schemas.end() ? static_cast<mcap::SchemaId>(schemas.size() + 1) : schema->second.id;
    // 同一类型可以被多个话题复用。同名话题的类型变化时，使用不同的 Channel 区分其 Schema。
    const auto key = std::make_pair(record.topic, schema_id);
    auto found = channels.find(key);
    if (found == channels.end()) {
      if (channels.size() >= std::numeric_limits<mcap::ChannelId>::max())
        throw std::runtime_error("MCAP 通道数量超过格式上限");
      uint64_t added_schema_bytes = 0;
      if (schema == schemas.end()) {
        if (schemas.size() >= std::numeric_limits<mcap::SchemaId>::max())
          throw std::runtime_error("MCAP 类型数量超过格式上限");
        auto definition = makeSchema(*record.type, static_cast<mcap::SchemaId>(schemas.size() + 1));
        added_schema_bytes = recordSize(definition);
        writer.addSchema(definition);
        schema = schemas.emplace(record.type.get(), SchemaEntry{record.type, definition.id}).first;
      }
      auto channel = makeChannel(record.topic, *record.type,
                                 static_cast<mcap::ChannelId>(channels.size() + 1), schema->second.id);
      tail_bytes = tailWith(channel, added_schema_bytes);
      schema_bytes += added_schema_bytes;
      writer.addChannel(channel);
      channel_bytes += recordSize(channel);
      found = channels.emplace(key, channel.id).first;
    }
    // 两个 MCAP 时间字段均使用对应的实时数据时间，便于回放对齐。
    mcap::Message message{};
    message.channelId = found->second;
    message.logTime = static_cast<uint64_t>(record.timestamp_ns);
    message.publishTime = message.logTime;
    message.dataSize = record.payload.size();
    message.data = record.payload.data();
    const auto status = writer.write(message);
    if (!status.ok()) throw std::runtime_error("写入 MCAP 样本失败：" + status.message);
    sink.check();
  }

  // 封尾并检查关闭过程中的 I/O。失败后终止写入器、记录已关闭状态并向上传递异常。
  void close() {
    if (closed) return;
    try {
      sink.check();
      writer.close();
      closed = true;
      sink.check();
    } catch (...) {
      writer.terminate();
      closed = true;
      throw;
    }
  }

  // 成员按声明的逆序析构，writer 先于它引用的 sink 释放。
  FileSink sink;
  mcap::McapWriter writer;
  struct SchemaEntry {
    std::shared_ptr<const DdsRecordingType> type;  // 延长映射键所指向的类型描述的生命期。
    mcap::SchemaId id;
  };
  std::map<const DdsRecordingType*, SchemaEntry, RecordingTypeLess> schemas;
  std::map<std::pair<std::string, mcap::SchemaId>, mcap::ChannelId> channels;
  uint64_t schema_bytes = 0, channel_bytes = 0, tail_bytes = 0;
  bool closed = false;
};

// 将 path 交给实现对象创建并初始化文件，失败时传播异常。
DdsMcapWriter::DdsMcapWriter(const std::filesystem::path& path) : impl_(std::make_unique<Impl>(path)) {}

// 通过实现对象释放写入器和底层文件资源。
DdsMcapWriter::~DdsMcapWriter() = default;

// 返回只写入 record 时的完整文件字节数。类型编号和通道编号取首个有效值 1。
uint64_t DdsMcapWriter::minimumFileSize(const DdsCdrRecord& record) {
  const auto schema = makeSchema(*record.type, 1);
  const auto channel = makeChannel(record.topic, *record.type, 1, 1);
  mcap::Statistics statistics{};
  statistics.channelMessageCounts.emplace(1, 1);
  const auto channel_bytes = recordSize(channel);
  const auto schema_bytes = recordSize(schema);
  return headerSize() + schema_bytes + channel_bytes + messageSize(record.payload.size()) +
         tailSize(schema_bytes, channel_bytes, statistics);
}

// 返回当前文件能否在 limit 字节内写入 record 并封尾。
bool DdsMcapWriter::fits(const DdsCdrRecord& record, uint64_t limit) const {
  return impl_->fits(record, limit);
}

// 将 record 交给当前文件的实现写入，I/O 异常向调用方传播。
void DdsMcapWriter::write(const DdsCdrRecord& record) {
  impl_->write(record);
}

// 交由实现写入文件尾并关闭文件。
void DdsMcapWriter::close() { impl_->close(); }
