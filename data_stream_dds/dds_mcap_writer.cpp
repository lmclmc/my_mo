#include "dds_mcap_writer.hpp"
#include "dds_cdr_record.hpp"

// Keep the uncompressed MCAP writer private to this plugin. Its upstream
// implementation lives in this translation unit only.
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
  bool operator()(const DdsRecordingType* lhs, const DdsRecordingType* rhs) const {
    if (lhs == rhs) return false;
    if (lhs->name != rhs->name) return lhs->name < rhs->name;
    return lhs->idl < rhs->idl;
  }
};

mcap::McapWriterOptions writerOptions() {
  mcap::McapWriterOptions options("");
  options.library = "PlotJuggler CycloneDDS / libmcap " MCAP_LIBRARY_VERSION;
  // Direct records keep the final file size predictable before each sample.
  options.noChunking = true;
  options.compression = mcap::Compression::None;
  return options;
}

mcap::Schema makeSchema(const DdsRecordingType& type, mcap::SchemaId id) {
  mcap::Schema schema(type.name, "omgidl", type.idl);
  schema.id = id;
  return schema;
}

mcap::Channel makeChannel(std::string_view topic, mcap::ChannelId id, mcap::SchemaId schema_id) {
  mcap::Channel channel(topic, "cdr", schema_id,
      {{"source", "CycloneDDS"}, {"timestamp_source", "host_receive_time"},
       {"serialization", "DDS_CDR"}, {"schema_source", "Complete_TypeObjects"}});
  channel.id = id;
  return channel;
}

class SizeSink final : public mcap::IWritable {
 public:
  void handleWrite(const std::byte*, uint64_t size) override { bytes += size; }
  uint64_t size() const override { return bytes; }
  void end() override {}
  uint64_t bytes = 0;
};

template <typename T>
uint64_t recordSize(const T& record) {
  SizeSink sink;
  return mcap::McapWriter::write(sink, record);
}

uint64_t headerSize() {
  const auto options = writerOptions();
  SizeSink sink;
  mcap::McapWriter::writeMagic(sink);
  mcap::McapWriter::write(sink, mcap::Header{options.profile, options.library});
  return sink.size();
}

uint64_t messageSize(uint64_t payload_bytes) {
  mcap::Message message{};
  message.dataSize = payload_bytes;
  return 9 + mcap::McapWriter::getRecordSize(message);
}

// Serialize the exact closing records into a counter. Schema/Channel records
// repeat in the summary; Statistics grows for every newly used channel.
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

// Upstream FileWriter checks I/O failures with assert(). Latch failures here
// instead, so release builds can report disk/flush/close errors as well.
class FileSink final : public mcap::IWritable {
 public:
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

  ~FileSink() override { end(); }

  void handleWrite(const std::byte* data, uint64_t size) override {
    if (error_.empty() && size) {
      if (!stream_) {
        error_ = "MCAP 文件已经关闭";
      } else if (std::fwrite(data, 1, static_cast<size_t>(size), stream_) != size) {
        latch("写入 MCAP 文件失败");
      }
    }
    // MCAP bookkeeping must advance consistently even after a latched failure.
    bytes_ += size;
  }

  uint64_t size() const override { return bytes_; }

  void flush() override {
    if (stream_ && std::fflush(stream_) != 0) latch("刷新 MCAP 文件失败");
  }

  void end() override {
    if (!stream_) return;
    flush();
    if (error_.empty()) {
      int result;
      do { result = ::fsync(::fileno(stream_)); } while (result < 0 && errno == EINTR);
      if (result < 0) latch("同步 MCAP 文件失败");
    }
    if (std::fclose(stream_) != 0) latch("关闭 MCAP 文件失败");
    stream_ = nullptr;
  }

  void check() const {
    if (!error_.empty()) throw std::runtime_error(error_);
  }

 private:
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
  explicit Impl(const std::filesystem::path& path) : sink(path) {
    try {
      writer.open(sink, writerOptions());
      sink.check();
    } catch (...) {
      writer.terminate();
      throw;
    }
  }

  // Closing is explicit and checked. In an error path never let ~McapWriter
  // attempt an unchecked second footer write into a failed/closed sink.
  ~Impl() { writer.terminate(); }

  uint64_t tailWith(const mcap::Channel& channel, uint64_t new_schema_bytes) const {
    auto statistics = writer.statistics();
    statistics.channelMessageCounts.emplace(channel.id, 0);
    return tailSize(schema_bytes + new_schema_bytes, channel_bytes + recordSize(channel), statistics);
  }

  bool fits(const DdsCdrRecord& record, uint64_t limit) const {
    if (closed) return false;
    uint64_t extra = messageSize(record.payload.size());
    uint64_t tail = tail_bytes;
    const auto schema = schemas.find(record.type.get());
    const bool new_schema = schema == schemas.end();
    if (new_schema && schemas.size() >= std::numeric_limits<mcap::SchemaId>::max()) return false;
    const auto schema_id = new_schema ? static_cast<mcap::SchemaId>(schemas.size() + 1) : schema->second.id;
    if (channels.find({record.topic, schema_id}) == channels.end()) {
      if (channels.size() >= std::numeric_limits<mcap::ChannelId>::max()) return false;
      const uint64_t added_schema_bytes = new_schema ? recordSize(makeSchema(*record.type, schema_id)) : 0;
      const auto channel = makeChannel(record.topic, static_cast<mcap::ChannelId>(channels.size() + 1), schema_id);
      extra += recordSize(channel);
      extra += added_schema_bytes;
      tail = tailWith(channel, added_schema_bytes);
    }
    return sink.size() <= limit && tail <= limit - sink.size() &&
           extra <= limit - sink.size() - tail;
  }

  void write(const DdsCdrRecord& record) {
    if (closed) throw std::runtime_error("MCAP 文件已经关闭");
    auto schema = schemas.find(record.type.get());
    const auto schema_id = schema == schemas.end() ? static_cast<mcap::SchemaId>(schemas.size() + 1) : schema->second.id;
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
      auto channel = makeChannel(record.topic, static_cast<mcap::ChannelId>(channels.size() + 1), schema->second.id);
      tail_bytes = tailWith(channel, added_schema_bytes);
      schema_bytes += added_schema_bytes;
      writer.addChannel(channel);
      channel_bytes += recordSize(channel);
      found = channels.emplace(key, channel.id).first;
    }
    mcap::Message message{};
    message.channelId = found->second;
    message.logTime = static_cast<uint64_t>(record.timestamp_ns);
    message.publishTime = message.logTime;  // Keep host receive time consistent with the live curves.
    message.dataSize = record.payload.size();
    message.data = record.payload.data();
    const auto status = writer.write(message);
    if (!status.ok()) throw std::runtime_error("写入 MCAP 样本失败：" + status.message);
    sink.check();
  }

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

  // The writer must be destroyed before its sink.
  FileSink sink;
  mcap::McapWriter writer;
  struct SchemaEntry {
    std::shared_ptr<const DdsRecordingType> type;  // Pins the pointer used as the key.
    mcap::SchemaId id;
  };
  std::map<const DdsRecordingType*, SchemaEntry, RecordingTypeLess> schemas;
  std::map<std::pair<std::string, mcap::SchemaId>, mcap::ChannelId> channels;
  uint64_t schema_bytes = 0, channel_bytes = 0, tail_bytes = 0;
  bool closed = false;
};

DdsMcapWriter::DdsMcapWriter(const std::filesystem::path& path) : impl_(std::make_unique<Impl>(path)) {}
DdsMcapWriter::~DdsMcapWriter() = default;

uint64_t DdsMcapWriter::minimumFileSize(const DdsCdrRecord& record) {
  const auto schema = makeSchema(*record.type, 1);
  const auto channel = makeChannel(record.topic, 1, 1);
  mcap::Statistics statistics{};
  statistics.channelMessageCounts.emplace(1, 1);
  const auto channel_bytes = recordSize(channel);
  const auto schema_bytes = recordSize(schema);
  return headerSize() + schema_bytes + channel_bytes + messageSize(record.payload.size()) +
         tailSize(schema_bytes, channel_bytes, statistics);
}

bool DdsMcapWriter::fits(const DdsCdrRecord& record, uint64_t limit) const {
  return impl_->fits(record, limit);
}

void DdsMcapWriter::write(const DdsCdrRecord& record) {
  impl_->write(record);
}

void DdsMcapWriter::close() { impl_->close(); }
