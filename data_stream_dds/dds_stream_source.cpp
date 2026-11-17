#include "dds_stream_source.hpp"

#include <exception>
#include <type_traits>
#include <variant>
#include <vector>

PJ::Status DdsStreamSource::loadConfig(std::string_view config_json) {
  if (!dialog_.loadConfig(config_json)) {
    runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kError, "Warning: Invalid config JSON");
    runtimeHost().showError("DDS import failed", "Warning: Invalid config JSON");
    return PJ::unexpected(std::string("invalid config JSON"));
  }
  return PJ::okStatus();
}

PJ::Status DdsStreamSource::onStart() {
  if (dds_stream_) return PJ::unexpected(std::string("DDS stream is already running"));
  try {
    dds_stream_ = std::make_unique<DdsDynamicStream>(dialog_.ddsSettings());
    return PJ::okStatus();
  } catch (const std::exception& error) {
    return PJ::unexpected(std::string("DDS connection failed: ") + error.what());
  }
}

// DDS I/O stays on the worker; only owned records are drained into the host here.
PJ::Status DdsStreamSource::onPoll() {
  if (dds_stream_) {
    for (const auto& notice : dds_stream_->takeNotices()) {
      runtimeHost().reportMessage(notice.warning ? PJ::DataSourceMessageLevel::kWarning
                                                : PJ::DataSourceMessageLevel::kInfo, notice.text);
    }
    for (const auto& record : dds_stream_->takeRecords()) {
      auto topic = writeHost().ensureTopic(record.topic);
      if (!topic) return PJ::unexpected(topic.error());
      std::vector<PJ::sdk::NamedFieldValue> fields;
      fields.reserve(record.fields.size());
      for (const auto& field : record.fields) {
        const auto value = std::visit([](const auto& item) -> PJ::sdk::ValueRef {
          using T = std::decay_t<decltype(item)>;
          if constexpr (std::is_same_v<T, std::string>) return std::string_view(item);
          else return item;
        }, field.value);
        fields.push_back({field.name, value});
      }
      const auto status = writeHost().appendRecord(*topic, PJ::Timestamp{record.timestamp_ns},
          PJ::Span<const PJ::sdk::NamedFieldValue>(fields.data(), fields.size()));
      if (!status) return status;
    }
  }
  return PJ::okStatus();
}

void DdsStreamSource::onStop() {
  if (!dds_stream_) return;
  dds_stream_->stop();
  for (const auto& notice : dds_stream_->takeNotices()) {
    runtimeHost().reportMessage(notice.warning ? PJ::DataSourceMessageLevel::kWarning
                                              : PJ::DataSourceMessageLevel::kInfo, notice.text);
  }
  dds_stream_.reset();
}

PJ_DATA_SOURCE_PLUGIN(DdsStreamSource, kDdsManifest)
PJ_DIALOG_PLUGIN(DdsDialog, kDdsManifest)
