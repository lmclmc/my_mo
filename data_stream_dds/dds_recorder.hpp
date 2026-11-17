#pragma once

#include "dds_recording_settings.hpp"
#include "dds_cdr_record.hpp"

#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>

// Owns a bounded queue and a disk worker independent of DDS and plotting.
class DdsRecorder {
 public:
  using Notice = std::function<void(bool warning, std::string text)>;

  DdsRecorder(const DdsRecordingSettings& settings, Notice notice);
  ~DdsRecorder();
  DdsRecorder(const DdsRecorder&) = delete;
  DdsRecorder& operator=(const DdsRecorder&) = delete;

  // Copies borrowed CDR synchronously, after checking the queue budget.
  void submit(std::shared_ptr<const DdsRecordingType> type, std::string_view topic,
              int64_t timestamp_ns, std::span<const std::byte> payload);
  void stop();  // Drain queued samples and close the current file.

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
