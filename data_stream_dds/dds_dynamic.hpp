#pragma once

#include "dds_settings.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <variant>
#include <vector>

// All values own their storage; no DDS loan or string_view crosses threads.
using DdsDynamicValue = std::variant<float, double, int8_t, int16_t, int32_t, int64_t,
                                     uint8_t, uint16_t, uint32_t, uint64_t, bool, std::string>;
struct DdsDynamicField {
  std::string name;
  DdsDynamicValue value;
};
struct DdsDynamicRecord {
  std::string topic;
  int64_t timestamp_ns;  // Host subscriber read time, nanoseconds since Unix epoch.
  std::vector<DdsDynamicField> fields;
};
struct DdsDynamicNotice {
  bool warning;
  std::string text;
};

// Discovers publishers, resolves Complete TypeObjects and creates readers at runtime.
// DDS entities and decoding stay on the worker; the host drains owned records onPoll.
class DdsDynamicStream {
 public:
  explicit DdsDynamicStream(const DdsSettings& settings);
  ~DdsDynamicStream();
  DdsDynamicStream(const DdsDynamicStream&) = delete;
  DdsDynamicStream& operator=(const DdsDynamicStream&) = delete;

  std::vector<DdsDynamicRecord> takeRecords();
  std::vector<DdsDynamicNotice> takeNotices();
  void stop();  // Stop receiving, then drain the recording worker.

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
