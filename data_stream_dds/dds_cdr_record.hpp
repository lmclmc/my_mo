#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Shared once per subscription; the disk worker never depends on DDS objects.
struct DdsRecordingType {
  std::string name;
  std::string idl;
};

struct DdsCdrRecord {
  std::shared_ptr<const DdsRecordingType> type;
  std::string topic;
  int64_t timestamp_ns;  // Host receive time, also used for the live curves.
  std::vector<std::byte> payload;  // Includes the four-byte DDS encapsulation header.
};
