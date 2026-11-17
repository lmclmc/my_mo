#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

struct DdsCdrRecord;

// One independently readable MCAP file: CDR channels, OMG IDL schemas and summary.
class DdsMcapWriter {
 public:
  explicit DdsMcapWriter(const std::filesystem::path& path);
  ~DdsMcapWriter();
  DdsMcapWriter(const DdsMcapWriter&) = delete;
  DdsMcapWriter& operator=(const DdsMcapWriter&) = delete;

  static uint64_t minimumFileSize(const DdsCdrRecord& record);
  bool fits(const DdsCdrRecord& record, uint64_t limit) const;
  void write(const DdsCdrRecord& record);
  void close();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
