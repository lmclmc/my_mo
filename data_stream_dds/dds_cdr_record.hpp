#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/// 同一类型的样本共享 MCAP Schema 描述。
struct DdsRecordingType {
  std::string name;
  std::string definition;
  std::string schema_encoding = "omgidl";
};

/// 写盘队列持有一条样本的完整副本。订阅线程可以在入队后归还 DDS 借用缓冲区。
struct DdsCdrRecord {
  std::shared_ptr<const DdsRecordingType> type;
  std::string topic;
  int64_t timestamp_ns;  // 与实时数据一致的 Unix epoch 纳秒时间。
  std::vector<std::byte> payload;  // 完整 CDR 数据，包含四字节封装头。
};
