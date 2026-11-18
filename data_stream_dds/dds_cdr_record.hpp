#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

/// 同一订阅的样本共享类型描述。写盘线程仅依赖以下类型名和 IDL 文本即可写出 Schema。
struct DdsRecordingType {
  std::string name;
  std::string idl;
};

/// 写盘队列持有一条样本的完整副本。订阅线程可以在入队后归还 DDS 借用缓冲区。
struct DdsCdrRecord {
  std::shared_ptr<const DdsRecordingType> type;
  std::string topic;
  int64_t timestamp_ns;  // 上位机订阅读取时的 Unix epoch 纳秒时间，与实时曲线的横轴一致。
  std::vector<std::byte> payload;  // DDS 原始 CDR 数据，包含四字节封装头。
};
