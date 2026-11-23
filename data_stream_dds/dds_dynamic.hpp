#pragma once

#include "dds_settings.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <variant>
#include <vector>

// 字段值拥有自己的存储，归还 DDS 样本后仍可以跨线程读取。
using DdsDynamicValue = std::variant<float, double, int8_t, int16_t, int32_t, int64_t,
                                     uint8_t, uint16_t, uint32_t, uint64_t, bool, std::string>;
struct DdsDynamicField {
  std::string name;
  DdsDynamicValue value;
};
struct DdsDynamicRecord {
  std::string topic;
  int64_t timestamp_ns;  // 上位机订阅线程读取样本的 Unix epoch 纳秒时间。
  std::vector<DdsDynamicField> fields;
};
struct DdsDynamicNotice {
  bool warning;
  std::string text;
};

// 后台线程发现发布端、解析 Complete TypeObject 并创建 Reader。
// 宿主在 onPoll 中取出已经复制到队列的字段和通知。
class DdsDynamicStream {
 public:
  // 复制 settings 并启动 DDS 接收线程，按配置启动录制。话题列表为空或初始化失败时抛出异常。
  explicit DdsDynamicStream(const DdsSettings& settings);
  // 停止接收并等待录制队列写完，然后释放 DDS 资源。
  ~DdsDynamicStream();
  // 删除拷贝构造，确保接收线程和 DDS 资源由单个流管理。
  DdsDynamicStream(const DdsDynamicStream&) = delete;
  // 删除拷贝赋值，避免多个流共享接收线程及其资源。
  DdsDynamicStream& operator=(const DdsDynamicStream&) = delete;

  // 取走当前缓存的样本并恢复队列容量，返回的字段数据由调用方拥有。
  std::vector<DdsDynamicRecord> takeRecords();
  // 取走当前缓存的通知，返回的文本由调用方拥有。
  std::vector<DdsDynamicNotice> takeNotices();
  // 将实时模型已发布的 TF 交给录制队列。录制关闭时直接返回。
  void recordRobotTf(int64_t timestamp_ns, std::span<const uint8_t> payload);
  // 唤醒并等待接收线程退出，再清空录制队列并关闭文件。可在同一控制线程上重复调用。
  void stop();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
