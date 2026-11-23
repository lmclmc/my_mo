#pragma once

#include "dds_recording_settings.hpp"
#include "dds_cdr_record.hpp"

#include <functional>
#include <memory>
#include <span>
#include <string>
#include <string_view>

/// 将 CDR 样本放入有界队列，由独立线程写入滚动 MCAP 文件。
class DdsRecorder {
 public:
  /// 录制状态回调。warning 表示是否为警告，text 为可直接显示的中文消息。
  /// 构造、样本提交和写盘线程均可调用此回调，调用方负责同步共享状态。
  using Notice = std::function<void(bool warning, std::string text)>;

  /// 校验设置、占用录制目录并启动写盘线程。配置无效、目录操作失败或线程创建失败会抛出异常。
  /// @param settings 录制目录、单文件大小与保留数量。构造时复制该设置。
  /// @param notice 必须可调用的状态回调。录制器接管回调实例。
  DdsRecorder(const DdsRecordingSettings& settings, Notice notice);

  /// 停止接收样本，等待写盘线程退出后释放目录锁。
  ~DdsRecorder();

  /// 写盘线程和目录锁归单个实例管理，禁止复制构造。
  DdsRecorder(const DdsRecorder&) = delete;

  /// 禁止通过复制赋值转移录制状态。
  DdsRecorder& operator=(const DdsRecorder&) = delete;

  /// 校验队列容量并同步复制样本供后台写入。停止收录后忽略后续提交，无返回值。
  /// @param type 与该样本对应的类型描述，以共享所有权传入。
  /// @param topic 话题名，成功入队时复制。
  /// @param timestamp_ns 样本的 Unix epoch 纳秒时间。
  /// @param payload 完整 CDR 视图，包含四字节封装头，提交返回后即可释放原缓冲区。
  void submit(std::shared_ptr<const DdsRecordingType> type, std::string_view topic,
              int64_t timestamp_ns, std::span<const std::byte> payload);

  /// 停止接收新样本，等待已排队样本处理完毕并关闭当前文件。写盘错误通过回调报告。
  void stop();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
