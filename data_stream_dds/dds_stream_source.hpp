#pragma once

#include <cstdint>
#include <chrono>
#include <map>
#include <memory>
#include <pj_base/sdk/data_source_patterns.hpp>
#include <string>
#include <string_view>

#include "dds_dialog.hpp"
#include "dds_dynamic.hpp"
#include "dds_robot_model.hpp"

class DdsStreamSource : public PJ::StreamSourceBase {
 public:
  /// 析构前结束订阅、录制和 3D 发布，释放数据源持有的资源。
  ~DdsStreamSource() override { onStop(); }

  /// 返回由数据源持有的连接对话框的借用句柄，宿主可在数据源存活期间使用。
  PJ_borrowed_dialog_t getDialog() override { return PJ::borrowDialog(dialog_); }

  /// 返回宿主能力位，声明直接写入、委托解析、连接对话框和暂停支持。
  uint64_t extraCapabilities() const override {
    return PJ::kCapabilityDirectIngest | PJ::kCapabilityDelegatedIngest |
           PJ::kCapabilityHasDialog | PJ::kCapabilitySupportsPause;
  }

  /// 返回连接对话框当前 DDS 配置的 JSON 字符串，供宿主保存到布局。
  std::string saveConfig() const override { return dialog_.saveConfig(); }

  /// 恢复 config_json 中的 DDS 配置。返回成功状态或包含校验错误的状态。
  PJ::Status loadConfig(std::string_view config_json) override;

  /// 按当前配置启动订阅、可选录制和 3D 数据发布，返回成功状态或启动失败原因。
  PJ::Status onStart() override;

  /// 把后台接收结果写入宿主，推进 3D 发布，返回成功状态或本次写入、发布错误。
  PJ::Status onPoll() override;

  /// 停止后台订阅和录制，重置本次会话的 3D 状态。可重复调用，无返回值。
  void onStop() override;

 private:
  /// 推送 3D 模型和姿态消息。force 为 true 时立即发布，否则按各自的周期和更新状态发布。
  /// 返回成功状态或模型转换、消息推送错误。
  PJ::Status publishRobot(bool force = false);
  DdsDialog dialog_;
  std::unique_ptr<DdsDynamicStream> dds_stream_;
  std::unique_ptr<DdsRobotModel> robot_model_;
  PJ::ParserBindingHandle robot_description_handle_{}, robot_tf_handle_{};
  std::shared_ptr<const std::vector<uint8_t>> robot_description_bytes_;
  std::map<std::string, std::string> robot_errors_;
  std::chrono::steady_clock::time_point last_robot_publish_{}, last_description_publish_{};
  int64_t robot_timestamp_ns_ = 0;
  bool robot_dirty_ = false;
};
