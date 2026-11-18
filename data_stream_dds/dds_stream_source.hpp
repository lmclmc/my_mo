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
  ~DdsStreamSource() override { onStop(); }

  PJ_borrowed_dialog_t getDialog() override { return PJ::borrowDialog(dialog_); }

  uint64_t extraCapabilities() const override {
    return PJ::kCapabilityDirectIngest | PJ::kCapabilityDelegatedIngest |
           PJ::kCapabilityHasDialog | PJ::kCapabilitySupportsPause;
  }

  std::string saveConfig() const override { return dialog_.saveConfig(); }
  PJ::Status loadConfig(std::string_view config_json) override;
  PJ::Status onStart() override;
  PJ::Status onPoll() override;
  void onStop() override;

 private:
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
