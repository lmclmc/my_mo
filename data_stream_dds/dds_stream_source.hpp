#pragma once

#include <cstdint>
#include <memory>
#include <pj_base/sdk/data_source_patterns.hpp>
#include <string>
#include <string_view>

#include "dds_dialog.hpp"
#include "dds_dynamic.hpp"

class DdsStreamSource : public PJ::StreamSourceBase {
 public:
  ~DdsStreamSource() override { onStop(); }

  PJ_borrowed_dialog_t getDialog() override { return PJ::borrowDialog(dialog_); }

  uint64_t extraCapabilities() const override {
    return PJ::kCapabilityDirectIngest | PJ::kCapabilityHasDialog | PJ::kCapabilitySupportsPause;
  }

  std::string saveConfig() const override { return dialog_.saveConfig(); }
  PJ::Status loadConfig(std::string_view config_json) override;
  PJ::Status onStart() override;
  PJ::Status onPoll() override;
  void onStop() override;

 private:
  DdsDialog dialog_;
  std::unique_ptr<DdsDynamicStream> dds_stream_;
};
