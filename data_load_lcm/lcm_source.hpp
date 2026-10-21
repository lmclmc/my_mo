#ifndef LCM_SOURCE_HPP
#define LCM_SOURCE_HPP

#include <pj_base/builtin/builtin_object.hpp>
#include <pj_base/sdk/data_source_patterns.hpp>
#include <pj_plugins/sdk/parser_array_policy.hpp>

#define MCAP_IMPLEMENTATION
#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm_coretypes.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include "mcap_dataset_metadata.hpp"
#include "lcmgen.h"
#include "mcap_helpers.hpp"
#include "mcap_manifest.hpp"
#include "robot_state_publisher.hpp"

#include "lcm_dialog.hpp"

// Vendored parallel-reader fork (contrib/mcap/). Pulled in after reader.hpp
// with MCAP_IMPLEMENTATION already defined, so parallel_reader.hpp's trailing
// #include "parallel_reader.inl" compiles the parallel-reader bodies here (the
// same .hpp/.inl split the rest of the library uses). message_byte_store.hpp
// adds the optional deferred-byte (hot/cold lazy) layer on top.
#include <mcap/message_byte_store.hpp>       // NOLINT(build/include_order)
#include <mcap/parallel_reader.hpp>          // NOLINT(build/include_order)
#include <mcap/unchunked_payload_store.hpp>  // NOLINT(build/include_order)

class CdrWriter
{
public:
  CdrWriter()
  {
    data_.reserve(640 * 480 * 3 + 128);

    // ROS2 CDR encapsulation header
    // little-endian CDR
    data_.push_back(0x00);
    data_.push_back(0x01);
    data_.push_back(0x00);
    data_.push_back(0x00);
  }

  void align(size_t alignment)
  {
    const size_t offset = data_.size();
    const size_t padding =
        (alignment - (offset % alignment)) % alignment;

    for (size_t i = 0; i < padding; ++i)
    {
      data_.push_back(0);
    }
  }

  void writeUint8(uint8_t value)
  {
    data_.push_back(value);
  }

  void writeUint32(uint32_t value)
  {
    align(4);

    data_.push_back(static_cast<uint8_t>(value & 0xff));
    data_.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
    data_.push_back(static_cast<uint8_t>((value >> 16) & 0xff));
    data_.push_back(static_cast<uint8_t>((value >> 24) & 0xff));
  }

  void writeInt32(int32_t value)
  {
    writeUint32(static_cast<uint32_t>(value));
  }

  void writeString(const std::string& value)
  {
    align(4);

    // CDR string length includes '\0'
    const uint32_t length =
        static_cast<uint32_t>(value.size() + 1);

    writeUint32(length);

    data_.insert(
        data_.end(),
        value.begin(),
        value.end());

    data_.push_back(0);
  }

  void writeByteSequence(
      const std::vector<uint8_t>& bytes)
  {
    align(4);

    writeUint32(
        static_cast<uint32_t>(bytes.size()));

    data_.insert(
        data_.end(),
        bytes.begin(),
        bytes.end());
  }

  const std::vector<uint8_t>& data() const
  {
    return data_;
  }

private:
  std::vector<uint8_t> data_;
};

class LcmSource : public PJ::FileSourceBase {
#ifdef PJ_TARGET_WASM
  static constexpr size_t kChunkCacheCapacityBytes = 32ULL * 1024 * 1024;
  static constexpr uint64_t kParallelImportBudgetBytes = 128ULL * 1024 * 1024;
  static constexpr uint64_t kParallelLookaheadBytes = 64ULL * 1024 * 1024;
  static constexpr uint64_t kMaxChunkUncompressedBytes = 128ULL * 1024 * 1024;
#else
  // Cold-path chunk cache budget, handed to the MessageByteStore. 128 MiB
  // comfortably holds tens of typical 4 MiB chunks so a scrubbing consumer
  // rarely re-decompresses, while staying well below host memory pressure.
  // Only post-import lazy pulls populate it — eager import-time fetches come
  // straight from the parallel reader's pinned chunk (zero-copy, no cache).
  static constexpr size_t kChunkCacheCapacityBytes = 128ULL * 1024 * 1024;
  // Parallel-import in-flight decompression budget.
  static constexpr uint64_t kParallelImportBudgetBytes = 256ULL * 1024 * 1024;
#endif
  // Progress / log throttling.
  static constexpr uint64_t kProgressUpdateInterval = 1000;
  // Push-failure threshold + log throttling. A handful of transient push
  // failures is acceptable (parser binding temporarily backpressured); a
  // sustained run means something structural is broken and the import
  // should abort rather than spam the message panel.
  static constexpr uint64_t kMaxConsecutivePushFailures = 100;
  static constexpr uint64_t kPushFailureLogInterval = 25;

public:
  LcmSource() {
    m_image_handle_ = PJ::ParserBindingHandle();
    m_tf_handle_ = PJ::ParserBindingHandle();
  }
  PJ_borrowed_dialog_t getDialog() override {
    return PJ::borrowDialog(dialog_);
  }

  uint64_t extraCapabilities() const override {
    return PJ::kCapabilityDelegatedIngest | PJ::kCapabilityHasDialog;
  }

  std::string saveConfig() const override {
    return dialog_.saveConfig();
  }

  PJ::Status loadConfig(std::string_view config_json) override;

  PJ::Status importData() override;

  void publishRobotState(const std::string&);
  void publishDepthImage(const lcm::LogEvent*, int, int, std::vector<uint8_t>&);
  void publishTF(const lcm::LogEvent*, const std::vector<geometry_msgs::msg::TransformStamped>&);
  void publishJointState(const lcm::LogEvent*, const sensor_msgs::msg::JointState&);

  //二级解析函数，调用lcm解析函数进行解析，调用不了的则递归调用一级解析函数处理
  int decode_type_2(const lcm::LogEvent*, int, const std::string&,
                    const std::string&, const std::string&,
                    std::unordered_map<std::string, int>&,
                    std::map<std::string, lcm_struct_t*>&,
                    const std::function<void(const std::string&, PJ::sdk::ValueRef)>&);
  //一级解析函数，解析LCM消息中，任意类型的结构体数据，解开结构体内部任意成员数据后，调用二级解析函数处理
  int decode_type_1(const lcm::LogEvent*, int, const std::string&, 
                    const std::string&, std::map<std::string, lcm_struct_t*>&,
                    const std::function<void(const std::string&, PJ::sdk::ValueRef)>&);

  void savePoint(const lcm::LogEvent*, const std::string&, PJ::sdk::ValueRef);

  uint32_t publishTfCount = 0;

private:
  LcmDialog dialog_;
  // Parser config from the embedded parser dialog (pj_parser_slot). Set by
  // loadConfig() when FileLoader embeds it under "_parser_config". When
  // non-empty, takes precedence over per-field accessors in McapDialog.
  std::string parser_config_override_;
  // Own the cold (post-import lazy) byte path while fetchers are created, one
  // store per recording file (a split rosbag2 bag has several). Each fetcher
  // retains its file's shared cold state, so deferred ObjectStore pulls still
  // work after PJ4 destroys the DataSourceHandle at the end of loadFile().
  std::vector<mcap::MessageByteStore> byte_stores_;

  RobotStatePublisher robot_state_publisher_;

  PJ::ParserBindingHandle m_image_handle_;
  PJ::ParserBindingHandle m_tf_handle_;
};

#endif