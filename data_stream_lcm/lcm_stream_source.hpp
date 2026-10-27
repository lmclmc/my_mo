#include <algorithm>
#include <array>
#include <asio.hpp>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <pj_base/sdk/data_source_patterns.hpp>
#include <pj_plugins/sdk/encoding_utils.hpp>
#include <pj_plugins/sdk/endpoint.hpp>
#include <pj_plugins/sdk/streaming_source.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm_coretypes.h>

#include "lcm_dialog.hpp"
#include "mcap_manifest.hpp"
#include "robot_state_publisher.hpp"
#include "lcmgen.h"
#include "logsink.hpp"

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

class LcmStreamSource : public PJ::StreamSourceBase {
  using WriteCB = std::function<void(const std::string&, const std::string&, PJ::sdk::ValueRef)>;
public:
  LcmStreamSource();
  PJ_borrowed_dialog_t getDialog() override {
    return PJ::borrowDialog(dialog_);
  }

  uint64_t extraCapabilities() const override {
    return PJ::kCapabilityDelegatedIngest | PJ::kCapabilityHasDialog | PJ::kCapabilitySupportsPause;
  }

  std::string saveConfig() const override {
    return dialog_.saveConfig();
  }

  PJ::Status loadConfig(std::string_view config_json);
  
  PJ::Status onStart() override;

  // onPoll() drains this stream's own inbox — never touches the network.
  PJ::Status onPoll() override;

  void onStop() override;

  void handleMessage(const lcm::ReceiveBuffer*, const std::string&);

  void publishRobotState(const std::string&);
  void publishDepthImage(void*, int, int64_t, int, int, std::vector<uint8_t>&);
  void publishTF(void*, int, int64_t, const std::vector<geometry_msgs::msg::TransformStamped>&);
  void publishJointState(void*, int, int64_t, const sensor_msgs::msg::JointState&);
  void publishVisualization(int64_t, const visualization_msgs::msg::Marker&);
  void publishVisualizationErrBar(int64_t, const visualization_msgs::msg::Marker&);

  //二级解析函数，调用lcm解析函数进行解析，调用不了的则递归调用一级解析函数处理
  int decode_type_2(void*, int, int64_t, int, const std::string&,
                    const std::string&, const std::string&,
                    std::unordered_map<std::string, int>&,
                    std::map<std::string, lcm_struct_t*>&,
                    const WriteCB&);
  //一级解析函数，解析LCM消息中，任意类型的结构体数据，解开结构体内部任意成员数据后，调用二级解析函数处理
  int decode_type_1(void*, int, int64_t, int, const std::string&, 
                    const std::string&, std::map<std::string, lcm_struct_t*>&,
                    const WriteCB&);

  void savePoint(void*, int, int64_t, const std::string&, const std::string&, PJ::sdk::ValueRef);

private:
  LcmDialog dialog_;

  std::string parser_config_override_;

  std::map<std::string, lcm_struct_t*> lcm_type_table;
  std::unordered_map<int, std::shared_ptr<lcm::LCM>> lcm_table;
  typedef struct TopicLcmItem {
    int64_t hash = 0;
    std::vector<lcmgen_t*> lcmgen_table = {};
    std::string lcmFilePath = "";
    std::optional<PJ::sdk::TopicHandle> topic_handle;
  } TopicLcmItem;
  
  std::unordered_map<std::string, TopicLcmItem> topic_map;
  std::shared_ptr<rotating_file_sink> lcm_log;
  SafeQueue<LcmMessage> message_queue;

  bool _running;
  bool _first_warning;

  std::thread lcm_runner_;
  std::thread render_runner_;

  std::string log_save_path;
  int log_max_size;
  int log_max_files_num;

  uint32_t publishTfCount = 0;
  int64_t last_TF_timestamp = 0;
  RobotStatePublisher robot_state_publisher_;

  PJ::ParserBindingHandle m_image_handle_;
  PJ::ParserBindingHandle m_tf_handle_;
  PJ::ParserBindingHandle m_marker_handle_;
  PJ::ParserBindingHandle m_marker_error_bar_handle_;

  std::vector<float> height_data;
  std::vector<float> std_data;
};