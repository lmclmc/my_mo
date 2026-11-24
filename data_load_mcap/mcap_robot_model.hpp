#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <pj_base/builtin/frame_transforms.hpp>
#include <string>
#include <vector>

// MCAP loader 内部使用的 X1 机器人模型。它只依赖 MCAP 中的 CDR 字节，
// 不依赖 DDS 运行时，因此离线加载和实时 DDS 插件彼此解耦。
class McapRobotModel {
 public:
  McapRobotModel(const std::string& urdf_path, const std::string& mujoco_path);
  const std::string& description() const { return description_; }
  const std::string& worldFrame() const { return world_frame_; }
  bool update(std::string_view topic, const std::vector<uint8_t>& payload);
  PJ::sdk::FrameTransforms transforms(int64_t timestamp_ns) const;

 private:
  struct Joint {
    std::string name, parent, child, type, mimic;
    PJ::sdk::Vector3 origin{}, axis{1, 0, 0};
    PJ::sdk::Quaternion rotation{};
    double multiplier = 1.0, offset = 0.0, position = 0.0;
  };
  double position(size_t index, unsigned depth) const;
  std::string description_;
  std::string root_link_;
  std::string world_frame_ = "world";
  PJ::sdk::Quaternion base_rotation_{0, 0, 0, 1};
  std::vector<Joint> joints_;
  std::vector<std::string> actuator_order_;
  std::vector<size_t> leg_, waist_, arm_, head_;
};

inline constexpr char kMcapRobotTfTopic[] = "/tf";
inline constexpr char kMcapRobotTfType[] = "tf2_msgs/msg/TFMessage";
inline constexpr char kMcapRobotTfSchema[] = R"(geometry_msgs/TransformStamped[] transforms
================================================================================
MSG: geometry_msgs/TransformStamped
std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Transform
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
)";

// 读取 X1 控制话题的固定小端 CDR 布局：q/joint_curPos 为 float 数组，
// state_estimator.quat 位于消息第 27 个 float，顺序为 wxyz。
bool decodeMcapRobotMessage(std::string_view topic, const uint8_t* data, size_t size,
                            std::vector<double>& values, std::array<double, 4>& quat,
                            bool& has_quat);
std::vector<uint8_t> mcapRobotDescriptionCdr(const std::string& description);
std::vector<uint8_t> mcapRobotTransformsCdr(const PJ::sdk::FrameTransforms& transforms);
