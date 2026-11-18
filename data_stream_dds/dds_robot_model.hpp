#pragma once

#include "dds_dynamic.hpp"
#include "dds_robot_settings.hpp"

#include <pj_base/builtin/frame_transforms.hpp>
#include <array>
#include <map>
#include <string>
#include <vector>

// Owns the URDF kinematics and matches DDS array indices to joint names using
// MuJoCo's actuator order. No ROS node, DDS publisher, Qt or MuJoCo runtime.
class DdsRobotModel {
 public:
  struct Mapping {
    std::string group, topic, joint;
    int index, xml_index;
  };

  explicit DdsRobotModel(const DdsRobotSettings& settings);
  const std::string& description() const { return description_; }
  const std::string& rootLink() const { return root_link_; }
  const std::string& worldFrame() const { return world_frame_; }
  const std::vector<Mapping>& mapping() const { return mapping_; }
  // Applies a complete joint q array or state_estimator quat (wxyz) atomically.
  // Retains the latest orientation and positions from the other topics.
  // A relevant but malformed record throws; unrelated records return false.
  bool update(const DdsDynamicRecord& record);
  PJ::sdk::FrameTransforms transforms(int64_t timestamp_ns) const;

 private:
  struct Joint {
    std::string name, parent, child, type;
    PJ::sdk::Vector3 origin, axis{1, 0, 0};
    PJ::sdk::Quaternion rotation;
    std::string mimic;
    double multiplier = 1, offset = 0, position = 0;
  };
  double position(size_t index, unsigned depth) const;

  DdsRobotSettings settings_;
  std::string description_, root_link_, world_frame_ = "world";
  PJ::sdk::Quaternion base_rotation_;
  std::vector<Joint> joints_;
  std::map<std::string, size_t> joint_indices_;
  std::array<std::vector<size_t>, 4> group_joints_;
  std::vector<Mapping> mapping_;
};

// ROS 2 CDR payloads understood by the same ros2msg parser used by the LCM
// plugin. Serializing these two simple schemas does not require ROS libraries.
std::vector<uint8_t> ddsRobotDescriptionCdr(const std::string& description);
std::vector<uint8_t> ddsRobotTransformsCdr(const PJ::sdk::FrameTransforms& transforms);
inline constexpr char kDdsRobotTfSchema[] = R"(geometry_msgs/TransformStamped[] transforms
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
