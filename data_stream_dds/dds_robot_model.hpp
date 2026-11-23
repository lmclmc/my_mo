#pragma once

#include "dds_dynamic.hpp"
#include "dds_robot_settings.hpp"

#include <pj_base/builtin/frame_transforms.hpp>
#include <array>
#include <map>
#include <string>
#include <vector>

// 持有 URDF 关节树与实时姿态，按 MuJoCo 执行器顺序将 DDS 数组索引映射到关节名。
// 解析 XML 和计算坐标变换均在插件内部完成。
class DdsRobotModel {
 public:
  // 一条 DDS 到 URDF 的关节映射，index 是话题内索引，xml_index 是执行器全局索引。
  struct Mapping {
    std::string group, topic, joint;
    int index, xml_index;
  };

  // 复制 settings 并构建关节树与话题映射。配置、模型文件或映射无效时抛出异常。
  explicit DdsRobotModel(const DdsRobotSettings& settings);
  // 返回用于 3D 查看器的 URDF 文本，引用随模型对象存活。
  const std::string& description() const { return description_; }
  // 返回 URDF 关节树的根连杆名，引用随模型对象存活。
  const std::string& rootLink() const { return root_link_; }
  // 返回用于发布根姿态的外部参考坐标系名，引用随模型对象存活。
  const std::string& worldFrame() const { return world_frame_; }
  // 返回各话题数组元素对应的 URDF 关节，引用随模型对象存活。
  const std::vector<Mapping>& mapping() const { return mapping_; }

  // 读取 record 的完整关节位置数组或 wxyz 姿态，校验后更新对应状态并返回 true。
  // 无关话题返回 false。相关话题的数据无效时抛出异常，保留已有状态。
  bool update(const DdsDynamicRecord& record);

  // 返回当前姿态下的根变换和每个父子连杆变换，统一采用 timestamp_ns（纳秒）。
  // 从动关节求值失败时抛出异常。
  PJ::sdk::FrameTransforms transforms(int64_t timestamp_ns) const;

 private:
  struct Joint {
    std::string name, parent, child, type;
    PJ::sdk::Vector3 origin, axis{1, 0, 0};
    PJ::sdk::Quaternion rotation;
    std::string mimic;
    double multiplier = 1, offset = 0, position = 0;
  };
  // 求关节 index 的位置，depth 是 mimic 依赖链深度，用于识别超限或循环。
  double position(size_t index, unsigned depth) const;

  DdsRobotSettings settings_;
  std::string description_, root_link_, world_frame_ = "world";
  PJ::sdk::Quaternion base_rotation_;
  std::vector<Joint> joints_;
  std::map<std::string, size_t> joint_indices_;
  std::array<std::vector<size_t>, 4> group_joints_;
  std::vector<Mapping> mapping_;
};

// 将 description 编码为 ROS 2 std_msgs/msg/String 的小端 CDR 字节。字符串过长时抛出异常。
std::vector<uint8_t> ddsRobotDescriptionCdr(const std::string& description);
// 将 transforms 编码为 ROS 2 tf2_msgs/msg/TFMessage 的小端 CDR 字节。
// 无法编码的秒数或过长的坐标系名称会引发异常。
std::vector<uint8_t> ddsRobotTransformsCdr(const PJ::sdk::FrameTransforms& transforms);
// 实时发布和 MCAP 录制共用的 TF 话题与 ROS 2 消息定义。
inline constexpr char kDdsRobotTfTopic[] = "/tf";
inline constexpr char kDdsRobotTfType[] = "tf2_msgs/msg/TFMessage";
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
