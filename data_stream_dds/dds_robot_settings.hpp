#pragma once

#include <array>
#include <set>
#include <string>

struct DdsRobotGroup {
  std::string topic;
  int start = 0;  // Index in the MuJoCo actuator joint order, not URDF order.
  int count = 0;
};

struct DdsRobotSettings {
  static constexpr int kMaxJoints = 8192;
  static constexpr char kStateEstimatorTopic[] = "state_estimator";
  static constexpr std::array<const char*, 4> kGroupNames{"leg", "waist", "arm", "head"};
  bool enabled = false;
  std::string urdf_path;
  std::string mujoco_path;
  // X1 actuator order: legs, waist, head, arms. Each topic has its own q[0].
  std::array<DdsRobotGroup, 4> groups{{
      {"leg_control_data", 0, 12}, {"waist_control_data", 12, 3},
      {"arm_control_data", 17, 14}, {"head_control_data", 15, 2}}};

  std::string validationError() const {
    if (urdf_path.empty() || mujoco_path.empty()) return "请导入 URDF 和 MuJoCo XML 文件。";
    std::set<std::string> topics;
    std::set<int> indices;
    for (const auto& group : groups) {
      if (group.start < 0 || group.count < 0 || group.start > kMaxJoints - group.count)
        return "关节范围无效：起始索引和数量必须在 0–8192 范围内。";
      if (!group.count) continue;
      if (group.topic.empty() || group.topic == kStateEstimatorTopic || !topics.insert(group.topic).second)
        return "每个启用的关节组需要一个非空且独立的 DDS 话题名，不能使用姿态话题 state_estimator。";
      for (int i = 0; i < group.count; ++i)
        if (!indices.insert(group.start + i).second) return "不同关节组的 XML 索引范围不能重叠。";
    }
    return indices.empty() ? "至少一个关节组的数量需要大于 0。" : std::string{};
  }

  void addTopics(std::set<std::string>& topics) const {
    if (!enabled) return;
    topics.insert(kStateEstimatorTopic);
    for (const auto& group : groups)
      if (group.count > 0 && !group.topic.empty()) topics.insert(group.topic);
  }
};
