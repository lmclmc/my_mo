#pragma once

#include <array>
#include <set>
#include <string>

// 将同一 DDS 话题中的关节位置数组绑定到一段 MuJoCo 执行器。
struct DdsRobotGroup {
  std::string topic;
  int start = 0;  // 在 MuJoCo 执行器顺序中的起始索引。
  int count = 0;
};

// 保存 3D 模型资源路径以及 DDS 话题到执行器区间的映射。
struct DdsRobotSettings {
  static constexpr int kMaxJoints = 8192;
  static constexpr char kStateEstimatorTopic[] = "state_estimator";
  static constexpr std::array<const char*, 4> kGroupNames{"leg", "waist", "arm", "head"};
  bool enabled = false;
  std::string urdf_path;
  std::string mujoco_path;
  // X1 的执行器依次为腿、腰、头和臂。每个话题中的位置数组从索引 0 开始。
  std::array<DdsRobotGroup, 4> groups{{
      {"leg_control_data", 0, 12}, {"waist_control_data", 12, 3},
      {"arm_control_data", 17, 14}, {"head_control_data", 15, 2}}};

  // 检查必要路径和已启用关节组的索引及话题配置。返回首条错误，无错误时返回空串。
  std::string validationError() const {
    if (urdf_path.empty() || mujoco_path.empty()) return "请导入 URDF 和 MuJoCo XML 文件。";
    std::set<std::string> topics;
    std::set<int> indices;
    for (const auto& group : groups) {
      if (group.start < 0 || group.count < 0 || group.start > kMaxJoints - group.count)
        return "关节范围无效：起始索引和数量必须在 0–8192 范围内。";
      // 数量为零表示禁用该组，可以保留尚未配置的话题名。
      if (!group.count) continue;
      if (group.topic.empty() || group.topic == kStateEstimatorTopic || !topics.insert(group.topic).second)
        return "每个启用的关节组需要一个非空且独立的 DDS 话题名，不能使用姿态话题 state_estimator。";
      for (int i = 0; i < group.count; ++i)
        if (!indices.insert(group.start + i).second) return "不同关节组的 XML 索引范围不能重叠。";
    }
    return indices.empty() ? "至少一个关节组的数量需要大于 0。" : std::string{};
  }

  // 启用 3D 模型时，将姿态话题和数量大于零的关节组非空话题名补入 topics。
  void addTopics(std::set<std::string>& topics) const {
    if (!enabled) return;
    topics.insert(kStateEstimatorTopic);
    for (const auto& group : groups)
      if (group.count > 0 && !group.topic.empty()) topics.insert(group.topic);
  }
};
