#include <robot_vis/urdf_visualizer.h>

#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  // 初始化ROS2节点
  rclcpp::init(argc, argv);

  // urdf joint names
  std::vector<std::string> joint_names(6*2 + 7*2 + 1 + 2);
  joint_names[0]  = "J_hip_roll_L";
  joint_names[1]  = "J_hip_yaw_L";
  joint_names[2]  = "J_hip_pitch_L";
  joint_names[3]  = "J_knee_pitch_L";
  joint_names[4]  = "J_ankle_pitch_L";
  joint_names[5]  = "J_ankle_roll_L";
  joint_names[6]  = "J_hip_roll_R";
  joint_names[7]  = "J_hip_yaw_R";
  joint_names[8]  = "J_hip_pitch_R";
  joint_names[9]  = "J_knee_pitch_R";
  joint_names[10] = "J_ankle_pitch_R";
  joint_names[11] = "J_ankle_roll_R";
  
  joint_names[12] = "joint_la1";
  joint_names[13] = "joint_la2";
  joint_names[14] = "joint_la3";
  joint_names[15] = "joint_la4";
  joint_names[16] = "joint_la5";
  joint_names[17] = "joint_la6";
  joint_names[18] = "joint_la7";
  joint_names[19] = "joint_ra1";
  joint_names[20] = "joint_ra2";
  joint_names[21] = "joint_ra3";
  joint_names[22] = "joint_ra4";
  joint_names[23] = "joint_ra5";
  joint_names[24] = "joint_ra6";
  joint_names[25] = "joint_ra7";
  
  joint_names[26] = "joint_wy";

  joint_names[27] = "joint_hy";
  joint_names[28] = "joint_hp";

  std::string urdf = "rviz_urdf_robot_description";
  magic::UrdfVisualizer p3b_vis(urdf,
                                joint_names,
                                "pelvis",
                                "world",
                                "/lcm_to_ros/state_estimator",
                                "/lcm_to_ros/leg_control_data", 0, 12,
                                "/lcm_to_ros/arm_control_data", 12, 14,
                                "/lcm_to_ros/waist_control_data", 26, 1,
                                "/lcm_to_ros/head_control_data", 27, 2,
                                "p_obv");

  // 自旋节点，使用get_node()方法获取节点指针
  rclcpp::spin(p3b_vis.get_node());

  // 关闭节点
  rclcpp::shutdown();

  return 0;
}