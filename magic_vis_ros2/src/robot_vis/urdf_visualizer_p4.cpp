#include <robot_vis/urdf_visualizer.h>

#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  // 初始化ROS2节点
  rclcpp::init(argc, argv);

  // urdf joint names
  std::vector<std::string> joint_names(6*2 + 7*2 + 2 + 2);
  joint_names[0]  = "JOINT_HIP_ROLL_L";
  joint_names[1]  = "JOINT_HIP_YAW_L";
  joint_names[2]  = "JOINT_HIP_PITCH_L";
  joint_names[3]  = "JOINT_KNEE_PITCH_L";
  joint_names[4]  = "JOINT_ANKLE_PITCH_L";
  joint_names[5]  = "JOINT_ANKLE_ROLL_L";
  joint_names[6]  = "JOINT_HIP_ROLL_R";
  joint_names[7]  = "JOINT_HIP_YAW_R";
  joint_names[8]  = "JOINT_HIP_PITCH_R";
  joint_names[9]  = "JOINT_KNEE_PITCH_R";
  joint_names[10] = "JOINT_ANKLE_PITCH_R";
  joint_names[11] = "JOINT_ANKLE_ROLL_R";
  
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
  
  joint_names[26] = "joint_wr";
  joint_names[27] = "joint_wy";

  joint_names[28] = "joint_hy";
  joint_names[29] = "joint_hp";

  std::string urdf = "rviz_urdf_robot_description";
  magic::UrdfVisualizer p4_vis(urdf,
                               joint_names,
                               "pelvis",
                               "world",
                               "/lcm_to_ros/state_estimator",
                               "/lcm_to_ros/leg_control_data", 0, 12,
                               "/lcm_to_ros/arm_control_data", 12, 14,
                               "/lcm_to_ros/waist_control_data", 26, 2,
                               "/lcm_to_ros/head_control_data", 28, 2,
                               "p_obv");

  // 自旋节点，使用get_node()方法获取节点指针
  rclcpp::spin(p4_vis.get_node());

  // 关闭节点
  rclcpp::shutdown();

  return 0;
}