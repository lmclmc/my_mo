#include <iostream>
#include <string>

#include <ros/init.h>

#include <magic_vis/robot_vis/urdf_visualizer.h>

int main(int argc, char *argv[])
{
  ::ros::init(argc, argv, "gen2_urdf_visualizer");

  // urdf joint names
  std::vector<std::string> joint_names(6*2 + 7*2 + 3 + 2);
  joint_names[0]  = "left_hip_pitch_joint";
  joint_names[1]  = "left_hip_roll_joint";
  joint_names[2]  = "left_hip_yaw_joint";
  joint_names[3]  = "left_knee_pitch_joint";
  joint_names[4]  = "left_ankle_pitch_joint";
  joint_names[5]  = "left_ankle_roll_joint";
  joint_names[6]  = "right_hip_pitch_joint";
  joint_names[7]  = "right_hip_roll_joint";
  joint_names[8]  = "right_hip_yaw_joint";
  joint_names[9]  = "right_knee_pitch_joint";
  joint_names[10] = "right_ankle_pitch_joint";
  joint_names[11] = "right_ankle_roll_joint";
  
  joint_names[12] = "left_shoulder_pitch_joint";
  joint_names[13] = "left_shoulder_roll_joint";
  joint_names[14] = "left_shoulder_yaw_joint";
  joint_names[15] = "left_elbow_pitch_joint";
  joint_names[16] = "left_wrist_yaw_joint";
  joint_names[17] = "left_wrist_pitch_joint";
  joint_names[18] = "left_wrist_roll_joint";

  joint_names[19] = "right_shoulder_pitch_joint";
  joint_names[20] = "right_shoulder_roll_joint";
  joint_names[21] = "right_shoulder_yaw_joint";
  joint_names[22] = "right_elbow_pitch_joint";
  joint_names[23] = "right_wrist_yaw_joint";
  joint_names[24] = "right_wrist_pitch_joint";
  joint_names[25] = "right_wrist_roll_joint";
  
  joint_names[26] = "waist_yaw_joint";
  joint_names[27] = "waist_roll_joint";
  joint_names[28] = "waist_pitch_joint";

  joint_names[29] = "head_yaw_joint";
  joint_names[30] = "head_pitch_joint";

  std::string urdf = "rviz_urdf_robot_description";
  magic::UrdfVisualizer gen2_vis(urdf,
                               joint_names,
                               "pelvis",
                               "world",
                               "/lcm_to_ros/state_estimator",
                               "/lcm_to_ros/leg_control_data", 0, 12,
                               "/lcm_to_ros/arm_control_data", 12, 14,
                               "/lcm_to_ros/waist_control_data", 26, 3,
                               "/lcm_to_ros/head_control_data", 29, 2,
                               "p_obv");

  ::ros::spin();

  return 1;
}