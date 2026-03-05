#include <magic_vis/robot_vis/urdf_visualizer.h>

namespace magic {

UrdfVisualizer::UrdfVisualizer(const std::string& urdf_name,
                               const std::vector<std::string>& joint_names_in_urdf,
                               const std::string& base_joint_in_urdf,
                               const std::string& fixed_frame,
                               const std::string& pose_state_topic,
                               const std::string& leg_state_topic, const int leg_id, const int leg_n,
                               const std::string& arm_state_topic, const int arm_id, const int arm_n,
                               const std::string& waist_state_topic, const int waist_id, const int waist_n,
                               const std::string& head_state_topic, const int head_id, const int head_n,
                               const std::string& tf_prefix)
{
  joint_names_in_urdf_ = joint_names_in_urdf;
  base_joint_in_urdf_  = base_joint_in_urdf;
  rviz_fixed_frame_   = fixed_frame;
  tf_prefix_ = tf_prefix;
  leg_id_ = leg_id;
  leg_n_ = leg_n;
  arm_id_ = arm_id;
  arm_n_ = arm_n;
  waist_id_ = waist_id;
  waist_n_ = waist_n;
  head_id_ = head_id;
  head_n_ = head_n;

  ::ros::NodeHandle nh;
  pose_sub_des_ = nh.subscribe(pose_state_topic, 1, &UrdfVisualizer::poseStateCallback, this);
  ROS_DEBUG("Subscribed to: %s", pose_sub_des_.getTopic().c_str());

  if (0 != leg_n) {
    leg_sub_des_ = nh.subscribe(leg_state_topic, 1, &UrdfVisualizer::legStateCallback, this);
    ROS_DEBUG("Subscribed to: %s", leg_sub_des_.getTopic().c_str());
  }

  if (0 != arm_n) {
    arm_sub_des_ = nh.subscribe(arm_state_topic, 1, &UrdfVisualizer::armStateCallback, this);
    ROS_DEBUG("Subscribed to: %s", arm_sub_des_.getTopic().c_str());
  }

  if (0 != waist_n) {
    waist_sub_des_ = nh.subscribe(waist_state_topic, 1, &UrdfVisualizer::waistStateCallback, this);
    ROS_DEBUG("Subscribed to: %s", waist_sub_des_.getTopic().c_str());
  }

  if (0 != head_n) {
    head_sub_des_ = nh.subscribe(head_state_topic, 1, &UrdfVisualizer::headStateCallback, this);
    ROS_DEBUG("Subscribed to: %s", head_sub_des_.getTopic().c_str());
  }

  // initialize joint values
  joint_values_ = std::vector<double>(leg_n + arm_n + waist_n + head_n, 0.);
  for (auto jm : joint_names_in_urdf)
    joint_positions_[jm] = 0.;

  // Load model from file
  KDL::Tree my_kdl_tree;
  urdf::Model my_urdf_model;
  bool model_ok  = my_urdf_model.initParam(urdf_name);
  if(!model_ok)
  {
    ROS_ERROR("Invalid URDF File");
    exit(EXIT_FAILURE);
  }
  ROS_DEBUG("URDF successfully parsed");
  kdl_parser::treeFromUrdfModel(my_urdf_model, my_kdl_tree);
  ROS_DEBUG("Robot tree is ready");

  robot_publisher = std::make_shared<robot_state_publisher::RobotStatePublisher>(my_kdl_tree);
}

void
UrdfVisualizer::poseStateCallback(const lcm_to_ros::state_estimator_lcmt& msg)
{
  // Converting from joint messages to robot state
  geometry_msgs::TransformStamped W_X_B_message;
  W_X_B_message.header.stamp    = ::ros::Time::now();
  W_X_B_message.header.frame_id = rviz_fixed_frame_;
  W_X_B_message.child_frame_id  = tf_prefix_ + "/" + base_joint_in_urdf_;

  W_X_B_message.transform.translation.x = msg.p[0];
  W_X_B_message.transform.translation.y = msg.p[1];
  W_X_B_message.transform.translation.z = msg.p[2];

  W_X_B_message.transform.rotation.w = msg.quat[0];
  W_X_B_message.transform.rotation.x = msg.quat[1];
  W_X_B_message.transform.rotation.y = msg.quat[2];
  W_X_B_message.transform.rotation.z = msg.quat[3];

  // Assign all joint values
  for (int i=0; i<joint_values_.size(); ++i)
    joint_positions_[joint_names_in_urdf_.at(i)] = joint_values_.at(i);

  tf_broadcaster_.sendTransform(W_X_B_message);
  robot_publisher->publishTransforms(joint_positions_, ::ros::Time::now(), tf_prefix_);
  robot_publisher->publishFixedTransforms(tf_prefix_);
}

void
UrdfVisualizer::legStateCallback(const lcm_to_ros::leg_control_data_lcmt& msg)
{
  for (int i=0; i<leg_n_; i++)
    joint_values_[i + leg_id_] = msg.q[i];
}

void
UrdfVisualizer::armStateCallback(const lcm_to_ros::arm_control_data_lcmt& msg)
{
  for (int i=0; i<arm_n_; i++)
    joint_values_[i + arm_id_] = msg.joint_curPos[i];
}

void
UrdfVisualizer::waistStateCallback(const lcm_to_ros::waist_control_data_lcmt& msg)
{
  for (int i=0; i<waist_n_; i++)
    joint_values_[i + waist_id_] = msg.q[i];
}

void
UrdfVisualizer::headStateCallback(const lcm_to_ros::head_control_data_lcmt& msg)
{
  for (int i=0; i<head_n_; i++)
    joint_values_[i + head_id_] = msg.q[i];
}

} /* namespace magic */
