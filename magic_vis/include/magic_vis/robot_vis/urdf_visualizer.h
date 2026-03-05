#ifndef MAGIC_VIS_URDF_VISUALIZER_H_
#define MAGIC_VIS_URDF_VISUALIZER_H_

#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <urdf/model.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>

#include <lcm_to_ros/state_estimator_lcmt.h>
#include <lcm_to_ros/leg_control_data_lcmt.h>
#include <lcm_to_ros/arm_control_data_lcmt.h>
#include <lcm_to_ros/waist_control_data_lcmt.h>
#include <lcm_to_ros/head_control_data_lcmt.h>

namespace magic {

/**
 * @brief Publishes RVIZ transforms used to visualize a robot's URDF.
 */
class UrdfVisualizer {
public:
  /**
   * @brief Constructs the visualizer for a specific URDF %urdf_name.
   * @param urdf_name  Robot description variable on the ROS parameter server.
   * @param joint_names_in_urdf  The names of the joints in the URDF file.
   * @param base_link_in_urdf  The name of the base_link in the URDF file.
   * @param rviz_fixed_frame  The Fixed Frame name specified in RVIZ.
   * @param pose_state_topic  Topic name lcm_to_ros::state_estimator_lcmt.
   * @param leg_state_topic   Topic name lcm_to_ros::leg_control_data_lcmt.
   * @param arm_state_topic   Topic name lcm_to_ros::arm_control_data_lcmt.
   * @param waist_state_topic Topic name lcm_to_ros::waist_control_data_lcmt.
   * @param tf_prefix  In case multiple URDFS are loaded, each can be given a
   *        unique tf_prefix in RIVZ to visualize different states simultaneously.
   */
  UrdfVisualizer(const std::string& urdf_name,
                 const std::vector<std::string>& joint_names_in_urdf,
                 const std::string& base_link_in_urdf,
                 const std::string& rviz_fixed_frame,
                 const std::string& pose_state_topic,
                 const std::string& leg_state_topic, const int leg_id, const int leg_n,
                 const std::string& arm_state_topic, const int arm_id, const int arm_n,
                 const std::string& waist_state_topic, const int waist_id, const int waist_n,
                 const std::string& head_state_topic, const int head_id, const int head_n,
                 const std::string& tf_prefix = "");
  virtual ~UrdfVisualizer() = default;

private:
  void poseStateCallback(const lcm_to_ros::state_estimator_lcmt& msg);
  void legStateCallback(const lcm_to_ros::leg_control_data_lcmt& msg);
  void armStateCallback(const lcm_to_ros::arm_control_data_lcmt& msg);
  void waistStateCallback(const lcm_to_ros::waist_control_data_lcmt& msg);
  void headStateCallback(const lcm_to_ros::head_control_data_lcmt& msg);

  ros::Subscriber pose_sub_des_, leg_sub_des_, arm_sub_des_, waist_sub_des_, head_sub_des_, head_upper_sub_des_;
  tf::TransformBroadcaster tf_broadcaster_;
  std::shared_ptr<robot_state_publisher::RobotStatePublisher> robot_publisher;

  std::vector<std::string> joint_names_in_urdf_;
  std::string base_joint_in_urdf_;
  std::string rviz_fixed_frame_;
  std::string tf_prefix_;

  std::map<std::string, double> joint_positions_;
  std::vector<double> joint_values_;
  int leg_id_, leg_n_, arm_id_, arm_n_, waist_id_, waist_n_, head_id_, head_n_;
};

} /* namespace magic */

#endif /* MAGIC_VIS_URDF_VISUALIZER_H_ */
