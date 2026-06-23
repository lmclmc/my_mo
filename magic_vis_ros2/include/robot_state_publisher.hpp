// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Wim Meeussen */

#ifndef ROBOT_STATE_PUBLISHER__ROBOT_STATE_PUBLISHER_HPP_
#define ROBOT_STATE_PUBLISHER__ROBOT_STATE_PUBLISHER_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <tinyxml2.h>  
#include "builtin_interfaces/msg/time.hpp"
#include "kdl/tree.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "urdf/model.h"

#include <lcm_to_ros2/msg/state_estimator_lcmt.hpp>
#include <lcm_to_ros2/msg/leg_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/arm_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/waist_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/head_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/lcm_response_lcmt.hpp>
#include <lcm_to_ros2/msg/wbc_test_data_t.hpp>
#include <lcm_to_ros2/msg/trajectory_joint_lcmt.hpp>

using MimicMap = std::map<std::string, urdf::JointMimicSharedPtr>;

namespace magic
{

struct Joint
{
  int idx;
  double min;
  double max;
  double zero;
  double position;
  double velocity;
  double effort;
  bool continuous;
  bool forward;
  
  Joint() : idx(-1), min(0.0), max(0.0), zero(0.0), position(0.0), velocity(0.0), effort(0.0), continuous(false), forward(true) {}
};

/// A class that represents a mapping between a KDL segment and its root and tip.
class SegmentPair final
{
public:
  /// Constructor
  explicit SegmentPair(
    const KDL::Segment & p_segment,
    const std::string & p_root,
    const std::string & p_tip)
  : segment(p_segment), root(p_root), tip(p_tip) {}

  KDL::Segment segment;  ///< The KDL segment
  std::string root;  ///< The name of the root element to which this link is attached
  std::string tip;  ///< The name of the element
};

struct DependentJoint
{
  std::string parent;
  double factor;
  double offset;
  
  DependentJoint() : parent(""), factor(1.0), offset(0.0) {}
};

/// The class that contains all of the functionality of the RobotStatePublisher ROS 2 node.
class RobotStatePublisher : public rclcpp::Node
{
public:
  /// Constructor
  explicit RobotStatePublisher(const rclcpp::NodeOptions & options);

protected:
  KDL::Tree parseURDF(const std::string & urdf_xml, urdf::Model & model);

  /// Setup the URDF for use.
  /**
   * This method first parses the URDF into an internal representation.
   * Based on that representation, it then generates the list of joint segments
   * and mimic pairs that it needs during runtime.  Finally, it publishes
   * the text of the URDF to the network on the /robot_description topic.
   *
   * \param[in] urdf_xml The string representing the URDF XML.
   */
  void setupURDF(const std::string & urdf_xml);

  /// Recursive method to add all children to the internal segment list.
  /**
   *
   * \param[in] segment An iterator to the SegmentMap to add to the internal segment list.
   */
  void addChildren(
    const urdf::Model & model,
    const KDL::SegmentMap::const_iterator segment);

  /// Publish transforms to /tf2.
  /**
   * This method is called by callbackJointState() when new transforms are available and need to be published.
   *
   * \param[in] joint_positions A map of joint names to joint positions.
   * \param[in] time The time at which the joint positions were recorded.
   */
  void publishTransforms(
    const std::map<std::string, double> & joint_positions,
    const builtin_interfaces::msg::Time & time);

  /// Publish fixed transforms at startup time to /tf2_static.
  void publishFixedTransforms();

  /// Initialize the URDF from the XML document.
  /**
   * This method parses the URDF from the XML document and sets up the internal data structures.
   *
   * \param[in] doc The XML document containing the URDF.
   */
  void initUrdf(tinyxml2::XMLDocument &doc);

  Joint initJoint(double minval, double maxval, double zeroval, bool continuous);

  void timerCallback();

  void update(double delta);

  /// The callback that is called when a new JointState message is received.
  /**
   * This method examines the incoming JointStates and applies a series of checks to
   * see if new transforms should be published.  If so, it calls publishTransforms() to do so.
   *
   * \param[in] state The JointState message that was delivered.
   */
  void callbackJointState(const sensor_msgs::msg::JointState::ConstSharedPtr state);

  /// The callback that is called to check that new parameters are valid.
  /**
   * This allows the class to reject parameter updates that are invalid.
   *
   * \param[in] parameters The vector of parameters that are going to change.
   * \return SetParametersResult with successful set to true on success, false otherwise.
   */
  rcl_interfaces::msg::SetParametersResult parameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);

  /// The callback that is called when parameters on the node are changed.
  /**
   * This allows the class to dynamically react to changes in parameters.
   *
   * \param[in] event The parameter change event that occurred.
   */
  void onParameterEvent(std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event);

  /// A map of dynamic segment names to SegmentPair structures
  std::map<std::string, SegmentPair> segments_;

  /// A map of fixed segment names to SegmentPair structures
  std::map<std::string, SegmentPair> segments_fixed_;

  /// A pointer to the tf2 TransformBroadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// A pointer to the tf2 StaticTransformBroadcaster
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  /// A pointer to the ROS 2 publisher for the robot_description
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub_;

  /// A pointer to the ROS 2 subscription for the joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  /// The last time a joint state message was received
  rclcpp::Time last_callback_time_;

  /// A map between a joint name and the last time its state was published
  std::map<std::string, builtin_interfaces::msg::Time> last_publish_time_;

  /// A map of the mimic joints that should be published
  MimicMap mimic_;

  /// The parameter event callback that will be called when a parameter is changed
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  /// The parameter event callback that will be called when a parameter is changed
  std::shared_ptr<rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent,
    std::allocator<void>>> parameter_subscription_;

private:
  void legStateCallback(const lcm_to_ros2::msg::LegControlDataLcmt::SharedPtr msg);
  void armStateCallback(const lcm_to_ros2::msg::ArmControlDataLcmt::SharedPtr msg);
  void waistStateCallback(const lcm_to_ros2::msg::WaistControlDataLcmt::SharedPtr msg);
  void stateEstimatorCallback(const lcm_to_ros2::msg::StateEstimatorLcmt::SharedPtr msg);

  void reRank(std::vector<std::string> &joint_list, const std::string &xml_file_path);
  
  rclcpp::Subscription<lcm_to_ros2::msg::LegControlDataLcmt>::SharedPtr leg_state_sub;
  rclcpp::Subscription<lcm_to_ros2::msg::ArmControlDataLcmt>::SharedPtr arm_state_sub;
  rclcpp::Subscription<lcm_to_ros2::msg::WaistControlDataLcmt>::SharedPtr waist_state_sub;
  rclcpp::Subscription<lcm_to_ros2::msg::StateEstimatorLcmt>::SharedPtr state_estimator_sub;

  geometry_msgs::msg::Quaternion imu_quaternion_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

  std::string root_link_name_ = "pelvis";

  std::map<std::string, Joint> free_joints_;
  std::vector<std::string> joint_list_;
  std::map<std::string, DependentJoint> dependent_joints_;
  std::map<std::string, double> zeros_;
  bool use_mimic_;
  bool use_small_;
  bool pub_def_positions_;
  bool pub_def_vels_;
  bool pub_def_efforts_;
  double delta_;

  size_t leg_count_ = 0;
  size_t leg_start_index_ = 0;
  size_t waist_count_ = 0;
  size_t waist_start_index_ = 0;
  size_t head_count_ = 0;
  size_t head_start_index_ = 0;
  size_t arm_count_ = 0;
  size_t arm_start_index_ = 0;
  size_t arm_free_count_ = 0;
  size_t arm_count_1 = 0;
  size_t arm_start_index_1 = 0;
};

}  // namespace robot_state_publisher

#endif  // ROBOT_STATE_PUBLISHER__ROBOT_STATE_PUBLISHER_HPP_
