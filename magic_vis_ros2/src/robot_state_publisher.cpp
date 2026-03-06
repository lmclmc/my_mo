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
//    * Neither the name of the copyright holder nor the names of its
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

#include "robot_state_publisher.hpp"

#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "urdf/model.h"

namespace magic
{

namespace
{

inline
geometry_msgs::msg::TransformStamped kdlToTransform(const KDL::Frame & k)
{
  geometry_msgs::msg::TransformStamped t;
  t.transform.translation.x = k.p.x();
  t.transform.translation.y = k.p.y();
  t.transform.translation.z = k.p.z();
  k.M.GetQuaternion(
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
    t.transform.rotation.w);
  return t;
}

}  // namespace


std::string readTextFileToString(const std::string& file_path) {
    // 1. 以二进制模式打开文件（避免Windows下换行符转换问题），并定位到文件末尾
    std::ifstream file(file_path, std::ios::in | std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件: " + file_path);
    }

    try {
        // 2. 获取文件大小（ate模式下，指针已在文件末尾，tellg()返回文件字节数）
        std::streamsize file_size = file.tellg();
        if (file_size < 0) {
            throw std::runtime_error("获取文件大小失败: " + file_path);
        }

        // 3. 调整string容量，避免多次内存分配（优化性能）
        std::string file_content;
        file_content.resize(static_cast<size_t>(file_size));

        // 4. 将文件指针移回开头，读取全部内容到string
        file.seekg(0, std::ios::beg);
        if (!file.read(&file_content[0], file_size)) {
            throw std::runtime_error("读取文件内容失败: " + file_path);
        }

        // 5. 关闭文件（ifstream析构时会自动关闭，此处显式关闭更规范）
        file.close();

        return file_content;
    } catch (...) {
        file.close();  // 确保异常时文件也能关闭
        throw;  // 重新抛出异常，让调用方处理
    }
}

RobotStatePublisher::RobotStatePublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_state_publisher", options)
{
  leg_state_sub = this->create_subscription<lcm_to_ros2::msg::LegControlDataLcmt>(
    "/robot_lcm/leg_control_data", 5, std::bind(&magic::RobotStatePublisher::legStateCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "leg_control_data publisher created");

  waist_state_sub = this->create_subscription<lcm_to_ros2::msg::WaistControlDataLcmt>(
    "/robot_lcm/waist_control_data", 5, std::bind(&magic::RobotStatePublisher::waistStateCallback, this, std::placeholders::_1));

  arm_state_sub = this->create_subscription<lcm_to_ros2::msg::ArmControlDataLcmt>(
    "/robot_lcm/arm_control_data", 5, std::bind(&magic::RobotStatePublisher::armStateCallback, this, std::placeholders::_1));
  // // subscribe to joint state
  // joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
  //   "joint_states",
  //   rclcpp::SensorDataQoS(),
  //   std::bind(&RobotStatePublisher::callbackJointState, this, std::placeholders::_1),
  //   subscriber_options);

  // 声明URDF文件路径参数
  std::string urdf_file_path = this->declare_parameter("urdf_file_path", "magic_vis_ros2/p5_humanoid.urdf");
  RCLCPP_INFO(this->get_logger(), "URDF file path: %s", urdf_file_path.c_str());
  // get the XML
  std::string urdf_xml = readTextFileToString(urdf_file_path);
  
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_xml.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "Invalid robot_description given, ignoring");
    return;
  }

  // set publish frequency
  double publish_freq = this->declare_parameter("publish_frequency", 20.0);
  if (publish_freq < 0.0 || publish_freq > 1000.0) {
    throw std::runtime_error("publish_frequency must be between 0 and 1000");
  }

  // set frame_prefix
  this->declare_parameter("frame_prefix", "");

  // ignore_timestamp_ == true, joint_state messages are accepted, no matter their timestamp
  this->declare_parameter("ignore_timestamp", false);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  description_pub_ = this->create_publisher<std_msgs::msg::String>(
    "robot_description",
    // Transient local is similar to latching in ROS 1.
    rclcpp::QoS(1).transient_local());

  setupURDF(urdf_xml);

  initUrdf(doc);

  auto subscriber_options = rclcpp::SubscriptionOptions();
  subscriber_options.qos_overriding_options =
    rclcpp::QosOverridingOptions::with_default_policies();

  // subscribe to joint state
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotStatePublisher::callbackJointState, this, std::placeholders::_1),
    subscriber_options);

  publishFixedTransforms();

  // Now that we have successfully declared the parameters and done all
  // necessary setup, install the callback for updating parameters.
  param_cb_ = add_on_set_parameters_callback(
    std::bind(&RobotStatePublisher::parameterUpdate, this, std::placeholders::_1));

  // Now that we have successfully declared the parameters and done all
  // necessary setup, install the callback for updating parameters.
  parameter_subscription_ = rclcpp::AsyncParametersClient::on_parameter_event(
    this->get_node_topics_interface(),
    std::bind(&RobotStatePublisher::onParameterEvent, this, std::placeholders::_1));

  // Create publisher
  pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  // Create timer
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(0.1),
    [this]() {
      this->timerCallback();
    });
}

void RobotStatePublisher::legStateCallback(const lcm_to_ros2::msg::LegControlDataLcmt::SharedPtr state_msg)
{
  for (int i=0; i<12; i++) {
    free_joints_[joint_list_[i]].position = state_msg->q[i];
    free_joints_[joint_list_[i]].velocity = state_msg->qd[i];
  }
}

void RobotStatePublisher::armStateCallback(const lcm_to_ros2::msg::ArmControlDataLcmt::SharedPtr state_msg) 
{
  for (int i=0; i<14; i++) {
    if (i+16 >= joint_list_.size()) {
      continue;
    }

    free_joints_[joint_list_[i+16]].position = state_msg->joint_cur_pos[i];
    free_joints_[joint_list_[i+16]].velocity = state_msg->joint_cur_vel[i];
  }
}

void RobotStatePublisher::waistStateCallback(const lcm_to_ros2::msg::WaistControlDataLcmt::SharedPtr state_msg) 
{
  for (int i=0; i<2; i++) {
    if (i+12 >= joint_list_.size()) {
      continue;
    }

    free_joints_[joint_list_[i+12]].position = state_msg->q[i];
    free_joints_[joint_list_[i+12]].velocity = state_msg->qd[i];
  }
}

void RobotStatePublisher::initUrdf(tinyxml2::XMLDocument & doc)
{
  int joint_idx = 0;
  auto robot = doc.FirstChildElement("robot");
  if (!robot) {
    RCLCPP_WARN(this->get_logger(), "No robot element found in URDF");
    return;
  }
  
  // Find all non-fixed joints
  for (auto child = robot->FirstChildElement("joint"); child; child = child->NextSiblingElement("joint")) {
    std::string jtype = child->Attribute("type");
    if (jtype == "gearbox" || jtype == "revolute2" || jtype == "ball" || jtype == "screw" || jtype == "universal" || jtype == "fixed") {
      continue;
    }
    std::string name = child->Attribute("name");
    
    double minval, maxval;
    if (jtype == "continuous") {
      minval = -M_PI;
      maxval = M_PI;
    } else {
      auto limit = child->FirstChildElement("limit");
      if (!limit) {
        RCLCPP_WARN(this->get_logger(), "%s is not fixed, nor continuous, but limits are not specified!", name.c_str());
        continue;
      }
      
      auto lower = limit->Attribute("lower");
      auto upper = limit->Attribute("upper");
      if (!lower || !upper) {
        RCLCPP_WARN(this->get_logger(), "%s limits are not valid!", name.c_str());
        continue;
      }
      
      try {
        minval = std::stod(lower);
        maxval = std::stod(upper);
      } catch (const std::invalid_argument &) {
        RCLCPP_WARN(this->get_logger(), "%s limits are not valid!", name.c_str());
        continue;
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    // Apply safety tags if needed
    if (use_small_) {
      auto safety_controller = child->FirstChildElement("safety_controller");
      if (safety_controller) {
        if (auto soft_lower = safety_controller->Attribute("soft_lower_limit")) {
          try {
            minval = std::max(minval, std::stod(soft_lower));
          } catch (const std::invalid_argument &) {
            // Ignore invalid value
          }
        }
        if (auto soft_upper = safety_controller->Attribute("soft_upper_limit")) {
          try {
            maxval = std::min(maxval, std::stod(soft_upper));
          } catch (const std::invalid_argument &) {
            // Ignore invalid value
          }
        }
      }
    }
    
    double zeroval;
    if (zeros_.find(name) != zeros_.end()) {
      zeroval = zeros_[name];
    } else if (minval > 0 || maxval < 0) {
      zeroval = (maxval + minval) / 2.0;
    } else {
      zeroval = 0.0;
    }
    
    // Check if this is a mimic joint
    if (use_mimic_) {
      auto mimic = child->FirstChildElement("mimic");
      if (mimic) {
        if (auto parent = mimic->Attribute("joint")) {
          DependentJoint dep_joint;
          dep_joint.parent = parent;
          
          if (auto factor = mimic->Attribute("multiplier")) {
            try {
              dep_joint.factor = std::stod(factor);
            } catch (const std::invalid_argument &) {
              dep_joint.factor = 1.0;
            }
          }
          
          if (auto offset = mimic->Attribute("offset")) {
            try {
              dep_joint.offset = std::stod(offset);
            } catch (const std::invalid_argument &) {
              dep_joint.offset = 0.0;
            }
          }
          
          dependent_joints_[name] = dep_joint;

          joint_list_.push_back(name);
          continue;
        }
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // Check if this joint is already a dependent joint
    if (dependent_joints_.find(name) != dependent_joints_.end()) {
      continue;
    }

    auto joint = initJoint(minval, maxval, zeroval, jtype == "continuous");
    joint.idx = joint_idx;
    RCLCPP_INFO(this->get_logger(), "joint %s idx %d", name.c_str(), joint.idx);
    joint_idx++;
    free_joints_[name] = joint;
    joint_list_.push_back(name);
  }
}

void RobotStatePublisher::update(double delta)
{
  for (auto & [name, joint] : free_joints_) {
    if (joint.forward) {
      joint.position += delta;
      if (joint.position > joint.max) {
        if (joint.continuous) {
          joint.position = joint.min;
        } else {
          joint.position = joint.max;
          joint.forward = false;
        }
      }
    } else {
      joint.position -= delta;
      if (joint.position < joint.min) {
        joint.position = joint.min;
        joint.forward = true;
      }
    }
  }
}

void RobotStatePublisher::timerCallback()
{
  // Publish Joint States
  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->header.stamp = this->now();
  
  if (delta_ > 0.0) {
    update(delta_);
  }
  
  // Initialize msg.position, msg.velocity, and msg.effort.
  bool has_position = !dependent_joints_.empty();
  bool has_velocity = false;
  bool has_effort = false;
  
  for (const auto & [name, joint] : free_joints_) {
    if (!has_position) {
      has_position = true;  // Free joints always have position
    }
    if (!has_velocity && joint.velocity != 0.0) {
      has_velocity = true;
    }
    if (!has_effort && joint.effort != 0.0) {
      has_effort = true;
    }
  }
  
  size_t num_joints = free_joints_.size() + dependent_joints_.size();
  if (has_position) {
    msg->position.resize(num_joints, 0.0);
  }
  if (has_velocity) {
    msg->velocity.resize(num_joints, 0.0);
  }
  if (has_effort) {
    msg->effort.resize(num_joints, 0.0);
  }
  
  for (size_t i = 0; i < joint_list_.size(); ++i) {
    const std::string & name = joint_list_[i];
    msg->name.push_back(name);
    
    const Joint * joint = nullptr;
    double factor = 1.0;
    double offset = 0.0;
    
    // Add Free Joint
    auto free_it = free_joints_.find(name);
    if (free_it != free_joints_.end()) {
      joint = &free_it->second;
    } 
    // Add Dependent Joint
    else {
      auto dep_it = dependent_joints_.find(name);
      if (dep_it != dependent_joints_.end()) {
        std::string parent = dep_it->second.parent;
        factor = dep_it->second.factor;
        offset = dep_it->second.offset;
        
        // Handle recursive mimic chain
        std::vector<std::string> recursive_mimic_chain_joints = {name};
        while (dependent_joints_.find(parent) != dependent_joints_.end()) {
          if (std::find(recursive_mimic_chain_joints.begin(), recursive_mimic_chain_joints.end(), parent) != recursive_mimic_chain_joints.end()) {
            RCLCPP_ERROR(this->get_logger(), "Found an infinite recursive mimic chain:");
            for (const auto & joint_name : recursive_mimic_chain_joints) {
              RCLCPP_ERROR(this->get_logger(), "  %s", joint_name.c_str());
            }
            RCLCPP_ERROR(this->get_logger(), "  %s", parent.c_str());
            rclcpp::shutdown();
            exit(1);
          }
          
          recursive_mimic_chain_joints.push_back(parent);
          auto parent_dep = dependent_joints_.find(parent);
          parent = parent_dep->second.parent;
          offset += factor * parent_dep->second.offset;
          factor *= parent_dep->second.factor;
        }
        
        auto free_parent_it = free_joints_.find(parent);
        if (free_parent_it != free_joints_.end()) {
          joint = &free_parent_it->second;
        }
      }
    }
    
    if (joint) {
      if (has_position) {
        msg->position[i] = joint->position * factor + offset;
      }
      if (has_velocity) {
        msg->velocity[i] = joint->velocity * factor;
      }
      if (has_effort) {
        msg->effort[i] = joint->effort;
      }
    }
  }
  
  // Only publish non-empty messages
  if (!msg->name.empty() || !msg->position.empty() || !msg->velocity.empty() || !msg->effort.empty()) {
    pub_->publish(*msg);
  }
}

Joint RobotStatePublisher::initJoint(double minval, double maxval, double zeroval, bool continuous)
{
  Joint joint;
  joint.min = minval;
  joint.max = maxval;
  joint.zero = zeroval;
  joint.continuous = continuous;
  
  if (pub_def_positions_) {
    joint.position = zeroval;
  }
  if (pub_def_vels_) {
    joint.velocity = 0.0;
  }
  if (pub_def_efforts_) {
    joint.effort = 0.0;
  }
  
  return joint;
}

KDL::Tree RobotStatePublisher::parseURDF(const std::string & urdf_xml, urdf::Model & model)
{
  // Initialize the model
  if (!model.initString(urdf_xml)) {
    throw std::runtime_error("Unable to initialize urdf::model from robot description");
  }

  // Initialize the KDL tree
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::runtime_error("Failed to extract kdl tree from robot description");
  }

  return tree;
}

void RobotStatePublisher::setupURDF(const std::string & urdf_xml)
{
  urdf::Model model;
  KDL::Tree tree = parseURDF(urdf_xml, model);

  // Initialize the mimic map
  mimic_.clear();
  for (const std::pair<const std::string, urdf::JointSharedPtr> & i : model.joints_) {
    if (i.second->mimic) {
      // Just taking a reference to the model shared pointers ends up in a crash.
      // Explicitly make a copy of the JointMimic.
      auto jm = std::make_shared<urdf::JointMimic>();
      jm->offset = i.second->mimic->offset;
      jm->multiplier = i.second->mimic->multiplier;
      jm->joint_name = i.second->mimic->joint_name;
      mimic_[i.first] = jm;
    }
  }

  KDL::SegmentMap segments_map = tree.getSegments();
  for (const std::pair<const std::string, KDL::TreeElement> & segment : segments_map) {
    RCLCPP_DEBUG(get_logger(), "Got segment %s", segment.first.c_str());
  }

  // walk the tree and add segments to segments_
  segments_.clear();
  segments_fixed_.clear();
  addChildren(model, tree.getRootSegment());

  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = urdf_xml;

  // Publish the robot description
  description_pub_->publish(std::move(msg));

  RCLCPP_INFO(get_logger(), "Robot initialized");
}

// add children to correct maps
void RobotStatePublisher::addChildren(
  const urdf::Model & model,
  const KDL::SegmentMap::const_iterator segment)
{
  const std::string & root = GetTreeElementSegment(segment->second).getName();

  std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
  for (unsigned int i = 0; i < children.size(); i++) {
    const KDL::Segment & child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model.getJoint(child.getJoint().getName()) &&
        model.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
      {
        RCLCPP_DEBUG(
          get_logger(), "Floating joint is not supported; skipping segment from %s to %s.",
          root.c_str(), child.getName().c_str());
      } else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        RCLCPP_DEBUG(
          get_logger(), "Adding fixed segment from %s to %s", root.c_str(),
          child.getName().c_str());
      }
    } else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      RCLCPP_DEBUG(
        get_logger(), "Adding moving segment from %s to %s", root.c_str(),
        child.getName().c_str());
    }
    addChildren(model, children[i]);
  }
}

// publish moving transforms
void RobotStatePublisher::publishTransforms(
  const std::map<std::string, double> & joint_positions,
  const builtin_interfaces::msg::Time & time)
{
  RCLCPP_DEBUG(get_logger(), "Publishing transforms for moving joints");

  std::string frame_prefix = get_parameter("frame_prefix").get_value<std::string>();

  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

  // loop over all joints
  for (const std::pair<const std::string, double> & jnt : joint_positions) {
    std::map<std::string, SegmentPair>::iterator seg = segments_.find(jnt.first);
    if (seg != segments_.end()) {
      geometry_msgs::msg::TransformStamped tf_transform =
        kdlToTransform(seg->second.segment.pose(jnt.second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = frame_prefix + seg->second.root;
      tf_transform.child_frame_id = frame_prefix + seg->second.tip;
      tf_transforms.push_back(tf_transform);
    }
  }
  tf_broadcaster_->sendTransform(tf_transforms);
}

// publish fixed transforms
void RobotStatePublisher::publishFixedTransforms()
{
  RCLCPP_DEBUG(get_logger(), "Publishing transforms for fixed joints");

  std::string frame_prefix = get_parameter("frame_prefix").get_value<std::string>();

  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

  // loop over all fixed segments
  rclcpp::Time now = this->now();
  for (const std::pair<const std::string, SegmentPair> & seg : segments_fixed_) {
    geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg.second.segment.pose(0));
    tf_transform.header.stamp = now;

    tf_transform.header.frame_id = frame_prefix + seg.second.root;
    tf_transform.child_frame_id = frame_prefix + seg.second.tip;
    tf_transforms.push_back(tf_transform);
  }
  static_tf_broadcaster_->sendTransform(tf_transforms);
}

void RobotStatePublisher::callbackJointState(
  const sensor_msgs::msg::JointState::ConstSharedPtr state)
{
  if (state->name.size() != state->position.size()) {
    if (state->position.empty()) {
      RCLCPP_WARN(
        get_logger(), "Robot state publisher ignored a JointState message about joint(s) "
        "\"%s\"(,...) whose position member was empty.", state->name[0].c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Robot state publisher ignored an invalid JointState message");
    }
    return;
  }

  // check if we moved backwards in time (e.g. when playing a bag file)
  rclcpp::Time now = this->now();
  if (last_callback_time_.nanoseconds() > now.nanoseconds()) {
    // force re-publish of joint ransforms
    RCLCPP_WARN(
      get_logger(), "Moved backwards in time, re-publishing joint transforms!");
    last_publish_time_.clear();
  }
  last_callback_time_ = now;

  // determine least recently published joint
  rclcpp::Time last_published = now;
  for (size_t i = 0; i < state->name.size(); i++) {
    rclcpp::Time t(last_publish_time_[state->name[i]]);
    last_published = (t.nanoseconds() < last_published.nanoseconds()) ? t : last_published;
  }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.

  // check if we need to publish
  rclcpp::Time current_time(state->header.stamp);
  double publish_freq = this->get_parameter("publish_frequency").get_value<double>();
  std::chrono::milliseconds publish_interval_ms =
    std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / publish_freq));
  rclcpp::Time max_publish_time = last_published + rclcpp::Duration(publish_interval_ms);
  if (get_parameter("ignore_timestamp").get_value<bool>() ||
    current_time.nanoseconds() >= max_publish_time.nanoseconds())
  {
    // get joint positions from state message
    std::map<std::string, double> joint_positions;
    for (size_t i = 0; i < state->name.size(); i++) {
      joint_positions.insert(std::make_pair(state->name[i], state->position[i]));
    }

    for (const std::pair<const std::string, urdf::JointMimicSharedPtr> & i : mimic_) {
      if (joint_positions.find(i.second->joint_name) != joint_positions.end()) {
        double pos = joint_positions[i.second->joint_name] * i.second->multiplier +
          i.second->offset;
        joint_positions.insert(std::make_pair(i.first, pos));
      }
    }

    publishTransforms(joint_positions, state->header.stamp);

    // store publish time in joint map
    for (size_t i = 0; i < state->name.size(); i++) {
      last_publish_time_[state->name[i]] = state->header.stamp;
    }
  }
}

rcl_interfaces::msg::SetParametersResult RobotStatePublisher::parameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const rclcpp::Parameter & parameter : parameters) {
    if (parameter.get_name() == "robot_description") {
      std::string new_urdf = parameter.as_string();
      // Ensure that it isn't empty
      if (new_urdf.empty()) {
        result.successful = false;
        result.reason = "Empty URDF is not allowed";
        break;
      }

      // And that we can successfully parse it
      try {
        urdf::Model dummy_model;
        parseURDF(new_urdf, dummy_model);
      } catch (const std::runtime_error & err) {
        RCLCPP_WARN(get_logger(), "%s", err.what());
        result.successful = false;
        result.reason = err.what();
        break;
      }
    } else if (parameter.get_name() == "publish_frequency") {
      double publish_freq = parameter.as_double();
      if (publish_freq < 0.0 || publish_freq > 1000.0) {
        result.successful = false;
        result.reason = "publish_frequency must be between 0.0 and 1000.0";
        break;
      }
    }
  }

  return result;
}

void RobotStatePublisher::onParameterEvent(
  std::shared_ptr<const rcl_interfaces::msg::ParameterEvent> event)
{
  // Filter out events from other nodes
  if (event->node != this->get_fully_qualified_name()) {
    return;
  }

  // Filter for 'robot_description' being changed.
  rclcpp::ParameterEventsFilter filter(event, {"robot_description"},
    {rclcpp::ParameterEventsFilter::EventType::CHANGED});
  for (auto & it : filter.get_events()) {
    if (it.second->name == "robot_description") {
      try {
        setupURDF(it.second->value.string_value);
        publishFixedTransforms();
      } catch (const std::runtime_error & err) {
        RCLCPP_WARN(get_logger(), "Failed to parse new URDF: %s", err.what());
      }
    }
  }
}

}  // namespace robot_state_publisher


int main(int argc, char *argv[])
{
  rclcpp::NodeOptions options;
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  options.arguments(args);
  auto robot_vis_node = std::make_shared<magic::RobotStatePublisher>(options);
//rclcpp::spin(node);
  rclcpp::spin(robot_vis_node);

  rclcpp::shutdown();
  return 0;
}

// int main(int argc, char * argv[])
// {
//   auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
//   rclcpp::Logger logger = rclcpp::get_logger(NODE_MAIN_LOGGER_NAME);
//   SingleThreadedExecutor exec;
//   rclcpp::NodeOptions options;
//   options.arguments(args);

//   std::string library_name = "libmagic_vis_ros2_node.so";
//   std::string class_name = "rclcpp_components::NodeFactoryTemplate<robot_state_publisher::RobotStatePublisher>";

//   RCLCPP_DEBUG(logger, "Load library %s", library_name.c_str());
//   auto loader = std::make_unique<class_loader::ClassLoader>(library_name);
//   std::vector<std::string> classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();

//   if (std::find(
//     classes.begin(),
//     classes.end(),
//     class_name) == classes.end()) {
//     RCLCPP_ERROR(
//       logger,
//       "Class %s not found in library %s",
//       class_name.c_str(),
//       library_name.c_str());
//     return 1;
//   }
//   RCLCPP_DEBUG(logger, "Instantiate class %s", class_name.c_str());
//   std::shared_ptr<rclcpp_components::NodeFactory> node_factory = nullptr;
//   try {
//     node_factory = loader->createInstance<rclcpp_components::NodeFactory>(class_name);
//   } catch (const std::exception & ex) {
//     RCLCPP_ERROR(logger, "Failed to load library %s", ex.what());
//     return 1;
//   } catch (...) {
//     RCLCPP_ERROR(logger, "Failed to load library");
//     return 1;
//   }
//   // Scope to destruct node_wrapper before shutdown
//   {
//     rclcpp_components::NodeInstanceWrapper node_wrapper = node_factory->create_node_instance(options);
//     rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node = node_wrapper.get_node_base_interface();
//     exec.add_node(node);

//     exec.spin();

//     exec.remove_node(node_wrapper.get_node_base_interface());
//   }

//   rclcpp::shutdown();

//   return 0;
// }