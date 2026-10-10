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
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "urdf/model.h"
#include "yaml-cpp/yaml.h"


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

std::string readTextFileToString(const std::string& file_path) {
    std::string file_content;
    // 1. 以二进制模式打开文件（避免Windows下换行符转换问题），并定位到文件末尾
    std::ifstream file(file_path, std::ios::in | std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cout << "无法打开文件: " << file_path << std::endl;
        return file_content;
    }

    try {
        // 2. 获取文件大小（ate模式下，指针已在文件末尾，tellg()返回文件字节数）
        std::streamsize file_size = file.tellg();
        if (file_size < 0) {
            throw std::runtime_error("获取文件大小失败: " + file_path);
        }

        // 3. 调整string容量，避免多次内存分配（优化性能）
        
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
    }
    return file_content;
}

RobotStatePublisher::RobotStatePublisher()
{
  // std::string yaml_file_path_ = this->declare_parameter("yaml_file_path", "magic_vis_ros25_human.yaml");
  // std::cout << "YAML file path: " << yaml_file_path_ << std::endl;
  // try {
  //   YAML::Node config = YAML::LoadFile(yaml_file_path_);

  //   leg_start_index_ = config["legs"]["start_index"].as<int>();
  //   leg_count_ = config["legs"]["count"].as<int>();
  //   std::cout << "\nleg_count: " << leg_count_ << std::endl;
  //   std::cout << "\nleg_start_index: " << leg_start_index_ << std::endl;
    
  //   waist_start_index_ = config["waist"]["start_index"].as<int>();
  //   waist_count_ = config["waist"]["count"].as<int>();
  //   std::cout << "\nwaist_count: " << waist_count_ << std::endl;
  //   std::cout << "\nwaist_start_index: " << waist_start_index_ << std::endl;

  //   head_start_index_ = config["head"]["start_index"].as<int>();
  //   head_count_ = config["head"]["count"].as<int>();
  //   std::cout << "\nhead_count: " << head_count_ << std::endl;
  //   std::cout << "\nhead_start_index: " << head_start_index_ << std::endl;

  //   arm_start_index_ = config["arms"]["start_index"].as<int>();
  //   arm_count_ = config["arms"]["count"].as<int>();
  //   arm_free_count_ = config["arms"]["free_count"].as<int>();
  //   std::cout << "\narm_count: " << arm_count_ << std::endl;
  //   std::cout << "\narm_start_index_: " << arm_start_index_ << std::endl;
  //   std::cout << "\narm_free_count_: " << arm_free_count_ << std::endl;

  //   arm_start_index_1 = config["arms1"]["start_index"].as<int>();
  //   arm_count_1 = config["arms1"]["count"].as<int>();
  //   std::cout << "\narm_count_1: " << arm_count_1 << std::endl;
  //   std::cout << "\narm_start_index_1: " << arm_start_index_1 << std::endl;
  // } catch (const YAML::Exception& e) {
  //   std::cout << "YAML 解析错误: " << e.what() << std::endl;
  // }

  
  // // 声明URDF文件路径参数
  // std::string urdf_file_path = "x1.urdf";
  // std::cout << "URDF file path: " << urdf_file_path << std::endl;
  // // get the XML
  // std::string urdf_xml = readTextFileToString(urdf_file_path);

  // tinyxml2::XMLDocument doc;
  // if (doc.Parse(urdf_xml.c_str()) != tinyxml2::XML_SUCCESS) {
  //   std::cout << "Invalid robot_description given, ignoring\n";
  //   return;
  // }

  // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  // static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  // setupURDF(urdf_xml);

  // initUrdf(doc);
  // std::string xml_file_path = "x1.xml";
  // reRank(joint_list_, xml_file_path);

  // auto subscriber_options = rclcpp::SubscriptionOptions();
  // subscriber_options.qos_overriding_options =
  //   rclcpp::QosOverridingOptions::with_default_policies();

 // publishFixedTransforms();
}

void RobotStatePublisher::init(const std::string& urdf_file_path, const std::string& xml_file_path, std::function<void(const std::string&)> callback){

  std::string urdf_xml = readTextFileToString(urdf_file_path);
  tinyxml2::XMLDocument doc;
  if (doc.Parse(urdf_xml.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cout << "Invalid robot_description given, ignoring\n";
    return;
  }
  
  waist_start_index_ = 12;
  arm_start_index_ = 17;
  
  setupURDF(urdf_xml);

  initUrdf(doc);
  reRank(joint_list_, xml_file_path);

  publishFixedTransforms();



  callback(urdf_xml);
}

void RobotStatePublisher::publishTransforms(std::function<void(const std::vector<geometry_msgs::msg::TransformStamped>&)> callback2,
std::function<void(const sensor_msgs::msg::JointState&)> callback3) {
  
  // Publish Joint States
  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->header.stamp.sec = 0;
msg->header.stamp.nanosec = 0;
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
            for (const auto & joint_name : recursive_mimic_chain_joints) {
            }
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
        // RCLCPP_DEBUG(this->get_logger(), "%s:%d, joint %s position is %f, factor is %f, offset is %f",
        //              __func__, __LINE__, name.c_str(), joint->position, factor, offset);
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


  


  callback3(*msg);





  // check if we moved backwards in time (e.g. when playing a bag file)
  rclcpp::Time now;


  // determine least recently published joint
  rclcpp::Time last_published = now;
  for (size_t i = 0; i < msg->name.size(); i++) {
    rclcpp::Time t(last_publish_time_[msg->name[i]]);
    last_published = (t.nanoseconds() < last_published.nanoseconds()) ? t : last_published;
  }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.

  // check if we need to publish
  rclcpp::Time current_time(msg->header.stamp);

    // get joint positions from state message
    std::map<std::string, double> joint_positions;
    for (size_t i = 0; i < msg->name.size(); i++) {
      joint_positions.insert(std::make_pair(msg->name[i], msg->position[i]));
    }

    for (const std::pair<const std::string, urdf::JointMimicSharedPtr> & i : mimic_) {
      if (joint_positions.find(i.second->joint_name) != joint_positions.end()) {
        double pos = joint_positions[i.second->joint_name] * i.second->multiplier +
          i.second->offset;
        joint_positions.insert(std::make_pair(i.first, pos));
      }
    }



    const builtin_interfaces::msg::Time time = msg->header.stamp;


   
  geometry_msgs::msg::TransformStamped tf_transform;
  tf_transform.header.stamp = time;
  //将根坐标系嫁接到世界坐标系上
  tf_transform.header.frame_id = "world"; // 假设根坐标系是 world
  tf_transform.child_frame_id =  root_link_name_; // 假设机器人基坐标系是 base_link
  
  // 设置平移（如果需要）
  tf_transform.transform.translation.x = 0.0;
  tf_transform.transform.translation.y = 0.0;
  tf_transform.transform.translation.z = 0.0;

  // 设置旋转（使用 IMU 四元数）
  tf_transform.transform.rotation.w = imu_quaternion_.w;
  tf_transform.transform.rotation.x = imu_quaternion_.x;
  tf_transform.transform.rotation.y = imu_quaternion_.y;
  tf_transform.transform.rotation.z = imu_quaternion_.z;

  // 发布变换
  tf_transforms.push_back(tf_transform);
  //tf_broadcaster_->sendTransform(tf_transform);
  
  // loop over all joints
  for (const std::pair<const std::string, double> & jnt : joint_positions) {
    std::map<std::string, SegmentPair>::iterator seg = segments_.find(jnt.first);
   // RCLCPP_DEBUG(get_logger(), "%s:%d, Processing joint %s, position %f", __func__, __LINE__, jnt.first.c_str(), jnt.second);
    if (seg != segments_.end()) {
      geometry_msgs::msg::TransformStamped tf_transform =
        kdlToTransform(seg->second.segment.pose(jnt.second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = seg->second.root;
      tf_transform.child_frame_id = seg->second.tip;
      tf_transforms.push_back(tf_transform);
    }
  }

  callback2(tf_transforms);
}
// void RobotStatePublisher::legStateCallback(const lcm_to_ros2::msg::LegControlDataLcmt::SharedPtr state_msg)
// {
//   for (size_t i = 0; i < leg_count_; i++) {
//     free_joints_[joint_list_[i + leg_start_index_]].position = state_msg->q[i];
//    // free_joints_[joint_list_[i + leg_start_index_]].velocity = state_msg->qd[i];
//   }
// }

// void RobotStatePublisher::stateEstimatorCallback(const lcm_to_ros2::msg::StateEstimatorLcmt::SharedPtr msg) {
//   imu_quaternion_.w = msg->quat[0];
//   imu_quaternion_.x = msg->quat[1];
//   imu_quaternion_.y = msg->quat[2];
//   imu_quaternion_.z = msg->quat[3];
// }

// void RobotStatePublisher::armStateCallback(const lcm_to_ros2::msg::ArmControlDataLcmt::SharedPtr state_msg) 
// {
//   for (size_t i = 0; i < arm_count_; i++) {
//     free_joints_[joint_list_[i + arm_start_index_]].position = state_msg->joint_cur_pos[i];
//     free_joints_[joint_list_[i + arm_start_index_]].velocity = state_msg->joint_cur_vel[i];
//   }

//   for (size_t i = 0; i < arm_count_1; i++) {
//     free_joints_[joint_list_[i + arm_start_index_1]].position = state_msg->joint_cur_pos[i + arm_count_ + arm_free_count_];
//     free_joints_[joint_list_[i + arm_start_index_1]].velocity = state_msg->joint_cur_vel[i + arm_count_ + arm_free_count_];
//   }
// }

void RobotStatePublisher::setArmPos(int idx, double pos) {
  free_joints_[joint_list_[idx + arm_start_index_]].position = pos;
  //free_joints_[joint_list_[27]].position = 0.9;
}

void RobotStatePublisher::setLegPos(int idx, double pos) {
  free_joints_[joint_list_[idx]].position = pos;
}

void RobotStatePublisher::setWaistPos(int idx, double pos) {
  free_joints_[joint_list_[idx + waist_start_index_]].position = pos;
}

void RobotStatePublisher::setStateEstimator(int idx, double pos) {
  switch (idx) {
  case 0:
    imu_quaternion_.w = pos;
    break;
  case 1:
    imu_quaternion_.x = pos;
    break;
  case 2:
    imu_quaternion_.y = pos;
    break;
  case 3:
    imu_quaternion_.z = pos;
    break;
  default:
    break;
  }
}

// void RobotStatePublisher::waistStateCallback(const lcm_to_ros2::msg::WaistControlDataLcmt::SharedPtr state_msg) 
// {
//   for (size_t i = 0; i < waist_count_; i++) {
//     free_joints_[joint_list_[i + waist_start_index_]].position = state_msg->q[i];
//     free_joints_[joint_list_[i + waist_start_index_]].velocity = state_msg->qd[i];
//   }
// }

void RobotStatePublisher::initUrdf(tinyxml2::XMLDocument & doc)
{
  int joint_idx = 0;
  auto robot = doc.FirstChildElement("robot");
  if (!robot) {
    std::cout << "No robot element found in URDF\n";
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
        std::cout << name << " is not fixed, nor continuous, but limits are not specified!\n";
        continue;
      }
      
      auto lower = limit->Attribute("lower");
      auto upper = limit->Attribute("upper");
      if (!lower || !upper) {
        std::cout << name << " limits are not valid!\n";
        continue;
      }
      
      try {
        minval = std::stod(lower);
        maxval = std::stod(upper);
      } catch (const std::invalid_argument &) {
        std::cout << name << " limits are not valid!\n";
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
    // // Check if this joint is already a dependent joint
    // if (dependent_joints_.find(name) != dependent_joints_.end()) {
    //   continue;
    // }

    auto joint = initJoint(minval, maxval, zeroval, jtype == "continuous");
    joint.idx = joint_idx;
    joint_idx++;
    free_joints_[name] = joint;
    std::cout << "joint " << name << " idx " << joint.idx << std::endl;
    joint_list_.push_back(name);
  }
}

void RobotStatePublisher::reRank(std::vector<std::string> &joint_list, const std::string &xml_file_path) {
  // 声明URDF文件路径参数
  std::vector<std::string> new_joint_list;
  std::cout << "XML file path: " << xml_file_path << std::endl;
  // get the XML

  std::string xml_content = readTextFileToString(xml_file_path);
  if (xml_content.empty()) {
    std::cout << "Failed to read XML file content\n";
    return;
  }

  tinyxml2::XMLDocument doc;
  if (doc.Parse(xml_content.c_str()) != tinyxml2::XML_SUCCESS) {
    std::cout << "Invalid magicbot_gen2_v2.xml content given, ignoring\n";
    return;
  }
  
  // Get the mujoco root element
  auto mujoco = doc.FirstChildElement("mujoco");
  if (!mujoco) {
    std::cout << "No mujoco element found in XML\n";
    return;
  }
  
  // Get the actuator element to map actuators to joints
  auto actuator = mujoco->FirstChildElement("actuator");
  if (actuator) {
    std::cout << "Parsing actuator\n";
  
    for (auto motor = actuator->FirstChildElement("motor"); motor; motor = motor->NextSiblingElement("motor")) {
      std::string motor_name = motor->Attribute("name") ? motor->Attribute("name") : "unknown";
      std::string joint_name = motor->Attribute("joint") ? motor->Attribute("joint") : "unknown";
      
      std::cout << "Found actuator: " << motor_name << " mapping to joint: " << joint_name;
      for (auto &j : joint_list) {
        if (j == joint_name) {
          std::cout << " found\n";
          new_joint_list.push_back(joint_name);
        }
      }
      // Here you can add code to map actuators to joints if needed
    }
  }
  if (new_joint_list.size() == joint_list.size()) {
    std::cout << "joint list rank is the same\n";
    joint_list = new_joint_list;
  }
}

void RobotStatePublisher::update(double delta)
{
  for (auto & [name, joint] : free_joints_) {
    std::cout << "Updating joint " << name << ": position " << joint.position << ", forward " << joint.forward << ", delta " << delta << std::endl;
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
  //msg->header.stamp = 1000;//this->now();
  
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
            std::cout << "Found an infinite recursive mimic chain:\n";
            for (const auto & joint_name : recursive_mimic_chain_joints) {
              std::cout << "  " << joint_name << "\n";
            }
            std::cout << "  " << parent << "\n";
          //  rclcpp::shutdown();
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
        // RCLCPP_DEBUG(this->get_logger(), "%s:%d, joint %s position is %f, factor is %f, offset is %f",
        //              __func__, __LINE__, name.c_str(), joint->position, factor, offset);
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
    //pub_->publish(*msg);
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
    std::cout << "Got segment " << segment.first << std::endl;
  }

  // walk the tree and add segments to segments_
  segments_.clear();
  segments_fixed_.clear();

  addChildren(model, tree.getRootSegment());

  std_msgs::msg::String msg;
  msg.data = urdf_xml;


  std::cout << "Robot initialized\n";
}

// add children to correct maps
void RobotStatePublisher::addChildren(
  const urdf::Model & model,
  const KDL::SegmentMap::const_iterator segment)
{
  const std::string & root = GetTreeElementSegment(segment->second).getName();
  if (root == "pelvis") {
    root_link_name_ = "pelvis";
  } else if (root == "base") {
    root_link_name_ = "base";
  }

  std::cout << "root: " << root << std::endl;
  std::vector<KDL::SegmentMap::const_iterator> children = GetTreeElementChildren(segment->second);
  for (unsigned int i = 0; i < children.size(); i++) {
    const KDL::Segment & child = GetTreeElementSegment(children[i]->second);
    SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None) {
      if (model.getJoint(child.getJoint().getName()) &&
        model.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING)
      {
        std::cout << "Skipping segment from " << root << " to " << child.getName() << std::endl;
      } else {
        segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
        std::cout << "Adding fixed segment from " << root << " to " << child.getName() << std::endl;
      }
    } else {
      segments_.insert(make_pair(child.getJoint().getName(), s));
      std::cout << "Adding moving segment from " << root << " to " << child.getName() << std::endl;
    }
    addChildren(model, children[i]);
  }
}

// publish moving transforms
void RobotStatePublisher::publishTransforms(
  const std::map<std::string, double> & joint_positions,
  const builtin_interfaces::msg::Time & time)
{
  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;
  geometry_msgs::msg::TransformStamped tf_transform;
  tf_transform.header.stamp = time;
  //将根坐标系嫁接到世界坐标系上
  tf_transform.header.frame_id = "world"; // 假设根坐标系是 world
  tf_transform.child_frame_id = root_link_name_; // 假设机器人基坐标系是 base_link
  
  // 设置平移（如果需要）
  tf_transform.transform.translation.x = 0.0;
  tf_transform.transform.translation.y = 0.0;
  tf_transform.transform.translation.z = 0.0;

  // 设置旋转（使用 IMU 四元数）
  tf_transform.transform.rotation.w = imu_quaternion_.w;
  tf_transform.transform.rotation.x = imu_quaternion_.x;
  tf_transform.transform.rotation.y = imu_quaternion_.y;
  tf_transform.transform.rotation.z = imu_quaternion_.z;

  // 发布变换
  tf_transforms.push_back(tf_transform);
  //tf_broadcaster_->sendTransform(tf_transform);
  
  // loop over all joints
  for (const std::pair<const std::string, double> & jnt : joint_positions) {
    std::map<std::string, SegmentPair>::iterator seg = segments_.find(jnt.first);
   // RCLCPP_DEBUG(get_logger(), "%s:%d, Processing joint %s, position %f", __func__, __LINE__, jnt.first.c_str(), jnt.second);
    if (seg != segments_.end()) {
      geometry_msgs::msg::TransformStamped tf_transform =
        kdlToTransform(seg->second.segment.pose(jnt.second));
      tf_transform.header.stamp = time;
      tf_transform.header.frame_id = seg->second.root;
      tf_transform.child_frame_id = seg->second.tip;
      tf_transforms.push_back(tf_transform);
    }
  }
  //tf_broadcaster_->sendTransform(tf_transforms);
}

// publish fixed transforms
void RobotStatePublisher::publishFixedTransforms()
{
  std::cout << "Publishing transforms for fixed joints\n";

  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms;

  // loop over all fixed segments
  //rclcpp::Time now = this->now();
  for (const std::pair<const std::string, SegmentPair> & seg : segments_fixed_) {
    geometry_msgs::msg::TransformStamped tf_transform = kdlToTransform(seg.second.segment.pose(0));
    //tf_transform.header.stamp = now;

    tf_transform.header.frame_id = seg.second.root;
    tf_transform.child_frame_id = seg.second.tip;
    tf_transforms.push_back(tf_transform);
  }
 // static_tf_broadcaster_->sendTransform(tf_transforms);
}

void RobotStatePublisher::callbackJointState(
  const sensor_msgs::msg::JointState::ConstSharedPtr state)
{
  // RCLCPP_INFO(get_logger(), "Received JointState message with %zu joints", state->name.size());
  // RCLCPP_INFO(get_logger(), "Received JointState message with %zu positions", state->position.size());

  // if (state->name.size() != state->position.size()) {
  //   if (state->position.empty()) {
  //     RCLCPP_WARN(
  //       get_logger(), "%s:%d, Robot state publisher ignored a JointState message about joint(s) "
  //       "\"%s\"(,...) whose position member was empty.", __func__, __LINE__, state->name[0].c_str());
  //   } else {
  //     RCLCPP_ERROR(get_logger(), "%s:%d, Robot state publisher ignored an invalid JointState message", __func__, __LINE__);
  //   }
  //   return;
  // }

  // check if we moved backwards in time (e.g. when playing a bag file)
 // rclcpp::Time now = this->now();
//   if (last_callback_time_.nanoseconds() > now.nanoseconds()) {
//     // force re-publish of joint ransforms
//     std::cout << "Moved backwards in time, re-publishing joint transforms!\n";
//     last_publish_time_.clear();
//   }
//  // last_callback_time_ = now;

//   // determine least recently published joint
//  // rclcpp::Time last_published = now;
//   for (size_t i = 0; i < state->name.size(); i++) {
//     rclcpp::Time t(last_publish_time_[state->name[i]]);
//     last_published = (t.nanoseconds() < last_published.nanoseconds()) ? t : last_published;
//   }
  // note: if a joint was seen for the first time,
  //       then last_published is zero.

  // check if we need to publish
 // rclcpp::Time current_time(state->header.stamp);

  std::chrono::milliseconds publish_interval_ms =
    std::chrono::milliseconds(static_cast<uint64_t>(1000.0 / 20));
}

// rcl_interfaces::msg::SetParametersResult RobotStatePublisher::parameterUpdate(
//   const std::vector<rclcpp::Parameter> & parameters)
// {
//   rcl_interfaces::msg::SetParametersResult result;
//   result.successful = true;

//   for (const rclcpp::Parameter & parameter : parameters) {
//     if (parameter.get_name() == "robot_description") {
//       std::string new_urdf = parameter.as_string();
//       // Ensure that it isn't empty
//       if (new_urdf.empty()) {
//         result.successful = false;
//         result.reason = "Empty URDF is not allowed";
//         break;
//       }

//       // And that we can successfully parse it
//       try {
//         urdf::Model dummy_model;
//         parseURDF(new_urdf, dummy_model);
//       } catch (const std::runtime_error & err) {
//         std::cout << "Failed to parse new URDF: " << err.what();
//         result.successful = false;
//         result.reason = err.what();
//         break;
//       }
//     }
//   }

//   return result;
// }