#include "joint_state_publisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace joint_state_publisher
{

JointStatePublisher::JointStatePublisher(const std::string & description_file)
  : Node("joint_state_publisher", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
{
  // Declare parameters
  rcl_interfaces::msg::ParameterDescriptor bool_desc;
  bool_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
  
  rcl_interfaces::msg::ParameterDescriptor int_desc;
  int_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  
  rcl_interfaces::msg::ParameterDescriptor double_desc;
  double_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  
  rcl_interfaces::msg::ParameterDescriptor string_array_desc;
  string_array_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY;
  
  declareRosParameter("publish_default_efforts", false, bool_desc);
  declareRosParameter("publish_default_positions", true, bool_desc);
  declareRosParameter("publish_default_velocities", false, bool_desc);
  declareRosParameter("rate", 10, int_desc);
  declareRosParameter("source_list", std::vector<std::string>(), string_array_desc);
  declareRosParameter("use_mimic_tags", true, bool_desc);
  declareRosParameter("use_smallest_joint_limits", true, bool_desc);
  declareRosParameter("delta", 0.0, double_desc);
  
  // Get parameters
  pub_def_positions_ = this->get_parameter("publish_default_positions").as_bool();
  pub_def_vels_ = this->get_parameter("publish_default_velocities").as_bool();
  pub_def_efforts_ = this->get_parameter("publish_default_efforts").as_bool();
  use_mimic_ = this->get_parameter("use_mimic_tags").as_bool();
  use_small_ = this->get_parameter("use_smallest_joint_limits").as_bool();
  
  // Parse zeros parameter
  auto zeros_params = this->get_parameters_by_prefix("zeros");
  for (const auto & [name, param] : zeros_params) {
    zeros_[name] = param.as_double();
  }
  
  // Parse dependent joints
  dependent_joints_ = parseDependentJoints();
  
  // Configure robot
  if (!description_file.empty()) {
    std::ifstream file(description_file);
    if (file.is_open()) {
      std::stringstream buffer;
      buffer << file.rdbuf();
      configureRobot(buffer.str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Could not open description file: %s", description_file.c_str());
    }
  } else {
    // Subscribe to robot_description topic
    RCLCPP_INFO(this->get_logger(), "Waiting for robot_description to be published on the robot_description topic...");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description", qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        this->configureRobot(msg->data);
      });
  }
  
  delta_ = this->get_parameter("delta").as_double();
  
  // Subscribe to source_list
  auto source_list = this->get_parameter("source_list").as_string_array();
  for (const auto & source : source_list) {
    sources_.push_back(this->create_subscription<sensor_msgs::msg::JointState>(
      source, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->sourceCallback(msg);
      }));
  }
  
  // Create publisher
  pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  
  // Create timer
  auto rate = this->get_parameter("rate").as_int();
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / rate),
    [this]() {
      this->timerCallback();
    });
}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::declareRosParameter(const std::string & name, const rclcpp::ParameterValue & default_value, const rclcpp::ParameterDescriptor & descriptor)
{
  try {
    this->declare_parameter(name, default_value, descriptor);
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
    // Ignore if parameter is already declared
  }
}

Joint JointStatePublisher::initJoint(double minval, double maxval, double zeroval, bool continuous)
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

void JointStatePublisher::configureRobot(const std::string & description)
{
  RCLCPP_INFO(this->get_logger(), "Got description, configuring robot");
  
  tinyxml2::XMLDocument doc;
  if (doc.Parse(description.c_str()) != tinyxml2::XML_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "Invalid robot_description given, ignoring");
    return;
  }
  
  // Clear old joints
  free_joints_.clear();
  joint_list_.clear();
  
  // Get root tag to parse file format
  auto root = doc.RootElement();
  if (root) {
    if (std::string(root->Name()) == "sdf") {
      initSdf(doc);
    } else if (std::string(root->Name()) == "COLLADA") {
      initCollada(doc);
    } else {
      initUrdf(doc);
    }
  }
  
  if (robot_description_update_cb_) {
    robot_description_update_cb_();
  }
}

void JointStatePublisher::initUrdf(tinyxml2::XMLDocument & doc)
{
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
      
      auto lower = limit->FirstChildElement("lower");
      auto upper = limit->FirstChildElement("upper");
      if (!lower || !upper || !lower->GetText() || !upper->GetText()) {
        RCLCPP_WARN(this->get_logger(), "%s limits are not valid!", name.c_str());
        continue;
      }
      
      try {
        minval = std::stod(lower->GetText());
        maxval = std::stod(upper->GetText());
      } catch (const std::invalid_argument &) {
        RCLCPP_WARN(this->get_logger(), "%s limits are not valid!", name.c_str());
        continue;
      }
    }
    
    // Apply safety tags if needed
    if (use_small_) {
      RCLCPP_INFO(this->get_logger(), "Publishing joint states 5555555555555555555555555555555555555555: %s", name.c_str());
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
      RCLCPP_INFO(this->get_logger(), "Publishing joint states 444444444444444444444444444444444444: %s", name.c_str());
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
    
    // Check if this joint is already a dependent joint
    if (dependent_joints_.find(name) != dependent_joints_.end()) {
      continue;
    }
    
    auto joint = initJoint(minval, maxval, zeroval, jtype == "continuous");
    free_joints_[name] = joint;
    joint_list_.push_back(name);
  }
}

void JointStatePublisher::initSdf(tinyxml2::XMLDocument & doc)
{
  auto model = doc.FirstChildElement("sdf")->FirstChildElement("model");
  if (!model) {
    RCLCPP_WARN(this->get_logger(), "No model element found in SDF");
    return;
  }
  
  // Find all non-fixed joints
  for (auto child = model->FirstChildElement("joint"); child; child = child->NextSiblingElement("joint")) {
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
      
      auto lower = limit->FirstChildElement("lower");
      auto upper = limit->FirstChildElement("upper");
      if (!lower || !upper || !lower->GetText() || !upper->GetText()) {
        RCLCPP_WARN(this->get_logger(), "%s limits are not valid!", name.c_str());
        continue;
      }
      
      try {
        minval = std::stod(lower->GetText());
        maxval = std::stod(upper->GetText());
      } catch (const std::invalid_argument &) {
        RCLCPP_WARN(this->get_logger(), "%s limits are not valid!", name.c_str());
        continue;
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
    
    auto joint = initJoint(minval, maxval, zeroval, jtype == "continuous");
    free_joints_[name] = joint;
    joint_list_.push_back(name);
  }
}

void JointStatePublisher::initCollada(tinyxml2::XMLDocument & doc)
{
  auto kinematics_model = doc.FirstChildElement("COLLADA")->FirstChildElement("kinematics_model");
  if (!kinematics_model) {
    RCLCPP_WARN(this->get_logger(), "No kinematics_model element found in COLLADA");
    return;
  }
  
  auto technique_common = kinematics_model->FirstChildElement("technique_common");
  if (!technique_common) {
    RCLCPP_WARN(this->get_logger(), "No technique_common element found in COLLADA");
    return;
  }
  
  for (auto child = technique_common->FirstChildElement("joint"); child; child = child->NextSiblingElement("joint")) {
    std::string name = child->Attribute("name");
    auto joint = child->FirstChildElement("revolute");
    if (!joint) {
      RCLCPP_WARN(this->get_logger(), "Unknown joint type for joint %s", name.c_str());
      continue;
    }
    
    auto limit = joint->FirstChildElement("limits");
    if (!limit) {
      continue;
    }
    
    auto min = limit->FirstChildElement("min");
    auto max = limit->FirstChildElement("max");
    if (!min || !max || !min->GetText() || !max->GetText()) {
      continue;
    }
    
    try {
      double minval = std::stod(min->GetText()) * M_PI / 180.0;
      double maxval = std::stod(max->GetText()) * M_PI / 180.0;
      
      if (minval == maxval) {
        continue;  // This is a fixed joint
      }
      
      joint_list_.push_back(name);
      auto joint_obj = initJoint(minval, maxval, 0.0);
      free_joints_[name] = joint_obj;
    } catch (const std::invalid_argument &) {
      // Ignore invalid values
    }
  }
}

std::map<std::string, DependentJoint> JointStatePublisher::parseDependentJoints()
{
  std::map<std::string, DependentJoint> result;
  auto dependent_joints_params = this->get_parameters_by_prefix("dependent_joints");
  
  for (const auto & [name, param] : dependent_joints_params) {
    // Split on the dots; there should be one and exactly one dot
    auto dot_pos = name.find('.');
    if (dot_pos == std::string::npos || dot_pos == name.size() - 1) {
      throw std::runtime_error("Invalid dependent_joint name '" + name + "'");
    }
    
    std::string newkey = name.substr(0, dot_pos);
    std::string newvalue = name.substr(dot_pos + 1);
    
    if (newvalue != "parent" && newvalue != "factor" && newvalue != "offset") {
      throw std::runtime_error("Invalid dependent_joint name '" + name + "' (allowed values are 'parent', 'factor', and 'offset')");
    }
    
    if (result.find(newkey) == result.end()) {
      result[newkey] = DependentJoint();
    }
    
    if (newvalue == "parent") {
      result[newkey].parent = param.as_string();
    } else if (newvalue == "factor") {
      result[newkey].factor = param.as_double();
    } else if (newvalue == "offset") {
      result[newkey].offset = param.as_double();
    }
  }
  
  // Ensure that there is at least a 'parent' in all keys
  for (const auto & [name, outdict] : result) {
    if (outdict.parent.empty()) {
      throw std::runtime_error("All dependent_joints must at least have a parent");
    }
  }
  
  return result;
}

void JointStatePublisher::sourceCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string & name = msg->name[i];
    auto it = free_joints_.find(name);
    if (it == free_joints_.end()) {
      continue;
    }
    
    Joint & joint = it->second;
    if (i < msg->position.size()) {
      joint.position = msg->position[i];
    }
    if (i < msg->velocity.size()) {
      joint.velocity = msg->velocity[i];
    }
    if (i < msg->effort.size()) {
      joint.effort = msg->effort[i];
    }
  }
  
  if (source_update_cb_) {
    source_update_cb_();
  }
}

void JointStatePublisher::timerCallback()
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

void JointStatePublisher::update(double delta)
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

void JointStatePublisher::setRobotDescriptionUpdateCallback(std::function<void()> callback)
{
  robot_description_update_cb_ = callback;
}

void JointStatePublisher::setSourceUpdateCallback(std::function<void()> callback)
{
  source_update_cb_ = callback;
}

}  // namespace joint_state_publisher

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Parse command line arguments
  std::string description_file;
  if (argc > 1) {
    description_file = argv[1];
  }
  
  auto node = std::make_shared<joint_state_publisher::JointStatePublisher>(description_file);
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node->get_logger(), "Exception in spin: %s", ex.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Unknown exception in spin");
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}