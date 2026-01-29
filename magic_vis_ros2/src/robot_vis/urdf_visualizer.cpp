#include <robot_vis/urdf_visualizer.h>

// 添加必要的头文件
#include <sensor_msgs/msg/joint_state.hpp>
#include <fstream>  // 添加文件操作头文件
#include <sstream>  // 添加字符串流头文件

namespace magic {

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

UrdfVisualizer::UrdfVisualizer(const std::string& urdf_name,
                               const std::vector<std::string>& joint_names_in_urdf,
                               const std::string& base_joint_in_urdf,
                               const std::string& fixed_frame,
                               const std::string& pose_state_topic,
                               const std::string& leg_state_topic, const int leg_id, const int leg_n,
                               const std::string& arm_state_topic, const int arm_id, const int arm_n,
                               const std::string& waist_state_topic, const int waist_id, const int waist_n,
                               const std::string& head_state_topic, const int head_id, const int head_n,
                               const std::string& tf_prefix) :
                               rclcpp::Node("urdf_viaualizer")
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

  // 创建节点作为类成员变量
  node_ = rclcpp::Node::make_shared("urdf_visualizer");
  
  pose_sub_des_ = node_->create_subscription<lcm_to_ros2::msg::StateEstimatorLcmt>(
    pose_state_topic, 1, std::bind(&UrdfVisualizer::poseStateCallback, this, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Subscribed to: %s", pose_sub_des_->get_topic_name());

  if (0 != leg_n) {
    leg_sub_des_ = node_->create_subscription<lcm_to_ros2::msg::LegControlDataLcmt>(
      leg_state_topic, 1, std::bind(&UrdfVisualizer::legStateCallback, this, std::placeholders::_1));
    RCLCPP_DEBUG(node_->get_logger(), "Subscribed to: %s", leg_sub_des_->get_topic_name());
  }

  if (0 != arm_n) {
    arm_sub_des_ = node_->create_subscription<lcm_to_ros2::msg::ArmControlDataLcmt>(
      arm_state_topic, 1, std::bind(&UrdfVisualizer::armStateCallback, this, std::placeholders::_1));
    RCLCPP_DEBUG(node_->get_logger(), "Subscribed to: %s", arm_sub_des_->get_topic_name());
  }

  if (0 != waist_n) {
    waist_sub_des_ = node_->create_subscription<lcm_to_ros2::msg::WaistControlDataLcmt>(
      waist_state_topic, 1, std::bind(&UrdfVisualizer::waistStateCallback, this, std::placeholders::_1));
    RCLCPP_DEBUG(node_->get_logger(), "Subscribed to: %s", waist_sub_des_->get_topic_name());
  }

  if (0 != head_n) {
    head_sub_des_ = node_->create_subscription<lcm_to_ros2::msg::HeadControlDataLcmt>(
      head_state_topic, 1, std::bind(&UrdfVisualizer::headStateCallback, this, std::placeholders::_1));
    RCLCPP_DEBUG(node_->get_logger(), "Subscribed to: %s", head_sub_des_->get_topic_name());
  }

  // 为JointState创建发布器
  joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

  // initialize joint values
  joint_values_ = std::vector<double>(leg_n + arm_n + waist_n + head_n, 0.);
  for (auto jm : joint_names_in_urdf)
    joint_positions_[jm] = 0.;

  // Load model from file
  KDL::Tree my_kdl_tree;
  urdf::Model my_urdf_model;
  
  // In ROS2, we need to get the URDF string from parameter server and use initString
  std::string urdf_string;
  bool model_ok = false;
  
  // 1. 首先尝试从参数服务器获取URDF字符串
  if (node_->get_parameter(urdf_name, urdf_string)) {
    RCLCPP_INFO(node_->get_logger(), "尝试从参数服务器加载URDF: %s", urdf_name.c_str());
    model_ok = my_urdf_model.initString(urdf_string);
    if (model_ok) {
      RCLCPP_INFO(node_->get_logger(), "成功从参数服务器加载URDF");
    } else {
      RCLCPP_WARN(node_->get_logger(), "从参数服务器获取到URDF字符串，但解析失败");
    }
  } else {
    RCLCPP_WARN(node_->get_logger(), "无法从参数服务器获取URDF参数: %s", urdf_name.c_str());
  }
  
  // 2. 如果从参数服务器加载失败，尝试将urdf_name作为文件路径直接加载
  if (!model_ok) {
    RCLCPP_INFO(node_->get_logger(), "尝试从文件路径加载URDF: %s", urdf_name.c_str());
    
    std::ifstream urdf_file(urdf_name);
    if (urdf_file.is_open()) {
      std::stringstream urdf_stream;
      urdf_stream << urdf_file.rdbuf();
      urdf_string = urdf_stream.str();
      urdf_file.close();
      
      model_ok = my_urdf_model.initString(urdf_string);
      if (model_ok) {
        RCLCPP_INFO(node_->get_logger(), "成功从文件路径加载URDF");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "从文件路径获取到URDF内容，但解析失败");
        exit(EXIT_FAILURE);
      }
    } else {
      RCLCPP_ERROR(node_->get_logger(), "无法打开URDF文件: %s", urdf_name.c_str());
      exit(EXIT_FAILURE);
    }
  }






  RCLCPP_DEBUG(node_->get_logger(), "URDF successfully parsed");
  kdl_parser::treeFromUrdfModel(my_urdf_model, my_kdl_tree);
  RCLCPP_DEBUG(node_->get_logger(), "Robot tree is ready");

  // 在ROS2中，我们不需要直接使用RobotStatePublisher类
  // 而是通过发布JointState消息和TransformStamped消息来实现可视化
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);









  // // 2. 配置针对 RViz2 的发布者
  // // 必须使用 TransientLocal，否则 RViz2 后打开时收不到模型
  // auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
  // rclcpp::QoS(1).transient_local()
  // 话题名必须与你配置文件中的 "rviz_urdf_robot_description" 一致
  robot_desc_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "robot_description", rclcpp::QoS(1).transient_local());

  // 3. 发布数据
  auto msg = std::make_unique<std_msgs::msg::String>();

  std::string urdf_xml_string = readTextFileToString("/home/lmc/Desktop/mnt/robot_tools/magic_vis_ros2/urdf/p5_humanoid.urdf");

  msg->data = urdf_xml_string; // 这里的字符串就是传给 KDL 的同一个 XML 文本
  robot_desc_pub_->publish(std::move(msg));

  RCLCPP_INFO(node_->get_logger(), "Published robot model to robot_description");
}

void
UrdfVisualizer::poseStateCallback(const lcm_to_ros2::msg::StateEstimatorLcmt::SharedPtr msg)
{
  // Converting from joint messages to robot state
  geometry_msgs::msg::TransformStamped W_X_B_message;
  W_X_B_message.header.stamp    = node_->now();
  W_X_B_message.header.frame_id = rviz_fixed_frame_;
  W_X_B_message.child_frame_id  = tf_prefix_ + "/" + base_joint_in_urdf_;

  W_X_B_message.transform.translation.x = msg->p[0];
  W_X_B_message.transform.translation.y = msg->p[1];
  W_X_B_message.transform.translation.z = msg->p[2];

  W_X_B_message.transform.rotation.w = msg->quat[0];
  W_X_B_message.transform.rotation.x = msg->quat[1];
  W_X_B_message.transform.rotation.y = msg->quat[2];
  W_X_B_message.transform.rotation.z = msg->quat[3];

  // Assign all joint values
  for (int i=0; i<joint_values_.size(); ++i)
    joint_positions_[joint_names_in_urdf_.at(i)] = joint_values_.at(i);

  tf_broadcaster_->sendTransform(W_X_B_message);
  // Publish joint states
  sensor_msgs::msg::JointState joint_state_msg;
  joint_state_msg.header.stamp = node_->now();
  joint_state_msg.name = joint_names_in_urdf_;
  joint_state_msg.position = joint_values_;
  joint_state_publisher_->publish(joint_state_msg);
}

void
UrdfVisualizer::legStateCallback(const lcm_to_ros2::msg::LegControlDataLcmt::SharedPtr msg)
{
  for (int i=0; i<leg_n_; i++)
    joint_values_[i + leg_id_] = msg->q[i];
}

void
UrdfVisualizer::armStateCallback(const lcm_to_ros2::msg::ArmControlDataLcmt::SharedPtr msg)
{
  for (int i=0; i<arm_n_; i++)
    joint_values_[i + arm_id_] = msg->joint_cur_pos[i];
}

void
UrdfVisualizer::waistStateCallback(const lcm_to_ros2::msg::WaistControlDataLcmt::SharedPtr msg)
{
  for (int i=0; i<waist_n_; i++)
    joint_values_[i + waist_id_] = msg->q[i];
}

void
UrdfVisualizer::headStateCallback(const lcm_to_ros2::msg::HeadControlDataLcmt::SharedPtr msg)
{
  for (int i=0; i<head_n_; i++)
    joint_values_[i + head_id_] = msg->q[i];
}

}

// ... 其他函数保持不变 ...