#ifndef MAGIC_VIS_URDF_VISUALIZER_H_
#define MAGIC_VIS_URDF_VISUALIZER_H_

#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

// 添加必要的头文件
#include <sensor_msgs/msg/joint_state.hpp>

#include <lcm_to_ros2/msg/state_estimator_lcmt.hpp>
#include <lcm_to_ros2/msg/leg_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/arm_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/waist_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/head_control_data_lcmt.hpp>
#include <std_msgs/msg/string.hpp>

namespace magic {

struct Joint {
  double min;           // 关节最小值
  double max;           // 关节最大值
  double zero;          // 关节零位值
  double position;      // 当前位置
  double velocity;      // 当前速度
  double effort;        // 当前力矩
  bool continuous;      // 是否为连续关节
  bool mimic;           // 是否为模拟关节
  std::string parent;   // 模拟关节的父关节
  double factor;        // 模拟关节的比例因子
  double offset;        // 模拟关节的偏移量

  Joint() : min(-M_PI), max(M_PI), zero(0.0), position(0.0), velocity(0.0), effort(0.0),
            continuous(false), mimic(false), factor(1.0), offset(0.0) {}
};

/**
 * @brief Publishes RVIZ transforms used to visualize a robot's URDF.
 */
class UrdfVisualizer : public rclcpp::Node {
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
  
  // 添加get_node()函数，用于外部获取node指针
  std::shared_ptr<rclcpp::Node> get_node() const { return node_; }

private:
  void poseStateCallback(const lcm_to_ros2::msg::StateEstimatorLcmt::SharedPtr msg);
  void legStateCallback(const lcm_to_ros2::msg::LegControlDataLcmt::SharedPtr msg);
  void armStateCallback(const lcm_to_ros2::msg::ArmControlDataLcmt::SharedPtr msg);
  void waistStateCallback(const lcm_to_ros2::msg::WaistControlDataLcmt::SharedPtr msg);
  void headStateCallback(const lcm_to_ros2::msg::HeadControlDataLcmt::SharedPtr msg);

  // 添加node_成员变量
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_desc_pub_;
  
  rclcpp::Subscription<lcm_to_ros2::msg::StateEstimatorLcmt>::SharedPtr pose_sub_des_;
  rclcpp::Subscription<lcm_to_ros2::msg::LegControlDataLcmt>::SharedPtr leg_sub_des_;
  rclcpp::Subscription<lcm_to_ros2::msg::ArmControlDataLcmt>::SharedPtr arm_sub_des_;
  rclcpp::Subscription<lcm_to_ros2::msg::WaistControlDataLcmt>::SharedPtr waist_sub_des_;
  rclcpp::Subscription<lcm_to_ros2::msg::HeadControlDataLcmt>::SharedPtr head_sub_des_;
  
  // 添加JointState发布器
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::string> joint_names_in_urdf_;
  std::string base_joint_in_urdf_;
  std::string rviz_fixed_frame_;
  std::string tf_prefix_;

  // 关节数据
  std::map<std::string, Joint> free_joints_;
  std::vector<std::string> joint_list_;
  std::map<std::string, Joint> dependent_joints_;

  std::map<std::string, double> joint_positions_;
  std::vector<double> joint_values_;
  int leg_id_, leg_n_, arm_id_, arm_n_, waist_id_, waist_n_, head_id_, head_n_;
};

} /* namespace magic */

#endif /* MAGIC_VIS_URDF_VISUALIZER_H_ */