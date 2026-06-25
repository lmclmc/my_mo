#ifndef STATE_PUBLISHER_ROS2TOPIC_H
#define STATE_PUBLISHER_ROS2TOPIC_H

#include <QObject>
#include <QtPlugin>
#include <map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <PlotJuggler/statepublisher_base.h>
#include "ros_parsers/ros2_parser.h"
#include "generic_publisher.h"

#include <lcm_to_ros2/msg/state_estimator_lcmt.hpp>
#include <lcm_to_ros2/msg/leg_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/arm_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/waist_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/head_control_data_lcmt.hpp>
#include <lcm_to_ros2/msg/lcm_response_lcmt.hpp>
#include <lcm_to_ros2/msg/wbc_test_data_t.hpp>
#include <lcm_to_ros2/msg/trajectory_joint_lcmt.hpp>

#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using MessageRefPtr = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;

class TopicPublisherROS2 : public PJ::StatePublisher
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.StatePublisher")
  Q_INTERFACES(PJ::StatePublisher)

public:
  TopicPublisherROS2();

  virtual ~TopicPublisherROS2() override;

  void updateState(double current_time) override;

  const char* name() const override
  {
    return "ROS2 Topic Re-Publisher";
  }

  bool enabled() const override
  {
    return _enabled;
  }

  const std::vector<QAction*>& availableActions() override;

  void play(double interval) override;

public slots:

  void setEnabled(bool enabled) override;

  void filterDialog();

private:
  std::shared_ptr<rclcpp::Context> _context;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;
  std::shared_ptr<rclcpp::Node> _node;

  bool _enabled;

  std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> _tf_static_broadcaster;

  std::unordered_map<std::string, std::shared_ptr<GenericPublisher>> _publishers;

  QAction* _select_topics_to_publish;

  std::unordered_map<std::string, bool> _topics_to_publish;

  double previous_time;

  int _previous_play_index;

  std::vector<TopicInfo> _topics_info;

  std::vector<QAction*> _available_actions;

  void broadcastTF(double current_time);

  void updatePublishers();

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sensor_joint_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr quat_publisher_;

  rclcpp::Publisher<lcm_to_ros2::msg::StateEstimatorLcmt>::SharedPtr state_estimator_publisher_;
  rclcpp::Publisher<lcm_to_ros2::msg::LegControlDataLcmt>::SharedPtr leg_data_publisher_;
  rclcpp::Publisher<lcm_to_ros2::msg::WaistControlDataLcmt>::SharedPtr waist_data_publisher_;
  rclcpp::Publisher<lcm_to_ros2::msg::HeadControlDataLcmt>::SharedPtr head_data_publisher_;
  rclcpp::Publisher<lcm_to_ros2::msg::ArmControlDataLcmt>::SharedPtr arm_data_publisher_;
  
  void publish_data(double current_time);
  void publish_state_estimator(double current_time);
  void publish_leg_data(double current_time);
  void publish_arm_data(double current_time);
  void publish_waist_data(double current_time);
  void publish_head_data(double current_time);
  void publish_csv_quaternion(double current_time);
  void publish_sensor_joint(double current_time);
};

#endif  // STATE_PUBLISHER_ROS2TOPIC_H
