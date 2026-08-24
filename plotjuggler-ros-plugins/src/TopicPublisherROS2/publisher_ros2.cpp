#include "publisher_ros2.h"

#include <QDebug>
#include <QDialog>
#include <QFormLayout>
#include <QCheckBox>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include <QScrollArea>
#include <QPushButton>
#include <QSettings>
#include <QRadioButton>
#include <unordered_map>
#include <QMessageBox>
#include <tf2_ros/qos.hpp>
#include <rosbag2_cpp/types.hpp>
#include <rmw/rmw.h>
#include <rmw/types.h>
#include "publisher_select_dialog.h"
#include <fstream>

TopicPublisherROS2::TopicPublisherROS2() : _node(nullptr), _enabled(false)
{
  _context = std::make_shared<rclcpp::Context>();
  _context->init(0, nullptr);

  auto exec_args = rclcpp::ExecutorOptions();
  exec_args.context = _context;
  _executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(exec_args, 2);

  _select_topics_to_publish = new QAction(QString("Select topics to be published"));
  connect(_select_topics_to_publish, &QAction::triggered, this, &TopicPublisherROS2::filterDialog);

  _available_actions.push_back(_select_topics_to_publish);
}

TopicPublisherROS2::~TopicPublisherROS2()
{
  _enabled = false;
}

const std::vector<QAction*>& TopicPublisherROS2::availableActions()
{
  return _available_actions;
}

void TopicPublisherROS2::updatePublishers()
{
  if (!_node)
  {
    return;
  }
  for (const auto& info : _topics_info)
  {
    auto to_publish = _topics_to_publish.find(info.topic_name);
    if (to_publish == _topics_to_publish.end() || to_publish->second == false)
    {
      continue;  // no publish
    }

    // auto publisher_it = _publishers.find(info.topic_name);
    // if (publisher_it == _publishers.end())
    // {
    //   _publishers.insert({ info.topic_name, GenericPublisher::create(*_node, info.topic_name, info.type) });
    // }
  }

  // remove already created publishers if not needed anymore
  for (auto it = _publishers.begin(); it != _publishers.end(); /* no increment */)
  {
    auto to_publish = _topics_to_publish.find(it->first);
    if (to_publish == _topics_to_publish.end() || to_publish->second == false)
    {
      it = _publishers.erase(it);
    }
    else
    {
      it++;
    }
  }
}

// 用于存储脚本执行结果的结构体
struct ScriptResult {
    std::string output;   // 脚本的打印内容
    int returnCode;       // 脚本的返回值
};

/**
 * 执行指定脚本并获取其输出和返回值
 * @param scriptPath 脚本路径
 * @param mode 模式参数 (play/record/playback/watch)
 * @param value 值参数 (0/1)
 * @return 包含脚本输出和返回值的结构体
 */
static ScriptResult executeScript(const std::string& scriptPath, const std::string& mode, int value) {
  ScriptResult result;
  result.returnCode = -1;
  
  // 构建完整的命令行
  std::string command = scriptPath + " " + mode + " " + std::to_string(value);
  
  // 使用popen执行命令并捕获输出
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
      result.output = "Error: Failed to open pipe";
      return result;
  }
  
  // 读取脚本输出
  std::array<char, 128> buffer;
  std::string output;
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
      output += buffer.data();
  }
  result.output = output;
  
  // 获取脚本返回值
  int status = pclose(pipe);
  if (WIFEXITED(status)) {
      result.returnCode = WEXITSTATUS(status);
  } else {
      result.output += "\nError: Script did not exit normally";
  }
  
  return result;
}

void TopicPublisherROS2::setEnabled(bool to_enable)
{
  if (!_node && to_enable)
  {
    auto node_opts = rclcpp::NodeOptions();
    node_opts.context(_context);
    _node = std::make_shared<rclcpp::Node>("plotjuggler", node_opts);
    sensor_joint_publisher_ = _node->create_publisher<sensor_msgs::msg::JointState>(
      "csv_pos",
      10);
    quat_publisher_ = _node->create_publisher<geometry_msgs::msg::Quaternion>(
      "csv_quat",
      10);

    state_estimator_publisher_ = _node->create_publisher<lcm_to_ros2::msg::StateEstimatorLcmt>(
      "/robot_lcm/state_estimator",
      10);
    leg_data_publisher_ = _node->create_publisher<lcm_to_ros2::msg::LegControlDataLcmt>(
      "/robot_lcm/leg_control_data",
      10);
    waist_data_publisher_ = _node->create_publisher<lcm_to_ros2::msg::WaistControlDataLcmt>(
      "/robot_lcm/waist_control_data",
      10);
    head_data_publisher_ = _node->create_publisher<lcm_to_ros2::msg::HeadControlDataLcmt>(
      "/robot_lcm/head_control_data",
      10);
    arm_data_publisher_ = _node->create_publisher<lcm_to_ros2::msg::ArmControlDataLcmt>(
      "/robot_lcm/arm_control_data",
      10);

    _executor->add_node(_node);
    std::thread([this]() {
      while (rclcpp::ok()) {
        // 非阻塞spin（每次处理1ms的回调，避免卡死）
        _executor->spin_once(std::chrono::milliseconds(1));
      }
    }).detach();
  }

  // ScriptResult result = executeScript("/opt/ros/motion_studio_mode_switch.sh", "playback", to_enable);
  // if (result.returnCode != 0) {
  //   _enabled = false;
  //   emit closed();
  //   std::cout << "Error: Failed to switch to playback mode. Return msg: " << result.output << std::endl;
  //   QMessageBox::critical(nullptr, "错误", result.output.c_str());
  //   return;
  // }

  _enabled = (to_enable && _node);

  if (_enabled)
  {
    auto metadata_it = _datamap->user_defined.find("plotjuggler::rosbag2_cpp::topics_metadata");
    if (metadata_it == _datamap->user_defined.end())
    {
      return;
    }
    // I stored it in a one point timeseries... shoot me
    const auto any_metadata = metadata_it->second[0].y;
    _topics_info = std::any_cast<std::vector<TopicInfo>>(any_metadata);

    // select all the topics by default
    for (const auto& info : _topics_info)
    {
      _topics_to_publish.insert({ info.topic_name, true });
    }

    updatePublishers();

    if (!_tf_broadcaster)
    {
      _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*_node);
    }
    if (!_tf_static_broadcaster)
    {
      _tf_static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*_node);
    }

    _previous_play_index = std::numeric_limits<int>::max();
  }
  else
  {
    _node.reset();
    _publishers.clear();
    _tf_broadcaster.reset();
    _tf_static_broadcaster.reset();
  }
}

void TopicPublisherROS2::filterDialog()
{
  auto metadata_it = _datamap->user_defined.find("plotjuggler::rosbag2_cpp::topics_metadata");
  if (metadata_it != _datamap->user_defined.end())
  {
    // I stored it in a one point timeseries... shoot me
    const auto any_metadata = metadata_it->second[0].y;
    _topics_info = std::any_cast<std::vector<TopicInfo>>(any_metadata);
  }

  PublisherSelectDialog* dialog = new PublisherSelectDialog();
  dialog->setAttribute(Qt::WA_DeleteOnClose);
  dialog->ui()->radioButtonClock->setHidden(true);
  dialog->ui()->radioButtonHeaderStamp->setHidden(true);

  // Sort topics alphabetically by topic name
  std::vector<TopicInfo> sorted_topics = _topics_info;
  std::sort(sorted_topics.begin(), sorted_topics.end(),
            [](const TopicInfo& a, const TopicInfo& b) { return a.topic_name < b.topic_name; });

  std::map<std::string, QCheckBox*> checkbox;

  for (const TopicInfo& info : _topics_info)
  {
    const std::string topic_name = info.topic_name;
    auto cb = new QCheckBox(dialog);
    auto filter_it = _topics_to_publish.find(topic_name);
    if (filter_it == _topics_to_publish.end())
    {
      cb->setChecked(true);
    }
    else
    {
      cb->setChecked(filter_it->second);
    }
    cb->setFocusPolicy(Qt::NoFocus);
    dialog->ui()->formLayout->addRow(new QLabel(QString::fromStdString(topic_name)), cb);
    checkbox.insert(std::make_pair(topic_name, cb));
    connect(dialog->ui()->pushButtonSelect, &QPushButton::pressed, [cb]() { cb->setChecked(true); });
    connect(dialog->ui()->pushButtonDeselect, &QPushButton::pressed, [cb]() { cb->setChecked(false); });
  }

  dialog->exec();

  if (dialog->result() == QDialog::Accepted)
  {
    _topics_to_publish.clear();
    for (const auto& it : checkbox)
    {
      _topics_to_publish.insert({ it.first, it.second->isChecked() });
    }

    updatePublishers();
  }
}

constexpr long NSEC_PER_SEC = 1000000000;

rcutils_time_point_value_t Convert(const builtin_interfaces::msg::Time& stamp)
{
  return stamp.nanosec + NSEC_PER_SEC * stamp.sec;
}

builtin_interfaces::msg::Time Convert(const rcutils_time_point_value_t& time_stamp)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(time_stamp / NSEC_PER_SEC);
  stamp.nanosec = static_cast<int32_t>(time_stamp - (NSEC_PER_SEC * stamp.sec));
  return stamp;
}

void TopicPublisherROS2::broadcastTF(double current_time)
{
  using StringPair = std::pair<std::string, std::string>;

  std::map<StringPair, geometry_msgs::msg::TransformStamped> transforms;
  std::map<StringPair, geometry_msgs::msg::TransformStamped> static_transforms;

  for (const auto& data_it : _datamap->user_defined)
  {
    const std::string& topic_name = data_it.first;
    const PJ::PlotDataAny& plot_any = data_it.second;

    if (topic_name != "/tf_static" && topic_name != "/tf")
    {
      continue;
    }

    const PJ::PlotDataAny* tf_data = &plot_any;
    int last_index = tf_data->getIndexFromX(current_time);
    if (last_index < 0)
    {
      continue;
    }

    auto transforms_ptr = (topic_name == "/tf_static") ? &static_transforms : &transforms;

    std::vector<uint8_t> raw_buffer;
    // 2 seconds in the past (to be configurable in the future)
    int initial_index = tf_data->getIndexFromX(current_time - 2.0);

    if (_previous_play_index < last_index && _previous_play_index > initial_index)
    {
      initial_index = _previous_play_index;
    }

    for (size_t index = std::max(0, initial_index); index <= last_index; index++)
    {
      const std::any& any_value = tf_data->at(index).y;

      const bool isRosbagMessage = (any_value.type() == typeid(MessageRefPtr));

      if (!isRosbagMessage)
      {
        continue;
      }

      const auto& msg_instance = std::any_cast<MessageRefPtr>(any_value);

      auto tf_type_support = rosidl_typesupport_cpp::get_message_type_support_handle<tf2_msgs::msg::TFMessage>();
      tf2_msgs::msg::TFMessage tf_msg;
      if (RMW_RET_OK != rmw_deserialize(msg_instance->serialized_data.get(), tf_type_support, &tf_msg))
      {
        throw std::runtime_error("failed to deserialize TF message");
      }

      for (const auto& stamped_transform : tf_msg.transforms)
      {
        const auto& parent_id = stamped_transform.header.frame_id;
        const auto& child_id = stamped_transform.child_frame_id;
        StringPair trans_id = std::make_pair(parent_id, child_id);
        auto it = transforms_ptr->find(trans_id);
        if (it == transforms_ptr->end())
        {
          transforms_ptr->insert({ trans_id, stamped_transform });
        }
        else if (Convert(it->second.header.stamp) <= Convert(stamped_transform.header.stamp))
        {
          it->second = stamped_transform;
        }
      }
    }

    // ready to broadacast
    std::vector<geometry_msgs::msg::TransformStamped> transforms_vector;
    transforms_vector.reserve(transforms_ptr->size());

    rcutils_time_point_value_t time_stamp;
    int error = rcutils_system_time_now(&time_stamp);

    if (error != RCUTILS_RET_OK)
    {
      qDebug() << "Error getting current time. Error:" << rcutils_get_error_string().str;
    }

    for (auto& trans : *transforms_ptr)
    {
      trans.second.header.stamp = Convert(time_stamp);
      transforms_vector.emplace_back(std::move(trans.second));
    }
    if (transforms_ptr == &transforms)
    {
      _tf_broadcaster->sendTransform(transforms_vector);
    }
    else
    {
      _tf_static_broadcaster->sendTransform(transforms_vector);
    }
  }
}

void TopicPublisherROS2::updateState(double current_time) {
  if (!_enabled || !_node) {
    return;
  }
  publish_data(current_time);
}

void TopicPublisherROS2::publish_state_estimator(double current_time) {
  auto state_estimator_msg = std::make_shared<lcm_to_ros2::msg::StateEstimatorLcmt>();
  for (int i = 0; i < 4; i++) {
    std::string str_quat = std::string("/robot_lcm/state_estimator/quat[") + 
                          std::to_string(i) + std::string("]");
    auto data_it = _datamap->numeric.find(str_quat);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      state_estimator_msg->quat[i] = data_it->second.at(index).y;
    } else {
      state_estimator_msg->quat[i] = 0;
      return;
    }
  }
  state_estimator_publisher_->publish(*state_estimator_msg);
}

void TopicPublisherROS2::publish_leg_data(double current_time) {
  auto leg_data_ros2_msg = std::make_shared<lcm_to_ros2::msg::LegControlDataLcmt>();
  for (int i = 0; i < 12; i++) {
    std::string str_q = std::string("/robot_lcm/leg_control_data/q[") + 
                        std::to_string(i) + std::string("]");
    auto data_it = _datamap->numeric.find(str_q);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      leg_data_ros2_msg->q[i] = data_it->second.at(index).y;
    } else {
      leg_data_ros2_msg->q[i] = 0;
      return;
    }
  }

  leg_data_publisher_->publish(*leg_data_ros2_msg);
}

void TopicPublisherROS2::publish_arm_data(double current_time) {
  auto arm_data_ros2_msg = std::make_shared<lcm_to_ros2::msg::ArmControlDataLcmt>();
  for (int i = 0; i < 14; i++) {
    std::string str_q = std::string("/robot_lcm/arm_control_data/joint_cur_pos[") + 
                        std::to_string(i) + std::string("]");
    auto data_it = _datamap->numeric.find(str_q);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      arm_data_ros2_msg->joint_cur_pos[i] = data_it->second.at(index).y;
    } else {
      arm_data_ros2_msg->joint_cur_pos[i] = 0;
      return;
    }
  }
  arm_data_publisher_->publish(*arm_data_ros2_msg);
}

void TopicPublisherROS2::publish_waist_data(double current_time) {
  auto waist_data_ros2_msg = std::make_shared<lcm_to_ros2::msg::WaistControlDataLcmt>();
  for (int i = 0; i < 3; i++) {
    std::string str_q = std::string("/robot_lcm/waist_control_data/q[") + 
                        std::to_string(i) + std::string("]");
    auto data_it = _datamap->numeric.find(str_q);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      waist_data_ros2_msg->q[i] = data_it->second.at(index).y;
    } else {
      waist_data_ros2_msg->q[i] = 0;
      return;
    }
  }
  waist_data_publisher_->publish(*waist_data_ros2_msg);
}

void TopicPublisherROS2::publish_head_data(double current_time) {
  auto head_data_ros2_msg = std::make_shared<lcm_to_ros2::msg::HeadControlDataLcmt>();
  for (int i = 0; i < 2; i++) {
    std::string str_q = std::string("/robot_lcm/head_control_data/q[") + 
                        std::to_string(i) + std::string("]");
    auto data_it = _datamap->numeric.find(str_q);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      head_data_ros2_msg->q[i] = data_it->second.at(index).y;
    } else {
      head_data_ros2_msg->q[i] = 0;
      return;
    }
  }
  head_data_publisher_->publish(*head_data_ros2_msg);
}

void TopicPublisherROS2::publish_lcm_leg_data(double current_time) {
  auto leg_data_ros2_msg = std::make_shared<lcm_to_ros2::msg::LegControlDataLcmt>();
  for (int i = 0; i < 12; i++) {
    std::string str_q = std::string("leg_control_data/q/") + std::to_string(i);
    auto data_it = _datamap->numeric.find(str_q);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      leg_data_ros2_msg->q[i] = data_it->second.at(index).y;
    } else {
      leg_data_ros2_msg->q[i] = 0;
      return;
    }
  }

  leg_data_publisher_->publish(*leg_data_ros2_msg);
}

void TopicPublisherROS2::publish_lcm_waist_data(double current_time) {
  auto waist_data_ros2_msg = std::make_shared<lcm_to_ros2::msg::WaistControlDataLcmt>();
  for (int i = 0; i < 3; i++) {
    std::string str_q = std::string("waist_control_data/q/") +  std::to_string(i);
    auto data_it = _datamap->numeric.find(str_q);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      waist_data_ros2_msg->q[i] = data_it->second.at(index).y;
    } else {
      waist_data_ros2_msg->q[i] = 0;
      return;
    }
  }
  waist_data_publisher_->publish(*waist_data_ros2_msg);
}

void TopicPublisherROS2::publish_lcm_head_data(double current_time) {
  auto head_data_ros2_msg = std::make_shared<lcm_to_ros2::msg::HeadControlDataLcmt>();
  for (int i = 0; i < 2; i++) {
    std::string str_q = std::string("head_control_data/q/") + std::to_string(i);
    auto data_it = _datamap->numeric.find(str_q);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      head_data_ros2_msg->q[i] = data_it->second.at(index).y;
    } else {
      head_data_ros2_msg->q[i] = 0;
      return;
    }
  }
  head_data_publisher_->publish(*head_data_ros2_msg);
}

void TopicPublisherROS2::publish_lcm_arm_data(double current_time) {
  auto arm_data_ros2_msg = std::make_shared<lcm_to_ros2::msg::ArmControlDataLcmt>();
  for (int i = 0; i < 14; i++) {
    std::string str_q = std::string("arm_control_data/joint_curPos/") + std::to_string(i);
    auto data_it = _datamap->numeric.find(str_q);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      arm_data_ros2_msg->joint_cur_pos[i] = data_it->second.at(index).y;
    } else {
      arm_data_ros2_msg->joint_cur_pos[i] = 0;
      return;
    }
  }
  arm_data_publisher_->publish(*arm_data_ros2_msg);
}

void TopicPublisherROS2::publish_lcm_state_estimator(double current_time) {
  auto state_estimator_msg = std::make_shared<lcm_to_ros2::msg::StateEstimatorLcmt>();
  for (int i = 0; i < 4; i++) {
    std::string str_quat = std::string("state_estimator/quat/") + std::to_string(i);
    auto data_it = _datamap->numeric.find(str_quat);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      state_estimator_msg->quat[i] = data_it->second.at(index).y;
    } else {
      state_estimator_msg->quat[i] = 0;
      return;
    }
  }
  state_estimator_publisher_->publish(*state_estimator_msg);
}

void TopicPublisherROS2::publish_sensor_joint(double current_time) {
  auto sensor_joint_msg = std::make_shared<sensor_msgs::msg::JointState>();
  sensor_joint_msg->name.resize(30);
  sensor_joint_msg->position.resize(30);
  for (int i = 0; i < 30; i++) {
    std::string str_csv_pos = std::string("csv_pos/") + std::to_string(i) + std::string("/position");
    auto data_it = _datamap->numeric.find(str_csv_pos);
    if (data_it != _datamap->numeric.end()) {
      int index = data_it->second.getIndexFromX(current_time);
      sensor_joint_msg->position[i] = data_it->second.at(index).y;
    } else {
      sensor_joint_msg->position[i] = 0;
      return;
    }

    sensor_joint_msg->name[i] = std::to_string(i);
  }
  sensor_joint_publisher_->publish(*sensor_joint_msg);
}

void TopicPublisherROS2::publish_csv_quaternion(double current_time) {
  auto quat_msg = std::make_shared<geometry_msgs::msg::Quaternion>();
{
  auto data_it = _datamap->numeric.find("csv_quat/x");
  if (data_it != _datamap->numeric.end()) {
    int index = data_it->second.getIndexFromX(current_time);
    quat_msg->x = data_it->second.at(index).y;
  } else {
    quat_msg->x = 0.0;
    return;
  }
}
{
  auto data_it = _datamap->numeric.find("csv_quat/y");
  if (data_it != _datamap->numeric.end()) {
    int index = data_it->second.getIndexFromX(current_time);
    quat_msg->y = data_it->second.at(index).y;
  } else {
    quat_msg->y = 0.0;
    return;
  }
}
{
  auto data_it = _datamap->numeric.find("csv_quat/z");
  if (data_it != _datamap->numeric.end()) {
    int index = data_it->second.getIndexFromX(current_time);
    quat_msg->z = data_it->second.at(index).y;
  } else {
    quat_msg->z = 0.0;
    return;
  }
}
{
  auto data_it = _datamap->numeric.find("csv_quat/w");
  if (data_it != _datamap->numeric.end()) {
    int index = data_it->second.getIndexFromX(current_time);
    quat_msg->w = data_it->second.at(index).y;
  } else {
    quat_msg->w = 0.0;
    return;
  }
}
  quat_publisher_->publish(*quat_msg);
}

void TopicPublisherROS2::publish_data(double current_time) {
  publish_csv_quaternion(current_time);
  publish_sensor_joint(current_time);
  publish_state_estimator(current_time);
  publish_leg_data(current_time);
  publish_waist_data(current_time);
  publish_head_data(current_time);
  publish_arm_data(current_time);

  publish_lcm_leg_data(current_time);
  publish_lcm_waist_data(current_time);
  publish_lcm_head_data(current_time);
  publish_lcm_arm_data(current_time);
  publish_lcm_state_estimator(current_time);
}

void TopicPublisherROS2::play(double current_time)
{
  if (!_enabled || !_node)
  {
    return;
  }
  publish_data(current_time);
  return;
  auto data_it = _datamap->user_defined.find("plotjuggler::rosbag2_cpp::consecutive_messages");
  if (data_it == _datamap->user_defined.end())
  {
    return;
  }
  const PJ::PlotDataAny& continuous_msgs = data_it->second;
  int current_index = continuous_msgs.getIndexFromX(current_time);

  if (_previous_play_index > current_index)
  {
    _previous_play_index = current_index;
    updateState(current_time);
    return;
  }
  else
  {
    const PJ::PlotDataAny& consecutive_msg = data_it->second;
    publish_data(current_index);
    for (int index = _previous_play_index + 1; index <= current_index; index++)
    {
     // publish_data(index);
      // const auto& any_value = consecutive_msg.at(index).y;
      // if (any_value.type() == typeid(MessageRefPtr))
      // {
      //   const auto& msg_instance = std::any_cast<MessageRefPtr>(any_value);

      //   auto publisher_it = _publishers.find(msg_instance->topic_name);
      //   if (publisher_it == _publishers.end())
      //   {
      //     continue;
      //   }

      //   publisher_it->second->publish(msg_instance->serialized_data);
      // }
    }
    _previous_play_index = current_index;
  }
}
