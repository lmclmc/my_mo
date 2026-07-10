#ifndef DATASTREAM_ROS2_TOPIC_H
#define DATASTREAM_ROS2_TOPIC_H

#include <QtPlugin>
#include <QTimer>
#include <PlotJuggler/datastreamer_base.h>
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm_coretypes.h>

#include "dialog_select_ros_topics.h"
#include "ros_parsers/ros2_parser.h"
#include "rclcpp/generic_subscription.hpp"
#include "lcmgen.h"
#include "tokenize.h"

class DataStreamLCM : public PJ::DataStreamer
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.ROS2DataStreamer")
  Q_INTERFACES(PJ::DataStreamer)

typedef struct TopicLcmItem_ {
    int64_t hash;
    std::vector<lcmgen_t*> lcmgen_table;
    std::string lcmFilePath;
} TopicLcmItem_;

public:
  DataStreamLCM();

  bool start(QStringList* selected_datasources) override;

  void shutdown() override;

  bool isRunning() const override;

  ~DataStreamLCM() override;

  const char* name() const override
  {
    return "LCM Topic Subscriber";
  }

  bool xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const override;

  bool xmlLoadState(const QDomElement& parent_element) override;

  const std::vector<QAction*>& availableActions() override;

private:
  void handleMessage(const lcm::ReceiveBuffer*, const std::string&);
  int decode_type(const lcm::ReceiveBuffer*, int, const std::string&, const std::string&, int deep= 0) ;

private:
  std::shared_ptr<rclcpp::Context> _context;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> _executor;
  std::shared_ptr<rclcpp::Node> _node;

  std::map<std::string, lcm_struct_t*> lcm_type_table;
  std::shared_ptr<lcm::LCM> lcm_;
  std::unordered_map<std::string, TopicLcmItem_> topic_map;

  PJ::CompositeParser _parser;

  bool _running;
  bool _first_warning;

  std::thread _spinner;

  PJ::RosParserConfig _config;

  rclcpp::Clock _clock;

  rcl_time_point_value_t _start_time;

  void saveDefaultSettings();

  void loadDefaultSettings();

  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> _subscriptions;

  void subscribeToTopic(const std::string& topic_name, const std::string& topic_type);
  void waitOneSecond();
};

#endif
