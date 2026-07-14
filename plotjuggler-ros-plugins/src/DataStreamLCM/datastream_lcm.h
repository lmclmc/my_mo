#ifndef DATASTREAM_ROS2_TOPIC_H
#define DATASTREAM_ROS2_TOPIC_H

#include <QtPlugin>
#include <QTimer>
#include <thread>
#include <PlotJuggler/datastreamer_base.h>
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm_coretypes.h>

#include "../parser_configuration.h"
#include "lcmgen.h"
#include "tokenize.h"
#include "logsink.hpp"

class DataStreamLCM : public PJ::DataStreamer
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID "facontidavide.PlotJuggler3.DataStreamer")
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
  std::map<std::string, lcm_struct_t*> lcm_type_table;
  std::shared_ptr<lcm::LCM> lcm_;
  std::unordered_map<std::string, TopicLcmItem_> topic_map;
  std::shared_ptr<rotating_file_sink> lcm_log;
  lcm::Subscription* lcm_subscription;

  bool _running;
  bool _first_warning;

  std::thread lcm_runner_;

  PJ::RosParserConfig _config;

  QString log_save_path;
  int log_max_size;
  int log_max_files_num;

  void saveDefaultSettings();

  void loadDefaultSettings();
};

#endif