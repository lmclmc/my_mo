#include "dataload_lcm.h"
#include "dialog_select_ros_topics.h"
#include "LcmTopicDialog.h"
#include "ros_parsers/ros2_parser.h"
#include "lcmgen.h"
#include "tokenize.h"

#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include <QPushButton>
#include <QDebug>
#include <QApplication>
#include <QProgressDialog>
#include <QFileInfo>
#include <QDir>
#include <QProcess>
#include <QSettings>
#include <QElapsedTimer>
#include <set>
#include <QDebug>
#include <rosidl_typesupport_introspection_cpp/identifier.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_cpp/identifier.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/types/introspection_message.hpp>
#include <unordered_map>
#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcm/lcm_coretypes.h>

DataLoadLCM::DataLoadLCM()
{
  _extensions.push_back("lcmlog");
  loadDefaultSettings();
}

#include <stdio.h>

#if defined(_WIN32)
#include <sys/stat.h>
long long get_file_size(FILE* fp)
{
    if (!fp) return -1;
    int fd = _fileno(fp);
    struct _stat64 st;
    if (_fstat64(fd, &st) != 0) return -1;
    return st.st_size;
}
#elif defined(__unix__) || defined(__linux__) || defined(__APPLE__)
#include <sys/stat.h>
#include <unistd.h>
long long get_file_size(FILE* fp)
{
    if (!fp) return -1;
    int fd = fileno(fp);
    struct stat st;
    if (fstat(fd, &st) != 0) return -1;
    return st.st_size;
}
#else
// 纯标准C fallback
long long get_file_size(FILE* fp)
{
    if (!fp) return -1;
    long long cur = ftell(fp);
    if (cur == -1) return -1;
    if (fseek(fp, 0, SEEK_END)) return -1;
    long long sz = ftell(fp);
    fseek(fp, cur, SEEK_SET);
    return sz;
}
#endif

const std::vector<const char*>& DataLoadLCM::compatibleFileExtensions() const
{
  return _extensions;
}

bool DataLoadLCM::readDataFromFile(PJ::FileLoadInfo* info, PJ::PlotDataMapRef& plot_map)
{
  auto metadata_io = std::make_unique<rosbag2_storage::MetadataIo>();
  LcmTopicDialog* dialog = new LcmTopicDialog();

  if (dialog->exec() != static_cast<int>(QDialog::Accepted))
  {
    std::cout << "Dialog canceled or closed." << std::endl;
    delete dialog;
    return false;
  }

  typedef struct TopicLcmItem {
    int64_t hash;
    lcmgen_t* lcmgen = nullptr;
    std::string lcmFilePath;
  } TopicLcmItem;
  
  std::unordered_map<std::string, TopicLcmItem> topic_map;
  auto _config = dialog->getConfig();

  for (const auto& item : _config) {
    lcmgen_t* lcmgen = lcmgen_create();
    tokenize_t* t = tokenize_create(item.lcmFilePath.toStdString().c_str());
    int res;
    do {
        res = parse_entity(lcmgen, item.lcmFilePath.toStdString().c_str(), t);
    } while (res == 0);

    int64_t hash = 0;
    if (lcmgen->structs->len > 0) {
        lcm_struct_t* last_item = (lcm_struct_t*)g_ptr_array_index(lcmgen->structs, lcmgen->structs->len - 1);
        hash = last_item->hash;
    }

    tokenize_destroy(t);

    topic_map[item.topic.toStdString()] = {hash, lcmgen, item.lcmFilePath.toStdString()};
    std::cout << "item.topic: " << item.topic.toStdString() << std::endl;
    std::cout << "item.lcmFilePath: " << item.lcmFilePath.toStdString() << std::endl;
  }
  delete dialog;
  
  // 3. 打开二进制 LCM 日志文件
  lcm::LogFile lcm_log(info->filename.toStdString(), "r");
  if (!lcm_log.good()) {
      std::cerr << "错误：无法打开或读取 LCM 日志文件: " << info->filename.toStdString() << std::endl;
      return -1;
  }
  int max_size = get_file_size(lcm_log.getFilePtr());
  QProgressDialog progress_dialog;
  progress_dialog.setWindowTitle("Loading the rosbag");
  progress_dialog.setLabelText("Loading... please wait");
  progress_dialog.setWindowModality(Qt::ApplicationModal);
  progress_dialog.setRange(0, 100);
  progress_dialog.show();
//return (hash<<1) + ((hash>>63)&1);
  // 4. 核心快转主循环
  while (true) {
    const lcm::LogEvent* event = lcm_log.readNextEvent();
    if (!event) break; // 读完整个 log 文件，平稳退出

    if (ftell(lcm_log.getFilePtr()) * 100 / max_size > progress_dialog.value()) {
      progress_dialog.setValue(ftell(lcm_log.getFilePtr()) * 100 / max_size);
      QApplication::processEvents();

      if (progress_dialog.wasCanceled())
      {
          return false;
      }
    }


    //   int64_t msg_hash;
    // thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    // if (thislen < 0) return thislen; else pos += thislen;
    // if (msg_hash != getHash()) return -1;

    // thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    // if (thislen < 0) return thislen; else pos += thislen;
    //std::string channel = event->channel;
    if (auto it = topic_map.find(event->channel); it != topic_map.end()) {
      lcmgen_t* topic_lcmgen = it->second.lcmgen;
      int64_t msg_hash;
      int thislen = __int64_t_decode_array(event->data, 0, event->datalen, &msg_hash, 1);
      if (msg_hash == ((it->second.hash<<1) + ((it->second.hash>>63)&1))) {
     // if (true) {
        lcm_struct_t* last_item = (lcm_struct_t*)g_ptr_array_index(topic_lcmgen->structs, topic_lcmgen->structs->len - 1);
        if (g_ptr_array_size(last_item->members)) {
          int pos = 8;
          for (unsigned int mind = 0; mind < g_ptr_array_size(last_item->members); mind++) {
            lcm_member_t *member = (lcm_member_t *) g_ptr_array_index(last_item->members, mind);
          //  std::cout << "event->channel: " << event->channel << ", member->membername: " << member->membername << ", member->type->lctypename: " << member->type->lctypename << std::endl;
            if (g_ptr_array_size(member->dimensions) == 0) {
              auto itr = plot_map.numeric.find(event->channel + "/" + member->membername);
              
              if (itr == plot_map.numeric.end()) {
                itr = plot_map.addNumeric(event->channel + "/" + member->membername);
              }

              if (!strcmp(member->type->lctypename, "int8_t")) {
                int8_t value;
                int tlen = __int8_t_decode_array(event->data, pos, event->datalen, &value, 1);
                if(tlen < 0) break; else pos += tlen;
                itr->second.pushBack({ (double)event->timestamp * 1000, value });
              } else if (!strcmp(member->type->lctypename, "int16_t")) {
                int16_t value;
                int tlen = __int16_t_decode_array(event->data, pos, event->datalen, &value, 1);
                if(tlen < 0) break; else pos += tlen;
                itr->second.pushBack({ (double)event->timestamp * 1000, value });
              } else if (!strcmp(member->type->lctypename, "int32_t")) {
                int32_t value;
                int tlen = __int32_t_decode_array(event->data, pos, event->datalen, &value, 1);
                if(tlen < 0) break; else pos += tlen;
                itr->second.pushBack({ (double)event->timestamp * 1000, value });
              } else if (!strcmp(member->type->lctypename, "int64_t")) {
                int64_t value;
                int tlen = __int64_t_decode_array(event->data, pos, event->datalen, &value, 1);
                if(tlen < 0) break; else pos += tlen;
                itr->second.pushBack({ (double)event->timestamp * 1000, value });
              } else if (!strcmp(member->type->lctypename, "float")) {
                float value;
                int tlen = __float_decode_array(event->data, pos, event->datalen, &value, 1);
                if(tlen < 0) break; else pos += tlen;
                itr->second.pushBack({ (double)event->timestamp * 1000, value });
              } else if (!strcmp(member->type->lctypename, "double")) {
                double value;
                int tlen = __double_decode_array(event->data, pos, event->datalen, &value, 1);
                if(tlen < 0) break; else pos += tlen;
                itr->second.pushBack({ (double)event->timestamp * 1000, value });
              }
            } else {
                if (lcm_is_constant_size_array(member)) {
                  for (unsigned int d = 0; d < g_ptr_array_size(member->dimensions); d++) {
                      lcm_dimension_t *ld =
                          (lcm_dimension_t *) g_ptr_array_index(member->dimensions, d);
                      
                      for (size_t i = 0; i < atoi(ld->size); i++) {
                        auto itr = plot_map.numeric.find(event->channel + "/" + member->membername + "/" + std::to_string(i));
                        if (itr == plot_map.numeric.end()) {
                          itr = plot_map.addNumeric(event->channel + "/" + member->membername + "/" + std::to_string(i));
                        }

                        if (!strcmp(member->type->lctypename, "int8_t")) {
                          int8_t value;
                          int tlen = __int8_t_decode_array(event->data, pos, event->datalen, &value, 1);
                          if(tlen < 0) break; else pos += tlen;
                          itr->second.pushBack({ (double)event->timestamp * 1000, value });
                        } else if (!strcmp(member->type->lctypename, "int16_t")) {
                          int16_t value;
                          int tlen = __int16_t_decode_array(event->data, pos, event->datalen, &value, 1);
                          if(tlen < 0) break; else pos += tlen;
                          itr->second.pushBack({ (double)event->timestamp * 1000, value });
                        } else if (!strcmp(member->type->lctypename, "int32_t")) {
                          int32_t value;
                          int tlen = __int32_t_decode_array(event->data, pos, event->datalen, &value, 1);
                          if(tlen < 0) break; else pos += tlen;
                          itr->second.pushBack({ (double)event->timestamp * 1000, value });
                        } else if (!strcmp(member->type->lctypename, "int64_t")) {
                          int64_t value;
                          int tlen = __int64_t_decode_array(event->data, pos, event->datalen, &value, 1);
                          if(tlen < 0) break; else pos += tlen;
                          itr->second.pushBack({ (double)event->timestamp * 1000, value });
                        } else if (!strcmp(member->type->lctypename, "float")) {
                          float value;
                          int tlen = __float_decode_array(event->data, pos, event->datalen, &value, 1);
                          if(tlen < 0) break; else pos += tlen;
                          itr->second.pushBack({ (double)event->timestamp * 1000, value });
                        } else if (!strcmp(member->type->lctypename, "double")) {
                          double value;
                          int tlen = __double_decode_array(event->data, pos, event->datalen, &value, 1);
                          if(tlen < 0) break; else pos += tlen;
                          itr->second.pushBack({ (double)event->timestamp * 1000, value });
                        }

                      }





                        
                  //      std::cout << "Dimension " << d << ": mode = " << ld->mode << ", size = " << ld->size << std::endl;
                    }
                }
            }
          }
        }
      }else {
        std::cerr << "Warning: Hash mismatch for channel " << event->channel << ". Skipping event." << std::endl;
        QMessageBox::warning(nullptr, tr("Error"), "Hash mismatch for channel " + QString::fromStdString(event->channel) + ". exit............");
        return false;
      }
    }

    // // 时间戳精准转换：LCM 标准微秒 (us) -> ROS2 标准纳秒 (ns)
    // rcutils_time_point_value_t timestamp_ns = event->timestamp * 1000;
    // std::cout << "Channel: " << channel << ", Timestamp (ns): " << timestamp_ns << std::endl;
    // std::cout << "Event Number: " << event->eventnum << std::endl;
    // std::cout << "Data Length: " << get_file_size(lcm_log.getFilePtr()) << std::endl;
    // std::cout << "Current Position: " << ftell(lcm_log.getFilePtr()) << std::endl;
  }
  std::cout << "Finished reading LCM log file." << std::endl;
  return true;
//   QString bagDir;
//   {
//     QFileInfo finfo(info->filename);
//     bagDir = finfo.dir().path();
//   }
//   std::cout << "bagDir: " << bagDir.toStdString() << std::endl;
//   rosbag2_storage::StorageOptions storageOptions;
//   const auto bag_metadata = metadata_io->read_metadata(bagDir.toStdString());
//   storageOptions.uri = bagDir.toStdString();
//   storageOptions.storage_id = bag_metadata.storage_identifier;
//   rosbag2_cpp::ConverterOptions converterOptions;
//   converterOptions.input_serialization_format = "cdr";
//   converterOptions.output_serialization_format = rmw_get_serialization_format();

//   std::shared_ptr<rosbag2_cpp::Reader> temp_bag_reader =
//       rosbag2_transport::ReaderWriterFactory::make_reader(storageOptions);

//   QString oldPath = QDir::currentPath();
//   QDir::setCurrent(QDir::cleanPath(bagDir + QDir::separator() + ".."));

//   try
//   {
//     temp_bag_reader->open(storageOptions, converterOptions);
//   }
//   catch (std::runtime_error& ex)
//   {
//     QMessageBox::warning(nullptr, tr("Error"), QString("rosbag::open thrown an exception:\n") + QString(ex.what()));
//     return false;
//   }

//   QDir::setCurrent(oldPath);

//   // Temporarily change the current directory as a workaround for rosbag2 relative directories not
//   // working properly
//   std::vector<rosbag2_storage::TopicMetadata> topic_metadata = temp_bag_reader->get_all_topics_and_types();

//   std::unordered_map<std::string, std::string> topicTypesByName;

//   std::vector<std::pair<QString, QString>> all_topics_qt;

   std::vector<TopicInfo> topics_info;

//   std::set<std::string> blacklist_topic_name;
//   std::set<std::string> failed_topic_type;

//   for (const rosbag2_storage::TopicMetadata& topic : topic_metadata)
//   {
//     all_topics_qt.push_back({ QString::fromStdString(topic.name), QString::fromStdString(topic.type) });
//     topicTypesByName.emplace(topic.name, topic.type);

//     const auto& typesupport_identifier = rosidl_typesupport_cpp::typesupport_identifier;
//     try
//     {
//       topics_info.emplace_back(CreateTopicInfo(topic.name, topic.type));
//     }
//     catch (...)
//     {
//       failed_topic_type.insert(topic.type);
//       blacklist_topic_name.insert(topic.type);
//     }
//   }

//   if (!failed_topic_type.empty())
//   {
//     QString msg("Can not recognize the following message types and those topics will be ignored:\n\n");
//     for (const auto& type : failed_topic_type)
//     {
//       msg += "  " + QString::fromStdString(type) + "\n";
//     }
//     msg += "\nHave you forgotten to source your workspace?";
//     QMessageBox::warning(nullptr, tr("Error"), msg);
//   }

//   // FIXME: we keep this for backward compatibility
//   if (info->plugin_config.hasChildNodes())
//   {
//     xmlLoadState(info->plugin_config.firstChildElement());
//   }

//   if (!info->plugin_config.hasChildNodes() || _config.topics.empty())
//   {
//     loadDefaultSettings();
//     DialogSelectRosTopics* dialog = new DialogSelectRosTopics(all_topics_qt, _config);

//     if (dialog->exec() != static_cast<int>(QDialog::Accepted))
//     {
//       delete dialog;
//       return false;
//     }
//     _config = dialog->getResult();
//     delete dialog;
//   }

//   //--- Swith the previous bag with this one
//   // clean up previous MessageInstances
//   plot_map.user_defined.clear();

//   if (_bag_reader)
//   {
//     _bag_reader->close();
//   }
//   _bag_reader = temp_bag_reader;
//   //---------------------------------------

//   saveDefaultSettings();

//   PJ::CompositeParser parser;

//   std::set<std::string> topic_selected;
//   for (const auto& topic_qt : _config.topics)
//   {
//     std::string topic_name = topic_qt.toStdString();
//     std::string topic_type = topicTypesByName.at(topic_name);
//     topic_selected.insert(topic_name);

//     auto ros2_parser = CreateParserROS2(*parserFactories(), topic_name, topic_type, plot_map);
//     parser.addParser(topic_name, ros2_parser);
//   }

//   parser.setConfig(_config);

//   QProgressDialog progress_dialog;
//   progress_dialog.setWindowTitle("Loading the rosbag");
//   progress_dialog.setLabelText("Loading... please wait");
//   progress_dialog.setWindowModality(Qt::ApplicationModal);
//   progress_dialog.setRange(0, bag_metadata.message_count);
//   progress_dialog.show();
//   int msg_count = 0;

//   PJ::PlotDataAny& plot_consecutive = plot_map.addUserDefined("plotjuggler::rosbag2_cpp::consecutive_messages")->second;
//   PJ::PlotDataAny& metadata_storage = plot_map.addUserDefined("plotjuggler::rosbag2_cpp::topics_metadata")->second;

//   // dirty trick. Store it in a one point timeseries
//   metadata_storage.pushBack({ 0, std::any(topics_info) });

//   auto time_prev = std::chrono::high_resolution_clock::now();

//   while (_bag_reader->has_next())
//   {
//     auto msg = _bag_reader->read_next();
//     const std::string& topic_name = msg->topic_name;
//     if (blacklist_topic_name.count(topic_name))
//     {
//       continue;
//     }

// #ifdef ROS_HUMBLE
//     const double msg_timestamp = 1e-9 * double(msg->time_stamp);  // nanoseconds to seconds
// #else
//     // from jazzy and later
//     const double msg_timestamp = 1e-9 * double(msg->send_timestamp);  // nanoseconds to seconds
// #endif

//     //------ progress dialog --------------
//     if (msg_count++ % 100 == 0)
//     {
//       progress_dialog.setValue(msg_count);
//       QApplication::processEvents();

//       if (progress_dialog.wasCanceled())
//       {
//         return false;
//       }
//     }

//     //----- skip not selected -----------
//     if (topic_selected.find(topic_name) == topic_selected.end())
//     {
//       continue;
//     }
//     //----- parse! -----------
//     double timestamp = msg_timestamp;
//     PJ::MessageRef msg_ref(msg->serialized_data->buffer, msg->serialized_data->buffer_length);
//     parser.parseMessage(topic_name, msg_ref, timestamp);

//     //---- save msg reference in PlotAny ----
//     auto data_point = PJ::PlotDataAny::Point(timestamp, std::any(msg));
//     plot_consecutive.pushBack(data_point);

//     auto plot_pair = plot_map.user_defined.find(topic_name);
//     if (plot_pair == plot_map.user_defined.end())
//     {
//       plot_pair = plot_map.addUserDefined(topic_name);
//     }
//     PJ::PlotDataAny& plot_raw = plot_pair->second;
//     plot_raw.pushBack(data_point);
//   }

//   auto now = std::chrono::high_resolution_clock::now();
//   double diff = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_prev).count();

//   qDebug() << "The rosbag loaded the data in " << diff << " milliseconds";

//   return true;
}

bool DataLoadLCM::xmlSaveState(QDomDocument& doc, QDomElement& plugin_elem) const
{
  _config.xmlSaveState(doc, plugin_elem);
  return true;
}

bool DataLoadLCM::xmlLoadState(const QDomElement& parent_element)
{
  _config.xmlLoadState(parent_element);
  return true;
}

void DataLoadLCM::saveDefaultSettings()
{
  QSettings settings;
  _config.saveToSettings(settings, "DataLoadLCM");
}

void DataLoadLCM::loadDefaultSettings()
{
  QSettings settings;
  _config.loadFromSettings(settings, "DataLoadLCM");
}
