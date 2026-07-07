#include "dataload_lcm.h"
#include "dialog_select_ros_topics.h"
#include "LcmTopicDialog.h"
#include "ros_parsers/ros2_parser.h"

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

const std::vector<const char*>& DataLoadLCM::compatibleFileExtensions() const {
    return _extensions;
}

int DataLoadLCM::decode_type(const lcm::LogEvent* event, int offset,
                             const std::string& type, const std::string& base, int deep) {
    int pos_offset = offset;
    int decode_len = 0;
   // std::cout << "deep: " << deep << std::endl;
   // std::cout << dec << std::endl;   // 255 切回十进制
   // std::cout << "offset: " << pos_offset << std::endl;
    if (auto lcm_struct_type = lcm_type_table.find(type); lcm_struct_type != lcm_type_table.end()) {
        lcm_struct_t* struct_item = lcm_struct_type->second;
        for (unsigned int mind = 0; mind < g_ptr_array_size(struct_item->members); mind++) {
            lcm_member_t* member = (lcm_member_t*) g_ptr_array_index(struct_item->members, mind);
            auto itr = plot_data->numeric.find(base + member->membername);
            if (itr == plot_data->numeric.end()) {
              itr = plot_data->addNumeric(base + member->membername);
            }

            if (g_ptr_array_size(member->dimensions) == 0) {
               // std::cout << "1111111111 Received message on channel \"" << member->type->lctypename << "\"\n";
                if (!strcmp(member->type->lctypename, "int8_t")) {
                  int8_t value;
                  int tlen = __int8_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                  if(tlen < 0) break; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                  itr->second.pushBack({ (double)event->timestamp, value });
                } else if (!strcmp(member->type->lctypename, "int16_t")) {
                  int16_t value;
                  int tlen = __int16_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                  if(tlen < 0) break; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                  itr->second.pushBack({ (double)event->timestamp, value });
                } else if (!strcmp(member->type->lctypename, "int32_t")) {
                  int32_t value;
                  int tlen = __int32_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                  if(tlen < 0) break; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                  itr->second.pushBack({ (double)event->timestamp, value });
                } else if (!strcmp(member->type->lctypename, "int64_t")) {
                  int64_t value;
                  int tlen = __int64_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                  if(tlen < 0) break; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                  itr->second.pushBack({ (double)event->timestamp, value });
                } else if (!strcmp(member->type->lctypename, "float")) {
                  float value;
                  int tlen = __float_decode_array(event->data, pos_offset, event->datalen, &value, 1);

                  if(tlen < 0) break; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                  itr->second.pushBack({ (double)event->timestamp, value });
                } else if (!strcmp(member->type->lctypename, "double")) {
                  double value;
                  int tlen = __double_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                  if(tlen < 0) break; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                  itr->second.pushBack({ (double)event->timestamp, value });
                } else {
                  std::cout << "pos_offset  : " << pos_offset << std::endl;
                  int tlen = decode_type(event, pos_offset, member->type->lctypename, base + member->membername + "/", deep + 1);
                  std::cout << "tlen decode_type : " << tlen << std::endl;
                  if(tlen < 0) break; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                }
            } else {
                if (lcm_is_constant_size_array(member)) {
                    for (unsigned int d = 0; d < g_ptr_array_size(member->dimensions); d++) {
                        lcm_dimension_t *ld =
                            (lcm_dimension_t *) g_ptr_array_index(member->dimensions, d);
                        
                        for (size_t i = 0; i < atoi(ld->size); i++) {
                            auto itr = plot_data->numeric.find(base + member->membername + "/" + std::to_string(i));
                            if (itr == plot_data->numeric.end()) {
                              itr = plot_data->addNumeric(base + member->membername + "/" + std::to_string(i));
                            }

                            if (!strcmp(member->type->lctypename, "int8_t")) {
                              int8_t value;
                              int tlen = __int8_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                              if(tlen < 0) break; else {
                                pos_offset += tlen;
                                decode_len += tlen;
                              }
                              itr->second.pushBack({ (double)event->timestamp, value });
                            } else if (!strcmp(member->type->lctypename, "int16_t")) {
                              int16_t value;
                              int tlen = __int16_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                              if(tlen < 0) break; else {
                                pos_offset += tlen;
                                decode_len += tlen;
                              }
                              itr->second.pushBack({ (double)event->timestamp, value });
                            } else if (!strcmp(member->type->lctypename, "int32_t")) {
                              int32_t value;
                              int tlen = __int32_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                              if(tlen < 0) break; else {
                                pos_offset += tlen;
                                decode_len += tlen;
                              }
                              itr->second.pushBack({ (double)event->timestamp, value });
                            } else if (!strcmp(member->type->lctypename, "int64_t")) {
                              int64_t value;
                              int tlen = __int64_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                              if(tlen < 0) break; else {
                                pos_offset += tlen;
                                decode_len += tlen;
                              }
                              itr->second.pushBack({ (double)event->timestamp, value });
                            } else if (!strcmp(member->type->lctypename, "float")) {
                              float value;
                              int tlen = __float_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                              if(tlen < 0) break; else {
                                pos_offset += tlen;
                                decode_len += tlen;
                              }
                              itr->second.pushBack({ (double)event->timestamp, value });
                            } else if (!strcmp(member->type->lctypename, "double")) {
                              double value;
                              int tlen = __double_decode_array(event->data, pos_offset, event->datalen, &value, 1);
                              if(tlen < 0) break; else {
                                pos_offset += tlen;
                                decode_len += tlen;
                              }
                              itr->second.pushBack({ (double)event->timestamp, value });
                            } else {
                              int tlen = decode_type(event, pos_offset, member->type->lctypename, base + member->membername + "/" + std::to_string(i) + "/", deep + 1);
                              if(tlen < 0) break; else {
                                pos_offset += tlen;
                                decode_len += tlen;
                              }
                            }
                        }
                    }
                }
            }
        }
    }

    return decode_len;
}

bool DataLoadLCM::readDataFromFile(PJ::FileLoadInfo* info, PJ::PlotDataMapRef& plot_map) {
  plot_data = &plot_map;
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
    std::vector<lcmgen_t*> lcmgen_table;
    std::string lcmFilePath;
  } TopicLcmItem;
  
  std::unordered_map<std::string, TopicLcmItem> topic_map;
  auto _config = dialog->getConfig();

  lcm_type_table.clear();
  lcm_type_table["int8_t"] = nullptr;
  lcm_type_table["int16_t"] = nullptr;
  lcm_type_table["int32_t"] = nullptr;
  lcm_type_table["int64_t"] = nullptr;
  lcm_type_table["float"] = nullptr;
  lcm_type_table["double"] = nullptr;
  
  for (const auto& item : _config) {
    lcmgen_t* lcmgen = lcmgen_create();
    tokenize_t* t = tokenize_create(item.lcmFilePath.toStdString().c_str());
    int res;
    do {
        res = parse_entity(lcmgen, item.lcmFilePath.toStdString().c_str(), t);
    } while (res == 0);

    for (int struct_idx = 0; struct_idx < lcmgen->structs->len; struct_idx++) {
      lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(lcmgen->structs, struct_idx);
      lcm_type_table[struct_item->structname->lctypename] = struct_item;
      std::cout << "struct_item->structname->lctypename: " << struct_item->structname->lctypename << std::endl;
    }

    int64_t hash = 0;
    if (lcmgen->structs->len > 0) {
        lcm_struct_t* last_item = (lcm_struct_t*)g_ptr_array_index(lcmgen->structs, lcmgen->structs->len - 1);
        hash = last_item->hash;
    }

    tokenize_destroy(t);

    if (auto it = topic_map.find(item.topic.toStdString()); it == topic_map.end()) {
      topic_map[item.topic.toStdString()] = {hash, {lcmgen}, item.lcmFilePath.toStdString()};
    } else {
      it->second.lcmgen_table.push_back(lcmgen);
    }

    std::cout << "item.topic: " << item.topic.toStdString() << std::endl;
    std::cout << "item.lcmFilePath: " << item.lcmFilePath.toStdString() << std::endl;
  }

  delete dialog;
  using namespace std;
  for (auto& pair : topic_map) {
    for (auto lcmgen : pair.second.lcmgen_table) {
      for (int struct_idx = 0; struct_idx < lcmgen->structs->len; struct_idx++) {
        lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(lcmgen->structs, struct_idx);
        if (g_ptr_array_size(struct_item->members)) {
          for (unsigned int mind = 0; mind < g_ptr_array_size(struct_item->members); mind++) {
            lcm_member_t *member = (lcm_member_t *) g_ptr_array_index(struct_item->members, mind);
            if (auto it = lcm_type_table.find(member->type->lctypename); it != lcm_type_table.end()) {
              if (it->second) {
                std::cout << "member->type->lctypename: " << member->type->lctypename << std::endl;
              //  uint64_t hash = (it->second->hash << 1) + ((it->second->hash>>63)&1);
                // std::cout << "hash: " << hex << uppercase << setfill('0') << setw(16)  << it->second->hash << std::endl;
                // std::cout << "hash1: " << hex << uppercase << setfill('0') << setw(16)  << (it->second->hash<<1) + ((it->second->hash>>63)&1) << std::endl;

                
                // std::cout << "hash: " << hex << uppercase << setfill('0') << setw(16)  <<  struct_item->hash << std::endl;

                
                uint64_t hash = struct_item->hash + (it->second->hash<<1) + ((it->second->hash>>63)&1);
                int64_t hashs = static_cast<int64_t>(hash);
            //    hash = struct_item->hash + (hash<<1) + ((hash>>63)&1);
            //    std::cout << "hash111: " << hex << uppercase << setfill('0') << setw(16)  <<  hashs << std::endl;
                struct_item->hash = hashs;
           //     std::cout << "hash111222: " << hex << uppercase << setfill('0') << setw(16)  <<  struct_item->hash << std::endl;
                pair.second.hash = hashs;

              }
             // struct_item->hash = struct_item->hash + (it->second->hash << 1) + ((it->second->hash>>63)&1);
            } else {
              QMessageBox::warning(nullptr, tr("Error"), "type  " + QString::fromStdString(member->type->lctypename) + " undefined. exit...........");
              return false;
            }
          }
        }
      }
    }
  }
  
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

    if (auto it = topic_map.find(event->channel); it != topic_map.end()) {
      bool channel_match = false;
      for (auto topic_lcmgen : it->second.lcmgen_table) {
        int64_t msg_hash;
        int thislen = __int64_t_decode_array(event->data, 0, event->datalen, &msg_hash, 1);
        for (int struct_idx = 0; struct_idx < topic_lcmgen->structs->len; struct_idx++) {
          lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(topic_lcmgen->structs, struct_idx);
          if (msg_hash == ((struct_item->hash<<1) + ((struct_item->hash>>63)&1))) {
            decode_type(event, 8, struct_item->structname->lctypename, event->channel + "/");

            // if (g_ptr_array_size(struct_item->members)) {
            //   int pos = 8;
            //   for (unsigned int mind = 0; mind < g_ptr_array_size(struct_item->members); mind++) {
            //     lcm_member_t *member = (lcm_member_t *) g_ptr_array_index(struct_item->members, mind);
            //   //  std::cout << "event->channel: " << event->channel << ", member->membername: " << struct_item->membername << ", member->type->lctypename: " << member->type->lctypename << "  leg:  " << topic_lcmgen->structs->len << std::endl;
            //     if (g_ptr_array_size(member->dimensions) == 0) {
            //       auto itr = plot_map.numeric.find(event->channel + "/" + member->membername);
                  
            //       if (itr == plot_map.numeric.end()) {
            //         itr = plot_map.addNumeric(event->channel + "/" + member->membername);
            //       }

            //       if (!strcmp(member->type->lctypename, "int8_t")) {
            //         int8_t value;
            //         int tlen = __int8_t_decode_array(event->data, pos, event->datalen, &value, 1);
            //         if(tlen < 0) break; else pos += tlen;
            //         itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //       } else if (!strcmp(member->type->lctypename, "int16_t")) {
            //         int16_t value;
            //         int tlen = __int16_t_decode_array(event->data, pos, event->datalen, &value, 1);
            //         if(tlen < 0) break; else pos += tlen;
            //         itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //       } else if (!strcmp(member->type->lctypename, "int32_t")) {
            //         int32_t value;
            //         int tlen = __int32_t_decode_array(event->data, pos, event->datalen, &value, 1);
            //         if(tlen < 0) break; else pos += tlen;
            //         itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //       } else if (!strcmp(member->type->lctypename, "int64_t")) {
            //         int64_t value;
            //         int tlen = __int64_t_decode_array(event->data, pos, event->datalen, &value, 1);
            //         if(tlen < 0) break; else pos += tlen;
            //         itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //       } else if (!strcmp(member->type->lctypename, "float")) {
            //         float value;
            //         int tlen = __float_decode_array(event->data, pos, event->datalen, &value, 1);
            //         if(tlen < 0) break; else pos += tlen;
            //         itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //       } else if (!strcmp(member->type->lctypename, "double")) {
            //         double value;
            //         int tlen = __double_decode_array(event->data, pos, event->datalen, &value, 1);
            //         if(tlen < 0) break; else pos += tlen;
            //         itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //       }
            //     } else {
            //         if (lcm_is_constant_size_array(member)) {
            //           for (unsigned int d = 0; d < g_ptr_array_size(member->dimensions); d++) {
            //               lcm_dimension_t *ld =
            //                   (lcm_dimension_t *) g_ptr_array_index(member->dimensions, d);
                          
            //               for (size_t i = 0; i < atoi(ld->size); i++) {
            //                 auto itr = plot_map.numeric.find(event->channel + "/" + member->membername + "/" + std::to_string(i));
            //                 if (itr == plot_map.numeric.end()) {
            //                   itr = plot_map.addNumeric(event->channel + "/" + member->membername + "/" + std::to_string(i));
            //                 }

            //                 if (!strcmp(member->type->lctypename, "int8_t")) {
            //                   int8_t value;
            //                   int tlen = __int8_t_decode_array(event->data, pos, event->datalen, &value, 1);
            //                   if(tlen < 0) break; else pos += tlen;
            //                   itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //                 } else if (!strcmp(member->type->lctypename, "int16_t")) {
            //                   int16_t value;
            //                   int tlen = __int16_t_decode_array(event->data, pos, event->datalen, &value, 1);
            //                   if(tlen < 0) break; else pos += tlen;
            //                   itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //                 } else if (!strcmp(member->type->lctypename, "int32_t")) {
            //                   int32_t value;
            //                   int tlen = __int32_t_decode_array(event->data, pos, event->datalen, &value, 1);
            //                   if(tlen < 0) break; else pos += tlen;
            //                   itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //                 } else if (!strcmp(member->type->lctypename, "int64_t")) {
            //                   int64_t value;
            //                   int tlen = __int64_t_decode_array(event->data, pos, event->datalen, &value, 1);
            //                   if(tlen < 0) break; else pos += tlen;
            //                   itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //                 } else if (!strcmp(member->type->lctypename, "float")) {
            //                   float value;
            //                   int tlen = __float_decode_array(event->data, pos, event->datalen, &value, 1);
            //                   if(tlen < 0) break; else pos += tlen;
            //                   itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //                 } else if (!strcmp(member->type->lctypename, "double")) {
            //                   double value;
            //                   int tlen = __double_decode_array(event->data, pos, event->datalen, &value, 1);
            //                   if(tlen < 0) break; else pos += tlen;
            //                   itr->second.pushBack({ (double)event->timestamp * 1000, value });
            //                 }

            //               }





                            
            //           //      std::cout << "Dimension " << d << ": mode = " << ld->mode << ", size = " << ld->size << std::endl;
            //             }
            //         }
            //     }
            //   }
            // }
            break;
          }
        }

        channel_match = true;
      }

      if (!channel_match) {
        std::cerr << "Warning: Hash mismatch for channel " << event->channel << ". Skipping event." << std::endl;
        QMessageBox::warning(nullptr, tr("Error"), "话题：" + QString::fromStdString(event->channel) + "类型不匹配，退出。。。。。。");
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
