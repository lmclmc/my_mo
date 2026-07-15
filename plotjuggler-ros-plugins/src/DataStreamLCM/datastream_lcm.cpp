#include "datastream_lcm.h"
#include "LcmTopicDialog.h"

#include <QDebug>
#include <QTimer>
#include <QSettings>
#include <QMessageBox>
#include <QApplication>
#include <QProgressDialog>

// function taken from rosbag2_transport

DataStreamLCM::DataStreamLCM() : DataStreamer(), _running(false), _first_warning(false)
{
  loadDefaultSettings();
}

int DataStreamLCM::decode_type(const lcm::ReceiveBuffer* recvbuf, int offset,
                               const std::string& type, const std::string& base, int deep) {
  //偏移量，递归调用时，给下一级函数使用
  int pos_offset = offset;
  //已解码长度，递归调用时，返回给上一级函数
  int decode_len = 0;
  std::unordered_map<std::string, int> member_map;
  if (auto lcm_struct_type = lcm_type_table.find(type); lcm_struct_type != lcm_type_table.end()) {
    lcm_struct_t* struct_item = lcm_struct_type->second;

    for (unsigned int mind = 0; mind < g_ptr_array_size(struct_item->members); mind++) {
      lcm_member_t* member = (lcm_member_t*) g_ptr_array_index(struct_item->members, mind);
      auto itr = dataMap().numeric.find(base + member->membername);
      if (itr == dataMap().numeric.end()) {
        itr = dataMap().addNumeric(base + member->membername);
      }

      double local_timestamp = (double)recvbuf->recv_utime / 1000000;
      if (g_ptr_array_size(member->dimensions) == 0) {
        if (!strcmp(member->type->lctypename, "int8_t")) {
          int8_t value;
          int tlen = __int8_t_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
          if(tlen < 0) break; else {
            pos_offset += tlen;
            decode_len += tlen;
          }
          itr->second.pushBack({ local_timestamp, value });
          member_map[member->membername] = value;
        } else if (!strcmp(member->type->lctypename, "int16_t")) {
          int16_t value;
          int tlen = __int16_t_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
          if(tlen < 0) break; else {
            pos_offset += tlen;
            decode_len += tlen;
          }
          itr->second.pushBack({ local_timestamp, value });
          member_map[member->membername] = value;
        } else if (!strcmp(member->type->lctypename, "int32_t")) {
          int32_t value;
          int tlen = __int32_t_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
          if(tlen < 0) break; else {
            pos_offset += tlen;
            decode_len += tlen;
          }
          itr->second.pushBack({ local_timestamp, value });
          member_map[member->membername] = value;
        } else if (!strcmp(member->type->lctypename, "int64_t")) {
          int64_t value;
          int tlen = __int64_t_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
          if(tlen < 0) break; else {
            pos_offset += tlen;
            decode_len += tlen;
          }
          itr->second.pushBack({ local_timestamp, value });
          member_map[member->membername] = value;
        } else if (!strcmp(member->type->lctypename, "float")) {
          float value;
          int tlen = __float_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);

          if(tlen < 0) break; else {
            pos_offset += tlen;
            decode_len += tlen;
          }
          itr->second.pushBack({ local_timestamp, value });
        } else if (!strcmp(member->type->lctypename, "double")) {
          double value;
          int tlen = __double_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
          if(tlen < 0) break; else {
            pos_offset += tlen;
            decode_len += tlen;
          }
          itr->second.pushBack({ local_timestamp, value });
        } else if (!strcmp(member->type->lctypename, "string")) {
          int32_t __d_len__;
          int tlen = __int32_t_decode_array(
              recvbuf->data, pos_offset, recvbuf->data_size, &__d_len__, 1);
          if(tlen < 0) break; else {
            pos_offset += tlen;
            decode_len += tlen;
          }
          if(__d_len__ > recvbuf->data_size - pos_offset || __d_len__ <= 0) break;
          std::string lcm_str(static_cast<const char*>(recvbuf->data) + pos_offset,  __d_len__ - 1);
          pos_offset += __d_len__;
          decode_len += __d_len__;
          auto str_itr = dataMap().strings.find(base + member->membername);
          if (str_itr == dataMap().strings.end()) {
            str_itr = dataMap().addStringSeries(base + member->membername);
          }
          str_itr->second.pushBack({ local_timestamp, lcm_str });
        } else if (!strcmp(member->type->lctypename, "boolean")) {
          int8_t value;
          int tlen = __boolean_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
          if(tlen < 0) break; else {
            pos_offset += tlen;
            decode_len += tlen;
          }
          itr->second.pushBack({ local_timestamp, value });
        } else {
          int tlen = decode_type(recvbuf, pos_offset, member->type->lctypename, base + member->membername + "/", deep + 1);
          if(tlen < 0) break; else {
            pos_offset += tlen;
            decode_len += tlen;
          }
        }
      } else {
        for (unsigned int d = 0; d < g_ptr_array_size(member->dimensions); d++) {
          lcm_dimension_t *ld =
              (lcm_dimension_t *) g_ptr_array_index(member->dimensions, d);
          if (!lcm_is_constant_size_array(member)) { //判断是否是变长大小数组
            if (auto it = member_map.find(ld->size); it != member_map.end()) {
              for (size_t i = 0; i < it->second; i++) {
                int tlen = decode_type(recvbuf, pos_offset, member->type->lctypename, base + member->membername + "/" + std::to_string(i) + "/", deep + 1);
                if(tlen < 0) break; else {
                  pos_offset += tlen;
                  decode_len += tlen;
                }
              }
              break;
            }
          }
          
          for (size_t i = 0; i < atoi(ld->size); i++) {
            auto itr = dataMap().numeric.find(base + member->membername + "/" + std::to_string(i));
            if (itr == dataMap().numeric.end()) {
              itr = dataMap().addNumeric(base + member->membername + "/" + std::to_string(i));
            }

            if (!strcmp(member->type->lctypename, "int8_t")) {
              int8_t value;
              int tlen = __int8_t_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
              if(tlen < 0) break; else {
                pos_offset += tlen;
                decode_len += tlen;
              }
              itr->second.pushBack({ local_timestamp, value });
            } else if (!strcmp(member->type->lctypename, "int16_t")) {
              int16_t value;
              int tlen = __int16_t_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
              if(tlen < 0) break; else {
                pos_offset += tlen;
                decode_len += tlen;
              }
              itr->second.pushBack({ local_timestamp, value });
            } else if (!strcmp(member->type->lctypename, "int32_t")) {
              int32_t value;
              int tlen = __int32_t_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
              if(tlen < 0) break; else {
                pos_offset += tlen;
                decode_len += tlen;
              }
              itr->second.pushBack({ local_timestamp, value });
            } else if (!strcmp(member->type->lctypename, "int64_t")) {
              int64_t value;
              int tlen = __int64_t_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
              if(tlen < 0) break; else {
                pos_offset += tlen;
                decode_len += tlen;
              }
              itr->second.pushBack({ local_timestamp, value });
            } else if (!strcmp(member->type->lctypename, "float")) {
              float value;
              int tlen = __float_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
              if(tlen < 0) break; else {
                pos_offset += tlen;
                decode_len += tlen;
              }
              itr->second.pushBack({ local_timestamp, value });
            } else if (!strcmp(member->type->lctypename, "double")) {
              double value;
              int tlen = __double_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
              if(tlen < 0) break; else {
                pos_offset += tlen;
                decode_len += tlen;
              }
              itr->second.pushBack({ local_timestamp, value });
            } else if (!strcmp(member->type->lctypename, "string")) { 
              int32_t __d_len__;
              int tlen = __int32_t_decode_array(
                  recvbuf->data, pos_offset, recvbuf->data_size, &__d_len__, 1);
              if(tlen < 0) break; else {
                pos_offset += tlen;
                decode_len += tlen;
              }
              if(__d_len__ > recvbuf->data_size - pos_offset || __d_len__ <= 0) break;
              std::string lcm_str(static_cast<const char*>(recvbuf->data) + pos_offset,  __d_len__ - 1);
              pos_offset += __d_len__;
              decode_len += tlen;
              auto str_itr = dataMap().strings.find(base + member->membername + "/" + std::to_string(i));
              if (str_itr == dataMap().strings.end()) {
                str_itr = dataMap().addStringSeries(base + member->membername + "/" + std::to_string(i));
              }
              str_itr->second.pushBack({ local_timestamp, lcm_str });
            } else if (!strcmp(member->type->lctypename, "boolean")) {
              int8_t value;
              int tlen = __boolean_decode_array(recvbuf->data, pos_offset, recvbuf->data_size, &value, 1);
              if(tlen < 0) return tlen; else pos_offset += tlen;
              itr->second.pushBack({ local_timestamp, value });
            } else {
              int tlen = decode_type(recvbuf, pos_offset, member->type->lctypename, base + member->membername + "/" + std::to_string(i) + "/", deep + 1);
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

  return decode_len;
}

void DataStreamLCM::handleMessage(const lcm::ReceiveBuffer* recvbuf, const std::string& topic_name) {
  if (recvbuf->data_size < 8) return; // 所有的 LCM 消息头至少 8 字节
  
  lcm_log->write(topic_name, recvbuf);
  if (auto it = topic_map.find(topic_name); it != topic_map.end()) {
    std::lock_guard<std::mutex> lock(mutex());
    bool channel_match = false;
    for (auto topic_lcmgen : it->second.lcmgen_table) {
      int64_t msg_hash;
      int thislen = __int64_t_decode_array(recvbuf->data, 0, recvbuf->data_size, &msg_hash, 1);
      for (int struct_idx = 0; struct_idx < topic_lcmgen->structs->len; struct_idx++) {
        lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(topic_lcmgen->structs, struct_idx);
        if (msg_hash == ((struct_item->hash<<1) + ((struct_item->hash>>63)&1))) {
          decode_type(recvbuf, 8, struct_item->structname->lctypename, topic_name + "/");
          channel_match = true;
          break;
        }
      }
    }

    if (!channel_match) {
      std::cerr << "Warning: Hash mismatch for channel " << topic_name << ". Skipping event." << std::endl;
      QMessageBox::warning(nullptr, tr("Error"), "话题：" + QString::fromStdString(topic_name) + "类型不匹配，退出。。。。。。");
      _running = false;
      if (lcm_runner_.joinable()) {
        lcm_runner_.join();
      }

      lcm_.reset();
      return;
    }
  }

  emit dataReceived();
}

bool DataStreamLCM::start(QStringList* selected_datasources)
{
  LcmTopicDialog* dialog = new LcmTopicDialog(1);
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
  
  topic_map.clear();
  auto _config = dialog->getConfig();

  lcm_type_table.clear();
  lcm_type_table["int8_t"] = nullptr;
  lcm_type_table["int16_t"] = nullptr;
  lcm_type_table["int32_t"] = nullptr;
  lcm_type_table["int64_t"] = nullptr;
  lcm_type_table["float"] = nullptr;
  lcm_type_table["double"] = nullptr;
  lcm_type_table["string"] = nullptr;
  lcm_type_table["boolean"] = nullptr;
  
  for (const auto& item : _config) {
    if (item.lcmFilePath.isEmpty()) continue;

    // 检查文件是否存在
    std::string lcm_file_path = item.lcmFilePath.toStdString();
    std::ifstream file_check(lcm_file_path);
    if (!file_check.good()) {
      QMessageBox::warning(nullptr, tr("Error"), "文件不存在: " + item.lcmFilePath + "，跳过加载。");
      std::cerr << "Warning: File does not exist: " << lcm_file_path << std::endl;
      return false;
    }

    lcmgen_t* lcmgen = lcmgen_create();
    tokenize_t* t = tokenize_create(item.lcmFilePath.toStdString().c_str());
    int res;
    do {
        res = parse_entity(lcmgen, item.lcmFilePath.toStdString().c_str(), t);
        if (res == -10) {
          QMessageBox::warning(nullptr, tr("Error"), "文件  " + item.lcmFilePath + "格式错误. 退出加载...........");
          return false;
        }
    } while (res == 0);

    for (int struct_idx = 0; struct_idx < lcmgen->structs->len; struct_idx++) {
      lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(lcmgen->structs, struct_idx);
      lcm_type_table[struct_item->structname->lctypename] = struct_item;
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
  QString lcm_url = QString::fromStdString("udpm://239.255.76.67:") + dialog->getPort();
  lcm_ = std::make_shared<lcm::LCM>(lcm_url.toStdString()); // 使用全局变量

  log_save_path = dialog->getLogSavePath();
  log_max_size = dialog->getLogMaxSize();
  log_max_files_num = dialog->getLogMaxFiles();

  lcm_log = std::make_shared<rotating_file_sink>(log_save_path.toStdString(), log_max_size, log_max_files_num);

  delete dialog;

  for (auto& pair : topic_map) {
    for (auto lcmgen : pair.second.lcmgen_table) {
      for (int struct_idx = 0; struct_idx < lcmgen->structs->len; struct_idx++) {
        lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(lcmgen->structs, struct_idx);
        if (g_ptr_array_size(struct_item->members)) {
          for (unsigned int mind = 0; mind < g_ptr_array_size(struct_item->members); mind++) {
            lcm_member_t *member = (lcm_member_t *) g_ptr_array_index(struct_item->members, mind);
            if (auto it = lcm_type_table.find(member->type->lctypename); it != lcm_type_table.end()) {
              if (it->second) {
                uint64_t hash = struct_item->hash + (it->second->hash<<1) + ((it->second->hash>>63)&1);
                struct_item->hash = hash;
                pair.second.hash = hash;
              }
            } else {
              QMessageBox::warning(nullptr, tr("Error"), "类型  " + QString::fromStdString(member->type->lctypename) + " 未定义. 退出加载...........");
              return false;
            }
          }
        }
      }
    }
  }

  lcm_->subscribe(".*", &DataStreamLCM::handleMessage, this);
  _running = true;
  lcm_runner_ = std::thread([this] {
    while (_running) {
      lcm_->handleTimeout(100);
    }
  });
  
  return true;
}

bool DataStreamLCM::isRunning() const
{
  std::cout << "DataStreamLCM::isRunning" << std::endl;
  return _running;
}

void DataStreamLCM::shutdown()
{
  std::cout << "DataStreamLCM::shutdown" << std::endl;
  _running = false;
  
  if (lcm_runner_.joinable()) {
    lcm_runner_.join();
  }

  lcm_.reset();
  lcm_log.reset();
}

DataStreamLCM::~DataStreamLCM()
{
  shutdown();
}

const std::vector<QAction*>& DataStreamLCM::availableActions()
{
  static std::vector<QAction*> empty;
  return empty;
}

void DataStreamLCM::saveDefaultSettings()
{
  QSettings settings;
  _config.saveToSettings(settings, "DataStreamLCM");
}

void DataStreamLCM::loadDefaultSettings()
{
  QSettings settings;
  _config.loadFromSettings(settings, "DataStreamLCM");
}

bool DataStreamLCM::xmlLoadState(const QDomElement& parent_element)
{
  _config.xmlLoadState(parent_element);
  return true;
}

bool DataStreamLCM::xmlSaveState(QDomDocument& doc, QDomElement& parent_element) const
{
  _config.xmlSaveState(doc, parent_element);
  return true;
}
