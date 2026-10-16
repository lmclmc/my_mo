#include "lcm_source.hpp"

struct TransparentStringHash {
  using is_transparent = void;

  std::size_t operator()(std::string_view value) const noexcept {
    return std::hash<std::string_view>{}(value);
  }

  std::size_t operator()(const std::string& value) const noexcept {
    return (*this)(std::string_view(value));
  }
};

using SelectedTopicSet = std::unordered_set<std::string, TransparentStringHash, std::equal_to<>>;

// Hardware-derived worker count. Floor at 2 so we still get parallelism on
// virtualized hosts that report 1 core; cap at 8 to avoid oversubscription on
// many-core machines where the byte-budget and consumer drain rate become the
// real limits anyway.
inline unsigned parallelImportThreadCount() {
#ifdef PJ_TARGET_WASM
  // Browser pthreads are comparatively expensive and WASM memory growth
  // expands in coarse steps. Two workers keep decompression parallel without
  // multiplying the resident chunk frontier.
  return 2;
#else
  const unsigned hw = std::thread::hardware_concurrency();
  return std::min(8u, std::max(2u, hw));
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
// LcmSource plugin
// ─────────────────────────────────────────────────────────────────────────────
  // ------------------------------------------------------------
// 2. 将 std_msgs::msg::String 序列化为 ROS 2 CDR
// ------------------------------------------------------------
static std::vector<uint8_t> serializeRosString(
    const std::string& urdf_xml) {
    std_msgs::msg::String msg;
    msg.data = urdf_xml;

    rclcpp::Serialization<std_msgs::msg::String> serializer;

    rclcpp::SerializedMessage serialized_msg;

    serializer.serialize_message(&msg, &serialized_msg);

    const auto& raw =
        serialized_msg.get_rcl_serialized_message();

    return std::vector<uint8_t>(
        raw.buffer,
        raw.buffer + raw.buffer_length);
}

// 将 TransformStamped 序列化为 CDR 字节
static std::vector<uint8_t> serializeTransform(
    geometry_msgs::msg::TransformStamped& msg)
{
    rclcpp::Serialization<
        geometry_msgs::msg::TransformStamped> serializer;

    rclcpp::SerializedMessage serialized_msg;

    serializer.serialize_message(&msg, &serialized_msg);

    const auto& raw =
        serialized_msg.get_rcl_serialized_message();

    return std::vector<uint8_t>(
        raw.buffer,
        raw.buffer + raw.buffer_length);
}

static std::vector<uint8_t> serializeJointState(
    sensor_msgs::msg::JointState& msg)
{
    rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;

    rclcpp::SerializedMessage serialized_msg;

    serializer.serialize_message(&msg, &serialized_msg);

    const auto& raw =
        serialized_msg.get_rcl_serialized_message();

    return std::vector<uint8_t>(
        raw.buffer,
        raw.buffer + raw.buffer_length);
}

PJ::Status LcmSource::loadConfig(std::string_view config_json) {
  if (!dialog_.loadConfig(config_json)) {
    return PJ::unexpected(std::string("invalid config JSON"));
  }
  auto cfg = nlohmann::json::parse(config_json, nullptr, false);
  if (!cfg.is_discarded()) {
    if (cfg.contains("_parser_config")) {
      // Parser config forwarded by FileLoader from the embedded pj_parser_slot
      // dialog — takes precedence over everything else.
      parser_config_override_ = cfg["_parser_config"].get<std::string>();
    } else {
      // Migration: old configs had use_timestamp / timestamp_field_name directly
      // in the MCAP dialog config. Synthesize a parser config from those keys so
      // existing saved sessions keep working after the controls moved to
      // JsonParserDialog.
      bool use_ts = cfg.value("use_timestamp", false);
      std::string ts_field = cfg.value("timestamp_field_name", std::string{});
      if (use_ts) {
        nlohmann::json migrated;
        migrated["use_embedded_timestamp"] = true;
        migrated["timestamp_field_name"] = ts_field;
        parser_config_override_ = migrated.dump();
      } else {
        parser_config_override_.clear();
      }
    }
  } else {
    parser_config_override_.clear();
  }
  return PJ::okStatus();
}

void LcmSource::publishRobotState(const std::string& urdf_xml) {
  // 构造 ROS 2 String 消息并进行 CDR 序列化
  auto robot_description_bytes = std::make_shared<std::vector<uint8_t>>(
      serializeRosString(urdf_xml));

  // ROS 2 String 的接口定义
  const std::string topic_name = "/robot_description";
  const std::string type_name = "std_msgs/msg/String";

  // ROS 2 String 的消息定义
  const std::string schema_text = "string data";

  const std::vector<uint8_t> robot_description_schema_bytes(
      schema_text.begin(),
      schema_text.end());

  // 配置 ROS 2 消息解析器
  nlohmann::json robot_description_parser_config;
  robot_description_parser_config["serialization"] = "cdr";
  robot_description_parser_config["schema_encoding"] = "ros2msg";
  robot_description_parser_config["topic_name"] = topic_name;

  PJ::ParserBindingRequest robot_description_request{
      .topic_name = topic_name,
      .parser_encoding = "ros2msg",
      .type_name = type_name,
      .schema = robot_description_schema_bytes,
      .parser_config_json = robot_description_parser_config.dump(),
  };

  // 注册或获取对应的解析器绑定
  auto robot_description_handle =
      runtimeHost().ensureParserBinding(robot_description_request);

  // 将 CDR 数据发送给 PlotJuggler
  (void)runtimeHost().pushMessage(
      *robot_description_handle,
      0,
      [robot_description_bytes]() {
          return PJ::sdk::PayloadView{robot_description_bytes};
      });
}

int LcmSource::decode_type_2(const lcm::LogEvent* event, int offset, const std::string& type,
                             const std::string& base, const std::string& memberName,
                             std::unordered_map<std::string, int>& memberMap,
                             std::map<std::string, lcm_struct_t*>& lcmTypeTable,
                             const std::function<void(const std::string&, PJ::sdk::ValueRef)>& writeCb) {
  int pos_offset = offset;
  int decode_len = 0;
  
  if (type == "int8_t") {
    int8_t value;
    int tlen = __int8_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, value);
    memberMap[memberName] = value;
  } else if (type == "int16_t") {
    int16_t value;
    int tlen = __int16_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, value);
    memberMap[memberName] = value;
  } else if (type == "int32_t") {
    int32_t value;
    int tlen = __int32_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, value);
    memberMap[memberName] = value;
  } else if (type == "int64_t") {
    int64_t value;
    int tlen = __int64_t_decode_array(event->data, pos_offset, event->datalen, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, value);
    memberMap[memberName] = value;
  } else if (type == "float") {
    float value;
    int tlen = __float_decode_array(event->data, pos_offset, event->datalen, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, value);
  } else if (type == "double") {
    double value;
    int tlen = __double_decode_array(event->data, pos_offset, event->datalen, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, value);
  } else if (type == "string") {
    int32_t __d_len__;
    int tlen = __int32_t_decode_array(
        event->data, pos_offset, event->datalen, &__d_len__, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    if(__d_len__ > event->datalen - pos_offset || __d_len__ <= 0) return 0;
    std::string lcm_str(static_cast<const char*>(event->data) + pos_offset,  __d_len__ - 1);
    pos_offset += __d_len__;
    decode_len += __d_len__;

    writeCb(base + memberName, lcm_str);
  } else if (type == "boolean") {
    int8_t value;
    int tlen = __boolean_decode_array(event->data, pos_offset, event->datalen, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, value);
  } else {
    int tlen = decode_type_1(event, pos_offset, type, base + memberName + "/", lcmTypeTable, writeCb);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
  }

  return decode_len;
}

int LcmSource::decode_type_1(const lcm::LogEvent* event, int offset, const std::string& type, 
                             const std::string& base, std::map<std::string, lcm_struct_t*>& lcmTypeTable,
                             const std::function<void(const std::string&, PJ::sdk::ValueRef)>& writeCb) {
  int pos_offset = offset;
  int decode_len = 0;
  std::unordered_map<std::string, int> memberMap;
  
  if (auto lcmStructType = lcmTypeTable.find(type); lcmStructType != lcmTypeTable.end()) {
    lcm_struct_t* structType = lcmStructType->second;
    for (unsigned int mind = 0; mind < g_ptr_array_size(structType->members); mind++) {
      lcm_member_t* member = (lcm_member_t*) g_ptr_array_index(structType->members, mind);

      if (g_ptr_array_size(member->dimensions) == 0) {
        int tlen = decode_type_2(event, pos_offset, member->type->lctypename, 
                                base + member->membername + "/", member->membername,
                                memberMap, lcmTypeTable, writeCb);
        if(tlen < 0) break; else {
          pos_offset += tlen;
          decode_len += tlen;
        }
      } else {
        for (unsigned int d = 0; d < g_ptr_array_size(member->dimensions); d++) {
          lcm_dimension_t *ld =
              (lcm_dimension_t *) g_ptr_array_index(member->dimensions, d);

          if (!lcm_is_constant_size_array(member)) { //判断是否是变长大小数组
            if (lcmStructType->first == "depth_data") {  //判断是否为深度图数据,该数据结构特殊处理
              int height = memberMap.find("height")->second;
              int width = memberMap.find("width")->second;

              std::vector<uint8_t> image_data(height * width);
              for (size_t e = 0; e < height * width; e++) {
                int tlen = decode_type_2(event, pos_offset, member->type->lctypename, "",
                                        member->membername, memberMap, lcmTypeTable,
                                        [&image_data, e] (const std::string&, PJ::sdk::ValueRef value) {
                                          image_data[e] = std::get<float>(value) * 255.0f + 127;
                                        });
                if(tlen < 0) break; else {
                  pos_offset += tlen;
                  decode_len += tlen;
                }
              }

              publishDepthImage(event, width, height, image_data);
              return decode_len;
            }

            if (auto it = memberMap.find(ld->size); it != memberMap.end()) {
              for (size_t i = 0; i < it->second; i++) {
                int tlen = decode_type_2(event, pos_offset, member->type->lctypename, 
                                        base + member->membername + "/" + std::to_string(i) + "/",
                                        member->membername, memberMap, lcmTypeTable, writeCb);
                if(tlen < 0) break; else {
                  pos_offset += tlen;
                  decode_len += tlen;
                }
              }
              continue;;
            }
          }
          
          for (size_t i = 0; i < atoi(ld->size); i++) {
            int tlen = decode_type_2(event, pos_offset, member->type->lctypename, 
                                    base + member->membername + "/" + std::to_string(i),
                                    member->membername, memberMap, lcmTypeTable, writeCb);
            if(tlen < 0) break; else {
              pos_offset += tlen;
              decode_len += tlen;
            }
          }
        }
      }
    }
  }

  return decode_len;
}

void LcmSource::publishDepthImage(const lcm::LogEvent* event, int height, int width, std::vector<uint8_t>& image_data) {
  static PJ::ParserBindingHandle image_handle = PJ::ParserBindingHandle();
  if (image_handle.id == 0) {
    const std::string topic_name = "/camera/image";
    const std::string type_name = "sensor_msgs/msg/Image";

    // ============================================================================
    // ROS2 sensor_msgs/msg/Image schema
    // ============================================================================

    const std::string schema_str = R"(
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data

================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id

================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
)";

    // ============================================================================
    // Parser configuration
    // ============================================================================

    nlohmann::json parser_config;

    parser_config["serialization"] = "cdr";
    parser_config["schema_encoding"] = "ros2msg";
    parser_config["topic_name"] = topic_name;

    const PJ::Span<const uint8_t> schema_bytes{
      reinterpret_cast<const uint8_t*>(
          schema_str.data()),
      schema_str.size()
    };

    // ============================================================================
    // Bind ROS2 parser
    // ============================================================================

    PJ::ParserBindingRequest request{
      .topic_name = topic_name,
      .parser_encoding = "ros2msg",
      .type_name = type_name,
      .schema = schema_bytes,
      .parser_config_json = parser_config.dump(),
    };

    auto image_handle_result =
      runtimeHost().ensureParserBinding(request);
    image_handle = *image_handle_result;
  // if (!image_handle)
  // {
  // return PJ::unexpected(
  //     std::string(
  //         "Failed to bind parser for Image: ") +
  //     image_handle.error());
  // }
  }

  CdrWriter cdr;
  // Header.stamp.sec
  cdr.writeInt32(static_cast<int32_t>(event->timestamp / 1000000));
  // Header.stamp.nanosec
  cdr.writeUint32(event->timestamp * 1000);
  // Header.frame_id
  cdr.writeString("camera_frame");
  // height
  cdr.writeUint32(height);
  // width
  cdr.writeUint32(width);
  // encoding
  cdr.writeString("mono8");
  // is_bigendian
  cdr.writeUint8(0);
  // step
  cdr.writeUint32(width);
  // data
  cdr.writeByteSequence(image_data);

  auto bytes = std::make_shared<std::vector<uint8_t>>(std::move(cdr.data()));
  (void)runtimeHost().pushMessage(image_handle, event->timestamp * 1000,
                           [bytes]() {return PJ::sdk::PayloadView{bytes};});
}


void LcmSource::publishTF(const lcm::LogEvent* event,
                          const std::vector<geometry_msgs::msg::TransformStamped>& tf_transforms) {
  static PJ::ParserBindingHandle tf_handle = PJ::ParserBindingHandle();
  if (tf_handle.id == 0) {
    // 处理 tf_transforms
    static constexpr char TRANSFORM_STAMPED_SCHEMA[] = R"SCHEMA(
std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Transform
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
)SCHEMA";

    const auto* schema_data = reinterpret_cast<const uint8_t*>(TRANSFORM_STAMPED_SCHEMA);
    const size_t schema_size = sizeof(TRANSFORM_STAMPED_SCHEMA) - 1;

    // 配置 ROS 2 消息解析器
    nlohmann::json robot_description_parser_config;
    robot_description_parser_config["serialization"] = "cdr";
    robot_description_parser_config["schema_encoding"] = "ros2msg";
    robot_description_parser_config["topic_name"] = "/tf";
    
    PJ::ParserBindingRequest tf_request{
        .topic_name = "/tf",
        .parser_encoding = "ros2msg",
        .type_name = "geometry_msgs/msg/TransformStamped",
        .schema = PJ::Span<const uint8_t>(schema_data, schema_size),
        .parser_config_json = robot_description_parser_config.dump(),
    };

    // 注册或获取对应的解析器绑定
    auto tf_result = runtimeHost().ensureParserBinding(tf_request);
    tf_handle = *tf_result;
  }

  for (auto transform : tf_transforms) {
    transform.header.stamp.sec = event->timestamp / 1000000;
    transform.header.stamp.nanosec = event->timestamp * 1000;
    auto tf_bytes = std::make_shared<std::vector<uint8_t>>(serializeTransform(transform));
    // 5. 将消息载荷交给 PlotJuggler
    (void)runtimeHost().pushMessage(
        tf_handle,
        event->timestamp * 1000,
        [tf_bytes]() mutable
        {
          return PJ::sdk::PayloadView{tf_bytes};
        });
  }
}

void LcmSource::publishJointState(const lcm::LogEvent* event,
                                  const sensor_msgs::msg::JointState& joint_state) {
  static PJ::ParserBindingHandle joint_state_handle = PJ::ParserBindingHandle();
  if (joint_state_handle.id == 0) {
    // 处理 joint_state
    static constexpr char JOINT_STATE_SCHEMA[] = R"SCHEMA(
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
)SCHEMA";

    const auto* schema_data = reinterpret_cast<const uint8_t*>(JOINT_STATE_SCHEMA);
    const size_t schema_size = sizeof(JOINT_STATE_SCHEMA) - 1;


    // 配置 ROS 2 消息解析器
    nlohmann::json robot_description_parser_config;
    robot_description_parser_config["serialization"] = "cdr";
    robot_description_parser_config["schema_encoding"] = "ros2msg";
    robot_description_parser_config["topic_name"] = "/joint_states";
    
    PJ::ParserBindingRequest joint_state_request{
        .topic_name = "/joint_states",
        .parser_encoding = "ros2msg",
        .type_name = "sensor_msgs/msg/JointState",
        .schema = PJ::Span<const uint8_t>(schema_data, schema_size),
        .parser_config_json = robot_description_parser_config.dump(),
    };

    // 注册或获取对应的解析器绑定
    auto joint_state_result = runtimeHost().ensureParserBinding(joint_state_request);
    joint_state_handle = *joint_state_result;
  }

  auto joint_state_tmp = joint_state;
  joint_state_tmp.header.stamp.sec = event->timestamp / 1000000;
  joint_state_tmp.header.stamp.nanosec = event->timestamp * 1000;
  auto joint_state_bytes = std::make_shared<std::vector<uint8_t>>(serializeJointState(joint_state_tmp));
  // 5. 将消息载荷交给 PlotJuggler
  (void)runtimeHost().pushMessage(
      joint_state_handle,
      event->timestamp * 1000,
      [joint_state_bytes]() mutable
      {
        return PJ::sdk::PayloadView{joint_state_bytes};
      });
}

void LcmSource::savePoint(const lcm::LogEvent* event, const std::string& topic_name,
                          PJ::sdk::ValueRef value) {
  auto topic_handle = writeHost().ensureTopic(topic_name);
                            
  const PJ::sdk::NamedFieldValue fields[] = {
      {.name = "value", .value = value}
  };

  // 2. 按正确顺序调用
  (void)writeHost().appendRecord(
      *topic_handle,                                              // TopicHandle
      PJ::Timestamp{event->timestamp * 1000},                   // Timestamp
      PJ::Span<const PJ::sdk::NamedFieldValue>(fields, 1)       // Span
  );

  if (publishTfCount < 34) {
    for (int leg_idx = 0; leg_idx < 12; leg_idx++) {
      std::string str_q = std::string("leg_control_data/q/") + std::to_string(leg_idx);
      if (str_q == topic_name) {
        robot_state_publisher_.setLegPos(leg_idx, std::get<float>(value));
        publishTfCount++;
        return;
      }
    }

    for (int arm_idx = 0; arm_idx < 14; arm_idx++) {
      std::string str_q = std::string("arm_control_data/joint_curPos/") + std::to_string(arm_idx);
      if (str_q == topic_name) {
        robot_state_publisher_.setArmPos(arm_idx, std::get<float>(value));
        publishTfCount++;
        return;
      }
    }

    for (int waist_idx = 0; waist_idx < 3; waist_idx++) {
      std::string str_q = std::string("waist_control_data/q/") + std::to_string(waist_idx);
      if (str_q == topic_name) {
        robot_state_publisher_.setWaistPos(waist_idx, std::get<float>(value));
        publishTfCount++;
        return;
      }
    }

    for (int quat_idx = 0; quat_idx < 4; quat_idx++) {
      std::string str_q = std::string("state_estimator/quat/") + std::to_string(quat_idx);
      if (str_q == topic_name) {
        robot_state_publisher_.setStateEstimator(quat_idx, std::get<float>(value));
        publishTfCount++;
        return;
      }
    }
  }
}

PJ::Status LcmSource::importData() {
  robot_state_publisher_.init(dialog_.urdfpath(), dialog_.xmlpath(),
                              [this] (const std::string& urdf_xml) {
                                  publishRobotState(urdf_xml);
                              });

  if (dialog_.filepath().empty()) {
    return PJ::unexpected(std::string("no filepath configured"));
  }

  auto topic_config = dialog_.getTopicConfig();
  for (const auto& tc : topic_config) {
    std::cout << "topic_name: " << tc.topic_name << std::endl;
  }

  typedef struct TopicLcmItem {
    int64_t hash = 0;
    std::vector<lcmgen_t*> lcmgen_table = {};
    std::string lcmFilePath = "";
    std::optional<PJ::sdk::TopicHandle> topic_handle;
  } TopicLcmItem;
  std::unordered_map<std::string, TopicLcmItem> topic_map;
  std::map<std::string, lcm_struct_t*> lcmTypeTable;
  lcmTypeTable.clear();
  lcmTypeTable["int8_t"] = nullptr;
  lcmTypeTable["int16_t"] = nullptr;
  lcmTypeTable["int32_t"] = nullptr;
  lcmTypeTable["int64_t"] = nullptr;
  lcmTypeTable["float"] = nullptr;
  lcmTypeTable["double"] = nullptr;
  lcmTypeTable["string"] = nullptr;
  lcmTypeTable["boolean"] = nullptr;

  for (const auto& item : topic_config) {
    if (item.lcm_file_path.empty()) continue;

    // 检查文件是否存在
    std::ifstream file_check(item.lcm_file_path);
    if (!file_check.good()) {
      // QMessageBox::warning(nullptr, tr("Error"), "文件不存在: " + item.lcmFilePath + "，跳过加载。");
      // std::cerr << "Warning: File does not exist: " << item.lcm_file_path << std::endl;
      return PJ::unexpected("");
    }

    lcmgen_t* lcmgen = lcmgen_create();
    tokenize_t* t = tokenize_create(item.lcm_file_path.c_str());
    int res;
    do {
        res = parse_entity(lcmgen, item.lcm_file_path.c_str(), t);
        if (res == -10) {
        //  QMessageBox::warning(nullptr, tr("Error"), "文件  " + item.lcmFilePath + "格式错误. 退出加载...........");
          return PJ::unexpected("");
        }
    } while (res == 0);

    for (int struct_idx = 0; struct_idx < lcmgen->structs->len; struct_idx++) {
      lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(lcmgen->structs, struct_idx);
      lcmTypeTable[struct_item->structname->lctypename] = struct_item;
      std::cout << "struct_item->structname->lctypename: " << struct_item->structname->lctypename << std::endl;
    }

    int64_t hash = 0;
    if (lcmgen->structs->len > 0) {
        lcm_struct_t* last_item = (lcm_struct_t*)g_ptr_array_index(lcmgen->structs, lcmgen->structs->len - 1);
        hash = last_item->hash;
    }

    tokenize_destroy(t);

    if (auto it = topic_map.find(item.topic_name); it == topic_map.end()) {
      topic_map[item.topic_name] = {hash, {lcmgen}, item.lcm_file_path};
    } else {
      it->second.lcmgen_table.push_back(lcmgen);
    }

    std::cout << "item.topic: " << item.topic_name << std::endl;
    std::cout << "item.lcmFilePath: " << item.lcm_file_path << std::endl;
  }

  using namespace std;
  for (auto& pair : topic_map) {
    for (auto lcmgen : pair.second.lcmgen_table) {
      for (int struct_idx = 0; struct_idx < lcmgen->structs->len; struct_idx++) {
        lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(lcmgen->structs, struct_idx);
        if (g_ptr_array_size(struct_item->members)) {
          for (unsigned int mind = 0; mind < g_ptr_array_size(struct_item->members); mind++) {
            lcm_member_t *member = (lcm_member_t *) g_ptr_array_index(struct_item->members, mind);
            if (auto it = lcmTypeTable.find(member->type->lctypename); it != lcmTypeTable.end()) {
              if (it->second) {
                std::cout << "member->type->lctypename: " << member->type->lctypename << std::endl;
                uint64_t hash = struct_item->hash + (it->second->hash<<1) + ((it->second->hash>>63)&1);
                struct_item->hash = hash;
                pair.second.hash = hash;
              }
            } else {
          //   QMessageBox::warning(nullptr, tr("Error"), "类型  " + QString::fromStdString(member->type->lctypename) + " 未定义. 退出加载...........");
              return PJ::unexpected("");
            }
          }
        }
      }
    }
  }
  std::cout << "open lcm log file: " << dialog_.filepath() << std::endl;
  // 3. 打开二进制 LCM 日志文件
  lcm::LogFile lcm_log(dialog_.filepath(), "r");
  if (!lcm_log.good()) {
      std::cerr << "错误：无法打开或读取 LCM 日志文件: " << dialog_.filepath() << std::endl;
      return PJ::unexpected("");
  }

  auto get_file_size = [] (FILE* fp) -> long long {
    if (!fp) return -1;
    int fd = fileno(fp);
    struct stat st;
    if (fstat(fd, &st) != 0) return -1;
    return st.st_size;
  };

  long long max_size = get_file_size(lcm_log.getFilePtr());
  (void)runtimeHost().progressStart("Importing LCM", max_size, true);
  // 4. 核心快转主循环
  int64_t last_TF_timestamp = 0;

  while (true) {
    const lcm::LogEvent* event = lcm_log.readNextEvent();
    // std::cout << "event->channel: " << event->channel << std::endl;

    if (!event) break; // 读完整个 log 文件，平稳退出
    //每100ms发布一次TF
    if (event->timestamp - last_TF_timestamp > 100000) {
      (void)runtimeHost().progressUpdate(ftell(lcm_log.getFilePtr()));
      last_TF_timestamp = event->timestamp;
      robot_state_publisher_.publishTransforms(
        [this, event] (const std::vector<geometry_msgs::msg::TransformStamped>& tf_transforms) {
          publishTF(event, tf_transforms);}, 
        [this, event] (const sensor_msgs::msg::JointState& joint_state) {
          publishJointState(event, joint_state);
        });
      publishTfCount = 0;
    }
    
    if (auto it = topic_map.find(event->channel); it != topic_map.end()) {
      if (!it->second.topic_handle.has_value()) {
        auto topic_result = writeHost().ensureTopic(event->channel);
        if (!topic_result) {
          return PJ::unexpected(topic_result.error());
        }
        it->second.topic_handle = *topic_result;
      }
      bool channel_match = false;
      for (auto topic_lcmgen : it->second.lcmgen_table) {
        int64_t msg_hash;
        int thislen = __int64_t_decode_array(event->data, 0, event->datalen, &msg_hash, 1);
        for (int struct_idx = 0; struct_idx < topic_lcmgen->structs->len; struct_idx++) {
          lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(topic_lcmgen->structs, struct_idx);
          if (msg_hash == ((struct_item->hash<<1) + ((struct_item->hash>>63)&1))) {
            decode_type_1(event, 8, struct_item->structname->lctypename, event->channel + "/", lcmTypeTable,
                          [event, this] (const std::string& topic_name, PJ::sdk::ValueRef value) {
                            savePoint(event, topic_name, value);
                          });
            channel_match = true;
            break;
          }
        }
      }

      if (!channel_match) {
        std::cerr << "Warning: Hash mismatch for channel " << event->channel << ". Skipping event." << std::endl;
      //  QMessageBox::warning(nullptr, tr("Error"), "话题：" + QString::fromStdString(event->channel) + "类型不匹配，退出。。。。。。");
        return PJ::unexpected("");
      }
    }
  }


  runtimeHost().progressFinish();
  
  return PJ::okStatus();
}


PJ_DATA_SOURCE_PLUGIN(LcmSource, kMcapManifest)

PJ_DIALOG_PLUGIN(LcmDialog, kMcapManifest)