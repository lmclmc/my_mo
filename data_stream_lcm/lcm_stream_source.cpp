#include "lcm_stream_source.hpp"

#include "lidar_subscriber.hpp"

static std::vector<uint8_t> serializeVisualization(
    visualization_msgs::msg::Marker& msg) {
    rclcpp::Serialization<visualization_msgs::msg::Marker> serializer;

    rclcpp::SerializedMessage serialized_msg;

    serializer.serialize_message(&msg, &serialized_msg);

    const auto& raw =
        serialized_msg.get_rcl_serialized_message();

    return std::vector<uint8_t>(
        raw.buffer,
        raw.buffer + raw.buffer_length);
}

// ------------------------------------------------------------
// 3. visualization_msgs::msg::Marker 辅助常量与工具
// ------------------------------------------------------------

static constexpr int32_t MARKER_TRIANGLE_LIST = 11;
static constexpr int32_t MARKER_ACTION_ADD = 0;

static std_msgs::msg::ColorRGBA makeColor(
    float r, float g, float b, float a = 1.0f) {
  std_msgs::msg::ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = a;
  return c;
}

static geometry_msgs::msg::Point makePoint(double x, double y, double z) {
  geometry_msgs::msg::Point p;
  p.x = x; p.y = y; p.z = z;
  return p;
}

static geometry_msgs::msg::Vector3 makeVector3(double x, double y, double z) {
  geometry_msgs::msg::Vector3 v;
  v.x = x; v.y = y; v.z = z;
  return v;
}


// 假设你已有以下辅助函数：
// makePoint(x, y, z)
// makeColor(r, g, b, a)
// makeVector3(x, y, z)

static visualization_msgs::msg::Marker makeElevationErrorBarMap(
    const std::string& frame_id,
    const std::string& ns,
    int32_t id,
    int rows,
    int cols,
    const std::vector<float>& height_data,
    const std::vector<float>& error_data,
    double resolution,
    double origin_x,
    double origin_y,
    double z_scale,
    float min_z,
    float max_z,
    double error_cap_width = 0.05,
    double error_line_width = 0.5,
    int error_stride = 1)
{
    auto idx = [&](int r, int c) -> size_t
    {
        return static_cast<size_t>(r) * cols + c;
    };

    // ========================================================
    // 误差棒 LINE_LIST
    // ========================================================

    visualization_msgs::msg::Marker error_marker;

    error_marker.header.frame_id = frame_id;

    error_marker.ns = ns + "_error_bars";
    error_marker.id = id;

    error_marker.type =
        visualization_msgs::msg::Marker::LINE_LIST;

    error_marker.action =
        visualization_msgs::msg::Marker::ADD;

    error_marker.pose.orientation.w = 1.0;

    // LINE_LIST 只使用 scale.x 表示线宽
    error_marker.scale.x = error_line_width;

    // 亮红色误差棒，半透明，便于在高程图上叠加显示
    error_marker.color.r = 1.0f;
    error_marker.color.g = 0.0f;
    error_marker.color.b = 0.0f;
    error_marker.color.a = 0.9f;

    // 误差棒 z 轴整体向上偏移，避免被高程表面遮挡
    const double z_offset = 0.05;

    error_marker.lifetime.sec = 0;
    error_marker.lifetime.nanosec = 0;

    auto addLine = [&](double x1, double y1, double z1,
                       double x2, double y2, double z2)
    {
        error_marker.points.push_back(
            makePoint(x1, y1, z1));

        error_marker.points.push_back(
            makePoint(x2, y2, z2));
    };

    // ========================================================
    // 4. 每个网格点添加误差棒
    // ========================================================

    for (int r = 0; r < rows; r += error_stride)
    {
        for (int c = 0; c < cols; c += error_stride)
        {
            const size_t i = idx(r, c);

            const double h = height_data[i];
            const double e = error_data[i];

            if (!std::isfinite(h) ||
                !std::isfinite(e) ||
                e < 0.0)
            {
                continue;
            }

            const double x =
                origin_x + c * resolution;

            const double y =
                origin_y + r * resolution;

            // 高度与误差使用相同的缩放比例
            const double z = h * z_scale;

            const double error =
                e * std::abs(z_scale);

            const double z_min = z - error;
            const double z_max = z + error;

            const double half_cap =
                error_cap_width * 0.5;

            // 1. 误差棒主体（z 轴偏移避免被表面遮挡）
            addLine(
                x, y, z_min + z_offset,
                x, y, z_max + z_offset
            );

            // 2. 顶部横线
            addLine(
                x - half_cap, y, z_max + z_offset,
                x + half_cap, y, z_max + z_offset
            );

            // 3. 底部横线
            addLine(
                x - half_cap, y, z_min + z_offset,
                x + half_cap, y, z_min + z_offset
            );
        }
    }

    return error_marker;
}

// 4. 高程图：2D 高度数组 -> TRIANGLE_LIST Marker
// ------------------------------------------------------------
static visualization_msgs::msg::Marker makeElevationMap(
    const std::string& frame_id,
    const std::string& ns,
    int32_t id,
    int rows, int cols,
    const std::vector<float>& height_data,
    double resolution,
    double origin_x, double origin_y,
    double z_scale,
    float min_z, float max_z) {
 
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = 11;
  marker.action = MARKER_ACTION_ADD;
  marker.scale = makeVector3(1.0, 1.0, 1.0);
  marker.pose.orientation.w = 1.0;
  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;
 
  const size_t n = static_cast<size_t>(rows) * cols;
  if (height_data.size() < n || rows < 2 || cols < 2) {
    return marker;
  }
 
  const int tri_count = (rows - 1) * (cols - 1) * 2;
  marker.points.reserve(tri_count * 3);
  marker.colors.reserve(tri_count * 3);
 
  const float z_range = (max_z > min_z) ? (max_z - min_z) : 1.0f;
 
  auto heightToColor = [&](double h) {
    float t = (static_cast<float>(h) - min_z) / z_range;
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    float r, g, b;
    if (t < 0.25f) {
      float s = t / 0.25f;
      r = 0.0f;  g = s;     b = 1.0f;
    } else if (t < 0.5f) {
      float s = (t - 0.25f) / 0.25f;
      r = 0.0f;  g = 1.0f;  b = 1.0f - s;
    } else if (t < 0.75f) {
      float s = (t - 0.5f) / 0.25f;
      r = s;     g = 1.0f;  b = 0.0f;
    } else {
      float s = (t - 0.75f) / 0.25f;
      r = 1.0f;  g = 1.0f - s;  b = 0.0f;
    }
    return makeColor(r, g, b, 0.6f);
  };
 
  for (int r = 0; r < rows - 1; ++r) {
    for (int c = 0; c < cols - 1; ++c) {
      auto idx = [&](int rr, int cc) -> size_t {
        return static_cast<size_t>(rr) * cols + static_cast<size_t>(cc);
      };
      size_t i00 = idx(r,   c);
      size_t i10 = idx(r+1, c);
      size_t i01 = idx(r,   c+1);
      size_t i11 = idx(r+1, c+1);
 
      double x0 = origin_x + c * resolution;
      double y0 = origin_y + r * resolution;
      double x1 = x0 + resolution;
      double y1 = y0 + resolution;
 
      double z00 = height_data[i00] * z_scale;
      double z10 = height_data[i10] * z_scale;
      double z01 = height_data[i01] * z_scale;
      double z11 = height_data[i11] * z_scale;
 
      auto p00 = makePoint(x0, y0, z00);
      auto p10 = makePoint(x0, y1, z10);
      auto p01 = makePoint(x1, y0, z01);
      auto p11 = makePoint(x1, y1, z11);
 
      auto c00 = heightToColor(z00);
      auto c10 = heightToColor(z10);
      auto c01 = heightToColor(z01);
      auto c11 = heightToColor(z11);
 
      // 三角形 1: (r,c) - (r+1,c) - (r,c+1)
      marker.points.push_back(p00);
      marker.points.push_back(p10);
      marker.points.push_back(p01);
      marker.colors.push_back(c00);
      marker.colors.push_back(c10);
      marker.colors.push_back(c01);
 
      // 三角形 2: (r+1,c+1) - (r,c+1) - (r+1,c)
      marker.points.push_back(p11);
      marker.points.push_back(p01);
      marker.points.push_back(p10);
      marker.colors.push_back(c11);
      marker.colors.push_back(c01);
      marker.colors.push_back(c10);
    }
  }
 
  return marker;
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

static std::vector<uint8_t> serializeTFMessage(
    const std::vector<geometry_msgs::msg::TransformStamped>& transforms)
{
    tf2_msgs::msg::TFMessage msg;
    msg.transforms = transforms;

    rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&msg, &serialized_msg);

    const auto& raw = serialized_msg.get_rcl_serialized_message();
    return std::vector<uint8_t>(raw.buffer, raw.buffer + raw.buffer_length);
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

PJ::Status LcmStreamSource::loadConfig(std::string_view config_json) {
  if (!dialog_.loadConfig(config_json)) {
    runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kError, std::string("Warning: Invalid config JSON"));
    runtimeHost().showError("LCM import failed", std::string("Warning: Invalid config JSON"));
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

LcmStreamSource::LcmStreamSource() {
  m_image_handle_ = PJ::ParserBindingHandle();
  m_tf_handle_ = PJ::ParserBindingHandle();
  m_marker_handle_ = PJ::ParserBindingHandle();
  m_marker_error_bar_handle_ = PJ::ParserBindingHandle();
}

void LcmStreamSource::handleMessage(const lcm::ReceiveBuffer* recvbuf, const std::string& topic_name) {
  if (recvbuf->data_size < 8) return; // 所有的 LCM 消息头至少 8 字节

  if (lcm_log)
    lcm_log->write(topic_name, recvbuf);

  struct timeval tv;
  gettimeofday(&tv, NULL);
  LcmMessage msg;
  msg.channel = topic_name;
  msg.timestamp = recvbuf->recv_utime;
  msg.data.assign(reinterpret_cast<const uint8_t*>(recvbuf->data), reinterpret_cast<const uint8_t*>(recvbuf->data) + recvbuf->data_size);
  message_queue.push(msg);
}

PJ::Status LcmStreamSource::onStart() {
  m_image_handle_ = PJ::ParserBindingHandle();
  m_tf_handle_ = PJ::ParserBindingHandle();
  m_marker_handle_ = PJ::ParserBindingHandle();
  m_marker_error_bar_handle_ = PJ::ParserBindingHandle();
  //   rclcpp::init(0, nullptr);

  // auto node = std::make_shared<LidarSubscriber>();

  // std::thread([node] () {
  //   rclcpp::spin(node);
  // }).detach();

  robot_state_publisher_.init(dialog_.urdfpath(), dialog_.xmlpath(),
                              [this] (const std::string& urdf_xml) {
                                  publishRobotState(urdf_xml);
                              });

  topic_map.clear();
  lcm_table.clear();
  auto _config = dialog_.getTopicConfig();

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
    // 检查文件是否存在
    std::ifstream file_check(item.lcm_file_path);
    if (!file_check.good()) {
      // QMessageBox::warning(nullptr, tr("Error"), "文件不存在: " + item.lcmFilePath + "，跳过加载。");
      // std::cerr << "Warning: File does not exist: " << item.lcm_file_path << std::endl;
      runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kError, std::string("Warning: File does not exist: ") + item.lcm_file_path);
      runtimeHost().showError("LCM import failed", std::string("Warning: File does not exist: ") + item.lcm_file_path);
      return PJ::unexpected(std::string("Warning: File does not exist: ") + item.lcm_file_path);
    }

    lcmgen_t* lcmgen = lcmgen_create();
    tokenize_t* t = tokenize_create(item.lcm_file_path.c_str());
    int res;
    do {
        res = parse_entity(lcmgen, item.lcm_file_path.c_str(), t);
        if (res == -10) {
//          QMessageBox::warning(nullptr, tr("Error"), "文件  " + item.lcm_file_path + "格式错误. 退出加载...........");
          runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kError, std::string("Warning: File format error: ") + item.lcm_file_path);
          runtimeHost().showError("LCM import failed", std::string("Warning: File format error: ") + item.lcm_file_path);
          return PJ::unexpected(std::string("Warning: File format error: ") + item.lcm_file_path);
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

    if (auto it = topic_map.find(item.topic_name); it == topic_map.end()) {
      topic_map[item.topic_name] = {hash, {lcmgen}, item.lcm_file_path};
    } else {
      it->second.lcmgen_table.push_back(lcmgen);
    }

    std::cout << "item.topic: " << item.topic_name << std::endl;
    std::cout << "item.lcm_file_path: " << item.lcm_file_path << std::endl;
    std::cout << "item.port: " << item.port << std::endl;

    if (auto l = lcm_table.find(std::stoi(item.port)); l == lcm_table.end()) {
      std::string lcm_url = std::string("udpm://239.255.76.67:") + item.port;
      auto lcm_ptr = std::make_shared<lcm::LCM>(lcm_url); // 使用全局变量
      lcm_ptr->subscribe(".*", &LcmStreamSource::handleMessage, this);
      lcm_table[std::stoi(item.port)] = lcm_ptr;
    }
  }

  log_save_path = dialog_.log_save_path();
  log_max_size = dialog_.log_max_size();
  log_max_files_num = dialog_.log_max_files_num();

  if (!log_save_path.empty()) {
    lcm_log = std::make_shared<rotating_file_sink>(log_save_path, log_max_size, log_max_files_num);
  }

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
        //      QMessageBox::warning(nullptr, tr("Error"), "类型  " + QString::fromStdString(member->type->lctypename) + " 未定义. 退出加载...........");
               runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kError, std::string("Warning: Type not defined: ") + member->type->lctypename);
               runtimeHost().showError("LCM import failed", std::string("Warning: Type not defined: ") + member->type->lctypename);
               return PJ::unexpected(std::string("Warning: Type not defined: ") + member->type->lctypename);
            }
          }
        }
      }
    }
  }

  _running = true;
  lcm_runner_ = std::thread([this] {
    struct timeval timeout;
    timeout.tv_sec  = 0;
    timeout.tv_usec = 0;

    while (_running) {
      //提高select的超时等待时间，优化数据实时性的抖动率，从4%降低到3.2%，并解决cpu占用过高的问题
      timeout.tv_usec = 300 * 1000;  // 300ms = 300000us
      fd_set fds;
      FD_ZERO(&fds);
      int max_fd = 0;
      for (auto [port, lcm_ptr] : lcm_table) {
        int fd = lcm_ptr->getFileno();
        FD_SET(fd, &fds);
        if (fd > max_fd) {
          max_fd = fd;
        }
      }

      int status = select(max_fd + 1, &fds, 0, 0, &timeout);
      if (status) {
        for (auto [port, lcm_ptr] : lcm_table) {
          int fd = lcm_ptr->getFileno();
          if (FD_ISSET(fd, &fds)) {
            if (lcm_ptr) {
              lcm_ptr->handle();
            }
          }
        }
      }
    }
  });

  // render_runner_ = std::thread([this] {
  //   while (_running) {
  //     //添加延时，避免cpu空转，占用过多的cpu资源
  //     usleep(1);
      
  //   }
  // });

  return PJ::okStatus();
}

void LcmStreamSource::publishRobotState(const std::string& urdf_xml) {
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

void LcmStreamSource::publishDepthImage(void* recvbuf, int data_size, int64_t timestamp, int height, int width, std::vector<uint8_t>& image_data) {
  if (m_image_handle_.id == 0) {
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
    m_image_handle_ = *image_handle_result;
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
  cdr.writeInt32(static_cast<int32_t>(timestamp / 1000000));
  // Header.stamp.nanosec
  cdr.writeUint32(timestamp * 1000);
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
  (void)runtimeHost().pushMessage(m_image_handle_, timestamp * 1000,
                           [bytes]() {return PJ::sdk::PayloadView{bytes};});
}

void LcmStreamSource::publishTF(void* recvbuf, int data_size, int64_t timestamp, const std::vector<geometry_msgs::msg::TransformStamped>& tf_transforms) {
  if (tf_transforms.empty()) return;

  if (m_tf_handle_.id == 0) {
    // 处理 tf_transforms
    static constexpr char TF_MESSAGE_SCHEMA[] = R"SCHEMA(
geometry_msgs/TransformStamped[] transforms
================================================================================
MSG: geometry_msgs/TransformStamped
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

    const auto* schema_data = reinterpret_cast<const uint8_t*>(TF_MESSAGE_SCHEMA);
    const size_t schema_size = sizeof(TF_MESSAGE_SCHEMA) - 1;

    nlohmann::json parser_config;
    parser_config["serialization"] = "cdr";
    parser_config["schema_encoding"] = "ros2msg";
    parser_config["topic_name"] = "/tf";

    PJ::ParserBindingRequest tf_request{
        .topic_name = "/tf",
        .parser_encoding = "ros2msg",
        .type_name = "tf2_msgs/msg/TFMessage",
        .schema = PJ::Span<const uint8_t>(schema_data, schema_size),
        .parser_config_json = parser_config.dump(),
    };

    auto tf_result = runtimeHost().ensureParserBinding(tf_request);
    m_tf_handle_ = *tf_result;
  }

  std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms = tf_transforms;
  for (auto& transform : stamped_transforms) {
    transform.header.stamp.sec = timestamp / 1000000;
    transform.header.stamp.nanosec = timestamp * 1000;
  }

  auto tf_bytes = std::make_shared<std::vector<uint8_t>>(serializeTFMessage(stamped_transforms));
  (void)runtimeHost().pushMessage(
      m_tf_handle_,
      timestamp * 1000,
      [tf_bytes]() mutable {
        return PJ::sdk::PayloadView{tf_bytes};
      });
}

void LcmStreamSource::publishJointState(void* recvbuf, int data_size, int64_t timestamp,
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
  joint_state_tmp.header.stamp.sec = timestamp / 1000000;
  joint_state_tmp.header.stamp.nanosec = timestamp * 1000;
  auto joint_state_bytes = std::make_shared<std::vector<uint8_t>>(serializeJointState(joint_state_tmp));
  // 5. 将消息载荷交给 PlotJuggler
  (void)runtimeHost().pushMessage(
      joint_state_handle,
      timestamp * 1000,
      [joint_state_bytes]() mutable
      {
        return PJ::sdk::PayloadView{joint_state_bytes};
      });
}

int LcmStreamSource::decode_type_2(void* recvbuf, int data_size, int64_t timestamp, int offset, const std::string& type,
                             const std::string& base, const std::string& memberName,
                             std::unordered_map<std::string, int>& memberMap,
                             std::map<std::string, lcm_struct_t*>& lcmTypeTable,
                             const WriteCB& writeCb) {
  int pos_offset = offset;
  int decode_len = 0;
  
  if (type == "int8_t") {
    int8_t value;
    int tlen = __int8_t_decode_array(recvbuf, pos_offset, data_size, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, memberName, value);
    memberMap[memberName] = value;
  } else if (type == "int16_t") {
    int16_t value;
    int tlen = __int16_t_decode_array(recvbuf, pos_offset, data_size, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, memberName, value);
    memberMap[memberName] = value;
  } else if (type == "int32_t") {
    int32_t value;
    int tlen = __int32_t_decode_array(recvbuf, pos_offset, data_size, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, memberName, value);
    memberMap[memberName] = value;
  } else if (type == "int64_t") {
    int64_t value;
    int tlen = __int64_t_decode_array(recvbuf, pos_offset, data_size, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, memberName, value);
    memberMap[memberName] = value;
  } else if (type == "float") {
    float value;
    int tlen = __float_decode_array(recvbuf, pos_offset, data_size, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, memberName, value);
  } else if (type == "double") {
    double value;
    int tlen = __double_decode_array(recvbuf, pos_offset, data_size, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, memberName, value);
  } else if (type == "string") {
    int32_t __d_len__;
    int tlen = __int32_t_decode_array(recvbuf, pos_offset, data_size, &__d_len__, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    if(__d_len__ > data_size - pos_offset || __d_len__ <= 0) return 0;
    std::string lcm_str(static_cast<const char*>(recvbuf) + pos_offset,  __d_len__ - 1);
    pos_offset += __d_len__;
    decode_len += __d_len__;

    writeCb(base, memberName, lcm_str);
  } else if (type == "boolean") {
    int8_t value;
    int tlen = __boolean_decode_array(recvbuf, pos_offset, data_size, &value, 1);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
    writeCb(base, memberName, value);
  } else {
    int tlen = decode_type_1(recvbuf, data_size, timestamp, pos_offset, type, base + memberName + "/", lcmTypeTable, writeCb);
    if(tlen > 0) {
      pos_offset += tlen;
      decode_len += tlen;
    }
  }

  return decode_len;
}

int LcmStreamSource::decode_type_1(void* recvbuf, int data_size, int64_t timestamp, int offset, const std::string& type, 
                             const std::string& base, std::map<std::string, lcm_struct_t*>& lcmTypeTable,
                             const WriteCB& writeCb) {
  int pos_offset = offset;
  int decode_len = 0;
  std::unordered_map<std::string, int> memberMap;
  if (auto lcmStructType = lcmTypeTable.find(type); lcmStructType != lcmTypeTable.end()) {
    lcm_struct_t* structType = lcmStructType->second;
    for (unsigned int mind = 0; mind < g_ptr_array_size(structType->members); mind++) {
      lcm_member_t* member = (lcm_member_t*) g_ptr_array_index(structType->members, mind);

      if (g_ptr_array_size(member->dimensions) == 0) {
        int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, 
                                base + "/", member->membername,
                                memberMap, lcmTypeTable, writeCb);
        if(tlen < 0) break; else {
          pos_offset += tlen;
          decode_len += tlen;
        }
      } else {
        //多维数组走该分支
        for (unsigned int d = 0; d < g_ptr_array_size(member->dimensions); d++) {
          lcm_dimension_t *ld =
              (lcm_dimension_t *) g_ptr_array_index(member->dimensions, d);

          if (!lcm_is_constant_size_array(member)) { //判断是否是变长大小数组
            if (lcmStructType->first == "depth_data") {  //判断是否为深度图数据,该数据结构特殊处理
              int height = memberMap.find("height")->second;
              int width = memberMap.find("width")->second;

              std::vector<uint8_t> image_data(height * width);
              for (size_t e = 0; e < height * width; e++) {
                int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, "",
                                        member->membername, memberMap, lcmTypeTable,
                                        [&image_data, e] (const std::string&, const std::string&,
                                                          PJ::sdk::ValueRef value) {
                                          image_data[e] = std::get<float>(value) * 255.0f + 127;
                                        });
                if(tlen < 0) break; else {
                  pos_offset += tlen;
                  decode_len += tlen;
                }
              }

              publishDepthImage(recvbuf, data_size, timestamp, width, height, image_data);
              return decode_len;
            }

            if (lcmStructType->first == "neural_mapping_lcm.neural_map_query_t") {
              int nx = 0;
              int ny = 0;
              if (auto it = memberMap.find("nx"); it != memberMap.end()) {
                nx = it->second;
              }
              if (auto it = memberMap.find("ny"); it != memberMap.end()) {
                ny = it->second;
              }
              if (nx <= 0 || ny <= 0) break;

              float center_x = 0.0f;
              float center_y = 0.0f;
              float resolution = 0.05f;
              if (auto it = memberMap.find("resolution"); it != memberMap.end()) {
                resolution = it->second;
              }
              if (auto it = memberMap.find("center_x"); it != memberMap.end()) {
                center_x = it->second;
              }
              if (auto it = memberMap.find("center_y"); it != memberMap.end()) {
                center_y = it->second;
              }

              if (!strcmp(member->membername, "height")) {
                height_data.resize(nx * ny);
                for (size_t e = 0; e < nx * ny; e++) {
                  int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, "",
                                          member->membername, memberMap, lcmTypeTable,
                                          [this, e] (const std::string&, const std::string&,
                                                     PJ::sdk::ValueRef value) {
                                            height_data[e] = std::get<float>(value);
                                          });
                  if(tlen < 0) break; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                }
                // 退出多维数组循环。
                break;
              }

              if (!strcmp(member->membername, "std")) {
                std_data.resize(nx * ny);
               // std::cout << "pos_offset: " << pos_offset << " size: " << nx * ny << " data_size: " << data_size << "recvbuf: " << recvbuf << std::endl;
                for (size_t e = 0; e < nx * ny; e++) {
                  
                  int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, "",
                                          member->membername, memberMap, lcmTypeTable,
                                          [this, e] (const std::string&, const std::string&,
                                                     PJ::sdk::ValueRef value) {
                                            std_data[e] = std::get<float>(value);
                                          });
                  if(tlen < 0) break; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                }

                auto elevation_marker_error_bar = makeElevationErrorBarMap(
                  "base",            // frame_id
                  "elevation_std",      // ns
                  1,                // id
                  ny, nx,    // rows, cols
                  height_data,      // 高度数据
                  std_data,           // 错误条误差
                  resolution,             // resolution: 每格 5cm
                  center_x, center_y,         // origin
                  1.0f,             // z_scale
                  -0.5f, 0.5f       // min_z, max_z（颜色范围）
                );

                auto elevation_marker = makeElevationMap(
                  "base",            // frame_id
                  "elevation",      // ns
                  0,                // id
                  ny, nx,    // rows, cols
                  height_data,      // 高度数据
                  resolution,             // resolution: 每格 5cm
                  center_x, center_y,         // origin
                  1.0f,             // z_scale
                  -0.5f, 0.99f       // min_z, max_z（颜色范围）
                );

                publishVisualizationErrBar(timestamp, elevation_marker_error_bar);
                publishVisualization(timestamp, elevation_marker);
                // 退出多维数组循环。
                break;
              }
            }

            if (auto it = memberMap.find(ld->size); it != memberMap.end()) {
              for (size_t i = 0; i < it->second; i++) {
                int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, 
                                        base + member->membername + "/",
                                        std::to_string(i), memberMap, lcmTypeTable, writeCb);
                if(tlen < 0) break; else {
                  pos_offset += tlen;
                  decode_len += tlen;
                }
              }
              continue;;
            }
          }
          
          for (size_t i = 0; i < atoi(ld->size); i++) {
            int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, 
                                    base + member->membername + "/",
                                    std::to_string(i), memberMap, lcmTypeTable, writeCb);
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

void LcmStreamSource::publishVisualizationErrBar(
    int64_t timestamp,
    const visualization_msgs::msg::Marker& marker_error_bar) {
  if (m_marker_error_bar_handle_.id == 0) {
    static constexpr char MARKER_ERROR_BAR_SCHEMA[] = R"SCHEMA(
std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
std_msgs/ColorRGBA color
builtin_interfaces/Duration lifetime
bool frame_locked
geometry_msgs/Point[] points
std_msgs/ColorRGBA[] colors
string text
string mesh_resource
bool mesh_use_embedded_materials
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: builtin_interfaces/Duration
int32 sec
uint32 nanosec
)SCHEMA";

    const auto* schema_data = reinterpret_cast<const uint8_t*>(MARKER_ERROR_BAR_SCHEMA);
    const size_t schema_size = sizeof(MARKER_ERROR_BAR_SCHEMA) - 1;

    nlohmann::json marker_parser_config;
    marker_parser_config["serialization"] = "cdr";
    marker_parser_config["schema_encoding"] = "ros2msg";
    marker_parser_config["topic_name"] = "/elevation_map";

    PJ::ParserBindingRequest marker_request{
        .topic_name = "/elevation_map_err_bar",
        .parser_encoding = "ros2msg",
        .type_name = "visualization_msgs/msg/Marker",
        .schema = PJ::Span<const uint8_t>(schema_data, schema_size),
        .parser_config_json = marker_parser_config.dump(),
    };

    auto marker_result = runtimeHost().ensureParserBinding(marker_request);
    m_marker_error_bar_handle_ = *marker_result;
  }

  auto marker_tmp = marker_error_bar;
  marker_tmp.header.stamp.sec = timestamp / 1000000;
  marker_tmp.header.stamp.nanosec = timestamp * 1000;
  auto marker_bytes = std::make_shared<std::vector<uint8_t>>(
      serializeVisualization(marker_tmp));

  (void)runtimeHost().pushMessage(
      m_marker_error_bar_handle_,
      timestamp * 1000,
      [marker_bytes]() {
        return PJ::sdk::PayloadView{marker_bytes};
      });
}

void LcmStreamSource::publishVisualization(
    int64_t timestamp,
    const visualization_msgs::msg::Marker& marker) {
  if (m_marker_handle_.id == 0) {
    static constexpr char MARKER_SCHEMA[] = R"SCHEMA(
std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
std_msgs/ColorRGBA color
builtin_interfaces/Duration lifetime
bool frame_locked
geometry_msgs/Point[] points
std_msgs/ColorRGBA[] colors
string text
string mesh_resource
bool mesh_use_embedded_materials
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
================================================================================
MSG: builtin_interfaces/Duration
int32 sec
uint32 nanosec
)SCHEMA";

    const auto* schema_data = reinterpret_cast<const uint8_t*>(MARKER_SCHEMA);
    const size_t schema_size = sizeof(MARKER_SCHEMA) - 1;

    nlohmann::json marker_parser_config;
    marker_parser_config["serialization"] = "cdr";
    marker_parser_config["schema_encoding"] = "ros2msg";
    marker_parser_config["topic_name"] = "/elevation_map";

    PJ::ParserBindingRequest marker_request{
        .topic_name = "/elevation_map",
        .parser_encoding = "ros2msg",
        .type_name = "visualization_msgs/msg/Marker",
        .schema = PJ::Span<const uint8_t>(schema_data, schema_size),
        .parser_config_json = marker_parser_config.dump(),
    };

    auto marker_result = runtimeHost().ensureParserBinding(marker_request);
    m_marker_handle_ = *marker_result;
  }

  auto marker_tmp = marker;
  marker_tmp.header.stamp.sec = timestamp / 1000000;
  marker_tmp.header.stamp.nanosec = timestamp * 1000;
  auto marker_bytes = std::make_shared<std::vector<uint8_t>>(
      serializeVisualization(marker_tmp));

  (void)runtimeHost().pushMessage(
      m_marker_handle_,
      timestamp * 1000,
      [marker_bytes]() {
        return PJ::sdk::PayloadView{marker_bytes};
      });
}

void LcmStreamSource::savePoint(void* recvbuf, int data_size, int64_t timestamp, const std::string& topic_name,
                                const std::string& field, PJ::sdk::ValueRef value) {
  auto topic_handle = writeHost().ensureTopic(topic_name);
                            
  const PJ::sdk::NamedFieldValue fields[] = {
      {.name = field, .value = value}
  };

  // 2. 按正确顺序调用
  (void)writeHost().appendRecord(
      *topic_handle,                                              // TopicHandle
      PJ::Timestamp{timestamp * 1000},                   // Timestamp
      PJ::Span<const PJ::sdk::NamedFieldValue>(fields, 1)       // Span
  );

  if (publishTfCount < 34) {
    for (int leg_idx = 0; leg_idx < 12; leg_idx++) {
      std::string str_q = std::string("leg_control_data/q/");
      if (str_q == topic_name && field == std::to_string(leg_idx)) {
        robot_state_publisher_.setLegPos(leg_idx, std::get<float>(value));
        publishTfCount++;
        return;
      }
    }

    for (int arm_idx = 0; arm_idx < 14; arm_idx++) {
      std::string str_q = std::string("arm_control_data/joint_curPos/");
      if (str_q == topic_name && field == std::to_string(arm_idx)) {
        robot_state_publisher_.setArmPos(arm_idx, std::get<float>(value));
        publishTfCount++;
        return;
      }
    }

    for (int waist_idx = 0; waist_idx < 3; waist_idx++) {
      std::string str_q = std::string("waist_control_data/q/");
      if (str_q == topic_name && field == std::to_string(waist_idx)) {
        robot_state_publisher_.setWaistPos(waist_idx, std::get<float>(value));
        publishTfCount++;
        return;
      }
    }

    for (int quat_idx = 0; quat_idx < 4; quat_idx++) {
      std::string str_q = std::string("state_estimator/quat/");
      if (str_q == topic_name && field == std::to_string(quat_idx)) {
        robot_state_publisher_.setStateEstimator(quat_idx, std::get<float>(value));
        publishTfCount++;
        return;
      }
    }
  }
}

// onPoll() drains this stream's own inbox — never touches the network.
PJ::Status LcmStreamSource::onPoll() {
  if (message_queue.empty())
        return PJ::okStatus();;
  
  int queue_size = message_queue.size();

  for (size_t i = 0; i < queue_size; i++) {
    LcmMessage msg = message_queue.pop();
    //每100ms发布一次TF
    if (msg.timestamp - last_TF_timestamp > 100000) {
      last_TF_timestamp = msg.timestamp;
      robot_state_publisher_.publishTransforms(
        [this, &msg] (const std::vector<geometry_msgs::msg::TransformStamped>& tf_transforms) {
          publishTF(msg.data.data(), msg.data.size(), msg.timestamp, tf_transforms);}, 
        [this, &msg] (const sensor_msgs::msg::JointState& joint_state) {
          return;
          publishJointState(msg.data.data(), msg.data.size(), msg.timestamp, joint_state);
        });
      publishTfCount = 0;
    }

    
    if (auto it = topic_map.find(msg.channel); it != topic_map.end()) {
      bool channel_match = false;
      for (auto topic_lcmgen : it->second.lcmgen_table) {
        int64_t msg_hash;
        int thislen = __int64_t_decode_array(msg.data.data(), 0, msg.data.size(), &msg_hash, 1);
        for (int struct_idx = 0; struct_idx < topic_lcmgen->structs->len; struct_idx++) {
          lcm_struct_t* struct_item = (lcm_struct_t*)g_ptr_array_index(topic_lcmgen->structs, struct_idx);
          if (msg_hash == ((struct_item->hash<<1) + ((struct_item->hash>>63)&1))) {
            decode_type_1(msg.data.data(), msg.data.size(), msg.timestamp, 8, struct_item->structname->lctypename, msg.channel + "/", lcm_type_table,
                      [&msg, this] (const std::string& topic_name, const std::string& field, PJ::sdk::ValueRef value) {
                        savePoint(msg.data.data(), msg.data.size(), msg.timestamp, topic_name, field, value);
                      });
            channel_match = true;
            break;
          }
        }
      }

      if (!channel_match) {
        std::cout << "channel_match: " << msg.channel << std::endl;
        runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kError, std::string("Warning: Hash mismatch for channel ") + msg.channel);
        runtimeHost().showError("LCM import failed", std::string("Warning: Hash mismatch for channel ") + msg.channel);
        return PJ::unexpected(std::string("channel_match: ") + msg.channel);
      }
    }
  }
  
  // auto batch = subscriber_->queue.drain();

  // while (!batch.empty()) {
  //   auto& dgram = batch.front();

  //   auto status = ingest_.push(
  //       runtimeHost(), "udp/data",
  //       {
  //           .topic_name = "udp/data",
  //           .parser_encoding = default_encoding_,
  //           .type_name = {},
  //           .schema = {},
  //           .parser_config_json = parser_config_override_,
  //       },
  //       PJ::Timestamp{dgram.timestamp_ns}, std::move(dgram.data));
  //   if (!status) {
  //     runtimeHost().reportMessage(
  //         PJ::DataSourceMessageLevel::kError, "Parse error — stopping stream: " + status.error());
  //     return PJ::unexpected(status.error());
  //   }

  //   batch.pop();
  // }

  return PJ::okStatus();
}

void LcmStreamSource::onStop() {
  std::cout << "DataStreamLCM::shutdown" << std::endl;
  _running = false;
  
  if (lcm_runner_.joinable()) {
    lcm_runner_.join();
  }
  std::cout << "DataStreamLCM::shutdown lcm_runner_ joined" << std::endl;
  if (render_runner_.joinable()) {
    render_runner_.join();
  }
  std::cout << "DataStreamLCM::shutdown render_runner_ joined" << std::endl;
  lcm_table.clear();
  lcm_log.reset();
  message_queue.clear();
}

PJ_DATA_SOURCE_PLUGIN(LcmStreamSource, kMcapManifest)

PJ_DIALOG_PLUGIN(LcmDialog, kMcapManifest)