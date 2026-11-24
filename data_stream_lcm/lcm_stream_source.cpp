#include "lcm_stream_source.hpp"

#include "lcm_endpoint.hpp"

#include <bit>
#include <cerrno>
#include <charconv>
#include <cstdlib>
#include <limits>
#include <poll.h>
#include <stdexcept>
#include <type_traits>

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

LcmStreamSource::~LcmStreamSource() {
  onStop();
}

namespace {

// LCM fingerprints include the rotated fingerprints of nested types.
// The field decoder requires a finite, non-recursive schema.
uint64_t typeFingerprint(const lcm_struct_t& type,
                         const std::map<std::string, const lcm_struct_t*>& types,
                         std::set<const lcm_struct_t*>& active) {
  if (!active.insert(&type).second) {
    throw std::runtime_error("暂不支持递归 LCM 类型");
  }
  auto hash = static_cast<uint64_t>(type.hash);
  for (guint i = 0; i < type.members->len; ++i) {
    const auto* member = static_cast<const lcm_member_t*>(g_ptr_array_index(type.members, i));
    const auto* name = member->type->lctypename;
    if (!lcm_is_primitive_type(name)) {
      const auto nested = types.find(name);
      if (nested == types.end()) {
        throw std::runtime_error(std::string("未定义的 LCM 类型 ") + name);
      }
      hash += typeFingerprint(*nested->second, types, active);
    }
  }
  active.erase(&type);
  return std::rotl(hash, 1);
}

}  // namespace

PJ::Status LcmStreamSource::loadTopicSchemas() {
  TypeTable shared_types;
  for (const auto& item : dialog_.getTopicConfig()) {
    if (item.topic_name.empty() || item.lcm_file_path.empty() || schema_files_.contains(item.lcm_file_path)) {
      continue;
    }
    SchemaFile file(lcmgen_create(), lcmgen_destroy);
    if (lcmgen_handle_file(file.get(), item.lcm_file_path.c_str()) != 0 || file->structs->len == 0) {
      return PJ::unexpected("无法读取 LCM 骨架文件 " + item.lcm_file_path);
    }
    for (guint i = 0; i < file->structs->len; ++i) {
      const auto* type = static_cast<const lcm_struct_t*>(g_ptr_array_index(file->structs, i));
      shared_types.try_emplace(type->structname->lctypename, type);
    }
    schema_files_.emplace(item.lcm_file_path, std::move(file));
  }

  std::map<TopicKey, std::vector<const lcmgen_t*>> files_by_topic;
  std::map<std::string, size_t> port_counts;
  for (const auto& item : dialog_.getTopicConfig()) {
    if (item.topic_name.empty() || item.lcm_file_path.empty()) {
      continue;
    }
    const auto port = lcm_stream::parseLcmPort(item.port);
    if (!port) {
      return PJ::unexpected("LCM 话题 " + item.topic_name + " 的端口无效 " + item.port);
    }
    auto [entry, inserted] = files_by_topic.try_emplace({*port, item.topic_name});
    if (inserted) {
      ++port_counts[item.topic_name];
    }
    entry->second.push_back(schema_files_.at(item.lcm_file_path).get());
  }
  for (const auto& [key, files] : files_by_topic) {
    const auto& [port, topic] = key;
    auto& schema = topic_map_[key];
    schema.types = shared_types;
    schema.curve_prefix = topic;
    if (port_counts.at(topic) > 1) {
      schema.curve_prefix = std::to_string(port) + "/" + topic;
    }
    for (const auto* file : files) {
      for (guint i = 0; i < file->structs->len; ++i) {
        const auto* type = static_cast<const lcm_struct_t*>(g_ptr_array_index(file->structs, i));
        schema.types.insert_or_assign(type->structname->lctypename, type);
      }
    }
    try {
      for (const auto* file : files) {
        for (guint i = 0; i < file->structs->len; ++i) {
          const auto* type = static_cast<const lcm_struct_t*>(g_ptr_array_index(file->structs, i));
          std::set<const lcm_struct_t*> active;
          schema.types_by_hash.try_emplace(typeFingerprint(*type, schema.types, active), type);
        }
      }
    } catch (const std::exception& error) {
      return PJ::unexpected("LCM 话题 " + topic + " 的骨架无法解析：" + error.what());
    }
  }
  return PJ::okStatus();
}

void LcmStreamSource::handleMessage(int port, const lcm::ReceiveBuffer* buffer, const std::string& topic) {
  if (lcm_log) {
    lcm_log->write(topic, buffer);
  }
  const TopicKey key{port, topic};
  if (buffer->data_size < sizeof(int64_t) || buffer->data_size > std::numeric_limits<int>::max() ||
      !topic_map_.contains(key)) {
    return;
  }

  const auto* begin = static_cast<const uint8_t*>(buffer->data);
  message_queue.push({key, buffer->recv_utime, {begin, begin + buffer->data_size}});
}

PJ::Status LcmStreamSource::onStart() {
  onStop();
  tf_interval_us_ = static_cast<int64_t>(dialog_.tfIntervalMs()) * 1000;
  last_tf_timestamp_.reset();
  m_image_handle_ = PJ::ParserBindingHandle();
  m_tf_handle_ = PJ::ParserBindingHandle();
  m_marker_handle_ = PJ::ParserBindingHandle();
  m_marker_error_bar_handle_ = PJ::ParserBindingHandle();

  const auto fail = [this](std::string error) {
    onStop();
    runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kError, error);
    runtimeHost().showError("LCM 连接失败", error);
    return PJ::unexpected(std::move(error));
  };
  if (auto status = loadTopicSchemas(); !status) {
    return fail(status.error());
  }

  std::vector<pollfd> descriptors;
  for (const int port : dialog_.listeningPorts()) {
    auto receiver = std::make_unique<Receiver>(this, port, lcm_stream::lcmUrl(port));
    auto& connection = receiver->connection;
    const int fd = connection.good() ? connection.getFileno() : -1;
    if (fd < 0 || !connection.subscribe(".*", &Receiver::receive, receiver.get())) {
      return fail("无法监听 LCM 端口 " + std::to_string(port));
    }
    descriptors.push_back({fd, POLLIN, 0});
    receivers_.push_back(std::move(receiver));
  }

  log_save_path = dialog_.log_save_path();
  log_max_size = dialog_.log_max_size();
  log_max_files_num = dialog_.log_max_files_num();
  if (!log_save_path.empty()) {
    lcm_log = std::make_shared<rotating_file_sink>(log_save_path, log_max_size, log_max_files_num);
  }

  robot_state_publisher_ = RobotStatePublisher{};
  if (!dialog_.urdfpath().empty() || !dialog_.xmlpath().empty()) {
    robot_state_publisher_.init(dialog_.urdfpath(), dialog_.xmlpath(),
                                [this](const std::string& urdf_xml) { publishRobotState(urdf_xml); });
  }

  running_ = true;
  lcm_runner_ = std::thread([this, descriptors = std::move(descriptors)]() mutable {
    while (running_) {
      const int ready = ::poll(descriptors.data(), descriptors.size(), 300);
      if (ready < 0 && errno != EINTR) {
        break;
      }
      if (ready <= 0) {
        continue;
      }
      for (size_t i = 0; i < descriptors.size(); ++i) {
        if (descriptors[i].revents & POLLIN) {
          receivers_[i]->connection.handle();
        }
      }
    }
  });
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
    transform.header.stamp.nanosec = (timestamp % 1000000) * 1000;
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
                             const TypeTable& lcmTypeTable,
                             const WriteCB& writeCb) {
  const int remaining = data_size - offset;
  const auto read = [&](auto decoder, auto value) -> int {
    const int length = decoder(recvbuf, offset, remaining, &value, 1);
    if (length < 0) {
      return length;
    }
    writeCb(base, memberName, value);
    if constexpr (std::is_integral_v<decltype(value)>) {
      if (std::in_range<int>(value)) {
        memberMap[memberName] = static_cast<int>(value);
      }
    }
    return length;
  };

  if (type == "int8_t") return read(__int8_t_decode_array, int8_t{});
  if (type == "int16_t") return read(__int16_t_decode_array, int16_t{});
  if (type == "int32_t") return read(__int32_t_decode_array, int32_t{});
  if (type == "int64_t") return read(__int64_t_decode_array, int64_t{});
  if (type == "byte") return read(__byte_decode_array, uint8_t{});
  if (type == "float") return read(__float_decode_array, float{});
  if (type == "double") return read(__double_decode_array, double{});
  if (type == "boolean") return read(__boolean_decode_array, int8_t{});
  if (type == "string") {
    int32_t size = 0;
    const int header = __int32_t_decode_array(recvbuf, offset, remaining, &size, 1);
    if (header < 0 || size <= 0 || size > remaining - header) {
      return -1;
    }
    const auto* text = static_cast<const char*>(recvbuf) + offset + header;
    writeCb(base, memberName, std::string_view(text, size - 1));
    return header + size;
  }
  return decode_type_1(recvbuf, data_size, timestamp, offset, type, base + memberName + "/", lcmTypeTable, writeCb);
}

int LcmStreamSource::decode_type_1(void* recvbuf, int data_size, int64_t timestamp, int offset, const std::string& type, 
                             const std::string& base, const TypeTable& lcmTypeTable,
                             const WriteCB& writeCb) {
  int pos_offset = offset;
  int decode_len = 0;
  std::unordered_map<std::string, int> memberMap;
  if (auto lcmStructType = lcmTypeTable.find(type); lcmStructType != lcmTypeTable.end()) {
    const lcm_struct_t* structType = lcmStructType->second;
    for (unsigned int mind = 0; mind < g_ptr_array_size(structType->members); mind++) {
      lcm_member_t* member = (lcm_member_t*) g_ptr_array_index(structType->members, mind);

      if (g_ptr_array_size(member->dimensions) == 0) {
        int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, 
                                base + "/", member->membername,
                                memberMap, lcmTypeTable, writeCb);
        if (tlen < 0) return -1; else {
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
              const auto h = memberMap.find("height");
              const auto w = memberMap.find("width");
              if (h == memberMap.end() || w == memberMap.end() || h->second <= 0 || w->second <= 0 ||
                  h->second > (data_size - pos_offset) / w->second) {
                return -1;
              }
              const int height = h->second;
              const int width = w->second;
              const size_t pixels = static_cast<size_t>(height) * width;
              std::vector<uint8_t> image_data(pixels);
              for (size_t e = 0; e < pixels; e++) {
                int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, "",
                                        member->membername, memberMap, lcmTypeTable,
                                        [&image_data, e] (const std::string&, const std::string&,
                                                          PJ::sdk::ValueRef value) {
                                          image_data[e] = std::get<float>(value) * 255.0f + 127;
                                        });
                if (tlen < 0) return -1; else {
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
              if (nx <= 0 || ny <= 0 || nx > (data_size - pos_offset) / ny) return -1;

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
                  if (tlen < 0) return -1; else {
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
                  if (tlen < 0) return -1; else {
                    pos_offset += tlen;
                    decode_len += tlen;
                  }
                }

                if (height_data.size() != std_data.size()) return -1;
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
              if (it->second < 0 || it->second > data_size - pos_offset) return -1;
              for (int i = 0; i < it->second; i++) {
                int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, 
                                        base + member->membername + "/",
                                        std::to_string(i), memberMap, lcmTypeTable, writeCb);
                if (tlen < 0) return -1; else {
                  pos_offset += tlen;
                  decode_len += tlen;
                }
              }
              continue;
            }
          }

          const long count = std::strtol(ld->size, nullptr, 0);
          for (long i = 0; i < count; i++) {
            int tlen = decode_type_2(recvbuf, data_size, timestamp, pos_offset, member->type->lctypename, 
                                    base + member->membername + "/",
                                    std::to_string(i), memberMap, lcmTypeTable, writeCb);
            if (tlen < 0) return -1; else {
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

void LcmStreamSource::savePoint(int64_t timestamp, const std::string& curve_path, std::string_view robot_path,
                                const std::string& field, PJ::sdk::ValueRef value) {
  auto topic_handle = writeHost().ensureTopic(curve_path);
  if (!topic_handle) {
    return;
  }
  const PJ::sdk::NamedFieldValue fields[] = {
      {.name = field, .value = value}
  };

  // 2. 按正确顺序调用
  (void)writeHost().appendRecord(
      *topic_handle,                                              // TopicHandle
      PJ::Timestamp{timestamp * 1000},                   // Timestamp
      PJ::Span<const PJ::sdk::NamedFieldValue>(fields, 1)       // Span
  );

  const auto* sample = std::get_if<float>(&value);
  if (!sample) {
    return;
  }
  int index = -1;
  const auto* end = field.data() + field.size();
  const auto [next, error] = std::from_chars(field.data(), end, index);
  if (error != std::errc{} || next != end || index < 0) {
    return;
  }
  if (robot_path == "leg_control_data/q/" && index < 12) {
    robot_state_publisher_.setLegPos(index, *sample);
  } else if (robot_path == "arm_control_data/joint_curPos/" && index < 14) {
    robot_state_publisher_.setArmPos(index, *sample);
  } else if (robot_path == "waist_control_data/q/" && index < 3) {
    robot_state_publisher_.setWaistPos(index, *sample);
  } else if (robot_path == "state_estimator/quat/" && index < 4) {
    robot_state_publisher_.setStateEstimator(index, *sample);
  }
}

// onPoll() drains this stream's own inbox — never touches the network.
PJ::Status LcmStreamSource::onPoll() {
  const int queue_size = message_queue.size();
  for (int i = 0; i < queue_size; ++i) {
    auto msg = message_queue.pop();
    const auto& schema = topic_map_.at(msg.topic);
    const auto bytes = static_cast<int>(msg.data.size());
    int64_t fingerprint = 0;
    __int64_t_decode_array(msg.data.data(), 0, bytes, &fingerprint, 1);
    const auto matched = schema.types_by_hash.find(static_cast<uint64_t>(fingerprint));
    if (matched == schema.types_by_hash.end()) {
      if (reported_mismatches_.insert(msg.topic).second) {
        const auto& [port, topic] = msg.topic;
        runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kWarning,
                                    "LCM 话题 " + topic + "（端口 " + std::to_string(port) +
                                        "）的消息类型与所选骨架不匹配");
      }
      continue;
    }

    const size_t port_prefix_size = schema.curve_prefix.size() - msg.topic.second.size();
    const int decoded = decode_type_1(
        msg.data.data(), bytes, msg.timestamp, sizeof(int64_t), matched->second->structname->lctypename,
        schema.curve_prefix + "/", schema.types,
        [this, &msg, port_prefix_size](const std::string& topic, const std::string& field, PJ::sdk::ValueRef value) {
          savePoint(msg.timestamp, topic, std::string_view(topic).substr(port_prefix_size), field, value);
        });
    if (decoded < 0) {
      continue;
    }
    if (!last_tf_timestamp_ || msg.timestamp - *last_tf_timestamp_ >= tf_interval_us_) {
      last_tf_timestamp_ = msg.timestamp;
      robot_state_publisher_.publishTransforms(
          [this, &msg, bytes](const auto& transforms) {
            publishTF(msg.data.data(), bytes, msg.timestamp, transforms);
          },
          {});
    }
  }
  return PJ::okStatus();
}

void LcmStreamSource::onStop() {
  running_ = false;
  if (lcm_runner_.joinable()) {
    lcm_runner_.join();
  }
  receivers_.clear();
  lcm_log.reset();
  message_queue.clear();
  reported_mismatches_.clear();
  topic_map_.clear();
  schema_files_.clear();
}

PJ_DATA_SOURCE_PLUGIN(LcmStreamSource, kMcapManifest)

PJ_DIALOG_PLUGIN(LcmDialog, kMcapManifest)
