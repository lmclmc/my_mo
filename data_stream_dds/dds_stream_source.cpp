#include "dds_stream_source.hpp"

#include <exception>
#include <type_traits>
#include <variant>
#include <vector>

/// 将宿主传入的 config_json 交给对话框解析。失败时同时报告错误并返回失败状态。
PJ::Status DdsStreamSource::loadConfig(std::string_view config_json) {
  if (!dialog_.loadConfig(config_json)) {
    runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kError, "Warning: Invalid config JSON");
    runtimeHost().showError("DDS import failed", "Warning: Invalid config JSON");
    return PJ::unexpected(std::string("invalid config JSON"));
  }
  return PJ::okStatus();
}

/// 使用当前配置启动 DDS 后台流，必要时初始化并发布 3D 模型。
/// 已在运行或初始化失败时返回失败状态，成功时返回 okStatus。
PJ::Status DdsStreamSource::onStart() {
  if (dds_stream_) return PJ::unexpected(std::string("DDS stream is already running"));
  try {
    auto settings = dialog_.ddsSettings();
    if (settings.robot.enabled) {
      // 开启 3D 时先校验模型并补齐其 DDS 输入话题，再为 URDF 和 TF 建立 ROS 消息解析绑定。
      tf_interval_ = std::chrono::milliseconds(settings.robot.tf_interval_ms);
      robot_model_ = std::make_unique<DdsRobotModel>(settings.robot);
      settings.robot.addTopics(settings.selected_topics);
      auto bind = [this](std::string_view topic, std::string_view type, std::string_view schema) {
        const auto config = nlohmann::json{{"serialization", "cdr"}, {"schema_encoding", "ros2msg"},
                                          {"topic_name", topic}}.dump();
        auto result = runtimeHost().ensureParserBinding({topic, "ros2msg", type,
            {reinterpret_cast<const uint8_t*>(schema.data()), schema.size()}, config});
        if (!result) throw std::runtime_error("3D 模型需要 ROS 消息解析插件（ros2msg）：" + result.error());
        return *result;
      };
      robot_description_handle_ = bind("/robot_description", "std_msgs/msg/String", "string data\n");
      robot_tf_handle_ = bind(kDdsRobotTfTopic, kDdsRobotTfType, kDdsRobotTfSchema);
      robot_description_bytes_ = std::make_shared<const std::vector<uint8_t>>(
          ddsRobotDescriptionCdr(robot_model_->description()));
      runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kInfo,
          "DDS 3D 模型已加载：" + std::to_string(robot_model_->mapping().size()) +
          " 个关节按 MuJoCo actuator 顺序映射，state_estimator.quat 驱动整体姿态。将 /robot_description 拖入 3D 窗口，Fixed frame 选择 " +
          robot_model_->worldFrame() + "。");
    }
    dds_stream_ = std::make_unique<DdsDynamicStream>(settings);
    if (robot_model_) {
      robot_timestamp_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      const auto status = publishRobot(true);
      if (!status) throw std::runtime_error(status.error());
    }
    return PJ::okStatus();
  } catch (const std::exception& error) {
    // 模型、解析器或后台流可能已部分初始化。走统一停止路径释放已建立的资源。
    onStop();
    return PJ::unexpected(std::string("DDS connection failed: ") + error.what());
  }
}

/// 消费后台流拥有的通知和样本，将字段写入宿主并更新 3D 姿态。
/// 写入或发布失败时返回对应错误，单个模型输入失效时记录告警并继续处理其余记录。
PJ::Status DdsStreamSource::onPoll() {
  if (dds_stream_) {
    for (const auto& notice : dds_stream_->takeNotices()) {
      runtimeHost().reportMessage(notice.warning ? PJ::DataSourceMessageLevel::kWarning
                                                : PJ::DataSourceMessageLevel::kInfo, notice.text);
    }
    // DDS 收包和解码在后台线程完成。这里在宿主轮询阶段将自有记录转成写入接口的临时视图。
    for (const auto& record : dds_stream_->takeRecords()) {
      auto topic = writeHost().ensureTopic(record.topic);
      if (!topic) return PJ::unexpected(topic.error());
      std::vector<PJ::sdk::NamedFieldValue> fields;
      fields.reserve(record.fields.size());
      for (const auto& field : record.fields) {
        const auto value = std::visit([](const auto& item) -> PJ::sdk::ValueRef {
          using T = std::decay_t<decltype(item)>;
          if constexpr (std::is_same_v<T, std::string>) return std::string_view(item);
          else return item;
        }, field.value);
        fields.push_back({field.name, value});
      }
      const auto status = writeHost().appendRecord(*topic, PJ::Timestamp{record.timestamp_ns},
          PJ::Span<const PJ::sdk::NamedFieldValue>(fields.data(), fields.size()));
      if (!status) return status;
      if (robot_model_) {
        try {
          if (robot_model_->update(record)) {
            // 各话题到达顺序可能不同，已发布变换使用的样本时间保持单调不减。
            robot_timestamp_ns_ = std::max(robot_timestamp_ns_, record.timestamp_ns);
            robot_dirty_ = true;
            robot_errors_.erase(record.topic);
          }
        } catch (const std::exception& error) {
          // 同一话题重复遇到相同问题时只提示一次，直到收到新的有效模型输入。
          if (robot_errors_[record.topic] != error.what()) {
            robot_errors_[record.topic] = error.what();
            runtimeHost().reportMessage(PJ::DataSourceMessageLevel::kWarning,
                "3D 模型跳过无效姿态或关节样本：" + std::string(error.what()));
          }
        }
      }
    }
  }
  if (robot_model_) return publishRobot();
  return PJ::okStatus();
}

/// force 为 true 时立即提交模型描述和 TF，用于会话初始化。
/// 常规轮询按描述刷新周期和姿态更新频率发布。成功或无需发布时返回成功状态。
PJ::Status DdsStreamSource::publishRobot(bool force) {
  const auto now = std::chrono::steady_clock::now();
  // 模型描述每隔 5 秒重新提交，供稍后打开的 3D 视图在宿主实时保留窗口内取得 URDF。
  // TF 仅在姿态有更新且达到配置的时间间隔时提交。
  if (force || now - last_description_publish_ >= std::chrono::seconds(5)) {
    const auto bytes = robot_description_bytes_;
    const auto description_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const auto status = runtimeHost().pushMessage(robot_description_handle_, description_timestamp,
        [bytes] { return PJ::sdk::PayloadView{bytes}; });
    if (!status) return status;
    last_description_publish_ = now;
  }
  if (!force && (!robot_dirty_ || now - last_robot_publish_ < tf_interval_))
    return PJ::okStatus();
  try {
    const auto bytes = std::make_shared<const std::vector<uint8_t>>(
        ddsRobotTransformsCdr(robot_model_->transforms(robot_timestamp_ns_)));
    const auto status = runtimeHost().pushMessage(robot_tf_handle_, robot_timestamp_ns_,
        [bytes] { return PJ::sdk::PayloadView{bytes}; });
    if (!status) return status;
    dds_stream_->recordRobotTf(robot_timestamp_ns_, *bytes);
    last_robot_publish_ = now;
    robot_dirty_ = false;
    return PJ::okStatus();
  } catch (const std::exception& error) {
    return PJ::unexpected(std::string("3D 模型变换失败：") + error.what());
  }
}

/// 同步停止后台流并转发停止时积累的通知，再清理模型、解析绑定句柄和发布状态。
void DdsStreamSource::onStop() {
  if (dds_stream_) {
    dds_stream_->stop();
    for (const auto& notice : dds_stream_->takeNotices()) {
      runtimeHost().reportMessage(notice.warning ? PJ::DataSourceMessageLevel::kWarning
                                                : PJ::DataSourceMessageLevel::kInfo, notice.text);
    }
    dds_stream_.reset();
  }
  robot_model_.reset();
  robot_description_handle_ = {};
  robot_tf_handle_ = {};
  robot_description_bytes_.reset();
  robot_errors_.clear();
  robot_timestamp_ns_ = 0;
  robot_dirty_ = false;
  last_robot_publish_ = {};
  last_description_publish_ = {};
}

// 注册宿主可加载的数据源和对话框入口，两者共用内嵌插件清单。
PJ_DATA_SOURCE_PLUGIN(DdsStreamSource, kDdsManifest)
PJ_DIALOG_PLUGIN(DdsDialog, kDdsManifest)
