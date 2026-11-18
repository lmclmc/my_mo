#pragma once

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <map>
#include <nlohmann/json.hpp>
#include <pj_plugins/sdk/dialog_plugin_typed.hpp>
#include <pj_plugins/sdk/widget_data.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unistd.h>
#include <vector>

#include "dds_discovery.hpp"
#include "dds_idl.hpp"
#include "dds_settings.hpp"
#include "dds_robot_model.hpp"
#include "datastream_dds_ui.hpp"
#include "dds_manifest.hpp"
#include "update_history.hpp"

class DdsDialog : public PJ::DialogPluginTyped {
  using PJ::DialogPluginTyped::onValueChanged;

 public:
  /// 返回当前 DDS 配置的只读引用。对话框持有该配置，需独立使用时应复制。
  const DdsSettings& ddsSettings() const { return settings_; }

  /// 返回插件清单的 JSON 文本，供宿主读取插件身份和元数据。
  std::string manifest() const override { return kDdsManifest; }

  /// 返回构建时嵌入的 Qt Designer UI 文本，供宿主创建对话框控件。
  std::string ui_content() const override { return kDataStreamDdsUi; }

  /// 处理已完成的后台导入并刷新发现结果，返回本次控件状态和宿主动作的 JSON。
  std::string widget_data() override {
    pollIdlImports();
    pollRobotImport();
    // 3D 模型使用的关节和姿态话题也是订阅输入，生成界面前将它们补入选中集合。
    settings_.robot.addTopics(settings_.selected_topics);
    if (restart_discovery_) {
      discovery_.start(settings_.domain_id);
      restart_discovery_ = false;
    }
    discovery_.poll();
    PJ::WidgetData wd;
    wd.setTabIndex("settingsTabWidget", settings_tab_index_);
    wd.setRange("domainIdSpinBox", 0, 232).setValue("domainIdSpinBox", settings_.domain_id);
    wd.setItems("reliabilityComboBox", {"Best effort", "Reliable"});
    wd.setCurrentIndex("reliabilityComboBox", settings_.reliable ? 1 : 0);
    wd.setItems("durabilityComboBox", {"Volatile", "Transient local"});
    wd.setCurrentIndex("durabilityComboBox", settings_.transient_local ? 1 : 0);
    wd.setItems("historyComboBox", {"Keep last", "Keep all"});
    wd.setCurrentIndex("historyComboBox", settings_.keep_all ? 1 : 0);
    wd.setRange("historyDepthSpinBox", 1, 100000).setValue("historyDepthSpinBox", settings_.history_depth);
    wd.setEnabled("historyDepthSpinBox", !settings_.keep_all);
    wd.setRange("blockingTimeSpinBox", 0, 60000).setValue("blockingTimeSpinBox", settings_.max_blocking_time_ms);
    wd.setEnabled("blockingTimeSpinBox", settings_.reliable);
    wd.setText("partitionEdit", settings_.partition);
    wd.setChecked("robotEnabledCheckBox", settings_.robot.enabled);
    wd.setText("robotUrdfPathEdit", settings_.robot.urdf_path);
    wd.setText("robotMujocoPathEdit", settings_.robot.mujoco_path);
    wd.setFilePicker("importUrdfButton", "导入 URDF…", "URDF 文件 (*.urdf *.xml);;所有文件 (*)", "选择机器人 URDF");
    wd.setFilePicker("importMujocoButton", "导入 MuJoCo XML…", "MuJoCo XML (*.xml);;所有文件 (*)", "选择 MuJoCo 模型 XML");
    for (size_t g = 0; g < settings_.robot.groups.size(); ++g) {
      const auto prefix = std::string("robot_") + DdsRobotSettings::kGroupNames[g];
      const auto& group = settings_.robot.groups[g];
      wd.setText(prefix + "TopicEdit", group.topic);
      wd.setRange(prefix + "StartSpinBox", 0, DdsRobotSettings::kMaxJoints)
        .setValue(prefix + "StartSpinBox", group.start);
      wd.setRange(prefix + "CountSpinBox", 0, DdsRobotSettings::kMaxJoints)
        .setValue(prefix + "CountSpinBox", group.count);
    }
    wd.setLabel("robotStatusLabel", robot_status_);
    wd.setTableHeaders("robotMappingTableWidget", {"部位", "DDS 话题", "位置索引", "XML 索引", "URDF / XML 关节名"});
    std::vector<std::vector<PJ::TableItem>> robot_rows;
    if (imported_robot_)
      for (const auto& entry : imported_robot_->mapping())
        robot_rows.push_back({entry.group, entry.topic, "q/[" + std::to_string(entry.index) + "]",
                             entry.xml_index, entry.joint});
    wd.setTableRows("robotMappingTableWidget", robot_rows);
    wd.setFilePicker("importIdlButton", "导入 IDL…", "DDS IDL (*.idl);;所有文件 (*)", "导入 DDS 消息定义");
    // 正在编译的 IDL 和待处理队列都会影响保存结果。导入结束后再允许保存和连接。
    const bool importing = idl_import_.valid() || !pending_idl_files_.empty();
    PJ::FilePickerOptions config_picker;
    config_picker.mode = PJ::FilePickerMode::SaveFile;
    config_picker.title = "保存 DDS 配置";
    config_picker.default_suffix = "json";
    config_picker.filters = {{"json", "DDS 配置", {"*.json"}}};
    config_picker.suggested_name = config_file_path_.empty() ? "dds_config.json"
        : std::filesystem::path(config_file_path_).filename().string();
    if (!config_file_path_.empty())
      config_picker.initial_directory = std::filesystem::path(config_file_path_).parent_path().string();
    wd.setFilePicker("saveConfigButton", "保存配置…", config_picker);
    config_picker.mode = PJ::FilePickerMode::OpenFile;
    config_picker.title = "加载 DDS 配置";
    config_picker.suggested_name.clear();
    config_picker.default_suffix.clear();
    wd.setFilePicker("loadConfigButton", "加载配置…", config_picker);
    wd.setEnabled("saveConfigButton", !importing);
    wd.setLabel("configStatusLabel", config_status_);
    wd.setEnabled("importIdlButton", !importing);
    wd.setEnabled("clearIdlButton", !importing && !settings_.idl_files.empty());
    std::string import_status = importing ? "正在导入 IDL：" + importing_idl_path_ :
        idl_status_.empty() ? "未导入 IDL。缺少完整 XTypes 信息的话题可通过 IDL 解析。" : idl_status_;
    for (const auto& [path, error] : idl_errors_) import_status += "\n" + path + "：" + error;
    wd.setText("idlStatusText", import_status);
    const auto& recording = settings_.recording;
    wd.setChecked("recordingEnabledCheckBox", recording.enabled);
    wd.setText("recordingDirectoryEdit", recording.directory);
    wd.setRange("recordingFileSizeSpinBox", 1, DdsRecordingSettings::kMaxFileSizeMb)
      .setValue("recordingFileSizeSpinBox", recording.max_file_size_mb);
    wd.setRange("recordingFileCountSpinBox", 1, DdsRecordingSettings::kMaxFiles)
      .setValue("recordingFileCountSpinBox", recording.max_files);
    PJ::FilePickerOptions directory_picker;
    directory_picker.mode = PJ::FilePickerMode::SelectDirectory;
    directory_picker.title = "选择 DDS 数据录制目录";
    directory_picker.initial_directory = recording.directory;
    wd.setFilePicker("recordingDirectoryButton", "选择目录…", directory_picker);
    for (const auto* widget : {"recordingDirectoryEdit", "recordingDirectoryButton",
                               "recordingFileSizeSpinBox", "recordingFileCountSpinBox"})
      wd.setEnabled(widget, recording.enabled);
    const auto recording_error = recording.validationError();
    wd.setLabel("recordingStatusLabel", !recording_error.empty() ? recording_error : recording.enabled
        ? "连接后录制已选话题，断开时完成写盘。文件达到上限后覆盖最早的录制文件；再次连接继续使用同一文件数量上限。"
        : "勾选后随连接开始录制。MCAP 保存所选话题的 CDR 数据、IDL 类型描述和上位机接收时间；1 MB = 1024 × 1024 字节。");

    struct Counts { int writers = 0; int readers = 0; };
    struct Topic : Counts {
      std::set<std::string> types, partitions, reliability, durability, xtypes;
    };
    std::map<std::string, Counts> partitions;
    // 分区表统计全部发现端点。话题表只聚合与当前分区匹配的端点，按完整话题名生成一行。
    // 宿主用第一列作为选中项的键，同名端点必须汇总在同一行。
    std::map<std::string, Topic> topics;
    for (const auto& [key, endpoint] : discovery_.endpoints()) {
      (void)key;
      std::set<std::string> matching_partitions;
      for (const auto& name : endpoint.partitions) {
        auto& partition = partitions[name];
        if (endpoint.writer) ++partition.writers;
        else ++partition.readers;
        if (settings_.matchesPartition(name)) matching_partitions.insert(partitionLabel(name));
      }
      if (matching_partitions.empty()) continue;
      auto& topic = topics[endpoint.topic];
      if (endpoint.writer) ++topic.writers;
      else ++topic.readers;
      if (endpoint.writer) topic.xtypes.insert(xtypesLabel(endpoint.xtypes));
      topic.types.insert(endpoint.type);
      topic.partitions.insert(matching_partitions.begin(), matching_partitions.end());
      if (!endpoint.reliability.empty()) topic.reliability.insert(endpoint.reliability);
      if (!endpoint.durability.empty()) topic.durability.insert(endpoint.durability);
    }
    wd.setTableHeaders("partitionTableWidget", {"Partition 分区", "发布端", "订阅端"});
    std::vector<std::vector<PJ::TableItem>> partition_rows;
    std::vector<std::string> selection;
    for (const auto& [name, count] : partitions) {
      const auto display = partitionLabel(name);
      partition_rows.push_back({display, count.writers, count.readers});
      if (name == settings_.partition) selection.push_back(display);
    }
    wd.setTableRows("partitionTableWidget", partition_rows);
    wd.setSelectedItems("partitionTableWidget", selection);
    wd.setTableHeaders("topicTableWidget", {"Topic 话题", "Type 类型", "XTypes / IDL 匹配", "IDL 文件路径", "Partition 分区", "发布端", "订阅端",
                                           "Reliability", "Durability"});
    discovered_topics_.clear();
    for (const auto& [name, topic] : topics) {
      (void)topic;
      discovered_topics_.insert(name);
    }
    // 将旧配置中已选的话题补回表中，发现刷新期间仍可看到和取消离线项。
    for (const auto& name : settings_.selected_topics) topics.try_emplace(name);
    std::vector<std::vector<PJ::TableItem>> topic_rows;
    size_t waiting = 0;
    for (const auto& [name, topic] : topics) {
      if (topic.writers == 0 && settings_.selected_topics.count(name)) ++waiting;
      auto xtypes = topic.xtypes.empty() ? "等待发布端" : join(topic.xtypes);
      size_t local_types = 0;
      std::set<std::string> idl_paths;
      for (const auto& type : topic.types) {
        if (const auto* idl = localIdl(type)) {
          ++local_types;
          idl_paths.insert(idl->sourcePath());
        }
      }
      // 宿主按纯文本渲染单元格。用颜色标记展示本地 IDL 匹配比例，
      // 同一列继续保留发布端声明的 XTypes 状态，悬停时提供类型与文件的对应关系。
      if (local_types) xtypes = std::string(local_types == topic.types.size()
          ? "🟢 IDL 已匹配；" : "🟡 部分类型已匹配 IDL；") + xtypes;
      std::string idl_path_text;
      for (const auto& path : idl_paths) {
        if (!idl_path_text.empty()) idl_path_text += " | ";
        idl_path_text += path;
      }
      topic_rows.push_back({name, topic.types.empty() ? "等待发现类型" : join(topic.types),
                            xtypes,
                            idl_path_text.empty() ? "—" : idl_path_text,
                            topic.partitions.empty() ? partitionLabel(settings_.partition) : join(topic.partitions),
                            topic.writers, topic.readers,
                            join(topic.reliability), join(topic.durability)});
    }
    wd.setTableRows("topicTableWidget", topic_rows);
    int row = 0;
    for (const auto& [name, topic] : topics) {
      (void)name;
      std::string source;
      for (const auto& type : topic.types) {
        if (!source.empty()) source += '\n';
        const auto* idl = localIdl(type);
        source += type + (idl ? " ← " + idl->sourcePath() : "：没有匹配的本地 IDL");
      }
      wd.setCellTooltip("topicTableWidget", row, 2,
          "绿色标记：全部类型已匹配 IDL；黄色标记：部分类型已匹配。\n"
          "发布端 XTypes 状态仍来自其声明的 TypeInformation。\n" + source);
      wd.setCellTooltip("topicTableWidget", row++, 3,
          source.empty() ? "等待发现类型，尚未匹配本地 IDL" : source);
    }
    wd.setSelectedItems("topicTableWidget", std::vector<std::string>(settings_.selected_topics.begin(),
                                                                    settings_.selected_topics.end()));
    wd.setEnabled("selectAllTopicsButton", !discovered_topics_.empty());
    wd.setEnabled("clearTopicsButton", !settings_.selected_topics.empty());
    wd.setEnabled("connectButton", !settings_.selected_topics.empty() && !importing &&
                  recording_error.empty() && robotReady());
    wd.setLabel("partitionCountLabel", std::to_string(partitions.size()) + " 个分区");
    wd.setLabel("topicCountLabel", "当前分区发现 " + std::to_string(discovered_topics_.size()) +
                " 个话题 · 已选 " + std::to_string(settings_.selected_topics.size()) + " 个" +
                (waiting ? "（" + std::to_string(waiting) + " 个等待发布端）" : ""));
    wd.setLabel("discoveryStatusLabel", discovery_.error().empty()
        ? "Domain " + std::to_string(settings_.domain_id) + " · 正在监听发现信息 · " +
          std::to_string(discovery_.endpoints().size()) + " 个端点"
        : "发现失败：" + discovery_.error());
    // 将按钮回调留下的请求交给宿主执行，发送后清除标志，避免下一次刷新重复执行。
    if (request_update_history_) {
      wd.requestSubDialog(PjUpdateHistory::dialogUi());
      request_update_history_ = false;
    }
    if (request_accept_) {
      wd.requestAccept();
      request_accept_ = false;
    }
    return wd.toJson();
  }

  /// 宿主周期回调。检查后台导入，每隔至少 250 毫秒轮询发现，状态有变化时返回 true。
  bool onTick() override {
    const bool idl_changed = pollIdlImports();
    const bool robot_changed = pollRobotImport();
    const auto now = std::chrono::steady_clock::now();
    if (now - last_poll_ < std::chrono::milliseconds(250)) return idl_changed || robot_changed;
    last_poll_ = now;
    return discovery_.poll() || idl_changed || robot_changed;
  }

  /// 处理整数控件。widget 是控件名，value 是输入值，按对应范围截断。
  /// 映射、域或录制项需要刷新时返回 true，历史深度和阻塞时间只写入配置并返回 false。
  bool onValueChanged(std::string_view widget, int value) override {
    for (size_t g = 0; g < settings_.robot.groups.size(); ++g) {
      const auto prefix = std::string("robot_") + DdsRobotSettings::kGroupNames[g];
      auto& group = settings_.robot.groups[g];
      int* target = widget == prefix + "StartSpinBox" ? &group.start
          : widget == prefix + "CountSpinBox" ? &group.count : nullptr;
      if (!target) continue;
      value = std::clamp(value, 0, DdsRobotSettings::kMaxJoints);
      if (*target == value) return false;
      *target = value;
      invalidateRobot();
      return true;
    }
    if (widget == "domainIdSpinBox") {
      const int domain = std::clamp(value, 0, 232);
      if (settings_.domain_id == domain) return false;
      settings_.domain_id = domain;
      // DDS 域切换后端点属于另一发现范围，原选项清空，下一次生成控件数据时重建发现实例。
      settings_.selected_topics.clear();
      restart_discovery_ = true;
      return true;
    }
    if (widget == "historyDepthSpinBox") settings_.history_depth = std::clamp(value, 1, 100000);
    if (widget == "blockingTimeSpinBox") settings_.max_blocking_time_ms = std::clamp(value, 0, 60000);
    if (widget == "recordingFileSizeSpinBox") {
      settings_.recording.max_file_size_mb = std::clamp(value, 1, DdsRecordingSettings::kMaxFileSizeMb);
      return true;
    }
    if (widget == "recordingFileCountSpinBox") {
      settings_.recording.max_files = std::clamp(value, 1, DdsRecordingSettings::kMaxFiles);
      return true;
    }
    return false;
  }

  /// 按控件名 widget 设置 3D 机器人或 MCAP 录制开关。checked 是新状态，发生变化时返回 true。
  bool onToggled(std::string_view widget, bool checked) override {
    if (widget == "robotEnabledCheckBox") {
      if (settings_.robot.enabled == checked) return false;
      settings_.robot.enabled = checked;
      invalidateRobot();
      return true;
    }
    if (widget != "recordingEnabledCheckBox" || settings_.recording.enabled == checked) return false;
    settings_.recording.enabled = checked;
    return true;
  }

  /// 将名为 widget 的 QoS 下拉框的索引 index 应用到对应策略。
  /// 支持索引 0 和 1，成功处理返回 true，未知控件或越界返回 false。
  bool onIndexChanged(std::string_view widget, int index) override {
    if (index < 0 || index > 1) return false;
    if (widget == "reliabilityComboBox") settings_.reliable = index == 1;
    else if (widget == "durabilityComboBox") settings_.transient_local = index == 1;
    else if (widget == "historyComboBox") settings_.keep_all = index == 1;
    else return false;
    return true;
  }

  /// 应用文本控件 widget 的新内容 text。更新模型路径或关节话题会触发重新校验，
  /// 更换分区会清空已选话题。已更新的控件返回 true，其余情况返回 false。
  bool onTextChanged(std::string_view widget, std::string_view text) override {
    std::string* robot_target = widget == "robotUrdfPathEdit" ? &settings_.robot.urdf_path
        : widget == "robotMujocoPathEdit" ? &settings_.robot.mujoco_path : nullptr;
    for (size_t g = 0; g < settings_.robot.groups.size(); ++g)
      if (widget == std::string("robot_") + DdsRobotSettings::kGroupNames[g] + "TopicEdit")
        robot_target = &settings_.robot.groups[g].topic;
    if (robot_target) {
      if (*robot_target == text) return false;
      *robot_target = std::string(text);
      invalidateRobot();
      return true;
    }
    if (widget == "recordingDirectoryEdit") {
      if (settings_.recording.directory == text) return false;
      settings_.recording.directory = std::string(text);
      return true;
    }
    if (widget != "partitionEdit") return false;
    if (settings_.partition == text) return false;
    settings_.partition = std::string(text);
    settings_.selected_topics.clear();
    return true;
  }

  /// 处理表格 widget 的选择变化。selected 是所选行的第一列文本。
  /// 话题表更新完整名称集合，分区表用第一项查找分区。配置变化时返回 true。
  bool onSelectionChanged(std::string_view widget, const std::vector<std::string>& selected) override {
    if (widget == "topicTableWidget") {
      std::set<std::string> next;
      for (const auto& topic : selected) {
        if (discovered_topics_.count(topic) || settings_.selected_topics.count(topic)) next.insert(topic);
      }
      // 离线但已选中的旧话题可以保留，启用 3D 模型时重新补齐其必需话题。
      settings_.robot.addTopics(next);
      if (next == settings_.selected_topics) return false;
      settings_.selected_topics = std::move(next);
      return true;
    }
    if (widget != "partitionTableWidget" || selected.empty()) return false;
    for (const auto& [key, endpoint] : discovery_.endpoints()) {
      (void)key;
      for (const auto& name : endpoint.partitions) {
        if (partitionLabel(name) == selected.front()) {
          if (settings_.partition == name) return false;
          settings_.partition = name;
          settings_.selected_topics.clear();
          return true;
        }
      }
    }
    return false;
  }

  /// 保存设置页签 widget 的新索引 index。索引 0 到 2 有效，实际切换时返回 true。
  bool onTabChanged(std::string_view widget, int index) override {
    if (widget != "settingsTabWidget" || index < 0 || index >= 3 || index == settings_tab_index_)
      return false;
    settings_tab_index_ = index;
    return true;
  }

  /// 按按钮名 widget 执行更新记录、重载模型、管理 IDL 或话题及连接操作。
  /// 已处理并需要刷新时返回 true，未识别按钮或操作条件不满足时返回 false。
  bool onClicked(std::string_view widget) override {
    if (widget == "updateHistoryButton") {
      request_update_history_ = true;
      return true;
    }
    if (widget == "robotReloadButton") {
      invalidateRobot();
      return true;
    }
    if (widget == "clearIdlButton") {
      if (idl_import_.valid()) return false;
      settings_.idl_files.clear();
      settings_.idl_libraries.clear();
      pending_idl_files_.clear();
      idl_status_.clear();
      idl_errors_.clear();
      return true;
    }
    if (widget == "selectAllTopicsButton") {
      settings_.selected_topics.insert(discovered_topics_.begin(), discovered_topics_.end());
      return true;
    }
    if (widget == "clearTopicsButton") {
      settings_.selected_topics.clear();
      settings_.robot.addTopics(settings_.selected_topics);
      return true;
    }
    if (widget == "refreshDiscoveryButton") {
      restart_discovery_ = true;
      return true;
    }
    if (widget == "connectButton") {
      // 点击时重新检查输入，确保 IDL 导入已完成，且话题、录制参数和 3D 模型满足连接条件。
      if (settings_.selected_topics.empty() || idl_import_.valid() || !pending_idl_files_.empty() ||
          !settings_.recording.validationError().empty() || !robotReady()) return false;
      request_accept_ = true;
      return true;
    }
    return false;
  }

  /// 根据文件按钮名 widget 将所选 path 分发给配置读写、模型重载或 IDL 导入。
  /// 已受理返回 true，包括需要在界面报告的配置读写失败。取消或未受理时返回 false。
  bool onFileSelected(std::string_view widget, std::string_view path) override {
    if (widget == "saveConfigButton" || widget == "loadConfigButton") {
      if (path.empty()) return false;
      return configFileSelected(widget == "saveConfigButton", path);
    }
    if (widget == "importUrdfButton" || widget == "importMujocoButton") {
      if (path.empty()) return false;
      if (widget == "importUrdfButton") settings_.robot.urdf_path = std::string(path);
      else settings_.robot.mujoco_path = std::string(path);
      settings_.robot.enabled = true;
      invalidateRobot();
      return true;
    }
    if (widget != "importIdlButton" || path.empty() || idl_import_.valid()) return false;
    pending_idl_files_.emplace_back(path);
    pollIdlImports();
    return true;
  }

  /// widget 标识选择目录的按钮，path 是所选路径。录制目录实际更新时返回 true。
  bool onFolderSelected(std::string_view widget, std::string_view path) override {
    if (widget != "recordingDirectoryButton" || path.empty()) return false;
    return onTextChanged("recordingDirectoryEdit", path);
  }

  /// 确认连接后释放对话框的发现资源。宿主传入的确认数据未使用，无返回值。
  void onAccepted(std::string_view) override { closeDiscovery(); }

  /// 取消对话框后释放发现资源，无返回值。
  void onRejected() override { closeDiscovery(); }

  /// 将订阅、QoS、IDL 路径、3D 映射和 MCAP 录制设置序列化为含 dds 对象的 JSON。
  /// 布局保存和独立配置文件共用此结构，IDL 只保存来源路径。
  std::string saveConfig() const override {
    auto cfg = nlohmann::json::object();
    cfg["dds"] = {{"domain_id", settings_.domain_id}, {"partition", settings_.partition},
      {"selected_topics", settings_.selected_topics},
      {"idl_files", settings_.idl_files},
      {"reliability", settings_.reliable ? "reliable" : "best_effort"},
      {"durability", settings_.transient_local ? "transient_local" : "volatile"},
      {"history", settings_.keep_all ? "keep_all" : "keep_last"},
      {"history_depth", settings_.history_depth}, {"max_blocking_time_ms", settings_.max_blocking_time_ms},
      {"recording", {{"enabled", settings_.recording.enabled}, {"directory", settings_.recording.directory},
        {"max_file_size_mb", settings_.recording.max_file_size_mb}, {"max_files", settings_.recording.max_files}}}};
    auto& robot = cfg["dds"]["robot"];
    robot = {{"enabled", settings_.robot.enabled}, {"urdf_path", settings_.robot.urdf_path},
             {"mujoco_path", settings_.robot.mujoco_path}};
    for (size_t g = 0; g < settings_.robot.groups.size(); ++g) {
      const auto& group = settings_.robot.groups[g];
      robot["groups"][DdsRobotSettings::kGroupNames[g]] = {
          {"topic", group.topic}, {"start", group.start}, {"count", group.count}};
    }
    return cfg.dump();
  }

  /// 校验并应用 json 中的 DDS 配置。宿主传入空文本或缺少 dds 对象时采用默认值。
  /// 应用成功返回 true，字段类型、范围或 JSON 格式错误返回 false 并保留原配置。
  bool loadConfig(std::string_view json) override {
    try {
      // 宿主首次打开时传入空字符串，将仅含空白的输入视为缺省配置。
      auto cfg = json.find_first_not_of(" \t\r\n") == std::string_view::npos
          ? nlohmann::json::object() : nlohmann::json::parse(json);
      if (!cfg.is_object()) return false;
      const auto dds = cfg.value("dds", nlohmann::json::object());
      if (!dds.is_object()) return false;
      const auto valid_integer = [](const nlohmann::json& object, const char* key, int low, int high) {
        const auto it = object.find(key);
        return it == object.end() || (it->is_number_integer() && *it >= low && *it <= high);
      };
      if (!valid_integer(dds, "domain_id", 0, 232) || !valid_integer(dds, "history_depth", 1, 100000) ||
          !valid_integer(dds, "max_blocking_time_ms", 0, 60000)) return false;
      // 在临时配置中完成字段校验。IDL 文件去重并保留输入顺序，后续按此顺序重新导入。
      DdsSettings next;
      next.domain_id = dds.value("domain_id", 200);
      next.partition = dds.value("partition", std::string{});
      if (const auto it = dds.find("idl_files"); it != dds.end()) {
        if (!it->is_array()) return false;
        std::set<std::string> unique;
        for (const auto& path : *it) {
          if (!path.is_string() || path.get_ref<const std::string&>().empty()) return false;
          const auto value = path.get<std::string>();
          if (unique.insert(value).second) next.idl_files.push_back(value);
        }
      }
      if (const auto it = dds.find("selected_topics"); it != dds.end()) {
        if (!it->is_array()) return false;
        for (const auto& topic : *it) {
          if (!topic.is_string() || topic.get_ref<const std::string&>().empty()) return false;
          next.selected_topics.insert(topic.get<std::string>());
        }
      }
      const auto reliability = dds.value("reliability", std::string("best_effort"));
      const auto durability = dds.value("durability", std::string("volatile"));
      const auto history = dds.value("history", std::string("keep_last"));
      next.history_depth = dds.value("history_depth", 1);
      next.max_blocking_time_ms = dds.value("max_blocking_time_ms", 100);
      if (const auto it = dds.find("recording"); it != dds.end()) {
        if (!it->is_object()) return false;
        if (!valid_integer(*it, "max_file_size_mb", 1, DdsRecordingSettings::kMaxFileSizeMb) ||
            !valid_integer(*it, "max_files", 1, DdsRecordingSettings::kMaxFiles)) return false;
        next.recording.enabled = it->value("enabled", false);
        next.recording.directory = it->value("directory", std::string{});
        next.recording.max_file_size_mb = it->value("max_file_size_mb", 128);
        next.recording.max_files = it->value("max_files", 10);
        if (!next.recording.validationError().empty()) return false;
      }
      if (const auto it = dds.find("robot"); it != dds.end()) {
        if (!it->is_object()) return false;
        next.robot.enabled = it->value("enabled", false);
        next.robot.urdf_path = it->value("urdf_path", std::string{});
        next.robot.mujoco_path = it->value("mujoco_path", std::string{});
        const auto groups = it->value("groups", nlohmann::json::object());
        if (!groups.is_object()) return false;
        for (size_t g = 0; g < next.robot.groups.size(); ++g) {
          const auto group = groups.find(DdsRobotSettings::kGroupNames[g]);
          if (group == groups.end()) continue;
          if (!group->is_object()) return false;
          if (!valid_integer(*group, "start", 0, DdsRobotSettings::kMaxJoints) ||
              !valid_integer(*group, "count", 0, DdsRobotSettings::kMaxJoints)) return false;
          auto& target = next.robot.groups[g];
          target.topic = group->value("topic", target.topic);
          target.start = group->value("start", target.start);
          target.count = group->value("count", target.count);
          if (target.start < 0 || target.count < 0 || target.start > DdsRobotSettings::kMaxJoints - target.count)
            return false;
        }
      }
      if (next.domain_id < 0 || next.domain_id > 232 || next.history_depth < 1 || next.history_depth > 100000 ||
          next.max_blocking_time_ms < 0 || next.max_blocking_time_ms > 60000 ||
          (reliability != "best_effort" && reliability != "reliable") ||
          (durability != "volatile" && durability != "transient_local") ||
          (history != "keep_last" && history != "keep_all")) return false;
      next.reliable = reliability == "reliable";
      next.transient_local = durability == "transient_local";
      next.keep_all = history == "keep_all";
      // 全部校验通过后替换配置。更新导入代次，使旧 IDL 任务完成时无法覆盖新配置。
      settings_ = std::move(next);
      invalidateRobot();
      ++idl_generation_;
      pending_idl_files_.assign(settings_.idl_files.begin(), settings_.idl_files.end());
      idl_status_.clear();
      idl_errors_.clear();
      closeDiscovery();
      return true;
    } catch (const nlohmann::json::exception&) {
      return false;
    }
  }

 private:
  /// 就地更新 dds 对象中的 IDL、模型和录制目录路径，无返回值。
  /// base 为相对路径基准，导入用配置文件目录，导出用当前进程的工作目录。
  static void resolveConfigPaths(nlohmann::json& dds, const std::filesystem::path& base) {
    const auto resolve = [&base](nlohmann::json& value) {
      if (!value.is_string() || value.get_ref<const std::string&>().empty()) return;
      const std::filesystem::path path(value.get<std::string>());
      if (path.is_relative()) value = (base / path).lexically_normal().string();
    };
    if (auto it = dds.find("idl_files"); it != dds.end() && it->is_array())
      for (auto& path : *it) resolve(path);
    if (auto it = dds.find("robot"); it != dds.end() && it->is_object())
      for (const auto* key : {"urdf_path", "mujoco_path"})
        if (it->contains(key)) resolve((*it)[key]);
    if (auto it = dds.find("recording"); it != dds.end() && it->is_object() && it->contains("directory"))
      resolve((*it)["directory"]);
  }

  /// 处理独立配置文件操作。save 选择保存或加载，selected_path 为对话框选出的路径。
  /// 成败均写入状态栏并返回 true，通知宿主刷新结果。
  bool configFileSelected(bool save, std::string_view selected_path) {
    std::filesystem::path temporary;
    int fd = -1;
    try {
      auto path = std::filesystem::absolute(std::filesystem::path(selected_path)).lexically_normal();
      if (save) {
        if (idl_import_.valid() || !pending_idl_files_.empty())
          throw std::runtime_error("请等待 IDL 导入完成后再保存。");
        const auto error = settings_.recording.validationError();
        if (!error.empty()) throw std::runtime_error(error);
        if (!path.has_extension()) path += ".json";
        auto cfg = nlohmann::json::parse(saveConfig());
        resolveConfigPaths(cfg["dds"], std::filesystem::current_path());
        const auto contents = cfg.dump(2) + '\n';
        // 在目标目录创建临时文件，循环处理短写和系统调用中断。
        // 写全、同步并关闭后才重命名到目标位置，替换前出现错误时可保留原文件。
        auto pattern = (path.parent_path() / ".dds-config-XXXXXX").string();
        fd = ::mkstemp(pattern.data());
        if (fd < 0) throw std::system_error(errno, std::generic_category(), "无法创建配置文件");
        temporary = pattern;
        size_t written = 0;
        while (written < contents.size()) {
          const auto count = ::write(fd, contents.data() + written, contents.size() - written);
          if (count < 0 && errno == EINTR) continue;
          if (count <= 0) throw std::system_error(count < 0 ? errno : EIO, std::generic_category(), "写入配置失败");
          written += static_cast<size_t>(count);
        }
        if (::fsync(fd) != 0) throw std::system_error(errno, std::generic_category(), "同步配置失败");
        const int closing = fd;
        fd = -1;
        if (::close(closing) != 0) throw std::system_error(errno, std::generic_category(), "关闭配置失败");
        std::filesystem::rename(temporary, path);
        temporary.clear();
      } else {
        std::ifstream file(path, std::ios::binary);
        if (!file) throw std::runtime_error("无法打开配置文件，请检查文件路径和读取权限。");
        const std::string contents{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
        if (file.bad()) throw std::runtime_error("读取配置文件失败。");
        auto cfg = nlohmann::json::parse(contents);
        const auto it = cfg.is_object() ? cfg.find("dds") : cfg.end();
        if (it == cfg.end() || !it->is_object() || it->empty())
          throw std::runtime_error("请选择包含 dds 设置的 DDS JSON 配置文件。");
        const auto known_fields = {"domain_id", "partition", "selected_topics", "idl_files", "reliability",
            "durability", "history", "history_depth", "max_blocking_time_ms", "robot", "recording"};
        if (std::none_of(known_fields.begin(), known_fields.end(), [&it](const auto* key) { return it->contains(key); }))
          throw std::runtime_error("文件中没有可识别的 DDS 配置项。");
        // 独立导入必须包含已知 DDS 字段。解析相对路径后复用布局配置的字段校验。
        resolveConfigPaths(*it, path.parent_path());
        if (!loadConfig(cfg.dump())) throw std::runtime_error("配置字段类型、取值或范围无效。");
      }
      config_file_path_ = path.string();
      config_status_ = std::string(save ? "已保存：" : "已加载：") + path.filename().string();
    } catch (const std::exception& error) {
      if (fd >= 0) ::close(fd);
      if (!temporary.empty()) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
      }
      config_status_ = save ? "保存失败，原配置文件未修改。" : "加载失败，当前设置未修改。";
      config_status_ += " " + std::string(error.what());
    }
    return true;
  }

  /// 清除旧模型预览并标记重新校验，增加配置代次以作废正在运行的旧任务，无返回值。
  void invalidateRobot() {
    ++robot_generation_;
    robot_import_requested_ = true;
    imported_robot_.reset();
    robot_status_.clear();
    settings_.robot.addTopics(settings_.selected_topics);
  }

  /// 3D 功能关闭或当前配置已通过模型校验时返回 true。
  bool robotReady() const { return !settings_.robot.enabled || imported_robot_ != nullptr; }

  /// 非阻塞收集模型校验结果，必要时用当前配置启动一次后台校验。
  /// 已处理任务或刷新状态时返回 true，无变化时返回 false。
  bool pollRobotImport() {
    bool changed = false;
    if (robot_import_.valid() && robot_import_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        auto model = robot_import_.get();
        // 校验执行期间用户可能再次编辑。当任务代次仍与当前配置相等时才接收结果。
        if (importing_robot_generation_ == robot_generation_) {
          imported_robot_ = std::move(model);
          robot_status_ = "模型与映射校验通过：" + std::to_string(imported_robot_->mapping().size()) +
              " 个关节；根坐标系 " + imported_robot_->rootLink() +
              "；整体姿态来自 state_estimator.quat。连接后将 /robot_description 拖入 3D 窗口，Fixed frame 选择 " +
              imported_robot_->worldFrame() + "。";
        }
      } catch (const std::exception& error) {
        if (importing_robot_generation_ == robot_generation_)
          robot_status_ = "模型导入失败：" + std::string(error.what());
      }
      changed = true;
    }
    if (!robot_import_.valid() && robot_import_requested_) {
      robot_import_requested_ = false;
      imported_robot_.reset();
      const auto error = settings_.robot.validationError();
      if (!settings_.robot.enabled) robot_status_ = "未启用 3D 机器人。导入 URDF 和 MuJoCo XML 后可使用关节位置和 state_estimator.quat 驱动模型。";
      else if (!error.empty()) robot_status_ = error;
      else {
        importing_robot_generation_ = robot_generation_;
        const auto robot = settings_.robot;
        robot_import_ = std::async(std::launch::async, [robot] {
          return std::make_shared<const DdsRobotModel>(robot);
        });
        robot_status_ = "正在读取 URDF、MuJoCo XML 并校验关节映射…";
      }
      changed = true;
    }
    return changed;
  }

  /// 将发现到的 XTypes 支持级别 support 转为话题表中的中文状态，返回显示文本。
  static std::string xtypesLabel(DdsDiscovery::Endpoint::XTypes support) {
    switch (support) {
      case DdsDiscovery::Endpoint::XTypes::Complete: return "支持（Complete）";
      case DdsDiscovery::Endpoint::XTypes::Minimal: return "仅 Minimal（需 IDL）";
      case DdsDiscovery::Endpoint::XTypes::None: return "未提供（需 IDL）";
    }
    return "未知";
  }
  /// 按完整类型名 type 查找最近导入且匹配的 IDL 库。
  /// 返回由当前配置持有的借用指针，无匹配时返回 nullptr。
  const DdsIdlLibrary* localIdl(const std::string& type) const {
    for (auto it = settings_.idl_libraries.rbegin(); it != settings_.idl_libraries.rend(); ++it)
      if ((*it)->find(type)) return it->get();
    return nullptr;
  }
  /// 非阻塞收集已完成的 IDL 导入，空闲时从队列取出下一项。
  /// 收集结果或启动任务时返回 true，队列和任务均无进展时返回 false。
  bool pollIdlImports() {
    bool changed = false;
    if (idl_import_.valid() && idl_import_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        auto library = idl_import_.get();
        // 只应用当前配置发起的任务。重新导入成功后再替换同源定义，失败时保留已加载的库。
        if (importing_idl_generation_ == idl_generation_) {
          const auto& path = library->sourcePath();
          std::erase(settings_.idl_files, importing_idl_path_);
          std::erase(settings_.idl_files, path);
          settings_.idl_files.push_back(path);
          std::erase_if(settings_.idl_libraries, [&path](const auto& old) { return old->sourcePath() == path; });
          settings_.idl_libraries.push_back(library);
          idl_errors_.erase(importing_idl_path_);
          idl_errors_.erase(path);
          idl_status_ = "已导入 " + std::to_string(settings_.idl_libraries.size()) + " 个 IDL 文件。最近导入：" +
                        std::filesystem::path(path).filename().string() + "（" +
                        std::to_string(library->typeNames().size()) + " 个类型）。匹配状态与路径显示在话题列表中，悬停可查看完整对应关系。";
        }
      } catch (const std::exception& error) {
        if (importing_idl_generation_ == idl_generation_) {
          idl_errors_[importing_idl_path_] = error.what();
          idl_status_ = "部分 IDL 未能导入，请查看下方原因。此前成功导入的定义仍可使用。";
        }
      }
      changed = true;
    }
    if (!idl_import_.valid() && !pending_idl_files_.empty()) {
      importing_idl_path_ = std::move(pending_idl_files_.front());
      pending_idl_files_.pop_front();
      importing_idl_generation_ = idl_generation_;
      const auto path = importing_idl_path_;
      idl_import_ = std::async(std::launch::async, [path] { return DdsIdlLibrary::load(path); });
      changed = true;
    }
    return changed;
  }
  /// 返回分区 name 的显示名，空名称显示为默认分区。
  static std::string partitionLabel(const std::string& name) { return name.empty() ? "(默认分区)" : name; }

  /// 按集合 values 的迭代顺序用分隔符拼接文本，空集合返回空字符串。
  static std::string join(const std::set<std::string>& values) {
    std::string result;
    for (const auto& value : values) {
      if (!result.empty()) result += " / ";
      result += value;
    }
    return result;
  }
  /// 停止发现、清除待确认请求，并安排下次生成控件数据时重新发现，无返回值。
  void closeDiscovery() {
    discovery_.stop();
    restart_discovery_ = true;
    request_accept_ = false;
  }

  DdsSettings settings_;
  std::string config_file_path_;
  std::string config_status_ = "配置可保存为 JSON 文件，下次直接加载。";
  int settings_tab_index_ = 0;
  std::set<std::string> discovered_topics_;
  DdsDiscovery discovery_;
  std::deque<std::string> pending_idl_files_;
  std::string importing_idl_path_, idl_status_;
  std::map<std::string, std::string> idl_errors_;
  size_t idl_generation_ = 0, importing_idl_generation_ = 0;
  std::future<std::shared_ptr<const DdsIdlLibrary>> idl_import_;
  std::shared_ptr<const DdsRobotModel> imported_robot_;
  std::future<std::shared_ptr<const DdsRobotModel>> robot_import_;
  std::string robot_status_;
  size_t robot_generation_ = 0, importing_robot_generation_ = 0;
  bool robot_import_requested_ = true;
  bool restart_discovery_ = true;
  bool request_accept_ = false;
  bool request_update_history_ = false;
  std::chrono::steady_clock::time_point last_poll_{};
};
