#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
#include <limits>
#include <lcm/lcm-cpp.hpp>
#include <mcap/reader.hpp>
#include <nlohmann/json.hpp>
#include <pj_plugins/sdk/dialog_plugin_typed.hpp>
#include <pj_plugins/sdk/widget_data.hpp>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "mcap_helpers.hpp"
#include "update_history.hpp"

// Generated at configure time
#include "dialog_lcm_ui.hpp"
#include "mcap_manifest.hpp"

struct ChannelInfo {
  std::string topic;
  std::string schema;
  std::string encoding;
  uint64_t msg_count = 0;
};

struct AlwaysIncludeRule {
  std::string schema_name;
  std::string encoding;
};

/// Channels whose (schema, encoding) match a rule here are always loaded,
/// regardless of user selection -- 3D rendering (Scene3D's transform tree)
/// silently breaks without them. Keyed on message type rather than topic
/// name because topic naming isn't consistent across producers: Foxglove's
/// own foxglove.FrameTransform has no fixed topic-name convention the way
/// ROS fixes /tf (real Foxglove-recorded files use topic names like plain
/// "tf", no leading slash, or something else entirely). The mechanism here
/// is generic -- this table is just its (currently two-entry) data.
const std::vector<AlwaysIncludeRule> kAlwaysIncludeRules = {
    {"tf2_msgs/msg/TFMessage", "cdr"},
    {"foxglove.FrameTransform", "protobuf"},
};

bool isAlwaysIncluded(const ChannelInfo& ch) {
  for (const auto& rule : kAlwaysIncludeRules) {
    if (ch.schema == rule.schema_name && ch.encoding == rule.encoding) {
      return true;
    }
  }
  return false;
}


class LcmDialog : public PJ::DialogPluginTyped {
  using PJ::DialogPluginTyped::onValueChanged;
  static constexpr int kDefaultTfIntervalMs = 10;

 public:
  // --- Accessors for McapSource ---
  const std::string& filepath() const {
    return filepath_;
  }

  const std::string& xmlpath() const {
    return xml_file_path_;
  }

  const std::string& urdfpath() const {
    return urdf_file_path_;
  }

  int tfIntervalMs() const {
    return tf_interval_ms_;
  }

  unsigned maxArraySize() const {
    return max_array_size_;
  }
  bool clampLargeArrays() const {
    return clamp_large_arrays_;
  }
  /// When true, use logTime from the MCAP envelope; otherwise use publishTime.
  bool useLogTime() const {
    return use_log_time_;
  }
  /// When true, the parser should extract the timestamp from inside the message
  /// payload (e.g. ROS Header.stamp) via use_embedded_timestamp in parser config.
  bool useHeaderTimestamp() const {
    return use_header_timestamp_;
  }
  const std::unordered_set<std::string>& selectedTopics() const {
    return selected_topics_;
  }
  /// Non-empty when the file could not be analyzed (unreadable, damaged
  /// beyond the reader's fallback scan). Surfaced in the dialog so an empty
  /// channel table is never mistaken for a recording without topics.
  const std::string& analyzeError() const {
    return analyze_error_;
  }

  // --- Dialog protocol ---

  std::string manifest() const override {
    return kMcapManifest;
  }

  std::string ui_content() const override {
    return kDialogLcmUi;
  }

  std::string widget_data() override {
    PJ::WidgetData wd;

    wd.setRange("spinBox", 0, 9999);
    wd.setValue("spinBox", static_cast<int>(max_array_size_));
    wd.setChecked("radioClamp", clamp_large_arrays_);
    wd.setChecked("radioSkip", !clamp_large_arrays_);
    wd.setChecked("radioPublishTime", !use_log_time_);
    wd.setChecked("radioLogTime", use_log_time_);
    wd.setChecked("checkBoxUseTimestamp", use_header_timestamp_);
    wd.setText("lineEditFilter", filter_text_);

    // Every channel discovered in the selected lcmlog gets a row.  A row may
    // have no skeleton yet.  Such rows remain visible so users can configure
    // only the channels they need.
    const auto filtered = filteredChannels();
    wd.setTableHeaders("topicTableWidget", {"Topic Name", "Messages", "LCM Skeleton File"});
    std::vector<std::vector<PJ::TableItem>> rows;
    rows.reserve(filtered.size());
    std::vector<std::string> selected;
    for (const auto* channel : filtered) {
      const auto mapping = mappingFor(channel->topic);
      const std::string skeleton = mapping && !mapping->lcm_file_path.empty()
                                       ? mapping->lcm_file_path
                                       : std::string{"未设置"};
      rows.push_back({channel->topic, PJ::TableItem(channel->msg_count), skeleton});
      if (channel->topic == selected_topic_name_) {
        selected.push_back(channel->topic);
      }
    }

    // Preserve legacy mappings if a config was loaded before the file could
    // be scanned, or if a hand-written config contains a topic absent in this
    // recording.
    for (const auto& mapping : topic_config_) {
      if (findChannel(mapping.topic_name) != nullptr) {
        continue;
      }
      rows.push_back({mapping.topic_name, PJ::TableItem(uint64_t{0}),
                      mapping.lcm_file_path.empty() ? std::string{"未设置"} : mapping.lcm_file_path});
      if (mapping.topic_name == selected_topic_name_) {
        selected.push_back(mapping.topic_name);
      }
    }
    wd.setTableRows("topicTableWidget", rows);
    wd.setSelectedItems("topicTableWidget", selected);

    wd.setVisible("labelAnalyzeError", !analyze_error_.empty());
    if (!analyze_error_.empty()) {
      wd.setLabel("labelAnalyzeError", analyze_error_);
    }

    wd.setText("urdfPathEdit", urdf_file_path_);
    wd.setFilePicker("urdfBrowseBtn", "Browse",
                     "URDF 文件 (*.urdf);;所有文件 (*)", "选择 URDF 文件");
    wd.setText("xmlPathEdit", xml_file_path_);
    wd.setFilePicker("xmlBrowseBtn", "Browse",
                     "XML 文件 (*.xml);;所有文件 (*)", "选择 XML 文件");
    wd.setRange("tfIntervalSpinBox", 1, std::numeric_limits<int>::max());
    wd.setValue("tfIntervalSpinBox", tf_interval_ms_);

    wd.setText("topicNameEdit", editing_topic_name_);
    wd.setValue("portSpinBox", editing_port_);
    wd.setText("lcmFileEdit", editing_lcm_file_);
    wd.setFilePicker("lcmFileBrowseBtn", "选择骨架…",
                     "LCM 骨架文件 (*.lcm);;所有文件 (*)", "选择 LCM 骨架文件");
    wd.setEnabled("lcmFileBrowseBtn", !selected_topic_name_.empty());
    wd.setEnabled("btnDel", !selected_topic_name_.empty() && mappingFor(selected_topic_name_) != nullptr);
    wd.setEnabled("btnClear", !topic_config_.empty());

    wd.setSaveFilePicker("btnSaveCfg", "Save Config JSON",
                         "JSON 文件 (*.json)", "保存配置文件", ".json");
    wd.setFilePicker("btnLoadCfg", "Load Config JSON",
                     "JSON 文件 (*.json)", "加载配置文件");

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

  bool onValueChanged(std::string_view widget_name, int value) override {
    if (widget_name == "spinBox") {
      max_array_size_ = static_cast<unsigned>(std::max(0, value));
      return false;
    }
    if (widget_name == "portSpinBox") {
      editing_port_ = value;
      return false;
    }
    if (widget_name == "tfIntervalSpinBox") {
      tf_interval_ms_ = std::max(1, value);
      return false;
    }
    return false;
  }

  bool onToggled(std::string_view widget_name, bool checked) override {
    if (widget_name == "checkBoxUseTimestamp") {
      use_header_timestamp_ = checked;
      return false;
    }
    if (!checked) {
      return false;
    }
    if (widget_name == "radioClamp") {
      clamp_large_arrays_ = true;
      return true;
    }
    if (widget_name == "radioSkip") {
      clamp_large_arrays_ = false;
      return true;
    }
    if (widget_name == "radioPublishTime") {
      use_log_time_ = false;
      return false;
    }
    if (widget_name == "radioLogTime") {
      use_log_time_ = true;
      return false;
    }
    return false;
  }

  bool onTextChanged(std::string_view widget_name, std::string_view text) override {
    if (widget_name == "lineEditFilter") {
      filter_text_ = std::string(text);
      return true;  // rebuild table with filtered rows
    }
    if (widget_name == "topicNameEdit") {
      editing_topic_name_ = std::string(text);
      return false;
    }
    if (widget_name == "urdfPathEdit") {
      urdf_file_path_ = std::string(text);
      return false;
    }
    if (widget_name == "xmlPathEdit") {
      xml_file_path_ = std::string(text);
      return false;
    }
    if (widget_name == "lcmFileEdit") {
      editing_lcm_file_ = std::string(text);
      return false;
    }
    return false;
  }

  bool onSelectionChanged(std::string_view widget_name,
                          const std::vector<std::string>& selected) override {
    if (widget_name != "topicTableWidget") {
      return false;
    }

    selected_topic_name_ = selected.empty() ? std::string{} : selected.front();
    selected_row_ = -1;
    if (selected_topic_name_.empty()) {
      editing_topic_name_.clear();
      editing_lcm_file_.clear();
      editing_port_ = 7667;
      return true;
    }

    for (size_t i = 0; i < topic_config_.size(); ++i) {
      if (topic_config_[i].topic_name == selected_topic_name_) {
        selected_row_ = static_cast<int>(i);
        break;
      }
    }
    editing_topic_name_ = selected_topic_name_;
    if (const auto mapping = mappingFor(selected_topic_name_)) {
      editing_lcm_file_ = mapping->lcm_file_path;
      editing_port_ = parsePort(mapping->port);
    } else {
      editing_lcm_file_.clear();
      editing_port_ = 7667;
    }
    return true;
  }

  bool onClicked(std::string_view widget_name) override {
    if (widget_name == "updateHistoryButton") {
      request_update_history_ = true;
      return true;
    }
    if (widget_name == "btnSelectAll") {
      for (const auto& channel : all_channels_) {
        if (channel.msg_count > 0) {
          selected_topics_.insert(channel.topic);
        }
      }
      return true;
    }
    if (widget_name == "btnDeselectAll") {
      selected_topics_.clear();
      return true;
    }

    if (widget_name == "btnAdd") {
      // Keep the legacy manual-add entry point useful for recordings whose
      // channel list could not be scanned. A mapping without a skeleton is
      // deliberately not stored and therefore cannot be parsed.
      if (!editing_topic_name_.empty() && !editing_lcm_file_.empty()) {
        assignMapping(editing_topic_name_, editing_port_, editing_lcm_file_);
      }
      selected_topic_name_ = editing_topic_name_;
      return true;
    }
    if (widget_name == "btnDel") {
      if (!selected_topic_name_.empty()) {
        clearMapping(selected_topic_name_);
        editing_lcm_file_.clear();
      }
      return true;
    }
    if (widget_name == "btnClear") {
      topic_config_.clear();
      selected_row_ = -1;
      editing_lcm_file_.clear();
      return true;
    }
    if (widget_name == "btnConfirm") {
      request_accept_ = true;
      return true;
    }
    return false;
  }

  bool onFileSelected(std::string_view widget_name, std::string_view path) override {
    if (path.empty()) {
      return false;
    }
    if (widget_name == "urdfBrowseBtn") {
      urdf_file_path_ = std::string(path);
      return true;
    }
    if (widget_name == "xmlBrowseBtn") {
      xml_file_path_ = std::string(path);
      return true;
    }
    if (widget_name == "lcmFileBrowseBtn") {
      if (selected_topic_name_.empty()) {
        return false;
      }
      editing_lcm_file_ = std::string(path);
      editing_topic_name_ = selected_topic_name_;
      assignMapping(selected_topic_name_, editing_port_, editing_lcm_file_);
      return true;
    }
    if (widget_name == "btnSaveCfg") {
      std::ofstream file{std::string(path)};
      if (!file) {
        return false;
      }
      file << saveConfig();
      return file.good();
    }
    if (widget_name == "btnLoadCfg") {
      std::ifstream file{std::string(path)};
      if (!file) {
        return false;
      }
      std::string json((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
      return loadUserConfig(json);
    }
    return false;
  }

  void onAccepted(std::string_view /*json*/) override {}
  void onRejected() override {}

  const auto& getTopicConfig() const {
    return topic_config_;
  }

  std::string saveConfig() const override {
    nlohmann::json cfg;
    cfg["filepath"] = filepath_;
    cfg["urdf_file_path"] = urdf_file_path_;
    cfg["xml_file_path"] = xml_file_path_;
    cfg["tf_interval_ms"] = tf_interval_ms_;
    cfg["max_array_size"] = max_array_size_;
    cfg["clamp_large_arrays"] = clamp_large_arrays_;
    cfg["use_log_time"] = use_log_time_;
    cfg["use_header_timestamp"] = use_header_timestamp_;
    cfg["selected_topics"] = std::vector<std::string>(selected_topics_.begin(), selected_topics_.end());

    nlohmann::json topic_arr = nlohmann::json::array();
    for (const auto& tc : topic_config_) {
      topic_arr.push_back({{"topic_name", tc.topic_name},
                           {"port", tc.port},
                           {"lcm_file_path", tc.lcm_file_path}});
    }
    cfg["topic_config"] = topic_arr;

    return cfg.dump();
  }

  bool loadConfig(std::string_view config_json) override {
    auto cfg = nlohmann::json::parse(config_json, nullptr, false);
    if (cfg.is_discarded()) {
      return false;
    }

    filepath_ = cfg.value("filepath", std::string{});
    urdf_file_path_ = cfg.value("urdf_file_path", std::string{});
    xml_file_path_ = cfg.value("xml_file_path", std::string{});
    readTfInterval(cfg);
    max_array_size_ = cfg.value("max_array_size", 500u);
    clamp_large_arrays_ = cfg.value("clamp_large_arrays", true);
    use_log_time_ = cfg.value("use_log_time", cfg.value("use_mcap_log_time", false));
    // Backward compatibility: old MCAP configs used "use_timestamp" for the
    // parser-level embedded timestamp option before the controls were split.
    use_header_timestamp_ = cfg.value("use_header_timestamp", cfg.value("use_timestamp", false));

    selected_topics_.clear();
    if (auto it = cfg.find("selected_topics"); it != cfg.end() && it->is_array()) {
      for (const auto& t : *it) {
        if (t.is_string()) {
          selected_topics_.insert(t.get<std::string>());
        }
      }
    }

    topic_config_.clear();
    if (auto it = cfg.find("topic_config"); it != cfg.end() && it->is_array()) {
      for (const auto& item : *it) {
        TopicConfig tc;
        tc.topic_name = item.value("topic_name", std::string{});
        tc.port = item.value("port", std::string{});
        tc.lcm_file_path = item.value("lcm_file_path", std::string{});
        topic_config_.push_back(std::move(tc));
      }
    }

    selected_topic_name_.clear();
    editing_topic_name_.clear();
    editing_lcm_file_.clear();
    editing_port_ = 7667;
    analyzeFile();

    return true;
  }

  bool loadUserConfig(std::string_view config_json) {
    auto cfg = nlohmann::json::parse(config_json, nullptr, false);
    if (cfg.is_discarded()) {
      return false;
    }

    urdf_file_path_ = cfg.value("urdf_file_path", std::string{});
    xml_file_path_ = cfg.value("xml_file_path", std::string{});
    readTfInterval(cfg);
    max_array_size_ = cfg.value("max_array_size", 500u);
    clamp_large_arrays_ = cfg.value("clamp_large_arrays", true);
    use_log_time_ = cfg.value("use_log_time", cfg.value("use_mcap_log_time", false));
    // Backward compatibility: old MCAP configs used "use_timestamp" for the
    // parser-level embedded timestamp option before the controls were split.
    use_header_timestamp_ = cfg.value("use_header_timestamp", cfg.value("use_timestamp", false));

    selected_topics_.clear();
    if (auto it = cfg.find("selected_topics"); it != cfg.end() && it->is_array()) {
      for (const auto& t : *it) {
        if (t.is_string()) {
          selected_topics_.insert(t.get<std::string>());
        }
      }
    }

    topic_config_.clear();
    if (auto it = cfg.find("topic_config"); it != cfg.end() && it->is_array()) {
      for (const auto& item : *it) {
        TopicConfig tc;
        tc.topic_name = item.value("topic_name", std::string{});
        tc.port = item.value("port", std::string{});
        tc.lcm_file_path = item.value("lcm_file_path", std::string{});
        topic_config_.push_back(std::move(tc));
      }
    }

    selected_topic_name_.clear();
    editing_topic_name_.clear();
    editing_lcm_file_.clear();
    editing_port_ = 7667;
    analyzeFile();

    return true;
  }

 private:
  struct TopicConfig {
    std::string topic_name;
    std::string port;
    std::string lcm_file_path;
  };

  static int parsePort(const std::string& value) {
    if (value.empty()) {
      return 7667;
    }
    try {
      const int port = std::stoi(value);
      return std::clamp(port, 1, 65535);
    } catch (...) {
      return 7667;
    }
  }

  const ChannelInfo* findChannel(std::string_view topic) const {
    for (const auto& channel : all_channels_) {
      if (channel.topic == topic) {
        return &channel;
      }
    }
    return nullptr;
  }

  const TopicConfig* mappingFor(std::string_view topic) const {
    for (const auto& mapping : topic_config_) {
      if (mapping.topic_name == topic) {
        return &mapping;
      }
    }
    return nullptr;
  }

  void assignMapping(std::string_view topic, int port, std::string_view path) {
    if (topic.empty() || path.empty()) {
      return;
    }
    for (auto& mapping : topic_config_) {
      if (mapping.topic_name == topic) {
        mapping.port = std::to_string(std::clamp(port, 1, 65535));
        mapping.lcm_file_path = path;
        selected_row_ = static_cast<int>(&mapping - topic_config_.data());
        return;
      }
    }
    topic_config_.push_back({std::string(topic), std::to_string(std::clamp(port, 1, 65535)),
                             std::string(path)});
    selected_row_ = static_cast<int>(topic_config_.size()) - 1;
  }

  void clearMapping(std::string_view topic) {
    topic_config_.erase(std::remove_if(topic_config_.begin(), topic_config_.end(),
                                       [topic](const TopicConfig& mapping) {
                                         return mapping.topic_name == topic;
                                       }),
                       topic_config_.end());
    selected_row_ = -1;
  }

  void analyzeFile() {
    all_channels_.clear();
    analyze_error_.clear();
    if (filepath_.empty()) {
      return;
    }

    lcm::LogFile log(filepath_, "r");
    if (!log.good()) {
      analyze_error_ = "无法打开 LCM 日志文件：" + filepath_;
      return;
    }

    std::unordered_map<std::string, size_t> channel_indexes;
    while (const auto* event = log.readNextEvent()) {
      auto [it, inserted] = channel_indexes.emplace(event->channel, all_channels_.size());
      if (inserted) {
        all_channels_.push_back({event->channel, {}, "lcm", 0});
      }
      ++all_channels_[it->second].msg_count;
    }

    std::sort(all_channels_.begin(), all_channels_.end(),
              [](const ChannelInfo& lhs, const ChannelInfo& rhs) { return lhs.topic < rhs.topic; });
    if (all_channels_.empty()) {
      analyze_error_ = "LCM 日志中没有找到话题，或日志已损坏。";
    }
  }

  void readTfInterval(const nlohmann::json& cfg) {
    tf_interval_ms_ = kDefaultTfIntervalMs;
    if (const auto it = cfg.find("tf_interval_ms"); it != cfg.end() && it->is_number_integer()) {
      const double max_interval = std::numeric_limits<int>::max();
      tf_interval_ms_ = static_cast<int>(std::clamp(it->get<double>(), 1.0, max_interval));
    }
  }

  /// Channels matching kAlwaysIncludeRules are always loaded: 3D rendering
  /// depends on them even when the user narrows their selection to a handful
  /// of unrelated topics. Idempotent -- safe to call after any mutation of
  /// selected_topics_.
  void reassertAlwaysIncluded() {
    for (const auto& ch : all_channels_) {
      if (ch.msg_count > 0 && isAlwaysIncluded(ch)) {
        selected_topics_.insert(ch.topic);
      }
    }
  }

  std::vector<const ChannelInfo*> filteredChannels() const {
    std::vector<const ChannelInfo*> result;
    if (filter_text_.empty()) {
      for (const auto& ch : all_channels_) {
        result.push_back(&ch);
      }
      return result;
    }

    auto lower = [](std::string s) {
      for (auto& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      }
      return s;
    };

    // Split filter by spaces — AND logic (all words must match), matched
    // case-insensitively like the streaming pickers' filters.
    std::vector<std::string> words;
    std::string word;
    for (char c : filter_text_) {
      if (c == ' ') {
        if (!word.empty()) {
          words.push_back(lower(word));
          word.clear();
        }
      } else {
        word += c;
      }
    }
    if (!word.empty()) {
      words.push_back(lower(word));
    }

    for (const auto& ch : all_channels_) {
      const std::string topic = lower(ch.topic);
      bool match = true;
      for (const auto& w : words) {
        if (topic.find(w) == std::string::npos) {
          match = false;
          break;
        }
      }
      if (match) {
        result.push_back(&ch);
      }
    }
    return result;
  }

  // Config state
  std::string analyze_error_;
  std::string filepath_;
  unsigned max_array_size_ = 500;
  bool clamp_large_arrays_ = true;
  bool use_log_time_ = false;
  bool use_header_timestamp_ = false;
  std::unordered_set<std::string> selected_topics_;
  std::string filter_text_;
  std::string urdf_file_path_;
  std::string xml_file_path_;
  int tf_interval_ms_ = kDefaultTfIntervalMs;

  std::vector<TopicConfig> topic_config_;

  void addRow(std::string& topic_name, int port, std::string& lcm_file_path) {
    topic_config_.push_back({topic_name, std::to_string(port), lcm_file_path});
  }

  // Row editor state
  int selected_row_ = -1;
  std::string selected_topic_name_;
  std::string editing_topic_name_;
  int editing_port_ = 7667;
  std::string editing_lcm_file_;

  // File analysis results
  std::vector<ChannelInfo> all_channels_;

  bool request_accept_ = false;
  bool request_update_history_ = false;
};
