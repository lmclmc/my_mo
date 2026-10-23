#pragma once

#include <algorithm>
#include <cctype>
#include <fstream>
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

    // Array size row
    wd.setRange("spinBox", 0, 9999);
    wd.setValue("spinBox", static_cast<int>(max_array_size_));
    wd.setChecked("radioClamp", clamp_large_arrays_);
    wd.setChecked("radioSkip", !clamp_large_arrays_);

    // Timestamp row
    wd.setChecked("radioPublishTime", !use_log_time_);
    wd.setChecked("radioLogTime", use_log_time_);
    wd.setChecked("checkBoxUseTimestamp", use_header_timestamp_);

    // Filter
    wd.setText("lineEditFilter", filter_text_);

    // Channel table — apply filter and build rows
    auto filtered = filteredChannels();
    std::vector<std::string> headers = {"Channel name", "Schema", "Encoding", "Msg Count"};
    wd.setTableHeaders("tableWidget", headers);

    std::vector<std::vector<PJ::TableItem>> rows;
    std::vector<std::string> selected_topic_names;
    std::vector<int> disabled_row_indices;
    rows.reserve(filtered.size());

    for (size_t i = 0; i < filtered.size(); ++i) {
      const auto& ch = *filtered[i];
      // Msg Count carries its native uint64 so the column sorts numerically;
      // without it the host could only compare the rendered digits.
      rows.push_back({ch.topic, ch.schema, ch.encoding, PJ::TableItem(ch.msg_count)});
      if (selected_topics_.count(ch.topic) > 0) {
        selected_topic_names.push_back(ch.topic);
      }
      bool locked_always_included = ch.msg_count > 0 && isAlwaysIncluded(ch);
      if (ch.msg_count == 0 || locked_always_included) {
        disabled_row_indices.push_back(static_cast<int>(i));
      }
      if (locked_always_included) {
        wd.setCellTooltip(
            "tableWidget", static_cast<int>(i), 0, "Always loaded — required for 3D transform rendering.");
      }
    }
    wd.setTableRows("tableWidget", rows);
    wd.setDisabledRows("tableWidget", disabled_row_indices);
    // Restore selection by first-column text (the topic name), not row index:
    // the host matches items by text, which is sort-agnostic, so the selection
    // survives the table's built-in column sorting (sortingEnabled=true).
    wd.setSelectedItems("tableWidget", selected_topic_names);

    // Explain an empty table rather than leaving it looking like a recording
    // with no topics.
    wd.setVisible("labelAnalyzeError", !analyze_error_.empty());
    if (!analyze_error_.empty()) {
      wd.setLabel("labelAnalyzeError", analyze_error_);
    }

    // LCM Topic Table — populate with user-added rows
    {
      std::vector<std::string> lcm_headers = {"Topic Name", "Port", "LCM Log File"};
      wd.setTableHeaders("topicTableWidget", lcm_headers);
      std::vector<std::vector<PJ::TableItem>> lcm_rows;
      for (const auto& cfg : topic_config_) {
        lcm_rows.push_back({cfg.topic_name, cfg.port, cfg.lcm_file_path});
      }
      wd.setTableRows("topicTableWidget", lcm_rows);

      // 高亮当前编辑行
      if (selected_row_ >= 0 && selected_row_ < static_cast<int>(topic_config_.size())) {
        wd.setSelectedRows("topicTableWidget", {selected_row_});
      }
    }

    // URDF 文件导入
    wd.setText("urdfPathEdit", urdf_file_path_);
    wd.setFilePicker("urdfBrowseBtn", "Browse",
                     "URDF 文件 (*.urdf);;所有文件 (*)", "选择 URDF 文件");

    // XML 文件导入
    wd.setText("xmlPathEdit", xml_file_path_);
    wd.setFilePicker("xmlBrowseBtn", "Browse",
                     "XML 文件 (*.xml);;所有文件 (*)", "选择 XML 文件");

    // 内联编辑栏 — 显示当前选中行的数据
    wd.setText("topicNameEdit", editing_topic_name_);
    wd.setValue("portSpinBox", editing_port_);
    wd.setText("lcmFileEdit", editing_lcm_file_);
    wd.setFilePicker("lcmFileBrowseBtn", "Browse",
                     "LCM 骨架文件 (*.lcm);;所有文件 (*)", "选择 LCM 骨架文件");

    // 配置文件的保存和加载按钮
    wd.setSaveFilePicker("btnSaveCfg", "Save Config JSON",
                         "JSON 文件 (*.json)", "保存配置文件", ".json");
    wd.setFilePicker("btnLoadCfg", "Load Config JSON",
                     "JSON 文件 (*.json)", "加载配置文件");

    // Confirm 按钮：请求关闭对话框
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

  bool onSelectionChanged(std::string_view widget_name, const std::vector<std::string>& selected) override {
    if (widget_name == "tableWidget") {
      // The table only reports the rows currently visible under the filter, so a
      // plain clear()+re-add would drop any selection the filter is hiding.
      // Reconcile only the visible topics: forget the visible ones that are no
      // longer selected, and leave everything the filter hides untouched.
      std::unordered_set<std::string> visible;
      for (const auto* ch : filteredChannels()) {
        visible.insert(ch->topic);
      }
      for (auto it = selected_topics_.begin(); it != selected_topics_.end();) {
        if (visible.count(*it) > 0) {
          it = selected_topics_.erase(it);
        } else {
          ++it;
        }
      }
      for (const auto& topic : selected) {
        selected_topics_.insert(topic);
      }
      reassertAlwaysIncluded();
      return true;  // update OK button state
    }

    if (widget_name == "topicTableWidget") {
      if (!selected.empty()) {
        // 查找选中行的 index
        for (size_t i = 0; i < topic_config_.size(); ++i) {
          if (topic_config_[i].topic_name == selected.front() ||
              (topic_config_[i].topic_name.empty() && i == topic_config_.size() - 1)) {
            selected_row_ = static_cast<int>(i);
            editing_topic_name_ = topic_config_[i].topic_name;
            editing_port_ = topic_config_[i].port.empty() ? 7667 : std::stoi(topic_config_[i].port);
            editing_lcm_file_ = topic_config_[i].lcm_file_path;
            return true;
          }
        }
        // 如果没精确匹配到，默认选第一行
        selected_row_ = 0;
        editing_topic_name_ = topic_config_[0].topic_name;
        editing_port_ = topic_config_[0].port.empty() ? 7667 : std::stoi(topic_config_[0].port);
        editing_lcm_file_ = topic_config_[0].lcm_file_path;
      } else {
        selected_row_ = -1;
      }
      return true;
    }

    return false;
  }

  bool onClicked(std::string_view widget_name) override {
    if (widget_name == "btnSelectAll") {
      auto filtered = filteredChannels();
      for (const auto* ch : filtered) {
        if (ch->msg_count > 0) {
          selected_topics_.insert(ch->topic);
        }
      }
      return true;
    }
    if (widget_name == "btnDeselectAll") {
      selected_topics_.clear();
      reassertAlwaysIncluded();
      return true;
    }

    if (widget_name == "btnAdd") {
      addRow(editing_topic_name_, editing_port_, editing_lcm_file_);
      // 自动选中新行
      selected_row_ = static_cast<int>(topic_config_.size()) - 1;
      return true;
    }
    if (widget_name == "btnDel") {
      if (selected_row_ >= 0 && selected_row_ < static_cast<int>(topic_config_.size())) {
        topic_config_.erase(topic_config_.begin() + selected_row_);
        if (topic_config_.empty()) {
          selected_row_ = -1;
        } else if (selected_row_ >= static_cast<int>(topic_config_.size())) {
          selected_row_ = static_cast<int>(topic_config_.size()) - 1;
        }
      }
      return true;
    }

    if (widget_name == "btnClear") {
      topic_config_.clear();
      selected_row_ = -1;
      return true;
    }
    if (widget_name == "btnConfirm") {
      request_accept_ = true;
      return true;
    }
    return false;
  }

  bool onFileSelected(std::string_view widget_name, std::string_view path) override {
    if (widget_name == "urdfBrowseBtn") {
      urdf_file_path_ = std::string(path);
      return true;
    }
    if (widget_name == "xmlBrowseBtn") {
      xml_file_path_ = std::string(path);
      return true;
    }
    if (widget_name == "lcmFileBrowseBtn") {
      editing_lcm_file_ = std::string(path);
      return true;
    }
    if (widget_name == "btnSaveCfg") {
      std::string json = saveConfig();
      std::ofstream file{std::string(path)};
      if (file) {
        file << json;
      }
      return true;
    }
    if (widget_name == "btnLoadCfg") {
      std::ifstream file{std::string(path)};
      if (file) {
        std::string json((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        loadUserConfig(json);
        return true;  // rebuild entire widget data
      }
      return false;
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

    return true;
  }

  bool loadUserConfig(std::string_view config_json) {
    auto cfg = nlohmann::json::parse(config_json, nullptr, false);
    if (cfg.is_discarded()) {
      return false;
    }

    urdf_file_path_ = cfg.value("urdf_file_path", std::string{});
    xml_file_path_ = cfg.value("xml_file_path", std::string{});
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

    return true;
  }

 private:
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

  struct TopicConfig {
    std::string topic_name;
    std::string port;
    std::string lcm_file_path;
  };
  std::vector<TopicConfig> topic_config_;

  void addRow(std::string& topic_name, int port, std::string& lcm_file_path) {
    topic_config_.push_back({topic_name, std::to_string(port), lcm_file_path});
  }

  // Row editor state
  int selected_row_ = -1;
  std::string editing_topic_name_;
  int editing_port_ = 7667;
  std::string editing_lcm_file_;

  // File analysis results
  std::vector<ChannelInfo> all_channels_;

  bool request_accept_ = false;
};