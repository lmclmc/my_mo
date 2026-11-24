#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <charconv>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <lcm/lcm-cpp.hpp>
#include <nlohmann/json.hpp>
#include <pj_plugins/sdk/dialog_plugin_typed.hpp>
#include <pj_plugins/sdk/widget_data.hpp>

#include "datastream_lcm_ui.hpp"
#include "lcm_endpoint.hpp"
#include "mcap_manifest.hpp"
#include "update_history.hpp"

class LcmDialog : public PJ::DialogPluginTyped {
  using PJ::DialogPluginTyped::onValueChanged;
  using TopicKey = std::pair<int, std::string>;

  struct TopicConfig {
    std::string topic_name;
    std::string port;
    std::string lcm_file_path;
  };

  static constexpr int kDefaultTfIntervalMs = 10;

 public:
  const std::string& xmlpath() const {
    return xml_file_path_;
  }

  const std::string& urdfpath() const {
    return urdf_file_path_;
  }

  int tfIntervalMs() const {
    return tf_interval_ms_;
  }

  const std::string& log_save_path() const {
    return log_save_path_;
  }

  int log_max_size() const {
    return log_max_size_ * 1024 * 1024;
  }

  int log_max_files_num() const {
    return log_max_files_num_;
  }

  const auto& getTopicConfig() const {
    return topic_config_;
  }

  std::set<int> listeningPorts() const {
    auto ports = discovery_ports_;
    ports.insert(lcm_stream::kDefaultLcmPort);
    for (const auto& topic : topic_config_) {
      if (const auto port = lcm_stream::parseLcmPort(topic.port)) {
        ports.insert(*port);
      }
    }
    return ports;
  }

  std::string manifest() const override {
    return kMcapManifest;
  }

  std::string ui_content() const override {
    return kDataStreamLcmUi;
  }

  std::string widget_data() override {
    discovery_active_ = true;
    updateDiscoveryPorts();
    PJ::WidgetData wd;
    wd.setText("urdfPathEdit", urdf_file_path_);
    wd.setFilePicker("urdfBrowseBtn", "Browse", "URDF 文件 (*.urdf);;所有文件 (*)", "选择 URDF 文件");
    wd.setText("xmlPathEdit", xml_file_path_);
    wd.setFilePicker("xmlBrowseBtn", "Browse", "XML 文件 (*.xml);;所有文件 (*)", "选择 XML 文件");
    wd.setRange("tfIntervalSpinBox", 1, std::numeric_limits<int>::max());
    wd.setValue("tfIntervalSpinBox", tf_interval_ms_);

    wd.setText("logPathEdit", log_save_path_);
    wd.setSaveFilePicker("logPathBrowseBtn", "Browse", "所有文件 (*)", "选择日志保存路径", ".lcmlog");
    wd.setText("logMaxSizeEdit", std::to_string(log_max_size_));
    wd.setText("logMaxFilesEdit", std::to_string(log_max_files_num_));

    std::set<TopicKey> topics = discovered_topics_;
    topics.insert(manual_topics_.begin(), manual_topics_.end());
    std::map<TopicKey, std::string> schema_paths;
    for (const auto& config : topic_config_) {
      if (const auto port = lcm_stream::parseLcmPort(config.port)) {
        const TopicKey key{*port, config.topic_name};
        topics.insert(key);
        auto& paths = schema_paths[key];
        if (!paths.empty()) {
          paths += " | ";
        }
        paths += config.lcm_file_path;
      }
    }

    wd.setTableHeaders("topicTableWidget", {"LCM 话题", "端口", "LCM 骨架文件"});
    std::vector<std::vector<PJ::TableItem>> rows;
    rows.reserve(topics.size());
    std::vector<std::string> selected;
    std::string selected_path;
    topic_keys_.clear();
    for (const auto& key : topics) {
      const auto& [port, topic] = key;
      const std::string label = topic + " [" + std::to_string(port) + "]";
      topic_keys_.emplace(label, key);
      const auto schema = schema_paths.find(key);
      const std::string file = schema == schema_paths.end() ? "未设置" : schema->second;
      rows.push_back({label, std::to_string(port), file});
      if (selected_topic_ == key) {
        selected.push_back(label);
        if (schema != schema_paths.end()) {
          selected_path = file;
        }
      }
    }
    wd.setTableRows("topicTableWidget", rows);
    wd.setSelectedItems("topicTableWidget", selected);
    wd.setLabel("discoveryStatusLabel", discoveryStatus());

    wd.setText("topicNameEdit", manual_topic_name_);
    wd.setValue("portSpinBox", editing_port_);
    wd.setText("lcmFileEdit", selected_path);
    wd.setFilePicker("lcmFileBrowseBtn", "选择骨架…", "LCM 骨架文件 (*.lcm);;所有文件 (*)", "选择 LCM 骨架文件");
    wd.setEnabled("lcmFileBrowseBtn", !selected.empty());
    wd.setEnabled("btnDel", !selected_path.empty());
    wd.setEnabled("btnClear", !topic_config_.empty());
    wd.setEnabled("btnClearTopics", !topics.empty());

    wd.setSaveFilePicker("btnSaveCfg", "Save Config JSON", "JSON 文件 (*.json)", "保存配置文件", ".json");
    wd.setFilePicker("btnLoadCfg", "Load Config JSON", "JSON 文件 (*.json)", "加载配置文件");
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

  bool onTick() override {
    if (!discovery_active_) {
      return false;
    }
    const auto previous_topic_count = discovered_topics_.size();
    bool listener_failed = false;
    for (auto it = listeners_.begin(); it != listeners_.end();) {
      int handled = 0;
      // Limit the messages drained on each UI tick even on a busy bus.
      for (int message = 0; message < 128; ++message) {
        handled = it->second->transport.handleTimeout(0);
        if (handled <= 0) {
          break;
        }
      }
      if (handled < 0) {
        failed_ports_.insert(it->first);
        it = listeners_.erase(it);
        listener_failed = true;
      } else {
        ++it;
      }
    }
    return listener_failed || discovered_topics_.size() != previous_topic_count;
  }

  bool onValueChanged(std::string_view widget_name, int value) override {
    if (widget_name == "portSpinBox") {
      editing_port_ = std::clamp(value, 1, 65535);
    } else if (widget_name == "tfIntervalSpinBox") {
      tf_interval_ms_ = std::max(1, value);
    }
    return false;
  }

  bool onTextChanged(std::string_view widget_name, std::string_view text) override {
    if (widget_name == "topicNameEdit") {
      manual_topic_name_ = text;
    } else if (widget_name == "urdfPathEdit") {
      urdf_file_path_ = text;
    } else if (widget_name == "xmlPathEdit") {
      xml_file_path_ = text;
    } else if (widget_name == "logPathEdit") {
      log_save_path_ = text;
    } else if (widget_name == "logMaxSizeEdit") {
      if (const auto value = parsePositiveInt(text)) {
        log_max_size_ = std::min(*value, std::numeric_limits<int>::max() / (1024 * 1024));
      }
    } else if (widget_name == "logMaxFilesEdit") {
      if (const auto value = parsePositiveInt(text)) {
        log_max_files_num_ = *value;
      }
    }
    return false;
  }

  bool onSelectionChanged(std::string_view widget_name, const std::vector<std::string>& selected) override {
    if (widget_name != "topicTableWidget") {
      return false;
    }
    const auto match = selected.empty() ? topic_keys_.end() : topic_keys_.find(selected.front());
    selected_topic_ = match == topic_keys_.end() ? std::nullopt : std::optional<TopicKey>{match->second};
    return true;
  }

  bool onClicked(std::string_view widget_name) override {
    if (widget_name == "updateHistoryButton") {
      request_update_history_ = true;
    } else if (widget_name == "btnListenPort") {
      discovery_ports_.insert(editing_port_);
      failed_ports_.erase(editing_port_);
    } else if (widget_name == "btnAdd") {
      if (manual_topic_name_.empty()) {
        return false;
      }
      discovery_ports_.insert(editing_port_);
      failed_ports_.erase(editing_port_);
      selected_topic_ = TopicKey{editing_port_, manual_topic_name_};
      manual_topics_.insert(*selected_topic_);
      manual_topic_name_.clear();
    } else if (widget_name == "btnDel") {
      if (!selected_topic_) {
        return false;
      }
      clearMapping(*selected_topic_);
    } else if (widget_name == "btnClear") {
      for (const auto& mapping : topic_config_) {
        if (const auto port = lcm_stream::parseLcmPort(mapping.port)) {
          discovery_ports_.insert(*port);
          manual_topics_.emplace(*port, mapping.topic_name);
        }
      }
      topic_config_.clear();
    } else if (widget_name == "btnClearTopics") {
      discovery_cutoff_us_ = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      discovered_topics_.clear();
      manual_topics_.clear();
      topic_config_.clear();
      topic_keys_.clear();
      selected_topic_.reset();
      manual_topic_name_.clear();
    } else if (widget_name == "btnConfirm") {
      request_accept_ = true;
    } else {
      return false;
    }
    return true;
  }

  bool onFileSelected(std::string_view widget_name, std::string_view path) override {
    if (path.empty()) {
      return false;
    }
    if (widget_name == "urdfBrowseBtn") {
      urdf_file_path_ = path;
    } else if (widget_name == "xmlBrowseBtn") {
      xml_file_path_ = path;
    } else if (widget_name == "lcmFileBrowseBtn") {
      if (!selected_topic_) {
        return false;
      }
      const auto& [port, name] = *selected_topic_;
      clearMapping(*selected_topic_);
      topic_config_.push_back({name, std::to_string(port), std::string(path)});
    } else if (widget_name == "logPathBrowseBtn") {
      log_save_path_ = path;
    } else if (widget_name == "btnSaveCfg") {
      std::ofstream file{std::filesystem::path(path)};
      if (!file) {
        return false;
      }
      file << saveConfig();
      return file.good();
    } else if (widget_name == "btnLoadCfg") {
      std::ifstream file{std::filesystem::path(path)};
      if (!file) {
        return false;
      }
      const std::string json((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
      return loadConfig(json);
    } else {
      return false;
    }
    return true;
  }

  void onAccepted(std::string_view /*json*/) override {
    stopDiscovery();
  }

  void onRejected() override {
    stopDiscovery();
  }

  std::string saveConfig() const override {
    auto config = saved_config_;
    config["urdf_file_path"] = urdf_file_path_;
    config["xml_file_path"] = xml_file_path_;
    config["tf_interval_ms"] = tf_interval_ms_;
    config["log_save_path"] = log_save_path_;
    config["log_max_size"] = log_max_size_;
    config["log_max_files_num"] = log_max_files_num_;
    config["discovery_ports"] = discovery_ports_;
    auto topics = nlohmann::json::array();
    for (const auto& item : topic_config_) {
      topics.push_back({{"topic_name", item.topic_name}, {"port", item.port}, {"lcm_file_path", item.lcm_file_path}});
    }
    config["topic_config"] = std::move(topics);
    return config.dump();
  }

  bool loadConfig(std::string_view config_json) override {
    auto config = nlohmann::json::parse(config_json, nullptr, false);
    if (!config.is_object()) {
      return false;
    }
    std::set<int> ports;
    std::vector<TopicConfig> topics;
    try {
      if (const auto it = config.find("discovery_ports"); it != config.end() && it->is_array()) {
        for (const auto& entry : *it) {
          if (entry.is_number_integer() && entry >= 1 && entry <= 65535) {
            ports.insert(entry.get<int>());
          }
        }
      }
      if (const auto it = config.find("topic_config"); it != config.end() && it->is_array()) {
        for (const auto& item : *it) {
          if (!item.is_object()) {
            return false;
          }
          auto name = item.value("topic_name", std::string{});
          auto file = item.value("lcm_file_path", std::string{});
          const auto text = item.value("port", std::string{});
          const auto port = lcm_stream::parseLcmPort(text);
          if (port && !name.empty()) {
            ports.insert(*port);
            if (!file.empty()) {
              topics.push_back({std::move(name), std::to_string(*port), std::move(file)});
            }
          }
        }
      }
      auto urdf_path = config.value("urdf_file_path", std::string{});
      auto xml_path = config.value("xml_file_path", std::string{});
      auto log_path = config.value("log_save_path", std::string{});
      const auto log_size = std::clamp(config.value("log_max_size", 100), 1,
                                       std::numeric_limits<int>::max() / (1024 * 1024));
      const auto log_files = std::max(1, config.value("log_max_files_num", 10));
      int tf_interval = kDefaultTfIntervalMs;
      if (const auto it = config.find("tf_interval_ms"); it != config.end() && it->is_number_integer()) {
        tf_interval = static_cast<int>(std::clamp(it->get<double>(), 1.0,
                                                 static_cast<double>(std::numeric_limits<int>::max())));
      }
      urdf_file_path_ = std::move(urdf_path);
      xml_file_path_ = std::move(xml_path);
      log_save_path_ = std::move(log_path);
      log_max_size_ = log_size;
      log_max_files_num_ = log_files;
      tf_interval_ms_ = tf_interval;
    } catch (const nlohmann::json::exception&) {
      return false;
    }
    discovery_ports_ = std::move(ports);
    topic_config_ = std::move(topics);
    saved_config_ = std::move(config);
    failed_ports_.clear();
    manual_topics_.clear();
    selected_topic_.reset();
    return true;
  }

 private:
  struct DiscoveryPort {
    LcmDialog& dialog;
    int port;
    lcm::LCM transport;

    DiscoveryPort(LcmDialog& owner, int number)
      : dialog(owner), port(number), transport(lcm_stream::lcmUrl(number)) {}

    void onMessage(const lcm::ReceiveBuffer* buffer, const std::string& name) {
      // Only messages received after a reset can repopulate the list.
      if (buffer->recv_utime >= dialog.discovery_cutoff_us_) {
        dialog.discovered_topics_.emplace(port, name);
      }
    }

    bool subscribe() {
      if (!transport.good() || transport.getFileno() < 0) {
        return false;
      }
      auto* subscription = transport.subscribe(".*", &DiscoveryPort::onMessage, this);
      return subscription && subscription->setQueueCapacity(256) == 0;
    }
  };

  static std::optional<int> parsePositiveInt(std::string_view text) {
    if (text.empty()) {
      return std::nullopt;
    }
    int result = 0;
    const auto* end = text.data() + text.size();
    const auto [next, error] = std::from_chars(text.data(), end, result);
    if (error != std::errc{} || next != end || result < 1) {
      return std::nullopt;
    }
    return result;
  }

  void updateDiscoveryPorts() {
    const auto ports = listeningPorts();
    std::erase_if(listeners_, [&](const auto& entry) { return !ports.contains(entry.first); });
    std::erase_if(discovered_topics_, [&](const TopicKey& key) { return !ports.contains(key.first); });
    for (int port : ports) {
      if (listeners_.contains(port) || failed_ports_.contains(port)) {
        continue;
      }
      auto listener = std::make_unique<DiscoveryPort>(*this, port);
      if (listener->subscribe()) {
        listeners_.emplace(port, std::move(listener));
      } else {
        failed_ports_.insert(port);
      }
    }
  }

  std::string discoveryStatus() const {
    std::string status = "监听端口";
    for (const auto& [port, listener] : listeners_) {
      (void)listener;
      status += " " + std::to_string(port);
    }
    if (listeners_.empty()) {
      status += " 暂无";
    } else {
      status += " · 已发现 " + std::to_string(discovered_topics_.size()) + " 个话题";
    }
    if (!failed_ports_.empty()) {
      status += " · 无法监听端口";
      for (const int port : failed_ports_) {
        status += " " + std::to_string(port);
      }
    }
    return status;
  }

  void clearMapping(const TopicKey& key) {
    const auto& [port, name] = key;
    const std::string port_text = std::to_string(port);
    manual_topics_.insert(key);
    discovery_ports_.insert(port);
    std::erase_if(topic_config_, [&](const TopicConfig& config) {
      return config.topic_name == name && config.port == port_text;
    });
  }

  void stopDiscovery() {
    discovery_active_ = false;
    listeners_.clear();
    discovered_topics_.clear();
    failed_ports_.clear();
  }

  nlohmann::json saved_config_ = nlohmann::json::object();
  std::string urdf_file_path_;
  std::string xml_file_path_;
  int tf_interval_ms_ = kDefaultTfIntervalMs;
  std::string log_save_path_;
  int log_max_size_ = 100;
  int log_max_files_num_ = 10;
  std::vector<TopicConfig> topic_config_;

  std::set<int> discovery_ports_;
  std::set<TopicKey> manual_topics_;
  std::set<TopicKey> discovered_topics_;
  std::int64_t discovery_cutoff_us_ = 0;
  std::set<int> failed_ports_;
  std::map<int, std::unique_ptr<DiscoveryPort>> listeners_;
  std::map<std::string, TopicKey> topic_keys_;
  std::optional<TopicKey> selected_topic_;
  int editing_port_ = lcm_stream::kDefaultLcmPort;
  std::string manual_topic_name_;

  bool discovery_active_ = false;
  bool request_accept_ = false;
  bool request_update_history_ = false;
};
