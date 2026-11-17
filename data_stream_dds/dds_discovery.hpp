#pragma once

#include <dds/dds.h>
#include <dds/ddsi/ddsi_xqos.h>
#include <dds/ddsi/ddsi_typelib.h>
#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// 发现所有分区的发布/订阅端点。Partition 是端点 QoS，并非独立 DDS 实体。
class DdsDiscovery {
 public:
  struct Endpoint {
    enum class XTypes { None, Minimal, Complete };
    std::string topic;
    std::string type;
    std::vector<std::string> partitions;
    bool writer = false;
    std::string reliability;
    std::string durability;
    XTypes xtypes = XTypes::None;
  };

  ~DdsDiscovery() { stop(); }
  DdsDiscovery() = default;
  DdsDiscovery(const DdsDiscovery&) = delete;
  DdsDiscovery& operator=(const DdsDiscovery&) = delete;

  bool start(int domain_id) {
    stop();
    try {
      participant_ = checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                             "创建发现 Participant");
      publications_ = checked(dds_create_reader(participant_, DDS_BUILTIN_TOPIC_DCPSPUBLICATION, nullptr, nullptr),
                              "创建发布端发现 Reader");
      subscriptions_ = checked(dds_create_reader(participant_, DDS_BUILTIN_TOPIC_DCPSSUBSCRIPTION, nullptr, nullptr),
                               "创建订阅端发现 Reader");
      return true;
    } catch (const std::exception& error) {
      error_ = error.what();
      closeParticipant();
      return false;
    }
  }

  void stop() {
    closeParticipant();
    endpoints_.clear();
    error_.clear();
  }

  bool active() const { return participant_ > 0; }
  const std::string& error() const { return error_; }
  const auto& endpoints() const { return endpoints_; }

  // 由对话框 tick 非阻塞调用，无后台线程访问 UI 或对话框状态。
  bool poll() {
    if (!active()) return false;
    try {
      const bool writers_changed = drain(publications_, true);
      const bool readers_changed = drain(subscriptions_, false);
      return writers_changed || readers_changed;
    } catch (const std::exception& error) {
      error_ = error.what();
      closeParticipant();
      return true;
    }
  }

 private:
  static dds_entity_t checked(dds_entity_t entity, const char* operation) {
    if (entity < 0) throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(entity));
    return entity;
  }

  void closeParticipant() {
    if (participant_ > 0) dds_delete(participant_);
    participant_ = publications_ = subscriptions_ = DDS_ENTITY_NIL;
  }

  static Endpoint describe(const dds_builtintopic_endpoint_t& sample, bool writer) {
    Endpoint endpoint;
    endpoint.topic = sample.topic_name ? sample.topic_name : "";
    endpoint.type = sample.type_name ? sample.type_name : "";
    endpoint.writer = writer;
    const dds_typeinfo_t* info = nullptr;
    if (dds_builtintopic_get_endpoint_type_info(const_cast<dds_builtintopic_endpoint_t*>(&sample), &info) ==
        DDS_RETCODE_OK && info) {
      const auto* complete = ddsi_typeinfo_complete_typeid(info);
      const auto* minimal = ddsi_typeinfo_minimal_typeid(info);
      if (complete && ddsi_typeid_is_complete(complete)) endpoint.xtypes = Endpoint::XTypes::Complete;
      else if (minimal && ddsi_typeid_is_minimal(minimal)) endpoint.xtypes = Endpoint::XTypes::Minimal;
    }
    std::uint32_t count = 0;
    char** partitions = nullptr;
    if (dds_qget_partition(sample.qos, &count, &partitions)) {
      for (std::uint32_t i = 0; i < count; ++i) {
        endpoint.partitions.emplace_back(partitions[i] ? partitions[i] : "");
        dds_free(partitions[i]);
      }
      dds_free(partitions);
    }
    if (endpoint.partitions.empty()) endpoint.partitions.emplace_back();
    std::sort(endpoint.partitions.begin(), endpoint.partitions.end());
    endpoint.partitions.erase(std::unique(endpoint.partitions.begin(), endpoint.partitions.end()),
                              endpoint.partitions.end());
    dds_reliability_kind_t reliability;
    if (dds_qget_reliability(sample.qos, &reliability, nullptr))
      endpoint.reliability = reliability == DDS_RELIABILITY_RELIABLE ? "Reliable" : "Best effort";
    dds_durability_kind_t durability;
    if (dds_qget_durability(sample.qos, &durability)) {
      switch (durability) {
        case DDS_DURABILITY_VOLATILE: endpoint.durability = "Volatile"; break;
        case DDS_DURABILITY_TRANSIENT_LOCAL: endpoint.durability = "Transient local"; break;
        case DDS_DURABILITY_TRANSIENT: endpoint.durability = "Transient"; break;
        case DDS_DURABILITY_PERSISTENT: endpoint.durability = "Persistent"; break;
      }
    }
    return endpoint;
  }

  bool drain(dds_entity_t reader, bool writer) {
    bool changed = false;
    // 限制每个 tick 的工作量，避免大量端点发现阻塞界面。
    for (int batch = 0; batch < 4; ++batch) {
      std::array<void*, 64> samples{};
      std::array<dds_sample_info_t, 64> info{};
      const auto count = dds_take(reader, samples.data(), info.data(), samples.size(), samples.size());
      checked(count, "读取发现信息");
      if (count == 0) break;
      try {
        for (dds_return_t i = 0; i < count; ++i) {
          const auto key = std::make_pair(writer, info[i].instance_handle);
          if (info[i].instance_state != DDS_IST_ALIVE) {
            changed = endpoints_.erase(key) > 0 || changed;
          } else if (info[i].valid_data) {
            const auto& sample = *static_cast<const dds_builtintopic_endpoint_t*>(samples[i]);
            if (!sample.topic_name || std::string(sample.topic_name).rfind("DCPS", 0) == 0) continue;
            endpoints_[key] = describe(sample, writer);
            changed = true;
          }
        }
      } catch (...) {
        dds_return_loan(reader, samples.data(), count);
        throw;
      }
      checked(dds_return_loan(reader, samples.data(), count), "归还发现样本");
      if (count < static_cast<dds_return_t>(samples.size())) break;
    }
    return changed;
  }

  dds_entity_t participant_ = DDS_ENTITY_NIL;
  dds_entity_t publications_ = DDS_ENTITY_NIL;
  dds_entity_t subscriptions_ = DDS_ENTITY_NIL;
  std::map<std::pair<bool, dds_instance_handle_t>, Endpoint> endpoints_;
  std::string error_;
};
