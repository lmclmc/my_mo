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

  /// 析构时释放 DDS Participant 及其下的内置 Reader，并清除发现缓存。
  ~DdsDiscovery() { stop(); }

  /// 创建一个尚未开始发现、端点列表为空的对象。
  DdsDiscovery() = default;

  /// 禁止复制和拷贝赋值。发现对象独占 DDS 实体句柄。
  DdsDiscovery(const DdsDiscovery&) = delete;
  DdsDiscovery& operator=(const DdsDiscovery&) = delete;

  /// 清除旧发现并在 domain_id 指定的 DDS 域中创建发布端和订阅端的内置 Reader。
  /// 启动成功返回 true，失败时释放已建实体，记录原因并返回 false。
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

  /// 停止发现并清空端点和错误缓存。可重复调用，无返回值。
  void stop() {
    closeParticipant();
    endpoints_.clear();
    error_.clear();
  }

  /// 已持有用于发现的有效 Participant 时返回 true。
  bool active() const { return participant_ > 0; }

  /// 返回最近一次发现错误的只读引用，无错误时为空字符串。
  const std::string& error() const { return error_; }

  /// 返回按端点角色和实例句柄索引的缓存的只读引用。更新由后续发现轮询负责。
  const auto& endpoints() const { return endpoints_; }

  /// 由对话框周期性调用，非阻塞读取已就绪的发布端和订阅端事件。
  /// 端点缓存更新或发现出错时返回 true，便于宿主刷新表格和错误提示。
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
  /// 检查 DDS 返回值 entity。非负时原样返回，负值时抛出含 operation 名称的运行时错误。
  static dds_entity_t checked(dds_entity_t entity, const char* operation) {
    if (entity < 0) throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(entity));
    return entity;
  }

  /// 删除 Participant 及其子实体，将句柄重置为空。端点缓存和错误信息由调用方处理。
  void closeParticipant() {
    if (participant_ > 0) dds_delete(participant_);
    participant_ = publications_ = subscriptions_ = DDS_ENTITY_NIL;
  }

  /// 将内置发现样本 sample 转为独立的端点描述，writer 指定其是否为发布端。
  /// 返回值包含复制的主题、类型、分区及 QoS 信息，可在归还 DDS 样本后继续使用。
  static Endpoint describe(const dds_builtintopic_endpoint_t& sample, bool writer) {
    Endpoint endpoint;
    endpoint.topic = sample.topic_name ? sample.topic_name : "";
    endpoint.type = sample.type_name ? sample.type_name : "";
    endpoint.writer = writer;
    const dds_typeinfo_t* info = nullptr;
    // 优先记录 Complete 类型标识，只有 Minimal 时界面会提示还需本地 IDL。
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
    // 空名称代表默认分区。去重后再统计，避免同一端点在同名分区重复计数。
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

  /// 从内置 Reader reader 批量处理待取样本，writer 区分发布端和订阅端的缓存键。
  /// 插入、更新或删除缓存项时返回 true，DDS 取样或归还失败时抛出异常。
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
          // 下线事件可只带实例状态。先按缓存键删除，再对带有效数据的在线事件读取样本内容。
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
        // 样本归 Reader 所有，解析中途出现异常也要归还这批借用样本。
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
