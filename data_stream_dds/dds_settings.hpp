#pragma once

#include "dds_recording_settings.hpp"
#include "dds_robot_settings.hpp"

#include <dds/dds.h>
#include <cstdint>
#include <fnmatch.h>
#include <memory>
#include <set>
#include <string>
#include <vector>

class DdsIdlLibrary;

struct DdsQosDeleter {
  /// 供 DdsQos 释放自己持有的 qos，使用 CycloneDDS 的配套释放函数，无返回值。
  void operator()(dds_qos_t* qos) const { dds_delete_qos(qos); }
};
using DdsQos = std::unique_ptr<dds_qos_t, DdsQosDeleter>;

struct DdsSettings {
  int domain_id = 200;
  bool reliable = false;
  bool transient_local = false;
  bool keep_all = false;
  int history_depth = 1;
  int max_blocking_time_ms = 100;
  std::string partition;
  // 保存完整话题名。空集合表示当前没有话题要订阅。
  std::set<std::string> selected_topics;
  std::vector<std::string> idl_files;
  // 导入后由对话框和数据流共享的类型库缓存，持久化配置只保存来源路径。
  std::vector<std::shared_ptr<const DdsIdlLibrary>> idl_libraries;
  DdsRecordingSettings recording;
  DdsRobotSettings robot;

  /// 检查发现端点声明的分区 offered 是否与当前分区匹配。
  /// 两侧分别作为通配模式匹配一次，任一方向匹配就返回 true。
  bool matchesPartition(const std::string& offered) const {
    return fnmatch(partition.c_str(), offered.c_str(), 0) == 0 ||
           fnmatch(offered.c_str(), partition.c_str(), 0) == 0;
  }

  /// 根据当前设置生成 Reader 的可靠性、持久性和历史策略。
  /// 返回独占的 QoS 对象，分配失败时抛出 std::bad_alloc。
  DdsQos readerQos() const {
    DdsQos qos(dds_create_qos());
    if (!qos) throw std::bad_alloc();
    dds_qset_reliability(qos.get(), reliable ? DDS_RELIABILITY_RELIABLE : DDS_RELIABILITY_BEST_EFFORT,
                        DDS_MSECS(max_blocking_time_ms));
    dds_qset_durability(qos.get(), transient_local ? DDS_DURABILITY_TRANSIENT_LOCAL : DDS_DURABILITY_VOLATILE);
    dds_qset_history(qos.get(), keep_all ? DDS_HISTORY_KEEP_ALL : DDS_HISTORY_KEEP_LAST, history_depth);
    return qos;
  }

  /// 根据当前分区生成 Subscriber QoS，空名称对应默认分区。
  /// 返回独占的 QoS 对象，分配失败时抛出 std::bad_alloc。
  DdsQos subscriberQos() const {
    DdsQos qos(dds_create_qos());
    if (!qos) throw std::bad_alloc();
    const char* name = partition.c_str();
    dds_qset_partition(qos.get(), 1, &name);
    return qos;
  }
};
