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
  // Exact topic names, not wildcard expressions. An empty set subscribes to nothing.
  std::set<std::string> selected_topics;
  std::vector<std::string> idl_files;
  // Transient cache shared with the stream; only source paths are persisted.
  std::vector<std::shared_ptr<const DdsIdlLibrary>> idl_libraries;
  DdsRecordingSettings recording;
  DdsRobotSettings robot;

  bool matchesPartition(const std::string& offered) const {
    return fnmatch(partition.c_str(), offered.c_str(), 0) == 0 ||
           fnmatch(offered.c_str(), partition.c_str(), 0) == 0;
  }

  DdsQos readerQos() const {
    DdsQos qos(dds_create_qos());
    if (!qos) throw std::bad_alloc();
    dds_qset_reliability(qos.get(), reliable ? DDS_RELIABILITY_RELIABLE : DDS_RELIABILITY_BEST_EFFORT,
                        DDS_MSECS(max_blocking_time_ms));
    dds_qset_durability(qos.get(), transient_local ? DDS_DURABILITY_TRANSIENT_LOCAL : DDS_DURABILITY_VOLATILE);
    dds_qset_history(qos.get(), keep_all ? DDS_HISTORY_KEEP_ALL : DDS_HISTORY_KEEP_LAST, history_depth);
    return qos;
  }

  DdsQos subscriberQos() const {
    DdsQos qos(dds_create_qos());
    if (!qos) throw std::bad_alloc();
    const char* name = partition.c_str();
    dds_qset_partition(qos.get(), 1, &name);
    return qos;
  }
};
