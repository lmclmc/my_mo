#pragma once

#include <dds/dds.h>
#include <memory>
#include <string>
#include <vector>

// Owns locally generated descriptors and their Complete TypeObjects. Keep this
// alive until every Topic/Reader and decoder using its descriptors is destroyed.
class DdsIdlLibrary {
 public:
  static std::shared_ptr<const DdsIdlLibrary> load(const std::string& path);
  ~DdsIdlLibrary();
  DdsIdlLibrary(const DdsIdlLibrary&) = delete;
  DdsIdlLibrary& operator=(const DdsIdlLibrary&) = delete;

  const std::string& sourcePath() const;
  std::vector<std::string> typeNames() const;
  const dds_topic_descriptor_t* find(const std::string& type_name) const;

 private:
  struct Impl;
  explicit DdsIdlLibrary(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};
