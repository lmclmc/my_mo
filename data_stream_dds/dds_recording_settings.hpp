#pragma once

#include <string>

struct DdsRecordingSettings {
  static constexpr int kMaxFileSizeMb = 1048576;
  static constexpr int kMaxFiles = 10000;

  bool enabled = false;
  std::string directory;
  int max_file_size_mb = 128;  // One MB is 1024 * 1024 bytes.
  int max_files = 10;

  std::string validationError() const {
    if (max_file_size_mb < 1 || max_file_size_mb > kMaxFileSizeMb)
      return "单文件录制大小超出允许范围";
    if (max_files < 1 || max_files > kMaxFiles) return "录制文件数量超出允许范围";
    if (enabled && directory.find_first_not_of(" \t\r\n") == std::string::npos)
      return "请设置录制目录";
    if (enabled && directory.find('\0') != std::string::npos) return "录制目录包含无效字符";
    return {};
  }
};
