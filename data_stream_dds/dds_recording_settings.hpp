#pragma once

#include <string>

struct DdsRecordingSettings {
  static constexpr int kMaxFileSizeMb = 1048576;
  static constexpr int kMaxFiles = 10000;

  bool enabled = false;
  std::string directory;
  int max_file_size_mb = 128;  // 界面上的 1 MB 按 1024 × 1024 字节计算。
  int max_files = 10;

  /// 检查当前的单文件大小与文件数量。启用录制时同时检查目录字符串。
  /// @return 第一个配置错误的中文说明。所有字段通过检查时返回空字符串。
  std::string validationError() const {
    if (max_file_size_mb < 1 || max_file_size_mb > kMaxFileSizeMb)
      return "单文件录制大小超出允许范围";
    if (max_files < 1 || max_files > kMaxFiles) return "录制文件数量超出允许范围";
    // 路径需要有有效字符，且必须能完整传给接受 C 字符串的文件系统接口。
    if (enabled && directory.find_first_not_of(" \t\r\n") == std::string::npos)
      return "请设置录制目录";
    if (enabled && directory.find('\0') != std::string::npos) return "录制目录包含无效字符";
    return {};
  }
};
