#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

struct DdsCdrRecord;

/// 写入一个独立可读的 MCAP 文件，包含 CDR 消息、OMG IDL 类型和文件摘要。
class DdsMcapWriter {
 public:
  /// 独占创建文件并写入 MCAP 文件头。路径已存在或写入失败时抛出异常。
  /// @param path 新文件路径，父目录需由调用方创建。
  explicit DdsMcapWriter(const std::filesystem::path& path);

  /// 释放写入器和文件资源。完整的 MCAP 文件尾由 close() 写入。
  ~DdsMcapWriter();

  /// 文件句柄由一个写入器独占，禁止复制构造。
  DdsMcapWriter(const DdsMcapWriter&) = delete;

  /// 禁止通过复制赋值转移文件状态。
  DdsMcapWriter& operator=(const DdsMcapWriter&) = delete;

  /// 计算仅写入一条样本所需的完整文件大小，包含类型、通道、摘要和文件尾。
  /// @param record 含有效类型描述、话题名和完整 CDR 载荷的样本。
  /// @return 完整文件的字节数。
  static uint64_t minimumFileSize(const DdsCdrRecord& record);

  /// 检查写入样本并正常封尾后，当前文件是否仍满足大小及 ID 上限。调用不修改文件。
  /// @param record 含有效类型描述的待写样本。
  /// @param limit 完整文件的字节上限。
  /// @return 可以写入时返回 true，文件已关闭或超出任一限制时返回 false。
  bool fits(const DdsCdrRecord& record, uint64_t limit) const;

  /// 同步写入一条样本，无返回值。ID 耗尽、文件已关闭或写入失败时抛出异常。
  /// @param record 供本次写入使用的样本，包含非负的上位机读取时间和有效类型描述。
  /// 写入器会共享所用的类型描述。调用方应先使用 fits() 检查文件上限。
  void write(const DdsCdrRecord& record);

  /// 写入摘要和文件尾，同步并关闭文件。发生 I/O 错误时抛出异常。关闭后可重复调用。
  void close();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
