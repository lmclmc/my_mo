#pragma once

#include <dds/dds.h>
#include <memory>
#include <string>
#include <vector>

// 持有从本地 IDL 生成的描述符及其 Complete TypeObject。须等依赖这些描述符的
// Topic、Reader 和解码器全部销毁后，再释放此对象。
class DdsIdlLibrary {
 public:
  // 从 path 指定的 IDL 文件编译并加载类型，返回共享持有的只读库。
  // 文件、外部工具或生成的 XTypes 元数据无效时抛出异常。
  static std::shared_ptr<const DdsIdlLibrary> load(const std::string& path);

  // 卸载生成的共享库并尝试删除临时目录。
  ~DdsIdlLibrary();
  // 动态库句柄和临时目录归此对象所有，实例不可复制。
  DdsIdlLibrary(const DdsIdlLibrary&) = delete;
  // 禁止复制赋值，确保描述符的所有权与库实例绑定。
  DdsIdlLibrary& operator=(const DdsIdlLibrary&) = delete;

  // 返回源 IDL 的规范化绝对路径，引用的有效期与库对象一致。
  const std::string& sourcePath() const;
  // 返回按名称排序的可订阅 DDS 类型全名。
  std::vector<std::string> typeNames() const;
  // 按 DDS 类型全名查找描述符。未找到时返回 nullptr，返回的指针由库对象持有。
  const dds_topic_descriptor_t* find(const std::string& type_name) const;

 private:
  struct Impl;
  // 接管已加载的共享库及其临时构建目录。
  explicit DdsIdlLibrary(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};
