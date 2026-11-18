#pragma once

#include <dds/ddsi/ddsi_xt_typeinfo.h>
#include <map>
#include <string>

// 实时解码器已解析的 Complete TypeObject，以 14 字节的等价哈希索引。
// 映射中的指针由解码器管理，调用期间须保持有效。
using DdsSchemaObjects = std::map<std::string, const DDS_XTypes_CompleteTypeObject*>;
// 从 root 和 objects 中的完整类型依赖重建 OMG IDL，返回独立持有声明文本的字符串。
// 只遍历已解析的类型。依赖缺失、类型不支持或超出导出限制时抛出 std::runtime_error。
std::string ddsTypeSchema(const DDS_XTypes_TypeIdentifier& root, const DdsSchemaObjects& objects);
