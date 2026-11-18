#include "dds_type_schema.hpp"

#include <set>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {
// 将完整类型标识 id 的等价哈希复制为二进制字符串，保留所有字节（包括零字节）。
std::string hashKey(const DDS_XTypes_TypeIdentifier& id) {
  return {reinterpret_cast<const char*>(id._u.equivalence_hash), sizeof(id._u.equivalence_hash)};
}

// 校验来自 TypeObject 的名称 name，返回合法的 IDL 标识符。空名称或非法字符会抛出异常。
std::string identifier(std::string_view name) {
  const auto letter = [](char c) { return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_'; };
  if (name.empty() || !letter(name.front())) throw std::runtime_error("IDL 类型或成员名称无效");
  for (char c : name)
    if (!letter(c) && !(c >= '0' && c <= '9')) throw std::runtime_error("IDL 类型或成员名称无效");
  static const std::set<std::string_view> keywords{
      "abstract", "annotation", "any", "attribute", "bitfield", "bitmask", "bitset", "boolean", "case", "char", "component",
      "const", "consumes", "context", "custom", "default", "double", "emits", "enum", "eventtype", "exception",
      "factory", "FALSE", "finder", "fixed", "float", "getraises", "home", "import", "in", "inout", "interface",
      "int8", "int16", "int32", "int64", "local", "long", "map", "module", "multiple", "native", "Object", "octet", "oneway", "out", "primarykey",
      "private", "provides", "public", "publishes", "raises", "readonly", "sequence", "setraises", "short",
      "string", "struct", "supports", "switch", "TRUE", "truncatable", "typedef", "typeid", "typeprefix",
      "uint8", "uint16", "uint32", "uint64", "unsigned", "union", "uses", "ValueBase", "valuetype", "void", "wchar", "wstring"};
  // OMG IDL 用前导下划线转义保留字，也用同样规则保留名称原有的前导下划线。
  return (keywords.count(name) || name.front() == '_' ? "_" : "") + std::string(name);
}

// 按作用域拆分完整名称 name，返回逐段转义后的模块名和末尾的类型名。作用域不完整时抛出异常。
std::vector<std::string> components(std::string_view name) {
  if (name.starts_with("::")) name.remove_prefix(2);
  std::vector<std::string> result;
  while (!name.empty()) {
    const auto delimiter = name.find("::");
    result.push_back(identifier(name.substr(0, delimiter)));
    if (delimiter == name.npos) return result;
    name.remove_prefix(delimiter + 2);
  }
  throw std::runtime_error("IDL 完整类型名称为空或不完整");
}

// 将 name 转为 ROS 解析插件接受的 IDL 限定名称。无效名称会抛出异常。
std::string qualified(std::string_view name) {
  // rosx_introspection 要求名称以标识符开头。生成的模块声明位于全局作用域，用 A::B 引用即可。
  std::string result;
  for (const auto& part : components(name)) {
    if (!result.empty()) result += "::";
    result += part;
  }
  return result;
}

// 返回 object 的名称视图，生命周期取决于 TypeObject。匿名 sequence 返回空视图，其他不支持的类型会抛出异常。
std::string_view objectName(const DDS_XTypes_CompleteTypeObject& object) {
  switch (object._d) {
    case DDS_XTypes_TK_STRUCTURE: return object._u.struct_type.header.detail.type_name;
    case DDS_XTypes_TK_ALIAS: return object._u.alias_type.header.detail.type_name;
    case DDS_XTypes_TK_ENUM: return object._u.enumerated_type.header.detail.type_name;
    case DDS_XTypes_TK_BITMASK: return object._u.bitmask_type.header.detail.type_name;
    case DDS_XTypes_TK_ARRAY: return object._u.array_type.header.detail.type_name;
    case DDS_XTypes_TK_SEQUENCE:
      return object._u.sequence_type.header.detail ? object._u.sequence_type.header.detail->type_name : "";
    default: throw std::runtime_error("无法导出此 XTypes 类型的 IDL");
  }
}

class SchemaWriter {
 public:
  // 借用 objects 中的类型图，并选择一个与用户顶层模块名不冲突的模块名供生成的 typedef 使用。
  explicit SchemaWriter(const DdsSchemaObjects& objects) : objects_(objects) {
    std::set<std::string> modules;
    for (const auto& [hash, object] : objects) {
      (void)hash;
      const auto name = objectName(*object);
      if (!name.empty()) modules.insert(components(name).front());
    }
    while (modules.count(identifier(internal_module_))) internal_module_ += "_";
  }

  // 以 root 为入口输出完整的 IDL 类型声明。类型无法导出时将异常传给调用方。
  std::string build(const DDS_XTypes_TypeIdentifier& root) {
    type(root, 0);
    return "// Reconstructed from CycloneDDS Complete TypeObjects.\n" + output_;
  }

 private:
  // 在本次导出的内部模块中分配一个未使用的集合类型完整名称。
  std::string generatedName() { return internal_module_ + "::Collection_" + std::to_string(++collection_); }

  // 将 declaration 写入 name 指定的模块作用域。累积输出超过 16 MB 时抛出异常。
  void emit(std::string_view name, const std::string& declaration) {
    const auto parts = components(name);
    for (size_t i = 0; i + 1 < parts.size(); ++i) output_ += "module " + parts[i] + " {\n";
    output_ += declaration + ";\n";
    for (size_t i = 1; i < parts.size(); ++i) output_ += "};\n";
    if (output_.size() > 16 * 1024 * 1024) throw std::runtime_error("录制 IDL 类型描述超过 16 MB");
  }

  // 根据元素类型表达式 element 和长度上限 bound 生成 sequence 表达式。上限为零时生成无界 sequence。
  static std::string sequence(const std::string& element, uint32_t bound) {
    return "sequence<" + element + (bound ? ", " + std::to_string(bound) : "") + ">";
  }

  // 将 bounds 转为按声明顺序排列的 IDL 数组维度后缀。维度缺失或为零时抛出异常。
  template <typename Bounds>
  static std::string dimensions(const Bounds& bounds) {
    if (!bounds._length || !bounds._buffer) throw std::runtime_error("IDL 数组缺少维度");
    std::string result;
    for (uint32_t i = 0; i < bounds._length; ++i) {
      if (!bounds._buffer[i]) throw std::runtime_error("IDL 数组维度为零");
      result += "[" + std::to_string(bounds._buffer[i]) + "]";
    }
    return result;
  }

  // 用 element 和数组维度 suffix 生成 typedef，返回可用于成员声明的限定名称。
  std::string collection(const std::string& element, const std::string& suffix) {
    const auto name = generatedName();
    emit(name, "typedef " + element + " " + components(name).back() + suffix);
    return qualified(name);
  }

  // 在递归深度 depth 处解析 id，返回 IDL 类型表达式，按需写入类型定义。
  // 元素类型缺失、标识种类不支持或嵌套层级超过限制时抛出异常。
  std::string type(const DDS_XTypes_TypeIdentifier& id, unsigned depth) {
    if (depth > 64) throw std::runtime_error("录制 IDL 类型嵌套超过 64 层");
    switch (id._d) {
      case DDS_XTypes_TK_BOOLEAN: return "boolean";
      case DDS_XTypes_TK_BYTE: return "octet";
      case DDS_XTypes_TK_CHAR8: return "char";
      case DDS_XTypes_TK_INT16: return "short";
      case DDS_XTypes_TK_INT32: return "long";
      case DDS_XTypes_TK_INT64: return "long long";
      case DDS_XTypes_TK_UINT16: return "unsigned short";
      case DDS_XTypes_TK_UINT32: return "unsigned long";
      case DDS_XTypes_TK_UINT64: return "unsigned long long";
      case DDS_XTypes_TK_FLOAT32: return "float";
      case DDS_XTypes_TK_FLOAT64: return "double";
      case DDS_XTypes_TI_STRING8_SMALL: case DDS_XTypes_TI_STRING8_LARGE: {
        const auto bound = id._d == DDS_XTypes_TI_STRING8_SMALL ? id._u.string_sdefn.bound : id._u.string_ldefn.bound;
        return "string" + (bound ? "<" + std::to_string(bound) + ">" : std::string{});
      }
      case DDS_XTypes_TI_PLAIN_SEQUENCE_SMALL: case DDS_XTypes_TI_PLAIN_SEQUENCE_LARGE: {
        const auto* element = id._d == DDS_XTypes_TI_PLAIN_SEQUENCE_SMALL
            ? id._u.seq_sdefn.element_identifier : id._u.seq_ldefn.element_identifier;
        const auto bound = id._d == DDS_XTypes_TI_PLAIN_SEQUENCE_SMALL ? id._u.seq_sdefn.bound : id._u.seq_ldefn.bound;
        if (!element) throw std::runtime_error("IDL sequence 缺少元素类型");
        return sequence(type(*element, depth + 1), bound);
      }
      case DDS_XTypes_TI_PLAIN_ARRAY_SMALL: case DDS_XTypes_TI_PLAIN_ARRAY_LARGE: {
        // IDL 的数组维度属于声明符。先生成命名的 typedef，便于在嵌套集合中引用。
        const auto* element = id._d == DDS_XTypes_TI_PLAIN_ARRAY_SMALL
            ? id._u.array_sdefn.element_identifier : id._u.array_ldefn.element_identifier;
        if (!element) throw std::runtime_error("IDL array 缺少元素类型");
        const auto suffix = id._d == DDS_XTypes_TI_PLAIN_ARRAY_SMALL
            ? dimensions(id._u.array_sdefn.array_bound_seq) : dimensions(id._u.array_ldefn.array_bound_seq);
        return collection(type(*element, depth + 1), suffix);
      }
      case DDS_XTypes_EK_COMPLETE: return object(id, depth + 1);
      default: throw std::runtime_error("无法导出此 XTypes 类型标识的 IDL：" + std::to_string(id._d));
    }
  }

  // 在递归深度 depth 处导出 id 对应的 Complete TypeObject，返回其限定名称。
  // 重复引用复用同一个声明，依赖缺失或遇到递归类型时抛出异常。
  std::string object(const DDS_XTypes_TypeIdentifier& id, unsigned depth) {
    const auto hash = hashKey(id);
    if (const auto found = names_.find(hash); found != names_.end()) {
      if (active_.count(hash)) throw std::runtime_error("暂不支持递归类型的录制 IDL 导出");
      return qualified(found->second);
    }
    const auto found = objects_.find(hash);
    if (found == objects_.end()) throw std::runtime_error("录制 IDL 缺少完整类型依赖");
    const auto& object = *found->second;
    std::string name(objectName(object));
    if (name.empty()) name = generatedName();
    // 访问依赖时用 active_ 识别回到当前递归链的引用。已完成的共享依赖可直接复用。
    names_.emplace(hash, name);
    active_.insert(hash);
    const auto simple = components(name).back();
    std::string declaration;
    switch (object._d) {
      case DDS_XTypes_TK_STRUCTURE: {
        const auto& structure = object._u.struct_type;
        const auto flags = structure.struct_flags;
        // 保留结构可扩展性与成员 ID。依赖类型在拼接字段声明时先写入输出。
        declaration = flags & DDS_XTypes_IS_MUTABLE ? "@mutable " :
                      flags & DDS_XTypes_IS_APPENDABLE ? "@appendable " : "@final ";
        declaration += "struct " + simple;
        if (structure.header.base_type._d != DDS_XTypes_TK_NONE)
          declaration += " : " + type(structure.header.base_type, depth);
        declaration += " {\n";
        for (uint32_t i = 0; i < structure.member_seq._length; ++i) {
          const auto& member = structure.member_seq._buffer[i];
          std::string annotations = "  @id(" + std::to_string(member.common.member_id) + ") ";
          if (member.common.member_flags & DDS_XTypes_IS_KEY) annotations += "@key ";
          if (member.common.member_flags & DDS_XTypes_IS_OPTIONAL) annotations += "@optional ";
          if (member.common.member_flags & DDS_XTypes_IS_EXTERNAL) annotations += "@external ";
          if (member.common.member_flags & DDS_XTypes_IS_MUST_UNDERSTAND) annotations += "@must_understand ";
          declaration += annotations + type(member.common.member_type_id, depth) + " " + identifier(member.detail.name) + ";\n";
        }
        declaration += "}";
        break;
      }
      case DDS_XTypes_TK_ALIAS:
        declaration = "typedef " + type(object._u.alias_type.body.common.related_type, depth) + " " + simple;
        break;
      case DDS_XTypes_TK_SEQUENCE: {
        const auto& seq = object._u.sequence_type;
        declaration = "typedef " + sequence(type(seq.element.common.type, depth), seq.header.common.bound) + " " + simple;
        break;
      }
      case DDS_XTypes_TK_ARRAY: {
        const auto& array = object._u.array_type;
        declaration = "typedef " + type(array.element.common.type, depth) + " " + simple + dimensions(array.header.common.bound_seq);
        break;
      }
      case DDS_XTypes_TK_ENUM: {
        const auto& enumeration = object._u.enumerated_type;
        // 使用显式值保留枚举的数值映射，包括非连续值和默认枚举项。
        declaration = "@bit_bound(" + std::to_string(enumeration.header.common.bit_bound) + ") enum " + simple + " {\n";
        for (uint32_t i = 0; i < enumeration.literal_seq._length; ++i) {
          const auto& literal = enumeration.literal_seq._buffer[i];
          declaration += (i ? ",\n" : "") + std::string("  @value(") + std::to_string(literal.common.value) + ") ";
          if (literal.common.flags & DDS_XTypes_IS_DEFAULT) declaration += "@default_literal ";
          declaration += identifier(literal.detail.name);
        }
        declaration += "\n}";
        break;
      }
      case DDS_XTypes_TK_BITMASK: {
        const auto& mask = object._u.bitmask_type;
        // bitmask 的声明顺序可以和位序不同，须保留每个标志所占的位。
        declaration = "@bit_bound(" + std::to_string(mask.header.common.bit_bound) + ") bitmask " + simple + " {\n";
        for (uint32_t i = 0; i < mask.flag_seq._length; ++i) {
          const auto& bit = mask.flag_seq._buffer[i];
          declaration += (i ? ",\n" : "") + std::string("  @position(") + std::to_string(bit.common.position) + ") " + identifier(bit.detail.name);
        }
        declaration += "\n}";
        break;
      }
      default: throw std::runtime_error("无法导出此 XTypes 类型的 IDL");
    }
    emit(name, declaration);
    active_.erase(hash);
    return qualified(name);
  }

  const DdsSchemaObjects& objects_;
  std::map<std::string, std::string> names_;
  std::set<std::string> active_;
  std::string internal_module_ = "PlotJugglerMcapInternal";
  size_t collection_ = 0;
  std::string output_;
};
}  // namespace

// 每次从 root 和解码器提供的 objects 单独生成 IDL，导出缓存随本次调用结束而释放。
std::string ddsTypeSchema(const DDS_XTypes_TypeIdentifier& root, const DdsSchemaObjects& objects) {
  return SchemaWriter(objects).build(root);
}
