#include "dds_dynamic.hpp"
#include "dds_cdr_record.hpp"
#include "dds_idl.hpp"
#include "dds_recorder.hpp"
#include "dds_type_schema.hpp"

#include <dds/ddsc/dds_opcodes.h>
#include <dds/ddsi/ddsi_xqos.h>
#include <dds/ddsi/ddsi_typelib.h>
#include <dds/ddsi/ddsi_xt_impl.h>
#include <dds/ddsi/ddsi_serdata.h>
#include <dds/features.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#ifndef DDS_HAS_TYPE_DISCOVERY
#error "The DDS stream plugin requires CycloneDDS built with type discovery"
#endif

namespace {
using Clock = std::chrono::steady_clock;
constexpr size_t kMaxFields = 100000;

// 检查 DDS 调用结果 result。非负值原样返回，负值携带 operation 和 DDS 错误说明抛出异常。
dds_entity_t checked(dds_entity_t result, const char* operation) {
  if (result < 0) throw std::runtime_error(std::string(operation) + ": " + dds_strretcode(result));
  return result;
}

struct Entity {
  dds_entity_t value = DDS_ENTITY_NIL;
  // 析构时删除已成功创建的 DDS 实体。
  ~Entity() { if (value > 0) dds_delete(value); }
};
struct DescriptorDeleter {
  // 释放由 CycloneDDS 动态创建的话题描述符 p。
  void operator()(dds_topic_descriptor_t* p) const { dds_delete_topic_descriptor(p); }
};
struct TypeInfoDeleter {
  // 释放本对象拥有的类型信息 p。
  void operator()(dds_typeinfo_t* p) const { dds_free_typeinfo(p); }
};
struct TypeObjectDeleter {
  // 释放本对象拥有的 TypeObject p。
  void operator()(dds_typeobj_t* p) const { dds_free_typeobj(p); }
};
using TypeInfo = std::unique_ptr<dds_typeinfo_t, TypeInfoDeleter>;

// 通过析构归还 Reader 借出的原生样本，覆盖解码和内存分配抛出异常的路径。
struct Loan {
  dds_entity_t reader;
  void** samples;
  dds_return_t count;
  // 将 samples 中的 count 个样本归还给 reader。
  ~Loan() { if (count > 0) dds_return_loan(reader, samples, count); }
};

struct CdrLoan {
  ddsi_serdata** samples;
  dds_return_t count;
  // 降低每个有效序列化样本的引用计数，释放 dds_takecdr 取得的引用。
  ~CdrLoan() {
    for (dds_return_t i = 0; i < count; ++i)
      if (samples[i]) ddsi_serdata_unref(samples[i]);
  }
};

struct CdrView {
  ddsrt_iovec_t bytes{};
  ddsi_serdata* reference;
  // 获取 sample 的完整 CDR 字节视图。引用获取失败时 reference 为空。
  explicit CdrView(const ddsi_serdata* sample)
      : reference(ddsi_serdata_to_ser_ref(sample, 0, ddsi_serdata_size(sample), &bytes)) {}
  // 释放 CDR 字节视图的引用，bytes 随即失效。
  ~CdrView() { if (reference) ddsi_serdata_to_ser_unref(reference, &bytes); }
};

struct NativeSample {
  const dds_topic_descriptor_t* descriptor;
  void* data;
  // 按话题描述符 type 分配并清零原生样本。借用的 type 须覆盖样本生命周期，分配失败时抛出 std::bad_alloc。
  explicit NativeSample(const dds_topic_descriptor_t* type) : descriptor(type), data(dds_alloc(type->m_size)) {
    if (!data) throw std::bad_alloc();
    std::memset(data, 0, type->m_size);
  }
  // 使用描述符释放原生样本及其嵌套分配。
  ~NativeSample() { dds_sample_free(data, descriptor, DDS_FREE_ALL); }
};

// 从 p 复制 sizeof(T) 字节并按原生类型解读，允许 p 未按 T 对齐，返回独立的值。
template <typename T>
T load(const void* p) {
  T result;
  std::memcpy(&result, p, sizeof(result));
  return result;
}

// 检查发布端 qos 中是否有分区匹配 settings。分区缺失时按默认空分区判断，并释放 DDS 返回的名称数组。
bool partitionMatches(const dds_qos_t* qos, const DdsSettings& settings) {
  uint32_t count = 0;
  char** names = nullptr;
  if (!dds_qget_partition(qos, &count, &names)) return settings.matchesPartition("");
  bool matched = count == 0 && settings.matchesPartition("");
  for (uint32_t i = 0; i < count; ++i) {
    const char* name = names[i] ? names[i] : "";
    matched = matched || settings.matchesPartition(name);
    dds_free(names[i]);
  }
  dds_free(names);
  return matched;
}

// Complete TypeObject 提供真实字段名和类型语义，话题描述符提供原生内存偏移和元素步长。
// CycloneDDS 完成 XCDR1/XCDR2 反序列化和字节序处理，解码器遍历反序列化后的原生样本。
class DynamicDecoder {
  struct Type;
  struct Member { uint32_t id; std::string name; Type* type; };
  struct Type {
    uint8_t kind = DDS_XTypes_TK_NONE;
    Type* element = nullptr;
    Type* base = nullptr;
    std::vector<Member> members;
    std::vector<size_t> dimensions;
  };
  struct Layout;
  struct NamedLayout { std::string name; Layout* layout; };
  struct Layout {
    dds_stream_typecode code = DDS_OP_VAL_STU;
    uint32_t flags = 0;
    size_t offset = 0;
    size_t size = 0;
    bool indirect = false;
    Layout* element = nullptr;
    std::vector<size_t> dimensions;
    std::vector<NamedLayout> members;
  };

 public:
  // 使用 participant 和完整类型信息 info 构建字段布局。可借用 local_descriptor，其生命周期须覆盖解码器。
  // 未提供本地描述符时由解码器取得并管理动态描述符。类型缺失、获取超时或布局不兼容时抛出异常。
  DynamicDecoder(dds_entity_t participant, const dds_typeinfo_t* info,
                 const dds_topic_descriptor_t* local_descriptor = nullptr)
      : participant_(participant), deadline_(Clock::now() + std::chrono::milliseconds(500)),
        local_descriptor_(local_descriptor) {
    const auto* id = ddsi_typeinfo_complete_typeid(info);
    if (!id || id->x._d != DDS_XTypes_EK_COMPLETE)
      throw std::runtime_error("发布端未提供 Complete TypeInformation，请在连接窗口导入匹配的 IDL 文件");
    if (!local_descriptor_) {
      dds_topic_descriptor_t* descriptor = nullptr;
      checked(dds_create_topic_descriptor(DDS_FIND_SCOPE_GLOBAL, participant, info, DDS_MSECS(100), &descriptor),
              "获取动态话题描述");
      descriptor_.reset(descriptor);
    }
    Type* type = resolve(id->x, 0);
    root_id_ = id->x;
    root_ = aggregate(type, this->descriptor()->m_ops, 0);
  }

  // 返回正在使用的话题描述符，调用方借用该指针。
  const dds_topic_descriptor_t* descriptor() const { return local_descriptor_ ? local_descriptor_ : descriptor_.get(); }

  // 从已解析的类型图构建录制用类型名和 IDL。返回可跨线程共享的数据，IDL 导出失败时传播异常。
  std::shared_ptr<const DdsRecordingType> recordingType() const {
    return std::make_shared<DdsRecordingType>(DdsRecordingType{descriptor()->m_typename,
                                                             ddsTypeSchema(root_id_, schema_objects_)});
  }

  // 按预先构建的布局解码原生样本 sample，返回各自拥有存储的标量字段。样本不合法或展开超限时抛出异常。
  std::vector<DdsDynamicField> decode(const void* sample) const {
    std::vector<DdsDynamicField> result;
    walk(*root_, static_cast<const unsigned char*>(sample), "", result, 0);
    return result;
  }

 private:
  // condition 为假时抛出带有 message 的 std::runtime_error。
  static void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
  }

  // 分配 kind 对应的语义类型节点，返回解码器拥有的稳定指针。节点数量超限时抛出异常。
  Type* newType(uint8_t kind) {
    require(types_.size() < 8192, "动态类型过于复杂");
    auto type = std::make_unique<Type>();
    type->kind = kind;
    auto* ptr = type.get();
    types_.push_back(std::move(type));
    return ptr;
  }

  // 分配原生指令类型 code 的布局节点，返回解码器拥有的稳定指针。节点数量超限时抛出异常。
  Layout* newLayout(dds_stream_typecode code) {
    require(layouts_.size() < 32768, "动态字段布局过于复杂");
    auto layout = std::make_unique<Layout>();
    layout->code = code;
    auto* ptr = layout.get();
    layouts_.push_back(std::move(layout));
    return ptr;
  }

  // 在递归深度 depth 处解析 XTypes 标识 id，返回解码器拥有的语义类型。
  // 依赖无法获取、类型不支持或解析超出时间与深度限制时抛出异常。
  Type* resolve(const DDS_XTypes_TypeIdentifier& id, unsigned depth) {
    require(depth < 64, "动态类型嵌套超过 64 层");
    switch (id._d) {
      case DDS_XTypes_TK_BOOLEAN: case DDS_XTypes_TK_BYTE:
      case DDS_XTypes_TK_INT16: case DDS_XTypes_TK_INT32: case DDS_XTypes_TK_INT64:
      case DDS_XTypes_TK_UINT16: case DDS_XTypes_TK_UINT32: case DDS_XTypes_TK_UINT64:
      case DDS_XTypes_TK_FLOAT32: case DDS_XTypes_TK_FLOAT64: case DDS_XTypes_TK_CHAR8:
        return newType(id._d);
      case DDS_XTypes_TI_STRING8_SMALL: case DDS_XTypes_TI_STRING8_LARGE:
        return newType(DDS_XTypes_TK_STRING8);
      case DDS_XTypes_TI_PLAIN_SEQUENCE_SMALL: case DDS_XTypes_TI_PLAIN_SEQUENCE_LARGE: {
        auto* type = newType(DDS_XTypes_TK_SEQUENCE);
        const auto* element = id._d == DDS_XTypes_TI_PLAIN_SEQUENCE_SMALL
            ? id._u.seq_sdefn.element_identifier : id._u.seq_ldefn.element_identifier;
        require(element != nullptr, "sequence 缺少元素类型");
        type->element = resolve(*element, depth + 1);
        return type;
      }
      case DDS_XTypes_TI_PLAIN_ARRAY_SMALL: case DDS_XTypes_TI_PLAIN_ARRAY_LARGE: {
        auto* type = newType(DDS_XTypes_TK_ARRAY);
        const DDS_XTypes_TypeIdentifier* element;
        if (id._d == DDS_XTypes_TI_PLAIN_ARRAY_SMALL) {
          const auto& array = id._u.array_sdefn;
          type->dimensions.assign(array.array_bound_seq._buffer,
                                  array.array_bound_seq._buffer + array.array_bound_seq._length);
          element = array.element_identifier;
        } else {
          const auto& array = id._u.array_ldefn;
          type->dimensions.assign(array.array_bound_seq._buffer,
                                  array.array_bound_seq._buffer + array.array_bound_seq._length);
          element = array.element_identifier;
        }
        require(element != nullptr, "array 缺少元素类型");
        type->element = resolve(*element, depth + 1);
        return type;
      }
      case DDS_XTypes_EK_COMPLETE: break;
      default:
        throw std::runtime_error("暂不支持的 XTypes 类型标识: " + std::to_string(id._d));
    }

    // 基础类型、字符串和内联集合可直接从标识还原。命名类型通过等价哈希缓存并获取完整 TypeObject。
    const std::string key(reinterpret_cast<const char*>(id._u.equivalence_hash), sizeof(id._u.equivalence_hash));
    if (const auto it = type_cache_.find(key); it != type_cache_.end()) return it->second;
    require(Clock::now() < deadline_, "完整类型依赖解析超时，将自动重试");
    ddsi_typeid type_id{};
    type_id.x = id;
    dds_typeobj_t* object = nullptr;
    checked(dds_get_typeobj(participant_, &type_id, DDS_MSECS(100), &object), "获取 Complete TypeObject");
    objects_.emplace_back(object);
    require(object->x._d == DDS_XTypes_EK_COMPLETE, "收到的 TypeObject 没有真实成员名");
    const auto& complete = object->x._u.complete;
    schema_objects_[key] = &complete;
    auto* type = newType(complete._d);
    // 先登记节点，再沿依赖遍历，使递归结构复用已创建的节点。
    type_cache_[key] = type;
    switch (complete._d) {
      case DDS_XTypes_TK_STRUCTURE: {
        const auto& structure = complete._u.struct_type;
        if (structure.header.base_type._d != DDS_XTypes_TK_NONE)
          type->base = resolve(structure.header.base_type, depth + 1);
        for (uint32_t i = 0; i < structure.member_seq._length; ++i) {
          const auto& member = structure.member_seq._buffer[i];
          require(member.detail.name[0] != '\0', "Complete TypeObject 成员名为空");
          type->members.push_back({member.common.member_id, member.detail.name,
                                   resolve(member.common.member_type_id, depth + 1)});
        }
        break;
      }
      case DDS_XTypes_TK_ALIAS:
        type->element = resolve(complete._u.alias_type.body.common.related_type, depth + 1);
        break;
      case DDS_XTypes_TK_SEQUENCE:
        type->element = resolve(complete._u.sequence_type.element.common.type, depth + 1);
        break;
      case DDS_XTypes_TK_ARRAY: {
        const auto& array = complete._u.array_type;
        const auto& bounds = array.header.common.bound_seq;
        type->dimensions.assign(bounds._buffer, bounds._buffer + bounds._length);
        type->element = resolve(array.element.common.type, depth + 1);
        break;
      }
      case DDS_XTypes_TK_ENUM: case DDS_XTypes_TK_BITMASK: break;
      default:
        throw std::runtime_error("暂不支持的 Complete TypeObject 种类: " + std::to_string(complete._d));
    }
    return type;
  }

  // 沿 type 的 alias 链返回底层类型指针。类型缺失或 alias 链超过限制时抛出异常。
  static Type* unalias(Type* type) {
    for (unsigned i = 0; type && type->kind == DDS_XTypes_TK_ALIAS; ++i) {
      require(i < 64, "循环 alias 类型");
      type = type->element;
    }
    require(type != nullptr, "缺失动态类型");
    return type;
  }

  // 判断原生类型码 code 是否代表具有独立布局的聚合或集合元素。
  static bool complex(dds_stream_typecode code) {
    return code == DDS_OP_VAL_STU || code == DDS_OP_VAL_UNI || code == DDS_OP_VAL_SEQ ||
           code == DDS_OP_VAL_BSQ || code == DDS_OP_VAL_ARR;
  }

  // 将基础 XTypes 类型 kind 映射为对应宽度的 CycloneDDS 指令类型码。无法映射时抛出异常。
  static dds_stream_typecode primitiveCode(uint8_t kind) {
    switch (kind) {
      case DDS_XTypes_TK_BOOLEAN: return DDS_OP_VAL_BLN;
      case DDS_XTypes_TK_BYTE: case DDS_XTypes_TK_CHAR8: return DDS_OP_VAL_1BY;
      case DDS_XTypes_TK_INT16: case DDS_XTypes_TK_UINT16: return DDS_OP_VAL_2BY;
      case DDS_XTypes_TK_INT32: case DDS_XTypes_TK_UINT32: case DDS_XTypes_TK_FLOAT32: return DDS_OP_VAL_4BY;
      case DDS_XTypes_TK_INT64: case DDS_XTypes_TK_UINT64: case DDS_XTypes_TK_FLOAT64: return DDS_OP_VAL_8BY;
      default: throw std::runtime_error("非基本类型不能使用标量布局");
    }
  }

  // 返回指令流中 op 所指字段之后的位置。要求 op 指向有效 ADR 指令，否则抛出异常。
  static const uint32_t* next(const uint32_t* op) {
    require(DDS_OP(*op) == DDS_OP_ADR, "动态描述字段不是 ADR 指令");
    const auto code = DDS_OP_TYPE(*op);
    // ADR 的参数长度随类型变化。嵌套类型和复杂集合在指令中记录了下一条指令的跳转位置。
    if (code == DDS_OP_VAL_EXT) return op + DDS_OP_ADR_JMP(op[2]);
    if (code == DDS_OP_VAL_SEQ || code == DDS_OP_VAL_BSQ || code == DDS_OP_VAL_ARR) {
      const auto sub = DDS_OP_SUBTYPE(*op);
      const size_t base = code == DDS_OP_VAL_SEQ ? 2 : 3;
      if (complex(sub)) return op + DDS_OP_ADR_JMP(op[base + (code == DDS_OP_VAL_ARR ? 0 : 1)]);
      if (sub == DDS_OP_VAL_BST) return op + base + (code == DDS_OP_VAL_ARR ? 2 : 1);
      if (sub == DDS_OP_VAL_ENU) return op + base + 1;
      if (sub == DDS_OP_VAL_BMK) return op + base + 2;
      return op + base;
    }
    if (code == DDS_OP_VAL_ENU || code == DDS_OP_VAL_BST) return op + 3;
    if (code == DDS_OP_VAL_BMK) return op + 4;
    return op + 2;
  }

  // 在递归深度 depth 处对照语义结构 input 和原生指令 op，生成并返回解码器拥有的结构体布局。
  // 不支持的结构类型、成员不一致或嵌套超限时抛出异常。
  Layout* aggregate(Type* input, const uint32_t* op, unsigned depth) {
    require(depth < 64, "动态布局嵌套超过 64 层");
    Type* type = unalias(input);
    require(type->kind == DDS_XTypes_TK_STRUCTURE, "当前动态解析仅支持 struct 话题及其嵌套成员");
    const auto key = std::make_pair(type, op);
    if (const auto it = layout_cache_.find(key); it != layout_cache_.end()) return it->second;
    auto* layout = newLayout(DDS_OP_VAL_STU);
    layout_cache_[key] = layout;
    if (DDS_OP(*op) == DDS_OP_DLC) ++op;
    if (DDS_OP(*op) == DDS_OP_PLC) {
      // Mutable 结构用 PLM 成员索引按成员 ID 寻址，其指令顺序可以与 TypeObject 声明顺序不同。
      size_t found = 0;
      bool found_base = false;
      for (++op; DDS_OP(*op) != DDS_OP_RTS; op += 2) {
        require(DDS_OP(*op) == DDS_OP_PLM, "mutable struct 缺少 PLM 字段索引");
        if (DDS_PLM_FLAGS(*op) & DDS_OP_FLAG_BASE) {
          require(type->base != nullptr, "mutable struct 基类信息不一致");
          layout->members.push_back({"", aggregate(type->base, op + DDS_OP_ADR_PLM(*op), depth + 1)});
          found_base = true;
          continue;
        }
        const auto it = std::find_if(type->members.begin(), type->members.end(),
                                     [op](const Member& member) { return member.id == op[1]; });
        require(it != type->members.end(), "mutable struct 的成员 ID 与 Complete TypeObject 不一致");
        layout->members.push_back({it->name, field(it->type, op + DDS_OP_ADR_PLM(*op), depth + 1)});
        ++found;
      }
      require(found == type->members.size() && found_base == (type->base != nullptr), "mutable struct 字段数量不一致");
    } else {
      // Final 和 Appendable 的原生指令按声明顺序排列，基类在本结构的成员之前。
      if (type->base) {
        layout->members.push_back({"", field(type->base, op, depth + 1)});
        op = next(op);
      }
      for (const auto& member : type->members) {
        layout->members.push_back({member.name, field(member.type, op, depth + 1)});
        op = next(op);
      }
      require(DDS_OP(*op) == DDS_OP_RTS, "动态描述与 Complete TypeObject 的字段数量不一致");
    }
    return layout;
  }

  // 在深度 depth 处把语义类型 type 和字段的 ADR 指令 op 组合为布局，记录字段偏移和间接存储方式。
  // 返回解码器拥有的布局，指令或布局无效时抛出异常。
  Layout* field(Type* type, const uint32_t* op, unsigned depth) {
    require(DDS_OP(*op) == DDS_OP_ADR, "动态字段缺少 ADR 布局");
    auto* layout = value(type, DDS_OP_TYPE(*op), *op, op + 2, op, depth);
    layout->offset = op[1];
    layout->indirect = (*op & DDS_OP_FLAG_EXT) && DDS_OP_TYPE(*op) != DDS_OP_VAL_STR;
    return layout;
  }

  // 根据 input 的类型语义及指令 code、flags 和 args 构建值布局。op 是解析相对跳转的基地址。
  // depth 限制递归层级，返回解码器拥有的布局。类型、指令和集合维度不一致时抛出异常。
  Layout* value(Type* input, dds_stream_typecode code, uint32_t flags,
                const uint32_t* args, const uint32_t* op, unsigned depth) {
    require(depth < 64, "动态字段嵌套超过 64 层");
    Type* type = unalias(input);
    auto* layout = newLayout(code);
    layout->flags = flags;
    switch (code) {
      case DDS_OP_VAL_EXT:
        layout->element = aggregate(type, op + DDS_OP_ADR_JSR(args[0]), depth + 1);
        break;
      case DDS_OP_VAL_SEQ: case DDS_OP_VAL_BSQ: case DDS_OP_VAL_ARR: {
        const bool array = code == DDS_OP_VAL_ARR;
        require(type->kind == (array ? DDS_XTypes_TK_ARRAY : DDS_XTypes_TK_SEQUENCE), "集合类型与原生布局不一致");
        if (array) {
          // 维度来自 TypeObject，原生数组指令保存展平后的元素总数，两者必须一致。
          layout->dimensions = type->dimensions;
          size_t count = 1;
          for (const auto bound : layout->dimensions) {
            require(bound > 0 && count <= std::numeric_limits<uint32_t>::max() / bound, "array 维度溢出");
            count *= bound;
          }
          require(!layout->dimensions.empty() && count == args[0], "array 维度与原生布局不一致");
          layout->size = count;
        }
        const size_t bound_arg = (array || code == DDS_OP_VAL_BSQ) ? 1 : 0;
        const auto sub = DDS_OP_SUBTYPE(flags);
        if (complex(sub)) {
          const uint32_t jump = args[bound_arg + (array ? 0 : 1)];
          // 复杂元素的原生步长包含内存对齐所需的填充。
          layout->size = args[bound_arg + (array ? 1 : 0)];
          const auto* element_ops = op + DDS_OP_ADR_JSR(jump);
          if (sub == DDS_OP_VAL_STU || sub == DDS_OP_VAL_UNI)
            layout->element = aggregate(type->element, element_ops, depth + 1);
          else
            layout->element = field(type->element, element_ops, depth + 1);
        } else {
          const auto* element_args = args + bound_arg;
          if (array && sub == DDS_OP_VAL_BST) ++element_args;  // 跳过有界字符串长度前的保留字段。
          layout->element = value(type->element, sub, flags, element_args, op, depth + 1);
          layout->size = layout->element->size;
        }
        require(layout->size != 0, "集合元素 stride 为零");
        break;
      }
      case DDS_OP_VAL_STR: case DDS_OP_VAL_BST:
        require(type->kind == DDS_XTypes_TK_STRING8, "字符串类型与原生布局不一致");
        layout->size = code == DDS_OP_VAL_STR ? sizeof(char*) : args[0];
        break;
      case DDS_OP_VAL_ENU: case DDS_OP_VAL_BMK:
        require(type->kind == (code == DDS_OP_VAL_ENU ? DDS_XTypes_TK_ENUM : DDS_XTypes_TK_BITMASK), "enum/bitmask 布局不一致");
        layout->size = DDS_OP_TYPE_SZ(flags);
        break;
      case DDS_OP_VAL_BLN: case DDS_OP_VAL_1BY: case DDS_OP_VAL_2BY:
      case DDS_OP_VAL_4BY: case DDS_OP_VAL_8BY:
        require(primitiveCode(type->kind) == code, "基本类型与原生布局不一致");
        layout->size = code == DDS_OP_VAL_BLN ? sizeof(bool) : size_t{1} << (code - DDS_OP_VAL_1BY);
        break;
      default: throw std::runtime_error("暂不支持的 CycloneDDS 原生布局: " + std::to_string(code));
    }
    return layout;
  }

  // 按 layout 指定的宽度、符号和浮点标记从 data 读取数值，返回保留原类型的值。未知宽度抛出异常。
  static DdsDynamicValue number(const Layout& layout, const unsigned char* data) {
    if (layout.code == DDS_OP_VAL_BLN) return load<bool>(data);
    if (layout.code != DDS_OP_VAL_ENU && layout.code != DDS_OP_VAL_BMK && (layout.flags & DDS_OP_FLAG_FP)) {
      if (layout.size == 4) return load<float>(data);
      if (layout.size == 8) return load<double>(data);
    }
    const bool sign = layout.code == DDS_OP_VAL_ENU || (layout.flags & DDS_OP_FLAG_SGN);
    switch (layout.size) {
      case 1: if (sign) return load<int8_t>(data); else return load<uint8_t>(data);
      case 2: if (sign) return load<int16_t>(data); else return load<uint16_t>(data);
      case 4: if (sign) return load<int32_t>(data); else return load<uint32_t>(data);
      case 8: if (sign) return load<int64_t>(data); else return load<uint64_t>(data);
      default: throw std::runtime_error("不支持的数值宽度");
    }
  }

  // 在 depth 层按 layout 遍历从 parent 开始的原生样本，沿 path 展开路径并将复制后的叶字段追加到 fields。
  // 集合内容无效、超出字段数量限制或嵌套过深时抛出异常，已追加的字段仍由 fields 持有。
  static void walk(const Layout& layout, const unsigned char* parent, const std::string& path,
                   std::vector<DdsDynamicField>& fields, unsigned depth) {
    require(depth < 64, "样本嵌套超过 64 层");
    const auto* data = parent + layout.offset;
    if (layout.indirect) {
      data = load<const unsigned char*>(data);
      if (!data) return;  // 缺省的 optional 成员不生成字段。
    }
    switch (layout.code) {
      case DDS_OP_VAL_STU:
        for (const auto& member : layout.members) {
          const auto name = member.name.empty() ? path : (path.empty() ? member.name : path + "/" + member.name);
          walk(*member.layout, data, name, fields, depth + 1);
        }
        return;
      case DDS_OP_VAL_EXT:
        walk(*layout.element, data, path, fields, depth + 1);
        return;
      case DDS_OP_VAL_SEQ: case DDS_OP_VAL_BSQ: case DDS_OP_VAL_ARR: {
        size_t count = 1;
        const auto* elements = data;
        if (layout.code == DDS_OP_VAL_ARR) {
          for (const auto dim : layout.dimensions) count *= dim;
        } else {
          const auto sequence = load<dds_sequence_t>(data);
          require(sequence._length <= sequence._maximum, "sequence 长度超过容量");
          count = sequence._length;
          elements = static_cast<const unsigned char*>(sequence._buffer);
          require(count == 0 || elements != nullptr, "sequence 数据为空");
        }
        require(count <= kMaxFields, "单个集合元素数量超过 100000");
        for (size_t i = 0; i < count; ++i) {
          std::string name = path;
          // 每个索引独立占一段路径，多维数组按维度拆分线性索引，便于宿主按数组层级分组。
          if (layout.code == DDS_OP_VAL_ARR) {
            size_t remaining = i, divisor = count;
            for (const auto dim : layout.dimensions) {
              divisor /= dim;
              name += "/[" + std::to_string(remaining / divisor) + "]";
              remaining %= divisor;
            }
          } else name += "/[" + std::to_string(i) + "]";
          walk(*layout.element, elements + i * layout.size, name, fields, depth + 1);
        }
        return;
      }
      default: break;
    }
    require(fields.size() < kMaxFields, "单个样本展开字段超过 100000");
    if (layout.code == DDS_OP_VAL_STR) {
      const char* text = load<const char*>(data);
      fields.push_back({path, text ? std::string(text) : std::string()});
    } else if (layout.code == DDS_OP_VAL_BST) {
      const char* text = reinterpret_cast<const char*>(data);
      fields.push_back({path, std::string(text, strnlen(text, layout.size))});
    } else fields.push_back({path, number(layout, data)});
  }

  dds_entity_t participant_;
  Clock::time_point deadline_;
  const dds_topic_descriptor_t* local_descriptor_;
  std::unique_ptr<dds_topic_descriptor_t, DescriptorDeleter> descriptor_;
  std::vector<std::unique_ptr<dds_typeobj_t, TypeObjectDeleter>> objects_;
  DDS_XTypes_TypeIdentifier root_id_{};
  DdsSchemaObjects schema_objects_;
  std::vector<std::unique_ptr<Type>> types_;
  std::map<std::string, Type*> type_cache_;
  std::vector<std::unique_ptr<Layout>> layouts_;
  std::map<std::pair<Type*, const uint32_t*>, Layout*> layout_cache_;
  Layout* root_ = nullptr;
};
}  // namespace

struct DdsDynamicStream::Impl {
  struct Candidate {
    std::string topic, type;
    TypeInfo info;
    Clock::time_point retry{};
    std::string last_error;
  };
  struct Subscription {
    std::shared_ptr<const DdsIdlLibrary> idl;
    std::unique_ptr<DynamicDecoder> decoder;
    std::shared_ptr<const DdsRecordingType> recording_type;
    Entity topic, reader;
    std::string name;
    std::string last_error;
    bool matched = false;
  };

  // 复制 configuration，创建发现 Reader、WaitSet 和可选录制器，最后启动接收线程。
  // 话题为空或资源初始化失败时抛出异常，已经创建的实体由 RAII 对象释放。
  explicit Impl(const DdsSettings& configuration) : settings(configuration) {
    if (settings.selected_topics.empty()) throw std::runtime_error("请先在连接窗口选择至少一个 DDS 话题");
    participant.value = checked(dds_create_participant(static_cast<dds_domainid_t>(settings.domain_id), nullptr, nullptr),
                                 "创建 DDS Participant");
    auto qos = settings.subscriberQos();
    subscriber = checked(dds_create_subscriber(participant.value, qos.get(), nullptr), "创建 DDS Subscriber");
    publications = checked(dds_create_reader(participant.value, DDS_BUILTIN_TOPIC_DCPSPUBLICATION, nullptr, nullptr),
                            "创建发布端发现 Reader");
    waitset = checked(dds_create_waitset(participant.value), "创建 DDS WaitSet");
    const auto condition = checked(dds_create_readcondition(publications, DDS_ANY_STATE), "创建发现条件");
    checked(dds_waitset_attach(waitset, condition, 0), "挂接发现条件");
    // WaitSet 自身也可作为条件，停止时用它主动唤醒阻塞的接收线程。
    checked(dds_waitset_attach(waitset, waitset, 0), "挂接停止条件");
    if (settings.recording.enabled) {
      recorder = std::make_unique<DdsRecorder>(settings.recording,
          [this](bool warning, std::string text) { notice(warning, std::move(text)); });
    }
    worker = std::thread([this] { run(); });
  }

  // 等待后台线程结束，再销毁订阅。Reader 先于创建它所用的动态描述符释放。
  ~Impl() {
    stop();
    subscriptions.clear();
  }

  // 停止并唤醒接收线程，等待它停止提交新样本后再排空和关闭录制器。
  void stop() {
    running = false;
    if (waitset > 0) dds_waitset_set_trigger(waitset, true);
    if (worker.joinable()) worker.join();
    if (recorder) recorder->stop();
  }

  // 将 text 和 warning 级别移入宿主通知队列。队列已有 128 条时忽略新通知。
  void notice(bool warning, std::string text) {
    std::lock_guard lock(mutex);
    if (notices.size() < 128) notices.push_back({warning, std::move(text)});
  }

  // 批量读取内置发布端数据，按实例句柄更新当前候选发布端。
  // 新增端点须匹配所选话题及分区，离线端点从候选表移除。DDS 读取失败时抛出异常。
  void discover() {
    std::array<void*, 32> samples{};
    std::array<dds_sample_info_t, 32> infos{};
    const auto count = checked(dds_take(publications, samples.data(), infos.data(), samples.size(), samples.size()),
                               "读取发布端发现信息");
    Loan loan{publications, samples.data(), count};
    for (dds_return_t i = 0; i < count; ++i) {
      const auto handle = infos[i].instance_handle;
      if (infos[i].instance_state != DDS_IST_ALIVE) { candidates.erase(handle); continue; }
      if (!infos[i].valid_data) continue;
      auto* endpoint = static_cast<dds_builtintopic_endpoint_t*>(samples[i]);
      if (!endpoint->topic_name || !endpoint->type_name ||
          std::string(endpoint->topic_name).rfind("DCPS", 0) == 0) continue;
      // 先筛选话题和分区，把类型查询及 Reader 创建限制在用户选中的话题。
      if (!settings.selected_topics.count(endpoint->topic_name) || !partitionMatches(endpoint->qos, settings)) {
        candidates.erase(handle);
        continue;
      }
      auto existing = candidates.find(handle);
      if (existing != candidates.end() && existing->second.info) continue;
      Candidate candidate;
      candidate.topic = endpoint->topic_name;
      candidate.type = endpoint->type_name;
      const dds_typeinfo_t* info = nullptr;
      if (dds_builtintopic_get_endpoint_type_info(endpoint, &info) == DDS_RETCODE_OK && info)
        candidate.info.reset(ddsi_typeinfo_dup(info));  // 原始类型信息随 DDS 样本归还，候选表持有副本。
      if (existing == candidates.end()) candidates.emplace(handle, std::move(candidate));
      else if (candidate.info) existing->second = std::move(candidate);
    }
  }

  // 每轮尝试建立一个尚未订阅的话题和类型组合，失败后延时重试并报告变化的错误。
  void subscribeOne() {
    for (auto& [handle, candidate] : candidates) {
      (void)handle;
      const auto key = std::make_pair(candidate.topic, candidate.type);
      if (subscriptions.count(key) || Clock::now() < candidate.retry) continue;
      candidate.retry = Clock::now() + std::chrono::seconds(2);
      try {
        auto subscription = std::make_unique<Subscription>();
        const dds_topic_descriptor_t* local_descriptor = nullptr;
        // 同名类型对应多个本地 IDL 时优先选用最近导入的定义，供在线解析失败时使用。
        for (auto it = settings.idl_libraries.rbegin(); it != settings.idl_libraries.rend(); ++it) {
          if ((local_descriptor = (*it)->find(candidate.type))) { subscription->idl = *it; break; }
        }
        if (candidate.info) {
          try {
            subscription->decoder = std::make_unique<DynamicDecoder>(participant.value, candidate.info.get());
          } catch (const std::exception&) {
            if (!local_descriptor) throw;
          }
        }
        subscription->name = candidate.topic;
        if (!subscription->decoder) {
          if (!local_descriptor)
            throw std::runtime_error("发布端未提供可用的完整 XTypes 信息，请在连接窗口导入与 " + candidate.type +
                                     " 匹配的 IDL 文件");
          // 创建本地 Topic 会把 IDL 生成的 Complete TypeMapping 注册到 CycloneDDS。
          // 解码器随后解析这些本地依赖，并与 Reader 使用同一个原生描述符。
          subscription->topic.value = checked(dds_create_topic(participant.value, local_descriptor,
                                                               candidate.topic.c_str(), nullptr, nullptr), "创建 IDL Topic");
          dds_typeinfo_t* info = nullptr;
          checked(dds_get_typeinfo(subscription->topic.value, &info), "读取本地 IDL 类型信息");
          TypeInfo local_info(info);
          subscription->decoder = std::make_unique<DynamicDecoder>(participant.value, local_info.get(), local_descriptor);
        } else {
          subscription->idl.reset();
          subscription->topic.value = checked(dds_create_topic(participant.value, subscription->decoder->descriptor(),
                                                             candidate.topic.c_str(), nullptr, nullptr), "创建动态 Topic");
        }
        if (recorder) {
          // 录制依赖可独立读取的 IDL。Schema 导出失败时保留该话题的实时绘图。
          try {
            subscription->recording_type = subscription->decoder->recordingType();
          } catch (const std::exception& error) {
            notice(true, "DDS " + candidate.topic + " 无法准备 CDR 录制类型：" + error.what() +
                         "。该话题继续订阅和绘图，暂不录制。");
          }
        }
        auto qos = settings.readerQos();
        subscription->reader.value = checked(dds_create_reader(subscriber, subscription->topic.value, qos.get(), nullptr),
                                              "创建动态 Reader");
        checked(dds_set_status_mask(subscription->reader.value, DDS_REQUESTED_INCOMPATIBLE_QOS_STATUS |
                                    DDS_SUBSCRIPTION_MATCHED_STATUS), "启用 Reader 匹配状态");
        const auto condition = checked(dds_create_readcondition(subscription->reader.value, DDS_ANY_STATE), "创建数据条件");
        checked(dds_waitset_attach(waitset, condition, 0), "挂接动态 Reader");
        const bool using_idl = static_cast<bool>(subscription->idl);
        subscriptions.emplace(key, std::move(subscription));
        notice(false, "DDS 已建立动态订阅：" + candidate.topic + " (" + candidate.type + ")" +
                      (using_idl ? " · 本地 IDL" : ""));
      } catch (const std::exception& error) {
        if (candidate.last_error != error.what()) {
          candidate.last_error = error.what();
          notice(true, "DDS " + candidate.topic + "：" + candidate.last_error);
        }
      }
      break;  // 每轮限制类型查询开销，以便及时读取已经建立的订阅。
    }
  }

  // 关闭已无对应候选端点的订阅。同一话题和类型只要还有一个端点在线，就保留 Reader。
  void removeUnusedSubscriptions() {
    for (auto it = subscriptions.begin(); it != subscriptions.end();) {
      const auto& key = it->first;
      const bool alive = std::any_of(candidates.begin(), candidates.end(), [&key](const auto& item) {
        return item.second.topic == key.first && item.second.type == key.second;
      });
      if (alive) ++it;
      else it = subscriptions.erase(it);
    }
  }

  // 更新 subscription 的匹配状态并读取一批样本，按配置提交给录制队列和绘图队列。
  // DDS 读取失败时抛出异常，单个样本的录制或解码错误通过通知报告后继续读取。
  void receive(Subscription& subscription) {
    dds_requested_incompatible_qos_status_t incompatible{};
    if (dds_get_requested_incompatible_qos_status(subscription.reader.value, &incompatible) == DDS_RETCODE_OK &&
        incompatible.total_count_change > 0)
      notice(true, "DDS " + subscription.name + "：订阅 QoS 与发布端不兼容，policy id=" +
                     std::to_string(incompatible.last_policy_id));
    dds_subscription_matched_status_t matched{};
    if (dds_get_subscription_matched_status(subscription.reader.value, &matched) == DDS_RETCODE_OK) {
      if (matched.current_count > 0 && !subscription.matched)
        notice(false, "DDS 已匹配发布端：" + subscription.name);
      subscription.matched = matched.current_count > 0;
    }
    // 录制模式每批只调用一次 dds_takecdr。先将 CDR 字节复制到录制队列，再从同一样本解码绘图字段。
    if (recorder && subscription.recording_type) {
      std::array<ddsi_serdata*, 32> samples{};
      std::array<dds_sample_info_t, 32> infos{};
      const auto count = checked(dds_takecdr(subscription.reader.value, samples.data(), samples.size(),
                                            infos.data(), DDS_ANY_STATE), "读取 CDR 话题样本");
      CdrLoan loan{samples.data(), count};
      for (dds_return_t i = 0; i < count; ++i) {
        if (!infos[i].valid_data || !samples[i] || samples[i]->kind != SDK_DATA) continue;
        // 录制与绘图共享上位机取样时间，时间戳在解码及排队之前采集。
        const auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        try {
          if (ddsi_serdata_size(samples[i]) < 4) throw std::runtime_error("CDR 样本缺少封装头");
          CdrView view(samples[i]);
          if (!view.reference || !view.bytes.iov_base) throw std::runtime_error("无法取得 CDR 样本内容");
          recorder->submit(subscription.recording_type, subscription.name, timestamp_ns,
                           {static_cast<const std::byte*>(view.bytes.iov_base), view.bytes.iov_len});
        } catch (const std::exception& error) {
          reportSampleError(subscription, "CDR 录制缓存失败：" + std::string(error.what()));
        }
        try {
          NativeSample sample(subscription.decoder->descriptor());
          if (!ddsi_serdata_to_sample(samples[i], sample.data, nullptr, nullptr))
            throw std::runtime_error("CDR 反序列化失败");
          queuePlotRecord(subscription, sample.data, timestamp_ns);
        } catch (const std::exception& error) {
          reportSampleError(subscription, error.what());
        }
      }
      return;
    }
    std::array<void*, 32> samples{};
    std::array<dds_sample_info_t, 32> infos{};
    const auto count = checked(dds_take(subscription.reader.value, samples.data(), infos.data(), samples.size(), samples.size()),
                               "读取动态话题样本");
    Loan loan{subscription.reader.value, samples.data(), count};
    for (dds_return_t i = 0; i < count; ++i) {
      if (!infos[i].valid_data) continue;
      try {
        // 绘图时间轴取本机订阅读取时间，在解码及写入宿主队列之前采集。
        const auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        queuePlotRecord(subscription, samples[i], timestamp_ns);
      } catch (const std::exception& error) {
        reportSampleError(subscription, error.what());
      }
    }
  }

  // 记录 subscription 最近一次样本处理错误 error，重复出现同一文本时只通知一次。
  void reportSampleError(Subscription& subscription, const std::string& error) {
    if (subscription.last_error == error) return;
    subscription.last_error = error;
    notice(true, "DDS " + subscription.name + "：" + error);
  }

  // 用 subscription 的解码器展开 sample，以 timestamp_ns 作为图表时间。解码异常交给调用方。
  // 解码结果拥有独立存储，只有含字段的样本才入队。
  void queuePlotRecord(Subscription& subscription, const void* sample, int64_t timestamp_ns) {
    DdsDynamicRecord record{subscription.name, timestamp_ns, subscription.decoder->decode(sample)};
    if (record.fields.empty()) return;
    std::lock_guard lock(mutex);
    // 按样本数和字段总数限制宿主待处理队列，超限时淘汰最旧的绘图样本。
    while (!records.empty() && (records.size() >= 2048 || pending_fields + record.fields.size() > kMaxFields)) {
      pending_fields -= records.front().fields.size();
      records.pop_front();
      if (!overflow_reported && notices.size() < 128) {
        notices.push_back({true, "DDS 接收队列已满，丢弃最旧的待处理样本"});
        overflow_reported = true;
      }
    }
    pending_fields += record.fields.size();
    records.push_back(std::move(record));
  }

  // 接收线程入口，加载配置中的 IDL，再循环发现发布端、读取样本和建立订阅。
  // 未在单个话题内处理的异常会发送停止通知并退出循环。
  void run() {
    try {
      // 从布局直接恢复时，对话框可能未加载 IDL。后台线程补齐尚未加载的文件，发现匹配发布端后再创建 Reader。
      for (const auto& path : settings.idl_files) {
        if (!running) return;
        if (std::any_of(settings.idl_libraries.begin(), settings.idl_libraries.end(),
                        [&path](const auto& library) { return library->sourcePath() == path; })) continue;
        try {
          settings.idl_libraries.push_back(DdsIdlLibrary::load(path));
        } catch (const std::exception& error) {
          notice(true, "DDS IDL " + path + "：" + error.what());
        }
      }
      while (running) {
        std::array<dds_attach_t, 64> triggered{};
        checked(dds_waitset_wait(waitset, triggered.data(), triggered.size(), DDS_MSECS(100)), "等待 DDS 数据");
        if (!running) break;
        // 优先清理及读取已有订阅，再尝试可能阻塞于类型查询的新订阅。
        discover();
        removeUnusedSubscriptions();
        for (auto& [key, subscription] : subscriptions) { (void)key; receive(*subscription); }
        subscribeOne();
      }
    } catch (const std::exception& error) {
      notice(true, std::string("DDS 接收停止：") + error.what());
      running = false;
    }
  }

  DdsSettings settings;
  Entity participant;
  dds_entity_t subscriber = DDS_ENTITY_NIL, publications = DDS_ENTITY_NIL, waitset = DDS_ENTITY_NIL;
  std::map<dds_instance_handle_t, Candidate> candidates;
  std::map<std::pair<std::string, std::string>, std::unique_ptr<Subscription>> subscriptions;
  std::atomic<bool> running{true};
  std::thread worker;
  std::mutex mutex;
  std::deque<DdsDynamicRecord> records;
  std::vector<DdsDynamicNotice> notices;
  size_t pending_fields = 0;
  bool overflow_reported = false;
  std::unique_ptr<DdsRecorder> recorder;
};

// 将 settings 交给接收实现，初始化或启动失败时传播异常。
DdsDynamicStream::DdsDynamicStream(const DdsSettings& settings) : impl_(std::make_unique<Impl>(settings)) {}
// 销毁接收实现，由其等待后台线程并释放资源。
DdsDynamicStream::~DdsDynamicStream() = default;
// 停止当前接收实现，并等待剩余录制数据写入文件。
void DdsDynamicStream::stop() { impl_->stop(); }

// 在互斥锁保护下按入队顺序转移当前缓存的样本，重置绘图队列的字段统计和溢出提示状态。
std::vector<DdsDynamicRecord> DdsDynamicStream::takeRecords() {
  std::lock_guard lock(impl_->mutex);
  std::vector<DdsDynamicRecord> records;
  records.reserve(impl_->records.size());
  while (!impl_->records.empty()) {
    records.push_back(std::move(impl_->records.front()));
    impl_->records.pop_front();
  }
  impl_->pending_fields = 0;
  impl_->overflow_reported = false;
  return records;
}

// 交换通知队列以清空缓存，返回独立拥有通知的容器。
std::vector<DdsDynamicNotice> DdsDynamicStream::takeNotices() {
  std::lock_guard lock(impl_->mutex);
  std::vector<DdsDynamicNotice> notices;
  notices.swap(impl_->notices);
  return notices;
}
