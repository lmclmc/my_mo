#include "dds_robot_model.hpp"

#include <tinyxml2.h>
#include <algorithm>
#include <bit>
#include <cmath>
#include <filesystem>
#include <functional>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace {
namespace fs = std::filesystem;
using Vec = PJ::sdk::Vector3;
using Quat = PJ::sdk::Quaternion;
using Element = tinyxml2::XMLElement;

// 校验条件 ok，失败时抛出带 message 的模型诊断。
void require(bool ok, const std::string& message) {
  if (!ok) throw std::runtime_error(message);
}

// 返回 element 的 name 属性，元素或属性缺失时返回 fallback。
std::string attribute(const Element* element, const char* name, const std::string& fallback = {}) {
  const char* value = element ? element->Attribute(name) : nullptr;
  return value ? value : fallback;
}

// 将 path 解析到 doc。文件不存在、超过 32 MB 或无法解析时抛出异常。
void readXml(tinyxml2::XMLDocument& doc, const fs::path& path) {
  require(fs::is_regular_file(path), "模型文件不存在：" + path.string());
  require(fs::file_size(path) <= 32 * 1024 * 1024, "模型 XML 超过 32 MB：" + path.string());
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("无法解析模型文件：" + path.string() + "：" + doc.ErrorStr());
}

// 从 name 属性读取三个有限数值。缺失或内容为空时返回 fallback，格式错误时抛出异常。
Vec vectorAttribute(const Element* element, const char* name, Vec fallback = {}) {
  const auto text = attribute(element, name);
  if (text.empty()) return fallback;
  Vec result;
  std::istringstream input(text);
  require(static_cast<bool>(input >> result.x >> result.y >> result.z) &&
              std::isfinite(result.x) && std::isfinite(result.y) && std::isfinite(result.z),
          std::string("无效的 URDF ") + name + "：" + text);
  input >> std::ws;
  require(input.eof(), std::string("无效的 URDF ") + name + "：" + text);
  return result;
}

// 读取 name 属性中的有限浮点值。属性缺失时返回 fallback，解析失败时抛出异常。
double numberAttribute(const Element* element, const char* name, double fallback) {
  if (!element || !element->Attribute(name)) return fallback;
  double result = 0;
  require(element->QueryDoubleAttribute(name, &result) == tinyxml2::XML_SUCCESS && std::isfinite(result),
          std::string("无效的 URDF 数值：") + name);
  return result;
}

// 返回四元数的 Hamilton 乘积 a * b。用于组合旋转时先应用 b，再应用 a。
Quat multiply(const Quat& a, const Quat& b) {
  return {a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
          a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
          a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
          a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z};
}

// 将绕单位轴 axis 旋转 angle 弧度表示为 xyzw 四元数。
Quat axisAngle(Vec axis, double angle) {
  const double scale = std::sin(angle / 2);
  return {axis.x * scale, axis.y * scale, axis.z * scale, std::cos(angle / 2)};
}

// 用单位四元数 q 旋转向量 v，通过 q * v * 共轭(q) 取得旋转结果。
Vec rotate(const Quat& q, Vec v) {
  const auto r = multiply(multiply(q, {v.x, v.y, v.z, 0}), {-q.x, -q.y, -q.z, q.w});
  return {r.x, r.y, r.z};
}

// 就地解析 element 子树中的 mesh 路径，directory 是 URDF 所在目录。
// 通过话题提供的 URDF 在 3D 查看器中没有源目录，因此需要提前解析本地引用。
void resolveMeshes(Element* element, const fs::path& directory) {
  for (auto* child = element->FirstChildElement(); child; child = child->NextSiblingElement()) {
    if (std::string_view(child->Name()) == "mesh") {
      const auto uri = attribute(child, "filename");
      if (!uri.empty() && uri.find("://") == std::string::npos) {
        child->SetAttribute("filename", fs::absolute(directory / uri).lexically_normal().c_str());
      } else if (uri.starts_with("package://")) {
        const fs::path package_path = uri.substr(10);
        if (package_path.empty()) throw std::runtime_error("URDF 包含空的 package:// mesh 路径。");
        // 从 URDF 目录向上查找包。若当前目录就是包根目录，则在目录下匹配包内路径。
        for (auto parent = directory; !parent.empty(); parent = parent.parent_path()) {
          const auto candidate = parent / package_path;
          const auto inside = parent.filename() == *package_path.begin()
              ? parent / package_path.lexically_relative(*package_path.begin()) : fs::path{};
          if (fs::is_regular_file(candidate)) {
            child->SetAttribute("filename", candidate.c_str());
            break;
          }
          if (!inside.empty() && fs::is_regular_file(inside)) {
            child->SetAttribute("filename", inside.c_str());
            break;
          }
          if (parent == parent.root_path()) break;
        }
      }
    }
    resolveMeshes(child, directory);
  }
}

// 原地展开 element 中的 MJCF include，保留执行器顺序。
// directory 是当前文件目录。stack 记录当前引用链，total_bytes 累加文件大小，
// depth 记录引用深度。发现循环引用、无效文件或超出大小与深度限制时抛出异常。
void expandIncludes(Element* element, const fs::path& directory,
                    std::set<fs::path>& stack, size_t& total_bytes, unsigned depth) {
  require(depth < 32, "MuJoCo include 嵌套超过 32 层。");
  for (auto* child = element->FirstChildElement(); child;) {
    auto* next = child->NextSiblingElement();
    if (std::string_view(child->Name()) == "include") {
      const auto file = attribute(child, "file");
      require(!file.empty(), "MuJoCo include 缺少 file 属性。");
      const auto path = fs::canonical(directory / file);
      require(stack.insert(path).second, "MuJoCo include 循环引用：" + path.string());
      total_bytes += fs::file_size(path);
      require(total_bytes <= 32 * 1024 * 1024, "MuJoCo XML 及 include 总大小超过 32 MB。");
      tinyxml2::XMLDocument included;
      readXml(included, path);
      auto* root = included.RootElement();
      require(root != nullptr, "MuJoCo include 文件为空：" + path.string());
      expandIncludes(root, path.parent_path(), stack, total_bytes, depth + 1);
      // 跨文档复制每个子元素并依次插在原 include 后面，最后移除 include。
      auto* anchor = static_cast<tinyxml2::XMLNode*>(child);
      for (auto* item = root->FirstChildElement(); item; item = item->NextSiblingElement()) {
        auto* clone = item->DeepClone(element->GetDocument());
        element->InsertAfterChild(anchor, clone);
        anchor = clone;
      }
      element->DeleteChild(child);
      stack.erase(path);
    } else {
      expandIncludes(child, directory, stack, total_bytes, depth);
    }
    child = next;
  }
}

// 解析 filename 及其 include，按执行器出现顺序返回所引用的关节名称。
// 缺少关节引用、重复引用或超过执行器数量限制时抛出异常。
std::vector<std::string> actuatorOrder(const std::string& filename) {
  const auto path = fs::canonical(filename);
  tinyxml2::XMLDocument doc;
  readXml(doc, path);
  auto* root = doc.FirstChildElement("mujoco");
  require(root != nullptr, "MuJoCo XML 的根元素必须是 <mujoco>。");
  std::set<fs::path> stack{path};
  size_t total_bytes = fs::file_size(path);
  expandIncludes(root, path.parent_path(), stack, total_bytes, 0);
  std::vector<std::string> order;
  std::set<std::string> seen;
  for (auto* actuators = root->FirstChildElement("actuator"); actuators;
       actuators = actuators->NextSiblingElement("actuator")) {
    for (auto* motor = actuators->FirstChildElement(); motor; motor = motor->NextSiblingElement()) {
      const auto name = attribute(motor, "joint");
      require(!name.empty(), "MuJoCo actuator 必须通过 joint 属性引用关节，无法映射 tendon/site 执行器。");
      require(seen.insert(name).second, "MuJoCo actuator 重复引用关节：" + name);
      order.push_back(name);
      require(order.size() <= DdsRobotSettings::kMaxJoints, "MuJoCo 执行器数量超过 8192。");
    }
  }
  require(!order.empty(), "MuJoCo XML 中没有带 joint 属性的 actuator。");
  return order;
}

// 按 ROS 2 消息字段顺序写入小端 CDR，供 3D 查看器使用现有解析器解码。
class CdrWriter {
 public:
  std::vector<uint8_t> bytes{0, 1, 0, 0};  // CDR_LE 封装头。
  // 以 width 字节对齐并按小端写入 value。对齐从封装头之后开始计算。
  void integer(uint64_t value, size_t width) {
    while ((bytes.size() - 4) % width) bytes.push_back(0);
    for (size_t i = 0; i < width; ++i) bytes.push_back(static_cast<uint8_t>(value >> (i * 8)));
  }
  // 写入含结尾空字符的 32 位长度，再写入字符串。长度无法表示时抛出异常。
  void string(const std::string& value) {
    require(value.size() < std::numeric_limits<uint32_t>::max(), "CDR 字符串过长。");
    integer(value.size() + 1, 4);
    bytes.insert(bytes.end(), value.begin(), value.end());
    bytes.push_back(0);
  }
  // 保留双精度浮点数的位表示，按 8 字节宽度对齐写入。
  void real(double value) { integer(std::bit_cast<uint64_t>(value), 8); }
};
}  // namespace

// 先校验 URDF 关节树，再按 MuJoCo 执行器顺序建立 DDS 数组到关节名的映射。
DdsRobotModel::DdsRobotModel(const DdsRobotSettings& settings) : settings_(settings) {
  const auto error = settings.validationError();
  require(error.empty(), error);
  const auto path = fs::canonical(settings.urdf_path);
  tinyxml2::XMLDocument doc;
  readXml(doc, path);
  auto* root = doc.FirstChildElement("robot");
  require(root != nullptr, "URDF 的根元素必须是 <robot>；xacro 请先展开为 URDF。");
  std::set<std::string> links, children;
  for (auto* link = root->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
    const auto name = attribute(link, "name");
    require(!name.empty() && links.insert(name).second, "URDF 存在空名称或重复 link：" + name);
  }
  require(!links.empty() && links.size() <= DdsRobotSettings::kMaxJoints + 1, "URDF link 数量无效。");
  // 避免外部参考坐标系与 URDF 内任何连杆重名。
  while (links.count(world_frame_)) world_frame_ = "dds_" + world_frame_;
  for (auto* item = root->FirstChildElement("joint"); item; item = item->NextSiblingElement("joint")) {
    Joint joint;
    joint.name = attribute(item, "name");
    joint.type = attribute(item, "type");
    joint.parent = attribute(item->FirstChildElement("parent"), "link");
    joint.child = attribute(item->FirstChildElement("child"), "link");
    require(!joint.name.empty() && joint_indices_.emplace(joint.name, joints_.size()).second,
            "URDF 存在空名称或重复 joint：" + joint.name);
    require(links.count(joint.parent) && links.count(joint.child) && joint.parent != joint.child &&
                children.insert(joint.child).second, "URDF 关节的 parent/child 无效：" + joint.name);
    require(joint.type == "fixed" || joint.type == "revolute" || joint.type == "continuous" ||
                joint.type == "prismatic", "URDF 暂不支持的关节类型：" + joint.name + " (" + joint.type + ")");
    const auto* origin = item->FirstChildElement("origin");
    joint.origin = vectorAttribute(origin, "xyz");
    const auto rpy = vectorAttribute(origin, "rpy");
    // URDF 的滚转、俯仰、偏航依次应用，组合得到父连杆到静止关节坐标系的旋转。
    joint.rotation = multiply(multiply(axisAngle({0, 0, 1}, rpy.z), axisAngle({0, 1, 0}, rpy.y)),
                              axisAngle({1, 0, 0}, rpy.x));
    if (joint.type != "fixed") {
      joint.axis = vectorAttribute(item->FirstChildElement("axis"), "xyz", {1, 0, 0});
      const double norm = std::hypot(joint.axis.x, joint.axis.y, joint.axis.z);
      require(std::isfinite(norm) && norm > 0, "URDF 关节轴为零或无效：" + joint.name);
      joint.axis = {joint.axis.x / norm, joint.axis.y / norm, joint.axis.z / norm};
    }
    if (auto* mimic = item->FirstChildElement("mimic")) {
      joint.mimic = attribute(mimic, "joint");
      require(!joint.mimic.empty(), "URDF mimic 缺少 joint：" + joint.name);
      joint.multiplier = numberAttribute(mimic, "multiplier", 1);
      joint.offset = numberAttribute(mimic, "offset", 0);
    }
    joints_.push_back(std::move(joint));
    require(joints_.size() <= DdsRobotSettings::kMaxJoints, "URDF 关节数量超过 8192。");
  }
  // 从唯一的根连杆遍历整棵树，关节可按任意顺序声明。
  // 每个子连杆只有一个父关节，无法从根到达的节点意味着树不连通或有环。
  std::map<std::string, std::vector<std::string>> descendants;
  for (const auto& joint : joints_) descendants[joint.parent].push_back(joint.child);
  for (const auto& link : links) {
    if (!children.count(link)) {
      require(root_link_.empty(), "URDF 存在多个根 link。");
      root_link_ = link;
    }
  }
  require(!root_link_.empty(), "URDF 没有根 link（可能存在关节环）。");
  std::vector<std::string> pending{root_link_};
  for (size_t i = 0; i < pending.size(); ++i)
    for (const auto& child : descendants[pending[i]]) pending.push_back(child);
  require(pending.size() == links.size(), "URDF 关节树不连通或存在关节环。");
  for (const auto& joint : joints_)
    require(joint.mimic.empty() || joint_indices_.count(joint.mimic), "URDF mimic 引用了未知关节：" + joint.mimic);
  // 初始化时求值一次所有从动关系，提前发现 mimic 循环及超深的依赖链。
  for (size_t i = 0; i < joints_.size(); ++i) (void)position(i, 0);

  const auto order = actuatorOrder(settings.mujoco_path);
  // start + i 定位 XML 中的执行器，i 定位当前 DDS 话题的位置数组。
  for (size_t g = 0; g < settings.groups.size(); ++g) {
    const auto& group = settings.groups[g];
    require(!group.count || static_cast<size_t>(group.start + group.count) <= order.size(),
            std::string(DdsRobotSettings::kGroupNames[g]) + " 的关节范围超过 MuJoCo actuator 数量 " + std::to_string(order.size()));
    for (int i = 0; i < group.count; ++i) {
      const auto& name = order[group.start + i];
      const auto found = joint_indices_.find(name);
      require(found != joint_indices_.end(), "MuJoCo 关节在 URDF 中不存在：" + name);
      const auto& joint = joints_[found->second];
      require(joint.type != "fixed" && joint.mimic.empty(), "执行器不能映射到 fixed/mimic 关节：" + name);
      group_joints_[g].push_back(found->second);
      mapping_.push_back({DdsRobotSettings::kGroupNames[g], group.topic, name, i, group.start + i});
    }
  }
  resolveMeshes(root, path.parent_path());
  tinyxml2::XMLPrinter printer;
  doc.Print(&printer);
  description_ = printer.CStr();
}

// 返回直接关节的位置，或递归应用 mimic 的比例和偏移。
// depth 是当前递归深度，依赖过深或结果不是有限数值时抛出异常。
double DdsRobotModel::position(size_t index, unsigned depth) const {
  require(depth < 64, "URDF mimic 存在环或嵌套超过 64 层。");
  const auto& joint = joints_[index];
  const double value = joint.mimic.empty() ? joint.position
      : position(joint_indices_.at(joint.mimic), depth + 1) * joint.multiplier + joint.offset;
  require(std::isfinite(value), "关节位置非有限数值：" + joint.name);
  return value;
}

// 按 record 所属话题选择姿态或关节组，整条消息校验成功后才写入状态。
bool DdsRobotModel::update(const DdsDynamicRecord& record) {
  if (record.topic == DdsRobotSettings::kStateEstimatorTopic) {
    std::array<double, 4> quat{};
    std::array<bool, 4> seen{};
    for (const auto& field : record.fields) {
      if (!field.name.starts_with("quat/[")) continue;
      const auto suffix = std::string_view(field.name).substr(6);
      require(suffix.size() == 2 && suffix[0] >= '0' && suffix[0] <= '3' && suffix[1] == ']',
              "state_estimator.quat 的索引必须为 0–3。");
      const size_t index = static_cast<size_t>(suffix[0] - '0');
      require(!seen[index], "state_estimator.quat 的数组索引重复。");
      quat[index] = std::visit([](const auto& value) -> double {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) return static_cast<double>(value);
        else throw std::runtime_error("state_estimator.quat 必须是数值类型。");
      }, field.value);
      require(std::isfinite(quat[index]), "state_estimator.quat 包含 NaN/Inf。");
      seen[index] = true;
    }
    require(std::all_of(seen.begin(), seen.end(), [](bool value) { return value; }),
            "state_estimator.quat 必须包含完整的四元数 [w, x, y, z]。");
    // 先按绝对值最大的分量缩放，再求模，可避免极大或极小输入直接求模时溢出或下溢。
    double scale = 0;
    for (const double value : quat) scale = std::max(scale, std::abs(value));
    require(scale > 0, "state_estimator.quat 不能为零四元数。");
    for (double& value : quat) value /= scale;
    const double norm = std::hypot(std::hypot(quat[0], quat[1]), std::hypot(quat[2], quat[3]));
    // X1 使用 wxyz，SDK/TF 使用 xyzw。该姿态将机体向量转到外部参考系，
    // 可直接作为外部参考系到根连杆的 TF 旋转。
    base_rotation_ = {quat[1] / norm, quat[2] / norm, quat[3] / norm, quat[0] / norm};
    return true;
  }
  for (size_t g = 0; g < settings_.groups.size(); ++g) {
    const auto& group = settings_.groups[g];
    if (!group.count || record.topic != group.topic) continue;
    // 按 prefix 提取形如 q/[索引] 的标量，按索引收集并校验数值和重复项。
    auto readArray = [&](const std::string& prefix) {
      std::map<size_t, double> values;
      for (const auto& field : record.fields) {
        if (!field.name.starts_with(prefix)) continue;
        const auto suffix = std::string_view(field.name).substr(prefix.size());
        if (suffix.size() < 2 || suffix.back() != ']') continue;
        size_t index = 0;
        bool valid = true;
        for (const char c : suffix.substr(0, suffix.size() - 1)) {
          if (c < '0' || c > '9' || index > DdsRobotSettings::kMaxJoints) { valid = false; break; }
          index = index * 10 + static_cast<unsigned>(c - '0');
        }
        if (!valid) continue;
        const auto value = std::visit([](const auto& v) -> double {
          using T = std::decay_t<decltype(v)>;
          if constexpr (std::is_arithmetic_v<T> && !std::is_same_v<T, bool>) return static_cast<double>(v);
          else throw std::runtime_error("关节位置必须是数值类型。");
        }, field.value);
        require(std::isfinite(value), record.topic + " 的关节位置包含 NaN/Inf。");
        require(values.emplace(index, value).second, record.topic + " 的关节数组索引重复。");
      }
      return values;
    };
    auto values = readArray("q/[");
    // 兼容 X1 控制器手臂话题中的 joint_curPos 数组。
    if (values.empty() && g == 2) values = readArray("joint_curPos/[");
    require(values.size() == static_cast<size_t>(group.count), record.topic +
            " 的位置数组长度与映射不符：期望 " + std::to_string(group.count) +
            "，收到 " + std::to_string(values.size()) + "（arm 兼容 joint_curPos）。");
    for (int i = 0; i < group.count; ++i)
      require(values.count(i), record.topic + " 的位置数组索引不连续。");
    // 数量和连续索引均通过校验后，才批量更新这一组关节。
    for (int i = 0; i < group.count; ++i) joints_[group_joints_[g][i]].position = values.at(i);
    return true;
  }
  return false;
}

// 将当前姿态和关节位置转成带 timestamp_ns 的 TF 边，先输出根变换，再输出各关节变换。
PJ::sdk::FrameTransforms DdsRobotModel::transforms(int64_t timestamp_ns) const {
  PJ::sdk::FrameTransforms result;
  result.transforms.reserve(joints_.size() + 1);
  result.transforms.push_back({timestamp_ns, world_frame_, root_link_, {}, base_rotation_});
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto& joint = joints_[i];
    auto translation = joint.origin;
    auto rotation = joint.rotation;
    const double q = position(i, 0);
    // 关节轴定义在静止关节坐标系。转动关节叠加旋转，移动关节先将位移转到父坐标系。
    if (joint.type == "revolute" || joint.type == "continuous") {
      rotation = multiply(rotation, axisAngle(joint.axis, q));
    } else if (joint.type == "prismatic") {
      const auto delta = rotate(rotation, {joint.axis.x * q, joint.axis.y * q, joint.axis.z * q});
      translation.x += delta.x; translation.y += delta.y; translation.z += delta.z;
    }
    result.transforms.push_back({timestamp_ns, joint.parent, joint.child, translation, rotation});
  }
  return result;
}

// 将 URDF 文本写为 ROS 2 String 的单个字段并返回完整 CDR 消息。
std::vector<uint8_t> ddsRobotDescriptionCdr(const std::string& description) {
  CdrWriter writer;
  writer.string(description);
  return std::move(writer.bytes);
}

// 以 TFMessage 的数组格式编码 transforms，每条记录包含时间、坐标系名、平移和旋转。
std::vector<uint8_t> ddsRobotTransformsCdr(const PJ::sdk::FrameTransforms& transforms) {
  CdrWriter writer;
  writer.integer(transforms.transforms.size(), 4);
  for (const auto& transform : transforms.transforms) {
    // 将纳秒时间拆为 ROS 2 Time 的 32 位秒数和纳秒余数。
    const int64_t seconds = transform.timestamp / 1000000000;
    require(seconds >= 0 && seconds <= std::numeric_limits<int32_t>::max(), "TF 时间戳超出 ROS 2 Time 范围。");
    writer.integer(static_cast<uint32_t>(seconds), 4);
    writer.integer(static_cast<uint32_t>(transform.timestamp % 1000000000), 4);
    writer.string(transform.parent_frame_id);
    writer.string(transform.child_frame_id);
    for (const double value : {transform.translation.x, transform.translation.y, transform.translation.z,
                               transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w})
      writer.real(value);
  }
  return std::move(writer.bytes);
}
