#include "mcap_robot_model.hpp"

#include <tinyxml2.h>
#include <algorithm>
#include <bit>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
using Element = tinyxml2::XMLElement;
using Vec = PJ::sdk::Vector3;
using Quat = PJ::sdk::Quaternion;

void require(bool ok, const std::string& text) { if (!ok) throw std::runtime_error(text); }
std::string attr(const Element* e, const char* name) {
  const char* value = e ? e->Attribute(name) : nullptr;
  return value ? value : std::string{};
}
Vec vec(const Element* e, const char* name, Vec fallback = {}) {
  const auto text = attr(e, name);
  if (text.empty()) return fallback;
  std::istringstream in(text); Vec value;
  require(static_cast<bool>(in >> value.x >> value.y >> value.z), "URDF 数值属性无效：" + text);
  return value;
}
double number(const Element* e, const char* name, double fallback) {
  if (!e || !e->Attribute(name)) return fallback;
  double value = 0;
  require(e->QueryDoubleAttribute(name, &value) == tinyxml2::XML_SUCCESS && std::isfinite(value),
          "URDF 数值属性无效：" + std::string(name));
  return value;
}
Quat multiply(const Quat& a, const Quat& b) {
  return {a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
          a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
          a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
          a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z};
}
Quat axisAngle(Vec axis, double angle) {
  const double s = std::sin(angle / 2.0);
  return {axis.x*s, axis.y*s, axis.z*s, std::cos(angle / 2.0)};
}
Vec rotate(const Quat& q, Vec v) {
  const auto r = multiply(multiply(q, {v.x, v.y, v.z, 0}), {-q.x, -q.y, -q.z, q.w});
  return {r.x, r.y, r.z};
}
void collectActuators(const Element* root, const std::filesystem::path& directory,
                      std::set<std::filesystem::path>& stack, std::set<std::string>& seen,
                      std::vector<std::string>& result) {
  for (auto* group = root->FirstChildElement("actuator"); group; group = group->NextSiblingElement("actuator"))
    for (auto* item = group->FirstChildElement(); item; item = item->NextSiblingElement()) {
      const auto joint = attr(item, "joint");
      if (!joint.empty() && seen.insert(joint).second) result.push_back(joint);
    }
  for (auto* include = root->FirstChildElement("include"); include; include = include->NextSiblingElement("include")) {
    const auto file = attr(include, "file");
    require(!file.empty(), "MuJoCo include 缺少 file 属性。");
    const auto included_path = std::filesystem::canonical(directory / file);
    require(stack.insert(included_path).second, "MuJoCo include 存在循环引用：" + included_path.string());
    tinyxml2::XMLDocument included;
    require(included.LoadFile(included_path.c_str()) == tinyxml2::XML_SUCCESS,
            "无法读取 MuJoCo include：" + included_path.string());
    auto* included_root = included.FirstChildElement("mujoco");
    require(included_root != nullptr, "MuJoCo include 根元素必须是 <mujoco>：" + included_path.string());
    collectActuators(included_root, included_path.parent_path(), stack, seen, result);
    stack.erase(included_path);
  }
}
std::vector<std::string> actuatorOrder(const std::string& path) {
  tinyxml2::XMLDocument doc;
  require(doc.LoadFile(path.c_str()) == tinyxml2::XML_SUCCESS, "无法解析 MuJoCo XML：" + path);
  auto* root = doc.FirstChildElement("mujoco");
  require(root != nullptr, "MuJoCo XML 根元素必须是 <mujoco>。");
  std::vector<std::string> result;
  std::set<std::string> seen;
  const auto canonical = std::filesystem::canonical(path);
  std::set<std::filesystem::path> stack{canonical};
  collectActuators(root, canonical.parent_path(), stack, seen, result);
  require(!result.empty(), "MuJoCo XML 中没有带 joint 属性的 actuator。");
  return result;
}

class CdrReader {
 public:
  CdrReader(const uint8_t* data, size_t size) : data_(data), size_(size) {}
  bool floats(size_t count, std::vector<double>& out) {
    if (size_ < 4) return false;
    for (size_t i = 0; i < count; ++i) {
      if (!align(4) || pos_ + 4 > size_) return false;
      const uint32_t bits = uint32_t(data_[pos_]) | (uint32_t(data_[pos_+1]) << 8) |
                            (uint32_t(data_[pos_+2]) << 16) | (uint32_t(data_[pos_+3]) << 24);
      pos_ += 4;
      const float value = std::bit_cast<float>(bits);
      if (!std::isfinite(value)) return false;
      out.push_back(value);
    }
    return true;
  }
 private:
  bool align(size_t width) { while ((pos_ - 4) % width) { if (pos_++ >= size_) return false; } return true; }
  const uint8_t* data_; size_t size_, pos_ = 4;
};
class CdrWriter {
 public:
  std::vector<uint8_t> bytes{0, 1, 0, 0};
  void integer(uint64_t value, size_t width) { while ((bytes.size()-4)%width) bytes.push_back(0); for (size_t i=0;i<width;++i) bytes.push_back(uint8_t(value>>(i*8))); }
  void string(const std::string& value) { require(value.size() < UINT32_MAX, "CDR 字符串过长。"); integer(value.size()+1,4); bytes.insert(bytes.end(),value.begin(),value.end()); bytes.push_back(0); }
  void real(double value) { integer(std::bit_cast<uint64_t>(value),8); }
};
}

McapRobotModel::McapRobotModel(const std::string& urdf_path, const std::string& mujoco_path) {
  std::ifstream input(urdf_path);
  require(input.good(), "无法读取 URDF：" + urdf_path);
  description_ = std::string(std::istreambuf_iterator<char>(input), {});
  tinyxml2::XMLDocument doc;
  require(doc.LoadFile(urdf_path.c_str()) == tinyxml2::XML_SUCCESS, "无法解析 URDF：" + urdf_path);
  auto* root = doc.FirstChildElement("robot");
  require(root != nullptr, "URDF 根元素必须是 <robot>。");
  std::set<std::string> links, children;
  for (auto* item=root->FirstChildElement("link"); item; item=item->NextSiblingElement("link"))
    require(links.insert(attr(item,"name")).second && !attr(item,"name").empty(), "URDF link 名称重复或为空。");
  for (auto* item=root->FirstChildElement("joint"); item; item=item->NextSiblingElement("joint")) {
    Joint joint;
    joint.name=attr(item,"name"); joint.type=attr(item,"type");
    joint.parent=attr(item->FirstChildElement("parent"),"link"); joint.child=attr(item->FirstChildElement("child"),"link");
    require(!joint.name.empty() && !joint.parent.empty() && !joint.child.empty() && links.count(joint.parent) && links.count(joint.child), "URDF joint 无效："+joint.name);
    const auto* origin=item->FirstChildElement("origin"); const auto rpy=vec(origin,"rpy"); joint.origin=vec(origin,"xyz");
    joint.rotation=multiply(multiply(axisAngle({0,0,1},rpy.z),axisAngle({0,1,0},rpy.y)),axisAngle({1,0,0},rpy.x));
    if (joint.type!="fixed") { joint.axis=vec(item->FirstChildElement("axis"),"xyz",{1,0,0}); const auto n=std::hypot(joint.axis.x,std::hypot(joint.axis.y,joint.axis.z)); require(n>0,"URDF joint 轴无效："+joint.name); joint.axis={joint.axis.x/n,joint.axis.y/n,joint.axis.z/n}; }
    if (auto* mimic=item->FirstChildElement("mimic")) { joint.mimic=attr(mimic,"joint"); joint.multiplier=number(mimic,"multiplier",1); joint.offset=number(mimic,"offset",0); }
    children.insert(joint.child); joints_.push_back(std::move(joint));
  }
  for (const auto& link:links) if (!children.count(link)) { require(root_link_.empty(), "URDF 存在多个根 link。"); root_link_=link; }
  require(!root_link_.empty(), "URDF 没有根 link。");
  actuator_order_=actuatorOrder(mujoco_path);
  const std::array<std::pair<int,int>,4> ranges{{{0,12},{12,3},{17,14},{15,2}}};
  const std::array<std::vector<size_t>*,4> groups{{&leg_,&waist_,&arm_,&head_}};
  for (size_t g=0;g<ranges.size();++g) for (int i=0;i<ranges[g].second;++i) {
    const int index=ranges[g].first+i; require(index<int(actuator_order_.size()), "MuJoCo actuator 数量不足。");
    auto it=std::find_if(joints_.begin(),joints_.end(),[&](const Joint& j){return j.name==actuator_order_[index];});
    require(it!=joints_.end(), "MuJoCo joint 不在 URDF 中："+actuator_order_[index]); groups[g]->push_back(size_t(it-joints_.begin()));
  }
}

double McapRobotModel::position(size_t index, unsigned depth) const {
  require(depth<64,"URDF mimic 依赖存在循环。"); const auto& j=joints_[index];
  if (j.mimic.empty()) return j.position;
  const auto it=std::find_if(joints_.begin(),joints_.end(),[&](const Joint& x){return x.name==j.mimic;});
  require(it!=joints_.end(),"URDF mimic 引用未知关节："+j.mimic);
  return position(size_t(it-joints_.begin()),depth+1)*j.multiplier+j.offset;
}

bool decodeMcapRobotMessage(std::string_view topic, const uint8_t* data, size_t size, std::vector<double>& values, std::array<double,4>& quat, bool& has_quat) {
  const auto bare = topic.starts_with('/') ? topic.substr(1) : topic;
  size_t count=0, offset=0; bool is_quat=false;
  if (bare=="leg_control_data") count=12; else if (bare=="waist_control_data") count=3;
  else if (bare=="arm_control_data") count=14; else if (bare=="head_control_data") count=2;
  else if (bare=="state_estimator") { count=31; offset=27; is_quat=true; } else return false;
  CdrReader reader(data,size); std::vector<double> all;
  if (!reader.floats(count,all)) return false;
  if (is_quat) { for(size_t i=0;i<4;++i) quat[i]=all[offset+i]; has_quat=true; values.clear(); }
  else values=std::move(all);
  return true;
}

bool McapRobotModel::update(std::string_view topic, const std::vector<uint8_t>& payload) {
  std::vector<double> values; std::array<double,4> q{}; bool has=false;
  if (!decodeMcapRobotMessage(topic,payload.data(),payload.size(),values,q,has)) return false;
  if (has) { double scale=0; for(auto v:q) scale=std::max(scale,std::abs(v)); require(scale>0,"state_estimator.quat 不能为零。"); for(auto& v:q)v/=scale; const double n=std::hypot(std::hypot(q[0],q[1]),std::hypot(q[2],q[3])); base_rotation_={q[1]/n,q[2]/n,q[3]/n,q[0]/n}; return true; }
  const auto bare=topic.starts_with('/')?topic.substr(1):topic; const std::vector<size_t>* group=nullptr;
  if(bare=="leg_control_data")group=&leg_; else if(bare=="waist_control_data")group=&waist_; else if(bare=="arm_control_data")group=&arm_; else if(bare=="head_control_data")group=&head_;
  if(!group || group->size()!=values.size()) throw std::runtime_error("控制话题数组长度不匹配："+std::string(topic));
  for (size_t i = 0; i < values.size(); ++i) {
    joints_[(*group)[i]].position = values[i];
  }
  return true;
}

PJ::sdk::FrameTransforms McapRobotModel::transforms(int64_t stamp) const {
  PJ::sdk::FrameTransforms out; out.transforms.reserve(joints_.size()+1); out.transforms.push_back({stamp,world_frame_,root_link_,{},base_rotation_});
  for(const auto& j:joints_) { auto t=j.origin; auto r=j.rotation; const auto value=position(size_t(&j-joints_.data()),0); if(j.type=="revolute"||j.type=="continuous")r=multiply(r,axisAngle(j.axis,value)); else if(j.type=="prismatic"){auto d=rotate(r,{j.axis.x*value,j.axis.y*value,j.axis.z*value});t.x+=d.x;t.y+=d.y;t.z+=d.z;} out.transforms.push_back({stamp,j.parent,j.child,t,r}); }
  return out;
}

std::vector<uint8_t> mcapRobotDescriptionCdr(const std::string& text) { CdrWriter w; w.string(text); return std::move(w.bytes); }
std::vector<uint8_t> mcapRobotTransformsCdr(const PJ::sdk::FrameTransforms& transforms) { CdrWriter w; w.integer(transforms.transforms.size(),4); for(const auto& t:transforms.transforms){const auto sec=t.timestamp/1000000000; require(sec>=0&&sec<=INT32_MAX,"TF 时间戳超出 ROS 2 范围。"); w.integer(uint32_t(sec),4); w.integer(uint32_t(t.timestamp%1000000000),4); w.string(t.parent_frame_id); w.string(t.child_frame_id); for(const auto v:{t.translation.x,t.translation.y,t.translation.z,t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w})w.real(v);} return std::move(w.bytes); }
