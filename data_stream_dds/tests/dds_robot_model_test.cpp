#include "dds_robot_model.hpp"

#include <nlohmann/json.hpp>
#include <pj_base/builtin/robot_description.hpp>
#include <pj_base/sdk/plugin_data_api.hpp>
#include <pj_base/sdk/service_traits.hpp>
#include <pj_base/sdk/testing/parser_write_recorder.hpp>
#include <pj_plugins/host/message_parser_handle.hpp>
#include <pj_plugins/host/service_registry_builder.hpp>
#include <dlfcn.h>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

namespace {
/// 断言 success 为真，失败时抛出包含 reason 的异常，无返回值。
void check(bool success, const std::string& reason) {
  if (!success) throw std::runtime_error(reason);
}
/// 校验 actual 为有限值且与 expected 相差小于 1e-10。失败时抛出异常，无返回值。
void near(double actual, double expected) {
  check(std::isfinite(actual) && std::abs(actual - expected) < 1e-10,
        "Expected " + std::to_string(expected) + ", got " + std::to_string(actual));
}
/// 执行 fn，要求其抛出标准异常以验证输入被拒绝，无返回值。
template <typename Fn>
void rejects(Fn fn) {
  bool rejected = false;
  try { fn(); } catch (const std::exception&) { rejected = true; }
  check(rejected, "Invalid mapping or sample was accepted");
}
/// 返回测试夹具的模型文件路径以及用于验证执行器顺序的四组关节配置。
DdsRobotSettings fixtureSettings() {
  DdsRobotSettings settings;
  settings.enabled = true;
  settings.urdf_path = std::string(PJ_DDS_TEST_FIXTURES) + "/robot.urdf";
  settings.mujoco_path = std::string(PJ_DDS_TEST_FIXTURES) + "/robot.xml";
  settings.groups = {{{"leg_control_data", 0, 1}, {"waist_control_data", 1, 1},
                      {"arm_control_data", 3, 1}, {"head_control_data", 2, 1}}};
  return settings;
}
/// 在 transforms 中查找子坐标系 child 对应的变换，返回容器内元素的引用。
/// 找不到时抛出异常。
const PJ::sdk::FrameTransform& edge(const PJ::sdk::FrameTransforms& transforms, std::string_view child) {
  for (const auto& transform : transforms.transforms)
    if (transform.child_frame_id == child) return transform;
  throw std::runtime_error("Missing transform for " + std::string(child));
}
/// 比较 rotation 的四个分量与预期的 x、y、z、w，任一分量偏差过大时抛出异常。
void rotationNear(const PJ::sdk::Quaternion& rotation, double x, double y, double z, double w) {
  near(rotation.x, x);
  near(rotation.y, y);
  near(rotation.z, z);
  near(rotation.w, w);
}

/// 验证模型映射、关节运动学及无效输入拒绝，断言失败时抛出异常，无返回值。
void testModel() {
  auto settings = fixtureSettings();
  DdsRobotModel model(settings);
  check(model.rootLink() == "base", "Wrong root link");
  check(model.mapping().size() == 4 && model.mapping()[0].joint == "leg_joint" &&
        model.mapping()[2].joint == "arm_joint" && model.mapping()[3].xml_index == 2,
        "Mapping follows URDF declaration order instead of MuJoCo actuator order");
  check(model.description().find(std::string(PJ_DDS_TEST_FIXTURES) + "/meshes/base.stl") != std::string::npos,
        "Relative meshes were not resolved for a topic-sourced model");
  check(!model.update({"leg_control_command", 0, {{"q/[0]", 9.0}}}), "Command topic changed robot pose");
  check(model.update({"leg_control_data", 0, {{"q/[0]", 0.5f}}}), "Leg update missing");
  check(model.update({"waist_control_data", 0, {{"q/[0]", 0.25}}}), "Waist update missing");
  check(model.update({"head_control_data", 0, {{"q/[0]", -0.4}}}), "Head update missing");
  check(model.update({"arm_control_data", 0, {{"q/[0]", 0.6}, {"joint_curPos/[0]", 2.0}}}), "Arm update missing");
  const auto tf = model.transforms(1790000000123456789);
  check(tf.transforms.size() == 7, "Root, fixed or mimic transforms missing");
  for (const auto& transform : tf.transforms) check(transform.timestamp == 1790000000123456789, "Wrong timestamp");
  near(edge(tf, "leg_link").rotation.z, std::sin(0.25));
  near(edge(tf, "mimic_link").rotation.z, std::sin(0.55));
  near(edge(tf, "waist_link").translation.x, 1);
  near(edge(tf, "waist_link").translation.y, 0.25);
  near(edge(tf, "head_link").rotation.x, std::sin(-0.2));
  near(edge(tf, "arm_link").rotation.x, -std::sqrt(0.5) * std::sin(0.3));
  near(edge(tf, "arm_link").rotation.z, std::sqrt(0.5) * std::cos(0.3));
  near(edge(tf, "tool_link").translation.z, 3);
  check(edge(tf, "tool_link").parent_frame_id == "arm_link", "Broken parent-child graph");
  // 无效样本包含多余索引、缺失索引和错误值类型，逐一验证上一帧有效关节状态仍可读取。
  rejects([&] { model.update({"leg_control_data", 0, {{"q/[0]", 1.0}, {"q/[1]", 2.0}}}); });
  rejects([&] { model.update({"leg_control_data", 0, {{"q/[1]", 1.0}}}); });
  rejects([&] { model.update({"leg_control_data", 0, {{"q/[0]", std::numeric_limits<double>::quiet_NaN()}}}); });
  rejects([&] { model.update({"leg_control_data", 0, {{"q/[0]", std::string("invalid")}}}); });
  rejects([&] { model.update({"leg_control_data", 0, {}}); });
  near(edge(model.transforms(0), "leg_link").rotation.z, std::sin(0.25));
  check(model.update({"arm_control_data", 0, {{"joint_curPos/[0]", -0.2}}}), "X1 arm field not supported");
  near(edge(model.transforms(0), "arm_link").rotation.x, std::sqrt(0.5) * std::sin(0.1));

  auto bad = settings;
  bad.groups[0].count = 20;
  rejects([&] { DdsRobotModel invalid(bad); });
  bad = settings;
  bad.groups[1].start = 0;
  rejects([&] { DdsRobotModel invalid(bad); });
  bad = settings;
  bad.groups[1].topic = bad.groups[0].topic;
  rejects([&] { DdsRobotModel invalid(bad); });
  bad = settings;
  bad.urdf_path += ".missing";
  rejects([&] { DdsRobotModel invalid(bad); });
  bad = settings;
  bad.mujoco_path = std::string(PJ_DDS_TEST_FIXTURES) + "/cycle.xml";
  rejects([&] { DdsRobotModel invalid(bad); });
  std::cout << "PASS: unordered URDF, MJCF includes and actuator types, four data topics, q priority, "
               "arm compatibility, axis/origin kinematics, fixed/mimic joints, atomic rejection and invalid mappings\n";
}

/// 验证根姿态订阅、四元数分量顺序、归一化和无效姿态处理，无返回值。
void testOrientation() {
  auto settings = fixtureSettings();
  std::set<std::string> topics{"existing_topic"};
  settings.addTopics(topics);
  check(topics.size() == 6 && topics.count("state_estimator") &&
        topics.count("leg_control_data") && topics.count("waist_control_data") &&
        topics.count("arm_control_data") && topics.count("head_control_data"),
        "Enabled robot did not subscribe to state_estimator and all joint groups");
  settings.enabled = false;
  std::set<std::string> disabled_topics{"existing_topic"};
  settings.addTopics(disabled_topics);
  check(disabled_topics == std::set<std::string>{"existing_topic"},
        "Disabled robot added subscriptions");

  DdsRobotModel model(settings);
  check(model.worldFrame() == "world", "Wrong default world frame");
  const auto initial = model.transforms(123);
  check(initial.transforms.front().parent_frame_id == model.worldFrame() &&
        initial.transforms.front().child_frame_id == model.rootLink(),
        "World-to-root transform must precede the joint transforms");
  const auto& root = edge(initial, model.rootLink());
  rotationNear(root.rotation, 0, 0, 0, 1);
  near(root.translation.x, 0);
  near(root.translation.y, 0);
  near(root.translation.z, 0);

  check(model.update({"leg_control_data", 0, {{"q/[0]", 0.5}}}), "Joint update missing");
  const double half_sqrt = std::sqrt(0.5);
  check(model.update({"state_estimator", 0,
                      {{"quat/[3]", 0.0}, {"quat/[1]", 2.0}, {"height", 0.6},
                       {"quat/[0]", 2.0}, {"quat/[2]", 0.0}}}),
        "Unordered, scaled quaternion update missing");
  auto pose = model.transforms(456);
  rotationNear(edge(pose, model.rootLink()).rotation, half_sqrt, 0, 0, half_sqrt);
  near(edge(pose, "leg_link").rotation.z, std::sin(0.25));
  check(model.update({"waist_control_data", 0, {{"q/[0]", 0.25}}}), "Waist update missing");
  rotationNear(edge(model.transforms(0), model.rootLink()).rotation, half_sqrt, 0, 0, half_sqrt);

  check(model.update({"state_estimator", 0,
                      {{"quat/[0]", half_sqrt}, {"quat/[1]", 0.0f},
                       {"quat/[2]", half_sqrt}, {"quat/[3]", 0.0}}}),
        "Y-axis quaternion update missing");
  rotationNear(edge(model.transforms(0), model.rootLink()).rotation, 0, half_sqrt, 0, half_sqrt);
  const double large = std::numeric_limits<double>::max();
  // 最大有限值与最小次正规值分别覆盖平方求和时可能上溢或下溢的输入。
  check(model.update({"state_estimator", 0,
                      {{"quat/[0]", large}, {"quat/[1]", 0.0},
                       {"quat/[2]", 0.0}, {"quat/[3]", large}}}),
        "Large finite quaternion rejected");
  rotationNear(edge(model.transforms(0), model.rootLink()).rotation, 0, 0, half_sqrt, half_sqrt);
  const double small = std::numeric_limits<double>::denorm_min();
  check(model.update({"state_estimator", 0,
                      {{"quat/[0]", small}, {"quat/[1]", 0.0},
                       {"quat/[2]", small}, {"quat/[3]", 0.0}}}),
        "Small finite quaternion rejected");
  rotationNear(edge(model.transforms(0), model.rootLink()).rotation, 0, half_sqrt, 0, half_sqrt);

  const auto before = model.transforms(789);
  const auto invalid = [&](std::vector<DdsDynamicField> fields) {
    rejects([&] { model.update({"state_estimator", 0, std::move(fields)}); });
    check(model.transforms(789).transforms == before.transforms,
          "Invalid quaternion changed the root or retained joint state");
  };
  invalid({});
  invalid({{"quat/[0]", 1.0}, {"quat/[1]", 0.0}, {"quat/[2]", 0.0}});
  invalid({{"quat/[0]", 1.0}, {"quat/[1]", 0.0}, {"quat/[2]", 0.0}, {"quat/[4]", 0.0}});
  invalid({{"quat/[0]", 1.0}, {"quat/[1]", 0.0}, {"quat/[2]", 0.0}, {"quat/[3]", 0.0},
           {"quat/[4]", 0.0}});
  invalid({{"quat/[0]", 1.0}, {"quat/[1]", 0.0}, {"quat/[2]", 0.0}, {"quat/[-1]", 0.0}});
  invalid({{"quat/[0]", 1.0}, {"quat/[1]", 0.0}, {"quat/[2]", 0.0}, {"quat/[2]", 0.0}});
  invalid({{"quat/[0]", 1.0}, {"quat/[1]", 0.0}, {"quat/[2]", 0.0}, {"quat/[3]", 0.0},
           {"quat/[3]", 0.0}});
  invalid({{"quat/[0]", std::string("invalid")}, {"quat/[1]", 0.0},
           {"quat/[2]", 0.0}, {"quat/[3]", 0.0}});
  invalid({{"quat/[0]", true}, {"quat/[1]", 0.0}, {"quat/[2]", 0.0}, {"quat/[3]", 0.0}});
  invalid({{"quat/[0]", std::numeric_limits<double>::quiet_NaN()}, {"quat/[1]", 0.0},
           {"quat/[2]", 0.0}, {"quat/[3]", 0.0}});
  invalid({{"quat/[0]", 1.0}, {"quat/[1]", std::numeric_limits<double>::infinity()},
           {"quat/[2]", 0.0}, {"quat/[3]", 0.0}});
  invalid({{"quat/[0]", 0.0}, {"quat/[1]", 0.0}, {"quat/[2]", 0.0}, {"quat/[3]", 0.0}});
  rejects([&] { model.update({"leg_control_data", 0, {{"q/[0]", true}}}); });
  check(model.transforms(789).transforms == before.transforms,
        "Invalid joint sample changed the retained orientation or joint state");
  check(!model.update({"unrelated_state_estimator", 0, {{"quat/[0]", 1.0}}}),
        "Unrelated topic changed the orientation");
  check(model.transforms(789).transforms == before.transforms, "Unrelated topic changed retained pose");
  std::cout << "PASS: state_estimator subscriptions, world-to-root pose, wxyz ordering, normalized rotations, "
               "independent joint/orientation state and atomic malformed-quaternion rejection\n";
}

/// 使用 urdf 和 xml 指定的 X1 模型验证 31 个关节及根姿态，失败时抛出异常，无返回值。
void testX1(const char* urdf, const char* xml) {
  DdsRobotSettings settings;
  settings.urdf_path = urdf;
  settings.mujoco_path = xml;
  DdsRobotModel model(settings);
  check(model.mapping().size() == 31 && model.rootLink() == "pelvis", "Wrong X1 mapping");
  check(model.mapping()[0].joint == "left_hip_pitch_joint" &&
        model.mapping()[11].joint == "right_ankle_roll_joint" &&
        model.mapping()[12].joint == "waist_yaw_joint" &&
        model.mapping()[15].joint == "left_shoulder_pitch_joint" &&
        model.mapping()[29].joint == "head_yaw_joint", "X1 group boundary mismatch");
  const auto before = model.transforms(123);
  for (size_t g = 0; g < settings.groups.size(); ++g) {
    DdsDynamicRecord record{settings.groups[g].topic, 1790000000123456789, {}};
    for (int i = 0; i < settings.groups[g].count; ++i)
      record.fields.push_back({"q/[" + std::to_string(i) + "]", (i + 1) * 0.01});
    check(model.update(record), "X1 group not updated");
  }
  const auto after = model.transforms(1790000000123456789);
  check(before.transforms.size() == after.transforms.size(), "X1 topology changed");
  size_t moving = 0;
  for (size_t i = 0; i < before.transforms.size(); ++i)
    if (before.transforms[i].rotation != after.transforms[i].rotation) ++moving;
  check(moving == 31, "Not all 31 X1 joints moved");
  check(model.update({"state_estimator", 0,
                      {{"quat/[0]", 1.0}, {"quat/[1]", 0.0},
                       {"quat/[2]", 0.0}, {"quat/[3]", 1.0}}}), "X1 root pose update missing");
  const auto oriented = model.transforms(1790000000123456789);
  rotationNear(edge(oriented, "pelvis").rotation, 0, 0, std::sqrt(0.5), std::sqrt(0.5));
  check(edge(oriented, "pelvis").parent_frame_id == model.worldFrame(), "X1 root is not world-relative");
  for (const auto& transform : after.transforms)
    if (transform.child_frame_id != model.rootLink())
      check(edge(oriented, transform.child_frame_id) == transform, "X1 root update changed joint pose");
  std::cout << "PASS: actual X1 model, all 31 joints moved in the correct groups, independent root orientation\n";
}

/// 动态加载 path 指定的 ROS 解析插件，验证 URDF 与 TF 的流式解析和对象解码。
/// 加载、解析或比较失败时抛出异常，无返回值。
void testRosParser(const char* path) {
  void* library = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (!library) throw std::runtime_error(dlerror());
  auto owner = std::shared_ptr<void>(library, [](void* handle) { dlclose(handle); });
  const auto get = reinterpret_cast<const PJ_message_parser_vtable_t* (*)()>(
      dlsym(library, "PJ_get_message_parser_vtable"));
  check(get != nullptr, "ROS parser entry point missing");
  auto parse = [&](std::string_view type, std::string_view topic, std::string_view schema,
                   const std::vector<uint8_t>& bytes) {
    PJ::sdk::testing::ParserWriteRecorder recorder;
    PJ::ServiceRegistryBuilder registry;
    registry.registerService<PJ::sdk::ParserWriteHostService>(recorder.makeHost());
    PJ::MessageParserHandle parser(get(), owner);
    check(parser.valid(), "Cannot create ROS parser");
    auto status = parser.bind(registry.view());
    if (!status) throw std::runtime_error(status.error());
    const auto config = nlohmann::json{{"serialization", "cdr"}, {"schema_encoding", "ros2msg"},
                                      {"topic_name", topic}}.dump();
    status = parser.loadConfig(config);
    if (!status) throw std::runtime_error(status.error());
    status = parser.bindSchema(type, {reinterpret_cast<const uint8_t*>(schema.data()), schema.size()});
    if (!status) throw std::runtime_error(status.error());
    // 实时流会先调用 parse() 再接收对象。先走该入口，覆盖旧版解析器缺少标量解析能力的情况。
    status = parser.parse(1790000000123456789, PJ::Span<const uint8_t>(bytes.data(), bytes.size()));
    if (!status) throw std::runtime_error(status.error());
    auto result = parser.parseObjectFunctional(1790000000123456789, PJ::Span<const uint8_t>(bytes.data(), bytes.size()));
    if (!result) throw std::runtime_error(result.error());
    return std::move(*result);
  };
  DdsRobotModel model(fixtureSettings());
  model.update({"leg_control_data", 0, {{"q/[0]", 0.3}}});
  model.update({"state_estimator", 0,
                {{"quat/[0]", 2.0}, {"quat/[1]", -1.0},
                 {"quat/[2]", 3.0}, {"quat/[3]", 4.0}}});
  const auto expected = model.transforms(1790000000123456789);
  auto tf = parse("tf2_msgs/msg/TFMessage", "/tf", kDdsRobotTfSchema, ddsRobotTransformsCdr(expected));
  const auto* decoded = tf.object.get<PJ::sdk::FrameTransforms>();
  check(decoded && decoded->transforms == expected.transforms, "ROS parser did not decode exact TF values/timestamps");
  auto description = parse("std_msgs/msg/String", "/robot_description", "string data\n",
                           ddsRobotDescriptionCdr(model.description()));
  const auto* robot = description.object.get<PJ::sdk::RobotDescription>();
  check(robot && robot->format == "urdf" && robot->text == model.description(), "RobotDescription promotion failed");
  std::cout << "PASS: ROS streaming parse accepts RobotDescription and TF; exact CDR objects decode\n";
}
}  // namespace

/// 执行基础回归。argv 按顺序可附带 X1 的 URDF 和 XML，以及 ROS 解析插件路径。
/// argc 决定执行哪些附加用例，全部通过返回 0，捕获到测试异常时返回 1。
int main(int argc, char** argv) {
  try {
    testModel();
    testOrientation();
    if (argc >= 3) testX1(argv[1], argv[2]);
    if (argc >= 4) testRosParser(argv[3]);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
