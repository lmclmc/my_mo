#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

// ROS2 序列化与 Bag 写入核心头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

#include "lcm_to_ros2/msg/StateEstimatorLcmt.hpp"
#include "lcm_to_ros2/msg/JointState.hpp"
#include "lcm_to_ros2/msg/CanfdCommand.hpp"
#include "lcm_to_ros2/msg/CanfdState.hpp"
#include "lcm_to_ros2/msg/HeadControlCommandLcmt.hpp"
#include "lcm_to_ros2/msg/HeadControlDataLcmt.hpp"
#include "lcm_to_ros2/msg/LcmResponseLcmt.hpp"
#include "lcm_to_ros2/msg/LegControlCommandLcmt.hpp"
#include "lcm_to_ros2/msg/LegControlDataLcmt.hpp"
#include "lcm_to_ros2/msg/LowlevelCommandStruct.hpp"
#include "lcm_to_ros2/msg/JointCommand.hpp"
#include "lcm_to_ros2/msg/Imu.hpp"
#include "lcm_to_ros2/msg/EstimatorState.hpp"
#include "lcm_to_ros2/msg/RobotPowerIoLcmt.hpp"
#include "lcm_to_ros2/msg/WaistControlCommandLcmt.hpp"
#include "lcm_to_ros2/msg/WaistControlDataLcmt.hpp"
#include "lcm_to_ros2/msg/ArmControlDataLcmt.hpp"
#include "lcm_to_ros2/msg/ArmControlCmdRlLcmt.hpp"
#include "lcm_to_ros2/msg/ArmControlCmdLcmt.hpp"
#include "lcm_to_ros2/msg/LegControlCommandRlLcmt.hpp"
#include "lcm_to_ros2/msg/WaistControlCmdRlLcmt.hpp"
#include "lcm_to_ros2/msg/HalArmCmdLcmt.hpp"
#include "lcm_to_ros2/msg/HalArmStateLcmt.hpp"
#include "lcm_to_ros2/msg/HalBmsLcmt.hpp"
#include "lcm_to_ros2/msg/HalLegCmdLcmt.hpp"
#include "lcm_to_ros2/msg/HalLegStateLcmt.hpp"
#include "lcm_to_ros2/msg/MonitorBodyState.hpp"
#include "lcm_to_ros2/msg/HalWaistCmdLcmt.hpp"
#include "lcm_to_ros2/msg/HalWaistStateLcmt.hpp"
#include "lcm_to_ros2/msg/LcmCommandStruct.hpp"
#include "exlcm/lcm_command_struct.hpp"
#include "exlcm/hal_waist_joint_state_t.hpp"
#include "exlcm/hal_waist_joint_command_t.hpp"
#include "exlcm/monitor_body_state_t.hpp"
#include "exlcm/hal_leg_joint_state_t.hpp"
#include "exlcm/hal_leg_joint_command_t.hpp"
#include "exlcm/hal_bms_t.hpp"
#include "exlcm/hal_arm_joint_state_t.hpp"
#include "exlcm/hal_arm_joint_command_t.hpp"
#include "exlcm/joint_state_t.hpp"
#include "exlcm/waist_control_cmd_rl_lcmt.hpp"
#include "exlcm/arm_control_cmd_rl_lcmt.hpp"
#include "exlcm/leg_control_command_rl_lcmt.hpp"
#include "exlcm/arm_control_MagicBotZ1_data_lcmt.hpp"
#include "exlcm/arm_control_data_lcmt.hpp"
#include "exlcm/arm_control_cmd_lcmt.hpp"
#include "exlcm/waist_control_data_lcmt.hpp"
#include "exlcm/waist_control_command_lcmt.hpp"
#include "exlcm/robot_power_io_lcmt.hpp"
#include "exlcm/estimator_state_t.hpp"
#include "exlcm/imu_t.hpp"
#include "exlcm/joint_command_t.hpp"
#include "exlcm/lowlevel_command_struct.hpp"
#include "exlcm/leg_control_data_lcmt_magicdog.hpp"
#include "exlcm/leg_control_data_lcmt.hpp"
#include "exlcm/leg_control_command_lcmt.hpp"
#include "exlcm/leg_control_data_lcmt_bigdog.hpp"
#include "exlcm/lcm_response_lcmt.hpp"
#include "exlcm/head_control_data_lcmt.hpp"
#include "exlcm/head_control_command_lcmt.hpp"
#include "exlcm/canfd_state_t.hpp"
#include "exlcm/canfd_command_t.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "exlcm/state_estimator_lcmt.hpp"

// 引入本地 LCM 核心头文件及你自研的 LCM 结构体
#include <lcm/lcm-cpp.hpp>
#include <fstream>

static int csv_count = -1;
static int g_fsm = -1;
static double state_estimator_quat_0 = 0;

template <typename lcm_type, typename ros2_type>
static std::shared_ptr<rclcpp::SerializedMessage> convert_function(rclcpp::Serialization<ros2_type>& serializer, void* lcm_data, size_t lcm_size) {
    lcm_type lcm_msg;
    lcm_msg.decode(lcm_data, 0, lcm_size);
    const ros2_type* ros2_msg = reinterpret_cast<const ros2_type*>(&lcm_msg);
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(); 
    serializer.serialize_message(ros2_msg, serialized_msg.get());
    return serialized_msg;
}

template <typename lcm_type, typename ros2_type>
static std::shared_ptr<rclcpp::SerializedMessage> state_estimator_convert_function(rclcpp::Serialization<ros2_type>& serializer, void* lcm_data, size_t lcm_size) {
    lcm_type lcm_msg;
    lcm_msg.decode(lcm_data, 0, lcm_size);
    if (csv_count == 0) {
        state_estimator_quat_0 = lcm_msg.quat[0];
    }
    const ros2_type* ros2_msg = reinterpret_cast<const ros2_type*>(&lcm_msg);
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(); 
    serializer.serialize_message(ros2_msg, serialized_msg.get());
    return serialized_msg;
}


template <typename lcm_type, typename ros2_type>
static std::shared_ptr<rclcpp::SerializedMessage> lowlevel_convert_function(rclcpp::Serialization<ros2_type>& serializer, void* lcm_data, size_t lcm_size) {
    lcm_type lcm_msg;
    lcm_msg.decode(lcm_data, 0, lcm_size);
    const ros2_type* ros2_msg = reinterpret_cast<const ros2_type*>(&lcm_msg);
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(); 
    serializer.serialize_message(ros2_msg, serialized_msg.get());
    return serialized_msg;
}

template <typename lcm_type, typename ros2_type>
static std::shared_ptr<rclcpp::SerializedMessage> lcm_response_convert_function(rclcpp::Serialization<ros2_type>& serializer, void* lcm_data, size_t lcm_size) {
    lcm_type lcm_msg;
    lcm_msg.decode(lcm_data, 0, lcm_size);
    if (g_fsm != -1 && lcm_msg.robot_fsm == g_fsm) {
        csv_count++;
    }
    const ros2_type* ros2_msg = reinterpret_cast<const ros2_type*>(&lcm_msg);
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(); 
    serializer.serialize_message(ros2_msg, serialized_msg.get());
    return serialized_msg;
}

template <typename lcm_type, typename ros2_type>
static std::shared_ptr<rclcpp::SerializedMessage> arm_convert_function(rclcpp::Serialization<ros2_type>& serializer, void* lcm_data, size_t lcm_size) {
    // 1. 提取消息头部的 64 位指纹 (Big-Endian)
    int64_t msg_hash;
    memcpy(&msg_hash, lcm_data, sizeof(int64_t));
    
    // 转换字节序（LCM 传输使用的是大端序）
    msg_hash = be64toh(msg_hash);
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(); 
    if (msg_hash == (int64_t)exlcm::arm_control_data_lcmt::getHash()) {
        exlcm::arm_control_data_lcmt arm_msg;
        if (arm_msg.decode(lcm_data, 0, lcm_size) > 0) {
            const ros2_type* ros2_msg = reinterpret_cast<const ros2_type*>(&arm_msg);
            
            serializer.serialize_message(ros2_msg, serialized_msg.get());
        }
    } else if (msg_hash == (int64_t)exlcm::arm_control_MagicBotZ1_data_lcmt::getHash()) {
        exlcm::arm_control_MagicBotZ1_data_lcmt arm_msg;
        if (arm_msg.decode(lcm_data, 0, lcm_size) > 0) {
            const ros2_type* ros2_msg = reinterpret_cast<const ros2_type*>(&arm_msg);
            serializer.serialize_message(ros2_msg, serialized_msg.get());
        }
    }
    return serialized_msg;
}

template <typename lcm_type, typename ros2_type>
static std::shared_ptr<rclcpp::SerializedMessage> leg_convert_function(rclcpp::Serialization<ros2_type>& serializer, void* lcm_data, size_t lcm_size) {
    // 1. 提取消息头部的 64 位指纹 (Big-Endian)
    int64_t msg_hash;
    memcpy(&msg_hash, lcm_data, sizeof(int64_t));
    
    // 转换字节序（LCM 传输使用的是大端序）
    msg_hash = be64toh(msg_hash);
    auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(); 
    if (msg_hash == (int64_t)exlcm::leg_control_data_lcmt::getHash()) {
        exlcm::leg_control_data_lcmt leg_msg;
        if (leg_msg.decode(lcm_data, 0, lcm_size) > 0) {
            const ros2_type* ros2_msg = reinterpret_cast<const ros2_type*>(&leg_msg);
            
            serializer.serialize_message(ros2_msg, serialized_msg.get());
        }
    } else if (msg_hash == (int64_t)exlcm::leg_control_data_lcmt_bigdog::getHash()) {
        exlcm::leg_control_data_lcmt_bigdog bigdog_msg;
        if (bigdog_msg.decode(lcm_data, 0, lcm_size) > 0) {
            ros2_type ros2_msg;
            std::copy(std::begin(bigdog_msg.q), std::end(bigdog_msg.q), ros2_msg.q.begin());
            std::copy(std::begin(bigdog_msg.qd), std::end(bigdog_msg.qd), ros2_msg.qd.begin());
            std::copy(std::begin(bigdog_msg.pos), std::end(bigdog_msg.pos), ros2_msg.pos.begin());
            std::copy(std::begin(bigdog_msg.vel), std::end(bigdog_msg.vel), ros2_msg.vel.begin());
            serializer.serialize_message(&ros2_msg, serialized_msg.get());
        }
    } else if (msg_hash == (int64_t)exlcm::leg_control_data_lcmt_magicdog::getHash()) {
        exlcm::leg_control_data_lcmt_magicdog magicdog_msg;
        if (magicdog_msg.decode(lcm_data, 0, lcm_size) > 0) {
            const ros2_type* ros2_msg = reinterpret_cast<const ros2_type*>(&magicdog_msg);
            serializer.serialize_message(ros2_msg, serialized_msg.get());
        }
    }

    return serialized_msg;
}

// =========================================================================
// 🛠️ 【核心配置区：话题/通道大表 (X-Macro)】
// =========================================================================
// 格式：DEF_TOPIC(LCM通道名, LCM类名, ROS2话题名, ROS2消息完整类型)
// 只要在这里加一行，底层的所有逻辑（注册、解包、路由、序列化）全部自动生成！
#define TOPIC_LIST(DEF_TOPIC) \
    DEF_TOPIC("/mjr/arm/joint_cmd",         hal::arm::joint_command_t,         "/mjr/arm/joint_cmd",               lcm_to_ros2::msg::HalArmCmdLcmt, convert_function)\
    DEF_TOPIC("/mjr/arm/joint_state",       hal::arm::joint_state_t,           "/mjr/arm/joint_state",             lcm_to_ros2::msg::HalArmStateLcmt, convert_function)\
    DEF_TOPIC("/mjr/bms",                   hal::bms::bms_t,                   "/mjr/bms",                         lcm_to_ros2::msg::HalBmsLcmt, convert_function)\
    DEF_TOPIC("/mjr/leg/joint_cmd",         hal::leg::joint_command_t,         "/mjr/leg/joint_cmd",               lcm_to_ros2::msg::HalLegCmdLcmt, convert_function)\
    DEF_TOPIC("/mjr/leg/joint_state",       hal::leg::joint_state_t,           "/mjr/leg/joint_state",             lcm_to_ros2::msg::HalLegStateLcmt, convert_function)\
    DEF_TOPIC("/mjr/waist/joint_cmd",       hal::waist::joint_command_t,       "/mjr/waist/joint_cmd",             lcm_to_ros2::msg::HalWaistCmdLcmt, convert_function)\
    DEF_TOPIC("/mjr/waist/joint_state",     hal::waist::joint_state_t,         "/mjr/waist/joint_state",           lcm_to_ros2::msg::HalWaistStateLcmt, convert_function)\
    DEF_TOPIC("/monitor/body/state",        monitor_body_state_t,              "/monitor/body/state",              lcm_to_ros2::msg::MonitorBodyState, convert_function)\
\
    DEF_TOPIC("lcm_com_cmd",                exlcm::lcm_command_struct,         "/robot_lcm/lcm_com_cmd",           lcm_to_ros2::msg::LcmCommandStruct, convert_function)\
    DEF_TOPIC("state_estimator",            exlcm::state_estimator_lcmt,       "/robot_lcm/state_estimator",       lcm_to_ros2::msg::StateEstimatorLcmt, state_estimator_convert_function)\
    DEF_TOPIC("arm_command",                exlcm::arm_control_cmd_rl_lcmt,    "/robot_lcm/arm_command",           lcm_to_ros2::msg::ArmControlCmdRlLcmt, convert_function)\
    DEF_TOPIC("arm_control_command",        exlcm::arm_control_cmd_lcmt,       "/robot_lcm/arm_control_command",   lcm_to_ros2::msg::ArmControlCmdLcmt, convert_function)\
    DEF_TOPIC("arm_control_data",           exlcm::arm_control_data_lcmt,      "/robot_lcm/arm_control_data",      lcm_to_ros2::msg::ArmControlDataLcmt, arm_convert_function)\
\
    DEF_TOPIC("/hal/canfd/joint_cmd",       hal::canfd::canfd_command_t,       "/robot_lcm/hal/canfd/joint_cmd",   lcm_to_ros2::msg::CanfdCommand, convert_function)\
    DEF_TOPIC("/hal/canfd/joint_state",     hal::canfd::canfd_state_t,         "/robot_lcm/hal/canfd/joint_state", lcm_to_ros2::msg::CanfdState, convert_function)\
\
    DEF_TOPIC("head_control_command",       exlcm::head_control_command_lcmt,  "/robot_lcm/head_control_command",  lcm_to_ros2::msg::HeadControlCommandLcmt, convert_function)\
    DEF_TOPIC("head_control_data",          exlcm::head_control_data_lcmt,     "/robot_lcm/head_control_data",     lcm_to_ros2::msg::HeadControlDataLcmt, convert_function)\
\
    DEF_TOPIC("lcm_response",               exlcm::lcm_response_lcmt,          "/robot_lcm/lcm_response",          lcm_to_ros2::msg::LcmResponseLcmt, lcm_response_convert_function)\
\
    DEF_TOPIC("leg_command",                exlcm::leg_control_command_rl_lcmt,"/robot_lcm/leg_command",           lcm_to_ros2::msg::LegControlCommandRlLcmt, convert_function)\
    DEF_TOPIC("leg_control_command",        exlcm::leg_control_command_lcmt,   "/robot_lcm/leg_control_command",   lcm_to_ros2::msg::LegControlCommandLcmt, convert_function)\
    DEF_TOPIC("leg_control_data",           exlcm::leg_control_data_lcmt,      "/robot_lcm/leg_control_data",      lcm_to_ros2::msg::LegControlDataLcmt, leg_convert_function)\
    DEF_TOPIC("lowlevel_command_rl",        exlcm::lowlevel_command_struct,    "/robot_lcm/lowlevel_command_rl",   lcm_to_ros2::msg::LowlevelCommandStruct, lowlevel_convert_function)\
\
    DEF_TOPIC("/magic/arm/joint_command",   exlcm::joint_command_t,            "/magic/arm/joint_command",         lcm_to_ros2::msg::JointCommand, convert_function)\
    DEF_TOPIC("/magic/arm/joint_state",     exlcm::joint_state_t,              "/magic/arm/joint_state",           lcm_to_ros2::msg::JointState, convert_function)\
    DEF_TOPIC("/magic/body/imu",            exlcm::imu_t,                      "/magic/body/imu",                  lcm_to_ros2::msg::Imu, convert_function)\
    DEF_TOPIC("/magic/head/joint_command",  exlcm::joint_command_t,            "/magic/head/joint_command",        lcm_to_ros2::msg::JointCommand, convert_function)\
    DEF_TOPIC("/magic/head/joint_state",    exlcm::joint_state_t,              "/magic/head/joint_state",          lcm_to_ros2::msg::JointState, convert_function)\
    DEF_TOPIC("/magic/leg/joint_command",   exlcm::joint_command_t,            "/magic/leg/joint_command",         lcm_to_ros2::msg::JointCommand, convert_function)\
    DEF_TOPIC("/magic/leg/joint_state",     exlcm::joint_state_t,              "/magic/leg/joint_state",           lcm_to_ros2::msg::JointState, convert_function)\
    DEF_TOPIC("/magic/waist/joint_command", exlcm::joint_command_t,            "/magic/waist/joint_command",       lcm_to_ros2::msg::JointCommand, convert_function)\
    DEF_TOPIC("/magic/waist/joint_state",   exlcm::joint_state_t,              "/magic/waist/joint_state",         lcm_to_ros2::msg::JointState, convert_function)\
\
    DEF_TOPIC("robot_power_io_lcmt",        exlcm::robot_power_io_lcmt,        "/robot_lcm/robot_power_io",        lcm_to_ros2::msg::RobotPowerIoLcmt, convert_function)\
\
    DEF_TOPIC("waist_command",              exlcm::waist_control_cmd_rl_lcmt,          "/robot_lcm/waist_command",         lcm_to_ros2::msg::WaistControlCmdRlLcmt, convert_function)\
    DEF_TOPIC("waist_control_command",      exlcm::waist_control_command_lcmt, "/robot_lcm/waist_control_command", lcm_to_ros2::msg::WaistControlCommandLcmt, convert_function)\
    DEF_TOPIC("waist_control_data",         exlcm::waist_control_data_lcmt,    "/robot_lcm/waist_control_data",    lcm_to_ros2::msg::WaistControlDataLcmt, convert_function)\
    // 示例：未来扩展只需在此追加，如：
    // DEF_TOPIC("MOTOR_TEMP",   motor_temp_t,   "/robot/joint_temp",   custom_interfaces::msg::HwTemp)

// =========================================================================
// 🚀 【快转引擎主程序】
// =========================================================================

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "使用方法: ./lcm_to_rosbag [输入LCM日志.log] [输出Bag目录路径]" << std::endl;
        return -1;
    }

    std::string lcm_log_path = argv[1];
    std::string bag_output_path = argv[2];

    // 1. 初始化并配置高效的基于文件的 rosbag2_cpp::Writer
    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_output_path;
    storage_options.storage_id = "mcap"; // 显式指定最高效的 mcap 格式而非默认的 sqlite3

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    try {
        writer->open(storage_options, converter_options);
    } catch (const std::exception& e) {
        std::cerr << "初始化 ROS2 Bag 失败: " << e.what() << std::endl;
        return -1;
    }

    // 2. 利用宏展开：一键向 Bag 注册清单里所有话题的元数据（零手写代码开销）
    #define REGISTER_METADATA(lcm_ch, lcm_type, ros2_top, ros2_type, convert_cb) { \
        rosbag2_storage::TopicMetadata meta; \
        meta.name = ros2_top; \
        meta.type = rosidl_generator_traits::name<ros2_type>(); \
        meta.serialization_format = "cdr"; \
        writer->create_topic(meta); \
    }
    TOPIC_LIST(REGISTER_METADATA)
    #undef REGISTER_METADATA

    // 3. 打开二进制 LCM 日志文件
    lcm::LogFile lcm_log(lcm_log_path, "r");
    if (!lcm_log.good()) {
        std::cerr << "错误：无法打开或读取 LCM 日志文件: " << lcm_log_path << std::endl;
        return -1;
    }

    std::cout << "开始高速离线转换（纯内存 IO 流）..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    uint64_t total_count = 0;

    std::string quat_csv_path = "";
    std::string pos_csv_path = "";
    std::string vel_csv_path = "";
    if (argc > 3) {
        quat_csv_path = argv[3];
        pos_csv_path = argv[4];
        vel_csv_path = argv[5];
        g_fsm = std::stoi(argv[6]);
    } 
    
    rosbag2_storage::TopicMetadata meta;
    meta.name = "csv_quat";
    meta.type = rosidl_generator_traits::name<geometry_msgs::msg::Quaternion>();
    meta.serialization_format = "cdr";
    writer->create_topic(meta);

    meta.name = "csv_pos";
    meta.type = rosidl_generator_traits::name<sensor_msgs::msg::JointState>();
    meta.serialization_format = "cdr";
    writer->create_topic(meta);

    meta.name = "csv_vel";
    meta.type = rosidl_generator_traits::name<sensor_msgs::msg::JointState>();
    meta.serialization_format = "cdr";
    writer->create_topic(meta);

    // 读取CSV文件的函数
    auto read_csv = [](const std::string& file_path) -> std::vector<std::vector<double>> {
        if (file_path.empty()) {
            return std::vector<std::vector<double>>();
        }

        std::vector<std::vector<double>> data;
        std::ifstream file(file_path);
        
        if (!file.is_open()) {
            throw std::runtime_error("无法打开文件: " + file_path);
        }
 
        std::string line;
        while (std::getline(file, line)) {
            std::vector<double> row;
            std::stringstream ss(line);
            std::string token;
 
            while (std::getline(ss, token, ',')) {
                // 去除可能的空白字符
                token.erase(token.find_last_not_of(" \t\r\n") + 1);
                token.erase(0, token.find_first_not_of(" \t\r\n"));
                
                if (!token.empty()) {
                    row.push_back(std::stod(token));
                }
            }
 
            if (!row.empty()) {
                data.push_back(row);
            }
        }
 
        file.close();
        return data;
    };

    std::vector<std::vector<double>> quat_data = read_csv(quat_csv_path);
    std::vector<std::vector<double>> pos_data = read_csv(pos_csv_path);
    std::vector<std::vector<double>> vel_data = read_csv(vel_csv_path);
    float init_quaternion_w = 0.0f;
    float init_quaternion_x = 0.0f;
    float init_quaternion_y = 0.0f;
    float init_quaternion_z = 0.0f;
    if (!quat_data.empty() && !pos_data.empty() && !vel_data.empty()) {
        init_quaternion_w = quat_data[0][0];
        init_quaternion_x = quat_data[0][1];
        init_quaternion_y = quat_data[0][2];
        init_quaternion_z = quat_data[0][3];
    }

    // 4. 核心快转主循环
    while (true) {
        const lcm::LogEvent* event = lcm_log.readNextEvent();
        if (!event) break; // 读完整个 log 文件，平稳退出

        std::string channel = event->channel;
        // 时间戳精准转换：LCM 标准微秒 (us) -> ROS2 标准纳秒 (ns)
        rcutils_time_point_value_t timestamp_ns = event->timestamp * 1000;
        int local_csv_count = csv_count / 10;
        static int last_csv_count = -1;
        if (local_csv_count >= 0 && local_csv_count != last_csv_count && local_csv_count < quat_data.size()) {
            geometry_msgs::msg::Quaternion quaternion;
            quaternion.x = quat_data[local_csv_count][1] - init_quaternion_x;
            quaternion.y = quat_data[local_csv_count][2] - init_quaternion_y;
            quaternion.z = quat_data[local_csv_count][3] - init_quaternion_z;
            quaternion.w = quat_data[local_csv_count][0] - init_quaternion_w + 1;
            rclcpp::Serialization<geometry_msgs::msg::Quaternion> serializer;
            auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(); 
            serializer.serialize_message(&quaternion, serialized_msg.get());
            writer->write(serialized_msg, "csv_quat", rosidl_generator_traits::name<geometry_msgs::msg::Quaternion>(), rclcpp::Time(timestamp_ns));

            rclcpp::Serialization<sensor_msgs::msg::JointState> pos_serializer;
            sensor_msgs::msg::JointState joint_state;

            joint_state.name.resize(pos_data[local_csv_count].size());
            joint_state.position.resize(pos_data[local_csv_count].size());
            for (size_t i = 0; i < pos_data[local_csv_count].size(); i++) {
                joint_state.name[i] = std::to_string(i);
                joint_state.position[i] = pos_data[local_csv_count][i];
            }
            auto serialized_msg1 = std::make_shared<rclcpp::SerializedMessage>(); 
            pos_serializer.serialize_message(&joint_state, serialized_msg1.get());
            writer->write(serialized_msg1, "csv_pos", rosidl_generator_traits::name<sensor_msgs::msg::JointState>(), rclcpp::Time(timestamp_ns));
            joint_state.position.clear();

            joint_state.velocity.resize(vel_data[local_csv_count].size());
            for (size_t i = 0; i < vel_data[local_csv_count].size(); i++) {
                joint_state.velocity[i] = vel_data[local_csv_count][i];
            }
            auto serialized_msg2 = std::make_shared<rclcpp::SerializedMessage>(); 
            pos_serializer.serialize_message(&joint_state, serialized_msg2.get());
            writer->write(serialized_msg2, "csv_vel", rosidl_generator_traits::name<sensor_msgs::msg::JointState>(), rclcpp::Time(timestamp_ns));
            joint_state.velocity.clear();
            joint_state.name.clear();

            last_csv_count = local_csv_count;
        }
        
        // 利用宏展开：自动生成纯强类型的高效路由分支树
        #define PROCESS_EVENT_BRANCH(lcm_ch, lcm_type, ros2_top, ros2_type, convert_cb) \
        if (channel == lcm_ch) { \
            /* 4.1 二进制解包，无网络拷贝 */ \
            rclcpp::Serialization<ros2_type> serializer; \
            auto serialized_msg = convert_cb<lcm_type, ros2_type>(serializer, event->data, event->datalen);\
            /* 4.4 内存直接转储到文件磁盘 */ \
            writer->write(serialized_msg, ros2_top, rosidl_generator_traits::name<ros2_type>(), rclcpp::Time(event->timestamp * 1000)); \
            total_count++; \
            continue; \
        }

        TOPIC_LIST(PROCESS_EVENT_BRANCH)
        #undef PROCESS_EVENT_BRANCH
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << "\n🎉 转换完成！" << std::endl;
    std::cout << "共处理高频运控数据: " << total_count << " 条" << std::endl;
    std::cout << "总耗时: " << duration << " ms (平均单条处理耗时极其微弱)" << std::endl;

    return 0;
}