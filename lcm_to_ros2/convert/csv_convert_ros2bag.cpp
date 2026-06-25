#include <iostream>

// ROS2 序列化与 Bag 写入核心头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <fstream>

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cout << "使用方法: ./csv_convert_ros2bag [输入quat.csv文件路径] [输入pos.csv文件路径] [输入vel.csv文件路径] [输出Bag目录路径]" << std::endl;
        return -1;
    }

    std::string quat_csv_path = argv[1];
    std::string pos_csv_path = argv[2];
    std::string vel_csv_path = argv[3];
    std::string bag_output_path = argv[4];

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
 
    try {
        // 读取三个CSV文件
        std::vector<std::vector<double>> quat_data = read_csv(quat_csv_path);
        std::vector<std::vector<double>> pos_data = read_csv(pos_csv_path);
        std::vector<std::vector<double>> vel_data = read_csv(vel_csv_path);
        
        rclcpp::Serialization<geometry_msgs::msg::Quaternion> serializer;

        geometry_msgs::msg::Quaternion quaternion;
        int count = 0;
        float init_quaternion_w = quat_data[0][0];
        float init_quaternion_z = quat_data[0][3];
        for (auto& v : quat_data) {
            quaternion.x = v[1];
            quaternion.y = v[2];
            quaternion.z = v[3];// - init_quaternion_z;
            quaternion.w = v[0] - init_quaternion_w + 1;
            auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(); 
            serializer.serialize_message(&quaternion, serialized_msg.get());
            writer->write(serialized_msg, "csv_quat", rosidl_generator_traits::name<geometry_msgs::msg::Quaternion>(), rclcpp::Time(count++));
        }

        rclcpp::Serialization<sensor_msgs::msg::JointState> pos_serializer;
        sensor_msgs::msg::JointState joint_state;
        count = 0;

        for (auto& v : pos_data) {
            joint_state.name.resize(v.size());
            joint_state.position.resize(v.size());
            for (int i = 0; i < v.size(); i++) {
                joint_state.name[i] = std::to_string(i);
                joint_state.position[i] = v[i];
            }
            auto serialized_msg = std::make_shared<rclcpp::SerializedMessage>(); 
            pos_serializer.serialize_message(&joint_state, serialized_msg.get());
            writer->write(serialized_msg, "csv_pos", rosidl_generator_traits::name<sensor_msgs::msg::JointState>(), rclcpp::Time(count++));
            joint_state.position.clear();
            joint_state.name.clear();
        }

        // 示例：打印读取到的数据
        std::cout << "四元数数据行数: " << quat_data.size() << std::endl;
        if (!quat_data.empty()) {
            std::cout << "第一行数据: ";
            for (double val : quat_data[0]) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
 
        std::cout << "位置数据行数: " << pos_data.size() << std::endl;
        std::cout << "速度数据行数: " << vel_data.size() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }


    std::cout << "开始高速离线转换（纯内存 IO 流）..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    uint64_t total_count = 0;

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    std::cout << "\n🎉 转换完成！" << std::endl;
    std::cout << "共处理高频运控数据: " << total_count << " 条" << std::endl;
    std::cout << "总耗时: " << duration << " ms (平均单条处理耗时极其微弱)" << std::endl;

    return 0;
}