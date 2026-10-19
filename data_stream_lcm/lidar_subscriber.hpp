#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

class LidarSubscriber : public rclcpp::Node
{
public:
    LidarSubscriber()
        : Node("lidar_subscriber111")
    {
        // 设置 QoS
        // Livox 发布端之前显示 RELIABLE + TRANSIENT_LOCAL。
        // 这里使用同样的 QoS，便于先验证通信。
        const auto qos = rclcpp::SensorDataQoS();

        subscription_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/lidar_point_cloud",
                qos,
                std::bind(
                    &LidarSubscriber::pointCloudCallback,
                    this,
                    std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Subscribed to topic: /lidar_point_cloud");
    }

private:
    void pointCloudCallback(
        const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::cout << "pointCloudCallback" << std::endl;
        RCLCPP_INFO(
            this->get_logger(),
            "Received PointCloud2:"
            " width=%u, height=%u, points=%u,"
            " fields=%zu, point_step=%u, row_step=%u,"
            " data_size=%zu bytes, is_dense=%s",
            msg->width,
            msg->height,
            msg->width * msg->height,
            msg->fields.size(),
            msg->point_step,
            msg->row_step,
            msg->data.size(),
            msg->is_dense ? "true" : "false");

        // 打印点云中的字段名称及数据类型
        for (const auto &field : msg->fields)
        {
            RCLCPP_INFO(
                this->get_logger(),
                "  Field: %s, offset=%u, datatype=%u, count=%u",
                field.name.c_str(),
                field.offset,
                field.datatype,
                field.count);
        }

        // 如果后续需要解析 XYZ 点坐标，
        // 可以根据 msg->fields、msg->point_step 和 msg->data 解析。
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        subscription_;
};

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<LidarSubscriber>();

//     rclcpp::spin(node);

//     rclcpp::shutdown();
//     return 0;
// }