# MotionStudio: 运控上位机调试工具

## 项目概述

MotionStudio 是一款针对运控工程化的调试工具

## 主要功能

### 1. LCM 到 ROS2 消息转换 (`lcm_to_ros2`)
- 将 [LCM](https://github.com/lcm-proj/lcm) 消息类型自动转换为 ROS2 消息类型
- 自动生成消息转换节点，实现 LCM 消息到 ROS2 消息的实时转发
- 支持多种机器人平台的消息格式，包括人形机器人、四足机器人等

### 2. 机器人可视化 (`magic_vis_ros2`)
- 基于 RViz2 实现机器人模型的 3D 可视化
- 实时显示机器人关节状态、传感器数据等关键信息
- 支持多种机器人模型的可视化配置

### 3. 数据可视化插件 (`plotjuggler-ros-plugins`)
- 为 PlotJuggler 提供 ROS2 支持
- 方便查看和分析 ROS2 话题数据
- 支持数据录制和回放

### 4. 运控上位机控制器 (`motion_controller`)
- 目录motion_controller 支持lcm日志播放，支持ros2 bag日志录制，支持连接真机在线调试

## 目录结构

```
motionstudio/
├── motion_controller/                # 运控上位机控制器
├── lcm_to_ros2/        # LCM 到 ROS2 消息转换工具
│   ├── autosrc/        # 自动生成的源代码
│   ├── cmake/          # CMake 配置文件
│   ├── exlcm/          # LCM 消息头文件
│   ├── launch/         # ROS2 launch 文件
│   ├── lcm/            # LCM 消息定义文件
│   ├── msg/            # ROS2 消息定义文件
│   ├── repub_configs/  # 消息转换配置文件
│   ├── src/            # 源代码模板
│   ├── CMakeLists.txt
│   ├── README.md
│   ├── package.xml
│   └── 工具脚本文件
├── magic_vis_ros2/     # 机器人可视化工具
│   ├── doc/            # 文档和图片
│   ├── include/        # 头文件
│   ├── launch/         # ROS2 launch 文件
│   ├── meshes/         # 机器人模型网格文件
│   ├── CMakeLists.txt
│   ├── README.md
│   └── package.xml
├── plotjuggler-ros-plugins/ # ROS2 的 PlotJuggler 插件
├── env_install.sh      # 环境安装脚本
├── build_ros2.sh       # 编译构建脚本
├── run_ros.sh          # 运行上位机脚本
├── stop_ros2.sh        # 停止上位机脚本
└── README.md           # 项目说明文件
```

## 环境要求

- Ubuntu 22.04 或更高版本
- ROS2 Jazzy 或兼容版本
- Conda (用于环境管理)
- CMake 3.20 或更高版本
- C++14 兼容编译器
- Python 3.13 或更高版本

## 安装与设置

# 说明
  `lcm_to_ros` 原 repo [文档](./lcm_to_ros/README.md)
  `magic_vis` [文档](./magic_vis/README.md)




## ros2 使用示例
### 完整系统启动

1. 构建
```bash
./build_ros2.sh
```
2. 启动LCM到ROS2的消息转换并可视化：

```bash
./run_ros2.sh
```
