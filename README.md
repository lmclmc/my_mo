# MotionStudio: 运控上位机调试工具

## 项目概述

MotionStudio 是一款针对运控工程化的调试工具
https://dreametech.feishu.cn/wiki/DiyKw9y6mikuEykygWyc0AZvnSO

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
├── lcm_turn_server/    # LCM 转发服务器
├── log_playback_ros2/  # 日志回放工具
├── magic_vis_ros2/     # 机器人可视化工具
├── scripts/            # 用于上位机模式切换的脚本目录
├── plotjuggler-ros-plugins/ # ROS2 的 PlotJuggler 插件
├── env_install.sh      # 环境安装脚本
├── env_init.sh         # 环境初始化脚本
├── build_motion_studio.sh # 编译构建脚本
├── run_motion_studio.sh # 运行上位机脚本
├── stop_motion_studio.sh # 停止上位机脚本
└── README.md           # 项目说明文件
```

## 目录详细说明
 
### 1. motion_controller
 
运控上位机控制器，提供图形界面和核心功能：
- **lcm_tcp_client.py**：LCM TCP 客户端实现，包含断线重连功能，负责与机器人控制系统建立通信
- **lcm_player_thread.py**：LCM 日志播放线程，用于回放录制的机器人状态日志
- **motion_window.py**：运控上位机主窗口，提供用户交互界面
- **lcm/**：LCM Python 库，用于处理 LCM 消息
 
### 2. lcm_to_ros2
 
LCM 到 ROS2 消息转换工具，实现不同消息系统间的桥接：
- **autosrc/**：自动生成的源代码，包含各种消息类型的转发器，负责将 LCM 消息转换为 ROS2 消息
- **lcm/**：LCM 消息定义文件（.lcm格式），定义了各种机器人控制和状态消息的结构
- **msg/**：ROS2 消息定义文件（.msg格式），与 LCM 消息对应
- **repub_configs/**：消息转换配置文件，定义了哪些 LCM 消息需要转发到 ROS2，以及如何映射
- **launch/**：ROS2 launch 文件，用于启动消息转发节点
 
### 3. lcm_turn_server
 
LCM 转发服务器，用于在不同网络间转发 LCM 消息：
- **lcm_turn_server.cpp**：服务器核心代码，实现 LCM 消息的接收和转发
- **build_lcm_turn_server.sh**：构建脚本，用于编译服务器
 
### 4. log_playback_ros2
 
日志回放工具，用于回放和分析机器人状态日志：
- **lcm_player.py**：LCM 日志播放器，支持读取和回放 .lcmlog 文件
- **bigdog.lcmlog**：四足机器人示例日志
- **humanoid.lcmlog**：人形机器人示例日志
 
### 5. magic_vis_ros2
 
机器人可视化工具，基于 RViz2 实现机器人模型的 3D 可视化：
- **launch/**：ROS2 launch 文件，用于启动不同机器人模型的可视化
- **meshes/**：机器人模型网格文件，包含各种机器人的 3D 模型
- **include/**：头文件，包含机器人模型定义和可视化相关代码
- **src/**：源代码，包含机器人模型定义和可视化相关代码

### 6. plotjuggler-ros-plugins
 
ROS2 的 PlotJuggler 插件，用于数据可视化和分析：
- 为 PlotJuggler 提供 ROS2 支持，方便查看和分析 ROS2 话题数据
- 支持数据录制和回放，帮助开发者分析机器人性能和行为

### 7. plotjuggler-ros-plugins
 
- **motion_studio_mode_switch.sh**: 用于切换运控上位机模式的脚本

## 环境要求

- Ubuntu 22.04 或更高版本
- ROS2 Jazzy 或兼容版本
- Conda (用于环境管理)
- CMake 3.20 或更高版本
- C++14 兼容编译器
- Python 3.13 或更高版本

## 安装与设置

### 1. 环境安装

```bash
# 运行环境安装脚本
./env_install.sh 1
```
该脚本会：
- 创建名为 `ros2` 的 Conda 环境
- 安装所需的 Python 依赖包

### 2. 环境初始化

```bash
# 运行环境初始化脚本
source env_init.sh
```

### 3. 编译构建

```bash
# 运行编译构建脚本
./build_motion_studio.sh
```

该脚本会：
- 设置 ROS2 环境变量
- 创建工作空间并链接相关包
- 编译 `lcm_to_ros2` 包
- 编译 `magic_vis_ros2` 包（Debug 模式）
- 编译 `plotjuggler-ros-plugins` 包
- 安装 PlotJuggler 插件到系统目录


## 使用方法

### 1. 运行上位机

```bash
# 运行上位机脚本
./run_motion_studio.sh
```

该脚本会：
- 启动 LCM 到 ROS2 的消息转换
- 启动机器人可视化
- 启动数据可视化工具

### 2. 停止上位机

```bash
# 停止上位机脚本
./stop_motion_studio.sh
```

该脚本会：
- 停止所有运行的 ROS2 节点
- 清理相关进程