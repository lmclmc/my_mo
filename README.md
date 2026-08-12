# MotionStudio: 运控上位机调试工具

```
                     _ooOoo_                     
                    o8888888o                    
                    88" . "88                    
                    (| -_- |)                    
                    O\  =  /O                    
                 ____/`---'\____                 
               .'  \|     |//  `.               
              /  \|||  :  |||//  \              
             /  _||||| -:- |||||-  \             
             |   | \\  -  /// |   |             
             | \_|  ''\---/''  |   |             
             \  .-\__  `-`  ___/-. /             
           ___`. .`  /--.--\  `. . __           
        ."" '<  `.____\_<|>_/___. `  >'"".        
       | | :  `- \`.;`\ _ /`;.`/ - ` : | |       
       \  \ `-.   \_ __\ /__ _/   .-` /  /       
  ======`-.____`-.___\_____/___.-`____.-'======  
                     `=---='                     
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
           佛祖保佑         永无BUG

 ███╗   ███╗ ██████╗ ████████╗ ██╗ ██████╗ ███╗   ██╗
 ████╗ ████║██╔═══██╗╚══██╔══╝ ██║██╔═══██╗████╗  ██║
 ██╔████╔██║██║   ██║   ██║    ██║██║   ██║██╔██╗ ██║
 ██║╚██╔╝██║██║   ██║   ██║    ██║██║   ██║██║╚██╗██║
 ██║ ╚═╝ ██║╚██████╔╝   ██║    ██║╚██████╔╝██║ ╚████║
 ╚═╝     ╚═╝ ╚═════╝    ╚═╝    ╚═╝ ╚═════╝ ╚═╝  ╚═══╝
                     S T U D I O
    MotionStudio · Multi-Function debug toolchain
```

[MotionStudio 项目文档](https://dreametech.feishu.cn/wiki/DiyKw9y6mikuEykygWyc0AZvnSO)

## 项目概述

MotionStudio 是一款针对机器人运控工程化的专业调试工具，提供完整的机器人调试工作流解决方案。

**作者**: 罗敏成

**仓库地址**: `git@gitlab.magiclab.top:dreamebiped/controlsoftware/motionstudio.git`

---

## 主要功能

### 1. 运控上位机控制器 (`motion_controller`)
- 🎮 **图形化操作界面**：基于 Qt/PySide6 的现代化 GUI
- 📊 **实时日志显示**：支持 ANSI 颜色代码的日志输出
- 🔧 **版本信息查询**：一键获取项目版本、作者、仓库信息

### 2. LCM 到 ROS2 消息转换 (`lcm_to_ros2`)
- 🔄 **自动消息转换**：将 LCM 消息类型自动转换为 ROS2 消息类型
- 📦 **多种机器人支持**：人形机器人、四足机器人（MagicDog）、机械臂（MagicBotZ1）等
- ⚡ **实时消息转发**：自动生成消息转换节点

### 3. 机器人可视化 (`magic_vis_ros2`)
- 🕶️ **3D 可视化**：基于 RViz2 的机器人模型展示
- 🔄 **实时状态显示**：关节状态、传感器数据可视化
- 🤖 **多模型支持**：MagicHumanoid、MagicDog、A2601-M-MINI-BOT 等

### 4. 数据可视化插件 (`plotjuggler-ros-plugins`)
- 📈 **PlotJuggler 集成**：为 PlotJuggler 提供 ROS2 支持
- 🎯 **数据录制回放**：支持数据录制和离线分析

### 5. LCM 日志回放 (`log_playback_ros2`)
- 🎬 **LCM 日志播放**：支持读取和回放 `.lcmlog` 文件
- ⏯️ **播放控制**：支持播放、暂停、倍速调节
- 📊 **进度显示**：实时显示播放进度

### 6. LCM 转发服务器 (`lcm_turn_server`)
- 🌐 **远程通信**：支持跨网络的 LCM 消息转发
- 🔗 **在线调试**：连接真机进行实时调试

---

## 目录结构

```
motionstudio/
├── motion_controller/          # 运控上位机控制器
│   ├── lcm/                   # LCM Python 库
│   ├── lcm_player_thread.py   # LCM 日志播放线程
│   ├── lcm_tcp_client.py      # LCM TCP 客户端
│   └── motion_window.py       # 主窗口界面
├── lcm_to_ros2/               # LCM 到 ROS2 消息转换工具
│   ├── autosrc/               # 自动生成的转发器源代码
│   ├── cmake/                 # CMake 配置
│   ├── convert/               # 日志转换工具
│   ├── exlcm/                 # LCM 消息头文件
│   ├── launch/                # ROS2 Launch 文件
│   ├── lcm/                   # LCM 消息定义
│   ├── msg/                   # ROS2 消息定义
│   └── repub_configs/         # 转发配置文件
├── lcm_turn_server/           # LCM 转发服务器
│   ├── lcm_turn_server.cpp    # 服务器核心代码
│   └── build_lcm_turn_server.sh
├── log_playback_ros2/         # 日志回放工具
│   ├── lcm/                   # LCM Python 库
│   ├── lcm_player.py          # LCM 日志播放器
│   └── *.lcmlog               # 示例日志文件
├── magic_vis_ros2/            # 机器人可视化工具
│   ├── config/                # 机器人配置文件
│   ├── launch/                # RViz2 Launch 文件
│   └── meshes/                # 机器人模型网格
├── plotjuggler-ros-plugins/   # PlotJuggler ROS2 插件
├── log/                       # 日志目录
├── version_info.sh            # 版本信息输出脚本
├── build_motion_studio.sh     # 编译构建脚本
├── run_motion_studio.sh       # 运行上位机脚本
├── stop_motion_studio.sh      # 停止上位机脚本
├── upgrade_and_build.sh       # 升级并构建脚本
└── README.md                  # 项目说明文件
```

---

## 环境要求

| 依赖项 | 版本要求 |
|--------|----------|
| 操作系统 | Ubuntu 22.04 或更高版本 |
| ROS2 | Jazzy 或兼容版本 |
| Python | 3.13 或更高版本 |
| Conda | 用于环境管理 |
| CMake | 3.20 或更高版本 |
| C++ 编译器 | C++14 兼容 |
| Qt/PySide6 | 6.x 版本 |

## 安装与设置

### 1. 编译构建

```bash
# 运行编译构建脚本
./build_motion_studio.sh <password>
```

该脚本会：
- 设置 ROS2 环境变量
- 创建工作空间并链接相关包
- 编译 `lcm_to_ros2` 包
- 编译 `magic_vis_ros2` 包（Debug 模式）
- 编译 `plotjuggler-ros-plugins` 包
- 安装 PlotJuggler 插件到系统目录

---

## 使用方法

### 1. 启动上位机

```bash
# 运行主程序
./run_motion_studio.sh
```

### 2. 主界面功能

| 功能模块 | 说明 |
|----------|------|
| **LCM 日志操作** | 播放 LCM 日志文件，支持播放/暂停控制、倍速调节、进度拖动 |
| **在线监听** | 连接运控板进行实时调试，支持远程启动 `lcm_turn_server` |
| **ROS2 Bag 操作** | CSV 融合 LCM 快速转换、ROS2 Bag 录制 |
| **版本信息** | 获取项目版本号、作者信息、仓库地址 |

### 3. 获取版本信息

点击主界面的 **"获取版本信息"** 按钮，即可在右侧日志窗口查看：
- 当前版本号
- 历史版本列表
- 作者信息
- 仓库地址
- 当前分支和提交信息

### 4. 停止上位机

```bash
./stop_motion_studio.sh
```

### 5. 更新工具链

```bash
./upgrade_and_build.sh
```

---

## 功能特性

### LCM 日志播放
- ✅ 支持播放/暂停/停止控制
- ✅ 支持 0.25x ~ 3x 倍速调节
- ✅ 支持进度条手动拖动
- ✅ 支持 LCM 转 ROS2 Bag

### 在线调试
- ✅ 远程传输 `lcm_turn_server`
- ✅ 自动启动远程服务
- ✅ 实时心跳监测
- ✅ 断线重连机制

### 数据转换
- ✅ LCM 日志转 ROS2 Bag
- ✅ CSV (quat/pos/vel) 融合转换
- ✅ 状态机编号支持

### 日志显示
- ✅ ANSI 颜色代码支持
- ✅ 实时滚动显示
- ✅ 时间戳标记

---


## 技术支持

如有问题或建议，请联系作者或查看项目文档：
- 📖 项目文档：https://dreametech.feishu.cn/wiki/DiyKw9y6mikuEykygWyc0AZvnSO
- 👨‍💻 作者：罗敏成

---

**Copyright © 2024 MagicLab. All rights reserved.**