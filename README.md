# Robot Tools

## 介绍
机器人开发辅助工具，以 ROS package 形式存在，当前仅支持 ROS1，含如下功能

* `lcm_to_ros` 将 [LCM](https://github.com/lcm-proj/lcm) 消息转化为 ros msg 发布

* `magic_vis` 基于 [RViz](https://wiki.ros.org/rviz) 可视化机器人及关键数据

## 安装与运行
* 创建工作空间并将相关package软链接到workspace

  ```bash
  mkdir -p catkin_ws/src
  ln -s ~/robot_tools/lcm_to_ros ~/robot_tools/catkin_ws/src/lcm_to_ros
  ln -s ~/robot_tools/magic_vis ~/robot_tools/catkin_ws/src/magic_vis
  ```

* 创建 `lcm_to_ros` 消息类型，自动生成转换 ros node 

  * 将需要转换成 ros msg 的 LCM 消息类型（ `*.lcm` 文件，注意加入 namespace `exlcm`）放在文件夹 `./lcm_to_ros/lcm` 下，执行

    ```bash
    cd ~/robot_tools/lcm_to_ros
    ./rosmsg-gen.sh lcm/*.lcm
    ```
    根据 lcm 消息类型自动生成的 ros msg 放置在 `./lcm_to_ros/msg/` 目录下，**手动确认 `*.msg` 是否符合 ros msg 格式**

  * 生成 ros node cpp 及相关 launch 文件
    将所需要的 lcm to ros 转换配置到 `./lcm_to_ros/repub_configs/*.cfg`，[example](./lcm_to_ros/repub_configs/example_republishers.cfg)，执行
    ```bash
    cd ~/robot_tools/lcm_to_ros
    ./rosrepub-gen.sh repub_configs/humanoid_republishers.cfg
    ```

* 编译
  ```bash
  cd ~/robot_tools/catkin_ws
  catkin_make
  ```
  **每次改变 lcm 消息类型，且该类型会在 `magic_vis`中被使用，需要删掉 workspace 下的 `build` 和 `devel` 文件夹，重头开始编译**

* 运行
  ```bash
  # 终端 1
  cd ~/robot_tools/catkin_ws
  source devel/setup.bash
  roslaunch lcm_to_ros humanoid_republishers.launch

  # 终端 2
  cd ~/robot_tools/catkin_ws
  source devel/setup.bash
  roslaunch magic_vis magic_humanoid.launch
  ```

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
