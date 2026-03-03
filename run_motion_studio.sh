#!/bin/bash
#

# 进入src目录并source setup文件
ROS_DISTRO="jazzy"
source /opt/ros/${ROS_DISTRO}/setup.bash
pushd src
source install/setup.bash

# 显示选择菜单
echo "请选择要运行的Python脚本："
echo "1) magic_humanoid.py"
echo "2) big_dog.py"
echo "3) magic_humanoid_gen2_v2.py"
echo "4) magicdog.py"

# 读取用户输入
read -p "请输入选择 (1 或 2): " choice

sudo chown -R $(id -gn):$(id -un) /opt/ros/
# 根据用户选择执行相应的命令
case $choice in
    1)
        echo "正在启动 magic_humanoid.py..."
        nohup ros2 launch magic_vis_ros2 magic_humanoid.py &
        ;;
    2)
        echo "正在启动 big_dog.py..."
        nohup ros2 launch magic_vis_ros2 big_dog.py &
        ;;
    3)
        echo "正在启动 magic_humanoid_gen2_v2.py..."
        nohup ros2 launch magic_vis_ros2 magic_humanoid_gen2_v2.py &
        ;;
    4)
        echo "正在启动 magicdog.py..."
        nohup ros2 launch magic_vis_ros2 magicdog.py &
        ;;
    *)
        echo "无效的选择，默认启动 magic_humanoid.py..."
        nohup ros2 launch magic_vis_ros2 magic_humanoid.py &
        ;;
esac

# 启动 republishers
nohup ros2 launch lcm_to_ros2 humanoid_republishers.py &
popd
nohup ros2 run plotjuggler plotjuggler &
nohup python motion_controller/motion_window.py &
