#!/bin/bash
#

# 进入src目录并source setup文件
ROS_DISTRO="jazzy"

password=""
# ======================== 检查是否为 root 用户 ========================
check_root() {
    password=$1
    if [[ -z $password ]]; then
        echo "用法：$0 <password>"
        exit 1
    fi
}

check_root $1

source /opt/ros/${ROS_DISTRO}/setup.bash
pushd src
source install/setup.bash

# 显示选择菜单
echo "请选择要运行的Python脚本："
echo "1) magic_humanoid.py"
echo "2) big_dog.py"
echo "3) magic_humanoid_gen2_v2.py"
echo "4) magicdog.py"
echo "5) MagicBotZ1_23dof.py"
echo "6) A2601-M-MINI-BOT.py"

# 读取用户输入
read -p "请输入选择 (1 2 3 4 5 6): " choice

echo $password | sudo -S chown -R $(id -gn):$(id -un) /opt/ros/
rm -rf /opt/ros/motionstudio_mode
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
    5)
        echo "正在启动 MagicBotZ1_23dof.py..."
        nohup ros2 launch magic_vis_ros2 MagicBotZ1_23dof.py &
        ;;
    6)
        echo "正在启动 MagicBotZ1_23dof.py..."
        nohup ros2 launch magic_vis_ros2 A2601-M-MINI-BOT.py &
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
