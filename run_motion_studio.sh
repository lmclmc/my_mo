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
popd
# 检查log目录是否存在，且是一个目录（排除同名文件的情况）
if [ ! -d log ]; then
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] 检测到${LOG_DIR}目录不存在，开始创建..."
    
    # 创建目录，-p确保多级目录也能创建（这里单级也适用），-v输出创建信息
    mkdir -pv "log"
fi

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
        nohup ros2 launch magic_vis_ros2 magic_humanoid.py >> log/magic_vis_ros2.log 2>&1 &
        ;;
    2)
        echo "正在启动 big_dog.py..."
        nohup ros2 launch magic_vis_ros2 big_dog.py >> log/magic_vis_ros2.log 2>&1 &
        ;;
    3)
        echo "正在启动 magic_humanoid_gen2_v2.py..."
        nohup ros2 launch magic_vis_ros2 magic_humanoid_gen2_v2.py >> log/magic_vis_ros2.log 2>&1 &
        ;;
    4)
        echo "正在启动 magicdog.py..."
        nohup ros2 launch magic_vis_ros2 magicdog.py >> log/magic_vis_ros2.log 2>&1 &
        ;;
    5)
        echo "正在启动 MagicBotZ1_23dof.py..."
        nohup ros2 launch magic_vis_ros2 MagicBotZ1_23dof.py >> log/magic_vis_ros2.log 2>&1 &
        ;;
    6)
        echo "正在启动 A2601-M-MINI-BOT.py..."
        nohup ros2 launch magic_vis_ros2 A2601-M-MINI-BOT.py >> log/magic_vis_ros2.log 2>&1 &
        ;;
    *)
        echo "无效的选择，默认启动 magic_humanoid.py..."
        nohup ros2 launch magic_vis_ros2 magic_humanoid.py >> log/magic_vis_ros2.log 2>&1 &
        ;;
esac

# 启动 republishers
nohup ros2 launch lcm_to_ros2 humanoid_republishers.py >> log/lcm_to_ros2.log 2>&1 &
nohup ros2 run plotjuggler plotjuggler >> log/plotjuggler_plugin.log 2>&1 &
nohup python -u motion_controller/motion_window.py >> log/motion_window.log 2>&1 &