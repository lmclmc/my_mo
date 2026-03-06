#!/bin/bash


ROS_DISTRO=jazzy
source /opt/ros/${ROS_DISTRO}/setup.bash
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

echo $password | sudo -S chown -R $(id -gn):$(id -un) /opt/ros/

pushd lcm_turn_server
echo $password | sudo -S ./build_lcm_turn_server.sh
cp lcm_turn_server /opt/ros/
popd

mkdir src
ln -svf $(pwd)/lcm_to_ros2 $(pwd)/src/lcm_to_ros2
ln -svf $(pwd)/magic_vis_ros2 $(pwd)/src/magic_vis_ros2
ln -svf $(pwd)/plotjuggler-ros-plugins $(pwd)/src/plotjuggler-ros-plugins

pushd scripts
echo $password | sudo -S cp motion_studio_mode_switch.sh /opt/ros/
popd

pushd src
colcon build --packages-select lcm_to_ros2
colcon build --packages-select magic_vis_ros2
source install/setup.bash
colcon build --packages-select plotjuggler-ros-plugins
popd

echo $password | sudo -S cp $(pwd)/src/install/plotjuggler-ros-plugins/lib/plotjuggler-ros-plugins/libTopicPublisherROS2.so /opt/ros/${ROS_DISTRO}/lib/plotjuggler_ros/libTopicPublisherROS2.so
