#!/bin/bash

source /opt/ros/jazzy/setup.bash

mkdir src
ln -svf $(pwd)/lcm_to_ros2 $(pwd)/src/lcm_to_ros2
ln -svf $(pwd)/magic_vis_ros2 $(pwd)/src/magic_vis_ros2
ln -svf $(pwd)/plotjuggler-ros-plugins $(pwd)/src/plotjuggler-ros-plugins

pushd src
colcon build --packages-select lcm_to_ros2
colcon build --packages-select magic_vis_ros2
source install/setup.bash
colcon build --packages-select plotjuggler-ros-plugins
popd

sudo cp $(pwd)/src/install/plotjuggler-ros-plugins/lib/plotjuggler-ros-plugins/libTopicPublisherROS2.so /opt/ros/${ROS_DISTRO}/lib/plotjuggler_ros/libTopicPublisherROS2.so