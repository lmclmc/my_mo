#!/bin/bash

source /opt/ros/jazzy/setup.bash

mkdir src
ln -svf $(pwd)/lcm_to_ros2 $(pwd)/src/lcm_to_ros2
ln -svf $(pwd)/magic_vis_ros2 $(pwd)/src/magic_vis_ros2

pushd src
colcon build --packages-select lcm_to_ros2
colcon build --packages-select magic_vis_ros2
popd
