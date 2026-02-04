#!/bin/bash
#
pushd src
source install/setup.bash

ros2 launch magic_vis_ros2 magic_humanoid.py &

ros2 launch lcm_to_ros2 humanoid_republishers.py
