#!/bin/bash


ROS_DISTRO=humble
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

if [ -f "/.dockerenv" ]; then
    echo "当前是 Docker 容器环境"
else
    echo "宿主机环境"
    echo $password | sudo -S chown -R $(id -gn):$(id -un) /opt/ros/
fi

ln -svf $(pwd)/plotjuggler-ros-plugins $(pwd)/src/plotjuggler-ros-plugins


pushd src
colcon build --packages-select plotjuggler-ros-plugins
popd

echo $password | sudo -S cp $(pwd)/src/install/plotjuggler-ros-plugins/lib/plotjuggler-ros-plugins/libTopicPublisherROS2.so /opt/ros/${ROS_DISTRO}/lib/plotjuggler_ros/libTopicPublisherROS2.so
echo $password | sudo -S cp $(pwd)/src/install/plotjuggler-ros-plugins/lib/plotjuggler-ros-plugins/libDataLoadLCM.so /opt/ros/${ROS_DISTRO}/lib/plotjuggler_ros/libDataLoadLCM.so
echo $password | sudo -S cp $(pwd)/src/install/plotjuggler-ros-plugins/lib/plotjuggler-ros-plugins/libDataStreamLCM.so /opt/ros/${ROS_DISTRO}/lib/plotjuggler_ros/libDataStreamLCM.so

mkdir pack
cp /opt/ros/humble/lib/plotjuggler/plotjuggler pack/
cp /opt/ros/humble/lib/plotjuggler/libToolbox* pack/
cp ./src/build/plotjuggler-ros-plugins/bin/libData* pack/
/work/lmc/Desktop/squashfs-root/AppRun pack/plotjuggler -bundle-non-qt-libs -always-overwrite
tar zcvf pack.tar.gz pack
./version_info.sh
