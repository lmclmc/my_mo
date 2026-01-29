cmake_minimum_required(VERSION 3.8)
project(lcm_to_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find LCM
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${LCM_INCLUDE_DIRS}
)

# ROS2 message generation
file(GLOB ros2_msg_files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

if(ros2_msg_files)
    rosidl_generate_interfaces(${PROJECT_NAME}
        ${ros2_msg_files}
        DEPENDENCIES std_msgs
    )
endif()

# Auto-generated republisher nodes
get_filename_component(_fullPath autosrc ABSOLUTE)
if(EXISTS ${_fullPath}/CMakeLists.txt)
    add_subdirectory(autosrc)
endif()

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()

