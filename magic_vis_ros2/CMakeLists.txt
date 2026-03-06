cmake_minimum_required(VERSION 3.8)
project(magic_vis_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lcm_to_ros2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(urdf REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tinyxml2 REQUIRED)

include_directories(
  include
  src
  ${pinocchio_INCLUDE_DIRS}
)

add_executable(magic_vis_ros2 src/robot_state_publisher.cpp)
ament_target_dependencies(magic_vis_ros2
  rclcpp
  tf2
  tf2_ros
  sensor_msgs
  urdf
  kdl_parser
  rclcpp_components
  tinyxml2
  lcm_to_ros2
)

target_link_libraries(magic_vis_ros2
  tinyxml2::tinyxml2
)

# Install libraries and executables
install(TARGETS
  magic_vis_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install URDF and mesh files
install(DIRECTORY urdf/ meshes/ rviz/
  DESTINATION share/${PROJECT_NAME}
)

# Install  mesh files
install(DIRECTORY  meshes/
  DESTINATION share/meshes
)

# Install  mesh files
install(DIRECTORY  meshes/
  DESTINATION share/ament_index/resource_index/packages/meshes
)

ament_package()