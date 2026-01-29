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

add_subdirectory(src/Goldfarb_Optimizer)

include_directories(
  include
  src
  ${pinocchio_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/model/model_humanoid.cpp
  src/robot_vis/urdf_visualizer.cpp
  src/marker_vis/marker_base.cpp
  src/marker_vis/marker_humanoid.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  kdl_parser
  visualization_msgs
  lcm_to_ros2
  geometry_msgs
  ament_index_cpp
  urdf
  std_msgs
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}
  ${pinocchio_LIBRARIES}
  Goldfarb_Optimizer
)

# some executable nodes
add_executable(node_marker_vis src/marker_vis/marker_visualizer.cpp)
ament_target_dependencies(node_marker_vis
  rclcpp
  tf2
  tf2_ros
  visualization_msgs
  lcm_to_ros2
)
target_link_libraries(node_marker_vis
  ${PROJECT_NAME}
)

add_executable(node_p3b_vis src/robot_vis/urdf_visualizer_p3b.cpp)
ament_target_dependencies(node_p3b_vis
  rclcpp
  tf2
  tf2_ros
  lcm_to_ros2
)
target_link_libraries(node_p3b_vis
  ${PROJECT_NAME}
)

add_executable(node_p4_vis src/robot_vis/urdf_visualizer_p4.cpp)
ament_target_dependencies(node_p4_vis
  rclcpp
  tf2
  tf2_ros
  lcm_to_ros2
)
target_link_libraries(node_p4_vis
  ${PROJECT_NAME}
)

add_executable(node_p5_vis src/robot_vis/urdf_visualizer_p5.cpp)
ament_target_dependencies(node_p5_vis
  rclcpp
  tf2
  tf2_ros
  lcm_to_ros2
)




add_executable(robot_vis src/robot_vis/robot_state_publisher.cpp)
ament_target_dependencies(robot_vis
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

target_link_libraries(robot_vis
  ${PROJECT_NAME}
  tinyxml2::tinyxml2
)

# add_library(
#   ${PROJECT_NAME}_node SHARED
#   src/robot_vis/robot_state_publisher.cpp)
# target_include_directories(${PROJECT_NAME}_node PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
# target_link_libraries(${PROJECT_NAME}_node PUBLIC
#   ${builtin_interfaces_TARGETS}
#   orocos-kdl
#   ${rcl_interfaces_TARGETS}
#   rclcpp::rclcpp
#   ${sensor_msgs_TARGETS}
#   ${std_msgs_TARGETS}
#   tf2_ros::tf2_ros
#   urdf::urdf
# )
# target_link_libraries(${PROJECT_NAME}_node PRIVATE
#   ${geometry_msgs_TARGETS}
#   kdl_parser::kdl_parser
#   rclcpp_components::component
# )
# ament_export_targets(export_${PROJECT_NAME}_node)

# rclcpp_components_register_node(${PROJECT_NAME}_node
#   PLUGIN "robot_state_publisher::RobotStatePublisher"
#   EXECUTABLE robot_state_publisher)

# install(
#   TARGETS
#   ${PROJECT_NAME}_node
#   EXPORT export_${PROJECT_NAME}_node
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(DIRECTORY include/
#   DESTINATION include/${PROJECT_NAME})

# install(DIRECTORY launch urdf
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_export_dependencies(robot_state_publisher
#   builtin_interfaces
#   orocos_kdl
#   rcl_interfaces
#   rclcpp
#   sensor_msgs
#   std_msgs
#   tf2_ros
#   urdf
# )














































target_link_libraries(node_p5_vis
  ${PROJECT_NAME}
)

# Install libraries and executables
install(TARGETS ${PROJECT_NAME}
  robot_vis
  node_marker_vis
  node_p3b_vis
  node_p4_vis
  node_p5_vis
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

ament_package()