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

# Install libraries and executables
install(TARGETS
  robot_vis
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