cmake_minimum_required(VERSION 2.8.3)
project(magic_vis)

add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(pinocchio REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  kdl_parser
  robot_state_publisher
  visualization_msgs
  lcm_to_ros
)

add_subdirectory(src/Goldfarb_Optimizer)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS lcm_to_ros
)


###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/model/model_humanoid.cpp

  src/robot_vis/urdf_visualizer.cpp

  src/marker_vis/marker_base.cpp
  src/marker_vis/marker_humanoid.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  Goldfarb_Optimizer
)

# some executable nodes
add_executable(node_marker_vis src/marker_vis/marker_visualizer.cpp)
target_link_libraries(node_marker_vis
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(node_p3b_vis src/robot_vis/urdf_visualizer_p3b.cpp)
target_link_libraries(node_p3b_vis
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(node_p4_vis src/robot_vis/urdf_visualizer_p4.cpp)
target_link_libraries(node_p4_vis
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(node_p5_vis src/robot_vis/urdf_visualizer_p5.cpp)
target_link_libraries(node_p5_vis
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(node_gen2_vis src/robot_vis/urdf_visualizer_gen2.cpp)
target_link_libraries(node_gen2_vis
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)