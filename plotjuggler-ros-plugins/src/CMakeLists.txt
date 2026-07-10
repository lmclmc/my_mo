add_definitions(${QT_DEFINITIONS})
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)

find_package(lcm_to_ros2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

QT5_WRAP_UI ( COMMON_UI_SRC
    qnodedialog.ui
    dialog_select_ros_topics.ui
    dialog_with_itemlist.ui
    publisher_select_dialog.ui )


add_library( commonROS STATIC
   dialog_select_ros_topics.h
    dialog_select_ros_topics.cpp
    dialog_with_itemlist.h
    publisher_select_dialog.h
    parser_configuration.cpp
    parser_configuration.h
    ros_parsers/ros2_parser.cpp
    tokenize.c
    lcmgen.c
    LcmTopicDialog.cpp
    ${COMMON_UI_SRC}
)

target_link_libraries( commonROS
    PUBLIC
        Qt5::Widgets
        Qt5::Xml
        rclcpp::rclcpp
        rcpputils::rcpputils
        rosbag2_transport::rosbag2_transport
        tf2_ros::tf2_ros
        plotjuggler::plotjuggler_base
        ${tf2_msgs_TARGETS}
    )

add_library( DataLoadLCM SHARED DataLoadLCM/dataload_lcm.cpp)
add_library( DataStreamLCM SHARED DataStreamLCM/datastream_lcm.cpp)

target_link_libraries( DataLoadLCM commonROS lcm)
target_link_libraries( DataStreamLCM commonROS lcm)

add_library( TopicPublisherROS2 SHARED TopicPublisherROS2/publisher_ros2.cpp)
target_link_libraries( TopicPublisherROS2 commonROS)

ament_target_dependencies(TopicPublisherROS2
  lcm_to_ros2
  sensor_msgs
  geometry_msgs
)

message("AMENT_PREFIX_PATH = ${AMENT_PREFIX_PATH}")
message("ROS_DISTRO (env) = $ENV{ROS_DISTRO}")

if("$ENV{ROS_DISTRO}" STREQUAL "humble")
        message(STATUS "Detected Humble")
        target_compile_definitions(TopicPublisherROS2 PUBLIC ROS_HUMBLE)
endif()

#######################################################################


install(TARGETS
        DataLoadLCM
        DataStreamLCM
        TopicPublisherROS2
        DESTINATION  lib/${PROJECT_NAME}
    )
