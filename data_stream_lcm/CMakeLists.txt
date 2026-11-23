find_package(asio REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(lz4 REQUIRED)
find_package(zstd REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB REQUIRED IMPORTED_TARGET glib-2.0)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/EmbedUi.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/EmbedManifest.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/EmbedGitHistory.cmake)

add_library(lcm_stream_source_plugin SHARED lcm_stream_source.cpp tokenize.c lcmgen.c robot_state_publisher.cpp)
pj_embed_git_history(lcm_stream_source_plugin)
target_compile_features(lcm_stream_source_plugin PRIVATE cxx_std_20)
target_compile_options(lcm_stream_source_plugin PRIVATE ${PJ_WARNING_FLAGS})
target_include_directories(lcm_stream_source_plugin SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/contrib
  ${orocos_kdl_INCLUDE_DIRS}
  /opt/ros/humble/include/builtin_interfaces/
  /opt/ros/humble/include/rosidl_runtime_cpp/
  /opt/ros/humble/include/rosidl_runtime_c/
  /opt/ros/humble/include/rosidl_typesupport_interface/
  /opt/ros/humble/include/rcutils/
  /opt/ros/humble/include/rcl_interfaces/
  /opt/ros/humble/include/sensor_msgs/
  /opt/ros/humble/include/std_msgs/
  /opt/ros/humble/include/tf2_ros/
  /opt/ros/humble/include/rclcpp/
  /opt/ros/humble/include/rcpputils/
  /opt/ros/humble/include/rcl/
  /opt/ros/humble/include/rmw/
  /opt/ros/humble/include/rcl_yaml_param_parser/
  /opt/ros/humble/include/type_description_interfaces/
  /opt/ros/humble/include/service_msgs/
  /opt/ros/humble/include/rosidl_dynamic_typesupport/
  /opt/ros/humble/include/tracetools/
  /opt/ros/humble/include/libstatistics_collector/
  /opt/ros/humble/include/statistics_msgs/
  /opt/ros/humble/include/rosidl_typesupport_introspection_cpp/
  /opt/ros/humble/include/geometry_msgs/
  /opt/ros/humble/include/tf2_msgs/
  /opt/ros/humble/include/urdf/
  /opt/ros/humble/include/kdl_parser/
  /opt/ros/humble/include/rclcpp_components/
  /opt/ros/humble/include/class_loader/
  /opt/ros/humble/include/urdf/
  /opt/ros/humble/include/urdfdom_headers/
  /opt/ros/humble/include/visualization_msgs/
)
target_link_libraries(lcm_stream_source_plugin PRIVATE
  asio::asio
  plotjuggler_sdk::plugin_sdk
  nlohmann_json::nlohmann_json
  LZ4::lz4_static
  zstd::libzstd_static
  PkgConfig::GLIB
  ${orocos_kdl_LIBRARIES}
  lcm
  tinyxml2
  /opt/ros/humble/lib/liburdf.so
  /opt/ros/humble/lib/libkdl_parser.so
  /opt/ros/humble/lib/librclcpp.so
  /opt/ros/humble/lib/librcutils.so
  /opt/ros/humble/lib/libstd_msgs__rosidl_typesupport_cpp.so
  /opt/ros/humble/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
  /opt/ros/humble/lib/libsensor_msgs__rosidl_typesupport_cpp.so
  /opt/ros/humble/lib/libtf2_msgs__rosidl_typesupport_cpp.so
  /opt/ros/humble/lib/libvisualization_msgs__rosidl_typesupport_cpp.so
)

pj_embed_ui(lcm_stream_source_plugin
  UI_FILE  ${CMAKE_CURRENT_SOURCE_DIR}/datastream_lcm.ui
  HEADER   ${CMAKE_CURRENT_BINARY_DIR}/generated/datastream_lcm_ui.hpp
  VAR_NAME kDataStreamLcmUi
)

pj_embed_manifest(lcm_stream_source_plugin
  HEADER   ${CMAKE_CURRENT_BINARY_DIR}/generated/mcap_manifest.hpp
  VAR_NAME kMcapManifest
)

# v4 plugin discovery: emit a sidecar .pjmanifest.json the host can scan
# pre-dlopen. Content derived from manifest.json + {abi_major, family}.
pj_emit_plugin_manifest(lcm_stream_source_plugin
  FAMILY        data_source
  MANIFEST_FILE ${CMAKE_CURRENT_SOURCE_DIR}/manifest.json
)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/HardenPluginExports.cmake)
pj_harden_plugin_exports(lcm_stream_source_plugin
  REQUIRED_EXPORTS PJ_get_data_source_vtable PJ_get_dialog_vtable)
