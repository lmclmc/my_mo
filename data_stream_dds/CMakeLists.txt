list(PREPEND CMAKE_PREFIX_PATH "/opt/ros/humble/lib/x86_64-linux-gnu/cmake/")

find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)
find_package(CycloneDDS REQUIRED)
if(TARGET CycloneDDS::idlc)
  set(PJ_DDS_IDLC "$<TARGET_FILE:CycloneDDS::idlc>")
else()
  find_program(PJ_DDS_IDLC NAMES idlc REQUIRED)
endif()
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/EmbedUi.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/EmbedManifest.cmake)

add_library(dds_stream_source_plugin SHARED
  dds_stream_source.cpp dds_dynamic.cpp dds_idl.cpp dds_type_schema.cpp dds_recorder.cpp dds_mcap_writer.cpp)
target_compile_features(dds_stream_source_plugin PRIVATE cxx_std_20)
# Only the upstream MCAP writer is vendored; uncompressed recording needs no
# extra compression or ROS dependencies.
target_include_directories(dds_stream_source_plugin SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/contrib)
target_compile_definitions(dds_stream_source_plugin PRIVATE
  PJ_DDS_IDLC="${PJ_DDS_IDLC}"
  PJ_DDS_CC="${CMAKE_C_COMPILER}"
  PJ_DDS_INCLUDE_DIRS="$<JOIN:$<TARGET_PROPERTY:CycloneDDS::ddsc,INTERFACE_INCLUDE_DIRECTORIES>,|>"
  PJ_DDS_LIBRARY="$<TARGET_FILE:CycloneDDS::ddsc>"
)
target_compile_options(dds_stream_source_plugin PRIVATE ${PJ_WARNING_FLAGS})
target_link_libraries(dds_stream_source_plugin PRIVATE
  plotjuggler_sdk::plugin_sdk
  nlohmann_json::nlohmann_json
  Threads::Threads
  CycloneDDS::ddsc
  ${CMAKE_DL_LIBS}
)

pj_embed_ui(dds_stream_source_plugin
  UI_FILE  ${CMAKE_CURRENT_SOURCE_DIR}/datastream_dds.ui
  HEADER   ${CMAKE_CURRENT_BINARY_DIR}/generated/datastream_dds_ui.hpp
  VAR_NAME kDataStreamDdsUi
)

pj_embed_manifest(dds_stream_source_plugin
  HEADER   ${CMAKE_CURRENT_BINARY_DIR}/generated/dds_manifest.hpp
  VAR_NAME kDdsManifest
)

# v4 plugin discovery: emit a sidecar .pjmanifest.json the host can scan
# pre-dlopen. Content derived from manifest.json + {abi_major, family}.
pj_emit_plugin_manifest(dds_stream_source_plugin
  FAMILY        data_source
  MANIFEST_FILE ${CMAKE_CURRENT_SOURCE_DIR}/manifest.json
)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/HardenPluginExports.cmake)
pj_harden_plugin_exports(dds_stream_source_plugin
  REQUIRED_EXPORTS PJ_get_data_source_vtable PJ_get_dialog_vtable)
