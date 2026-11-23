# Embed a build-time, offline snapshot of the current repository's recent commits.
# Usage (after add_library):
#   include(${CMAKE_CURRENT_LIST_DIR}/../cmake/EmbedGitHistory.cmake)
#   pj_embed_git_history(my_plugin)
# The plugin can then include <update_history.hpp> and pass
# PjUpdateHistory::dialogUi() to WidgetData::requestSubDialog().
include_guard(GLOBAL)

function(pj_embed_git_history TARGET)
  find_package(Git QUIET)
  set(_history_root "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../common/update_history")
  set(_history_generator "${_history_root}/GenerateGitHistory.cmake")
  set(_history_output "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET}/pj_git_history.hpp")
  set(_history_arguments
    "-DPJ_HISTORY_SOURCE_DIR=${CMAKE_CURRENT_FUNCTION_LIST_DIR}/.."
    "-DPJ_HISTORY_OUTPUT=${_history_output}"
    "-DPJ_HISTORY_GIT=${GIT_EXECUTABLE}")

  # Also provide the header at configure time for IDEs and dependency scanners.
  execute_process(COMMAND "${CMAKE_COMMAND}" ${_history_arguments} -P "${_history_generator}"
    RESULT_VARIABLE _history_result)
  if(NOT _history_result EQUAL 0)
    message(FATAL_ERROR "Failed to generate the update history header")
  endif()

  # Always inspect Git at build time: commits, checkout, reset and worktree refs
  # must all refresh the snapshot, including with Ninja. The generator writes
  # only on content changes, so an unchanged log does not recompile the plugin.
  add_custom_target(${TARGET}_git_history
    COMMAND "${CMAKE_COMMAND}" ${_history_arguments} -P "${_history_generator}"
    BYPRODUCTS "${_history_output}"
    VERBATIM)
  add_dependencies(${TARGET} ${TARGET}_git_history)
  target_include_directories(${TARGET} PRIVATE
    "${_history_root}" "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET}")
endfunction()
