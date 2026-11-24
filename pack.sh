#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Concurrent calls share the build outputs and the pack directory.
exec 9< .
flock 9

git_executable="$(command -v git)"
unset PJ_PACK_GIT_HISTORY
scratch="$(mktemp -d "$SCRIPT_DIR/.pack.XXXXXX")"
history_file="$scratch/pj_git_history.hpp"
archive_tmp="$scratch/pack.tar.gz"
trap 'rm -f -- "$history_file" "$archive_tmp"; rmdir -- "$scratch"' EXIT
cmake -DPJ_HISTORY_SOURCE_DIR="$SCRIPT_DIR" \
  -DPJ_HISTORY_OUTPUT="$history_file" \
  -DPJ_HISTORY_GIT="$git_executable" \
  -DPJ_HISTORY_REQUIRE_GIT=ON \
  -P common/update_history/GenerateGitHistory.cmake
export PJ_PACK_GIT_HISTORY="$history_file"

./build.sh data_load_lcm
./build.sh data_stream_lcm
./build.sh data_stream_dds
./build.sh data_load_mcap

copy_plugin() {
  local plugin_dir="$1" target="$2" destination="pack/lcm_plugin/$3"
  local bin="build/$plugin_dir/Release/bin"
  mkdir -p "$destination"
  cp -v -- "$bin/lib$target.so" "$bin/$target.pjmanifest.json" "$destination/"
}

copy_plugin data_load_lcm lcm_source_plugin lcm-loader
copy_plugin data_load_mcap mcap_source_plugin mcap-loader
copy_plugin data_stream_lcm lcm_stream_source_plugin lcm-streamer
copy_plugin data_stream_dds dds_stream_source_plugin dds-streamer

tar -czf "$archive_tmp" pack
mv -f -- "$archive_tmp" pack.tar.gz
