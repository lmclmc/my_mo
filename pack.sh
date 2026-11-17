#!/bin/bash

./build.sh data_load_lcm
./build.sh data_stream_lcm
./build.sh data_stream_dds

mkdir -p pack/lcm_plugin/lcm-loader/
mkdir -p pack/lcm_plugin/lcm-streamer/
mkdir -p pack/lcm_plugin/dds-streamer/

cp -v build/data_load_lcm/Release/bin/* pack/lcm_plugin/lcm-loader/
cp -v build/data_stream_lcm/Release/bin/* pack/lcm_plugin/lcm-streamer/
cp -v build/data_stream_dds/Release/bin/* pack/lcm_plugin/dds-streamer/

tar zcvf pack.tar.gz pack
