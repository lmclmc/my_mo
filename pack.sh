#!/bin/bash

./build.sh data_load_lcm
./build.sh data_stream_lcm

cp -v build/data_load_lcm/Release/bin/* pack/lcm_plugin/lcm-loader/
cp -v build/data_stream_lcm/Release/bin/* pack/lcm_plugin/lcm-streamer/

tar zcvf pack.tar.gz pack
