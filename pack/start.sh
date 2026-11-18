#!/bin/bash

rm -rf ~/.local/share/PlotJuggler/PlotJuggler4/extensions/*
rm -rf ~/.config/PlotJuggler/PlotJuggler4.conf

mkdir -p ~/.config/PlotJuggler/
mkdir -p ~/.local/share/PlotJuggler/PlotJuggler4/extensions/lcm-loader/
mkdir -p ~/.local/share/PlotJuggler/PlotJuggler4/extensions/lcm-streamer/
mkdir -p ~/.local/share/PlotJuggler/PlotJuggler4/extensions/ros-parser/
mkdir -p ~/.local/share/PlotJuggler/PlotJuggler4/extensions/mcap-loader/
mkdir -p ~/.local/share/PlotJuggler/PlotJuggler4/extensions/dds-streamer/
mkdir -p ~/.local/share/PlotJuggler/PlotJuggler4/extensions/toolbox-transform-editor/
mkdir -p ~/.local/share/PlotJuggler/PlotJuggler4/extensions/toolbox-quaternion/
mkdir -p ~/.local/share/PlotJuggler/PlotJuggler4/extensions/toolbox-fft/

cp -r lcm_plugin/lcm-loader/* ~/.local/share/PlotJuggler/PlotJuggler4/extensions/lcm-loader/
cp -r lcm_plugin/lcm-streamer/* ~/.local/share/PlotJuggler/PlotJuggler4/extensions/lcm-streamer/
cp -r lcm_plugin/ros-parser/* ~/.local/share/PlotJuggler/PlotJuggler4/extensions/ros-parser/
cp -r lcm_plugin/mcap-loader/* ~/.local/share/PlotJuggler/PlotJuggler4/extensions/mcap-loader/
cp -r lcm_plugin/dds-streamer/* ~/.local/share/PlotJuggler/PlotJuggler4/extensions/dds-streamer/
cp -r lcm_plugin/toolbox-transform-editor/* ~/.local/share/PlotJuggler/PlotJuggler4/extensions/toolbox-transform-editor/
cp -r lcm_plugin/toolbox-fft/* ~/.local/share/PlotJuggler/PlotJuggler4/extensions/toolbox-fft/
cp -r lcm_plugin/toolbox-quaternion/* ~/.local/share/PlotJuggler/PlotJuggler4/extensions/toolbox-quaternion/
cp PlotJuggler4.conf ~/.config/PlotJuggler/

export AMENT_PREFIX_PATH=$(pwd)/lcm_plugin/
export LD_LIBRARY_PATH=$(pwd)/lcm_plugin/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/ros/humble/lib/x86_64-linux-gnu/

#nohup ./plotjuggler4 & > plot.log
./plotjuggler4
