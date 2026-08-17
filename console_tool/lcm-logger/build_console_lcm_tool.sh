#!/bin/bash

/usr/bin/aarch64-linux-gnu-g++ -O3 -s main.cpp -L/rk3588_rootfs/usr/local/lib/ -llcm -lpthread -o console_lcm_tool