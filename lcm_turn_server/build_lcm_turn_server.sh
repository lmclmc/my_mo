#!/bin/bash

# 定义变量
DOCKER_IMAGE="harbor.dreame.tech/eame/rk3588-cross-compile:v2"
SOURCE_FILE="lcm_turn_server.cpp"
OUTPUT_BINARY="lcm_turn_server"
CONTAINER_NAME="temp_compile_container"

echo "开始编译 lcm_turn_server.cpp..."

# 启动临时容器并编译
docker run --rm --name $CONTAINER_NAME \
  -v $(pwd):/workspace \
  -w /workspace \
  $DOCKER_IMAGE \
  bash -c "aarch64-linux-gnu-g++ -O3 -o $OUTPUT_BINARY $SOURCE_FILE -lpthread -L/rk3588_rootfs/usr/local/lib/ -llcm &&
           aarch64-linux-gnu-strip $OUTPUT_BINARY"

# 检查编译结果
if [ $? -eq 0 ]; then
    echo "编译成功！生成的可执行文件: $OUTPUT_BINARY"
    # 显示文件信息
    ls -la $OUTPUT_BINARY
else
    echo "编译失败，请检查错误信息。"
    exit 1
fi

echo "编译完成，临时容器已自动删除。"