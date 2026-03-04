#!/bin/bash

# 脚本名称: manage_motionstudio_mode.sh
# 功能: 修改 motionstudio_mode 文件中的模式值，文件不存在时自动创建
# 参数: 第一个参数是模式(play/record/playback/watch)，第二个参数是值(0/1)

# 检查参数数量
if [ "$#" -ne 2 ]; then
    echo "错误: 需要两个参数"
    echo "用法: $0 <模式> <值>"
    echo "模式: play_lcm, record, playback, watch"
    echo "值: 0 (关闭), 1 (打开)"
    exit 1
fi

MODE=$1
VALUE=$2
FILE="motionstudio_mode"

# 检查文件是否存在，如果不存在则创建
if [ ! -f "$FILE" ]; then
    echo "文件 $FILE 不存在，正在创建..."
    # 创建文件并设置默认值
    cat > "$FILE" << EOF
play_lcm 0
record 0
playback 0
watch 0
EOF
    echo "已创建文件 $FILE，设置默认值为 0"
fi

# 检查值是否合法
if [ "$VALUE" != "0" ] && [ "$VALUE" != "1" ]; then
    echo "错误: 值必须是 0 或 1"
    exit 1
fi

# 检查模式是否合法
if [ "$MODE" != "play_lcm" ] && [ "$MODE" != "record" ] && [ "$MODE" != "playback" ] && [ "$MODE" != "watch" ]; then
    echo "错误: 模式必须是 play_lcm, record, playback 或 watch"
    exit 1
fi

# 读取当前值
CURRENT_PLAY=$(grep -E "^play_lcm" "$FILE" | awk '{print $2}')
CURRENT_RECORD=$(grep -E "^record" "$FILE" | awk '{print $2}')
CURRENT_PLAYBACK=$(grep -E "^playback" "$FILE" | awk '{print $2}')
CURRENT_WATCH=$(grep -E "^watch" "$FILE" | awk '{print $2}')

# 验证规则
if [ "$VALUE" -eq 1 ]; then
    case "$MODE" in
        "play_lcm")
            # 当 play_lcm 为 1 时，其他字段不允许为 1（除了 record）
            if [ "$CURRENT_PLAYBACK" -eq 1 ]; then
                echo "错误: 请先关闭ros2回放，再开启lcm播放"
                exit 1
            fi

            if [ "$CURRENT_WATCH" -eq 1 ]; then
                echo "错误: 请先停止在线监控，再开启lcm播放"
                exit 1
            fi
            ;;
        "record")
            # 当 record 为 1 时，没有特别限制
            ;;
        "playback")
            if [ "$CURRENT_PLAY" -eq 1 ];then
                echo "错误: 请先关闭lcm播放功能，再开启ros2回放"
                exit 1
            fi

            if [ "$CURRENT_RECORD" -eq 1 ];then
                echo "错误: 请先停止ros2 bag的录制功能，再开启ros2回放"
                exit 1
            fi

            if [ "$CURRENT_WATCH" -eq 1 ];then
                echo "错误: 请先停止在线监控，再开启ros2的回放"
                exit 1
            fi
            ;;
        "watch")
            # 当 watch 为 1 时，只允许 record 为 1
            if [ "$CURRENT_PLAY" -eq 1 ]; then
                echo "错误: 请先关闭lcm播放功能，再开启在线监控"
                exit 1
            fi

            if [ "$CURRENT_PLAYBACK" -eq 1 ]; then
                echo "错误: 请先关闭ros2回放，再开启在线监控"
                exit 1
            fi
            ;;
    esac
else
    # 值为 0 时，没有特别限制
    :
fi


# 修改文件
sed -i "s/^$MODE .*/$MODE $VALUE/" "$FILE"
echo "模式切换成功: $MODE $VALUE"
exit 0