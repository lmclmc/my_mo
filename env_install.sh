#!/bin/bash

set -e  # 遇到错误立即退出
password=""
# ======================== 配置项 ========================
# 可修改为你需要的 ROS 2 版本（如 humble/iron/jazzy）
ROS2_DISTRO="jazzy"
# 适配的 Ubuntu 版本（Humble 对应 22.04，Iron 对应 22.04，Jazzy 对应 24.04）
UBUNTU_CODENAME="jammy"

# ======================== 颜色输出函数 ========================
green_echo() {
    echo -e "\033[32m$1\033[0m"
}
red_echo() {
    echo -e "\033[31m$1\033[0m"
}
yellow_echo() {
    echo -e "\033[33m$1\033[0m"
}

# ======================== 检查是否为 root 用户 ========================
check_root() {
    password=$1
    if [[ -z $password ]]; then
        red_echo "错误：该脚本需要以 root 权限运行，请使用 sudo ./install_ros2.sh 执行！"
        exit 1
    fi
}

# ======================== 安装 ROS 2 核心流程 ========================
install_ros2() {
    # 1. 设置语言环境
    yellow_echo "步骤 1/6：配置语言环境..."
    locale  # 检查当前 locale
    echo $password | sudo -S apt update && sudo apt install -y locales
    echo $password | sudo -S locale-gen en_US en_US.UTF-8
    echo $password | sudo -S update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # 2. 添加 ROS 2 软件源
    yellow_echo "步骤 2/6：添加 ROS 2 软件源..."
    echo $password | sudo -S apt install -y software-properties-common
    echo $password | sudo -S add-apt-repository universe -y

    # 3. 安装 ROS 2 官方 GPG 密钥（必须，否则源验证失败）
    echo $password | sudo -S apt update && sudo apt install -y curl
    echo $password | sudo -S curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

    # 4. 添加 Jazzy 专属源（重点：ubuntu 24.04 对应 noble）
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    # 3. 更新软件源
    yellow_echo "步骤 3/6：更新软件源索引..."
    echo $password | sudo -S apt update -y
    echo $password | sudo -S apt upgrade -y

    # 4. 安装 ROS 2 基础包（桌面版，包含 rviz、示例等）
    yellow_echo "步骤 4/6：安装 ROS 2 $ROS2_DISTRO 核心包..."
    echo $password | sudo -S apt install -y ros-$ROS2_DISTRO-desktop-full

    # 6. 配置环境变量（永久生效）
    yellow_echo "步骤 6/6：配置 ROS 2 环境变量..."
    if ! grep -q "ros2" ~/.bashrc; then
        echo "source /opt/ros/$ROS2_DISTRO/setup.bash" >> ~/.bashrc
        green_echo "✅ 环境变量已添加到 ~/.bashrc，新终端自动生效。"
    fi

    # 临时加载环境变量
    source /opt/ros/$ROS2_DISTRO/setup.bash

    # 验证安装
    green_echo "🎉 ROS 2 $ROS2_DISTRO 安装完成！"
    green_echo "验证命令：ros2 --version 或 ros2 run turtlesim turtlesim_node"
}

# ======================== 检查 ROS 2 是否已安装 ========================
check_ros2_installed() {
    yellow_echo "正在检查 ROS 2 $ROS2_DISTRO 是否已安装..."
    TARGET_FILE="/opt/ros/${ROS2_DISTRO}/setup.bash"
    # 检查 ros2 命令是否存在，且能识别版本
    if [ -f "$TARGET_FILE" ]; then
        green_echo "✅ ROS 2 $ROS2_DISTRO 已安装，无需重复安装，脚本退出。"
    else
        yellow_echo "❌ 未检测到 ROS 2 $ROS2_DISTRO，开始自动安装.
        install_ros2.."
    fi
}
# ===================== 配置项（可根据需求修改）=====================
# Miniconda 安装版本（适配 Linux x86_64）
MINICONDA_VERSION="py310_24.1.2-0"
# Miniconda 下载地址
MINICONDA_URL="https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/Miniconda3-latest-Linux-x86_64.sh"
# 安装目录（默认安装到用户家目录）
INSTALL_DIR="$HOME/miniconda3"
# 安装完成后是否自动初始化 conda（yes/no）
CONDA_INIT="yes"

# ===================== 核心函数 =====================
# 检查 conda 是否已安装
check_conda_exists() {
    if command -v conda &> /dev/null; then
        echo -e "\033[32m✅ conda 已安装，版本信息：\033[0m"
        conda --version
        return 0  # 存在则返回 0（成功）
    else
        echo -e "\033[31m❌ 未检测到 conda，准备安装 Miniconda...\033[0m"
        return 1  # 不存在则返回 1（失败）
    fi
}

# 下载并安装 Miniconda
install_miniconda() {
    # 1. 创建临时目录存放安装包
    TMP_DIR=$(mktemp -d)
    echo "📁 临时目录：$TMP_DIR"
    
    # 2. 下载 Miniconda 安装脚本
    echo "🌐 开始下载 Miniconda 安装包..."

    echo "$MINICONDA_URL"
    rm -rf Miniconda3-latest-Linux-x86_64.sh*
    wget "$MINICONDA_URL"
    
    # 3. 赋予安装脚本执行权限
    chmod +x Miniconda3-latest-Linux-x86_64.sh
    
    # 4. 静默安装 Miniconda（无交互）
    echo 🔧 开始安装 Miniconda
    ./Miniconda3-latest-Linux-x86_64.sh -b
    
    # 5. 清理临时文件
    rm -rf Miniconda3-latest-Linux-x86_64.sh
    echo "✅ Miniconda 安装完成！"
}

# 初始化 conda
init_conda() {
    if [ "$CONDA_INIT" = "yes" ]; then
        echo "🔧 初始化 conda 环境..."
        # 加载 conda 环境
        source ~/miniconda3/etc/profile.d/conda.sh
        # 初始化 conda（添加到 shell 配置文件，如 ~/.bashrc）
        conda init bash
        echo -e "\033[32m✅ conda 初始化完成，重启终端后生效\033[0m"
    else
        echo "ℹ️  未初始化 conda，如需使用请执行：source $INSTALL_DIR/bin/activate"
    fi
}

# ===================== 配置区 =====================
# 请在这里添加/修改你要检查/安装的软件包列表
# 格式：每个包名单独一行，用空格/换行分隔都可以
PACKAGE_LIST=(
    "colcon"
    "liblcm-dev"
    "ros-${ROS2_DISTRO}-plotjuggler"
    "ros-${ROS2_DISTRO}-plotjuggler-ros"
)
# ==================================================

# 函数：检查单个软件包是否已安装
check_package_installed() {
    local package="$1"
    # dpkg -s 检查包状态，重定向所有输出避免冗余
    if dpkg -s "$package" &> /dev/null; then
        return 0  # 已安装，返回成功状态码
    else
        return 1  # 未安装，返回失败状态码
    fi
}

# 函数：使用 apt 安装单个软件包
install_single_package() {
    local package="$1"
    echo "📦 开始安装 $package..."
    
    # 安装指定包，-y 自动确认所有交互提示
    if echo $password | sudo -S apt install -y "$package"; then
        # 验证安装结果
        if check_package_installed "$package"; then
            echo "✅ $package 安装成功！"
        else
            echo "❌ $package 安装失败（命令执行成功但检测不到安装状态）"
            return 1
        fi
    else
        echo "❌ $package 安装命令执行失败，请检查包名或网络"
        return 1
    fi
}

# 主逻辑：遍历包列表逐个处理
install_packages() {
    # 先检查系统是否支持 apt
    echo "🔍 检测系统包管理器..."
    if ! command -v apt &> /dev/null; then
        echo "❌ 当前系统不支持 apt 包管理器，脚本退出"
        exit 1
    fi
    echo "✅ 确认系统支持 apt，开始处理软件包列表"
    echo "=========================================="

    # 先更新 apt 缓存（只更一次，提升效率）
    echo "🔄 正在更新 apt 包索引缓存..."
    sudo apt update -y
    echo "✅ apt 缓存更新完成"
    echo "=========================================="

    # 遍历包列表逐个检查/安装
    local total_packages=${#PACKAGE_LIST[@]}
    local current_package_num=0
    local failed_packages=()

    for package in "${PACKAGE_LIST[@]}"; do
        current_package_num=$((current_package_num + 1))
        echo "📌 处理第 $current_package_num/$total_packages 个包：$package"

        if check_package_installed "$package"; then
            echo "✅ $package 已安装，跳过"
        else
            echo "⚠️ $package 未安装，准备安装"
            if ! install_single_package "$package"; then
                failed_packages+=("$package")  # 记录安装失败的包
            fi
        fi
        echo "------------------------------------------"
    done

    # 输出最终结果汇总
    echo "=========================================="
    echo "📊 批量处理完成！"
    echo "✅ 已安装/跳过的包数量：$((total_packages - ${#failed_packages[@]}))"
    echo "❌ 安装失败的包数量：${#failed_packages[@]}"

    if [ ${#failed_packages[@]} -gt 0 ]; then
        echo "❌ 安装失败的包列表：${failed_packages[*]}"
        return 1
    else
        echo "🎉 所有包处理成功！"
        return 0
    fi
}

# ======================== 主流程 ========================
main() {
    check_root $1
    check_ros2_installed

    # 1. 检查 conda 是否存在
    if ! check_conda_exists; then
	# 2. 不存在则安装
	install_miniconda
	# 3. 初始化 conda
	init_conda
    fi
    install_packages
    echo $passowrd | sudo -S chown -R $(id -gn):$(id -un) /opt/
}

# 执行主流程
main $1
