#!/bin/bash

source ~/miniconda3/etc/profile.d/conda.sh

# ===================== 配置区 =====================
CONDA_ENV_NAME="motion_studio"       # 要检查/创建的conda环境名
PYTHON_VERSION="3.13.11"    # 指定的Python版本
# ==================================================

# 函数：检查conda环境是否存在
check_conda_env_exists() {
    local env_name="$1"
    # conda info --envs 列出所有环境，过滤目标环境（排除注释行）
    if conda info --envs | grep -E "^$env_name\s+" &> /dev/null; then
        return 0  # 环境存在
    else
        return 1  # 环境不存在
    fi
}

# 函数：创建conda环境
create_conda_env() {
    local env_name="$1"
    local py_version="$2"

    echo "⚠️ 注意：Python $py_version 并非主流稳定版本，可能存在兼容性问题"
    echo "📦 开始创建conda环境：$env_name (Python $py_version)..."
    conda init
    echo "✅ conda环境 $env_name 创建成功！"
    # 创建环境，-y 自动确认所有提示
    if conda create -n "$env_name" python="$py_version"; then
        # 验证环境是否创建成功
        if check_conda_env_exists "$env_name"; then
            echo "✅ conda环境 $env_name 创建成功！"
        else
            echo "❌ conda环境 $env_name 创建失败（命令执行成功但检测不到环境）"
            return 1
        fi
    else
        echo "❌ conda环境 $env_name 创建命令执行失败，请检查Python版本是否合法"
        return 1
    fi
}

# 主逻辑
main() {
    echo "🔍 检查conda环境 $CONDA_ENV_NAME 是否存在..."
    if check_conda_env_exists "$CONDA_ENV_NAME"; then
        echo "✅ conda环境 $CONDA_ENV_NAME 已存在，跳过创建步骤"
    else
        echo "⚠️ conda环境 $CONDA_ENV_NAME 不存在，准备创建"
        create_conda_env "$CONDA_ENV_NAME" "$PYTHON_VERSION"
    fi

    echo "🎉 脚本执行完成！"
    echo "💡 激活环境命令：conda activate $CONDA_ENV_NAME"
}

# 执行主函数
main
conda activate ${CONDA_ENV_NAME}
pip install -r requirements.txt
