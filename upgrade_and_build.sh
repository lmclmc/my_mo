#!/bin/bash

# 检查参数
if [ -z "$1" ]; then
    echo "用法: $0 <gitlab_token>"
    echo "示例: $0 glpat-xxxXXxxxXXxxXxxXXXxXXX"
    exit 1
fi

# 获取 token 参数
GIT_TOKEN="$1"

# 配置 - 将 token 拼接到 gitlab URL
GIT_URL="https://bot:${GIT_TOKEN}@gitlab.magiclab.top/dreamebiped/controlsoftware/motionstudio.git"

# 脚本所在目录
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_PATH="${SCRIPT_DIR}"

# 获取远程最大语义化Tag（格式 V数字.数字.数字）
get_latest_tag() {
    # 列出所有远程tag，过滤 V开头，按版本号降序，取第一条
    # 使用临时变量存储输出和错误
    local output
    local exit_code
    
    output=$(git ls-remote --tags "${GIT_URL}" 2>&1)
    exit_code=$?
    
    # 检查 git ls-remote 是否成功
    if [ $exit_code -ne 0 ]; then
        echo "ERROR:git_ls_remote_failed"
        echo "错误：git ls-remote 执行失败" >&2
        echo "输出信息: ${output}" >&2
        # 检测是否为认证失败
        if echo "${output}" | grep -q -i "authentication\|fatal\|denied\|token"; then
            echo "可能的原因：GitLab token 无效或权限不足，请检查您的 token" >&2
        fi
        return 1
    fi

    git ls-remote --tags "${GIT_URL}" \
    | awk '{print $2}' \
    | sed 's|refs/tags/||' \
    | grep -E '^V[0-9]+\.[0-9]+\.[0-9]+$' \
    | awk '{raw=$0; sub(/^V/,""); print $0}' \
    | sort -t '.' -k1,1nr -k2,2nr -k3,3nr \
    | head -n1
}

LATEST_TAG="V$(get_latest_tag)"

# 检查是否发生错误
if echo "${LATEST_TAG}" | grep -q "ERROR:"; then
    exit 1
fi
 
if [ -z "${LATEST_TAG}" ]; then
    echo "错误：未在仓库找到任何 Vx.y.z 格式的Tag"
    exit 1
fi
echo "检测到最新版本Tag：${LATEST_TAG}"

if git remote -v &> /dev/null; then
    echo "仓库已存在，拉取远程标签并切换至最新Tag ${LATEST_TAG}"
    cd "${REPO_PATH}"
    git fetch --prune --tags -f
    git checkout "${LATEST_TAG}"
else
    echo "初始化并拉取源码至当前目录"
    git init
    git remote add origin "${GIT_URL}"
    git fetch --tags
    git checkout "${LATEST_TAG}"
fi

echo "已更新至最新版本 ${LATEST_TAG}，开始构建 Motion Studio......................"

./build_motion_studio.sh 1
