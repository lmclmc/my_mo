#!/bin/bash
# 配置
GIT_URL="https://bot:glpat-0vRpfOUvzAj9NjHGZxClHG86MQp1Ojd3CA.01.0y08ziqgv@gitlab.magiclab.top/dreamebiped/controlsoftware/motionstudio.git"

# 脚本所在目录
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
REPO_PATH="${SCRIPT_DIR}"

# 获取远程最大语义化Tag（格式 V数字.数字.数字）
get_latest_tag() {
    # 列出所有远程tag，过滤 V开头，按版本号降序，取第一条
    git ls-remote --tags "${GIT_URL}" \
    | awk '{print $2}' \
    | sed 's|refs/tags/||' \
    | grep -E '^V[0-9]+\.[0-9]+\.[0-9]+$' \
    | awk '{raw=$0; sub(/^V/,""); print $0}' \
    | sort -t '.' -k1,1nr -k2,2nr -k3,3nr \
    | head -n1
}

LATEST_TAG="V$(get_latest_tag)"

if [ -z "${LATEST_TAG}" ]; then
    echo "错误：未在仓库找到任何 Vx.y.z 格式的Tag"
    exit 1
fi
echo "检测到最新版本Tag：${LATEST_TAG}"

if git remote -v &> /dev/null; then
    echo "仓库已存在，拉取远程标签并切换至最新Tag ${LATEST_TAG}"
    cd "${REPO_PATH}"
    git fetch --prune --tags
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