#!/usr/bin/env bash
# ============================================================
#  [uav 或 gcs 容器] 把 px4_msgs 放進 ros2_ws/src，版本和 PX4 對齊
#  只需要執行一次；之後用 cbuild 編譯
# ============================================================
set -euo pipefail

VERSION="${PX4_VERSION:-v1.17.0}"
TARGET="${HOME}/CBBA_BT/ros2_ws/src/px4_msgs"

if [ -d "${TARGET}" ]; then
    echo "[px4_msgs] 已存在：${TARGET}"
    git -C "${TARGET}" describe --tags 2>/dev/null || true
    exit 0
fi

mkdir -p "$(dirname "${TARGET}")"
git clone --branch "${VERSION}" --depth 1 https://github.com/PX4/px4_msgs.git "${TARGET}"
echo "[px4_msgs] 已下載 ${VERSION} -> ${TARGET}"
echo "[px4_msgs] 下一步：cbuild --packages-select px4_msgs"
