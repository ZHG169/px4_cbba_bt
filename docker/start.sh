#!/usr/bin/env bash
# ============================================================
#  依 .env 產生 docker-compose.yaml，並啟動 sim / uav1..N / gcs
#  用法：./start.sh           有 NVIDIA 顯示卡時自動給 sim 用 GPU
#        NO_GPU=1 ./start.sh  強制不用 GPU
# ============================================================
set -euo pipefail

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${DOCKER_DIR}"

# 執行時 UID 對應：容器裡的 ncrl 會變成和你同一個 UID
export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"

python3 "${DOCKER_DIR}/gen_compose.py"

COMPOSE_FILES=(-f docker-compose.yaml)
if [ "${NO_GPU:-0}" != "1" ] && command -v nvidia-smi >/dev/null 2>&1; then
    COMPOSE_FILES+=(-f docker-compose.gpu.yaml)
    GPU_MSG="NVIDIA GPU"
else
    GPU_MSG="不使用 GPU（軟體渲染）"
fi

echo "======================================"
echo " CBBA_BT Docker Start"
echo " User : $(id -un) (${HOST_UID}:${HOST_GID})"
echo " GPU  : ${GPU_MSG}"
echo "======================================"

# 只允許「你自己這個使用者」連到 X server（比 xhost +local: 安全）
if command -v xhost >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    xhost +SI:localuser:"$(id -un)" >/dev/null
fi

# --remove-orphans：UAV_COUNT 變少時，把多出來的 uavN 容器移除
docker compose "${COMPOSE_FILES[@]}" up -d --remove-orphans
docker compose "${COMPOSE_FILES[@]}" ps

SERVICES="$(docker compose -f docker-compose.yaml config --services | tr '\n' ' ')"
cat <<EOF

容器：${SERVICES}
進入容器（每個終端機開一個）：
  ./enter.sh sim     # 真實世界 + 飛控（所有無人機的 PX4 都在這裡）
  ./enter.sh uav1    # 第 1 台無人機的樹莓派（uav2、uav3… 依此類推）
  ./enter.sh gcs     # 地面站
停止：./stop.sh
EOF
