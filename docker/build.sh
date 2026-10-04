#!/usr/bin/env bash
# ============================================================
#  建置所有映像：cbba-base → cbba-sim / cbba-robot / cbba-gcs
#  用法：./build.sh            全部
#        ./build.sh sim        只重建某個服務（base 仍會先檢查，沒改就用快取）
# ============================================================
set -euo pipefail

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${DOCKER_DIR}/.." && pwd)"
cd "${DOCKER_DIR}"

echo "======================================"
echo " CBBA_BT Docker Build"
echo " Project : ${PROJECT_DIR}"
echo "======================================"

python3 "${DOCKER_DIR}/gen_compose.py"

echo "[build] 1/2 cbba-base:jazzy"
docker build \
    -f "${DOCKER_DIR}/base.Dockerfile" \
    -t cbba-base:jazzy \
    "${PROJECT_DIR}"

echo "[build] 2/2 sim / uav / gcs"
# uav 的映像只掛在 uav1 上 build，其他 uavN 共用同一個映像
ARGS=("$@")
for i in "${!ARGS[@]}"; do
    [ "${ARGS[$i]}" = "uav" ] && ARGS[$i]="uav1"
done
docker compose -f docker-compose.yaml build "${ARGS[@]}"

echo "[build] 完成。下一步：./start.sh"
