#!/usr/bin/env bash
# ============================================================
#  停止並移除所有容器（映像和 ros2_ws 的檔案都會保留）
#  只想暫停、保留容器：docker compose -f docker-compose.yaml stop
# ============================================================
set -euo pipefail

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${DOCKER_DIR}"

docker compose -f docker-compose.yaml down --remove-orphans
