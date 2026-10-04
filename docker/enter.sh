#!/usr/bin/env bash
# ============================================================
#  進入容器（以 ncrl 身分）
#  用法：./enter.sh sim | uav1 | uav2 | ... | gcs
#        ./enter.sh uav   等同 uav1
# ============================================================
set -euo pipefail

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${DOCKER_DIR}"

SERVICE="${1:-uav1}"
[ "${SERVICE}" = "uav" ] && SERVICE="uav1"

SERVICES="$(docker compose -f docker-compose.yaml config --services)"
if ! grep -qx "${SERVICE}" <<< "${SERVICES}"; then
    echo "沒有 ${SERVICE}。目前的容器：$(tr '\n' ' ' <<< "${SERVICES}")"
    exit 1
fi

exec docker compose -f docker-compose.yaml exec -u ncrl "${SERVICE}" bash
