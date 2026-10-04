#!/usr/bin/env bash
# ============================================================
#  [uav 容器] 啟動 Micro-XRCE-DDS-Agent，等 sim 容器裡的 PX4 連過來
#  實機時把 udp4 換成：MicroXRCEAgent serial --dev /dev/ttyXXX -b 921600
# ============================================================
set -euo pipefail

PORT="${XRCE_AGENT_PORT:-8888}"
echo "[xrce_agent] 監聽 UDP ${PORT}（domain ${ROS_DOMAIN_ID:-42}）"
exec MicroXRCEAgent udp4 -p "${PORT}" "$@"
