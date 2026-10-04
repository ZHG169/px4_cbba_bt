#!/usr/bin/env bash
# ============================================================
#  映像內建的 entrypoint（只是轉接）
#  真正的邏輯在掛載進來的 docker/entrypoint.sh，
#  修改那個檔不需要重新 build，重啟容器就生效
# ============================================================
REAL_ENTRYPOINT=/home/ncrl/CBBA_BT/docker/entrypoint.sh

if [ -f "${REAL_ENTRYPOINT}" ]; then
    exec bash "${REAL_ENTRYPOINT}" "$@"
fi

echo "[entrypoint] 找不到 ${REAL_ENTRYPOINT}（專案目錄沒有掛載？），以 ncrl 身分直接啟動"
if [ "$(id -u)" = "0" ]; then
    exec gosu ncrl "$@"
fi
exec "$@"
