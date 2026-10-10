#!/usr/bin/env bash
# ============================================================
#  CBBA_BT entrypoint（以 root 執行，最後切換成 ncrl）
#   1. 把 ncrl 的 UID/GID 改成和 host 一樣  → 掛載目錄不會 Permission denied
#   2. 由 LOCAL_IPS / PEER_IPS 把 fastdds/profile.xml.template 轉成 /tmp/cbba_fastdds.xml
#   3. 用 ncrl 身分執行 CMD
# ============================================================
set -euo pipefail

USERNAME=ncrl
HOME_DIR=/home/${USERNAME}
PROJECT_DIR=${HOME_DIR}/CBBA_BT
ROLE="${CBBA_ROLE:-unknown}"

log() { echo "[entrypoint:${ROLE}] $*"; }

# ---------- 1. UID / GID 對應 ----------
if [ "$(id -u)" = "0" ]; then
    HOST_UID="${HOST_UID:-1000}"
    HOST_GID="${HOST_GID:-1000}"
    CUR_UID="$(id -u "${USERNAME}")"
    CUR_GID="$(id -g "${USERNAME}")"

    if [ "${CUR_GID}" != "${HOST_GID}" ]; then
        groupmod -o -g "${HOST_GID}" "${USERNAME}"
    fi
    if [ "${CUR_UID}" != "${HOST_UID}" ]; then
        usermod -o -u "${HOST_UID}" -g "${HOST_GID}" "${USERNAME}"
    fi
    if [ "${CUR_UID}" != "${HOST_UID}" ] || [ "${CUR_GID}" != "${HOST_GID}" ]; then
        log "ncrl UID:GID ${CUR_UID}:${CUR_GID} -> ${HOST_UID}:${HOST_GID}"
        # 只改家目錄，跳過 host 掛載進來的 CBBA_BT
        find "${HOME_DIR}" -xdev -path "${PROJECT_DIR}" -prune -o \
             -exec chown -h "${HOST_UID}:${HOST_GID}" {} +
    fi
    # cbba_node 的程序鎖目錄（主機的 /tmp/cbba_locks，所有容器共用；docker 建立時是 root 的）
    if [ -n "${CBBA_LOCK_DIR:-}" ]; then
        mkdir -p "${CBBA_LOCK_DIR}"
        chown "${HOST_UID}:${HOST_GID}" "${CBBA_LOCK_DIR}"
        chmod 1777 "${CBBA_LOCK_DIR}"
    fi
fi

# ---------- 2. Fast DDS 設定 ----------
# 由 LOCAL_IPS（自己的網卡）和 PEER_IPS（要探索的對象）產生 XML
TEMPLATE="${PROJECT_DIR}/docker/fastdds/profile.xml.template"
OUTPUT="${FASTRTPS_DEFAULT_PROFILES_FILE:-/tmp/cbba_fastdds.xml}"
LOCAL_IPS="${LOCAL_IPS:-}"
PEER_IPS="${PEER_IPS:-}"

if [ -f "${TEMPLATE}" ] && [ -n "${LOCAL_IPS}" ]; then
    CBBA_WHITELIST=""
    for ip in ${LOCAL_IPS}; do
        CBBA_WHITELIST+="          <address>${ip}</address>"$'\n'
    done
    CBBA_PEERS=""
    for ip in ${LOCAL_IPS} ${PEER_IPS}; do
        CBBA_PEERS+="            <locator><udpv4><address>${ip}</address></udpv4></locator>"$'\n'
    done
    export CBBA_WHITELIST CBBA_PEERS

    envsubst '${CBBA_WHITELIST} ${CBBA_PEERS}' < "${TEMPLATE}" > "${OUTPUT}"
    chmod 0644 "${OUTPUT}"

    if xmllint --noout "${OUTPUT}" 2>/dev/null; then
        log "Fast DDS: 網卡 [${LOCAL_IPS}]  探索 [${PEER_IPS}]"
    else
        log "警告：${OUTPUT} 不是合法的 XML，請檢查 fastdds/profile.xml.template"
    fi
else
    # 沒有設定時寫一份空的設定檔，讓 Fast DDS 用預設值（環境變數仍指向合法檔案）
    log "沒有 LOCAL_IPS 或範本，Fast DDS 使用預設設定"
    printf '<?xml version="1.0" encoding="UTF-8"?>\n<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"><profiles/></dds>\n' \
        > "${OUTPUT}"
    chmod 0644 "${OUTPUT}"
fi

# ---------- 3. 切換成 ncrl ----------
if [ "$(id -u)" = "0" ]; then
    exec gosu "${USERNAME}" "$@"
fi
exec "$@"
