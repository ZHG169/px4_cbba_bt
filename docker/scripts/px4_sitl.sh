#!/usr/bin/env bash
# ============================================================
#  [sim 容器] 啟動第 N 台無人機的 PX4 SITL
#
#  用法：px4_sitl.sh [N] [機型]
#    px4_sitl.sh              第 1 台（同時開啟 Gazebo）
#    px4_sitl.sh 2            第 2 台（加入已開啟的 Gazebo）
#    px4_sitl.sh 1 gz_x500    指定機型
#    HEADLESS=1 px4_sitl.sh   不開 Gazebo 視窗
#    CBBA_WORLD=fire_site px4_sitl.sh 1   用 docker/gz/worlds 的場景（第 1 台負責開；之後的台不用設，
#                                         自動加入正在跑的場景）
#
#    UAV_SPAWN_ORIGIN=-40,-30 CBBA_WORLD=yard px4_sitl.sh 1
#                                         出生原點（map ENU，預設 0,0）；uav 容器的 cbba_uav.sh 要設同一個值。
#                                         yard 的 (0, 0) 是變壓器，起降板在 (−40, −30)，沒設對會拒絕啟動
#
#  每台會：
#    - 用 PX4 實例編號 -i N，出生點 = 出生原點 ＋ (0, (N − 1) × UAV_SPAWN_SPACING)
#    - 把 uXRCE-DDS client 指向 uavN 容器的 Agent（domain 42）
#    - topic 加上命名空間：/uavN/fmu/...
#  多台時每台開一個 terminal（或用 tmux 分割），先開第 1 台再開第 2 台
# ============================================================
set -euo pipefail

: "${PX4_DIR:?找不到 PX4_DIR，這個腳本要在 sim 容器裡執行}"
: "${UAV_AGENT_IPS:?找不到 UAV_AGENT_IPS，請用 ./start.sh 重新啟動容器}"

N="${1:-1}"
MODEL="${2:-${PX4_SIM_MODEL:-gz_x500}}"
AGENT_PORT="${XRCE_AGENT_PORT:-8888}"
DOMAIN="${ROS_DOMAIN_ID:-42}"
SPACING="${UAV_SPAWN_SPACING:-2.0}"
IFS=, read -r ORIGIN_X ORIGIN_Y <<< "${UAV_SPAWN_ORIGIN:-0,0}"
NS="uav${N}"
BIN="${PX4_DIR}/build/px4_sitl_default/bin"
AIRFRAME_DIR="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"

# source ROS 之後，GZ_CONFIG_PATH 只指向 ROS 內建的 gz 工具（只有 topic/service…），
# 找不到 PX4 安裝的 Gazebo（gz sim）。把系統的 /usr/share/gz 放到最前面。
export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:${GZ_CONFIG_PATH}}"

# 專案的模型（AprilTag、低解析度的 mono_cam）放在最前面：PX4 的 gz_env.sh 只會往後加自己的路徑，
# 同名的模型（mono_cam）用專案的版本
CBBA_GZ_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../gz" && pwd)"
export GZ_SIM_RESOURCE_PATH="${CBBA_GZ_DIR}/models:${CBBA_GZ_DIR}/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

# ---------- 第 N 台的 Agent IP ----------
read -r -a AGENTS <<< "${UAV_AGENT_IPS}"
if ! [[ "${N}" =~ ^[0-9]+$ ]] || [ "${N}" -lt 1 ] || [ "${N}" -gt "${#AGENTS[@]}" ]; then
    echo "[px4_sitl] N 必須在 1~${#AGENTS[@]} 之間（目前 .env 的 UAV_COUNT=${#AGENTS[@]}）"
    exit 1
fi
AGENT_IP="${AGENTS[$((N - 1))]}"

# ---------- 機型 -> 機架編號（例如 4001_gz_x500 -> 4001）----------
AIRFRAME="$(find "${AIRFRAME_DIR}" -maxdepth 1 -name "*_${MODEL}" -printf '%f\n' | grep -E "^[0-9]+_${MODEL}$" | head -n 1 || true)"
if [ -z "${AIRFRAME}" ]; then
    echo "[px4_sitl] 找不到機型 ${MODEL}。可用的 gz 機型："
    find "${AIRFRAME_DIR}" -maxdepth 1 -name '*_gz_*' -printf '  %f\n' | sort
    exit 1
fi
AUTOSTART="${AIRFRAME%%_*}"

# ---------- 出生點：出生原點起沿 y 方向排開（2026-10-24 加出生原點）----------
POSE_X="$(awk -v x="${ORIGIN_X}" 'BEGIN { printf "%.2f", x }')"
POSE_Y="$(awk -v y="${ORIGIN_Y}" -v n="${N}" -v s="${SPACING}" 'BEGIN { printf "%.2f", y + (n - 1) * s }')"

# ---------- Gazebo：第一個啟動的負責開，之後的加入 ----------
GZ_ENV=()
# 用 "gz[ ]sim" 而不是 "gz sim"：其他 shell 的指令文字裡若含有 gz sim（例如等待 Gazebo 的迴圈），
# 用 "gz sim" 會誤判成 Gazebo 已經在跑；中括號寫法不會比對到自己的文字
if pgrep -f "gz[ ]sim" >/dev/null 2>&1; then
    GZ_ENV+=(PX4_GZ_STANDALONE=1)
    GZ_MSG="加入已開啟的 Gazebo"
else
    GZ_MSG="開啟 Gazebo"
    if [ "${N}" != "1" ]; then
        echo "[px4_sitl] 提醒：Gazebo 還沒開，由第 ${N} 台負責開啟"
    fi
fi

# ---------- Gazebo 已經在跑：加入正在跑的世界 ----------
# PX4 要知道世界名稱（PX4_GZ_WORLD），沒給時會一直等 default 世界。第 2 台之後沒設 CBBA_WORLD 也要加入
# 第 1 台開的場景，所以從 Gazebo 讀出世界名稱（2026-10-11 SITL 測試發現：fire_site 的第 2、3 台起不來）
if pgrep -f "gz[ ]sim" >/dev/null 2>&1; then
    RUNNING_WORLD=""
    for _ in $(seq 1 30); do
        RUNNING_WORLD="$(gz topic -l 2>/dev/null | grep -m 1 -E '^/world/[^/]+/clock$' \
            | sed -E 's#^/world/([^/]+)/clock$#\1#' || true)"
        [ -n "${RUNNING_WORLD}" ] && break
        sleep 1
    done
    if [ -z "${RUNNING_WORLD}" ]; then
        echo "[px4_sitl] Gazebo 在跑，但 30 s 內讀不到世界名稱（gz topic -l 沒有 /world/<名稱>/clock）"
        echo "[px4_sitl] 多半是上一次殘留、已經卡住的 Gazebo（不理一般的 pkill），而且這種情況不會開視窗。先強制關掉再重開："
        pgrep -af "gz[ ]sim" | sed 's/^/[px4_sitl]   /'
        echo "[px4_sitl]   pkill -9 -f \"gz sim\"     （還在的話在主機 docker restart cbba_sim）"
        exit 1
    fi
    if [ -n "${CBBA_WORLD:-}" ] && [ "${CBBA_WORLD}" != "${RUNNING_WORLD}" ]; then
        echo "[px4_sitl] 提醒：正在跑的是 ${RUNNING_WORLD}，不是 CBBA_WORLD=${CBBA_WORLD}；加入 ${RUNNING_WORLD}（要換場景先關掉 Gazebo）"
    fi
    CBBA_WORLD="${RUNNING_WORLD}"
    GZ_MSG="加入已開啟的場景 ${RUNNING_WORLD}"
fi

# ---------- 專案的場景：自己開 Gazebo，PX4 會偵測到「已經在跑的世界」直接加入 ----------
# （PX4 的 PX4_GZ_WORLD 只能選 PX4 自己 worlds 目錄裡的場景）
if [ -n "${CBBA_WORLD:-}" ] && ! pgrep -f "gz[ ]sim" >/dev/null 2>&1; then
    WORLD_FILE="${CBBA_GZ_DIR}/worlds/${CBBA_WORLD}.sdf"
    [ -f "${WORLD_FILE}" ] || { echo "[px4_sitl] 找不到場景 ${WORLD_FILE}"; exit 1; }
    # 和 PX4 的 gz_env.sh 相同的外掛、伺服器設定
    ROOTFS="${PX4_DIR}/build/px4_sitl_default/rootfs"
    # shellcheck disable=SC1091
    set +u; . "${ROOTFS}/gz_env.sh"; set -u     # gz_env.sh 會讀還沒設定的變數
    echo "[px4_sitl] 開啟場景 ${WORLD_FILE}"
    gz sim --verbose=1 -r -s "${WORLD_FILE}" > /tmp/gz_server.log 2>&1 &
    if [ -z "${HEADLESS:-}" ]; then
        # 專案的視窗設定（docker/gz/gui/cbba_gui.config）：地面網格、俯瞰鏡頭（依場景，西南方往東北看）；
        # 錄影用場景裡的俯瞰攝影機（overview_camera，gen_patrol_site.py）
        case "${CBBA_WORLD}" in
            patrol_site) CAMERA_POSE="-60 -55 55 0 0.57 0.76" ;;
            demo_site)   CAMERA_POSE="-22 -24 20 0 0.52 0.78" ;;
            fire_site)   CAMERA_POSE="-28 -30 28 0 0.61 0.77" ;;
            yard)        CAMERA_POSE="-75 -70 60 0 0.52 0.76" ;;
            *)           CAMERA_POSE="-30 -30 30 0 0.6 0.785" ;;
        esac
        GUI_CONFIG="/tmp/cbba_gui_${CBBA_WORLD}.config"
        sed "s|@CAMERA_POSE@|${CAMERA_POSE}|" "${CBBA_GZ_DIR}/gui/cbba_gui.config" > "${GUI_CONFIG}"
        gz sim -g --gui-config "${GUI_CONFIG}" > /tmp/gz_gui.log 2>&1 &
    fi
    for _ in $(seq 1 60); do
        gz topic -l 2>/dev/null | grep -q "^/world/${CBBA_WORLD}/clock" && break
        sleep 1
    done
    GZ_ENV=()            # 世界已經在跑：PX4 走「gazebo already running world」，自己生成機體
    GZ_MSG="加入場景 ${CBBA_WORLD}"
fi

# ---------- 場景要求的出生原點（原點上有障礙物的場景）----------
# uav 容器的節點用同一個 UAV_SPAWN_ORIGIN 換算座標，這裡沒辦法替它改，所以不合就停下來（2026-10-24）
case "${CBBA_WORLD:-}" in
    yard) NEED_ORIGIN="-40,-30" ;;     # 起降板 uav_pad；(0, 0) 是變壓器 TB4
    *)    NEED_ORIGIN="" ;;
esac
if [ -n "${NEED_ORIGIN}" ] && [ "${UAV_SPAWN_ORIGIN:-0,0}" != "${NEED_ORIGIN}" ]; then
    echo "[px4_sitl] 場景 ${CBBA_WORLD} 的出生原點要是 ${NEED_ORIGIN}（目前 ${UAV_SPAWN_ORIGIN:-0,0}）。sim 和每台 uav 都要設："
    echo "[px4_sitl]   sim：UAV_SPAWN_ORIGIN=${NEED_ORIGIN} CBBA_WORLD=${CBBA_WORLD} px4_sitl.sh ${N}"
    echo "[px4_sitl]   uav：UAV_SPAWN_ORIGIN=${NEED_ORIGIN} CBBA_ALTITUDE=8 cbba_uav.sh"
    exit 1
fi

# ---------- 背景：等 PX4 開好後，把 DDS client 指向 uavN ----------
LOG="/tmp/px4_dds_config_${N}.log"
configure_dds() {
    for _ in $(seq 1 120); do
        if "${BIN}/px4-uxrce_dds_client" --instance "${N}" status >/dev/null 2>&1; then
            break
        fi
        sleep 1
    done
    sleep 2
    "${BIN}/px4-param" --instance "${N}" set UXRCE_DDS_DOM_ID "${DOMAIN}"

    # 純模擬用：沒有 QGC、沒有遙控器，關掉會干擾測試的 failsafe
    # ⚠ 只適用於模擬，實機必須保留這些安全檢查
    "${BIN}/px4-param" --instance "${N}" set NAV_DLL_ACT 0      # 失去地面站連線時不觸發 failsafe
    "${BIN}/px4-param" --instance "${N}" set COM_RC_IN_MODE 4   # 不使用遙控器
    "${BIN}/px4-param" --instance "${N}" set COM_RCL_EXCEPT 4   # offboard 模式下失去遙控器訊號不觸發 failsafe
    echo "[px4_sitl] uav${N}: 已設定模擬用參數 NAV_DLL_ACT=0 COM_RC_IN_MODE=4 COM_RCL_EXCEPT=4"

    # 模擬電池：PX4 預設解鎖後 60 s 就從 100% 降到底、停在 50%，和 cbba 的能量模型對不上。
    # 改成續航 SIM_BAT_DRAIN 秒（解鎖後依時間線性下降，和飛多遠無關），最低停在 SIM_BAT_MIN_PCT
    # （= cbba 的安全存量，不會觸發 PX4 的低電量 failsafe）。cbba_uav.sh 用同一個續航算能量模型。
    "${BIN}/px4-param" --instance "${N}" set SIM_BAT_DRAIN "${SIM_BAT_DRAIN:-900}"
    "${BIN}/px4-param" --instance "${N}" set SIM_BAT_MIN_PCT "${SIM_BAT_MIN_PCT:-20}"
    echo "[px4_sitl] uav${N}: 模擬電池續航 ${SIM_BAT_DRAIN:-900} s，最低 ${SIM_BAT_MIN_PCT:-20}%"

    "${BIN}/px4-uxrce_dds_client" --instance "${N}" stop || true
    sleep 1
    "${BIN}/px4-uxrce_dds_client" --instance "${N}" start \
        -t udp -h "${AGENT_IP}" -p "${AGENT_PORT}" -n "${NS}"
    echo "[px4_sitl] uav${N}: uXRCE-DDS client -> ${AGENT_IP}:${AGENT_PORT}  domain ${DOMAIN}  namespace /${NS}"
}

echo "[px4_sitl] uav${N}  機型 ${MODEL} (${AUTOSTART})  出生點 (${POSE_X}, ${POSE_Y})  ${GZ_MSG}"
echo "[px4_sitl] Agent ${AGENT_IP}:${AGENT_PORT}，DDS 設定紀錄：${LOG}"
configure_dds > "${LOG}" 2>&1 &

cd "${PX4_DIR}"
exec env \
    PX4_SYS_AUTOSTART="${AUTOSTART}" \
    PX4_SIM_MODEL="${MODEL}" \
    PX4_GZ_MODEL_POSE="${POSE_X},${POSE_Y}" \
    "${GZ_ENV[@]}" \
    ${CBBA_WORLD:+PX4_GZ_WORLD="${CBBA_WORLD}"} \
    "${BIN}/px4" -i "${N}"
