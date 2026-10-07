#!/usr/bin/env bash
# ============================================================
#  [sim 容器] 啟動第 N 台無人機的 PX4 SITL
#
#  用法：px4_sitl.sh [N] [機型]
#    px4_sitl.sh              第 1 台（同時開啟 Gazebo）
#    px4_sitl.sh 2            第 2 台（加入已開啟的 Gazebo）
#    px4_sitl.sh 1 gz_x500    指定機型
#    HEADLESS=1 px4_sitl.sh   不開 Gazebo 視窗
#
#  每台會：
#    - 用 PX4 實例編號 -i N，在 y 方向錯開出生點
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
NS="uav${N}"
BIN="${PX4_DIR}/build/px4_sitl_default/bin"
AIRFRAME_DIR="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"

# source ROS 之後，GZ_CONFIG_PATH 只指向 ROS 內建的 gz 工具（只有 topic/service…），
# 找不到 PX4 安裝的 Gazebo（gz sim）。把系統的 /usr/share/gz 放到最前面。
export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:${GZ_CONFIG_PATH}}"

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

# ---------- 出生點：沿 y 方向排開 ----------
POSE_Y="$(awk -v n="${N}" -v s="${SPACING}" 'BEGIN { printf "%.2f", (n - 1) * s }')"

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

echo "[px4_sitl] uav${N}  機型 ${MODEL} (${AUTOSTART})  出生點 y=${POSE_Y}  ${GZ_MSG}"
echo "[px4_sitl] Agent ${AGENT_IP}:${AGENT_PORT}，DDS 設定紀錄：${LOG}"
configure_dds > "${LOG}" 2>&1 &

cd "${PX4_DIR}"
exec env \
    PX4_SYS_AUTOSTART="${AUTOSTART}" \
    PX4_SIM_MODEL="${MODEL}" \
    PX4_GZ_MODEL_POSE="0,${POSE_Y}" \
    "${GZ_ENV[@]}" \
    "${BIN}/px4" -i "${N}"
