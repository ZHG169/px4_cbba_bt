#!/usr/bin/env bash
# ============================================================
#  [uav 容器] 一次啟動這台無人機的 XRCE Agent、task_executor（代替 BT）、cbba_node
#
#  用法：cbba_uav.sh [cbba_node 的額外參數…]
#    cbba_uav.sh
#    cbba_uav.sh -p lost_timeout:=2.0
#    EXEC_ARGS="-p altitude:=7.0" cbba_uav.sh      task_executor 的參數
#
#  能量模型對齊 PX4 的模擬電池（px4_sitl.sh 設 SIM_BAT_DRAIN）：模擬電池依時間耗電，所以
#    懸停每秒 = 100 / SIM_BAT_DRAIN（%）、每公尺 = 懸停每秒 / CBBA_SPEED。
#  CBBA_SPEED（預設 5 m/s）、CBBA_ALTITUDE（預設 5 m）同時給 cbba_node 和 task_executor，
#  出價時估的飛行時間、距離才會和實際飛的一致。
#
#  順序：先在每台 uav 執行這個腳本，再到 sim 執行 px4_sitl.sh N（Agent 要先開）。
#  Agent 的輸出寫到 /tmp/cbba_uavN/xrce_agent.log；另外兩個節點的輸出加上前綴印在這個 terminal。
#  Ctrl+C 會一起停掉三個（停掉 task_executor 後 PX4 會因為沒有 setpoint 觸發 failsafe）。
# ============================================================
set -uo pipefail

: "${UAV_ID:?這個腳本要在 uav 容器裡執行}"
LOG_DIR="/tmp/cbba_uav${UAV_ID}"
mkdir -p "${LOG_DIR}"

if ! ros2 pkg executables uav_cbba 2>/dev/null | grep -q cbba_node ||
   ! ros2 pkg executables px4_waypoint_node 2>/dev/null | grep -q task_executor; then
    echo "[cbba_uav] 找不到 cbba_node 或 task_executor，先在 uav1 編譯："
    echo "  cbuild --packages-select swarm_interfaces uav_cbba px4_waypoint_node"
    exit 1
fi

PIDS=()
cleanup() {
    trap - INT TERM EXIT
    echo "[cbba_uav] 停止 uav${UAV_ID} 的節點"
    # 只 kill ros2 run 外層的話節點會繼續跑，所以依參數比對節點本身
    pkill -f "cbba_node.*agent_id:=${UAV_ID}" 2>/dev/null
    pkill -f "task_executor.*agent_id:=${UAV_ID}" 2>/dev/null
    kill "${PIDS[@]}" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup INT TERM EXIT

prefix() { sed -u "s/^/[$1] /"; }

if pgrep -x MicroXRCEAgent >/dev/null; then
    echo "[cbba_uav] XRCE Agent 已在執行"
else
    xrce_agent.sh > "${LOG_DIR}/xrce_agent.log" 2>&1 &
    PIDS+=($!)
    echo "[cbba_uav] XRCE Agent 啟動，紀錄：${LOG_DIR}/xrce_agent.log"
fi

DRAIN="${SIM_BAT_DRAIN:-900}"
SPEED="${CBBA_SPEED:-5.0}"
ALT="${CBBA_ALTITUDE:-5.0}"
HOVER=$(awk -v d="${DRAIN}" 'BEGIN { printf "%.4f", 100.0 / d }')
PER_M=$(awk -v d="${DRAIN}" -v v="${SPEED}" 'BEGIN { printf "%.4f", 100.0 / d / v }')
echo "[cbba_uav] 能量模型：懸停 ${HOVER} %/s、每公尺 ${PER_M} %（續航 ${DRAIN} s、${SPEED} m/s）；巡檢高度 ${ALT} m"

# shellcheck disable=SC2086  # EXEC_ARGS 要拆成多個參數
ros2 run px4_waypoint_node task_executor --ros-args -p agent_id:="${UAV_ID}" \
    -p cruise_speed:="${SPEED}" -p altitude:="${ALT}" ${EXEC_ARGS:-} 2>&1 | prefix exec &
PIDS+=($!)
ros2 run uav_cbba cbba_node --ros-args -p agent_id:="${UAV_ID}" \
    -p cruise_speed:="${SPEED}" -p recon_altitude:="${ALT}" \
    -p hover_energy_per_sec:="${HOVER}" -p energy_per_meter:="${PER_M}" "$@" 2>&1 | prefix cbba &
PIDS+=($!)

echo "[cbba_uav] uav${UAV_ID} 已啟動；接著到 sim 執行 px4_sitl.sh ${UAV_ID}"
wait
