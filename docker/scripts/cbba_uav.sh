#!/usr/bin/env bash
# ============================================================
#  [uav 容器] 一次啟動這台無人機的 XRCE Agent、執行者、px4_state_bridge、cbba_node
#
#  用法：cbba_uav.sh [cbba_node 的額外參數…]
#    cbba_uav.sh
#    cbba_uav.sh -p lost_timeout:=2.0
#    EXEC_ARGS="-p altitude:=7.0" cbba_uav.sh      執行者的參數
#    EXECUTOR=task_executor cbba_uav.sh           用舊的代理執行者（沒有避碰、沒有火警偵測）
#
#  執行者（EXECUTOR）：
#    bt（預設）      uav_px4_bt/uav_bt_node（備用 BT：CPF 避碰、AprilTag 火警判定、電量返航）
#                   ＋ apriltag_fire_detector/fire_detector_node（要先在 sim 執行 gz_bridge.sh 送相機影像）
#    task_executor  px4_waypoint_node/task_executor
#
#  能量模型對齊 PX4 的模擬電池（px4_sitl.sh 設 SIM_BAT_DRAIN）：模擬電池依時間耗電，所以
#    懸停每秒 = 100 / SIM_BAT_DRAIN（%）、每公尺 = 懸停每秒 / CBBA_SPEED。
#  CBBA_SPEED（預設 5 m/s）、CBBA_ALTITUDE（預設 5 m）同時給 cbba_node 和執行者，
#  出價時估的飛行時間、距離才會和實際飛的一致。
#
#  順序：先在每台 uav 執行這個腳本，再到 sim 執行 px4_sitl.sh N（Agent 要先開）。
#  節點名稱加上機號（uavN_cbba_node、uavN_px4_state_bridge、uavN_bt、uavN_fire_detector 或 uavN_task_executor），
#  ros2 node list 分得出是哪一台。
#  Agent 的輸出寫到 /tmp/cbba_uavN/xrce_agent.log；其他節點的輸出加上前綴印在這個 terminal。
#  Ctrl+C 會一起停掉（停掉執行者後 PX4 會因為沒有 setpoint 觸發 failsafe）。
# ============================================================
set -uo pipefail

: "${UAV_ID:?這個腳本要在 uav 容器裡執行}"
LOG_DIR="/tmp/cbba_uav${UAV_ID}"
mkdir -p "${LOG_DIR}"

EXECUTOR="${EXECUTOR:-bt}"
case "${EXECUTOR}" in
    bt) NEED_PKGS="cbba_core px4_waypoint_node uav_px4_bt apriltag_fire_detector" ;;
    task_executor) NEED_PKGS="cbba_core px4_waypoint_node" ;;
    *) echo "[cbba_uav] EXECUTOR 只能是 bt 或 task_executor"; exit 1 ;;
esac
for pkg in ${NEED_PKGS}; do
    if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
        echo "[cbba_uav] 找不到 ${pkg}，先在 uav1 編譯："
        echo "  colcon build --symlink-install --packages-select swarm_interfaces cbba_core px4_waypoint_node uav_px4_bt apriltag_fire_detector"
        exit 1
    fi
done

PIDS=()
cleanup() {
    trap - INT TERM EXIT
    echo "[cbba_uav] 停止 uav${UAV_ID} 的節點"
    # 只 kill ros2 run 外層的話節點會繼續跑，所以依節點名稱比對節點本身
    pkill -f "__node:=uav${UAV_ID}_cbba_node( |$)" 2>/dev/null
    pkill -f "__node:=uav${UAV_ID}_px4_state_bridge( |$)" 2>/dev/null
    pkill -f "__node:=uav${UAV_ID}_task_executor( |$)" 2>/dev/null
    pkill -f "__node:=uav${UAV_ID}_bt( |$)" 2>/dev/null
    pkill -f "__node:=uav${UAV_ID}_fire_detector( |$)" 2>/dev/null
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

if [ "${EXECUTOR}" = bt ]; then
    # shellcheck disable=SC2086  # EXEC_ARGS 要拆成多個參數
    ros2 run uav_px4_bt uav_bt_node --ros-args -r __node:="uav${UAV_ID}_bt" \
        -p agent_id:="${UAV_ID}" -p cruise_speed:="${SPEED}" -p altitude:="${ALT}" ${EXEC_ARGS:-} \
        2>&1 | prefix bt &
    PIDS+=($!)
    ros2 run apriltag_fire_detector fire_detector_node --ros-args -r __node:="uav${UAV_ID}_fire_detector" \
        -p agent_id:="${UAV_ID}" 2>&1 | prefix tag &
    PIDS+=($!)
else
    # shellcheck disable=SC2086
    ros2 run px4_waypoint_node task_executor --ros-args -r __node:="uav${UAV_ID}_task_executor" \
        -p agent_id:="${UAV_ID}" -p cruise_speed:="${SPEED}" -p altitude:="${ALT}" ${EXEC_ARGS:-} \
        2>&1 | prefix exec &
    PIDS+=($!)
fi
# PX4 → robot_state：解鎖且離地才送，cbba_node 收到才參與出價
ros2 run px4_waypoint_node px4_state_bridge --ros-args -r __node:="uav${UAV_ID}_px4_state_bridge" \
    -p agent_id:="${UAV_ID}" -p px4_ns:="uav${UAV_ID}" 2>&1 | prefix state &
PIDS+=($!)
ros2 run cbba_core cbba_node --ros-args -r __node:="uav${UAV_ID}_cbba_node" \
    -p agent_id:="${UAV_ID}" -p vehicle_type:=uav -p ns:="uav${UAV_ID}" \
    -p cruise_speed:="${SPEED}" -p recon_altitude:="${ALT}" \
    -p hover_energy_per_sec:="${HOVER}" -p energy_per_meter:="${PER_M}" "$@" 2>&1 | prefix cbba &
PIDS+=($!)

echo "[cbba_uav] uav${UAV_ID} 已啟動；接著到 sim 執行 px4_sitl.sh ${UAV_ID}"
wait
