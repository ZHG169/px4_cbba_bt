#!/usr/bin/env bash
# ============================================================
#  [uav 容器] 一次啟動這台無人機的 XRCE Agent、執行者、px4_state_bridge、cbba_node
#
#  用法：cbba_uav.sh [cbba_node 的額外參數…]
#    cbba_uav.sh
#    cbba_uav.sh -p lost_timeout:=2.0
#    EXEC_ARGS="-p cbf.safe_radius:=1.5" cbba_uav.sh   執行者的參數（臨時改；常改的改 uav_px4_bt/config/uav_bt.yaml）
#    EXECUTOR=bt cbba_uav.sh                      用備用 BT（uav_bt_node）
#    EXECUTOR=task_executor cbba_uav.sh           用舊的代理執行者（沒有避碰、沒有火警偵測）
#
#  執行者（EXECUTOR）：
#    task_manager（預設，2026-10-17 起）
#                   uav_px4_bt/uav_task_manager_node（正式 BT：伯宇的 task_manager 骨架＋NavigateTo／AirRecon；
#                   樹 trees/uav_mission.xml；mode 由它決定，bridge 照抄）
#                   ＋ apriltag_fire_detector/fire_detector_node（要先在 sim 執行 gz_bridge.sh 送相機影像）
#    bt             uav_px4_bt/uav_bt_node（備用 BT：CBF 避碰、AprilTag 火警判定、電量返航；mode 由 bridge 自己判斷）
#                   ＋ fire_detector_node；參數和 task_manager 相同
#    task_executor  px4_waypoint_node/task_executor
#
#  能量模型對齊 PX4 的模擬電池（px4_sitl.sh 設 SIM_BAT_DRAIN）：模擬電池依時間耗電，所以
#    懸停每秒 = 100 / SIM_BAT_DRAIN（%）、每公尺 = 懸停每秒 / CBBA_SPEED。
#  CBBA_START_GATE=<檔案>：cbba_node 等這個檔案出現才啟動（其他節點照常，飛機照常起飛）；itest demo 用來讓全隊同時開始。
#  CBBA_TAKEOFF_GATE=<檔案>：BT 等這個檔案出現才啟動（才解鎖起飛）；itest demo 用來讓全隊同時起飛。
#  CBBA_SPEED（預設 0.5 m/s，2026-10-24 從 5 改）、CBBA_ALTITUDE（預設 5 m；yard 場景用 8 m）同時給 cbba_node 和執行者，
#  出價時估的飛行時間、距離才會和實際飛的一致。
#  出生點（map ENU）＝ UAV_SPAWN_ORIGIN（"x,y"，預設 "0,0"）＋ (0, (UAV_ID − 1) × UAV_SPAWN_SPACING)，用 spawn_enu 參數
#  給 BT、火警偵測、bridge、task_executor（2026-10-24，yard 場景的起降板在 (−40, −30)）；要和 sim 的 px4_sitl.sh 用同一個值。
#
#  順序：先在每台 uav 執行這個腳本，再到 sim 執行 px4_sitl.sh N（Agent 要先開）。
#  節點名稱加上機號（uavN_cbba_node、uavN_px4_state_bridge、uavN_bt 或 uavN_task_manager、uavN_fire_detector
#  或 uavN_task_executor），
#  ros2 node list 分得出是哪一台。
#  Agent 的輸出寫到 /tmp/cbba_uavN/xrce_agent.log；其他節點的輸出加上前綴印在這個 terminal。
#  Ctrl+C 會一起停掉（停掉執行者後 PX4 會因為沒有 setpoint 觸發 failsafe）。
# ============================================================
set -uo pipefail

: "${UAV_ID:?這個腳本要在 uav 容器裡執行}"
LOG_DIR="/tmp/cbba_uav${UAV_ID}"
mkdir -p "${LOG_DIR}"

EXECUTOR="${EXECUTOR:-task_manager}"   # 2026-10-17 起預設正式 BT（原本 bt）
case "${EXECUTOR}" in
    bt|task_manager) NEED_PKGS="swarm_cbba px4_waypoint_node uav_px4_bt apriltag_fire_detector" ;;
    task_executor) NEED_PKGS="swarm_cbba px4_waypoint_node" ;;
    *) echo "[cbba_uav] EXECUTOR 只能是 bt、task_manager 或 task_executor"; exit 1 ;;
esac
for pkg in ${NEED_PKGS}; do
    if ! ros2 pkg prefix "${pkg}" >/dev/null 2>&1; then
        echo "[cbba_uav] 找不到 ${pkg}，先在 uav1 編譯："
        echo "  colcon build --symlink-install --packages-select swarm_interfaces cbba_core swarm_cbba px4_waypoint_node uav_px4_bt apriltag_fire_detector"
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
    pkill -f "__node:=uav${UAV_ID}_task_manager( |$)" 2>/dev/null
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
# 一律寫成小數：ROS 的 double 參數收到整數（CBBA_ALTITUDE=8）會拒絕啟動（2026-10-24 yard 第一次跑，BT 起不來）
SPEED="$(awk -v v="${CBBA_SPEED:-0.5}" 'BEGIN { printf "%.3f", v }')"
ALT="$(awk -v v="${CBBA_ALTITUDE:-5.0}" 'BEGIN { printf "%.3f", v }')"
# 出生點：和 px4_sitl.sh 的 PX4_GZ_MODEL_POSE 一致（PX4 local 以出生點為原點，節點靠 spawn_enu 換成 map）
IFS=, read -r ORIGIN_X ORIGIN_Y <<< "${UAV_SPAWN_ORIGIN:-0,0}"
SPAWN="$(awk -v x="${ORIGIN_X}" -v y="${ORIGIN_Y}" -v n="${UAV_ID}" -v s="${UAV_SPAWN_SPACING:-2.0}" \
    'BEGIN { printf "[%.2f,%.2f,0.0]", x, y + (n - 1) * s }')"
HOVER=$(awk -v d="${DRAIN}" 'BEGIN { printf "%.4f", 100.0 / d }')
PER_M=$(awk -v d="${DRAIN}" -v v="${SPEED}" 'BEGIN { printf "%.4f", 100.0 / d / v }')
echo "[cbba_uav] 能量模型：懸停 ${HOVER} %/s、每公尺 ${PER_M} %（續航 ${DRAIN} s、${SPEED} m/s）；巡檢高度 ${ALT} m；出生點 ${SPAWN}"

# 等 gate 檔案出現（itest demo 讓全隊同時起飛、同時開始 CBBA）；cbba_uav.sh 結束時不要留下孤兒
wait_gate() {
    [ -n "${1:-}" ] || return 0
    echo "[cbba_uav] 等 $1 出現才啟動 $2"
    while [ ! -e "$1" ]; do
        kill -0 $$ 2>/dev/null || exit 0
        sleep 0.3
    done
    echo "[cbba_uav] 啟動 $2"
}

BRIDGE_ARGS=()
if [ "${EXECUTOR}" = bt ] || [ "${EXECUTOR}" = task_manager ]; then
    if [ "${EXECUTOR}" = bt ]; then
        BT_EXE=uav_bt_node; BT_NODE="uav${UAV_ID}_bt"
    else
        BT_EXE=uav_task_manager_node; BT_NODE="uav${UAV_ID}_task_manager"
        # 正式 BT 決定 mode（起飛完成、被接管、PX4 失效保護、返航），bridge 照抄進 robot_state（2026-10-17）
        BRIDGE_ARGS=(-p bt_mode_topic:="/uav${UAV_ID}/bt_mode")
    fi
    # CBBA_TAKEOFF_GATE=<檔案>：BT 等這個檔案出現才啟動（沒有 BT 就不會解鎖起飛），全隊一起起飛
    # shellcheck disable=SC2086  # EXEC_ARGS 要拆成多個參數
    # 飛行參數：uav_px4_bt/config/uav_bt.yaml（說明在 uav_px4_bt/doc/parameters.md）；agent_id、速度、高度由這裡蓋掉
    BT_PARAMS="$(ros2 pkg prefix uav_px4_bt)/share/uav_px4_bt/config/uav_bt.yaml"
    { wait_gate "${CBBA_TAKEOFF_GATE:-}" "BT（起飛）"; exec ros2 run uav_px4_bt "${BT_EXE}" --ros-args -r __node:="${BT_NODE}" \
        --params-file "${BT_PARAMS}" \
        -p agent_id:="${UAV_ID}" -p cruise_speed:="${SPEED}" -p altitude:="${ALT}" -p spawn_enu:="${SPAWN}" ${EXEC_ARGS:-}; } \
        2>&1 | prefix bt &
    PIDS+=($!)
    ros2 run apriltag_fire_detector fire_detector_node --ros-args -r __node:="uav${UAV_ID}_fire_detector" \
        -p agent_id:="${UAV_ID}" -p spawn_enu:="${SPAWN}" 2>&1 | prefix tag &
    PIDS+=($!)
else
    # shellcheck disable=SC2086
    ros2 run px4_waypoint_node task_executor --ros-args -r __node:="uav${UAV_ID}_task_executor" \
        -p agent_id:="${UAV_ID}" -p cruise_speed:="${SPEED}" -p altitude:="${ALT}" -p spawn_enu:="${SPAWN}" ${EXEC_ARGS:-} \
        2>&1 | prefix exec &
    PIDS+=($!)
fi
# PX4 → robot_state（位置、電量、mode）。mode：task_manager 照 BT 的 bt_mode；其他執行者由 bridge 自己判斷：
# 在空中、爬到巡檢高度（−0.5 m）而且 offboard 才 AUTO（起飛完成才出價；被接管、失效保護、返航時不出價）
ros2 run px4_waypoint_node px4_state_bridge --ros-args -r __node:="uav${UAV_ID}_px4_state_bridge" \
    -p agent_id:="${UAV_ID}" -p px4_ns:="uav${UAV_ID}" -p spawn_enu:="${SPAWN}" \
    -p auto_min_altitude:="$(awk -v a="${ALT}" 'BEGIN {printf "%.2f", (a > 1.0 ? a - 0.5 : a / 2)}')" \
    "${BRIDGE_ARGS[@]}" 2>&1 | prefix state &
PIDS+=($!)
# 統一版 cbba_node（swarm_cbba，2026-10-12）：namespace 給 /uavN、參數用 config/cbba_uav.yaml（和機器狗端相同的寫法）
# CBBA_START_GATE=<檔案>：cbba_node 等這個檔案出現才啟動（itest demo：全隊都起飛到同一個高度後才開始 CBBA）
start_cbba() {
    wait_gate "${CBBA_START_GATE:-}" "cbba_node（開始 CBBA）"
    exec ros2 run swarm_cbba cbba_node --ros-args -r __ns:="/uav${UAV_ID}" -r __node:="uav${UAV_ID}_cbba_node" \
        --params-file "$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config/cbba_uav.yaml" \
        -p agent_id:="${UAV_ID}" \
        -p cruise_speed:="${SPEED}" -p recon_altitude:="${ALT}" \
        -p hover_energy_per_sec:="${HOVER}" -p energy_per_meter:="${PER_M}" "$@"
}
start_cbba "$@" 2>&1 | prefix cbba &
PIDS+=($!)

echo "[cbba_uav] uav${UAV_ID} 已啟動；接著到 sim 執行 px4_sitl.sh ${UAV_ID}"
wait
