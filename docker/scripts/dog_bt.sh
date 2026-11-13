#!/usr/bin/env bash
# ============================================================
#  [uav 容器] 模擬裡跑伯宇的狗 BT（task_manager，2026-10-27），身體用方塊（sim 容器的 fake_nav_body.py）
#
#  用法：
#    dog_bt.sh build                       編譯伯宇的 task_manager、task_manager_interfaces（handoff；2026-11-11 起介接層 task_id 改 uint64，見 docker/dog/task_manager_uint64.patch）
#                                          到 <repo>/dog_ws（overlay 在 ros2_ws/install 上；不碰共用的 ros2_ws/build、install）
#    dog_bt.sh start <ns> <機號> [任務檔]     啟動一隻狗的 task_manager（前景；itest 用 docker exec -d 開）；
#                                          任務檔預設 docker/dog/idle_mission.yaml（不巡邏）；itest 給巡邏檔時照伯宇的巡檢線走（2026-11-05）
#    dog_bt.sh stop                        關掉全部的 task_manager
#
#  一隻狗要三樣（itest.py demo --dog bt 會一起開）：
#    sim 容器   fake_nav_body.py --ns <ns>        方塊＋假 Nav2（/<ns>/navigate_to_pose）＋假雲台（/<ns>/ptz_inspect）＋TF
#    uav1 容器  dog_bt.sh start <ns> <機號>       伯宇的 BT（這個腳本）
#    uav1 容器  swarm_cbba/cbba_node（cbba_v60.yaml）  和假狗時一樣
#
#  和伯宇預設不同的參數（他的程式不改，只給參數）：
#    nav_action  /<ns>/navigate_to_pose   他的預設 /navigate_to_pose 是全域的，3 隻狗會撞
#    ptz_action  /<ns>/ptz_inspect        同上
#    base_frame  <ns>/base_link           TF 的 frame 每隻分開（fake_nav_body.py 發 map → <ns>/base_link）
#    battery_topic /<ns>/battery          沒人發，用 battery_default 100
#    mission     docker/dog/idle_mission.yaml＋autostart:=true   沒有巡檢點；他的樹要第一次開始任務才建，
#                                         沒建時閒置中收到的外部任務不會開跑（task_manager_node.cpp 第 228 行）
#  他程式裡寫死的全域話題：/emergency_stop（3 隻一起急停）、/start_mission（3 隻一起重新載入任務檔）
#
#  環境變數：HANDOFF（預設 <repo>/docs/handoff_uav_2026-10-12/bt：伯宇的原檔，不進 repo）、DOG_WS（預設 <repo>/dog_ws）
# ============================================================
set -eo pipefail

REPO="${CBBA_REPO:-/home/ncrl/CBBA_BT}"
HANDOFF="${HANDOFF:-${REPO}/docs/handoff_uav_2026-10-12/bt}"
DOG_WS="${DOG_WS:-${REPO}/dog_ws}"

source /opt/ros/jazzy/setup.bash
source "${REPO}/ros2_ws/install/setup.bash"

case "${1:-}" in
    build)
        if [ ! -d "${HANDOFF}/task_manager" ] || [ ! -d "${HANDOFF}/task_manager_interfaces" ]; then
            echo "找不到伯宇的 task_manager：${HANDOFF}（handoff 不在 repo 裡，要另外放）"; exit 1
        fi
        mkdir -p "${DOG_WS}"
        cd "${DOG_WS}"
        # --base-paths 直接指到 handoff，不複製；2026-11-11 起他的介接層 task_id 改 uint64（docker/dog/task_manager_uint64.patch），
        # 2026-11-13 起讀 current_task.spec（docker/dog/task_manager_spec.patch，要先套 uint64 的），
        # 他更新時覆蓋 handoff、照 patch 開頭的說明重套，再 build 一次
        python3 -m colcon build --base-paths "${HANDOFF}" \
            --build-base "${DOG_WS}/build" --install-base "${DOG_WS}/install" \
            --packages-select task_manager_interfaces task_manager --cmake-args -DCMAKE_BUILD_TYPE=Release
        echo "完成：${DOG_WS}/install（sim、uav 容器用之前都要 source）"
        ;;
    start)
        NS="${2:?用法：dog_bt.sh start <ns> <機號>}"
        ID="${3:?用法：dog_bt.sh start <ns> <機號>}"
        MISSION="${4:-${REPO}/docker/dog/idle_mission.yaml}"
        if [ ! -f "${DOG_WS}/install/setup.bash" ]; then
            echo "還沒編譯：先 dog_bt.sh build"; exit 1
        fi
        source "${DOG_WS}/install/setup.bash"
        SHARE="$(ros2 pkg prefix task_manager)/share/task_manager"
        # 直接 ros2 run（不用他的 launch）：節點名稱 <ns>_task_manager，stop 時用名稱比對節點本身
        exec ros2 run task_manager task_manager_node --ros-args -r __ns:="/${NS}" -r __node:="${NS}_task_manager" \
            -p mission_file:="${MISSION}" -p bt_file:="${SHARE}/bt/mission.xml" \
            -p autostart:=true -p agent_id:="${ID}" \
            -p nav_action:="/${NS}/navigate_to_pose" -p ptz_action:="/${NS}/ptz_inspect" \
            -p base_frame:="${NS}/base_link" -p battery_topic:="/${NS}/battery"
        ;;
    stop)
        # 比對執行檔路徑（無人機的 BT 叫 uavN_task_manager、執行檔 uav_task_manager_node，不能被關到）
        pkill -f "lib/task_manager/task_manager_node" || true
        echo "已關掉 task_manager"
        ;;
    *)
        echo "用法：$0 build | start <ns> <機號> | stop"; exit 1
        ;;
esac
