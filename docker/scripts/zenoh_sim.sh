#!/usr/bin/env bash
# ============================================================
#  [主機] 模擬裡一次開／關 GCS ↔ 載具的 Zenoh bridge（swarm_zenoh 套件，2026-10-21）
#
#  用法：
#    docker/scripts/zenoh_sim.sh start     GCS（router）＋ uav1～3 ＋ 3 隻假狗的 bridge 全部背景啟動
#    docker/scripts/zenoh_sim.sh stop      全部關掉
#    docker/scripts/zenoh_sim.sh status    各容器有幾個 bridge 在跑
#    docker/scripts/zenoh_sim.sh logs      各 bridge 的 log 最後幾行
#
#  配置（和實機不同的地方）：
#    - gcs 容器：--role gcs，聽 mesh 的 172.30.0.100:7447
#    - uavN 容器：--role vehicle --ns uavN，連 GCS（經過 mesh，mesh_tc.sh 的掉包會影響它）
#    - 假狗：sim 容器不在 mesh 上，所以狗的 bridge 放在 uav1（uav1 的 DDS 看得到 sim 裡的 fake_dog.py），
#      每隻一個 bridge（假裝各自一台機器），共用 uav1 的 mesh 連線
#  環境變數：UAV_IDS（預設 "1 2 3"）、DOG_NS（預設 "v60 v60_1 v60_2"，和 itest config 的 dog.units 一致；2026-10-24 照伯宇的狗改名）、
#           DOG_HOST（狗的 bridge 放哪台，預設 1）
#  log：各容器的 /tmp/swarm_zenoh_<名稱>.log
#  前提：映像已裝 zenoh-bridge-ros2dds（robot.Dockerfile）、swarm_zenoh 已編譯（共用的 ros2_ws/install）
# ============================================================
set -euo pipefail

UAV_IDS="${UAV_IDS:-1 2 3}"
DOG_NS="${DOG_NS:-v60 v60_1 v60_2}"
DOG_HOST="${DOG_HOST:-1}"
GCS="cbba_gcs"
SETUP="source /opt/ros/jazzy/setup.bash; source /home/ncrl/CBBA_BT/ros2_ws/install/setup.bash"

# 在容器裡背景啟動一個 bridge：run <容器> <名稱> <zenoh_bridge 的參數…>
run() {
    local c="$1" name="$2"
    shift 2
    docker exec -d -u ncrl "$c" bash -c \
        "${SETUP}; exec ros2 run swarm_zenoh zenoh_bridge $* --out-dir /tmp/swarm_zenoh_${name} > /tmp/swarm_zenoh_${name}.log 2>&1"
    echo "  ${c}：${name}（$*）"
}

containers() {
    echo "${GCS}"
    for v in ${UAV_IDS}; do echo "cbba_uav${v}"; done
}

case "${1:-status}" in
    start)
        for c in $(containers); do
            if ! docker ps --format '{{.Names}}' | grep -qx "$c"; then
                echo "容器 $c 沒有在跑（先 docker/start.sh）"; exit 1
            fi
            if docker exec "$c" pgrep -f zenoh-bridge-ros2dds > /dev/null; then
                echo "$c 已經有 bridge 在跑，先 $0 stop"; exit 1
            fi
        done
        echo "啟動 Zenoh bridge："
        run "${GCS}" gcs --role gcs
        for v in ${UAV_IDS}; do
            run "cbba_uav${v}" "uav${v}" --role vehicle --ns "uav${v}"
        done
        for ns in ${DOG_NS}; do
            run "cbba_uav${DOG_HOST}" "${ns}" --role vehicle --ns "${ns}"
        done
        sleep 2
        "$0" status
        echo "GCS 上檢查：docker/enter.sh gcs → ros2 topic list | grep -E 'robot_state|current_task'"
        ;;
    stop)
        for c in $(containers); do
            docker exec "$c" pkill -f zenoh-bridge-ros2dds 2> /dev/null || true
        done
        echo "已關掉全部 bridge"
        ;;
    status)
        for c in $(containers); do
            n=$(docker exec "$c" pgrep -fc zenoh-bridge-ros2dds 2> /dev/null || true)
            echo "  ${c}：${n:-0} 個 bridge"
        done
        ;;
    logs)
        for c in $(containers); do
            for f in $(docker exec "$c" bash -c 'ls /tmp/swarm_zenoh_*.log 2>/dev/null' || true); do
                echo "== ${c} ${f}"
                docker exec "$c" bash -c "grep -v 'Zenoh Config' $f | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-200 | tail -5"
            done
        done
        ;;
    *)
        echo "用法：$0 start | stop | status | logs"; exit 1
        ;;
esac
