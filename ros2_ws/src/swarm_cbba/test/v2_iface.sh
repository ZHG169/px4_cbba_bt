#!/bin/bash
# 介面 v2.0（2026-11-14）的測試：閘道＋一隻狗的 cbba_node（不用 BT，用 ros2 topic pub 假裝 BT），約 1 分鐘
#   V1 current_task 帶新欄位（priority、platform、required_capabilities、expires）
#   V2 全隊沒有能做的載具 → task_orphaned NO_CAPABLE（約 3 s）
#   V3 E_HW＋failed_capability → reason 照表（PERMANENT）、拿掉能力、之後需要它的任務 NO_CAPABLE
#   V4 E_PRECONDITION → PERMANENT＋task_orphaned PRECONDITION
#   V5 不認得的 failure_code → 照 BT 填的 reason
#   V6 過期 → task_orphaned EXPIRED
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/v2_iface.sh；log 在 $OUT（預設 /tmp/cbba_v2）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 77、UDP port 14677、程序鎖與世代檔另開；節點名稱 v2t_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} CBBA_LOCK_DIR=/tmp/cbba_v2_lock
L=${OUT:-/tmp/cbba_v2}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
COMMON=(-p udp_port:=14677 -p state_dir:=$L/state)
mkdir -p $L/state
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
cnt() { clean "$1" | grep -cF -- "$2"; }
# wait_more 字串 檔案 已有幾次 秒：等到出現次數超過「已有幾次」
wait_more() { local t=0; until [ "$(cnt "$2" "$1")" -gt "$3" ] || [ $t -ge $(( $4 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; [ "$(cnt "$2" "$1")" -gt "$3" ]; }
check() { wait_more "$2" "$3" "${5:-0}" "$4" && ok "$1" || bad "$1（等不到「$2」）"; }
cleanup() {
  pkill -f "__node:=v2t_(gcs|v60)( |$)" 2>/dev/null
  sleep 0.5
}
trap cleanup EXIT

ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=v2t_gcs \
  --params-file $CFG/cbba_gateway.yaml "${COMMON[@]}" > $L/gcs.log 2>&1 &
# 狗：預設能力（UGV：mobility_ground、rgb_camera）
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=v2t_v60 \
  --params-file $CFG/cbba_v60.yaml "${COMMON[@]}" -p participate:=always \
  -p initial_position:="[0.0, 0.0, 0.0]" -p battery:=100.0 > $L/v60.log 2>&1 &
sleep 4

inject() {   # inject 事件 平台 能力(YAML 陣列) 失效秒數
  ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
    "{incident_id: '$1', revision: 1, platform: $2, required_capabilities: $3, priority: 1,
      position: {x: 5.0, y: 2.0, z: 0.0}, value: 100.0, service_time_s: 10.0, expires_sec: $4}" \
    >> $L/inject.log 2>&1
}
# 等 current_task 換成新的任務（task_id 不是 0、也不是 $1），印出 task_id
next_task() {
  local t=0 id
  while [ $t -lt 30 ]; do
    id=$(timeout 3 ros2 topic echo --once --field task_id /v60/current_task swarm_interfaces/msg/Task 2>/dev/null | head -1)
    if [ -n "$id" ] && [ "$id" != "0" ] && [ "$id" != "$1" ]; then echo "$id"; return 0; fi
    sleep 0.5; t=$((t+1))
  done
  return 1
}
result() {   # result task_id reason failure_code failed_capability
  ros2 topic pub --once -w 1 /v60/task_result swarm_interfaces/msg/TaskResult \
    "{task_id: $1, success: false, reason: $2, detail: 'v2 test', failure_code: '$3', failed_capability: '$4'}" \
    >> $L/pub.log 2>&1
}

# ---------- V1：current_task 的新欄位 ----------
inject v2-cam 2 "[rgb_camera]" 600.0
ID1=$(next_task 0) && ok "V1 狗拿到任務 $ID1" || bad "V1 狗沒拿到任務"
# 帶型別、重試：剛開始 discovery 還沒抓到話題時，不帶型別的 echo 會直接失敗
for i in 1 2 3 4 5; do
  timeout 5 ros2 topic echo --once /v60/current_task swarm_interfaces/msg/Task > $L/current_task.txt 2>&1
  grep -qF "task_id: $ID1" $L/current_task.txt && break
  sleep 1
done
for f in "priority: 1" "platform: 2" "- rgb_camera" "expires:"; do
  grep -qF -- "$f" $L/current_task.txt && ok "V1 current_task 有「$f」" || bad "V1 current_task 沒有「$f」"
done

# ---------- V2：全隊沒有 thermal_camera ----------
inject v2-thermal 2 "[thermal_camera]" 600.0
check "V2 閘道報 NO_CAPABLE" "NO_CAPABLE" $L/gcs.log 8
check "V2 狗也報 NO_CAPABLE" "NO_CAPABLE" $L/v60.log 8

# ---------- V3：E_HW 拿掉 rgb_camera ----------
N_GCS=$(cnt $L/gcs.log "NO_CAPABLE")
result "$ID1" 2 E_HW rgb_camera     # BT 填 TRANSIENT，照表要變 PERMANENT
check "V3 E_HW → PERMANENT" "failure_code E_HW → reason PERMANENT" $L/v60.log 5
check "V3 拿掉能力" "能力 rgb_camera 故障" $L/v60.log 5
check "V3 這台不再接" "這台不再接" $L/v60.log 5
check "V3 閘道對 rgb_camera 任務報 NO_CAPABLE" "NO_CAPABLE" $L/gcs.log 10 "$N_GCS"

# ---------- V4：E_PRECONDITION ----------
inject v2-pre 2 "[mobility_ground]" 600.0
ID2=$(next_task "$ID1") && ok "V4 狗拿到任務 $ID2" || bad "V4 狗沒拿到任務"
result "$ID2" 0 E_PRECONDITION ""
check "V4 E_PRECONDITION → PERMANENT" "failure_code E_PRECONDITION → reason PERMANENT" $L/v60.log 5
check "V4 task_orphaned PRECONDITION" "：PRECONDITION" $L/v60.log 5

# ---------- V5：不認得的 failure_code ----------
inject v2-foo 2 "[mobility_ground]" 600.0
ID3=$(next_task "$ID2") && ok "V5 狗拿到任務 $ID3" || bad "V5 狗沒拿到任務"
result "$ID3" 2 E_FOO ""
check "V5 不認得的碼照 reason" "不認得的 failure_code「E_FOO」，照 reason 2" $L/v60.log 5
check "V5 TRANSIENT 排除 60 s" "這台 60 s 內不接" $L/v60.log 5

# ---------- V6：過期 ----------
inject v2-exp 2 "[mobility_ground]" 5.0
check "V6 閘道 task_orphaned EXPIRED" "：EXPIRED" $L/gcs.log 12
check "V6 狗 task_orphaned EXPIRED" "：EXPIRED" $L/v60.log 12

echo "----"
echo "PASS $PASS  FAIL $FAIL   （log：$L）"
[ $FAIL -eq 0 ]
