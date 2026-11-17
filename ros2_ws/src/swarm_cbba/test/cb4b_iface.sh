#!/bin/bash
# CB-4 第二批（同步與整組提交，2026-11-16，cbba_core/doc/cb4_design.md）的測試：閘道＋UAV（1）＋狗（50）的 cbba_node，
# 不用 BT，用 ros2 topic pub 假裝 BT，約 1.5 分鐘
#   S1 會合點：兩個角色的 step 1 都是 sync "meet"（sync_window_s 15）：狗先到（ARRIVED_WAITING）→ 還 WAITING；
#      UAV 也到 → 狗的 gate OPEN
#   S2 時間窗：狗到了、UAV 一直沒到 → 6 s 後狗的 gate 照樣 OPEN（逾時開始，log 警告）
#   S3 all-or-nothing：只有狗、必要的空中角色沒人能做 → 狗進 WAITING_GROUP、不發 current_task；
#      group_commit_timeout（測試給 8 s）後 task_orphaned GROUP_INCOMPLETE、繼續等
#   S4 UAV 上線 → 組湊齊、穩定 1 s → 狗提交、拿到 current_task
# 用法（uav 容器裡，編譯完）：bash src/swarm_cbba/test/cb4b_iface.sh（在 ros2_ws 裡）；log 在 $OUT（預設 /tmp/cbba_cb4b）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 81、UDP port 14681、程序鎖與世代檔另開；節點名稱 cb4b_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-81} CBBA_LOCK_DIR=/tmp/cbba_cb4b_lock
L=${OUT:-/tmp/cbba_cb4b}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
COMMON=(-p udp_port:=14681 -p state_dir:=$L/state -p group_commit_timeout:=8.0)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
cnt() { clean "$1" | grep -cF -- "$2"; }
wait_more() { local t=0; until [ "$(cnt "$2" "$1")" -gt "$3" ] || [ $t -ge $(( $4 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; [ "$(cnt "$2" "$1")" -gt "$3" ]; }
stop_all() { pkill -f "__node:=cb4b_[a-z0-9]+( |$)" 2>/dev/null; sleep 1; }
trap stop_all EXIT
# 這支只測第一、二批：閘道的連鎖撤銷（第三批：暫時不完整 10 s 就取消）時限拉長，不干擾
GW_ISOLATE="-p group_recovery_timeout_s:=120.0"

start_gcs() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=cb4b_gcs \
  --params-file $CFG/cbba_gateway.yaml "${COMMON[@]}" $GW_ISOLATE > $L/$1 2>&1 & }
start_uav() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/uav1 -r __node:=cb4b_uav \
  --params-file $CFG/cbba_uav.yaml "${COMMON[@]}" -p agent_id:=1 -p participate:=always \
  -p initial_position:="[0.0, 0.0, 5.0]" -p battery:=100.0 > $L/$1 2>&1 & }
start_dog() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=cb4b_dog \
  --params-file $CFG/cbba_v60.yaml "${COMMON[@]}" -p participate:=always \
  -p initial_position:="[2.0, 0.0, 0.0]" -p battery:=100.0 > $L/$1 2>&1 & }

SYNC_SPEC='{"steps": [{"action": "go_to", "location": {"x": 10.0, "y": 0.0, "z": 0.0}}, {"action": "inspect", "object": "person", "sync": "meet"}]}'

# inject 事件 role 平台 能力 role_index all_or_nothing sync_window_s → 印 task_id
inject() {
  ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
    "{incident_id: '$1', role: '$2', revision: 1, platform: $3, required_capabilities: $4, priority: 0,
      position: {x: 10.0, y: 0.0, z: 0.0}, value: 100.0, service_time_s: 30.0,
      role_index: $5, role_count: 2, all_or_nothing: $6, sync_window_s: $7, spec: '$SYNC_SPEC'}" \
    > $L/inject_$1_$2.log 2>&1
  grep -o 'task_id=[0-9]*' $L/inject_$1_$2.log | head -1 | cut -d= -f2
}
inject_group() {   # inject_group 事件 all_or_nothing sync_window_s → 設 AIR、GROUND
  AIR=$(inject "$1" overwatch 1 "['mobility_air', 'rgb_camera']" 0 "$2" "$3")
  GROUND=$(inject "$1" front 2 "['mobility_ground', 'rgb_camera']" 1 "$2" "$3")
  echo "  $1：air $AIR、ground $GROUND"
}
current() { timeout 3 ros2 topic echo --once --field task_id /$1/current_task swarm_interfaces/msg/Task 2>/dev/null | head -1; }
wait_current() { local t=0; until [ "$(current $1)" = "$2" ] || [ $t -ge $3 ]; do sleep 1; t=$((t+1)); done; [ "$(current $1)" = "$2" ]; }
progress() {   # progress ns task_id phase step
  ros2 topic pub --once -w 1 /$1/task_progress swarm_interfaces/msg/TaskProgress \
    "{task_id: $2, phase: $3, step_index: $4}" > /dev/null 2>&1
}
cancel() { ros2 topic pub --once -w 1 /gcs/cancel_task swarm_interfaces/msg/Task "{task_id: $1}" > /dev/null 2>&1; }

# ---------- S1、S2：會合點 ----------
echo "== S：閘道＋UAV＋狗，會合點 meet"
start_gcs gcs_s.log; start_uav uav_s.log; start_dog dog_s.log
sleep 5
inject_group cb4b-s false 15
if wait_current v60 "$GROUND" 15 && wait_current uav1 "$AIR" 10; then ok "S0 指派"; else bad "S0 指派（狗 $(current v60)、UAV $(current uav1)）"; fi
wait_more "step 1：WAITING" $L/dog_s.log 0 5 || bad "S1 狗的會合點一開始不是 WAITING（看 $L/dog_s.log）"
progress v60 "$GROUND" 6 1
sleep 2
if [ "$(cnt $L/dog_s.log "WAITING → OPEN")" -eq 0 ]; then ok "S1 只有狗到：還在等"; else bad "S1 只有狗到就 OPEN 了"; fi
progress uav1 "$AIR" 6 1
if wait_more "WAITING → OPEN" $L/dog_s.log 0 5; then ok "S1 UAV 也到了 → 狗的 gate OPEN"; else bad "S1（看 $L/dog_s.log）"; fi
cancel "$AIR"; cancel "$GROUND"
sleep 2
inject_group cb4b-t false 6
wait_current v60 "$GROUND" 15 || bad "S2 狗沒拿到第二組"
opens=$(cnt $L/dog_s.log "WAITING → OPEN")
progress v60 "$GROUND" 6 1
if wait_more "照樣開始" $L/dog_s.log 0 12 && wait_more "WAITING → OPEN" $L/dog_s.log $opens 2; then
  ok "S2 UAV 沒到：時間窗 6 s 後照樣開始（逾時）"
else
  bad "S2（看 $L/dog_s.log）"
fi
stop_all

# ---------- S3、S4：all-or-nothing ----------
echo "== G：all-or-nothing（group_commit_timeout 8 s）"
start_gcs gcs_g.log; start_dog dog_g.log
sleep 5
inject_group cb4b-g true 0
if wait_more "進入 WAITING_GROUP" $L/dog_g.log 0 10 && [ "$(current v60)" != "$GROUND" ]; then
  ok "S3 必要角色沒人能做：狗 WAITING_GROUP、不發 current_task"
else
  bad "S3（狗 $(current v60)；看 $L/dog_g.log）"
fi
if wait_more "GROUP_INCOMPLETE" $L/dog_g.log 0 15; then ok "S3 等 8 s 後通知 GROUP_INCOMPLETE、繼續等"; else bad "S3 沒有 GROUP_INCOMPLETE"; fi
start_uav uav_g.log
if wait_current v60 "$GROUND" 20 && wait_more "提交" $L/dog_g.log 0 2; then
  ok "S4 UAV 上線 → 組湊齊、狗提交拿到 current_task"
else
  bad "S4（狗 $(current v60)；看 $L/dog_g.log、$L/uav_g.log）"
fi

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
