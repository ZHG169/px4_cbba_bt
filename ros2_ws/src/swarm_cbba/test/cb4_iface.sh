#!/bin/bash
# CB-4 第一批（precedence，2026-11-16，cbba_core/doc/cb4_design.md）的測試：閘道＋UAV（1）＋狗（50）的 cbba_node，
# 不用 BT，用 ros2 topic pub 假裝 BT，約 1.5 分鐘
#   注入一組兩個角色：role 0 aerial_overwatch（uav、optional）steps go_to → track；
#                     role 1 front（狗、precedence = role 0）steps go_to → inspect（wait_for role 0 target_localized）
#   P1 狗拿到 front，step 1 的 gate WAITING（step 0 不用等：狗可以先出發）；teammates 有兩個角色
#   P2 假 UAV BT 發 task_progress（target_localized）→ 狗的 gate 變 OPEN（主要驗收）
#   P3 不是自己 current_task 的回報被忽略（狗替 UAV 的任務回報）
#   P4 狗回報 BLOCKED：保持正式指派、不計入執行逾時（log）
#   P5 晚加入：UAV 已經回報定位之後才啟動狗 → 狗從進度補發（快照／定期重送）拿到，gate OPEN
#   P6 沒有 UAV：role 0 被略過（O1 約 10 s），skip_satisfies = true → 狗的 gate OPEN
#   P7 skip_satisfies = false：略過後狗的 gate UNSATISFIABLE（第一批只回報）
# 用法（uav 容器裡，編譯完）：bash src/swarm_cbba/test/cb4_iface.sh（在 ros2_ws 裡）；log 在 $OUT（預設 /tmp/cbba_cb4）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 80、UDP port 14680、程序鎖與世代檔另開；節點名稱 cb4t_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-80} CBBA_LOCK_DIR=/tmp/cbba_cb4_lock
L=${OUT:-/tmp/cbba_cb4}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
COMMON=(-p udp_port:=14680 -p state_dir:=$L/state)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
cnt() { clean "$1" | grep -cF -- "$2"; }
# wait_more 字串 檔案 已有幾次 秒：等到出現次數超過「已有幾次」
wait_more() { local t=0; until [ "$(cnt "$2" "$1")" -gt "$3" ] || [ $t -ge $(( $4 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; [ "$(cnt "$2" "$1")" -gt "$3" ]; }
check() { wait_more "$2" "$3" "${5:-0}" "$4" && ok "$1" || bad "$1（等不到「$2」，看 $3）"; }
stop_all() { pkill -f "__node:=cb4t_[a-z0-9]+( |$)" 2>/dev/null; sleep 1; }
trap stop_all EXIT

start_gcs() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=cb4t_gcs \
  --params-file $CFG/cbba_gateway.yaml "${COMMON[@]}" > $L/$1 2>&1 & }
start_uav() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/uav1 -r __node:=cb4t_uav \
  --params-file $CFG/cbba_uav.yaml "${COMMON[@]}" -p agent_id:=1 -p participate:=always \
  -p initial_position:="[0.0, 0.0, 5.0]" -p battery:=100.0 > $L/$1 2>&1 & }
start_dog() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=cb4t_dog \
  --params-file $CFG/cbba_v60.yaml "${COMMON[@]}" -p participate:=always \
  -p initial_position:="[2.0, 0.0, 0.0]" -p battery:=100.0 > $L/$1 2>&1 & }

AIR_SPEC='{"steps": [{"action": "go_to", "location": {"x": 10.0, "y": 0.0, "z": 5.0}}, {"action": "track", "object": "person:trk_1"}]}'
GROUND_SPEC='{"steps": [{"action": "go_to", "location": {"x": 10.0, "y": -2.0, "z": 0.0}}, {"action": "inspect", "object": "person", "wait_for": [{"role_index": 0, "milestone": "target_localized"}]}]}'

# inject 事件 role 平台 能力 role_index optional skip_satisfies precedence spec → 印 task_id
inject() {
  ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
    "{incident_id: '$1', role: '$2', revision: 1, platform: $3, required_capabilities: $4, priority: 0,
      position: {x: 10.0, y: 0.0, z: 0.0}, value: 100.0, service_time_s: 30.0,
      role_index: $5, role_count: 2, optional: $6, skip_satisfies: $7, precedence: $8, spec: '$9'}" \
    > $L/inject_$1_$2.log 2>&1
  grep -o 'task_id=[0-9]*' $L/inject_$1_$2.log | head -1 | cut -d= -f2
}
inject_group() {   # inject_group 事件 skip_satisfies → 設 AIR、GROUND
  AIR=$(inject "$1" aerial_overwatch 1 "['mobility_air', 'rgb_camera']" 0 true "$2" 0 "$AIR_SPEC")
  GROUND=$(inject "$1" front 2 "['mobility_ground', 'rgb_camera']" 1 false false 1 "$GROUND_SPEC")
  echo "  $1：air $AIR、ground $GROUND"
}
# current ns：current_task 的 task_id
current() { timeout 3 ros2 topic echo --once --field task_id /$1/current_task swarm_interfaces/msg/Task 2>/dev/null | head -1; }
wait_current() {   # wait_current ns task_id 秒
  local t=0; until [ "$(current $1)" = "$2" ] || [ $t -ge $3 ]; do sleep 1; t=$((t+1)); done; [ "$(current $1)" = "$2" ]
}
progress() {   # progress ns task_id phase step 里程碑（YAML 陣列）
  ros2 topic pub --once -w 1 /$1/task_progress swarm_interfaces/msg/TaskProgress \
    "{task_id: $2, phase: $3, step_index: $4, milestones: $5}" > /dev/null 2>&1
}

# ---------- A：P1～P4 ----------
echo "== A：閘道＋UAV＋狗"
start_gcs gcs_a.log; start_uav uav_a.log; start_dog dog_a.log
sleep 5
inject_group cb4-a true
if [ -n "$AIR" ] && [ -n "$GROUND" ] && wait_current v60 "$GROUND" 15 && wait_current uav1 "$AIR" 10; then
  ok "P0 UAV 拿到 aerial_overwatch、狗拿到 front"
else
  bad "P0 指派（狗 $(current v60)、UAV $(current uav1)；看 $L/inject_*.log）"
fi
check "P1 狗的 step 1 gate WAITING" "step 1：WAITING" $L/dog_a.log 5
roles=$(timeout 4 ros2 topic echo --once /v60/teammates swarm_interfaces/msg/Teammates 2>/dev/null | grep -c "role_index:")
[ "$roles" -ge 3 ] && ok "P1 teammates 有兩個角色（＋自己的 role_index）" || bad "P1 teammates（role_index 出現 $roles 次）"

progress v60 "$AIR" 4 1 "['target_localized']"
check "P3 不是 current_task 的回報忽略" "不是目前 current_task 上的任務" $L/dog_a.log 5

progress v60 "$GROUND" 5 1 "[]"
check "P4 狗 BLOCKED：保持指派、不計入逾時" "停在 wait_for（BLOCKED）" $L/dog_a.log 5

T0=$(date +%s.%N)
progress uav1 "$AIR" 4 1 "['arrived', 'target_localized']"
if wait_more "WAITING → OPEN" $L/dog_a.log 0 5; then
  ok "P2 UAV 回報 target_localized → 狗的 gate OPEN（約 $(awk "BEGIN{printf \"%.1f\", $(date +%s.%N) - $T0}") s，含 ros2 topic pub 啟動）"
else
  bad "P2 狗的 gate 沒有變 OPEN（看 $L/dog_a.log、$L/uav_a.log）"
fi
progress v60 "$GROUND" 4 1 "['arrived']"
check "P4 解除等待" "解除等待" $L/dog_a.log 5
stop_all

# ---------- B：P5 晚加入 ----------
echo "== B：狗在 UAV 回報之後才啟動"
start_gcs gcs_b.log; start_uav uav_b.log
sleep 5
inject_group cb4-b true
wait_current uav1 "$AIR" 10 || bad "P5 UAV 沒拿到 aerial_overwatch"
progress uav1 "$AIR" 4 1 "['target_localized']"
sleep 1
start_dog dog_b.log
if wait_current v60 "$GROUND" 20 && wait_more "step 1：OPEN" $L/dog_b.log 0 10; then
  ok "P5 晚加入的狗從補發拿到定位，gate OPEN"
else
  bad "P5（狗 $(current v60)；看 $L/dog_b.log）"
fi
stop_all

# ---------- C：P6、P7 沒有 UAV ----------
echo "== C：沒有 UAV（optional 角色被略過）"
start_gcs gcs_c.log; start_dog dog_c.log
sleep 5
inject_group cb4-c true
wait_current v60 "$GROUND" 15 || bad "P6 狗沒拿到 front"
if wait_more "WAITING → OPEN" $L/dog_c.log 0 20; then
  ok "P6 role 0 被略過（skip_satisfies = true）→ 狗的 gate OPEN"
else
  bad "P6（看 $L/dog_c.log、$L/gcs_c.log）"
fi
ros2 topic pub --once -w 1 /gcs/cancel_task swarm_interfaces/msg/Task "{task_id: $GROUND}" > /dev/null 2>&1
inject_group cb4-d false
wait_current v60 "$GROUND" 15 || bad "P7 狗沒拿到第二組的 front"
if wait_more "WAITING → UNSATISFIABLE" $L/dog_c.log 0 20; then
  ok "P7 skip_satisfies = false：略過後 gate UNSATISFIABLE"
else
  bad "P7（看 $L/dog_c.log）"
fi

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
