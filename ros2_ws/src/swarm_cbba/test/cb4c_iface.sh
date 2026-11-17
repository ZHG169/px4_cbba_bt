#!/bin/bash
# CB-4 第三批（閘道管理＋distinct_agent_set，2026-11-16，cbba_core/doc/cb4_design.md）的測試：閘道＋UAV＋狗的 cbba_node，
# 不用 BT，用 ros2 topic pub 假裝 BT，約 2 分鐘
#   I1 incident：UAV（role 0 的執行者）發 incident_update＋宣告定位 → 閘道接受（incident revision 1、發 incident_updates）
#      → 狗（等 role 0 target_localized）的 gate OPEN
#   D1 distinct 案例 1：dog50 靠近 front、rear，dog51 很遠 → 兩隻各拿一個
#   D2 distinct 案例 2：只有 dog50、all_or_nothing → 只拿一個角色、GROUP_INCOMPLETE；group_recovery_timeout_s（10 s）後
#      閘道整組取消、FallbackRequest（single_dog_verify）
#   C1 依賴失效：role 0 空中（必要、沒有 UAV）、role 1 等 role 0、role 2 不依賴 → 暫時不完整（沒有能做的載具）
#      等 group_recovery_timeout_s（10 s）→ RECOVERY_TIMEOUT：role 0、1 取消，role 2 照做
# 用法（uav 容器裡，編譯完）：bash src/swarm_cbba/test/cb4c_iface.sh（在 ros2_ws 裡）；log 在 $OUT（預設 /tmp/cbba_cb4c）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 82、UDP port 14682、程序鎖與世代檔另開；節點名稱 cb4c_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-82} CBBA_LOCK_DIR=/tmp/cbba_cb4c_lock
L=${OUT:-/tmp/cbba_cb4c}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
COMMON=(-p udp_port:=14682 -p state_dir:=$L/state -p group_commit_timeout:=5.0)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
cnt() { clean "$1" | grep -cF -- "$2"; }
wait_more() { local t=0; until [ "$(cnt "$2" "$1")" -gt "$3" ] || [ $t -ge $(( $4 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; [ "$(cnt "$2" "$1")" -gt "$3" ]; }
stop_all() { pkill -f "__node:=cb4c_[a-z0-9]+( |$)" 2>/dev/null; pkill -f "ros2 topic echo .*/gcs/(incident_updates|fallback_request)" 2>/dev/null; sleep 1; }
trap stop_all EXIT

start_gcs() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=cb4c_gcs \
  --params-file $CFG/cbba_gateway.yaml "${COMMON[@]}" > $L/$1 2>&1 & }
start_uav() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/uav1 -r __node:=cb4c_uav \
  --params-file $CFG/cbba_uav.yaml "${COMMON[@]}" -p agent_id:=1 -p participate:=always \
  -p initial_position:="[0.0, 0.0, 5.0]" -p battery:=100.0 > $L/$1 2>&1 & }
start_dog() {   # start_dog log ns 機號 x
  ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/$2 -r __node:=cb4c_dog$3 \
    --params-file $CFG/cbba_v60.yaml "${COMMON[@]}" -p agent_id:=$3 -p participate:=always \
    -p initial_position:="[$4, 0.0, 0.0]" -p battery:=100.0 > $L/$1 2>&1 &
}

# inject 事件 role 平台 能力 role_index role_count precedence all_or_nothing distinct fallback x spec → 印 task_id
inject() {
  ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
    "{incident_id: '$1', role: '$2', revision: 1, platform: $3, required_capabilities: $4, priority: 0,
      position: {x: ${11}, y: 0.0, z: 0.0}, value: 100.0, service_time_s: 30.0, role_index: $5, role_count: $6,
      precedence: $7, all_or_nothing: $8, distinct_agent_set: $9, fallback_template: '${10}', spec: '${12}'}" \
    > $L/inject_$1_$2.log 2>&1
  grep -o 'task_id=[0-9]*' $L/inject_$1_$2.log | head -1 | cut -d= -f2
}
AIR="['mobility_air', 'rgb_camera']"
GROUND="['mobility_ground', 'rgb_camera']"
WAIT_SPEC='{"steps": [{"action": "go_to", "location": {"x": 10.0, "y": -2.0, "z": 0.0}}, {"action": "inspect", "object": "person", "wait_for": [{"role_index": 0, "milestone": "target_localized"}]}]}'
current() { timeout 3 ros2 topic echo --once --field task_id /$1/current_task swarm_interfaces/msg/Task 2>/dev/null | head -1; }
wait_current() { local t=0; until [ "$(current $1)" = "$2" ] || [ $t -ge $3 ]; do sleep 1; t=$((t+1)); done; [ "$(current $1)" = "$2" ]; }

# ---------- I1：incident ----------
echo "== I：閘道＋UAV＋狗，incident 更新"
start_gcs gcs_i.log; start_uav uav_i.log; start_dog dog_i.log v60 50 2.0
timeout 60 ros2 topic echo /gcs/incident_updates swarm_interfaces/msg/IncidentState > $L/incident_updates.txt 2>&1 &
sleep 5
A=$(inject cb4c-i overwatch 1 "$AIR" 0 2 0 false 0 "" 10.0 "")
G=$(inject cb4c-i front 2 "$GROUND" 1 2 1 false 0 "" 10.0 "$WAIT_SPEC")
echo "  air $A、ground $G"
if wait_current uav1 "$A" 15 && wait_current v60 "$G" 10; then ok "I0 指派"; else bad "I0 指派（UAV $(current uav1)、狗 $(current v60)）"; fi
wait_more "step 1：WAITING" $L/dog_i.log 0 5 || bad "I1 狗的 gate 一開始不是 WAITING"
ros2 topic pub --once -w 1 /uav1/incident_update swarm_interfaces/msg/IncidentUpdate \
  "{task_id: $A, position_valid: true, position: {x: 14.0, y: 3.0, z: 0.0}, radius: 1.5, track_id: 88, declares_localized: true}" \
  > /dev/null 2>&1
if wait_more "incident 接受" $L/gcs_i.log 0 5 && wait_more "incident 提案 1 接受" $L/uav_i.log 0 5; then
  ok "I1 閘道接受提案（incident revision 1）"
else
  bad "I1 閘道沒接受（看 $L/gcs_i.log、$L/uav_i.log）"
fi
if wait_more "WAITING → OPEN" $L/dog_i.log 0 5; then ok "I1 定位通過驗證 → 狗的 gate OPEN"; else bad "I1 狗的 gate（看 $L/dog_i.log）"; fi
if grep -q "incident_revision: 1" $L/incident_updates.txt && grep -q "incident_id: cb4c-i" $L/incident_updates.txt; then
  ok "I1 /gcs/incident_updates 給 Planner（incident_id、revision 1）"
else
  bad "I1 沒有收到 incident_updates（看 $L/incident_updates.txt）"
fi
stop_all

# ---------- D1：distinct 案例 1 ----------
echo "== D1：dog50 靠近、dog51 很遠，front、rear distinct"
start_gcs gcs_d1.log; start_dog dog50_d1.log v60 50 0.0; start_dog dog51_d1.log v60_1 51 60.0
sleep 5
F=$(inject cb4c-d1 front 2 "$GROUND" 0 2 0 false 3 "" 10.0 "")
R=$(inject cb4c-d1 rear 2 "$GROUND" 1 2 0 false 3 "" 12.0 "")
sleep 6
c50=$(current v60); c51=$(current v60_1)
if { [ "$c50" = "$F" ] && [ "$c51" = "$R" ]; } || { [ "$c50" = "$R" ] && [ "$c51" = "$F" ]; }; then
  ok "D1 兩隻狗各拿一個角色（dog50 $c50、dog51 $c51）"
else
  bad "D1（front $F、rear $R；dog50 $c50、dog51 $c51）"
fi
stop_all

# ---------- D2：distinct 案例 2（只有 dog50、all_or_nothing） ----------
echo "== D2：只有 dog50，all_or_nothing＋distinct＋fallback"
start_gcs gcs_d2.log; start_dog dog_d2.log v60 50 0.0
timeout 60 ros2 topic echo /gcs/fallback_request swarm_interfaces/msg/FallbackRequest > $L/fallback.txt 2>&1 &
sleep 5
F=$(inject cb4c-d2 front 2 "$GROUND" 0 2 0 true 3 single_dog_verify 10.0 "")
R=$(inject cb4c-d2 rear 2 "$GROUND" 1 2 0 true 3 single_dog_verify 12.0 "")
if wait_more "進入 WAITING_GROUP" $L/dog_d2.log 0 10 && wait_more "GROUP_INCOMPLETE" $L/dog_d2.log 0 10; then
  ok "D2 只拿一個角色：WAITING_GROUP、GROUP_INCOMPLETE"
else
  bad "D2 WAITING_GROUP／GROUP_INCOMPLETE（看 $L/dog_d2.log）"
fi
if wait_more "FallbackRequest：" $L/gcs_d2.log 0 15 && grep -q "fallback_template: single_dog_verify" $L/fallback.txt; then
  ok "D2 等不到恢復 → 整組取消、FallbackRequest（failure_version 1）"
else
  bad "D2 沒有 FallbackRequest（看 $L/gcs_d2.log、$L/fallback.txt）"
fi
[ "$(current v60)" != "$F" ] && [ "$(current v60)" != "$R" ] && ok "D2 狗沒有拿到 current_task" || bad "D2 狗拿到了 current_task"
stop_all

# ---------- C1：非 all-or-nothing 的依賴失效 ----------
echo "== C1：role 0 空中沒人能做、role 1 依賴它、role 2 不依賴"
start_gcs gcs_c1.log; start_dog dog50_c1.log v60 50 0.0; start_dog dog51_c1.log v60_1 51 5.0
sleep 5
T0=$(date +%s)
A=$(inject cb4c-c1 overwatch 1 "$AIR" 0 3 0 false 6 fb 10.0 "")
G1=$(inject cb4c-c1 front 2 "$GROUND" 1 3 1 false 6 fb 10.0 "")
G2=$(inject cb4c-c1 rear 2 "$GROUND" 2 3 0 false 6 fb 12.0 "")
while [ $(( $(date +%s) - T0 )) -lt 6 ]; do sleep 0.2; done   # 從注入 role 0 起 6 s（< 10 s）
if [ "$(cnt $L/gcs_c1.log "取消 role 0、1")" -eq 0 ]; then ok "C1 沒有能做的載具不是永久：6 s 還沒取消"; else bad "C1 太早取消（看 $L/gcs_c1.log）"; fi
if wait_more "取消 role 0、1" $L/gcs_c1.log 0 15 && wait_more "最初：沒有存活的、平台與能力符合的載具" $L/gcs_c1.log 0 1; then
  ok "C1 等 10 s → RECOVERY_TIMEOUT（最初觸發：沒有能做的載具）→ role 0、1 取消"
else
  bad "C1（看 $L/gcs_c1.log）"
fi
sleep 2
if [ "$(current v60)" = "$G2" ] || [ "$(current v60_1)" = "$G2" ]; then ok "C1 不依賴的 role 2 照做"; else bad "C1 role 2 沒人做"; fi

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
