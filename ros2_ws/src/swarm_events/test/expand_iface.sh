#!/bin/bash
# L6 範本展開服務（2026-11-23，docs/l5_l6_plan.md A 批）的節點測試：event_manager＋閘道（機號 200），不用 BT，約 30 秒
#   X1 入侵（空地）→ ok、4 個角色、front／rear 限狗、空中 optional、規則 perimeter_intrusion
#   X2 入侵在 Z_HV_A 裡 → front、rear 用那個區域的觀察點（poi:OBS_HV_A_FRONT／REAR）
#   X3 火警 → 2 個角色、report_policy 有 gcs、operator
#   X4 降級規則 single_dog_verify → 1 個角色 verify
#   X5 熱異常（設備 machine:TA2）→ FLEET_INFEASIBLE（yard 的機隊沒有熱像儀）；不認得的事件類型 → UNKNOWN_EVENT_TYPE
#   X6 只回傳規劃結果、不注入：展開了幾次，閘道仍然沒有任何任務（log 沒有「注入」、assignments 沒有任務）
# 用法（uav 或 gcs 容器裡，編譯完）：bash ros2_ws/src/swarm_events/test/expand_iface.sh；log 在 $OUT（預設 /tmp/cbba_expand）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 88、UDP port 14688、程序鎖與世代檔另開；節點名稱 expt_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-88} CBBA_LOCK_DIR=/tmp/cbba_expand_lock
L=${OUT:-/tmp/cbba_expand}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
REG=$REPO/registry
SITE=$REPO/site-config/yard
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
stop_all() { pkill -f "__node:=expt_[a-z0-9]+( |$)" 2>/dev/null; pkill -f "timeout 60 ros2 topic echo /gcs/assignments" 2>/dev/null; sleep 1; }
trap stop_all EXIT

ros2 run swarm_events event_manager --ros-args -r __ns:=/gcs -r __node:=expt_events \
  -p registry_dir:=$REG -p site_dir:=$SITE > $L/events.log 2>&1 &
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=expt_gcs --params-file $CFG/cbba_gateway.yaml \
  -p udp_port:=14688 -p state_dir:=$L/state -p registry_dir:=$REG -p site_dir:=$SITE > $L/gcs.log 2>&1 &
timeout 60 ros2 topic echo /gcs/assignments swarm_interfaces/msg/TaskAssignments > $L/assignments.txt 2>&1 &
sleep 5

# expand 名稱 事件類型 x y 其他欄位（YAML，可空）
expand() {
  ros2 service call /gcs/expand_incident swarm_interfaces/srv/ExpandIncident \
    "{event_id: '$1', event_type: '$2', location: {x: $3, y: $4, z: 0.0} $5}" > $L/expand_$1.log 2>&1
}
# 只看回應（ros2 service call 會先印請求，請求裡也有 rule=''、event_type=… 這些欄位）
resp() { sed -n '/^response:/,$p' $L/expand_$1.log; }
field() { resp $1 | grep -o "$2=[^,)]*" | head -1 | cut -d= -f2 | tr -d "'"; }
count() { resp $1 | grep -o "role='[a-z_]*'" | wc -l; }
has() { resp $1 | grep -q -- "$2"; }

# ---------- X1 ----------
expand x1 perimeter_intrusion -20.0 8.0
if [ "$(field x1 ok)" = "True" ] && [ "$(count x1)" = "4" ] && [ "$(field x1 rule)" = "perimeter_intrusion" ] &&
  has x1 "role='front'" && has x1 "role='observer'"; then
  ok "X1 入侵：4 個角色（空中、front、rear、observer）"
else
  bad "X1（看 $L/expand_x1.log）"
fi

# ---------- X2 ----------
expand x2 perimeter_intrusion -10.0 15.0
has x2 "location_source='poi:OBS_HV_A_FRONT'" && has x2 "location_source='poi:OBS_HV_A_REAR'" &&
  ok "X2 事件在 Z_HV_A：front、rear 用那個區域的觀察點" || bad "X2（看 $L/expand_x2.log）"

# ---------- X3 ----------
expand x3 fire 20.0 -25.0
[ "$(field x3 ok)" = "True" ] && [ "$(count x3)" = "2" ] && has x3 'gcs' && has x3 'operator' &&
  ok "X3 火警：2 個角色、report_policy" || bad "X3（看 $L/expand_x3.log）"

# ---------- X4 ----------
expand x4 fire 0.0 -25.0 ", rule: 'single_dog_verify'"
[ "$(field x4 ok)" = "True" ] && [ "$(count x4)" = "1" ] && has x4 "role='verify'" &&
  ok "X4 降級規則 single_dog_verify：1 個角色" || bad "X4（看 $L/expand_x4.log）"

# ---------- X5 ----------
expand x5 thermal_anomaly -20.0 15.0 ", asset: 'machine:TA2'"
expand x5b no_such_event 0.0 0.0
[ "$(field x5 ok)" = "False" ] && [ "$(field x5 code)" = "FLEET_INFEASIBLE" ] && [ "$(field x5b code)" = "UNKNOWN_EVENT_TYPE" ] &&
  ok "X5 機隊沒有熱像儀 → FLEET_INFEASIBLE；不認得的事件 → UNKNOWN_EVENT_TYPE" || bad "X5（看 $L/expand_x5*.log）"

# ---------- X6 ----------
sleep 3
if ! clean $L/gcs.log | grep -q "注入 .* → 任務" && ! grep -q "task_id: [1-9]" $L/assignments.txt; then
  ok "X6 只回傳規劃結果：閘道沒有任何任務"
else
  bad "X6 閘道收到任務了（看 $L/gcs.log、$L/assignments.txt）"
fi

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
