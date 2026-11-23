#!/bin/bash
# L5 事件狀態機（2026-11-23，docs/l5_l6_plan.md B 批）的節點測試：閘道（200）＋ event_manager（自動派遣）＋ 一隻狗（50），
# 不用 BT（測試代替狗回報），偵測直接發在 /gcs/detections，約 1 分半
#   N1 火的偵測（每秒 2 筆、4 筆）→ 事件 DISPATCHED（fire_response）→ 閘道注入 2 個角色
#   N2 同一處再來 3 筆 → 還是同一個事件，閘道沒有多注入（2-1 案例 2）
#   N3 狗拿到 ground_fire_verify、回報完成 → 事件 RESOLVED
#   N4 Z_HV_A 的入侵（每秒 2 筆、6 筆；上班時段設成空的 = 一律下班）→ 派 perimeter_intrusion（4 個角色）
#   N5 重開 event_manager → 讀回開著的事件
#   N6 值班 /gcs/resolve_incident 標誤報 → FALSE_ALARM
#   N7 一筆弱偵測（0.86 < theta_on）→ 證據消退取消，沒有注入
# 用法（uav 或 gcs 容器裡，編譯完）：bash ros2_ws/src/swarm_events/test/incident_iface.sh；log 在 $OUT（預設 /tmp/cbba_incident）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 89、UDP port 14689、程序鎖與世代檔另開；節點名稱 inct_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-89} CBBA_LOCK_DIR=/tmp/cbba_incident_lock
L=${OUT:-/tmp/cbba_incident}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$L/ev" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
REG=$REPO/registry
SITE=$REPO/site-config/yard
COMMON=(-p udp_port:=14689 -p state_dir:=$L/state -p registry_dir:=$REG -p site_dir:=$SITE)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
has() { clean "$1" | grep -q -- "$2"; }
wait_has() { local t=0; until has "$1" "$2" || [ $t -ge $(( $3 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; has "$1" "$2"; }
stop_events() { pkill -f "__node:=inct_events( |$)" 2>/dev/null; sleep 1; }
stop_all() { pkill -f "__node:=inct_[a-z0-9]+( |$)" 2>/dev/null; pkill -f "timeout 150 ros2 topic echo /gcs/" 2>/dev/null; sleep 1; }
trap stop_all EXIT

start_events() {
  ros2 run swarm_events event_manager --ros-args -r __ns:=/gcs -r __node:=inct_events \
    -p registry_dir:=$REG -p site_dir:=$SITE -p state_dir:=$L/ev -p on_hours_start:=0 -p on_hours_end:=0 >> $L/events.log 2>&1 &
}
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=inct_gcs --params-file $CFG/cbba_gateway.yaml \
  "${COMMON[@]}" > $L/gcs.log 2>&1 &
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=inct_dog --params-file $CFG/cbba_v60.yaml \
  "${COMMON[@]}" -p agent_id:=50 -p participate:=always -p initial_position:="[0.0, -20.0, 0.0]" -p battery:=100.0 \
  > $L/dog.log 2>&1 &
start_events
timeout 150 ros2 topic echo /gcs/incidents swarm_interfaces/msg/EventIncident > $L/incidents.txt 2>&1 &
sleep 6

# detect 偵測器 label 分數 x y track
detect() {
  timeout 10 ros2 topic pub --once -w 1 /gcs/detections swarm_interfaces/msg/Detection \
    "{detector: $1, label: $2, score: $3, source: 'robot_camera:test', track_id: $6, position_valid: true,
      position: {x: $4, y: $5, z: 0.0}, agent_id: 3}" > /dev/null 2>&1
}
# 連續送 N 筆（每秒 2 筆，像真的偵測器的串流）：detect_stream 偵測器 label 分數 x y track 筆數。
# 事件要「分數 ≥ theta_on 持續 min_duration_s」才派遣、超過 1 s 沒有新偵測就開始遺忘，只送一兩筆（CLI 每次開要 1～2 s）撐不住
detect_stream() {
  timeout 20 ros2 topic pub -r 2 -t $7 -w 1 /gcs/detections swarm_interfaces/msg/Detection \
    "{detector: $1, label: $2, score: $3, source: 'robot_camera:test', track_id: $6, position_valid: true,
      position: {x: $4, y: $5, z: 0.0}, agent_id: 3}" > /dev/null 2>&1
}
injected() { clean $L/gcs.log | grep -c "注入 $1.* → 任務"; }
current() { timeout 3 ros2 topic echo --once --field task_id /v60/current_task swarm_interfaces/msg/Task 2>/dev/null | head -1; }

# ---------- N1 ----------
detect_stream apriltag_fire fire 0.9 0.0 -25.0 101 4
if wait_has $L/incidents.txt "rule: fire_response" 8 && wait_has $L/events.log "DISPATCH fire_response" 3; then
  FIRE=$(clean $L/events.log | grep -o "ev[0-9]*_[0-9]* DISPATCH fire_response" | head -1 | cut -d' ' -f1)
  sleep 3
  [ "$(injected "$FIRE")" = "2" ] && ok "N1 火：事件 $FIRE 派 fire_response、閘道注入 2 個角色" ||
    bad "N1 注入數 $(injected "$FIRE")（看 $L/gcs.log）"
else
  bad "N1 沒有派遣（看 $L/events.log、$L/incidents.txt）"
fi

# ---------- N2 ----------
for i in 1 2 3; do detect apriltag_fire fire 0.9 0.3 -25.0 101; sleep 0.4; done
sleep 2
N_FIRE=$(clean $L/events.log | grep -o "ev[0-9]*_[0-9]* 新候選 fire" | wc -l)
[ "$N_FIRE" = "1" ] && [ "$(injected "$FIRE")" = "2" ] && ok "N2 同一處連續告警：還是 1 個事件、沒有多注入" ||
  bad "N2 火的事件 $N_FIRE 個、注入 $(injected "$FIRE") 次（看 $L/events.log）"

# ---------- N3 ----------
T=""
for i in $(seq 1 15); do T=$(current); [ -n "$T" ] && [ "$T" != "0" ] && break; sleep 1; done
if [ -n "$T" ] && [ "$T" != "0" ]; then
  timeout 10 ros2 topic pub --once -w 1 /v60/task_result swarm_interfaces/msg/TaskResult "{task_id: $T, success: true}" > /dev/null 2>&1
  wait_has $L/events.log "$FIRE [A-Z_]* -> RESOLVED" 10 &&
    ok "N3 狗回報完成 → $FIRE RESOLVED" || bad "N3 沒有結案（看 $L/events.log、$L/dog.log）"
else
  bad "N3 狗沒拿到任務（看 $L/dog.log）"
fi

# ---------- N4 ----------
detect_stream person_yolo_v1 person 0.9 -10.0 15.0 77 6
if wait_has $L/events.log "DISPATCH perimeter_intrusion" 8; then
  INTR=$(clean $L/events.log | grep -o "ev[0-9]*_[0-9]* DISPATCH perimeter_intrusion" | head -1 | cut -d' ' -f1)
  sleep 3
  [ "$(injected "$INTR")" = "4" ] && ok "N4 入侵：$INTR 派 perimeter_intrusion、注入 4 個角色" ||
    bad "N4 注入數 $(injected "$INTR")（看 $L/gcs.log）"
else
  bad "N4 沒有派遣（看 $L/events.log）"
fi

# ---------- N5 ----------
stop_events
start_events
wait_has $L/events.log "讀回 .*event_manager_state.yaml" 10 && ok "N5 重開：讀回開著的事件" ||
  bad "N5 沒有讀回（看 $L/events.log）"
sleep 2

# ---------- N6 ----------
ros2 service call /gcs/resolve_incident swarm_interfaces/srv/ResolveIncident \
  "{event_id: '$INTR', false_alarm: true, operator_name: 'op_test', note: 'shadow'}" > $L/resolve.log 2>&1
grep -q "ok=True" $L/resolve.log && grep -q "state='FALSE_ALARM'" $L/resolve.log &&
  ok "N6 值班標誤報 → $INTR FALSE_ALARM" || bad "N6（看 $L/resolve.log）"

# ---------- N7 ----------
BEFORE=$(clean $L/gcs.log | grep -c "注入 .* → 任務")
detect apriltag_fire fire 0.86 25.0 -25.0 0
wait_has $L/events.log "證據消退" 15 && [ "$(clean $L/gcs.log | grep -c "注入 .* → 任務")" = "$BEFORE" ] &&
  ok "N7 弱偵測：證據消退取消、沒有注入" || bad "N7（看 $L/events.log）"

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
