#!/bin/bash
# 任務欄位補齊第一批（2026-11-17：01 識別與追溯、02 任務語意、04 空間；docs/planner_task_definition_plan.md）的節點測試：
# 閘道（給 registry_dir）＋狗（50）的 cbba_node，不用 BT，約 1 分鐘
#   M1 注入帶 meta 的任務：回應有 event_type_id、task_type_id；狗的 current_task.spec 最上層有 incident_id、來源、
#      事件／任務語意（名稱＋編號）、observe_pose、target、規劃器的 steps 不變
#   M2 閘道的 assignments 列出 incident、事件與任務語意（名稱＋編號）
#   M3 不認得的 event_type（不在註冊表）→ 拒絕
#   M4 觀察點離目標不到 standoff_m → 拒絕；moving 沒有 track_id → 拒絕
#   M5 規劃器的 spec 自己帶 incident_id（閘道的鍵）→ 拒絕
#   M6 小幅移動（1 m，revision 2）：狗照舊拿著、current_task 換成新位置
#   M7 閘道重開（同一個 state_dir）：讀回對照表；同一個事件再注入 → 補回來之前拒絕，之後是同一個 task_id（duplicate）
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/meta_iface.sh；log 在 $OUT（預設 /tmp/cbba_meta）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 83、UDP port 14683、程序鎖與世代檔另開；節點名稱 metat_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-83} CBBA_LOCK_DIR=/tmp/cbba_meta_lock
L=${OUT:-/tmp/cbba_meta}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
COMMON=(-p udp_port:=14683 -p state_dir:=$L/state)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
has() { clean "$1" | grep -qF -- "$2"; }
stop_gcs() { pkill -f "__node:=metat_gcs( |$)" 2>/dev/null; sleep 1; }
stop_all() { pkill -f "__node:=metat_[a-z0-9]+( |$)" 2>/dev/null; sleep 1; }
trap stop_all EXIT

start_gcs() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=metat_gcs \
  --params-file $CFG/cbba_gateway.yaml "${COMMON[@]}" -p registry_dir:=$REPO/registry > $L/$1 2>&1 & }
start_dog() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=metat_dog \
  --params-file $CFG/cbba_v60.yaml "${COMMON[@]}" -p participate:=always \
  -p initial_position:="[0.0, 0.0, 0.0]" -p battery:=100.0 > $L/$1 2>&1 & }

STEPS='{"steps": [{"action": "go_to", "location": {"x": 10.0, "y": 0.0}}, {"action": "inspect", "object": "person:trk_88"}]}'
# inject 名稱 事件 revision x 其他欄位（YAML，可空）spec → 回應存在 $L/inject_名稱.log
inject() {
  ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
    "{incident_id: '$2', revision: $3, platform: 2, required_capabilities: ['mobility_ground'], priority: 1,
      position: {x: $4, y: 0.0, z: 0.0}, value: 100.0, service_time_s: 30.0, source: 4,
      event_type: perimeter_intrusion, task_type: confirm_intrusion, description: '入侵：前方確認',
      target_kind: 3, target: {x: 15.0, y: 0.0, z: 0.0}, track_id: trk_88, standoff_m: 3.0 $5, spec: '$6'}" \
    > $L/inject_$1.log 2>&1
}
field() { grep -o "$2=[^,)]*" $L/inject_$1.log | head -1 | cut -d= -f2; }
spec_of() { timeout 3 ros2 topic echo --once --full-length --field spec /v60/current_task swarm_interfaces/msg/Task 2>/dev/null; }
current() { timeout 3 ros2 topic echo --once --field task_id /v60/current_task swarm_interfaces/msg/Task 2>/dev/null | head -1; }
# ros2 topic echo 偶爾在 timeout 內拿不到（剛啟動、探索慢）：判斷用同一次讀到的值，不再另外讀一次
wait_current() {   # wait_current task_id 秒
  local t=0 c
  while [ $t -lt $2 ]; do c=$(current); [ "$c" = "$1" ] && return 0; sleep 1; t=$((t+1)); done
  return 1
}
# wait_x 開頭 秒：current_task 的 position.x 以這個數字開頭（印出讀到的值）
wait_x() {
  local t=0 x
  while [ $t -lt $2 ]; do
    x=$(timeout 3 ros2 topic echo --once --field position.x /v60/current_task swarm_interfaces/msg/Task 2>/dev/null | head -1)
    [ "${x%%.*}" = "$1" ] && { echo "$x"; return 0; }
    sleep 1; t=$((t+1))
  done
  echo "$x"; return 1
}

start_gcs gcs_a.log; start_dog dog_a.log
sleep 5

# ---------- M1、M2 ----------
inject m1 meta-1 1 10.0 "" "$STEPS"
TID=$(field m1 task_id)
if [ "$(field m1 accepted)" = "True" ] && [ "$(field m1 event_type_id)" = "1" ] && [ "$(field m1 task_type_id)" = "4" ]; then
  ok "M1 注入：task $TID、event_type_id 1、task_type_id 4"
else
  bad "M1 注入（看 $L/inject_m1.log）"
fi
if wait_current "$TID" 15; then
  for k in 1 2 3 4 5; do spec_of > $L/spec_m1.txt; [ -s $L/spec_m1.txt ] && break; sleep 1; done
  missing=""
  for part in '"schema_version":"1.1"' '"incident_id":"meta-1"' '"source":"VMS"' '"event_type":"perimeter_intrusion"' \
    '"event_type_id":1' '"task_type":"confirm_intrusion"' '"task_type_id":4' '"observe_pose":{' '"target_kind":"moving"' \
    '"track_id":"trk_88"' '"standoff_m":3.000' '"object": "person:trk_88"'; do
    grep -qF -- "$part" $L/spec_m1.txt || missing="$missing $part"
  done
  [ -z "$missing" ] && ok "M1 current_task.spec 有識別、語意、空間，steps 不變" || bad "M1 spec 缺$missing（看 $L/spec_m1.txt）"
else
  bad "M1 狗沒拿到任務（current $(current)）"
fi
timeout 4 ros2 topic echo --once /gcs/assignments swarm_interfaces/msg/TaskAssignments > $L/assignments.txt 2>/dev/null
if grep -q "incident_id: meta-1" $L/assignments.txt && grep -q "event_type: perimeter_intrusion" $L/assignments.txt &&
  grep -q "task_type_id: 4" $L/assignments.txt && grep -q "source: 4" $L/assignments.txt; then
  ok "M2 assignments 有 incident、事件與任務語意（名稱＋編號）、來源"
else
  bad "M2 assignments（看 $L/assignments.txt）"
fi

# ---------- M3～M5：拒絕 ----------
inject m3 meta-3 1 10.0 ", event_type: alien_landing" "$STEPS"
[ "$(field m3 accepted)" = "False" ] && grep -q "event_type" $L/inject_m3.log && ok "M3 不認得的 event_type 拒絕" ||
  bad "M3（看 $L/inject_m3.log）"
inject m4 meta-4 1 13.0 "" "$STEPS"   # 觀察點 13、目標 15：2 m < 3 m
[ "$(field m4 accepted)" = "False" ] && grep -q "standoff_m" $L/inject_m4.log && ok "M4 觀察點太近拒絕" ||
  bad "M4 standoff（看 $L/inject_m4.log）"
inject m4b meta-4b 1 10.0 ", track_id: ''" "$STEPS"
[ "$(field m4b accepted)" = "False" ] && ok "M4 moving 沒有 track_id 拒絕" || bad "M4 track_id（看 $L/inject_m4b.log）"
inject m5 meta-5 1 10.0 "" '{"incident_id": "other", "steps": [{"action": "inspect"}]}'
[ "$(field m5 accepted)" = "False" ] && grep -q "incident_id" $L/inject_m5.log && ok "M5 spec 自己帶閘道的鍵拒絕" ||
  bad "M5（看 $L/inject_m5.log）"

# ---------- M6：小幅移動 ----------
inject m6 meta-1 2 11.0 "" "$STEPS"
x=$(wait_x 11 10)
if [ "$(field m6 accepted)" = "True" ] && wait_current "$TID" 5 && [ "${x%%.*}" = "11" ] &&
  ! grep -q "terminate called\|Aborted" $L/gcs_a.log; then
  ok "M6 小幅移動（1 m）：狗照舊拿著，current_task 換成 x = $x"
else
  bad "M6（accepted $(field m6 accepted)、current $(current)、x $x；看 $L/inject_m6.log、$L/dog_a.log）"
fi

# ---------- M7：閘道重開 ----------
[ -s $L/state/gateway_200.state ] && ok "M7 對照表存檔 $L/state/gateway_200.state" || bad "M7 沒有存檔（看 $L/state）"
stop_gcs
start_gcs gcs_b.log
sleep 3
has $L/gcs_b.log "讀回" && ok "M7 重開讀回對照表" || bad "M7 讀回（看 $L/gcs_b.log）"
got=""; first=""
for k in $(seq 1 15); do
  inject m7_$k meta-1 2 11.0 "" "$STEPS"
  [ -z "$first" ] && first=$(field m7_$k accepted)
  if [ "$(field m7_$k accepted)" = "True" ]; then got=$k; break; fi
  sleep 1
done
if [ -n "$got" ] && [ "$(field m7_$got task_id)" = "$TID" ] && [ "$(field m7_$got duplicate)" = "True" ]; then
  ok "M7 重開後同一個事件：同一個 task_id $TID（duplicate；第 $got 次，第一次 accepted=$first）"
else
  bad "M7 重開後（看 $L/inject_m7_*.log、$L/gcs_b.log）"
fi

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
