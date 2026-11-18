#!/bin/bash
# 任務欄位補齊第三批（2026-11-18：10 執行規格、12 人機與安全）的節點測試：閘道（registry_dir、site_dir）＋狗（50），
# 不用 BT，約 1 分鐘
#   E1 不認得的動作 → 閘道拒絕（UNKNOWN_ACTION，不會到 BT 才 BAD_SPEC）
#   E2 參數超出範圍、缺 required 參數 → 拒絕
#   E3 不認得的參數（狗的舊名稱 snapshots）→ 照收、回應有 warnings
#   E4 任務的 timeout_s 5 s：狗拿到後 5 s 沒回報 → cbba_node 當成 E_TIMEOUT 失敗、釋放
#   E5 requires_approval：回 waiting_approval、沒建任務；approve_task 核准 → 注入、狗拿到；approvals 話題、稽核紀錄
#   E6 拒絕 → REJECTED、不建任務；同一個鍵再送回同一張核准單
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/exec_iface.sh；log 在 $OUT（預設 /tmp/cbba_exec）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 85、UDP port 14685、程序鎖與世代檔另開；節點名稱 exect_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-85} CBBA_LOCK_DIR=/tmp/cbba_exec_lock
L=${OUT:-/tmp/cbba_exec}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
COMMON=(-p udp_port:=14685 -p state_dir:=$L/state -p registry_dir:=$REPO/registry -p site_dir:=$REPO/site-config/yard)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
has() { clean "$1" | grep -qF -- "$2"; }
wait_has() { local t=0; until has "$2" "$1" || [ $t -ge $(( $3 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; has "$2" "$1"; }
stop_all() { pkill -f "__node:=exect_[a-z0-9]+( |$)" 2>/dev/null; pkill -f "timeout 120 ros2 topic echo /gcs/approvals" 2>/dev/null; sleep 1; }
trap stop_all EXIT

ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=exect_gcs --params-file $CFG/cbba_gateway.yaml \
  "${COMMON[@]}" > $L/gcs.log 2>&1 &
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=exect_dog --params-file $CFG/cbba_v60.yaml \
  "${COMMON[@]}" -p agent_id:=50 -p participate:=always -p initial_position:="[0.0, 25.0, 0.0]" -p battery:=100.0 \
  > $L/dog.log 2>&1 &
timeout 120 ros2 topic echo /gcs/approvals swarm_interfaces/msg/ApprovalState > $L/approvals.txt 2>&1 &
sleep 5

# inject 名稱 spec 其他欄位（YAML，可空）→ 回應存在 $L/inject_名稱.log
inject() {
  ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
    "{incident_id: '$1', revision: 1, platform: 2, required_capabilities: ['mobility_ground'], priority: 1,
      position: {x: 10.0, y: 25.0, z: 0.0}, value: 100.0, service_time_s: 30.0, spec: '$2' $3}" > $L/inject_$1.log 2>&1
}
field() { grep -o "$2=[^,)]*" $L/$1.log | head -1 | cut -d= -f2; }
current() { timeout 3 ros2 topic echo --once --field task_id /v60/current_task swarm_interfaces/msg/Task 2>/dev/null | head -1; }
wait_current() {
  local t=0 c
  while [ $t -lt $2 ]; do c=$(current); [ "$c" = "$1" ] && return 0; sleep 1; t=$((t+1)); done
  return 1
}
GOOD='{"steps": [{"action": "go_to", "location": {"x": 10.0, "y": 25.0}}, {"action": "inspect", "object": "person"}]}'

# ---------- E1～E3 ----------
inject e1 '{"steps": [{"action": "dance"}]}'
[ "$(field inject_e1 accepted)" = "False" ] && grep -q "UNKNOWN_ACTION" $L/inject_e1.log && ok "E1 不認得的動作：拒絕" ||
  bad "E1（看 $L/inject_e1.log）"
inject e2 '{"steps": [{"action": "inspect", "params": {"duration_s": 90}}]}'
[ "$(field inject_e2 accepted)" = "False" ] && grep -q "PARAM_OUT_OF_RANGE" $L/inject_e2.log && ok "E2 參數超出範圍：拒絕" ||
  bad "E2 範圍（看 $L/inject_e2.log）"
inject e2b '{"steps": [{"action": "go_to"}]}'
[ "$(field inject_e2b accepted)" = "False" ] && grep -q "LOCATION_MISSING" $L/inject_e2b.log && ok "E2 go_to 沒有 location：拒絕" ||
  bad "E2 location（看 $L/inject_e2b.log）"
inject e3 '{"steps": [{"action": "go_to", "location": {"x": 10.0, "y": 25.0}}, {"action": "inspect", "params": {"snapshots": 2}}]}'
[ "$(field inject_e3 accepted)" = "True" ] && grep -q "PARAM_UNKNOWN" $L/inject_e3.log && ok "E3 不認得的參數：照收、回警告" ||
  bad "E3（看 $L/inject_e3.log）"
E3=$(field inject_e3 task_id)
# 先把 E3 取消，免得狗手上一直有它
ros2 topic pub --once -w 1 /gcs/cancel_task swarm_interfaces/msg/Task "{task_id: $E3}" > /dev/null 2>&1
sleep 2

# ---------- E4 ----------
inject e4 "${GOOD%\}}, \"timeout_s\": 5}"
E4=$(field inject_e4 task_id)
if wait_current "$E4" 10 && wait_has "超過任務的 timeout_s 5 s" $L/dog.log 10; then
  ok "E4 任務 timeout_s：超過 5 s 當成 E_TIMEOUT 失敗"
else
  bad "E4 逾時（current $(current)；看 $L/dog.log、$L/inject_e4.log）"
fi
has $L/dog.log "failure_code E_TIMEOUT → reason TRANSIENT" || has $L/dog.log "failure_code「E_TIMEOUT」" &&
  ok "E4 走一般的失敗流程（failure_code E_TIMEOUT）" || bad "E4 失敗流程（看 $L/dog.log）"
sleep 2

# ---------- E5 ----------
inject e5 "$GOOD" ", requires_approval: true"
AID=$(field inject_e5 approval_id)
if [ "$(field inject_e5 waiting_approval)" = "True" ] && [ "$(field inject_e5 task_id)" = "0" ]; then
  ok "E5 要核准：回 waiting_approval（核准單 $AID），沒建任務"
else
  bad "E5 等核准（看 $L/inject_e5.log）"
fi
sleep 2
if clean $L/gcs.log | grep -q "注入 e5.*→ 任務"; then bad "E5 核准前就建了任務"; else ok "E5 核准前沒有注入"; fi
ros2 service call /gcs/approve_task swarm_interfaces/srv/ApproveTask \
  "{approval_id: $AID, approve: true, operator_name: 'op_test', note: 'e5 ok'}" > $L/approve_e5.log 2>&1
E5=$(grep -o "task_ids=\[[0-9]*" $L/approve_e5.log | grep -o "[0-9]*$")
if [ "$(field approve_e5 ok)" = "True" ] && [ -n "$E5" ] && [ "$E5" != "0" ] && wait_current "$E5" 15; then
  ok "E5 核准 → 注入，狗拿到任務 $E5"
else
  bad "E5 核准（看 $L/approve_e5.log、$L/gcs.log）"
fi
grep -q "state: 1" $L/approvals.txt && grep -q "state: 3" $L/approvals.txt && ok "E5 approvals 話題：WAITING_APPROVAL → INJECTED" ||
  bad "E5 approvals（看 $L/approvals.txt）"
grep -q "INJECTED.*op_test" $L/state/gateway_200_approvals.log && ok "E5 稽核紀錄有 INJECTED 與操作者" ||
  bad "E5 稽核紀錄（看 $L/state/gateway_200_approvals.log）"

# ---------- E6 ----------
inject e6 "$GOOD" ", requires_approval: true"
AID6=$(field inject_e6 approval_id)
ros2 service call /gcs/approve_task swarm_interfaces/srv/ApproveTask \
  "{incident_id: 'e6', approve: false, operator_name: 'op_test', note: 'false alarm'}" > $L/approve_e6.log 2>&1
inject e6 "$GOOD" ", requires_approval: true"
mv $L/inject_e6.log $L/inject_e6_again.log
if [ "$(field approve_e6 ok)" = "True" ] && grep -q "REJECTED.*false alarm" $L/state/gateway_200_approvals.log &&
  [ "$(field inject_e6_again approval_id)" = "$AID6" ] && [ "$(field inject_e6_again duplicate)" = "True" ]; then
  ok "E6 拒絕 → REJECTED；同一個鍵再送回同一張核准單 $AID6"
else
  bad "E6（看 $L/approve_e6.log、$L/inject_e6_again.log）"
fi

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
