#!/bin/bash
# ExecuteStep.action 草案＋假 adapter（2026-11-19）的節點測試：fake_adapter（狗、無人機各一），用 ros2 action send_goal
# 代替 BT 的 ExecuteSteps，約 1.5 分鐘
#   A1 狗 go_to：有回饋（phase moving、progress）、SUCCEEDED
#   A2 無人機 go_to：LANDED 時自動起飛（phase takeoff）、SUCCEEDED；return_to_dock → 降落、平台 LANDED
#   A3 不合法的一步（不認得的動作、參數超出範圍）→ FAILED、E_PRECONDITION（和閘道同一套檢查）
#   A4 取消（BT 的 halt）：rclpy 的 action client 送 goal、2 s 後取消 → CANCELED
#   A5 故障注入（執行中 ros2 param set faults）：inspect → FAILED、E_HW、failed_capability thermal_camera
#   A6 逾時：goal 帶 deadline（3 s 後）、目標很遠 → TIMEOUT、E_TIMEOUT（slowdown 的逾時在 test_fake_executor）
#   A7 執行中收到新的 goal：舊的結束（CANCELED、「被新的 goal 取代」），新的照做
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/exec_step_iface.sh；log 在 $OUT（預設 /tmp/cbba_exst）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 88；節點名稱 exst_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-88}
L=${OUT:-/tmp/cbba_exst}; rm -rf "$L"; mkdir -p "$L"
REG=$REPO/registry
SITE=$REPO/site-config/yard
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
has() { clean "$1" | grep -qF -- "$2"; }
cleanup() {
  pkill -f "__node:=exst_[a-z0-9]+( |$)" 2>/dev/null
  sleep 0.5
}
trap cleanup EXIT

ros2 run swarm_cbba fake_adapter --ros-args -r __ns:=/exst_dog -r __node:=exst_dog \
  -p platform:=quadruped -p registry_dir:=$REG -p site_dir:=$SITE > $L/dog.log 2>&1 &
ros2 run swarm_cbba fake_adapter --ros-args -r __ns:=/exst_uav -r __node:=exst_uav \
  -p platform:=uav -p speed:=2.0 -p registry_dir:=$REG > $L/uav.log 2>&1 &
sleep 4

# goal 名稱 YAML 輸出檔：送一個 goal、等結果（--feedback）；最多 60 s
goal() {
  timeout 60 ros2 action send_goal --feedback /exst_$1/execute_step swarm_interfaces/action/ExecuteStep "$2" > "$3" 2>&1
}

# ---------- A1 ----------
goal dog '{task_id: 1, step_id: 0, action: go_to, location_json: "{\"x\": 3.0, \"y\": 0.0}"}' $L/a1.out
if has $L/a1.out "phase: moving" && has $L/a1.out "status: 0" && has $L/a1.out "SUCCEEDED"; then
  ok "A1 狗 go_to：回饋 moving、SUCCEEDED"; else bad "A1（看 $L/a1.out）"; fi

# ---------- A2 ----------
goal uav '{task_id: 2, step_id: 0, action: go_to, location_json: "{\"x\": 2.0, \"y\": 0.0}"}' $L/a2a.out
goal uav '{task_id: 2, step_id: 1, action: return_to_dock}' $L/a2b.out
if has $L/a2a.out "phase: takeoff" && has $L/a2a.out "platform_state: AIRBORNE" && has $L/a2a.out "SUCCEEDED" &&
  has $L/a2b.out "phase: landing" && has $L/a2b.out "SUCCEEDED" && has $L/uav.log "return_to_dock：SUCCEEDED，平台 LANDED"; then
  ok "A2 無人機：自動起飛、回起點降落（LANDED）"; else bad "A2（看 $L/a2a.out、$L/a2b.out、$L/uav.log）"; fi

# ---------- A3 ----------
goal dog '{task_id: 3, action: fly_away}' $L/a3a.out
goal dog '{task_id: 3, action: inspect, params_json: "{\"duration_s\": 1000}"}' $L/a3b.out
if has $L/a3a.out "status: 1" && has $L/a3a.out "E_PRECONDITION" && has $L/a3a.out "UNKNOWN_ACTION" &&
  has $L/a3b.out "PARAM_OUT_OF_RANGE"; then
  ok "A3 不合法的一步 → FAILED、E_PRECONDITION（UNKNOWN_ACTION、PARAM_OUT_OF_RANGE）"; else bad "A3（看 $L/a3a.out、$L/a3b.out）"; fi

# ---------- A4 ----------
# ros2 action send_goal 收到 SIGINT 不一定會取消（從腳本送時 goal 照樣做完），所以用 rclpy 的 client 明確取消
timeout 30 python3 - > $L/a4.out 2>&1 <<'PY'
import time
import rclpy
from rclpy.action import ActionClient
from swarm_interfaces.action import ExecuteStep
rclpy.init()
node = rclpy.create_node('exst_cancel_client')
client = ActionClient(node, ExecuteStep, '/exst_dog/execute_step')
client.wait_for_server(timeout_sec=10.0)
goal = ExecuteStep.Goal(task_id=4, action='go_to', location_json='{"x": 100.0, "y": 0.0}')
f = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
handle = f.result()
end = time.time() + 2.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
c = handle.cancel_goal_async()
rclpy.spin_until_future_complete(node, c, timeout_sec=5.0)
r = handle.get_result_async()
rclpy.spin_until_future_complete(node, r, timeout_sec=10.0)
res = r.result()
print(f'goal_status: {res.status} result_status: {res.result.status} detail: {res.result.detail}')
PY
if has $L/a4.out "result_status: 2" && has $L/dog.log "go_to：CANCELED（BT 取消）"; then
  ok "A4 取消 → CANCELED"; else bad "A4（看 $L/a4.out、$L/dog.log）"; fi

# ---------- A5 ----------
ros2 param set /exst_dog/exst_dog faults "['inspect=E_HW:thermal_camera@0.3']" > $L/a5_param.out 2>&1
goal dog '{task_id: 5, action: inspect, params_json: "{\"mode\": \"thermal\", \"duration_s\": 4}"}' $L/a5.out
ros2 param set /exst_dog/exst_dog faults "['none=E_NONE']" > /dev/null 2>&1   # 清掉（none 不是動作名稱）
if has $L/a5.out "status: 1" && has $L/a5.out "E_HW" && has $L/a5.out "thermal_camera"; then
  ok "A5 故障注入 → FAILED、E_HW、thermal_camera"; else bad "A5（看 $L/a5.out）"; fi

# ---------- A6 ----------
DEADLINE=$(( $(date +%s) + 3 ))
goal dog "{task_id: 6, action: go_to, location_json: '{\"x\": 300.0, \"y\": 0.0}', deadline: {sec: $DEADLINE, nanosec: 0}}" $L/a6.out
if has $L/a6.out "status: 3" && has $L/a6.out "E_TIMEOUT"; then
  ok "A6 逾時 → TIMEOUT、E_TIMEOUT"; else bad "A6（看 $L/a6.out）"; fi

# ---------- A7 ----------
timeout 60 ros2 action send_goal /exst_dog/execute_step swarm_interfaces/action/ExecuteStep \
  '{task_id: 7, step_id: 0, action: go_to, location_json: "{\"x\": 300.0, \"y\": 0.0}"}' > $L/a7a.out 2>&1 &
A7A=$!
sleep 3
goal dog '{task_id: 7, step_id: 1, action: wait, params_json: "{\"duration_s\": 1.0}"}' $L/a7b.out
wait $A7A   # 只等這一個（fake_adapter 也是背景程式，等全部會卡住）
if has $L/a7a.out "status: 2" && has $L/a7a.out "被新的 goal 取代" && has $L/a7b.out "SUCCEEDED"; then
  ok "A7 新的 goal 取代舊的（CANCELED）、新的照做"; else bad "A7（看 $L/a7a.out、$L/a7b.out）"; fi

echo "----"
echo "PASS $PASS  FAIL $FAIL   （log：$L）"
[ $FAIL -eq 0 ]
