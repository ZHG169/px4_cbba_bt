#!/bin/bash
# 偵測送閘道（2-1b M0 第 5 步，2026-11-14）的測試：閘道＋一隻狗的 cbba_node（用 ros2 topic pub 假裝偵測器），約 40 s
#   D1 狗的偵測送到閘道：閘道 log、/gcs/detections 的欄位（agent_id、label、track_id、attrs）
#   D2 閘道關掉：狗印「閘道斷線」，偵測只暫存；同一個 track 的合併成一筆
#   D3 閘道重開：狗印「閘道在線：送出暫存的 N 筆」，閘道收到、標「斷線期間暫存的」
#   D4 太長的字串拒收
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/det_iface.sh；log 在 $OUT（預設 /tmp/cbba_det）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 78、UDP port 14678、程序鎖與世代檔另開；節點名稱 dett_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-78} CBBA_LOCK_DIR=/tmp/cbba_det_lock
L=${OUT:-/tmp/cbba_det}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
COMMON=(-p udp_port:=14678 -p state_dir:=$L/state)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
cnt() { clean "$1" | grep -cF -- "$2"; }
# wait_more 字串 檔案 已有幾次 秒：等到出現次數超過「已有幾次」
wait_more() { local t=0; until [ "$(cnt "$2" "$1")" -gt "$3" ] || [ $t -ge $(( $4 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; [ "$(cnt "$2" "$1")" -gt "$3" ]; }
check() { wait_more "$2" "$3" "${5:-0}" "$4" && ok "$1" || bad "$1（等不到「$2」）"; }
gcs() {
  ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=dett_gcs \
    --params-file $CFG/cbba_gateway.yaml "${COMMON[@]}" >> $L/gcs.log 2>&1 &
}
stop_gcs() { pkill -f "__node:=dett_gcs( |$)" 2>/dev/null; sleep 1; }
cleanup() {
  pkill -f "__node:=dett_(gcs|v60)( |$)" 2>/dev/null
  [ -n "$ECHO" ] && kill $ECHO 2>/dev/null
  sleep 0.5
}
trap cleanup EXIT

detect() {   # detect x y track_id [label]
  ros2 topic pub --once -w 1 /v60/detection swarm_interfaces/msg/Detection \
    "{detector: apriltag_fire, source: v60/front_cam, label: '${4:-fire}', score: 0.9, track_id: $3,
      position_valid: true, position: {x: $1, y: $2, z: 0.0}, attrs: '{\"tag\":101}'}" >> $L/pub.log 2>&1
}

gcs
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=dett_v60 \
  --params-file $CFG/cbba_v60.yaml "${COMMON[@]}" -p participate:=always \
  -p initial_position:="[0.0, 0.0, 0.0]" -p battery:=100.0 > $L/v60.log 2>&1 &
sleep 2
ros2 topic echo /gcs/detections swarm_interfaces/msg/Detection > $L/detections.txt 2>&1 &
ECHO=$!
sleep 3

# ---------- D1：送到閘道 ----------
detect 12.0 -4.0 9
check "D1 閘道收到 agent 50 #1" "偵測 agent 50 #1：apriltag_fire/fire" $L/gcs.log 5
sleep 1
for f in "agent_id: 50" "label: fire" "track_id: 9" "buffered: false" "tag"; do
  grep -qF -- "$f" $L/detections.txt && ok "D1 detections 有「$f」" || bad "D1 detections 沒有「$f」"
done

# ---------- D2：閘道關掉 → 暫存、合併 ----------
stop_gcs
check "D2 狗發現閘道斷線" "閘道斷線（" $L/v60.log 6
detect 20.0 0.0 0          # 沒有 track_id
detect 30.0 0.0 7          # track 7
detect 35.0 0.0 7          # 同一個 track（目標移動了）→ 合併
check "D2 暫存" "暫存（2 筆" $L/v60.log 5
sleep 1
[ "$(cnt $L/v60.log "暫存（3 筆")" -eq 0 ] && ok "D2 同一個 track 合併（沒有變成 3 筆）" || bad "D2 同一個 track 沒合併"
N_RECV=$(cnt $L/gcs.log "斷線期間暫存的")
[ "$N_RECV" -eq 0 ] && ok "D2 斷線期間閘道沒收到" || bad "D2 斷線期間閘道收到 $N_RECV 筆"

# ---------- D3：閘道重開 → 送出暫存的 ----------
gcs
check "D3 狗發現閘道在線、送出暫存的 2 筆" "送出暫存的 2 筆偵測" $L/v60.log 10
check "D3 閘道收到暫存的偵測（2 筆）" "斷線期間暫存的" $L/gcs.log 5 1
sleep 1
N_BUF=$(cnt $L/gcs.log "斷線期間暫存的")
[ "$N_BUF" -eq 2 ] && ok "D3 剛好 2 筆（合併後）" || bad "D3 收到 $N_BUF 筆暫存的（應為 2）"
clean $L/gcs.log | grep -F "斷線期間暫存的" | grep -qF "（35.0, 0.0" && ok "D3 合併留下最新的（35, 0）" \
  || bad "D3 合併後的位置不是最新的"

# ---------- D4：太長的字串 ----------
detect 1.0 1.0 0 "$(printf 'x%.0s' {1..65})"
check "D4 太長的 label 拒收" "拒收：detector、source、label 要 ≤ 64 B" $L/v60.log 5

echo "----"
echo "PASS $PASS  FAIL $FAIL   （log：$L）"
[ $FAIL -eq 0 ]
