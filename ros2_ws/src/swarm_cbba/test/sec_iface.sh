#!/bin/bash
# 任務欄位補齊第五批（2026-11-18：13 簽章與資安，協定版本 5）的節點測試：閘道（200）＋狗（50）＋無人機（1）＋狗（51），
# 金鑰檔臨時產生，不用 BT，約 1 分鐘
#   S1 security_mode require 啟動；金鑰檔裡沒有自己的金鑰 → 拒絕啟動
#   S2 閘道注入的任務（有簽章）→ 狗驗過、拿到任務
#   S3 未授權的 issuer：task_issuers = [200]，無人機自己建的任務（new_task，簽章對）→ 狗丟掉
#   S4 沒簽章：狗 51 用 security_mode none（不簽）建的任務 → REQUIRE 的狗丟掉
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/sec_iface.sh；log 在 $OUT（預設 /tmp/cbba_sec）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 87、UDP port 14687、程序鎖與世代檔另開；節點名稱 sect_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-87} CBBA_LOCK_DIR=/tmp/cbba_sec_lock
L=${OUT:-/tmp/cbba_sec}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
KEYS=$L/keys.txt
{ echo "# 測試用金鑰（每次隨機產生）：機號 key_id 金鑰"
  for a in 200 1 50; do echo "$a 1 $(head -c 32 /dev/urandom | od -An -tx1 | tr -d ' \n')"; done; } > $KEYS
SEC=(-p security_mode:=require -p security_keys_file:=$KEYS -p task_issuers:="[200, 1]")
COMMON=(-p udp_port:=14687 -p state_dir:=$L/state)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
has() { clean "$1" | grep -qF -- "$2"; }
wait_has() { local t=0; until has "$2" "$1" || [ $t -ge $(( $3 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; has "$2" "$1"; }
# wait_re 正規式 檔案 秒
wait_re() { local t=0; until clean "$2" | grep -qE -- "$1" || [ $t -ge $(( $3 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; clean "$2" | grep -qE -- "$1"; }
alive() { pgrep -f "__node:=sect_$1( |$)" > /dev/null; }
stop_all() { pkill -f "__node:=sect_[a-z0-9]+( |$)" 2>/dev/null; sleep 1; }
trap stop_all EXIT

ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=sect_gcs --params-file $CFG/cbba_gateway.yaml \
  "${COMMON[@]}" "${SEC[@]}" > $L/gcs.log 2>&1 &
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=sect_dog --params-file $CFG/cbba_v60.yaml \
  "${COMMON[@]}" "${SEC[@]}" -p agent_id:=50 -p participate:=always -p initial_position:="[0.0, 0.0, 0.0]" \
  -p battery:=100.0 > $L/dog.log 2>&1 &
# 金鑰檔裡沒有機號 7 → 拒絕啟動
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60_7 -r __node:=sect_nokey --params-file $CFG/cbba_v60.yaml \
  "${COMMON[@]}" "${SEC[@]}" -p agent_id:=7 -p participate:=always > $L/nokey.log 2>&1 &
sleep 5

# ---------- S1 ----------
has $L/dog.log "簽章：require，金鑰 3 把" && has $L/gcs.log "簽章：require" && ok "S1 require 模式啟動" ||
  bad "S1 啟動（看 $L/dog.log、$L/gcs.log）"
has $L/nokey.log "has no signing key for agent 7" && ! alive nokey && ok "S1 沒有自己的金鑰：拒絕啟動" ||
  bad "S1 沒有金鑰（看 $L/nokey.log）"

# ---------- S2 ----------
ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
  "{incident_id: 's2', revision: 1, platform: 2, required_capabilities: ['mobility_ground'], priority: 1,
    position: {x: 10.0, y: 0.0, z: 0.0}, value: 100.0, service_time_s: 30.0}" > $L/inject_s2.log 2>&1
TID=$(grep -o "task_id=[0-9]*" $L/inject_s2.log | head -1 | cut -d= -f2)
wait_re "指派給 BT：[0-9A-F]{4}\\." $L/dog.log 10 &&   # 啟動時那行「指派給 BT：無」不算
  ok "S2 閘道注入（有簽章）→ 狗驗過、拿到任務" || bad "S2（看 $L/dog.log、$L/inject_s2.log）"

# ---------- S3：task_issuers 換成只有閘道，重開狗 ----------
pkill -f "__node:=sect_dog( |$)"; sleep 1
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=sect_dog --params-file $CFG/cbba_v60.yaml \
  "${COMMON[@]}" -p security_mode:=require -p security_keys_file:=$KEYS -p task_issuers:="[200]" \
  -p agent_id:=50 -p participate:=always -p initial_position:="[0.0, 0.0, 0.0]" -p battery:=100.0 > $L/dog_b.log 2>&1 &
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/uav1 -r __node:=sect_uav --params-file $CFG/cbba_uav.yaml \
  "${COMMON[@]}" "${SEC[@]}" -p agent_id:=1 -p participate:=always -p initial_position:="[0.0, 5.0, 5.0]" \
  -p battery:=100.0 > $L/uav.log 2>&1 &
sleep 5
timeout 10 ros2 topic pub --once -w 1 /uav1/new_task swarm_interfaces/msg/Task \
  "{task_id: $((1 * 65536 + 9)), type: 1, position: {x: 5.0, y: 5.0, z: 5.0}, value: 100.0, duration_sec: 5.0, deadline_sec: 120.0}" \
  > /dev/null 2>&1
wait_re "未授權的 issuer [1-9]" $L/dog_b.log 10 && ok "S3 無人機自己建的任務（不在 task_issuers）：狗丟掉" ||
  bad "S3（看 $L/dog_b.log、$L/uav.log）"

# ---------- S4 ----------
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60_1 -r __node:=sect_dog51 --params-file $CFG/cbba_v60.yaml \
  "${COMMON[@]}" -p security_mode:=none -p agent_id:=51 -p participate:=always -p initial_position:="[20.0, 0.0, 0.0]" \
  -p battery:=100.0 > $L/dog51.log 2>&1 &
sleep 4
timeout 10 ros2 topic pub --once -w 1 /v60_1/new_task swarm_interfaces/msg/Task \
  "{task_id: $((51 * 65536 + 3)), type: 2, position: {x: 20.0, y: 0.0, z: 0.0}, value: 100.0, duration_sec: 20.0, deadline_sec: 120.0}" \
  > /dev/null 2>&1
wait_re "沒簽章 [1-9]" $L/dog_b.log 20 && ok "S4 沒簽章的任務：REQUIRE 的狗丟掉" || bad "S4（看 $L/dog_b.log、$L/dog51.log）"

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
