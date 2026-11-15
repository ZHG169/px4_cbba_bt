#!/bin/bash
# 註冊表（2026-11-16 照 2-1b 原文改寫）給 cbba_node 載入的測試：不用 BT，約 30 s
#   R1 狗（50）給 registry_dir：從 fleet.yaml 讀到 dog1（quadruped）、能力，節點正常跑
#   R2 無人機（1）：uav1（uav）、platforms.yaml 的 max_bundle 2
#   R3 閘道（200）：在 fleet.yaml 的 gateways，沒有警告
#   R4 不在 fleet.yaml 的機號（60）：警告、照參數，節點照常跑
#   R5 註冊表有錯（能力的 bit 和程式不一樣）：印錯誤、拒絕啟動
#   R6 註冊表的格式錯（actions.yaml 是舊的清單格式）：拒絕啟動
#   R7 平台和 agent_type 不一致（機號 1 用狗的 yaml）：拒絕啟動
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/reg_load.sh；log 在 $OUT（預設 /tmp/cbba_reg）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 79、UDP port 14679、程序鎖與世代檔另開；節點名稱 regt_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-79} CBBA_LOCK_DIR=/tmp/cbba_reg_lock
L=${OUT:-/tmp/cbba_reg}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
REG=$REPO/registry
COMMON=(-p udp_port:=14679 -p state_dir:=$L/state)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
has() { clean "$1" | grep -qF -- "$2"; }
cleanup() {
  pkill -f "__node:=regt_[a-z0-9]+( |$)" 2>/dev/null
  sleep 0.5
}
trap cleanup EXIT

# start 名稱 log 參數…：背景開一個節點
start() {
  local name=$1 log=$2; shift 2
  ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/$name -r __node:=regt_$name "$@" "${COMMON[@]}" > "$log" 2>&1 &
}
# alive 名稱：節點還在跑
alive() { pgrep -f "__node:=regt_$1( |$)" > /dev/null; }

# ---------- R1～R4：正確的註冊表 ----------
start dog $L/r1.log --params-file $CFG/cbba_v60.yaml -p agent_id:=50 -p participate:=always \
  -p initial_position:="[0.0, 0.0, 0.0]" -p battery:=100.0 -p registry_dir:=$REG
start uav $L/r2.log --params-file $CFG/cbba_uav.yaml -p agent_id:=1 -p participate:=always \
  -p initial_position:="[0.0, 5.0, 0.0]" -p battery:=100.0 -p registry_dir:=$REG
start gcs $L/r3.log --params-file $CFG/cbba_gateway.yaml -p registry_dir:=$REG
start other $L/r4.log --params-file $CFG/cbba_v60.yaml -p agent_id:=60 -p participate:=always \
  -p initial_position:="[5.0, 0.0, 0.0]" -p battery:=100.0 -p registry_dir:=$REG
sleep 6

has $L/r1.log "：dog1（quadruped），能力" && alive dog && ok "R1 狗讀到 fleet.yaml 的 dog1" || bad "R1 狗（看 $L/r1.log）"
has $L/r2.log "：uav1（uav），能力" && alive uav && ok "R2 無人機讀到 fleet.yaml 的 uav1" || bad "R2 無人機（看 $L/r2.log）"
if alive gcs && ! has $L/r3.log "註冊表：" ; then ok "R3 閘道在 gateways、沒有錯誤或警告"; else bad "R3 閘道（看 $L/r3.log）"; fi
has $L/r4.log "機號 60 不在 fleet.yaml" && alive other && ok "R4 不在清單的機號：警告、照參數" || bad "R4（看 $L/r4.log）"
cleanup

# ---------- R5～R7：拒絕啟動 ----------
BADREG=$L/bad_registry
rm -rf $BADREG && cp -r $REG $BADREG
sed -i 's/{name: rgb_camera, bit: 4,/{name: rgb_camera, bit: 40,/' $BADREG/tools.yaml
start bad5 $L/r5.log --params-file $CFG/cbba_v60.yaml -p agent_id:=50 -p participate:=always -p registry_dir:=$BADREG
sleep 4
if has $L/r5.log "rgb_camera 的 bit 是 40" && ! alive bad5; then ok "R5 bit 和程式不一樣：拒絕啟動"; else bad "R5（看 $L/r5.log）"; fi

rm -rf $BADREG && cp -r $REG $BADREG
printf 'schema_version: "1.0"\nactions:\n  - {name: go_to}\n' > $BADREG/actions.yaml
start bad6 $L/r6.log --params-file $CFG/cbba_v60.yaml -p agent_id:=50 -p participate:=always -p registry_dir:=$BADREG
sleep 4
if has $L/r6.log "actions.yaml" && ! alive bad6; then ok "R6 舊格式的 actions.yaml：拒絕啟動"; else bad "R6（看 $L/r6.log）"; fi

start bad7 $L/r7.log --params-file $CFG/cbba_v60.yaml -p agent_id:=1 -p participate:=always -p registry_dir:=$REG
sleep 4
if has $L/r7.log "is platform uav in fleet.yaml" && ! alive bad7; then ok "R7 平台和 agent_type 不一致：拒絕啟動"; else bad "R7（看 $L/r7.log）"; fi

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
