#!/bin/bash
# 任務欄位補齊第二批（2026-11-18：05 區域、08 能力、07 搶占；協定版本 5）的節點測試：閘道＋狗（50）的 cbba_node，
# 都給 registry_dir、site_dir（yard），不用 BT，約 1 分鐘
#   Z1 載入區域註冊表（版本、區域、政策）；狗從 fleet.yaml 讀到樓層 1
#   Z2 狗的任務、觀察點在高壓區（Z_HV_A，狗 no_entry）→ 閘道拒絕
#   Z3 不認得的 zone_policy_id → 閘道拒絕
#   Z4 狗的區域註冊表版本不同（2 vs 1）：zone_warning＋log，照常出價、拿到任務
#   Z5 E_HW 拿掉 rgb_camera → capability_restored 放回來（log）
#   Z6 單獨 P0、在 2 樓（狗在 1 樓 → WRONG_FLOOR，沒人接）：10 s 後閘道的看門狗 → task_orphaned P0_UNASSIGNED、
#      FallbackRequest P0_UNASSIGNED（有 fallback_template）
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/zone_iface.sh；log 在 $OUT（預設 /tmp/cbba_zone）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 84、UDP port 14684、程序鎖與世代檔另開；節點名稱 zonet_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84} CBBA_LOCK_DIR=/tmp/cbba_zone_lock
L=${OUT:-/tmp/cbba_zone}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
REG=$REPO/registry
SITE=$REPO/site-config/yard
SITE2=$L/site_v2   # 狗用：版本 2（內容一樣）
cp -r $SITE $SITE2 && sed -i 's/"version": 1,/"version": 2,/' $SITE2/zones.geojson
COMMON=(-p udp_port:=14684 -p state_dir:=$L/state -p registry_dir:=$REG)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
has() { clean "$1" | grep -qF -- "$2"; }
# wait_has 字串 檔案 秒
wait_has() { local t=0; until has "$2" "$1" || [ $t -ge $(( $3 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; has "$2" "$1"; }
stop_all() { pkill -f "__node:=zonet_[a-z0-9]+( |$)" 2>/dev/null; pkill -f "timeout 90 ros2 topic echo" 2>/dev/null; sleep 1; }
trap stop_all EXIT

ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=zonet_gcs --params-file $CFG/cbba_gateway.yaml \
  "${COMMON[@]}" -p site_dir:=$SITE > $L/gcs.log 2>&1 &
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=zonet_dog --params-file $CFG/cbba_v60.yaml \
  "${COMMON[@]}" -p site_dir:=$SITE2 -p agent_id:=50 -p participate:=always \
  -p initial_position:="[20.0, 25.0, 0.0]" -p battery:=100.0 > $L/dog.log 2>&1 &
timeout 90 ros2 topic echo /v60/zone_warning std_msgs/msg/String > $L/zone_warning.txt 2>&1 &
timeout 90 ros2 topic echo /gcs/fallback_request swarm_interfaces/msg/FallbackRequest > $L/fallback.txt 2>&1 &
timeout 90 ros2 topic echo /gcs/task_orphaned swarm_interfaces/msg/TaskOrphaned > $L/orphaned.txt 2>&1 &
sleep 5

# inject 名稱 x y 其他欄位（YAML，可空）→ 回應存在 $L/inject_名稱.log
inject() {
  ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
    "{incident_id: '$1', revision: 1, platform: 2, required_capabilities: ['mobility_ground', 'rgb_camera'], priority: 1,
      position: {x: $2, y: $3, z: 0.0}, floor: 1, value: 100.0, service_time_s: 30.0 $4}" > $L/inject_$1.log 2>&1
}
field() { grep -o "$2=[^,)]*" $L/inject_$1.log | head -1 | cut -d= -f2; }

# ---------- Z1 ----------
has $L/gcs.log "區域註冊表 $SITE：版本 1、8 個區域、2 個政策" && ok "Z1 閘道載入區域註冊表（版本 1、8 區、2 政策）" ||
  bad "Z1 閘道（看 $L/gcs.log）"
has $L/dog.log "版本 2、8 個區域、2 個政策；樓層 1" && ok "Z1 狗載入（版本 2）、fleet.yaml 的樓層 1" || bad "Z1 狗（看 $L/dog.log）"

# ---------- Z2、Z3 ----------
inject z2 -10.0 15.0 ""
[ "$(field z2 accepted)" = "False" ] && grep -q "Z_HV_A" $L/inject_z2.log && ok "Z2 觀察點在 Z_HV_A（狗不能進）：閘道拒絕" ||
  bad "Z2（看 $L/inject_z2.log）"
inject z3 20.0 20.0 ", zone_policy_id: 9"
[ "$(field z3 accepted)" = "False" ] && grep -q "zone_policy_id" $L/inject_z3.log && ok "Z3 不認得的 zone_policy_id：拒絕" ||
  bad "Z3（看 $L/inject_z3.log）"

# ---------- Z4 ----------
inject z4 20.0 22.0 ""
TID=$(field z4 task_id)
if [ "$(field z4 accepted)" = "True" ] && wait_has "區域註冊表版本 1（建立者）≠ 本機 2" $L/dog.log 5; then
  ok "Z4 版本不同：狗告警（log）"
else
  bad "Z4 告警（看 $L/dog.log、$L/inject_z4.log）"
fi
sleep 1
grep -q "區域註冊表版本 1" $L/zone_warning.txt && ok "Z4 zone_warning 話題" || bad "Z4 zone_warning（看 $L/zone_warning.txt）"
wait_has "指派給 BT：" $L/dog.log 5 && clean $L/dog.log | grep "指派給 BT：" | grep -qv "無$" &&
  ok "Z4 照常出價：狗拿到任務" || bad "Z4 狗沒拿到任務（看 $L/dog.log）"

# ---------- Z5 ----------
ros2 topic pub --once -w 1 /v60/task_result swarm_interfaces/msg/TaskResult \
  "{task_id: $TID, success: false, detail: 'camera dead', reason: 2, failure_code: E_HW, failed_capability: rgb_camera}" > /dev/null 2>&1
wait_has "能力 rgb_camera 故障" $L/dog.log 5 && ok "Z5 E_HW：拿掉 rgb_camera" || bad "Z5 故障（看 $L/dog.log）"
ros2 topic pub --once -w 1 /v60/capability_restored std_msgs/msg/String "{data: rgb_camera}" > /dev/null 2>&1
wait_has "能力 rgb_camera 恢復" $L/dog.log 5 && ok "Z5 capability_restored：放回來" || bad "Z5 恢復（看 $L/dog.log）"

# ---------- Z6 ----------
inject z6 30.0 20.0 ", priority: 0, floor: 2, fallback_template: single_dog_verify"
if [ "$(field z6 accepted)" = "True" ] && wait_has "看門狗：" $L/gcs.log 15; then
  ok "Z6 2 樓的 P0 沒人接（狗在 1 樓）：10 s 後看門狗"
else
  bad "Z6 看門狗（看 $L/gcs.log、$L/inject_z6.log）"
fi
sleep 2
grep -q "reason: 10" $L/orphaned.txt && ok "Z6 task_orphaned P0_UNASSIGNED" || bad "Z6 task_orphaned（看 $L/orphaned.txt）"
grep -q "fallback_template: single_dog_verify" $L/fallback.txt && grep -q "reason: 5" $L/fallback.txt &&
  ok "Z6 FallbackRequest P0_UNASSIGNED" || bad "Z6 FallbackRequest（看 $L/fallback.txt）"

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
