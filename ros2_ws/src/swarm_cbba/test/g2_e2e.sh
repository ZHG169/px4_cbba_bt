#!/bin/bash
# cbba_node 的回歸測試（不用 PX4、BT，用 ros2 topic pub 假裝 BT），約 3 分鐘：
#   G2：伯宇 testdata_for_uav_2026-10-12（他給的測資，不進 repo）的 T1、T2、T6、T8、T9、T10（log 字串照他的 grep）
#   E1～E4：和他 10/07 版本不同的行為（失敗排除 60 s、PERMANENT、回報檢查、火警不排第二個）
#   DEMO3：三隻狗＋uav1 搶一個火點、鎖定、全隊確認
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/g2_e2e.sh；log 在 $OUT（預設 /tmp/cbba_g2）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 93、UDP port 14693、程序鎖目錄另開；節點名稱 g2t_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-93} CBBA_LOCK_DIR=/tmp/cbba_g2_lock
L=${OUT:-/tmp/cbba_g2}; rm -rf "$L"; mkdir -p "$L"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
PORT="-p udp_port:=14693"
declare -A RSP   # 每個 namespace 的假 robot_state 程序
BG=()            # 其他背景程序（topic echo）
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
has() { clean "$1" | grep -qF -- "$2"; }
cnt() { clean "$1" | grep -cF -- "$2"; }
wait_for() { local t=0; until has "$2" "$1" || [ $t -ge $(( $3 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; has "$2" "$1"; }
wait_more() { local t=0; until [ "$(cnt "$2" "$1")" -gt "$3" ] || [ $t -ge $(( $4 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; [ "$(cnt "$2" "$1")" -gt "$3" ]; }
check() { wait_for "$2" "$3" "$4" && ok "$1" || bad "$1（等不到「$2」）"; }
node() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/$1 -r __node:=g2t_$1 --params-file $CFG/$2 -p agent_id:=$3 $PORT "${@:4}" > $L/$1.log 2>&1 & }
# 假的 robot_state（2 Hz）：mode 從檔案讀，切換 mode 不用重開 publisher
# （ros2 topic pub 重開要 1～2 s，空檔超過 state_timeout，會變成逾時釋放而不是 MANUAL 釋放）
cat > "$L/rs.py" <<'PY'
import sys
import rclpy
from swarm_interfaces.msg import RobotState
ns, aid, x, y, z, mode_file = sys.argv[1:7]
rclpy.init()
node = rclpy.create_node('g2t_rs_' + ns)
pub = node.create_publisher(RobotState, f'/{ns}/robot_state', 10)
def tick():
    m = RobotState()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = 'map'
    m.agent_id = int(aid)
    m.position.x, m.position.y, m.position.z = float(x), float(y), float(z)
    m.battery = 90.0
    try:
        m.mode = int(open(mode_file).read().strip() or 0)
    except (OSError, ValueError):
        m.mode = 0
    pub.publish(m)
node.create_timer(0.5, tick)
rclpy.spin(node)
PY
rs() { echo "${6:-0}" > "$L/mode_$1"; python3 "$L/rs.py" $1 $2 $3 $4 $5 "$L/mode_$1" > /dev/null 2>&1 & RSP[$1]=$!; }
rs_stop() { kill "${RSP[$1]}" 2>/dev/null; unset "RSP[$1]"; }
set_mode() { echo "$2" > "$L/mode_$1"; }
fire() { ros2 topic pub --once -w 1 /$1/new_task swarm_interfaces/msg/Task "{task_id: $2, type: 2, position: {x: $3, y: $4, z: 0.0}, value: 100.0, duration_sec: 20.0, deadline_sec: 120.0}" > /dev/null 2>&1; }
result() { ros2 topic pub --once -w 1 /$1/task_result swarm_interfaces/msg/TaskResult "{task_id: $2, success: $3, detail: '$4', reason: $5}" > /dev/null 2>&1; }
stopall() { kill "${RSP[@]}" "${BG[@]}" 2>/dev/null; RSP=(); BG=(); pkill -f "__node:=g2t_"; sleep 1.5; }
trap stopall EXIT

D=$L/v60.log; U=$L/uav1.log
echo "▷ G2 起節點"
node v60 cbba_v60.yaml 50; node uav1 cbba_uav.yaml 1
ros2 topic echo /v60/current_task swarm_interfaces/msg/Task --qos-durability transient_local --qos-reliability reliable > $L/current_task.log 2>&1 & BG+=($!)
check "狗 node 起來" "agent 50（UGV）" $D 15
check "uav1 node 起來" "agent 1（UAV）" $U 15
rs uav1 1 0.0 0.0 5.0
check "uav1 開始參與出價" "開始參與出價" $U 10

echo "▷ T8 機號不對"
rs v60 51 5.0 20.0 0.0
check "T8 略過 51" "agent_id 是 51，不是 50：略過" $D 5
sleep 1; has $D "開始參與出價" && bad "T8 不該參與" || ok "T8 沒開始參與出價"
rs_stop v60

echo "▷ 狗上線"
rs v60 50 5.0 20.0 0.0
check "返航點" "返航點 map (5.0, 20.0, 0.0)，電量 90%" $D 5
check "狗 開始參與出價" "開始參與出價" $D 5

echo "▷ T1 火點"
fire uav1 65537 6.0 15.0
check "T1 uav1 建立" "新任務 00010001 (6.0, 15.0, 0.0)" $U 5
check "T1 狗 路徑" "路徑：00010001" $D 5
check "T1 指派給 BT" "指派給 BT：00010001 (6.0, 15.0, 0.0)" $D 5
check "T1 current_task 收到 65537" "task_id: 65537" $L/current_task.log 5
has $U "指派給 BT：00010001" && bad "T1 uav1 不該接地面任務" || ok "T1 uav1 沒接地面任務"

echo "▷ T2 完成"
result v60 65537 true "" 0
check "T2 task_result 格式" "task_result 00010001：success 1，reason 0，detail「」（NONE）" $D 5
check "T2 等待全隊確認" "任務 00010001 完成，等待全隊確認" $D 5
check "T2 已確認完成" "任務 00010001 已確認完成（回報後" $D 5
check "T2 指派給 BT：無" "指派給 BT：無" $D 5
check "T2 uav1 收到完成證明" "任務 00010001 已確認完成（收到完成證明）" $U 5

echo "▷ T6 BT 停掉再恢復"
fire uav1 65538 2.0 16.0
check "T6 指派 00010002" "指派給 BT：00010002" $D 5
S0=$(cnt $D "停止參與出價"); P0=$(cnt $D "開始參與出價"); A0=$(cnt $D "指派給 BT：00010002")
rs_stop v60; T0=$(date +%s.%N)
wait_more "停止參與出價，釋放手上的任務" $D $S0 5 && ok "T6 停掉後 $(python3 -c "print(round($(date +%s.%N)-$T0,1))") s 內停止參與" || bad "T6 沒停止參與"
check "T6 指派給 BT：無" "指派給 BT：無" $D 3
sleep 2; rs v60 50 5.0 20.0 0.0
wait_more "開始參與出價" $D $P0 5 && ok "T6 恢復參與" || bad "T6 沒恢復"
wait_more "指派給 BT：00010002" $D $A0 5 && ok "T6 接回 00010002" || bad "T6 沒接回"

echo "▷ T9 mode MANUAL"
A0=$(cnt $D "指派給 BT：00010002"); S0=$(cnt $D "停止參與出價")
set_mode v60 1
check "T9 mode 0 → 1" "mode 0 → 1（AUTO → MANUAL）" $D 5
wait_more "停止參與出價" $D $S0 3 && ok "T9 MANUAL 停止參與" || bad "T9 沒停止參與"
# 要是 MANUAL 造成的（緊接在 mode 0 → 1 後面），不是 robot_state 逾時
clean $D | grep -A1 -F "mode 0 → 1" | grep -qF "停止參與出價，釋放手上的任務" \
  && ok "T9 緊接在 mode 0 → 1 之後釋放" || bad "T9 釋放不是 MANUAL 造成的"
sleep 2; set_mode v60 0
check "T9 mode 1 → 0" "mode 1 → 0（MANUAL → AUTO）" $D 5
wait_more "指派給 BT：00010002" $D $A0 5 && ok "T9 00010002 回來" || bad "T9 沒回來"
result v60 65538 true "" 0
check "00010002 完成" "任務 00010002 已確認完成（回報後" $D 5

echo "▷ E1 暫時失敗：60 s 後回來"
fire uav1 65539 3.0 18.0
check "E1 指派 00010003" "指派給 BT：00010003" $D 5
result v60 65539 false NAV_NO_ACK 2; TF=$(date +%s)
check "E1 task_result 格式" "task_result 00010003：success 0，reason 2，detail「NAV_NO_ACK」" $D 5
check "E1 失敗格式（伯宇 grep）" "任務 00010003 失敗（NAV_NO_ACK），交回競標池" $D 3
check "E1 60 s 內不接" "這台 60 s 內不接" $D 1
check "E1 失敗次數與成功率（2026-10-17）" "任務 00010003 這台失敗 1 次，估計成功率 0.50" $D 3

echo "▷ E2 永久失敗（等 E1 的同時）"
fire uav1 65540 4.0 18.0
check "E2 指派 00010004" "指派給 BT：00010004" $D 5
result v60 65540 false OUT_OF_MAP 1
check "E2 不再接" "任務 00010004 失敗（OUT_OF_MAP），交回競標池，這台不再接" $D 5

echo "▷ E3 回報不是手上的任務"
result v60 65537 true "" 0
check "E3 拒絕" "任務 00010001 的回報拒絕：不是目前 current_task 上的任務" $D 5

echo "▷ E1 等 60 s"
check "E1 約 60 s 後回來" "指派給 BT：00010003" $D 1
A3=$(cnt $D "指派給 BT：00010003")
wait_more "指派給 BT：00010003" $D $A3 75 && ok "E1 失敗後 $(( $(date +%s) - TF )) s 回來" || bad "E1 沒回來"

echo "▷ E4 手上有火警時不排第二個"
fire uav1 65541 5.0 19.0
check "E4 uav1 建立 00010005" "新任務 00010005" $U 5
sleep 2
has $D "指派給 BT：00010005" && bad "E4 第二個火警被指派了" || ok "E4 00010005 沒有指派（手上的火警在做）"
result v60 65539 true "" 0
check "E4 做完後接 00010005" "指派給 BT：00010005" $D 5
sleep 1; N4=$(cnt $D "指派給 BT：00010004"); [ "$N4" -eq 1 ] && ok "E2 00010004 只指派過 1 次（PERMANENT 之後沒回來）" || bad "E2 00010004 指派了 $N4 次"
result v60 65541 true "" 0

echo "▷ T10 相對名稱"
stopall
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=g2t_t10 --params-file $CFG/cbba_v60.yaml $PORT > $L/t10.log 2>&1 &
check "T10 起來" "話題 /v60/current_task /v60/robot_state /v60/task_result /v60/new_task" $L/t10.log 15
for i in $(seq 25); do   # 剛啟動時 DDS 還沒發現，重試 5 s
  ros2 topic list --no-daemon > $L/t10_topics.log 2>&1
  grep -qx "/v60/current_task" $L/t10_topics.log && break; sleep 0.2
done
grep -qx "/v60/current_task" $L/t10_topics.log && ok "T10 有 /v60/current_task" || bad "T10 沒 /v60/current_task"
grep -qx "/current_task" $L/t10_topics.log && bad "T10 有裸的 /current_task" || ok "T10 沒有裸的 /current_task"
pkill -f "__node:=g2t_"; sleep 1.5
mv $L/v60.log $L/g2_v60.log; mv $L/uav1.log $L/g2_uav1.log

echo "▷ DEMO3 三隻狗＋uav1"
for a in "v60 50" "v60_1 51" "v60_2 52"; do set -- $a; node $1 cbba_v60.yaml $2; done
node uav1 cbba_uav.yaml 1
for a in "v60 50 0.0" "v60_1 51 10.0" "v60_2 52 20.0"; do set -- $a; rs $1 $2 $3 0.0 0.0; done
rs uav1 1 0.0 0.0 5.0
for n in v60 v60_1 v60_2 uav1; do check "D0 $n 開始參與" "開始參與出價" $L/$n.log 15; done
sleep 2
fire uav1 65537 12.0 1.0
check "D1 51 得標" "指派給 BT：00010001" $L/v60_1.log 5
sleep 2
for n in v60 v60_2 uav1; do has $L/$n.log "指派給 BT：00010001" && bad "D1 $n 也被指派" || ok "D1 $n 沒被指派"; done
echo "▷ D2 52 站到火點上，搶不走"
N0=$(cnt $L/v60_1.log "指派給 BT：無"); rs_stop v60_2; rs v60_2 52 12.0 1.0 0.0
sleep 5
has $L/v60_2.log "指派給 BT：00010001" && bad "D2 52 搶走了" || ok "D2 52 沒搶走"
[ "$(cnt $L/v60_1.log "指派給 BT：無")" -eq "$N0" ] && ok "D2 51 一直保有" || bad "D2 51 被撤下"
echo "▷ D3 完成"
result v60_1 65537 true "" 0
check "D3 51 確認完成" "任務 00010001 已確認完成（回報後" $L/v60_1.log 5
for n in v60 v60_2 uav1; do check "D3 $n 收到完成證明" "任務 00010001 已確認完成（收到完成證明）" $L/$n.log 5; done

stopall
echo "── PASS $PASS  FAIL $FAIL（log 在 $L）"
[ "$FAIL" -eq 0 ]
