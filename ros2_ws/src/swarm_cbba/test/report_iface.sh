#!/bin/bash
# 任務欄位補齊第四批（2026-11-18：11 回報、03 證據）的節點測試：閘道（registry_dir、site_dir）＋狗（50），不用 BT，約 1 分鐘
#   R1 注入帶證據與 report_policy：incident_evidence 有整筆紀錄（攝影機、截圖、VLM）；狗的 current_task.spec 只有 evidence 的 uri、信心值
#   R2 假 BT 回報成功（做完 2 步、2 個附件、最後位置）→ 閘道的 task_reports 有、channels = [gcs]；紀錄檔有一行
#   R3 report_policy 要 findings、成功卻沒有 findings、附件 → missing_findings
#   R4 閘道斷線時的結果：狗先暫存，閘道恢復後補送（buffered）
#   R5 證據的 URI 不合法（ftp://）、report_policy 的頻道不認得 → 閘道拒絕
# 用法（uav 容器裡，編譯完）：bash ros2_ws/src/swarm_cbba/test/report_iface.sh；log 在 $OUT（預設 /tmp/cbba_report）
# 和正在跑的節點隔離：ROS_DOMAIN_ID 86、UDP port 14686、程序鎖與世代檔另開；節點名稱 rept_*，結束時只關自己開的
WS=$(cd "$(dirname "$0")/../../.." && pwd)
REPO=$(cd "$WS/.." && pwd)
source /opt/ros/jazzy/setup.bash && source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-86} CBBA_LOCK_DIR=/tmp/cbba_report_lock
L=${OUT:-/tmp/cbba_report}; rm -rf "$L" "$CBBA_LOCK_DIR"; mkdir -p "$L/state" "$CBBA_LOCK_DIR"
CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
COMMON=(-p udp_port:=14686 -p state_dir:=$L/state -p registry_dir:=$REPO/registry -p site_dir:=$REPO/site-config/yard)
PASS=0; FAIL=0
ok()  { echo "PASS  $1"; PASS=$((PASS+1)); }
bad() { echo "FAIL  $1"; FAIL=$((FAIL+1)); }
clean() { sed 's/\x1b\[[0-9;]*m//g' "$1"; }
has() { clean "$1" | grep -qF -- "$2"; }
wait_has() { local t=0; until has "$2" "$1" || [ $t -ge $(( $3 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; has "$2" "$1"; }
wait_grep() { local t=0; until grep -q -- "$1" "$2" 2>/dev/null || [ $t -ge $(( $3 * 5 )) ]; do sleep 0.2; t=$((t+1)); done; grep -q -- "$1" "$2"; }
stop_gcs() { pkill -f "__node:=rept_gcs( |$)" 2>/dev/null; sleep 1; }
stop_all() { pkill -f "__node:=rept_[a-z0-9]+( |$)" 2>/dev/null; pkill -f "timeout 150 ros2 topic echo /gcs/" 2>/dev/null; sleep 1; }
trap stop_all EXIT

start_gcs() { ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=rept_gcs --params-file $CFG/cbba_gateway.yaml \
  "${COMMON[@]}" > $L/$1 2>&1 & }
echo_gcs() {   # echo_gcs 話題 型別 檔案
  # QoS 明確給：auto 在還沒發現發布者時會用 volatile，收不到閘道剛起來就發的（transient_local 的歷史）
  timeout 150 ros2 topic echo --qos-reliability reliable --qos-durability transient_local /gcs/$1 $2 > $L/$3 2>&1 &
}
start_gcs gcs_a.log
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=rept_dog --params-file $CFG/cbba_v60.yaml \
  "${COMMON[@]}" -p agent_id:=50 -p participate:=always -p initial_position:="[0.0, 25.0, 0.0]" -p battery:=100.0 \
  > $L/dog.log 2>&1 &
echo_gcs incident_evidence swarm_interfaces/msg/IncidentEvidence evidence_a.txt
echo_gcs task_reports swarm_interfaces/msg/TaskReport reports_a.txt
sleep 5

SPEC_GCS='{"steps": [{"action": "go_to", "location": {"x": 10.0, "y": 25.0}}, {"action": "inspect", "object": "person"}], "report_policy": {"channels": ["gcs"], "required_findings": ["image"]}}'
# inject 名稱 spec 其他欄位（YAML，可空）
inject() {
  ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
    "{incident_id: '$1', revision: 1, platform: 2, required_capabilities: ['mobility_ground'], priority: 1,
      position: {x: 10.0, y: 25.0, z: 0.0}, value: 100.0, service_time_s: 30.0, spec: '$2' $3}" > $L/inject_$1.log 2>&1
}
field() { grep -o "$2=[^,)]*" $L/inject_$1.log | head -1 | cut -d= -f2; }
current() { timeout 3 ros2 topic echo --once --field task_id /v60/current_task swarm_interfaces/msg/Task 2>/dev/null | head -1; }
wait_current() {
  local t=0 c
  while [ $t -lt $2 ]; do c=$(current); [ "$c" = "$1" ] && return 0; sleep 1; t=$((t+1)); done
  return 1
}
result() {   # result task_id 其他欄位（YAML）
  ros2 topic pub --once -w 1 /v60/task_result swarm_interfaces/msg/TaskResult "{task_id: $1 $2}" > /dev/null 2>&1
}

# ---------- R1 ----------
EV=", evidence_uri: 'evidence://r1/snapshot/cam07.jpg', evidence_confidence: 0.9, camera_ids: ['CAM-07'],
  snapshot_uri: 'evidence://r1/snapshot/cam07.jpg', clip_uri: 'evidence://r1/clip/cam07.mp4', vlm_result_uri: 'evidence://r1/vlm/1.json'"
inject r1 "$SPEC_GCS" "$EV"
T1=$(field r1 task_id)
if [ "$(field r1 accepted)" = "True" ] && wait_grep "vlm_result_uri: evidence://r1/vlm/1.json" $L/evidence_a.txt 5 &&
  grep -q "CAM-07" $L/evidence_a.txt; then
  ok "R1 incident_evidence 有整筆證據（攝影機、截圖、影片、VLM）"
else
  bad "R1 證據紀錄（看 $L/evidence_a.txt、$L/inject_r1.log）"
fi
if wait_current "$T1" 15; then
  timeout 3 ros2 topic echo --once --full-length --field spec /v60/current_task swarm_interfaces/msg/Task > $L/spec_r1.txt 2>/dev/null
  grep -qF '"evidence":{"uri":"evidence://r1/snapshot/cam07.jpg","confidence":0.900}' $L/spec_r1.txt &&
    ! grep -q "vlm_result_uri\|CAM-07" $L/spec_r1.txt && ok "R1 TaskSpec 只帶 evidence 的參照與信心值" ||
    bad "R1 TaskSpec（看 $L/spec_r1.txt）"
else
  bad "R1 狗沒拿到任務（current $(current)）"
fi

# ---------- R2 ----------
result "$T1" ", success: true, completed_steps: 2, findings_uri: 'evidence://r1/findings/dog1.json',
  attachments: ['evidence://r1/snapshot/dog1_1.jpg', 'evidence://r1/snapshot/dog1_2.jpg'],
  final_position_valid: true, final_position: {x: 9.5, y: 25.0, z: 0.0}"
if wait_grep "dog1_2.jpg" $L/reports_a.txt 5 && grep -q "completed_steps: 2" $L/reports_a.txt && grep -q "x: 9.5" $L/reports_a.txt &&
  grep -q "\- gcs" $L/reports_a.txt; then
  ok "R2 task_reports：成功、做完 2 步、2 個附件、最後位置、channels gcs"
else
  bad "R2 結果（看 $L/reports_a.txt、$L/dog.log、$L/gcs_a.log）"
fi
wait_grep "dog1_2.jpg" $L/state/gateway_200_reports.log 3 && ok "R2 紀錄檔有一行" ||
  bad "R2 紀錄檔（看 $L/state/gateway_200_reports.log）"

# ---------- R3 ----------
inject r3 "$SPEC_GCS"
T3=$(field r3 task_id)
if wait_current "$T3" 15; then
  result "$T3" ", success: true, completed_steps: 2"
  wait_grep "missing_findings: true" $L/reports_a.txt 5 && ok "R3 要 findings 卻沒有 → missing_findings" ||
    bad "R3（看 $L/reports_a.txt）"
else
  bad "R3 狗沒拿到任務"
fi

# ---------- R4 ----------
inject r4 "$SPEC_GCS"
T4=$(field r4 task_id)
if wait_current "$T4" 15; then
  stop_gcs
  sleep 4
  result "$T4" ", success: false, reason: 2, failure_code: E_TARGET_LOST, detail: 'lost him', completed_steps: 1"
  wait_has "閘道斷線，先暫存" $L/dog.log 5 && ok "R4 閘道斷線：狗先暫存結果" || bad "R4 暫存（看 $L/dog.log）"
  echo_gcs task_reports swarm_interfaces/msg/TaskReport reports_b.txt   # 閘道起來前就先聽：補送在閘道一起來就到
  sleep 1
  start_gcs gcs_b.log
  if wait_grep "failure_code: E_TARGET_LOST" $L/reports_b.txt 15 && grep -q "buffered: true" $L/reports_b.txt; then
    ok "R4 閘道恢復後補送（buffered）"
  else
    bad "R4 補送（看 $L/reports_b.txt、$L/gcs_b.log）"
  fi
else
  bad "R4 狗沒拿到任務"
fi

# ---------- R5 ----------
inject r5 "$SPEC_GCS" ", clip_uri: 'ftp://nas/clip.mp4'"
[ "$(field r5 accepted)" = "False" ] && ok "R5 證據 URI 不合法：拒絕" || bad "R5 URI（看 $L/inject_r5.log）"
inject r5b '{"steps": [{"action": "inspect"}], "report_policy": {"channels": ["email"]}}'
[ "$(field r5b accepted)" = "False" ] && grep -q "BAD_REPORT_POLICY" $L/inject_r5b.log && ok "R5 report_policy 的頻道不認得：拒絕" ||
  bad "R5 report_policy（看 $L/inject_r5b.log）"

echo "== $PASS PASS, $FAIL FAIL（log：$L）"
[ $FAIL -eq 0 ]
