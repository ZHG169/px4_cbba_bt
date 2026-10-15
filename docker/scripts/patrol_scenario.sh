#!/usr/bin/env bash
# ============================================================
#  [sim 容器] 巡檢＋隨機插入測試（docs/sim_scenario_plan.md）：在 terminal 輸入編號建立任務
#
#  用法：patrol_scenario.sh                互動模式：輸入 g、1～5、r、l、q（見下面）
#        patrol_scenario.sh init           建立 3 個綠色巡檢任務（G1～G3）
#        patrol_scenario.sh insert 3 5     插入紅色點 R3、R5：生成 tag、建立任務
#        patrol_scenario.sh fire 2         R2 起火：灰色方塊變紅（生成 fire_block）＋火警 tag 101、建立偵巡任務；
#                                          飛過去的無人機看到 tag 101 會丟出地面處置任務（給狗）
#        patrol_scenario.sh fire-at 12.5 -3 [名稱]   任意位置起火（紅色方塊＋tag 101），不建立任務：
#                                          無人機路過時自己發現（itest demo 放在飛行路線上）
#        patrol_scenario.sh mark-at 12.5 -3 [名稱]   任意位置生成藍色平台（models/task_pad），不建立任務：
#                                          itest demo --interactive 手動插入任務點時標出位置
#        patrol_scenario.sh replay 檔案    照排程插入：每行「秒數,編號」（從執行時算起；編號 g = 綠色巡檢任務），
#                                          例如 cbba_sim insert 輸出的 schedules/trial_000.csv（和離線的第 0 次試驗相同）
#        patrol_scenario.sh reset          刪掉已生成的紅色 tag（下一次試驗前；cbba_node 也要重開）
#        patrol_scenario.sh list           列出任務點
#
#  任務點在 docker/gz/worlds/patrol_site_points.csv（和 Gazebo 場景、cbba_sim 共用）。
#  場景要先開好：CBBA_WORLD=patrol_site px4_sitl.sh 1
#  展示的小場景：CBBA_SCENARIO=demo_site patrol_scenario.sh …（點在 demo_site_points.csv，場景 CBBA_WORLD=demo_site）
#
#  任務由 uav${CREATOR} 建立（task_id 的建立者必須是 CBBA 成員；sim 容器不是），經 DDS 發到 /uav${CREATOR}/new_task。
#  流水號用 0x1000～0x7FFF（BT 的火警用 0x8000 以上），存在 /tmp/cbba_scenario_seq_uavN，重開 cbba_node 也不會重複。
#  環境變數：CREATOR（預設 1）、VALUE（80）、DURATION（停留秒數，10）、DEADLINE（期限秒數，300）
#  ros2 topic pub 啟動約 1 s，replay 的時刻會整體晚約 1 s。
# ============================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 場景：預設 patrol_site；CBBA_SCENARIO=demo_site 用展示的小場景（點在 worlds/<場景>_points.csv）
WORLD="${CBBA_SCENARIO:-patrol_site}"
POINTS="${HERE}/../gz/worlds/${WORLD}_points.csv"
TAG_Z_RED=0.302                  # 灰色方塊 R 頂面（0.3 m）上方 2 mm，和 gen_patrol_site.py 一致
TAG_Z_FIRE=0.322                 # 火點方塊（models/fire_block，高 0.32 m）頂面上方 2 mm
CREATOR="${CREATOR:-1}"
VALUE="${VALUE:-80}"
DURATION="${DURATION:-10}"
DEADLINE="${DEADLINE:-300}"
SEQ_FILE="/tmp/cbba_scenario_seq_uav${CREATOR}"
export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:${GZ_CONFIG_PATH}}"

usage() { sed -n '4,18p' "$0" | sed -E 's/^# ?//'; exit 1; }

# 任務點：point x y tag
point_row() { grep -v '^#' "${POINTS}" | awk -F, -v p="$1" 'NR > 1 && $1 == p {print $1, $2, $3, $4}'; }

next_seq() {
    local seq
    if [ -f "${SEQ_FILE}" ]; then
        seq=$(( $(cat "${SEQ_FILE}") + 1 ))
    else
        seq=$(( 0x1000 + $(date +%s) % 0x6000 ))     # 第一次用時間錯開，避免和之前的執行重複
    fi
    [ "${seq}" -gt $(( 0x7FFF )) ] && seq=$(( 0x1000 ))
    echo "${seq}" > "${SEQ_FILE}"
    echo "${seq}"
}

publish_task() {
    local name="$1" x="$2" y="$3" seq id
    seq=$(next_seq)
    id=$(( CREATOR * 65536 + seq ))
    printf '[scenario] %s：uav%s 建立任務 %08X (%s, %s) value %s 停留 %s s 期限 %s s\n' \
        "${name}" "${CREATOR}" "${id}" "${x}" "${y}" "${VALUE}" "${DURATION}" "${DEADLINE}"
    if ! timeout 10 ros2 topic pub --once -w 1 "/uav${CREATOR}/new_task" swarm_interfaces/msg/Task \
        "{task_id: ${id}, type: 1, position: {x: ${x}, y: ${y}, z: 0.0}, value: ${VALUE}, duration_sec: ${DURATION}, deadline_sec: ${DEADLINE}, status: 0}" \
        > /dev/null; then
        echo "[scenario] 發不出去：uav${CREATOR} 的 cbba_node 沒開（10 s 內沒有訂閱者），" \
            "或這個 terminal 沒有載入 swarm_interfaces（source ~/CBBA_BT/ros2_ws/install/setup.bash）" >&2
        return 1
    fi
}

gz_call() {   # service reqtype req
    gz service -s "/world/${WORLD}/$1" --reqtype "$2" --reptype gz.msgs.Boolean --timeout 3000 --req "$3" 2>&1 || true
}

# Gazebo 的 create／remove 只回覆「收到請求」（名稱重複、不存在也是 data: true），所以用模型清單判斷
tag_exists() { gz model --list 2>/dev/null | grep -q -- "- tag_$1\$"; }

spawn_tag() {
    local name="$1" x="$2" y="$3" tag="$4" reply
    if tag_exists "${name}"; then
        echo "[scenario] ${name}：tag ${tag} 已經在了（同一個點再插入一次，只建立新任務）"
        return 0
    fi
    reply=$(gz_call create gz.msgs.EntityFactory \
        "sdf_filename: \"model://tag36h11_${tag}\", name: \"tag_${name}\", allow_renaming: false, pose: {position: {x: ${x}, y: ${y}, z: ${TAG_Z_RED}}}")
    if echo "${reply}" | grep -q "data: true"; then
        echo "[scenario] ${name}：tag ${tag} 出現"
    else
        echo "[scenario] ${name}：tag ${tag} 沒有生成（Gazebo 的 ${WORLD} 有開嗎？）：${reply}" >&2
    fi
}

insert_point() {
    local k="$1" row name x y tag
    [[ "${k}" =~ ^[1-5]$ ]] || { echo "[scenario] 紅色點編號是 1～5：${k}" >&2; return 1; }
    row=$(point_row "R${k}")
    [ -n "${row}" ] || { echo "[scenario] ${POINTS} 沒有 R${k}" >&2; return 1; }
    read -r name x y tag <<< "${row}"
    spawn_tag "${name}" "${x}" "${y}" "${tag}"
    publish_task "${name}" "${x}" "${y}"
}

# 起火：紅色方塊（models/fire_block）＋火警 tag 101（BT 只認 101；R 點自己的 tag 3～7 不算火警）。
# 名稱 fire_<名稱>、tag_fire_<名稱>（reset 會刪掉）
spawn_fire() {
    local name="$1" x="$2" y="$3" reply
    if gz model --list 2>/dev/null | grep -q -- "- fire_${name}\$"; then
        echo "[scenario] ${name} 已經在燒了"
        return 0
    fi
    reply=$(gz_call create gz.msgs.EntityFactory \
        "sdf_filename: \"model://fire_block\", name: \"fire_${name}\", allow_renaming: false, pose: {position: {x: ${x}, y: ${y}, z: 0}}")
    echo "${reply}" | grep -q "data: true" || echo "[scenario] ${name}：火點方塊沒有生成：${reply}" >&2
    reply=$(gz_call create gz.msgs.EntityFactory \
        "sdf_filename: \"model://tag36h11_101\", name: \"tag_fire_${name}\", allow_renaming: false, pose: {position: {x: ${x}, y: ${y}, z: ${TAG_Z_FIRE}}}")
    echo "${reply}" | grep -q "data: true" && echo "[scenario] ${name} (${x}, ${y}) 起火：紅色方塊＋tag 101" ||
        echo "[scenario] ${name}：tag 101 沒有生成：${reply}" >&2
}

# 手動插入的任務點：藍色平台（models/task_pad），名稱 task_<名稱>（reset 會刪掉）。2026-10-14
spawn_pad() {
    local name="$1" x="$2" y="$3" reply
    if gz model --list 2>/dev/null | grep -q -- "- task_${name}\$"; then
        echo "[scenario] ${name} 的平台已經在了"
        return 0
    fi
    reply=$(gz_call create gz.msgs.EntityFactory \
        "sdf_filename: \"model://task_pad\", name: \"task_${name}\", allow_renaming: false, pose: {position: {x: ${x}, y: ${y}, z: 0}}")
    echo "${reply}" | grep -q "data: true" && echo "[scenario] ${name} (${x}, ${y})：藍色平台" ||
        echo "[scenario] ${name}：平台沒有生成：${reply}" >&2
}

fire_point() {
    local k="$1" row name x y tag
    [[ "${k}" =~ ^[1-5]$ ]] || { echo "[scenario] R 點編號是 1～5：${k}" >&2; return 1; }
    row=$(point_row "R${k}")
    [ -n "${row}" ] || { echo "[scenario] ${POINTS} 沒有 R${k}" >&2; return 1; }
    read -r name x y tag <<< "${row}"
    spawn_fire "${name}" "${x}" "${y}"
    publish_task "${name}（火警）" "${x}" "${y}"
}

init_known() {
    local name x y tag
    for p in G1 G2 G3; do
        read -r name x y tag <<< "$(point_row "${p}")"
        publish_task "${name}" "${x}" "${y}"
    done
}

reset_tags() {
    for k in 1 2 3 4 5; do
        if tag_exists "R${k}"; then
            gz_call remove gz.msgs.Entity "name: \"tag_R${k}\", type: MODEL" > /dev/null
            echo "[scenario] 刪掉 tag_R${k}"
        fi
    done
    for m in $(gz model --list 2>/dev/null | sed -n 's/^ *- \(\(\(tag_\)\{0,1\}fire_\|task_\).*\)$/\1/p'); do
        gz_call remove gz.msgs.Entity "name: \"${m}\", type: MODEL" > /dev/null
        echo "[scenario] 刪掉 ${m}"
    done
    echo "[scenario] 插入的 tag、火點、藍色平台已清掉"
}

list_points() {
    echo "任務點（${POINTS}）："
    grep -v '^#' "${POINTS}" | awk -F, 'NR > 1 {printf "  %-3s (%6s, %6s)  tag %s\n", $1, $2, $3, $4}'
}

replay() {
    local file="${1:?缺排程檔}" t k pids=()
    [ -f "${file}" ] || { echo "[scenario] 找不到 ${file}" >&2; exit 1; }
    echo "[scenario] 照 ${file} 插入（從現在算起）"
    while IFS=, read -r t k; do
        [[ "${t}" =~ ^[0-9.]+$ ]] || continue          # 表頭、註解
        if [ "${k}" = g ]; then
            ( sleep "${t}"; echo "[scenario] t = ${t} s"; init_known ) &
        else
            ( sleep "${t}"; echo "[scenario] t = ${t} s"; insert_point "${k}" ) &
        fi
        pids+=($!)
    done < "${file}"
    wait "${pids[@]}"
}

interactive() {
    list_points
    echo "輸入：g 建立綠色巡檢任務｜1～5 插入紅色點（可一次輸入多個，例如 2 4）｜f1～f5 R 點起火（紅色方塊＋tag 101）｜r 清掉紅色 tag｜l 列出｜q 結束"
    local line
    while read -r -p "> " line; do
        for c in ${line}; do
            case "${c}" in
                g|G) init_known || true ;;
                [1-5]) insert_point "${c}" || true ;;
                [fF][1-5]) fire_point "${c:1}" || true ;;
                r|R) reset_tags ;;
                l|L) list_points ;;
                q|Q) return 0 ;;
                *) echo "不認得：${c}" ;;
            esac
        done
    done
}

case "${1:-}" in
    "") interactive ;;
    init) init_known ;;
    insert) shift; [ $# -gt 0 ] || usage; for k in "$@"; do insert_point "${k}"; done ;;
    fire) shift; [ $# -gt 0 ] || usage; for k in "$@"; do fire_point "${k}"; done ;;
    fire-at) [ $# -ge 3 ] || usage; spawn_fire "${4:-$(date +%H%M%S)}" "$2" "$3" ;;
    mark-at) [ $# -ge 3 ] || usage; spawn_pad "${4:-$(date +%H%M%S)}" "$2" "$3" ;;
    replay) replay "${2:-}" ;;
    reset) reset_tags ;;
    list) list_points ;;
    *) usage ;;
esac
