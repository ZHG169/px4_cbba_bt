#!/usr/bin/env bash
# ============================================================
#  發任務給 cbba_node、或手動回報任務結果（代替 BT）
#
#  用法：
#    cbba_task.sh new  <N> <流水號> <x> <y> [value] [duration_s] [deadline_s]
#        由 uavN 建立任務，task_id = N × 65536 + 流水號；座標是 map ENU（公尺）
#        例：cbba_task.sh new 1 1 10 5          → task 00010001 在 (10, 5)
#    cbba_task.sh ground <N> <流水號> <x> <y> [value] [duration_s] [deadline_s]
#        由 uavN 建立地面處置任務（GROUND_INTERVENTION，給機器狗）；預設照介面規格的火警任務：
#        value 100、停留 20 s、期限 120 s。無人機不會出價
#    cbba_task.sh done <N> <task_id> [版本]     uavN 回報完成（task_id 用 8 位十六進位，例如 00010001）
#    cbba_task.sh fail <N> <task_id> [版本]     uavN 回報失敗（detail = MANUAL）：任務保留、交回競標池、自己不再接
#        版本 = assigned_task 的 assignment_version；省略時讀 uavN 目前的 assigned_task（task_id 要相同）
#    cbba_task.sh cancel <N> <task_id>          uavN 取消任務（要是建立者或 cancel_authorities）
#    cbba_task.sh watch <N>                     看 uavN 目前被指派的任務與版本
#
#  建議在 uavN 自己的容器執行（話題只走機內）。
# ============================================================
set -euo pipefail

usage() { sed -n '4,18p' "$0" | sed -E 's/^# ?//'; exit 1; }

CMD="${1:-}"
N="${2:-}"
[ -n "${CMD}" ] && [ -n "${N}" ] || usage
NS="/uav${N}"

case "${CMD}" in
    new|ground)
        SEQ="${3:?缺流水號}"; X="${4:?缺 x}"; Y="${5:?缺 y}"
        if [ "${CMD}" = new ]; then
            TYPE=1; KIND="任務"; VALUE="${6:-80}"; DURATION="${7:-5}"; DEADLINE="${8:-300}"
        else
            TYPE=2; KIND="地面處置任務"; VALUE="${6:-100}"; DURATION="${7:-20}"; DEADLINE="${8:-120}"
        fi
        ID=$(( N * 65536 + SEQ ))
        printf '[cbba_task] uav%s 建立%s %08X (%s, %s) value %s 停留 %s s 期限 %s s\n' \
            "${N}" "${KIND}" "${ID}" "${X}" "${Y}" "${VALUE}" "${DURATION}" "${DEADLINE}"
        ros2 topic pub --once -w 1 "${NS}/new_task" swarm_interfaces/msg/Task \
            "{task_id: ${ID}, type: ${TYPE}, position: {x: ${X}, y: ${Y}, z: 0.0}, value: ${VALUE}, duration_sec: ${DURATION}, deadline_sec: ${DEADLINE}, status: 0}" \
            > /dev/null
        ;;
    done|fail)
        HEX="${3:?缺 task_id（8 位十六進位）}"
        ID=$(( 16#${HEX} ))
        VERSION="${4:-}"
        if [ -z "${VERSION}" ]; then
            # 目前的 assigned_task（transient_local，訂閱就收得到最後一則）
            CUR=$(ros2 topic echo --once --qos-reliability reliable --qos-durability transient_local \
                "${NS}/assigned_task" swarm_interfaces/msg/Task | awk '/^task_id:/{t=$2} /^assignment_version:/{v=$2} END{print t, v}')
            read -r CUR_ID VERSION <<< "${CUR}"
            if [ "${CUR_ID:-0}" != "${ID}" ]; then
                echo "[cbba_task] uav${N} 目前指派的不是 ${HEX}（是 $(printf '%08X' "${CUR_ID:-0}")）：請指定版本，或確認 task_id" >&2
                exit 1
            fi
        fi
        if [ "${CMD}" = done ]; then
            RESULT="success: true"
        else
            RESULT="success: false, detail: 'MANUAL'"
        fi
        echo "[cbba_task] uav${N} 回報 ${HEX} v${VERSION} ${CMD}"
        ros2 topic pub --once -w 1 "${NS}/task_result" swarm_interfaces/msg/TaskResult \
            "{task_id: ${ID}, ${RESULT}, assignment_version: ${VERSION}}" > /dev/null
        ;;
    cancel)
        HEX="${3:?缺 task_id（8 位十六進位）}"
        ID=$(( 16#${HEX} ))
        echo "[cbba_task] uav${N} 取消 ${HEX}"
        ros2 topic pub --once -w 1 "${NS}/cancel_task" swarm_interfaces/msg/Task \
            "{task_id: ${ID}}" > /dev/null
        ;;
    watch)
        ros2 topic echo --qos-reliability reliable --qos-durability transient_local \
            "${NS}/assigned_task" swarm_interfaces/msg/Task | grep --line-buffered -E '^(task_id|assignment_version):'
        ;;
    *)
        usage
        ;;
esac
