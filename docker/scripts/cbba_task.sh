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
#    cbba_task.sh done <N> <task_id> [reason]   uavN 回報完成（task_id 用 8 位十六進位，例如 00010001）
#    cbba_task.sh fail <N> <task_id> [reason]   uavN 回報失敗（detail = MANUAL）：任務保留、交回競標池
#        reason（TaskResult）：fail 預設 2 TRANSIENT（這台 60 s 內不接）；1 PERMANENT（這台不再接）、
#        3 PREEMPTED、4 STOPPED。task_id 要是 uavN 目前 current_task 上的任務，不是會被拒絕
#    cbba_task.sh cancel <N> <task_id>          uavN 取消任務（要是建立者或 cancel_authorities）
#    cbba_task.sh watch <N>                     看 uavN 目前被指派的任務（current_task）
#
#  <N> 是哪一台，就要在 uavN 自己的容器（或 sim 容器）執行：DDS 只在機內（.env 的 DDS_OVER_MESH=0，
#  2026-10-12 起），在別台的容器發不到 /uavN/…，ros2 topic pub -w 1 會一直等訂閱者、看起來像沒反應。
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
        if [ "${CMD}" = done ]; then
            RESULT="success: true, reason: ${4:-0}"
        else
            RESULT="success: false, detail: 'MANUAL', reason: ${4:-2}"
        fi
        echo "[cbba_task] uav${N} 回報 ${HEX} ${CMD}（${RESULT}）"
        ros2 topic pub --once -w 1 "${NS}/task_result" swarm_interfaces/msg/TaskResult \
            "{task_id: ${ID}, ${RESULT}}" > /dev/null
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
            "${NS}/current_task" swarm_interfaces/msg/Task | grep --line-buffered -E '^task_id:'
        ;;
    *)
        usage
        ;;
esac
