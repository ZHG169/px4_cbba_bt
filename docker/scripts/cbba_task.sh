#!/usr/bin/env bash
# ============================================================
#  發任務給 cbba_node、或手動回報任務結果（代替 BT）
#
#  用法：
#    cbba_task.sh new  <N> <流水號> <x> <y> [value] [duration_s] [deadline_s]
#        由 uavN 建立任務，task_id = N × 65536 + 流水號；座標是 map ENU（公尺）
#        例：cbba_task.sh new 1 1 10 5          → task 00010001 在 (10, 5)
#    cbba_task.sh done <N> <task_id>            uavN 回報完成（task_id 用 8 位十六進位，例如 00010001）
#    cbba_task.sh fail <N> <task_id>            uavN 回報失敗：交回競標池、自己不再接
#    cbba_task.sh watch <N>                     看 uavN 目前被指派的任務
#
#  建議在 uavN 自己的容器執行（話題只走機內）。
# ============================================================
set -euo pipefail

usage() { sed -n '4,12p' "$0" | sed -E 's/^# ?//'; exit 1; }

CMD="${1:-}"
N="${2:-}"
[ -n "${CMD}" ] && [ -n "${N}" ] || usage
NS="/uav${N}"

case "${CMD}" in
    new)
        SEQ="${3:?缺流水號}"; X="${4:?缺 x}"; Y="${5:?缺 y}"
        VALUE="${6:-80}"; DURATION="${7:-5}"; DEADLINE="${8:-300}"
        ID=$(( N * 65536 + SEQ ))
        printf '[cbba_task] uav%s 建立任務 %08X (%s, %s) value %s 停留 %s s 期限 %s s\n' \
            "${N}" "${ID}" "${X}" "${Y}" "${VALUE}" "${DURATION}" "${DEADLINE}"
        ros2 topic pub --once -w 1 "${NS}/new_task" swarm_interfaces/msg/Task \
            "{task_id: ${ID}, type: 1, position: {x: ${X}, y: ${Y}, z: 0.0}, value: ${VALUE}, duration_sec: ${DURATION}, deadline_sec: ${DEADLINE}, status: 0}" \
            > /dev/null
        ;;
    done|fail)
        HEX="${3:?缺 task_id（8 位十六進位）}"
        ID=$(( 16#${HEX} ))
        STATUS=$([ "${CMD}" = done ] && echo 1 || echo 2)
        echo "[cbba_task] uav${N} 回報 ${HEX} ${CMD}"
        ros2 topic pub --once -w 1 "${NS}/task_result" swarm_interfaces/msg/Task \
            "{task_id: ${ID}, status: ${STATUS}}" > /dev/null
        ;;
    watch)
        ros2 topic echo --qos-reliability reliable --qos-durability transient_local \
            "${NS}/assigned_task" swarm_interfaces/msg/Task \
            --field task_id
        ;;
    *)
        usage
        ;;
esac
