#!/usr/bin/env bash
# ============================================================
#  [uav / gcs 容器] 對 mesh 網卡套用 tc netem（模擬差的無線電）
#  只動 mesh_net，不會影響 sim <-> uav 的線
#
#  用法：
#    mesh_tc.sh set 30            丟包 30%，延遲 50±30 ms（預設）
#    mesh_tc.sh set 70 80 20      丟包 70%，延遲 80±20 ms
#    mesh_tc.sh show              顯示目前設定
#    mesh_tc.sh clear             恢復正常
#
#  注意：tc 只影響「送出」方向。要雙向都變差，uav 和 gcs 兩邊都要設。
# ============================================================
set -euo pipefail

: "${MESH_IP:?這個容器沒有 mesh 網卡（只有 uav / gcs 有）}"

IFACE="$(ip -o -4 addr show | awk -v ip="${MESH_IP}" '{split($4, a, "/"); if (a[1] == ip) print $2}')"
IFACE="${IFACE%%@*}"
if [ -z "${IFACE}" ]; then
    echo "找不到 IP 為 ${MESH_IP} 的網卡"
    exit 1
fi

case "${1:-show}" in
    set)
        LOSS="${2:?請給丟包率，例如 mesh_tc.sh set 30}"
        DELAY="${3:-50}"
        JITTER="${4:-30}"
        sudo tc qdisc replace dev "${IFACE}" root netem \
            loss "${LOSS}%" delay "${DELAY}ms" "${JITTER}ms"
        echo "[mesh_tc] ${IFACE}: loss ${LOSS}%, delay ${DELAY}±${JITTER} ms"
        ;;
    clear)
        sudo tc qdisc del dev "${IFACE}" root 2>/dev/null || true
        echo "[mesh_tc] ${IFACE}: 已恢復正常"
        ;;
    show)
        echo "[mesh_tc] mesh 網卡：${IFACE} (${MESH_IP})"
        tc qdisc show dev "${IFACE}"
        ;;
    *)
        echo "用法：mesh_tc.sh set <丟包%> [延遲ms] [抖動ms] | show | clear"
        exit 1
        ;;
esac
