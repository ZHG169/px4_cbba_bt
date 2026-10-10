#!/usr/bin/env bash
# ============================================================
#  [sim 容器] Gazebo → ROS 2：模擬時鐘 /clock，以及每台無人機的下視相機
#
#  用法：gz_bridge.sh                 /clock＋uav1..UAV_COUNT 的相機
#        CAMERA=0 gz_bridge.sh        只送 /clock
#        gz_bridge.sh -p use_sim_time:=true   之後的參數是 parameter_bridge 的 ROS 參數
#
#  相機（機型要有 mono_cam，例如 gz_x500_mono_cam_down）：
#    /world/<世界>/model/<機型>_N/link/camera_link/sensor/imager/image        → /uavN/camera/image_raw
#    /world/<世界>/model/<機型>_N/link/camera_link/sensor/imager/camera_info  → /uavN/camera/camera_info
#  影像從 sim 容器經線材網路送到 uavN 容器（機上電腦），lazy：沒有訂閱者就不送。
#  世界名稱自動從 Gazebo 讀，所以要在 Gazebo 開好之後才執行；無人機可以之後才生成。
# ============================================================
set -euo pipefail

export GZ_CONFIG_PATH="/usr/share/gz${GZ_CONFIG_PATH:+:${GZ_CONFIG_PATH}}"
CONFIG="/tmp/cbba_gz_bridge.yaml"
MODEL="${PX4_SIM_MODEL:-gz_x500_mono_cam_down}"
MODEL="${MODEL#gz_}"

cat > "${CONFIG}" <<EOF
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
EOF

if [ "${CAMERA:-1}" = "1" ]; then
    WORLD=""
    for _ in $(seq 1 30); do
        WORLD="$(gz topic -l 2>/dev/null | grep -m 1 -E '^/world/[^/]+/clock$' | sed -E 's#^/world/([^/]+)/clock$#\1#' || true)"
        [ -n "${WORLD}" ] && break
        echo "[gz_bridge] 等 Gazebo…"
        sleep 1
    done
    if [ -z "${WORLD}" ]; then
        echo "[gz_bridge] 找不到 Gazebo 的世界，先執行 px4_sitl.sh"
        exit 1
    fi
    read -r -a AGENTS <<< "${UAV_AGENT_IPS:-x}"
    for N in $(seq 1 "${#AGENTS[@]}"); do
        BASE="/world/${WORLD}/model/${MODEL}_${N}/link/camera_link/sensor/imager"
        cat >> "${CONFIG}" <<EOF
- ros_topic_name: "/uav${N}/camera/image_raw"
  gz_topic_name: "${BASE}/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
  lazy: true
- ros_topic_name: "/uav${N}/camera/camera_info"
  gz_topic_name: "${BASE}/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
  lazy: true
EOF
    done
    echo "[gz_bridge] 世界 ${WORLD}：/clock＋uav1..uav${#AGENTS[@]} 的相機（${MODEL}_N）"
fi

exec ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="${CONFIG}" "$@"
