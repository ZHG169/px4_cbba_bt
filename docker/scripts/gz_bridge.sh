#!/usr/bin/env bash
# ============================================================
#  [sim 容器] 把 Gazebo 的模擬時鐘送進 ROS 2（/clock）
#  其他容器的節點設 use_sim_time:=true 時需要它
#  之後要送相機影像，在後面加上對應的 topic 即可
# ============================================================
set -euo pipefail

exec ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    "$@"
