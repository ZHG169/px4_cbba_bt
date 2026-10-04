# ============================================================
#  CBBA_BT shell 環境（由映像裡的 loader 從掛載的專案目錄載入，
#  所以修改這個檔不需要重新 build，重開 terminal 就生效）
# ============================================================

source /opt/ros/jazzy/setup.bash

export ROS2_WS="${HOME}/CBBA_BT/ros2_ws"
export PATH="${HOME}/CBBA_BT/docker/scripts:${PATH}"

# 標準的 colcon 工作區：build/ install/ log/
# 所有容器共用同一份編譯結果，請只在 uav1 裡編譯
if [ -f "${ROS2_WS}/install/setup.bash" ]; then
    source "${ROS2_WS}/install/setup.bash"
fi

# cbuild：在 ros2_ws 裡執行 colcon build 並載入結果（只是省打字，等同標準指令）
#   cbuild                              全部編譯
#   cbuild --packages-skip px4_msgs     跳過 px4_msgs
#   cbuild --packages-select my_pkg     只編某個套件
cbuild() {
    ( cd "${ROS2_WS}" && colcon build --symlink-install "$@" ) \
        && source "${ROS2_WS}/install/setup.bash"
}

export PS1="\[\e[1;33m\][${CBBA_ROLE}]\[\e[0m\] ${PS1}"
