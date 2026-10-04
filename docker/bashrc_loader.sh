# ============================================================
#  映像內建的 bashrc loader（只是轉接）
#  真正的設定在掛載進來的 docker/bashrc_cbba.sh，修改不需要重新 build
# ============================================================
if [ -f "${HOME}/CBBA_BT/docker/bashrc_cbba.sh" ]; then
    source "${HOME}/CBBA_BT/docker/bashrc_cbba.sh"
else
    source /opt/ros/jazzy/setup.bash
fi
