# syntax=docker/dockerfile:1
# ============================================================
#  cbba-sim：真實世界 + 飛控
#  - PX4-Autopilot（build 時就先編譯好 px4_sitl）
#  - Gazebo Harmonic（由 PX4 的 ubuntu.sh 安裝）
#  - ros_gz_bridge（把 /clock、相機等 Gazebo 資料送進 ROS 2）
# ============================================================
ARG BASE_IMAGE=cbba-base:jazzy
FROM ${BASE_IMAGE}

ARG PX4_VERSION=v1.17.0
ENV PX4_DIR=/opt/PX4-Autopilot

# 全部放在同一個 RUN：clone → 修補相機編碼器 → 安裝相依 → 預先編譯 → 開放寫入權限
# （分開寫的話 chmod 會把整個 PX4 再複製一層，映像會大一倍）
#
# 相機編碼器修補（PX4 PR #27944）：
#   帶相機的機型（例如 gz_x500_mono_cam_down）在 GstCameraSystem 會用到 nvh264enc，
#   不改成 nvautogpuh264enc 會出問題。必須在 make 之前改，因為這個 plugin 是編譯時才建出來的。
#   已經內含此修改的新版 PX4 會直接通過；兩種寫法都找不到時 build 會失敗，
#   避免 PX4 升版後修補「悄悄沒套用」。
#
# numpy 鎖版本：
#   numpy 由 apt 安裝（ROS 的 Python 套件依賴它），pip 無法解除安裝 apt 的套件。
#   鎖定成目前的版本，讓 pip 改選相容 numpy 1.x 的 matplotlib / pandas，
#   不去動 apt 的 numpy（也避免 ROS 套件遇到 numpy 2 而壞掉）。
RUN set -eux; \
    git clone --branch "${PX4_VERSION}" --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/PX4/PX4-Autopilot.git "${PX4_DIR}"; \
    gst_camera_file="${PX4_DIR}/src/modules/simulation/gz_plugins/gstreamer/GstCameraSystem.cpp"; \
    if grep -q 'gst_element_factory_make("nvh264enc"' "${gst_camera_file}"; then \
        sed -i 's/gst_element_factory_make("nvh264enc"/gst_element_factory_make("nvautogpuh264enc"/' \
            "${gst_camera_file}"; \
    fi; \
    grep -q 'gst_element_factory_make("nvautogpuh264enc"' "${gst_camera_file}"; \
    numpy_ver="$(python3 -c 'import numpy; print(numpy.__version__)' 2>/dev/null || true)"; \
    if [ -n "${numpy_ver}" ]; then \
        echo "numpy==${numpy_ver}" > /tmp/pip-constraints.txt; \
        export PIP_CONSTRAINT=/tmp/pip-constraints.txt; \
    fi; \
    RUNS_IN_DOCKER=true bash "${PX4_DIR}/Tools/setup/ubuntu.sh" --no-nuttx; \
    unset PIP_CONSTRAINT; \
    rm -f /tmp/pip-constraints.txt; \
    make -C "${PX4_DIR}" px4_sitl; \
    chmod -R a+rwX "${PX4_DIR}"; \
    rm -rf /var/lib/apt/lists/*

# PX4 的 make 會呼叫 git；目錄擁有者是 root，執行者是 ncrl，
# 不加這行 git 會報 "dubious ownership" 導致 make 失敗
RUN git config --system --add safe.directory '*'

RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-jazzy-ros-gz-bridge \
 && rm -rf /var/lib/apt/lists/*
