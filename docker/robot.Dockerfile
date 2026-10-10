# syntax=docker/dockerfile:1
# ============================================================
#  cbba-robot / cbba-gcs：機載電腦與地面站
#  - Micro-XRCE-DDS-Agent（在 /tmp 編譯，裝完即刪原始碼）
#  - WITH_GUI=1 時額外安裝 RViz2（給 gcs 用）
#  px4_msgs 不在映像裡，放在 ros2_ws/src/px4_msgs，由 cbuild 編譯
# ============================================================
ARG BASE_IMAGE=cbba-base:jazzy
FROM ${BASE_IMAGE}

ARG XRCE_AGENT_VERSION=v2.4.3
ARG WITH_GUI=0

RUN set -eux; \
    git clone --branch "${XRCE_AGENT_VERSION}" --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce_agent; \
    cmake -S /tmp/xrce_agent -B /tmp/xrce_agent/build -DCMAKE_BUILD_TYPE=Release; \
    cmake --build /tmp/xrce_agent/build --parallel "$(nproc)"; \
    cmake --install /tmp/xrce_agent/build; \
    ldconfig; \
    rm -rf /tmp/xrce_agent; \
    command -v MicroXRCEAgent

# 無人機的備用 BT（uav_px4_bt：BehaviorTree.CPP v4）與 AprilTag 火情偵測（apriltag_fire_detector：apriltag C 函式庫）
RUN apt-get update \
 && apt-get install -y --no-install-recommends ros-jazzy-behaviortree-cpp ros-jazzy-apriltag \
 && rm -rf /var/lib/apt/lists/*

RUN if [ "${WITH_GUI}" = "1" ]; then \
        apt-get update \
     && apt-get install -y --no-install-recommends ros-jazzy-rviz2 \
     && rm -rf /var/lib/apt/lists/*; \
    fi
