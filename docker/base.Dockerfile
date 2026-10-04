# syntax=docker/dockerfile:1
# ============================================================
#  cbba-base：所有容器共用的基礎
#  - ROS 2 Jazzy (ros-base，之後可直接用在樹莓派 / Jetson)
#  - 使用者 ncrl（UID 在「執行時」由 entrypoint 對應成 host 的 UID）
# ============================================================
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      build-essential cmake git curl wget ca-certificates sudo tmux vim \
      gosu gettext-base libxml2-utils \
      iproute2 iputils-ping tcpdump \
      python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool ros-dev-tools \
 && rm -rf /var/lib/apt/lists/*

# 建立 ncrl。Ubuntu 24.04 映像內建 ubuntu (UID 1000)，直接改名沿用。
RUN set -eux; \
    if id ubuntu >/dev/null 2>&1; then \
        usermod -l ncrl -d /home/ncrl -m ubuntu; \
        groupmod -n ncrl ubuntu; \
    else \
        useradd -m -u 1000 -s /bin/bash ncrl; \
    fi; \
    usermod -s /bin/bash ncrl; \
    echo "ncrl ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ncrl; \
    chmod 0440 /etc/sudoers.d/ncrl

# 先用一般權限 (755) 建立資料夾。
# 若讓 COPY --chmod 自動建立資料夾，BuildKit 會把 0644 也套到資料夾上，
# 資料夾少了 x，ncrl 就進不去（會出現 Permission denied）。
RUN mkdir -p /usr/local/share/cbba && chmod 0755 /usr/local/share/cbba

# 映像裡只放「轉接」檔，真正的 entrypoint.sh / bashrc_cbba.sh 從掛載的專案目錄讀取，
# 之後修改它們只要重啟容器，不需要重新 build（也不會觸發 PX4 重新編譯）
COPY --chmod=0755 docker/entrypoint_stub.sh /usr/local/bin/cbba_entrypoint.sh
COPY --chmod=0644 docker/bashrc_loader.sh /usr/local/share/cbba/bashrc.sh
COPY docker/tmux.conf /home/ncrl/.tmux.conf

RUN echo 'source /usr/local/share/cbba/bashrc.sh' >> /home/ncrl/.bashrc \
 && chown -R ncrl:ncrl /home/ncrl \
 && gosu ncrl test -r /usr/local/share/cbba/bashrc.sh \
 && gosu ncrl test -r /home/ncrl/.tmux.conf

RUN (rosdep init 2>/dev/null || true) \
 && gosu ncrl rosdep update --rosdistro jazzy

WORKDIR /home/ncrl/CBBA_BT

# 容器以 root 啟動，entrypoint 對應完 UID 後才切換成 ncrl
ENTRYPOINT ["/usr/local/bin/cbba_entrypoint.sh"]
CMD ["sleep", "infinity"]
