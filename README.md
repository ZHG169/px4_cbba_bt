# px4_cbba_bt

空地異質自主系統的**無人機端**：PX4 旋翼無人機以 CBBA 分散式拍賣分配任務，搭配行為樹（BT）執行，
與 Ghost Robotics V60 機器狗經由 Ad-hoc mesh 協同。

## 架構

```
sim ─┬─ sim_uav1_net（線）─ uav1 ─┐
     ├─ sim_uav2_net（線）─ uav2 ─┼─ mesh_net（無線電，可加 tc 丟包）─ gcs
     └─ ...                       ┘
```

| 容器 | 代表 | 主要內容 |
|---|---|---|
| sim | 真實世界＋所有飛控 | Gazebo、每台無人機一個 PX4 SITL |
| uav1、uav2… | 每台無人機的機上電腦 | XRCE Agent、控制節點、CBBA |
| gcs | 地面站 | 任務注入、監控 |

- 控制路徑（節點 → XRCE Agent → PX4）走每台自己的線，**不經過 mesh**。
- 機間協商（`/swarm/tasks`、`/swarm/cbba`）走 mesh，可用 `mesh_tc.sh` 模擬丟包與延遲。

## 目錄

```
docker/                       模擬環境（Dockerfile、啟動腳本、Fast DDS、mesh tc）→ docker/README.md
ros2_ws/src/
├── swarm_interfaces/         CBBA 訊息與介面規格 v1.0 → swarm_interfaces/README.md
├── uav_cbba/                 CBBA 核心（不依賴 ROS）、離線模擬、畫圖、單元測試 → uav_cbba/README.md
│   └── test/results/         測試結果（只保留圖片）
└── px4_waypoint_node/        PX4 offboard 控制：起飛並懸停 → px4_waypoint_node/README.md
```

`px4_msgs` 不在 repo 裡，由 `docker/scripts/setup_px4_msgs.sh` 依 PX4 版本（v1.17.0）下載。

## 快速開始

**1. 建置並啟動容器**（host）

```bash
cd docker
./build.sh          # 第一次，或 Dockerfile 有改時
./start.sh          # 依 .env（UAV_COUNT 等）產生 docker-compose.yaml 並啟動
./enter.sh uav1     # 進入容器：sim | uav1 | uav2 | ... | gcs
```

**2. 編譯 ROS 2 工作區**（在 uav1 執行一次，所有容器共用編譯結果）

```bash
setup_px4_msgs.sh                     # 第一次：下載 px4_msgs
cd ~/CBBA_BT/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

只編部分套件：`colcon build --symlink-install --packages-select uav_cbba`。
不要在兩個容器同時編譯（共用同一個 build/、install/）。

**3. 分散式起飛懸停**

| Terminal | 容器 | 指令 |
|---|---|---|
| 1 | uav1 | `xrce_agent.sh` |
| 2 | uav2 | `xrce_agent.sh` |
| 3 | sim | `px4_sitl.sh 1`（同時開啟 Gazebo） |
| 4 | sim | `px4_sitl.sh 2` |
| 5 | uav1 | `ros2 run px4_waypoint_node takeoff_hover` |
| 6 | uav2 | `ros2 run px4_waypoint_node takeoff_hover` |

起飛到 5 m 後原地懸停（不含降落）。結束時直接關閉 PX4／Gazebo，詳見 `px4_waypoint_node/README.md`。

**4. CBBA 測試與離線模擬**（uav 容器）

```bash
cd ~/CBBA_BT/ros2_ws
colcon test --packages-select uav_cbba --event-handlers console_direct+

SCN=src/uav_cbba/scenarios
ros2 run uav_cbba cbba_sim run $SCN/fire_3uav.csv --loss 0 --out /tmp/fire_3uav
```

畫圖與 GIF（需要 matplotlib，sim 容器裡有）：

```bash
python3 src/uav_cbba/scripts/plot_results.py /tmp/fire_3uav --scenario $SCN/fire_3uav.csv
python3 src/uav_cbba/scripts/animate_results.py mission /tmp/fire_3uav $SCN/fire_3uav.csv
```

## 目前進度

| 項目 | 狀態 |
|---|---|
| PX4 SITL＋XRCE-DDS 模擬環境（多台） | ✅ |
| Ad-hoc mesh 與 tc 弱網環境 | ✅ |
| 分散式起飛懸停（驗收 PASS，結果見 `uav_cbba/test/results/takeoff_hover/`） | ✅ |
| CBBA 訊息與介面規格 v1.0 | ✅ 待隊友審閱 |
| CBBA 核心（17 條消解規則、連鎖退標、電池計分）＋單元測試 | ✅ |
| 3D 航點介面 | ⏳ |
| CBBA ROS 2 節點（接上 `/swarm/tasks`、`/swarm/cbba`） | ⏳ |
| AprilTag 火情偵測、無人機行為樹 | ⏳ |

## 授權

MIT
