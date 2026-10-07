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
| uav1、uav2… | 每台無人機的機上電腦 | XRCE Agent、cbba_node、task_executor（暫代 BT） |
| gcs | 地面站 | 任務注入、監控 |

- 控制路徑（節點 → XRCE Agent → PX4）走每台自己的線，**不經過 mesh**。
- 機間協商走 mesh 上的 **UDP multicast**，封包依「機間通訊封包規格」2026-10-06
  （`ros2_ws/src/uav_cbba/doc/`）。DDS 只用在機內（PX4、BT）。可用 `mesh_tc.sh` 模擬丟包與延遲。

## 目錄

```
docker/                       模擬環境（Dockerfile、啟動腳本、Fast DDS、mesh tc）→ docker/README.md
ros2_ws/src/
├── swarm_interfaces/         ROS 訊息（Task 等）→ swarm_interfaces/README.md
├── uav_cbba/                 CBBA 核心、機間封包、協定層、cbba_node、離線模擬、單元測試 → uav_cbba/README.md
├── ugv_cbba/                 機器狗的 CBBA 節點（共用 uav_cbba 的核心與協定）→ ugv_cbba/README.md
│   ├── doc/                  封包規格書、參數說明（parameters.md）、SITL 測試步驟（sitl_test.md）
│   └── test/results/         測試結果（只保留圖片）
└── px4_waypoint_node/        PX4 offboard：takeoff_hover 起飛懸停、task_executor 暫代 BT → px4_waypoint_node/README.md
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

**3. CBBA 分配任務（PX4 SITL）**

| Terminal | 容器 | 指令 |
|---|---|---|
| 1、2、3 | uav1、uav2、uav3 | `cbba_uav.sh`（XRCE Agent＋task_executor＋cbba_node） |
| 4 | sim | `px4_sitl.sh 1`、`px4_sitl.sh 2`、`px4_sitl.sh 3`（第 1 台同時開 Gazebo） |
| 5 | uav1 | `cbba_task.sh new 1 1 10 0` 建立任務 |

每台起飛到 5 m 後參與出價，得標的飛過去、停留後回報完成。失敗交回、墜毀、晚加入的測試步驟見
`ros2_ws/src/uav_cbba/doc/sitl_test.md`。

只要起飛懸停的話：每台 `xrce_agent.sh`，sim 開 `px4_sitl.sh N`，再 `ros2 run px4_waypoint_node takeoff_hover`
（見 `px4_waypoint_node/README.md`）。

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
| CBBA 核心（17 條消解規則、連鎖退標、電池計分）＋單元測試 | ✅ |
| 機間通訊封包（依規格書 2026-10-06）、協定層、UDP | ✅ 106 項單元測試 |
| cbba_node：PX4 SITL 驗證分配、完成確認、失敗交回、墜毀重分配、晚加入、重開機 | ✅ |
| 移動時的避碰（CPF 之類） | ⏳ 目前 SITL 同高度直線飛，會撞機 |
| 能量模型校正（實機 ulog） | ⏳ |
| 3D 航點介面、AprilTag 火情偵測、無人機行為樹 | ⏳ |

## 授權

MIT
