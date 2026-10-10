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
| uav1、uav2… | 每台無人機的機上電腦 | XRCE Agent、px4_state_bridge、cbba_node、uav_bt_node（BT）、fire_detector_node（AprilTag） |
| gcs | 地面站 | 任務注入、監控；測試時可以跑假的機器狗 cbba_node |

- 控制路徑（節點 → XRCE Agent → PX4）走每台自己的線，**不經過 mesh**。
- 機間協商走 mesh 上的 **UDP multicast**，協定版本 2（2026-10-09，每筆任務資料帶 task_id；不相容於「機間通訊封包規格」
  2026-10-06，見 `ros2_ws/src/cbba_core/doc/protocol.md`）。DDS 只用在機內（PX4、BT）。可用 `mesh_tc.sh` 模擬丟包與延遲。

每台載具（無人機、機器狗）跑**同一個 `cbba_node`**（`vehicle_type:=uav`／`ugv`），它不依賴 PX4、BT、Nav2，
只透過固定格式的話題（`swarm_interfaces`）和機上的 BT 溝通；機間走固定格式的 UDP 封包。

```
            ┌──────────── 每台載具的機上電腦 ────────────┐
 其他載具 ◄═UDP═► cbba_node ◄─ robot_state ── 狀態轉接（無人機：px4_state_bridge）
 （協定版本 2）       │  ▲
          assigned_task │  │ task_result、exec_state、new_task
     （狗：assignment_request／response 接受握手）
                      ▼  │
                       BT（無人機：uav_px4_bt；狗：V60 的 BT）→ PX4／Nav2
            └────────────────────────────────────────────┘
```

話題與規則見 `ros2_ws/src/cbba_core/README.md`、`swarm_interfaces/README.md`。

## 目錄

```
docs/                         模擬測試的開啟方式（sim_launch_guide.md）、第 4～6 週和程式的差異（week4_6_status.md）、
                              巡檢＋隨機插入與電池權重測試（sim_scenario_plan.md）、協定版本 2 的變更通知
docker/                       模擬環境（Dockerfile、啟動腳本、Fast DDS、mesh tc）→ docker/README.md
├── gz/                       Gazebo 場景（fire_site、patrol_site）與 AprilTag 模型
└── scripts/                  cbba_uav.sh、px4_sitl.sh、gz_bridge.sh、cbba_task.sh、patrol_scenario.sh、mesh_tc.sh…
ros2_ws/src/
├── swarm_interfaces/         ROS 訊息（Task 等）→ swarm_interfaces/README.md
├── cbba_core/                CBBA 核心、機間封包、協定層、通用 cbba_node（無人機和狗共用、不依賴 PX4）、
│   │                         離線模擬、單元測試 → cbba_core/README.md
│   ├── doc/                  封包規格書、參數說明（parameters.md）、SITL 測試步驟（sitl_test.md）、
│   │                         機器狗的執行與測試（ugv_test.md、ugv_tuning.md）
│   └── test/results/         測試結果（只保留圖片）
├── px4_waypoint_node/        PX4 端：takeoff_hover 起飛懸停、task_executor（簡易執行者）、
│                             px4_state_bridge（PX4 → cbba_node 的 RobotState）→ px4_waypoint_node/README.md
├── uav_px4_bt/               無人機的備用 BT（BehaviorTree.CPP v4）：起飛、CPF 避碰、火警判定、低電量返航
└── apriltag_fire_detector/   下視相機的 AprilTag 偵測（confidence）→ /uavN/fire_detection
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
colcon build --symlink-install --packages-select \
  swarm_interfaces cbba_core px4_waypoint_node uav_px4_bt apriltag_fire_detector
source install/setup.bash
```

只編部分套件：`colcon build --symlink-install --packages-select cbba_core`。切換分支後要重新編譯。
不要在兩個容器同時編譯（共用同一個 build/、install/）。

**3. CBBA 分配任務（PX4 SITL）**

| Terminal | 容器 | 指令 |
|---|---|---|
| 1、2、3 | uav1、uav2、uav3 | `cbba_uav.sh`（XRCE Agent＋BT＋AprilTag 偵測＋px4_state_bridge＋cbba_node；`EXECUTOR=task_executor` 換成簡易執行者） |
| 4 | sim | `px4_sitl.sh 1`、`px4_sitl.sh 2`、`px4_sitl.sh 3`（第 1 台同時開 Gazebo；`CBBA_WORLD=fire_site`／`patrol_site` 換場景） |
| 5 | uav1 | `cbba_task.sh new 1 1 10 0` 建立任務 |

每台起飛到 5 m 後參與出價，得標的飛過去、停留後回報完成。各種測試（基本分配、火情偵測、
巡檢＋隨機插入、弱網、離線模擬）的開啟方式整理在 **`docs/sim_launch_guide.md`**；
細節見 `ros2_ws/src/cbba_core/doc/sitl_test.md`。

只要起飛懸停的話：每台 `xrce_agent.sh`，sim 開 `px4_sitl.sh N`，再 `ros2 run px4_waypoint_node takeoff_hover`
（見 `px4_waypoint_node/README.md`）。

**4. CBBA 測試與離線模擬**（uav 容器）

```bash
cd ~/CBBA_BT/ros2_ws
colcon test --packages-select cbba_core --event-handlers console_direct+

SCN=src/cbba_core/scenarios
ros2 run cbba_core cbba_sim run $SCN/fire_3uav.csv --loss 0 --out /tmp/fire_3uav

# 巡檢＋隨機插入：100 次 × 電池權重 0／1／5／20／100（docs/sim_scenario_plan.md）
ros2 run cbba_core cbba_sim insert ~/CBBA_BT/docker/gz/worlds/patrol_site_points.csv --out /tmp/insert
```

畫圖與 GIF（需要 matplotlib，sim 容器裡有）：

```bash
python3 src/cbba_core/scripts/plot_results.py /tmp/fire_3uav --scenario $SCN/fire_3uav.csv
python3 src/cbba_core/scripts/animate_results.py mission /tmp/fire_3uav $SCN/fire_3uav.csv
```

## 目前進度

| 項目 | 狀態 |
|---|---|
| PX4 SITL＋XRCE-DDS 模擬環境（多台） | ✅ |
| Ad-hoc mesh 與 tc 弱網環境 | ✅ |
| 分散式起飛懸停（驗收 PASS，結果見 `cbba_core/test/results/takeoff_hover/`） | ✅ |
| CBBA 核心（17 條消解規則、連鎖退標、電池計分）＋單元測試 | ✅ |
| 機間通訊協定版本 2（task_id 識別任務、分批、session）、協定層、UDP | ✅ 真 UDP 整合測試（含正式指派版本、取消權限、BT 接受／搶占／排隊）；SITL ⏳ |
| 通用 cbba_node（無人機、機器狗共用） | ✅ 真 UDP＋假狗的整合測試；真的狗 ⏳ |
| cbba_node：PX4 SITL 驗證分配、完成確認、失敗交回、墜毀重分配、晚加入、重開機 | ✅（協定版本 1、task_executor） |
| 無人機行為樹（起飛、CPF 避碰、火警判定、返航） | ✅ 單元測試；SITL ⏳ |
| AprilTag 火情偵測、Gazebo 場景（fire_site、patrol_site） | ✅ 單元測試、場景載入；SITL ⏳ |
| 巡檢＋隨機插入、電池權重的離線模擬（mission_sim、cbba_sim insert） | ✅ |
| 全部單元測試 | ✅ 164 項、零警告（2026-10-10） |
| 能量模型校正（實機 ulog） | ⏳ |
| 弱網壓測（協定版本 2 的收斂統計、DDS 不經過 mesh） | ⏳ 見 `docs/week4_6_status.md` |

## 授權

MIT
