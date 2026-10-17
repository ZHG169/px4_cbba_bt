# px4_cbba_bt

空地異質自主系統的**無人機端**：PX4 旋翼無人機以 CBBA 分散式拍賣分配任務，搭配行為樹（BT）執行，
與 Ghost Robotics V60 機器狗經由 Ad-hoc mesh 協同。

閉環情境（週計畫）：無人機飛巡檢點 → 下視相機看到 AprilTag 101（火警）→ 建立地面處置任務 → 懸停監控 →
機器狗競標、前往處理 → 處理完 → 無人機完成、回起飛點。

## 架構

```
sim ─┬─ sim_uav1_net（線）─ uav1 ─┐
     ├─ sim_uav2_net（線）─ uav2 ─┼─ mesh_net（無線電，可加 tc 丟包）─ gcs
     └─ ...                       ┘
```

| 容器 | 代表 | 主要內容 |
|---|---|---|
| sim | 真實世界＋所有飛控 | Gazebo、每台無人機一個 PX4 SITL、相機橋接、假狗的 Gazebo 替身 |
| uav1、uav2… | 每台無人機的機上電腦 | XRCE Agent、px4_state_bridge、cbba_node、BT（uav_task_manager_node）、AprilTag 偵測 |
| gcs | 地面站 | 監控；測試時可以跑機器狗的 cbba_node |

- 控制路徑（節點 → XRCE Agent → PX4）走每台自己的線，**不經過 mesh**；**DDS 只在機內**。
- 機間只有兩條 UDP multicast：
  - **CBBA 協定版本 2**（`cbba_node`，port 14600）：每筆任務資料帶 task_id，不相容於「機間通訊封包規格」2026-10-06，
    見 `ros2_ws/src/cbba_core/doc/protocol.md`。
  - **飛行層位置廣播**（無人機 BT 的 `flight_beacon`，port 14601，10 Hz）：給 CBF 避碰用的鄰機位置與速度。
- 用 `docker/scripts/mesh_tc.sh` 模擬丟包與延遲。

每台載具（無人機、機器狗）跑**同一個 `swarm_cbba/cbba_node`**（`agent_type: UAV／UGV`），它不依賴 PX4、BT、Nav2，
只透過固定格式的話題（`swarm_interfaces`）和機上的 BT 溝通：

```
            ┌──────────────── 每台載具的機上電腦 ────────────────┐
 其他載具 ◄═UDP═► cbba_node ◄─ robot_state ── 狀態轉接（無人機：px4_state_bridge）
 （協定版本 2）       │  ▲
   current_task       │  │ task_result、new_task（例如火警）
   task_closed        ▼  │
   task_announced*     BT（無人機：uav_px4_bt；狗：伯宇的 task_manager）→ PX4／Nav2
            └──────────────────────────────────────────────────┘
   * 只有無人機的 cbba_node 發（全隊的火點去重複）；狗的 cbba_node 和介面不變
```

話題與規則見 `ros2_ws/src/swarm_cbba/README.md`（照機器狗端的介面）、`swarm_interfaces/README.md`。
**機器狗端的使用說明**：`ros2_ws/src/swarm_cbba/README_v60.md`。

### 無人機的 BT（`uav_px4_bt`）

| 執行檔 | 說明 |
|---|---|
| `uav_task_manager_node`（正式） | 照機器狗端（伯宇）的 task_manager 骨架：急停 → 哨兵 → 巡檢佇列；葉節點 `NavigateTo`、`AirRecon` |
| `uav_bt_node`（備用） | 10/09 的版本，行為大致相同 |

- 移動：名目速度（剎車曲線）→ **CBF 避碰**（控制障礙函數，2026-10-14 取代 CPF）→ 運動平滑層 → PX4 位置＋速度 setpoint。
  公式與參數：`ros2_ws/src/uav_px4_bt/doc/parameters.md` 第五節；值在 `uav_px4_bt/config/uav_bt.yaml`。
- 火警：飛行途中或到點懸停時看到 tag 101 → 發布地面處置任務 → **飛到火點上空懸停監控**，等地面任務結束（狗處理完）
  或 `fire_task.monitor_timeout` → 回原本的航線或完成；別台已經報過的火不再發布、不轉向。
- 起飛、待命盤旋、讓路、做完回起飛點、低電量返航在樹外（`skeleton/uav_flight.hpp`）。
- **被接管**（2026-10-17，照機器狗端的平板接管）：起飛完成後 offboard 掉了 → PX4 的 `failsafe` = 0（遙控器、地面站切了模式）
  → 手上的任務以 `PREEMPTED_BY_MANUAL` 交還（5 s 不接、不算失敗次數）；`failsafe` = 1（PX4 自己的失效保護）→ `PX4_FAILSAFE`（STOPPED）。
  期間 mode MANUAL、不出價；切回 offboard 就從目前位置接回、mode AUTO（照狗：直接放行）。
- **mode 由 BT 決定**：正式 BT 送 `/uavN/bt_mode`，`px4_state_bridge` 照抄進 robot_state（起飛完成只有 BT 一套標準）；
  備用 BT 時 bridge 自己判斷（在空中、爬到巡檢高度 − 0.5 m、offboard）。

## 目錄

```
docs/                         跨套件的文件 → docs/README.md：模擬測試的開啟方式（sim_launch_guide.md）、
                              插入任務點的使用方法（task_insertion_guide.md）、目前進度（progress.md）、
                              Nav2 導入規劃（nav2_integration_plan.md）、協定版本 2 的變更通知
docker/                       模擬環境（Dockerfile、啟動腳本、Fast DDS、mesh tc）→ docker/README.md
├── gz/                       Gazebo 場景（fire_site、patrol_site、demo_site）、模型（AprilTag、火點方塊、藍色任務平台）、
│                             視窗設定；demo_site、patrol_site 由 gen_patrol_site.py 從 *_points.csv 產生
├── itest/                    自動整合測試（comm、uav、loss、demo）、情境檔（scenarios/）、手動插入（insert.py）
│                             → docker/itest/README.md
└── scripts/                  cbba_uav.sh、px4_sitl.sh、gz_bridge.sh、cbba_task.sh、patrol_scenario.sh、fake_dog.py、mesh_tc.sh…
ros2_ws/src/
├── swarm_interfaces/         ROS 訊息（Task、RobotState、TaskResult…）→ swarm_interfaces/README.md
├── cbba_core/                CBBA 核心、機間封包、協定層、指派流程、離線模擬、單元測試（不依賴 ROS）→ cbba_core/README.md
│   └── doc/                  協定（protocol.md）、參數（parameters.md）、SITL 手動測試（sitl_test.md）、機器狗、外部規格書
├── swarm_cbba/               統一版 cbba_node（無人機和狗同一支）、launch、yaml → swarm_cbba/README.md
├── px4_waypoint_node/        takeoff_hover、task_executor（簡易執行者）、px4_state_bridge（PX4 → RobotState）
├── uav_px4_bt/               無人機的 BT（正式、備用）、CBF、運動平滑層、位置廣播 → uav_px4_bt/doc/parameters.md
└── apriltag_fire_detector/   下視相機的 AprilTag 偵測 → /uavN/fire_detection
```

`px4_msgs` 不在 repo 裡，由 `docker/scripts/setup_px4_msgs.sh` 依 PX4 版本（v1.17.0）下載。
隊友的 handoff 資料（`docs/handoff_uav_*` 等）不進 repo（`.gitignore`）。

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
  swarm_interfaces cbba_core swarm_cbba px4_waypoint_node uav_px4_bt apriltag_fire_detector
source install/setup.bash
```

切換分支後要重新編譯；不要在兩個容器同時編譯（共用同一個 build/、install/）。
只改 yaml、Python、shell、場景 SDF 不用重新編譯。

**3. 自動整合測試**（host，最快看到整條流程）

```bash
python3 docker/itest/itest.py uav                          # UAV 整串 U1～U6（fire_site，不開視窗）
python3 docker/itest/itest.py demo                         # 展示：Gazebo 俯視錄影＋俯視動畫（含電量）
python3 docker/itest/itest.py demo --scenario cbba_overrun # CBBA 正常分配下的火警監控、超時交還、CBF 交會
```

**手動插入任務點**（兩個 terminal，說明見 `docs/task_insertion_guide.md`）：

```bash
# terminal 1：開環境（G1～G3 自動建立）
python3 docker/itest/itest.py demo --interactive --area -15 15 -12 8
# terminal 2：輸入 t（範圍內隨機任務點）、t x y、tf（任務點＋起火）、f（只起火）、l、q
python3 docker/itest/insert.py
```

**4. 手動開 PX4 SITL**

| Terminal | 容器 | 指令 |
|---|---|---|
| 1、2、3 | uav1、uav2、uav3 | `EXECUTOR=task_manager cbba_uav.sh`（XRCE Agent＋BT＋AprilTag 偵測＋px4_state_bridge＋cbba_node；不給 `EXECUTOR` 時是備用 BT） |
| 4 | sim | `CBBA_WORLD=demo_site px4_sitl.sh 1`，再 `px4_sitl.sh 2`、`px4_sitl.sh 3`（第 1 台開 Gazebo） |
| 5 | sim | `gz_bridge.sh`（相機、`/clock`；要偵測火警時開） |
| 6 | uav1 | `cbba_task.sh new 1 1 10 0` 建立任務 |

每台起飛到巡檢高度後參與出價，得標的飛過去、停留後回報完成。各種測試的開啟方式整理在 **`docs/sim_launch_guide.md`**。

**5. 單元測試與離線模擬**（uav 容器）

```bash
cd ~/CBBA_BT/ros2_ws
colcon test --packages-select cbba_core uav_px4_bt && colcon test-result --verbose
bash src/swarm_cbba/test/g2_e2e.sh                # cbba_node 回歸測試（機器狗端的 T1～T10 等，不用 PX4）

SCN=src/cbba_core/scenarios
ros2 run cbba_core cbba_sim run $SCN/fire_3uav.csv --loss 0 --out /tmp/fire_3uav
ros2 run cbba_core cbba_sim insert ~/CBBA_BT/docker/gz/worlds/patrol_site_points.csv --out /tmp/insert
```

## 目前進度（2026-10-17）

| 項目 | 狀態 |
|---|---|
| PX4 SITL＋XRCE-DDS 模擬環境（多台）、mesh＋tc 弱網 | ✅ |
| CBBA 核心（17 條消解規則、連鎖退標、電池計分）＋單元測試 | ✅ |
| 機間通訊協定版本 2、協定層、UDP（擴散、完成確認、失敗交回、失聯接手、重開機、鎖定、指派版本） | ✅ itest comm；30% 掉包 5 s 內收斂 95% |
| 統一版 cbba_node（無人機、機器狗共用，照機器狗端介面） | ✅ `g2_e2e.sh`；全隊火點去重複（`task_announced`，只有無人機發） |
| 無人機 BT（照機器狗端骨架）：起飛、巡檢、火警懸停監控、路過發現轉向、回起飛點、低電量返航 | ✅ itest uav 6／6、demo 全過 |
| 避碰：CPF → **CBF**（2026-10-14） | ✅ SITL：正對懸停的鄰機時往右繞開；參數之後再調 |
| AprilTag 火情偵測、Gazebo 場景（fire_site、patrol_site、demo_site）、俯視錄影 | ✅ |
| 動態重新分配（執行超時交還、PX4 斷線接手） | ✅ itest U6、`--scenario cbba_overrun` |
| 展示與測試工具：情境檔、兩個 terminal 手動／隨機插入、俯視動畫（電量長條）、D7 CBF 距離 | ✅ |
| 無人機被人工接管（切出 offboard）時停止出價、交還任務 | ✅ 單元測試；SITL 已確認分得出人接管和 PX4 失效保護（`watch_px4_status.py`），整串待 SITL 重測 |
| 機器狗 | 假狗（`fake_dog.py`）✅；真的 V60 ⏳ 等伯宇換成協定版本 2 |
| Nav2 接入模擬（伯宇的 task_manager＋Nav2 取代假狗） | ⏳ 規劃見 `docs/nav2_integration_plan.md` |
| 能量模型校正（實機 ulog）、Jetson、實機 mesh | ⏳ |

缺失清單與週進度對照：`docs/progress.md`。

## 授權

MIT
