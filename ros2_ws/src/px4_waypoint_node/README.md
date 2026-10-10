# px4_waypoint_node

PX4 無人機端的節點：offboard 控制（起飛並懸停、代替 BT 的 task_executor），以及把 PX4 的狀態轉給通用 cbba_node 的
`px4_state_bridge`。cbba_node 本身在 `cbba_core`，不依賴 PX4。

## 分散式架構

每台無人機的節點跑在**自己的 uav 容器**（代表機上電腦）：

```
uavN 容器：takeoff_hover → 本機 XRCE Agent → uavN 的線 → sim 裡的 PX4 #N
gcs 容器： hover_report.py record（經 mesh 只監控，不下指令）
```

控制路徑不經過 mesh，台與台之間互不依賴。同一份程式靠 `UAV_NS`（容器環境變數）分辨命名空間。

## 編譯

```bash
cd ~/CBBA_BT/ros2_ws
colcon build --symlink-install --packages-select px4_waypoint_node   # 在 uav1 執行一次，所有容器共用
source install/setup.bash
```

## takeoff_hover

```bash
ros2 run px4_waypoint_node takeoff_hover                       # 命名空間取自 UAV_NS
ros2 run px4_waypoint_node takeoff_hover --ros-args -p altitude:=5.0
```

| 參數 | 預設 | 說明 |
|---|---|---|
| `px4_ns` | `$UAV_NS` | PX4 話題前綴，例如 `uav1` → `/uav1/fmu/...` |
| `altitude` | 5.0 | 起飛高度（公尺，相對起飛點） |
| `rate_hz` | 10 | setpoint 頻率（PX4 要求至少 2 Hz） |
| `reach_tolerance` | 0.3 | 視為到達高度的誤差 |
| `auto_start` | true | 準備好就自動起飛 |

流程：`WAIT_READY`（飛行前檢查通過、位置有效）→ `STREAM`（先送 1 秒 setpoint）→
`ENGAGE`（切 offboard、解鎖，每秒重送）→ `CLIMB` → `HOVER`。

注意事項：

- PX4 v1.17 的 `vehicle_status`、`vehicle_local_position` 話題帶 `_v1` 後綴。
- SITL 實例 `-i N` 的 system id 是 **N+1**，節點從 `vehicle_status.system_id` 讀取，不寫死。
- 每台的 local 座標以自己的出生點為原點（NED）。
- offboard 需要持續的 setpoint：**停掉節點 PX4 會觸發 failsafe**。測試結束時直接關 PX4／Gazebo。

## task_executor（代替 BT，測試 cbba_node 用）

```bash
ros2 run px4_waypoint_node task_executor        # 機號、命名空間取自 UAV_ID、UAV_NS
```

起飛流程同 takeoff_hover，之後依 `/uavN/assigned_task` 飛到任務點正上方 `altitude` 公尺，
停留 `duration_sec` 後對 `/uavN/task_result` 發 `swarm_interfaces/TaskResult`（`success: true`，`assignment_version` 帶回目前 assigned_task 的版本），回到懸停等下一個指派。指派變成 task_id = 0 時原地懸停。

| 參數 | 預設 | 說明 |
|---|---|---|
| `agent_id` | `$UAV_ID` | 機號，用來算出生點 |
| `px4_ns` | `$UAV_NS` | PX4 話題前綴 |
| `altitude` | 5.0 | 飛行高度（公尺，相對出生點） |
| `cruise_speed` | 5.0 | setpoint 移動速度（m/s），要和 cbba_node 的 `cruise_speed` 一致 |
| `reach_tolerance` | 0.5 | 視為到達的距離 |
| `spawn_enu` | (0, (id−1)·`UAV_SPAWN_SPACING`, 0) | 出生點（map ENU），要和 px4_sitl.sh、px4_state_bridge 一致 |

通常用 `docker/scripts/cbba_uav.sh` 和 px4_state_bridge、cbba_node 一起啟動，步驟見 `cbba_core/doc/sitl_test.md`。

## px4_state_bridge（PX4 → cbba_node 的轉接節點）

通用 cbba_node 只吃標準介面 `RobotState`（map ENU 的位置、電量、飛行狀態）。PX4 的格式不一樣（4 個話題、NED、以出生點為原點），
所以由這個節點轉換：

```
/uavN/fmu/out/vehicle_local_position_v1 ┐
/uavN/fmu/out/vehicle_status_v1         ├─→ px4_state_bridge ─→ /uavN/robot_state ─→ cbba_node
/uavN/fmu/out/battery_status_v1         │   NED → map ENU、加出生點
/uavN/fmu/out/vehicle_land_detected     ┘   真實的 armed／offboard／landed
```

```bash
ros2 run px4_waypoint_node px4_state_bridge --ros-args -r __node:=uav1_px4_state_bridge   # 機號、命名空間取自 UAV_ID、UAV_NS
```

| 參數 | 預設 | 說明 |
|---|---|---|
| `agent_id` | `$UAV_ID` | 機號，填進 RobotState、用來算出生點；要和 cbba_node 一樣 |
| `px4_ns` | `$UAV_NS` | PX4 話題前綴，robot_state 也發在這底下；要和 cbba_node 的 `ns` 一樣 |
| `position_timeout` | 0.5 s | PX4 的位置這麼久沒更新就停止送（PX4 斷線） |
| `status_timeout` | 2.0 s | `vehicle_status` 這麼久沒更新，`flight_state_valid` 就是 false |
| `rate_hz` | 10 | 發送頻率 |
| `spawn_enu` | (0, (id−1)·`UAV_SPAWN_SPACING`, 0) | 出生點（map ENU），要和 px4_sitl.sh、task_executor 一致 |

| RobotState 欄位 | 來源 |
|---|---|
| `armed` | `vehicle_status.arming_state == ARMING_STATE_ARMED` |
| `offboard` | `vehicle_status.nav_state == NAVIGATION_STATE_OFFBOARD` |
| `landed` | `vehicle_land_detected.landed` |
| `flight_state_valid` | 收到過 `vehicle_status` 和 `vehicle_land_detected`，而且 `vehicle_status` 在 `status_timeout` 內有更新。逾時後 armed 等保留最後的值但不算數 |

- PX4 的位置有在更新就一直送，**地面上、未解鎖時也送**，鄰居的 AGENT_STATE 才看得到真實的狀態。
  要不要出價由 cbba_node 判斷：飛行狀態有效、已解鎖，而且（`require_airborne`，預設）沒有 landed。
- PX4 斷線（位置沒在更新）時停止送，cbba_node 在 `state_timeout`（1.5 s）後停止參與出價、釋放任務。
- cbba_node 收到的第一筆位置當返航點（只用在出價時估返航的耗電），也就是地面的出生點。
- 電池沒連上時電量送 −1，cbba_node 沿用上一筆。

## 測試紀錄與報告：hover_report.py

```bash
# gcs：經 mesh 紀錄（events.csv 可另外附加 time,label 標示階段）
ros2 run px4_waypoint_node hover_report.py record <輸出目錄> --duration 170
# 有 matplotlib 的容器（sim）：畫圖並判定
python3 scripts/hover_report.py plot <輸出目錄>
```

驗收標準（預設）：進入 ±0.3 m 後等 2 秒，接下來 30 秒高度誤差 ≤ 0.3 m、水平偏移 ≤ 0.5 m、全程維持 offboard。
輸出 `takeoff_hover.png`、`summary.csv`。
