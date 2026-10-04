# px4_waypoint_node

PX4 無人機的 offboard 控制介面。目前只有**起飛並懸停**（不含降落），第 2 週再加 3D 航點。

## 分散式架構

每台無人機的節點跑在**自己的 uav 容器**（代表機上電腦）：

```
uavN 容器：takeoff_hover → 本機 XRCE Agent → uavN 的線 → sim 裡的 PX4 #N
gcs 容器： hover_report.py record（經 mesh 只監控，不下指令）
```

控制路徑不經過 mesh，台與台之間互不依賴。同一份程式靠 `UAV_NS`（容器環境變數）分辨命名空間。

## 編譯

```bash
cbuild --packages-select px4_waypoint_node     # 在 uav1 執行一次，所有容器共用
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

## 測試紀錄與報告：hover_report.py

```bash
# gcs：經 mesh 紀錄（events.csv 可另外附加 time,label 標示階段）
ros2 run px4_waypoint_node hover_report.py record <輸出目錄> --duration 170
# 有 matplotlib 的容器（sim）：畫圖並判定
python3 scripts/hover_report.py plot <輸出目錄>
```

驗收標準（預設）：進入 ±0.3 m 後等 2 秒，接下來 30 秒高度誤差 ≤ 0.3 m、水平偏移 ≤ 0.5 m、全程維持 offboard。
輸出 `takeoff_hover.png`、`summary.csv`。
