# 無人機 BT 的飛行參數（2026-10-13）

適用 `uav_task_manager_node`（正式，伯宇骨架）和 `uav_bt_node`（備用），兩者共用 CBF、運動平滑層、PX4 介面。
預設值都在 `config/uav_bt.yaml`，這份說明每個參數的作用、調大／調小會怎樣、要和誰一致。

## 一、怎麼改

| 方式 | 用法 | 什麼時候用 |
|---|---|---|
| yaml | 改 `ros2_ws/src/uav_px4_bt/config/uav_bt.yaml`，重開 `cbba_uav.sh` | 常改的。`colcon build --symlink-install` 時 install 裡是連結，**不用重新編譯**（第一次加這個檔案要編譯一次） |
| 臨時 | `EXEC_ARGS="-p cbf.safe_radius:=1.5 -p motion.max_accel:=2.0" cbba_uav.sh` | 試一兩個值 |
| 環境變數 | `CBBA_SPEED=3 CBBA_ALTITUDE=6 cbba_uav.sh` | 速度、高度：**cbba_node 和 BT 一起改**（兩邊不一致時出價估的時間不準） |
| 樹（XML） | `trees/uav_mission.xml` 的葉節點屬性（第六節） | 火警判定的門檻、相機範圍 |
| itest | `python3 docker/itest/itest.py demo --speed 3` | 測試時的速度 |

優先順序：`-p`（`cbba_uav.sh` 給的 agent_id、cruise_speed、altitude，和 `EXEC_ARGS`）＞ yaml ＞ 程式預設。
所以 yaml 裡的 `altitude`、`cruise_speed` 經過 `cbba_uav.sh` 時**不會生效**，要用 `CBBA_ALTITUDE`、`CBBA_SPEED`。
改完看節點啟動時印的那行（`uavN：樹 …，速度 … m/s …`），或 `ros2 param get /uavN_task_manager cbf.safe_radius`。

## 二、參數在哪裡作用

```
起飛（樹外 UavFlight）     → 爬到 altitude ± takeoff_tolerance → 起飛完成，開始 tick 樹
NavigateTo / AirRecon / 待命 / 回起飛點
   │ 每 tick_period_ms：UavContext::moveTo(目標)
   ├─ CBF：名目速度（cruise_speed、k_att、brake_*）＋避碰約束（safe_radius、gamma、yield_radius…）→ 想要的速度
   ├─ 運動平滑層：加速度上限（motion.*）→ 參考位置＋速度
   └─ PX4 offboard：TrajectorySetpoint（位置＋速度前饋）→ PX4 的位置控制器（PX4 參數 MPC_*）
到達：離目標 < reach_tolerance；逾時：距離 / 速度 × 2 + fly_timeout_margin
```

## 三、節點（只有 `uav_task_manager_node`）

| 參數 | 預設 | 作用 | 調整 |
|---|---|---|---|
| `tick_period_ms` | 50 | BT tick 週期 | 換任務的中斷是 callback 直接 tick，不受它影響；調小只讓動作更新得更密 |
| `min_battery` | 20 % | 電量 ≤ 這個值就返航（PX4 RTL），手上的任務回報 `BATTERY_RTL`；觸發後不恢復 | 要和 cbba_node 的 `safety_reserve`（20 %）一致 |
| `return_home_delay` | 2 s | 做完任務、待命這麼久後飛回起飛點上空 | 負值 = 原地懸停；新任務來時中途就走 |
| `fire_task.monitor_timeout` | 120 s | 看到火警後懸停監控，等地面任務結束（`task_closed`）最多多久 | 0 = 不等（停留時間到就走）；沒有狗時會等滿這麼久，排在後面的任務在 `overrun_margin` 後交給別台 |
| `mission_timeout` | 0（= 1800 s） | 一趟（佇列跑完）最長多久，超過就中止 | 通常不用改 |
| `emergency_topic` | `/<ns>/emergency_stop` | 急停話題（std_msgs/Bool）；急停 = 樹 halt、原地懸停 | |
| `log_transitions` | false | 印每個 BT 節點的狀態變化 | 除錯時開 |

## 四、高度、速度、到達

| 參數 | 預設 | 作用 | 調整 |
|---|---|---|---|
| `altitude` | 5 m | 巡檢高度（起飛高度、NavigateTo 的高度） | 用 `CBBA_ALTITUDE`；`px4_state_bridge` 的 `auto_min_altitude` 由 `cbba_uav.sh` 設成 altitude − 0.5 m |
| `cruise_speed` | 5 m/s | 最大水平速度（CBF 的上限） | 用 `CBBA_SPEED`（cbba_node 的出價速度一起改）；itest demo 用 3 |
| `max_vertical_speed` | 2 m/s | 最大垂直速度 | 起飛太慢調大，但垂直衝過頭要配合 `cbf.brake_vertical_accel` |
| `takeoff_tolerance` | 0.3 m | 爬到 altitude ± 這個值才算起飛完成 | 太小：PX4 高度晃就一直不算完成 |
| `reach_tolerance` | 0.5 m | 離任務點這麼近算到達（開始懸停、認 tag） | 要 > `motion.hold_radius`；調小到得比較準，但最後一段比較慢 |
| `fly_timeout_margin` | 30 s | 飛行逾時 = 距離 / cruise_speed × 2 + 這個 | 避碰繞路多、速度設很低時調大；逾時回報 `NAV_FAILED(TIMEOUT)`（TRANSIENT，交回 CBBA） |
| `fly_timeout_abort` | yaml false（程式預設 true） | 逾時要不要中止 | false：只印「飛太久」警告（之後每 10 s 一次）、繼續飛，調 CBF 時用；true：回報失敗、交回 CBBA |

## 五、CBF（避碰）與運動平滑層

2026-10-14 由 CPF（人工位勢場）換成 CBF（控制障礙函數）；CPF 的原始碼與當時的 yaml 備份在 `docs/archive/cpf_20261014/`。

### 名目速度（往目標飛）

u_nom 大小 = min(cruise_speed, k_att × 距離, √(2 × brake_accel × 距離))；垂直分量再限制在 √(2 × brake_vertical_accel × 高度差)。

| 參數 | 預設 | 作用 | 調大 | 調小 |
|---|---|---|---|---|
| `cbf.k_att` | 1.0 1/s | 最後幾公尺的收斂 | 收得快，可能衝過頭 | 最後一段慢 |
| `cbf.brake_accel` | 0.2 m/s² | 水平剎車曲線 | 晚剎車，容易衝過頭 | 早剎車、比較穩，飛得比較久 |
| `cbf.brake_vertical_accel` | 0.1 m/s² | 垂直剎車曲線 | 爬升快，容易衝過頭 | 爬升、換高度比較慢；巡檢高度 25 m 以上時爬升可能超過 20 s，會出現「爬升 20 s 還沒到」的警告 |

`brake_accel` 要 ≤ `motion.max_accel`，`brake_vertical_accel` 要 ≤ `motion.max_vertical_accel`。

### 避碰（只看水平）

對 `influence_radius` 內的每台鄰機 j（p_j 用廣播的速度外推 age 秒，最多 `max_extrapolation`）：

```
h_j = |p − p_j|² − r²                    r = safe_radius（讓路時 yield_radius）
約束  a_j·u ≥ b_j
      a_j = 2 (p − p_j)
      b_j = λ a_j·v_j − ρ γ h_j
```

| 情況 | λ | ρ | r |
|---|---|---|---|
| 同樣狀態（都執行中或都待命） | 0 | 0.5 | safe_radius |
| 自己待命、鄰機執行中（讓路） | 1 | 1 | yield_radius |
| 自己執行中、鄰機待命 | 0 | 0.5 | safe_radius |

1. 往右繞：名目速度違反的約束，名目速度加 `k_tan` × 違反量（(b − a·u_nom)/|a|）的「面向鄰機的右手邊」分量。
2. 求 u = argmin |u − u_nom'|²，使所有約束成立（Dykstra 交替投影）。
3. 垂直用名目值、限制在 `max_vertical_speed`；總和限制在 `cruise_speed`。

**推導量**（ρ = 0.5、對單一鄰機）：

| 量 | 公式 |
|---|---|
| 距離 d 時允許「往鄰機靠近」的速度 | c(d) = ρ γ (d² − r²) / (2d) |
| γ 的上限（剎得住） | γ ≤ 2 × `motion.max_accel` / (ρ × cruise_speed) |
| influence_radius 的下限（一進來不急剎） | R ≥ (V + √(V² + (ρ γ r)²)) / (ρ γ) |
| 讓路時退到的距離（鄰機以 v 飛過來） | d² − 2 d v / γ − yield_radius² = 0 的正根 |
| 懸停的會不會被推 | u = 0 在 h ≥ 0 時滿足，不會（只有 h < 0 或讓路時才動） |

| 參數 | 預設（程式／yaml） | 作用 | 調大 | 調小 |
|---|---|---|---|---|
| `cbf.safe_radius` | 1.2／1.0 m | 水平最小距離 | 保守，但**要 < 出生點間隔（2 m）** | 允許更近；x500 約 0.7 m，不建議 < 1.0 |
| `cbf.gamma` | 1.0 1/s | 靠近速度隨距離放寬的速率 | 靠得快、晚剎，PX4 落後時可能進 safe_radius | 早減速、繞得早 |
| `cbf.influence_radius` | 21／3 m | 只對這距離內的鄰機加約束 | 多算一些、不影響結果（約束本來就鬆） | 太小時一進範圍就急剎 |
| `cbf.yield_radius` | 2.5／2.0 m | 待命讓路給執行中的鄰機時保持的距離 | 讓得多 | 要 > safe_radius，否則執行中的那台到不了停在待命機旁的任務點 |
| `cbf.k_tan` | 0.8 | 往右繞的比例 | 繞得大 | 0 = 不繞，正對面會停住 |
| `cbf.max_extrapolation` | 0.5 s | 鄰機位置最多外推多久 | | |
| `cbf.neighbor_timeout` | 1 s | 鄰機的廣播超過這麼久沒更新就不管 | 掉包時還記得，但位置舊 | 掉包時當作沒有鄰機 |

限制：速度命令還要經過運動平滑層與 PX4，實際速度會落後，最近距離可能比 safe_radius 小一點（單元測試要求 > 0.9 × safe_radius）。
不跑這套的鄰機（PX4 自己 RTL、手動）也被當成會分攤一半。

### 運動平滑層（給 PX4 的 setpoint）

| 參數 | 預設 | 作用 | 調整 |
|---|---|---|---|
| `motion.max_accel` | 1.5 m/s² | 參考速度的水平變化上限（0 → 3 m/s 約 2 s） | 大：起步、閃避快，機身傾斜大；小：平順但閃避慢 |
| `motion.max_vertical_accel` | 1.0 m/s² | 垂直變化上限 | 要 ≥ `cbf.brake_vertical_accel` |
| `motion.leash` | 1.5 m | 參考位置最多領先實際位置多遠（PX4 追不上時不會越拉越遠） | 大：PX4 追得猛；小：平順但慢 |
| `motion.hold_radius` | 0.2 m | 離目標這麼近、沒有鄰機要閃時，直接把目標交給 PX4 位置保持 | 要 < `reach_tolerance` |
| `motion.reset_gap` | 0.5 s | 超過這麼久沒下命令，從目前位置重新開始（換動作時不會跳） | 通常不用改 |

## 六、火警

### 參數（yaml）

| 參數 | 預設 | 作用 |
|---|---|---|
| `fire_task.value` | 100 | 建立的地面處置任務的價值（介面規格的火警預設） |
| `fire_task.duration` | 20 s | 地面任務的處置時間（狗在現場停留） |
| `fire_task.deadline` | 120 s | 地面任務的期限 |
| `fire_task.dedup_radius` | 3 m | 離已回報的火點這麼近視為同一個，不再建立；也用來判斷「火點就是目的地」（途中不報、到了由 AirRecon 判斷） |

只有備用 BT 用的：`fire_task.divert_on_sight`（路過發現時放下任務去監控，預設 false）、`fire_task.monitor_value`、`fire_task.monitor_sec`。

### 樹的屬性（`trees/uav_mission.xml`，改完重開節點）

| 節點 | 屬性 | 預設 | 作用 |
|---|---|---|---|
| `NavigateTo` | `watch_fire_tag` | 101 | 飛行途中看哪個 tag（0 = 不看） |
| | `divert` | true | 路過發現火點時轉向火點上空懸停監控，處理完（或 `monitor_timeout`）再回航線；false = 只發布地面任務、繼續飛 |
| | `monitor_timeout` | `{fire_monitor_timeout}` | 轉向後監控最多幾秒（從 yaml 的 `fire_task.monitor_timeout` 來） |
| | `min_confidence`、`min_frames`、`max_age` | 0.85、5、0.5 s | 偵測結果要多可信、多新才算 |
| `AirRecon` | `fire_tag` | 101 | 到點懸停時認哪個 tag |
| | `settle_sec` | 1 s | 到點後先穩定這麼久才判斷（減速、傾斜時的影像位置不準） |
| | `view_radius` | 6 m | 只認離懸停點這麼近的火點（下視相機 5 m 高約看 12 m × 8 m）；要 > `fire_task.dedup_radius` |
| | `min_confidence`、`min_frames`、`max_age` | 0.85、5、0.5 s | 同上 |
| | `monitor_timeout` | `{fire_monitor_timeout}` | 從 yaml 的 `fire_task.monitor_timeout` 來 |

偵測本身的參數在 `apriltag_fire_detector`（`min_margin` 30、`window_sec` 1 s、`tag_size` 0.8 m）。

## 七、其他地方、要一致的參數

| 參數 | 在哪裡 | 要和誰一致 |
|---|---|---|
| `cruise_speed` | cbba_node（出價估時間）、BT | 兩邊相同：`CBBA_SPEED` |
| `recon_altitude` | cbba_node（AIR_RECON 的 z 太低時抬高） | BT 的 `altitude`：`CBBA_ALTITUDE` |
| `auto_min_altitude` | `px4_state_bridge`（爬到這個高度才 mode AUTO、才出價） | `cbba_uav.sh` 給 altitude − 0.5 m |
| `overrun_margin` | cbba_node（執行中超過預估多久就不排隊接新任務，10 s） | 懸停監控時，排在後面的任務在這之後交給別台 |
| `safety_reserve`、能量模型 | cbba_node（`cbba_uav.sh` 依 `SIM_BAT_DRAIN` 算） | BT 的 `min_battery` |
| `UAV_SPAWN_SPACING` | `docker/.env`（出生點間隔 2 m） | 要 > `cbf.safe_radius` |
| PX4 的 `MPC_XY_CRUISE`、`MPC_ACC_HOR_MAX`、`MPC_Z_VEL_MAX_UP`、`MPC_XY_P`、`MPC_Z_P` | PX4（offboard 位置控制器） | BT 給的速度、加速度不要超過 PX4 的上限；PX4 追不上 setpoint 時才需要動這些 |

## 八、症狀 → 先調哪個

| 症狀 | 先看 | 調整 |
|---|---|---|
| 起飛衝過頭、換高度上下擺 | `cbf.brake_vertical_accel`、`motion.max_vertical_accel` | 剎車調小（例如 0.2 → 0.1，2026-10-17 改成 0.1） |
| 到點衝過頭再拉回 | `cbf.brake_accel`、`cbf.k_att` | 調小 |
| 懸停時幾台互推、晃 | `cbf.safe_radius`、`cbf.yield_radius` | safe_radius、yield_radius 要 < 出生間隔（待命的被執行中的推開） |
| 交會時太近（log「避碰：這次最接近鄰機 x m」） | `cbf.gamma`、`cbf.safe_radius` | gamma 調小、safe_radius 調大 |
| 交會前一下子急剎 | `cbf.influence_radius` | 調大（看上面的下限公式） |
| 對飛時卡在中間 | `cbf.k_tan` | 調大 |
| 閃避時機身傾斜大、看起來很急 | `motion.max_accel`、`motion.leash` | 調小 |
| 起飛一直不算完成（沒有「到達 5.0 m」；爬升 20 s 後會印「警告：爬升 20 s 還沒到 …」） | `takeoff_tolerance` | 調大 |
| 「警告：在空中但不是 offboard」／「警告：切回 offboard，高度 … 差 …」 | 不是參數（被遙控器、地面站接管） | 只是提示，行為不變；接管期間 robot_state 仍是 AUTO（待和機器狗端定） |
| 常常回報 `NAV_FAILED(TIMEOUT)` | `fly_timeout_margin`、`cruise_speed` | margin 調大 |
| 火點判定太慢或誤判 | 樹的 `settle_sec`、`min_confidence`、`min_frames` | 慢：settle 調小；誤判：門檻調高 |
| 看到旁邊別的點的火 | 樹的 `view_radius` | 調小 |
| 監控火點等太久（沒有狗） | `fire_task.monitor_timeout` | 調小，或 0 = 不等 |

判斷是 BT 的命令在擺、還是 PX4 追不上：比對 PX4 ulog 的 `trajectory_setpoint` 和 `vehicle_local_position`
（sim 容器 `/opt/PX4-Autopilot/build/px4_sitl_default/rootfs/<N>/log/`，用 pyulog）。setpoint 自己在擺 → 調這份的參數；
setpoint 穩、實際位置跟不上 → 調 PX4 的 `MPC_*`。

## 九、寫死、不是參數的

- CBF 的公式（只看水平、往右繞、λ／ρ 的分攤）、運動平滑層的「參考位置不越過目標」：`src/cbf.cpp`、`src/motion.cpp`。
- 航向固定在第一次收到位置時的方向（下視相機的影像方向不會一直轉）：`src/px4_platform.cpp`。
- 外部任務的幾何（目標正上方、`air_recon` → `report_result`）：`skeleton/external_task.hpp`。
- 高度與接管警告（爬升 `kClimbWarnSec` 20 s、高度差門檻沿用 `takeoff_tolerance`）：`skeleton/uav_flight.hpp`。
