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
| `return_home_delay` | 1 s（2026-10-20 從 2 s） | 做完任務、待命這麼久後飛回起飛點上空 | 負值 = 原地懸停；新任務來時中途就走 |
| `fire_task.idle_watch_tag` | 101 | 待命（含做完任務飛回起飛點）時也看這個火警 tag：看到新火點 → 發布地面任務 → 到火點上空監控到地面任務結束或 `monitor_timeout` → 回到待命（2026-10-19）；監控中 CBBA 派了任務就去做任務。0 = 待命時不看（只有執行任務時看） | 正式 BT（`uav_task_manager_node`）才有 |
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
| `reach_tolerance` | 0.25 m（2026-10-19 從 0.5 調小：0.5 m 就算到達時，懸停的前 1.2～1.5 s 還在靠近；`docker/itest/hover_error.py` 量） | 離任務點這麼近算到達（開始懸停、認 tag） | 要 > `motion.hold_radius`；調小到得比較準，但最後一段比較慢 |
| `fly_timeout_margin` | 30 s | 飛行逾時 = 距離 / cruise_speed × 2 + 這個 | 避碰繞路多、速度設很低時調大；逾時回報 `NAV_FAILED(TIMEOUT)`（TRANSIENT，交回 CBBA） |
| `fly_timeout_abort` | yaml false（程式預設 true） | 逾時要不要中止 | false：只印「飛太久」警告（之後每 10 s 一次）、繼續飛，調 CBF 時用；true：回報失敗、交回 CBBA |

## 五、CBF（避碰）與運動平滑層

2026-10-14 由 CPF（人工位勢場）換成 CBF（控制障礙函數）；CPF 的原始碼與當時的 yaml 備份在 `docs/archive/cpf_20261014/`。

### 名目速度（往目標飛）

u_nom 大小 = min(cruise_speed, k_att × 距離, √(2 × brake_accel × 距離))；垂直分量再限制在 √(2 × brake_vertical_accel × 高度差)。

| 參數 | 預設 | 作用 | 調大 | 調小 |
|---|---|---|---|---|
| `cbf.k_att` | 1.0 1/s | 最後幾公尺的收斂 | 收得快，可能衝過頭 | 最後一段慢 |
| `cbf.brake_accel` | 0.5 m/s²（2026-10-20 從 0.2：最後 6 m 要約 8 s、看起來拖；0.15、0.2 實測都沒有衝過頭，0.5 要用 `hover_error.py` 確認） | 水平剎車曲線 | 晚剎車，容易衝過頭 | 早剎車、比較穩，飛得比較久 |
| `cbf.brake_vertical_accel` | 0.1 m/s² | 垂直剎車曲線 | 爬升快，容易衝過頭 | 爬升、換高度比較慢；巡檢高度 25 m 以上時爬升可能超過 20 s，會出現「爬升 20 s 還沒到」的警告 |

`brake_accel` 要 ≤ `motion.max_accel`，`brake_vertical_accel` 要 ≤ `motion.max_vertical_accel`。

### 避碰（只看水平；2026-10-24 定案，`include/uav_px4_bt/avoidance.hpp`）

目標：路線交叉時不碰撞、不互相卡住（不要「停下來等人過」）。每個控制週期、每台鄰機 j（p_j 用廣播的速度外推 age 秒，
最多 `max_extrapolation`）依序做：

**1. 預測會不會碰（ConflictPredictor）**：自己用名目速度 u_nom（意圖，不是量到的速度——剛起步時量到的是 0）、鄰機用廣播的速度：
```
r = p − p_j          v_r = u_nom − v_j
T  = min(ttc_horizon, 自己到目標的水平距離 / |u_nom|)     自己幾乎不動（|u_nom| < 0.05 m/s）時 T = ttc_horizon
t* = clamp(−r·v_r / |v_r|², 0, T)       d* = |r + v_r t*|
衝突 = 正在靠近（−r·v_r > 0、|v_r| ≥ 1 cm/s）而且 d* < 要求的距離
要求的距離 = miss_clearance；自己沒在動時 ≤ safe_radius + 0.5 m；
             最近點就是自己到目標的那一刻（目標就在鄰機旁）時只要 safe_radius；
             現在已經比要求的近時，不要再更近就好（往外、維持距離都可以）
```
沒有衝突：不比路權、不繞、不加 CBF 約束（距離 < `guard_radius` 時例外），照原路飛。
「到目標那一刻只要 safe_radius」是因為目標本來就在鄰機旁時（起飛點只隔 2 m）做不到 miss_clearance，原本會一直繞（10/24 SITL 相撞）；
最近點在到目標之前（對方從路上經過）照常要 miss_clearance——否則待命的讓開後，名目速度又把它拉回對方的航線上。

**2. 誰讓（RightOfWayPolicy）**：路權鍵 K = (任務中懸停, 執行中, −機號)，大的直行、小的讓。
- 任務中懸停：BT 正在任務點停留、辨識 AprilTag、監控火點（`UavContext::moveTo`：執行中而且已經到 goal，水平 < max(2 × `reach_tolerance`, 0.5 m)），
  用位置廣播 flags 的 bit 3 送出去；**不是看速度猜**（剛起步、減速、短暫停頓都會是 0）。
- 執行中：ExecMode 是 NAVIGATING／EXECUTING（位置廣播的 busy）。
- 兩台用同一份位置廣播算，結果一定唯一（不會兩台都不讓）。

**3. 鎖定角色（EncounterState）**：衝突開始時記下誰讓、往哪邊繞；交會途中任務完成、busy 改變、速度變 0 都不換。
解除：已經通過（不再靠近）而且距離連續 3 個週期變大；或預測不衝突而且距離 > `yield_exit`；或沒有位置廣播。

**4. 讓的那台怎麼繞（AvoidancePlanner，VO）**：找離 u_nom 最近、對所有鄰機都安全的速度（讓的對象：預測最近距離 ≥ 要求的距離；
其他鄰機 ≥ safe_radius）：
- 先保持速度只轉方向（每 3°，往左右各到 180°）：左右都行時選偏轉小的，一樣時往自己的右邊；選定後鎖定那一側。
- 保持速度找不到才降速（75、50、25 %，log「保持速度繞不開，降到 …」）；再找不到取餘裕最大的，交給 CBF 和緊急處理。
- 自己沒在動（懸停的讓路，例如待命的停在執行中那台的航線上）：留在原地就安全就不動，否則往旁邊移開，
  速度只要「剛好來得及讓開」= 1.5 ×（要求的距離 − 預測最近距離）/ 距離交會的時間，最低 0.2 m/s，找不到安全方向才加快到 cruise_speed
  （2026-10-24：原本用 max(0.3 × cruise_speed, 對方的速度)，3 m/s 時起降板上只要讓 1 m 的那台衝出去 4.6 m 再繞回來）。
  讓行**不是**設成零速度。
- 在動的讓路、保持速度找不到時，先用讓路速度 max(0.3 × cruise_speed, 對方的速度) 試，再降速。
- 直行的那台照 u_nom。

**5. CBF 保底**：只對需要的鄰機加約束（2026-10-24 起預設**二階**，`cbf.second_order: true`）

| 鄰機 | 什麼時候加 | ρ |
|---|---|---|
| 自己讓的對象 | VO 找不到安全速度、繞完預測還不夠遠、距離 < `guard_radius`，或（二階）剎車距離的餘裕 < 0.5 m | 1（全額負責） |
| 直行時的對象、沒有衝突的鄰機 | 距離 < `guard_radius`，或（二階）剎車距離的餘裕 < 0.5 m | 0.5 |

二階（`cbf2Filter`；模型 ṗ = v、v̇ = a、|a| ≤ `motion.max_accel`）：
```
d = |p − p_j|、n = (p − p_j)/d、ḋ = n·(v − v_j)（< 0 在靠近；v 用實際量到的速度）、A = max_accel / ρ（相對減速度）
r = safe_radius ＋ response_delay × 靠近速度          反應延遲期間還會往前飛
h = √(2 A (d − r)) + ḋ                              照 A 全力剎，靠近速度在剩下的距離內剎得住才 ≥ 0
約束 ḣ ≥ −γ h、鄰機的加速度當 0：  n·a ≥ ρ [ −(|Δv|² − ḋ²)/d − A ḋ / √(2A(d − r)) − γ h ]
剎車距離的餘裕 = d − safe_radius − response_delay × 靠近速度 − 靠近速度² / (2A)
```
ρ（二階）：讓的那台 1；其他 0.5，但對方往自己的速度 < 0.2 m/s（停著或正在離開）時也是 1——它不會替你剎那一半
（2026-10-24 單元測試：直行的那台以為對方會剎一半，3 m/s 時剎太晚，靠到 0.57 m）。
VO 的速度先換成加速度 a_ref = (u_ref − v_ref)/dt（v_ref = 平滑層上一個週期的參考速度，限制在 max_accel），投影到所有約束和
|a| ≤ max_accel 的交集（Dykstra），再積分回速度 v_ref + a·dt 交給平滑層——平滑層的加速度上限就是 max_accel，不會再改掉 CBF 的結果
（一階時平滑層會把 CBF 要的急轉、急剎拖慢，3 m/s 時 CBF 的保證被延遲吃掉）。速度越快，h 越早變小、越早開始剎，
所以 3 m/s 時不用把 guard_radius 加大到 4 m（一階時那樣做，起降板上三台互相減速）。

一階（`cbfFilter`，`cbf.second_order: false`，對照用）：h = |p − p_j|² − r²、a·u ≥ λ a·v_j − ρ γ h（a = 2(p − p_j)；讓的 λ = 1、其他 λ = 0），
u = argmin |u − u_ref|²。

**6. 緊急**：距離 < `safe_radius`，或投影後還有約束不成立 → 不用投影的結果。二階：最大加速度、在一圈方向（每 7.5°）裡
挑對**所有**鄰機的約束違反最少的（`cbf2Escape`；原本只往遠離最近那台的方向，最近的是停著的鄰機時被推向高速飛來的另一台）；
一階：往外退，速度 = max(`emergency_speed`, 最近那台往自己靠近的速度)，最多 `cruise_speed`。log「警告：避碰緊急退讓 …」。

log（BT）：「避碰：預測 t* s 後和 uavN 最近 d* m（現在 x m）→ 我讓行（VO 繞）｜對方讓行，我直行」、
「避碰：和 uavN 解除（我讓行、往右繞，現在 x m）」、「避碰：這次最接近鄰機 x m」。

| 參數 | 預設（yaml） | 作用 | 調大 | 調小 |
|---|---|---|---|---|
| `cbf.safe_radius` | 1.0 m（程式預設也是 1.0，全專案統一） | 水平最小距離 | 保守，但**要 < 出生點間隔（2 m）** | x500 約 0.7 m，不建議 < 1.0 |
| `cbf.ttc_horizon` | 5 s | 預測最近點只看這麼多秒 | 更早發現、更早繞 | 晚一點才繞 |
| `cbf.miss_clearance` | 1.5 m | 預測最近距離小於這個才算衝突；讓的那台繞到這麼遠 | 繞得早、繞得大 | **要 > guard_radius**，否則繞完 CBF 還是會砍速度 |
| `cbf.guard_radius` | 1.3 m | 小於這個距離時 CBF 一定加約束 | 保守（速度快時要大：剎車距離） | 要 > safe_radius |
| `cbf.yield_exit` | 2.0 m | 交會解除的距離 | 角色鎖得久 | 要 > miss_clearance |
| `cbf.second_order` | true | 二階 CBF（管加速度、考慮剎車距離）；false = 一階（對照） | | |
| `cbf.gamma` | 1.0 1/s | CBF 約束裡 h 下降的速率 | 晚剎，PX4 落後時可能進 safe_radius | 早減速 |
| `cbf.emergency_speed` | 0.3 m/s | 一階的緊急退讓速度（二階用最大減速度） | | |
| `cbf.response_delay` | 0.3 s | 二階：反應延遲；安全距離加上 延遲 × 靠近速度 | 速度快時更早剎 | PX4 追得很緊時可以小一點 |
| `cbf.max_extrapolation` | 0.5 s | 鄰機位置最多外推多久 | | |
| `cbf.neighbor_timeout` | 1 s | 鄰機的廣播超過這麼久沒更新就不管 | 掉包時還記得，但位置舊 | 掉包時當作沒有鄰機 |
| `cbf.influence_radius`、`cbf.yield_radius`、`cbf.k_tan` | 3 m、2 m、0.8 | 只有舊的 `cbfVelocity`（單元測試對照）用；`yield_radius` 另外是 skeleton 待命讓路的範圍（2 ×） | | |

**換速度時**（itest `--speed` 不是 0.5 時自動給，`itest.py` 的 `cbf_speed_args`）：二階時只改 `miss_clearance` = safe_radius + 0.5 + 0.2 V
（3 m/s → 2.1 m）、`yield_exit` = miss + 0.5 m；剎車距離由 CBF 自己處理。（一階時要 γ 1.5、guard 4.0 m、miss 4.5 m。）

限制：PX4 追 setpoint 還有延遲（沒有算進 CBF）；鄰機的加速度當 0（對方急轉時保守度不夠，靠 safe_radius 的餘量）。
不跑這套的鄰機（PX4 自己 RTL、手動）沒有位置廣播的 busy，當成待命。

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
| `auto_min_altitude` | `px4_state_bridge`，**只在備用 BT 時用**（爬到這個高度才 mode AUTO）；正式 BT 時 mode 照 BT（起飛完成 = altitude ± `takeoff_tolerance`） | `cbba_uav.sh` 給 altitude − 0.5 m |
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
| 交會時還是減速、停一下才過（log 沒有「避碰：預測 …」） | `cbf.ttc_horizon`、`cbf.miss_clearance` | 調大（更早發現、繞得開一點） |
| 沒有要撞也在繞（log 一直出現「避碰：預測 …」） | `cbf.miss_clearance` | 調小（要 > guard_radius） |
| 閃避時機身傾斜大、看起來很急 | `motion.max_accel`、`motion.leash` | 調小 |
| 起飛一直不算完成（沒有「到達 5.0 m」；爬升 20 s 後會印「警告：爬升 20 s 還沒到 …」） | `takeoff_tolerance` | 調大 |
| 「警告：被接管」／「警告：PX4 失效保護」／「警告：切回 offboard，高度 … 差 …」 | 不是參數 | 被遙控器、地面站切了模式，或 PX4 自己的失效保護：任務交還、mode MANUAL；切回 offboard 從目前位置接回 |
| 一直不接任務（cbba_node 沒「開始參與出價」） | 看 bridge 的 log「BT 的 mode」 | 正式 BT 起飛完成（altitude ± `takeoff_tolerance`）才送 AUTO；BT 沒開時 bridge 1.5 s 後送 MANUAL |
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
- 待命時停下讓路的範圍：執行中的鄰機在 `kYieldStopFactor`（2）× `cbf.yield_radius` 內（yaml 2 m → 4 m）才停在被推開的位置；
  更遠的照常飛（例如飛回起飛點），由 CBF 繞開（2026-10-19，原本用 `cbf.influence_radius`，demo 設 12 m 時離 10 m 就停）：`skeleton/uav_flight.hpp`。
- 高度與接管警告（爬升 `kClimbWarnSec` 20 s、高度差門檻沿用 `takeoff_tolerance`）、被接管的判斷（PX4 `failsafe`、
  `failsafe_and_user_took_over`）：`skeleton/uav_flight.hpp`。
- mode（2026-10-17）：正式 BT 的參數 `bt_mode_topic`（預設 `/<ns>/bt_mode`）、`mode_period`（0.5 s）；bridge 的參數
  `bt_mode_topic`（`cbba_uav.sh` 在 `EXECUTOR=task_manager` 時給）、`bt_mode_timeout`（1.5 s）。
