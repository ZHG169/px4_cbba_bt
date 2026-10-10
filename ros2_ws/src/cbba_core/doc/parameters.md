# CBBA 參數說明

cbba_core · 2026-10-07，2026-10-09 改成協定版本 2（見 [`protocol.md`](protocol.md)）

所有和 CBBA 有關的參數：意義、預設值、為什麼是這個值、改了會怎樣。
出價公式各參數的推導與實驗數據，見 [`cbba_parameters.md`](../cbba_parameters.md)。

參數分成三種，改的方式不一樣：

| 種類 | 在哪裡 | 怎麼改 |
|---|---|---|
| **ROS 參數** | `cbba_node` 的 `declare_parameter` | `ros2 run cbba_core cbba_node --ros-args -p 名稱:=值`，或 `cbba_uav.sh -p 名稱:=值` |
| **程式常數** | `CommConfig`（`cbba_comm.hpp`）、`ScoringParams`（`scoring.hpp`）、`wire_header.hpp` | 改程式碼、重新編譯 |
| **SITL 環境變數** | `docker/scripts/cbba_uav.sh`、`px4_sitl.sh` | `變數=值 cbba_uav.sh`；或寫進 `docker/.env` 後 `./start.sh` |

---

## 先講結論：最常要調的

| 參數 | 預設 | 什麼時候要改 |
|---|---|---|
| `energy_per_meter`、`hover_energy_per_sec` | 0.5、0.2（SITL：0.022、0.111） | **一定要校正**。換機型、換電池、上實機都要重量 |
| `cost_ref` | 50 m | 換場地時，取場地半寬左右 |
| `battery_weight` | 1.0 | 只在 0～2 之間調；超過 2 沒有額外效果 |
| `lost_timeout` | 1.5 s | 網路很差、常誤判失聯時加大（全隊要一樣） |
| `assign_hold` | 0.6 s | BT 被來回切換時加大；接手太慢時減小 |

---

## 一、cbba_node（ROS 參數）

### 1. 身分與網路

| 參數 | 預設 | 說明 |
|---|---|---|
| `agent_id` | 必填 | 機號 1～254，全隊不能重複。協定版本 2 以 task_id 識別任務，任何機號都能建立任務（版本 1 只有 1～8 能）。協定本身是 1～65534，節點限制在 254 是因為 `RobotState.agent_id` 是 uint8。狗用 50 |
| `vehicle_type` | `uav` | `uav` 或 `ugv`：AGENT_STATE 的載具種類（無人機只接 AIR_RECON，狗只接 GROUND_INTERVENTION、PATROL），以及參與出價的條件、能量模型、`recon_altitude`、`ns` 的預設值 |
| `ns` | `uav<agent_id>`；ugv 是 `v60` | 話題前綴：`/<ns>/robot_state`、`/<ns>/new_task`、`/<ns>/task_result`、`/<ns>/assigned_task` |
| `mesh_ip` | 環境變數 `MESH_IP` | UDP 只走這張網卡（mesh）。空字串由系統決定，可能會走到 PX4 的線材網路 |
| `udp_group`、`udp_port` | `239.255.42.99`、`14600` | multicast 群組。**全隊（含機器狗）要一樣** |

節點名稱：程式裡固定叫 `cbba_node`，啟動時用 `-r __node:=uav1_cbba_node`（狗 `v60_cbba_node`）加上載具名稱，
`ros2 node list`、`ros2 param get` 才分得出是哪一台。`cbba_uav.sh` 會自動加。

### 2. 參與出價

| 參數 | 預設 | 說明 |
|---|---|---|
| `participate` | `state` | `state`＝`robot_state` 在 `state_timeout` 內有更新才參與；無人機另外要 `flight_state_valid`、`armed`，而且 `require_airborne` 時要沒有 `landed`。`always`＝一律參與、不訂閱 `robot_state`，位置和電量用 `initial_position`、`battery`（只用來測協商）。不參與時 CBBA_STATE 沒有 s、沒有紀錄（36 B），手上的任務立刻釋放 |
| `require_airborne` | true | 無人機要離地才參與出價；false = 解鎖就參與（原本轉接節點的 `participate:=armed`） |
| `state_timeout` | 1.5 s | `robot_state` 這麼久沒更新就停止參與出價。無人機的轉接節點在 PX4 的位置有在更新時就一直送（地面上也送），PX4 斷線時停送 |
| `initial_position`、`battery` | `[0, 0, 0]`、100 | 只在 `participate:=always` 時用 |
| `recon_altitude` | uav 5.0 m、ugv 0 | 巡檢任務（AIR_RECON）的高度。BT 常給地面座標（z = 0），z 比它低時改成它；不改的話，每段距離都多算到地面的高度差，耗電也估錯。0 = 不調整。**要和執行者的飛行高度一致** |

返航點是第一筆 `robot_state` 的位置，只用在出價時估返航的耗電（不會叫載具飛回去）。轉接節點在地面上就開始送，
所以無人機的返航點是地面的出生點。

AGENT_STATE 的 armed、offboard、landed 是 `robot_state` 的真實值，只在 `flight_state_valid` 時送（否則都是 0）。

**無人機的轉接節點 `px4_state_bridge`**（`px4_waypoint_node`）的參數：

| 參數 | 預設 | 說明 |
|---|---|---|
| `agent_id`、`px4_ns` | 環境變數 `UAV_ID`、`UAV_NS` | 要和 cbba_node 的 `agent_id`、`ns` 一樣 |
| `position_timeout` | 0.5 s | PX4 的位置這麼久沒更新就停止送（PX4 斷線）；有在更新就一直送，地面上、未解鎖也送 |
| `status_timeout` | 2.0 s | `vehicle_status` 這麼久沒更新，`flight_state_valid` 就是 false（armed 等保留最後的值但不算數）。還要收到過 `vehicle_land_detected` |
| `spawn_enu` | `[0, (機號−1)×UAV_SPAWN_SPACING, 0]` | 出生點（map ENU）。PX4 的 local position 以出生點為原點，加上它才是 map 座標。**要和 `px4_sitl.sh`、`task_executor` 一致** |

### 3. 交給 BT（正式指派、接受、搶占、排隊）

| 參數 | 預設 | 說明 |
|---|---|---|
| `assign_hold` | 0.6 s | 同一個任務要連續排在路徑第一位這麼久，才請 BT 接受（`require_accept = false` 時直接發到 `assigned_task`）。協商途中的 10 ms 級來回換手不會傳給 BT。**分段量測（2026-10-09）顯示它是火警插單延遲的大部分（建立 → BT 執行約 0.8 s），目前不能宣稱符合 300 ms**；要調低時要一起驗證會不會造成指派震盪 |
| `require_accept` | uav false、ugv true | 要 BT 接受才執行。false 只跳過握手（請求當下就算接受），可行性、版本、回報檢查照常。無人機的 BT 完成後改成 true |
| `accept_timeout` | 3.0 s | BT 多久沒回覆就當成暫時拒絕、送 RELEASE；之後才到的接受回覆不會啟動任務。這是故障處理的上限，不是正常的反應時間 |
| `exec_timeout` | 1.5 s | ExecState 多久沒更新就不能接新任務（`require_accept = true` 時）；用本機收到訊息的單調時間判斷 |
| `max_queued_fires` | 1 | 執行中任務之後最多保留幾個火警 |
| `max_queue_wait` | 120 s | 沒有期限的任務，排隊時最多等多久（從任務建立時算，不會每輪競標重新起算）。有期限時用期限 |
| `tick_ms` | 10 ms | 主迴圈週期：收 UDP、跑協定、送封包。改大會讓所有反應變慢 |
| `lock_dir` | `CBBA_LOCK_DIR` → `$XDG_RUNTIME_DIR/cbba` → `/tmp/cbba` | 程序鎖 `agentN.lock` 的目錄（同一機號只能跑一個 cbba_node）。docker 的所有容器共用 `/run/cbba`（主機 `/tmp/cbba_locks`） |

### 4. 能量模型

| 參數 | 預設 | SITL（`cbba_uav.sh`） | 說明 |
|---|---|---|---|
| `safety_reserve` | 20 % | 20 | 做完整條路徑並返航後至少要剩這麼多，否則不出價。PX4 的低電量警告 `BAT_LOW_THR` 是 15%，多留 5% 給估計誤差 |
| `energy_per_meter` | 0.5 %/m | 0.022 | 每公尺耗電。**預設值是舊測試值**，刻意放大到 100 m 的場地也碰得到電量限制（滿電只能飛 200 m），和真實的多旋翼差 20 倍以上 |
| `hover_energy_per_sec` | 0.2 %/s | 0.111 | 現場懸停（任務的 `duration_sec`）每秒耗電 |
| `cruise_speed` | 5.0 m/s | 5.0 | 估到達時間用（再乘 `speed_margin` 0.8）。**要和執行者實際的速度一致** |

電量來自 `robot_state.battery`；無人機由轉接節點從 PX4 的 `battery_status.remaining`（× 100）換算。
上面是 `vehicle_type:=uav` 的預設值；`ugv` 的預設是 0.1 %/m、0.05 %/s、1.0 m/s（佔位值，見 [`ugv_tuning.md`](ugv_tuning.md)）。

SITL 的值是從模擬電池換算的：PX4 的模擬電池依時間線性耗電、和飛多遠無關，續航 `SIM_BAT_DRAIN` = 900 s，
所以懸停 = 100 / 900 ≈ 0.111 %/s，以 5 m/s 前飛 = 0.111 / 5 ≈ 0.022 %/m。
上實機時要用 ulog 的 `battery_status`、`vehicle_local_position` 實際量。

> **電量會決定任務接不接得起。** 例如用預設值、電量 50% 時，可用只有 30%：飛 20 m（10%）＋懸停 60 s（12%）＋返航 17 m（8.5%）
> 就超過了，這個任務沒有人會出價，也沒有任何提示（見最後的已知限制）。

### 5. 出價

| 參數 | 預設 | 說明 |
|---|---|---|
| `battery_weight` | 1.0 | 電池因子的權重。1 表示用完可用電量的路徑，成本變兩倍。實測只在 0～2 之間會改變分配 |
| `cost_ref` | 50 m | 成本轉分數的基準：邊際成本 = 50 時分數是 value 的一半。不影響同一個任務誰贏，影響的是**不同價值的任務之間怎麼取捨**。取場地半寬左右 |
| `max_bundle` | 5 | 一台最多同時排幾個任務。限制計算量（約 O(任務數 × n³)）和規劃視野 |

公式與實驗見 `cbba_parameters.md`。

### 6. 協定

| 參數 | 預設 | 說明 |
|---|---|---|
| `lost_timeout` | 1.5 s | 某台的時間戳 s 這麼久沒更新就判定失聯，釋放它的任務重新分配（規格的 1.5 s）。CBBA_STATE 每 200 ms 一則，1.5 s 約 7 則都沒收到才判定。**全隊要一樣**，否則對同一台的判斷會不一致 |
| `max_packet` | 1200 B | CBBA_STATE 每批（整個 UDP 封包）的上限。任務多時分批，每批 (1200 − 36 − 6 × 機數) / 16 筆；依 mesh 的 MTU 調整 |
| `cancel_authorities` | `[]` | 除了任務的建立者，可以取消任何任務的機號（授權的管理端）。**全隊要一樣**：收到取消的 TASK_CLOSE 時也用它檢查，不一樣的話有的節點會拒絕取消 |

---

## 二、協定層（`CommConfig`，程式常數）

這些沒有開成 ROS 參數，改了要重新編譯。大多數是規格書的數字，或和其他參數有連動，不建議單獨改。

### 傳送時機

| 常數 | 值 | 說明 |
|---|---|---|
| `state_period` | 0.2 s | CBBA_STATE 週期（規格的 AGENT_STATE：每架每秒 5 次） |
| `status_period` | 0.5 s | AGENT_STATE（載具狀態）週期 |
| `min_gap` | 0.01 s | 分配表有變動時立即送出，兩則之間最短間隔 |
| `fast_period`、`fast_max` | 0.05 s、60 次 | 和鄰居的得標者不一致時，每 50 ms 重送，同一步最多 60 次（3 s）。規格的「50 ms 沒進展就重送」 |
| `stable_steps` | 3 | 連續 3 個週期（0.6 s）分配不變才算收斂。log 的「收斂」通常在變動後約 0.5 s 才印 |

### 鄰居與失聯

| 常數 | 值 | 說明 |
|---|---|---|
| `neighbor_timeout` | 1.5 s | 這麼久沒直接收到某台的任何封包，就不算直接鄰居。影響完成確認要等誰、轉送的判斷、開機同步 |
| `lost_timeout` | 1.5 s | 見上（ROS 參數）。和 `neighbor_timeout` 不同：多跳時 A、C 不是直接鄰居，但 A 的 s 經由 B 一直更新，C 不會判定 A 失聯 |
| `reevaluate_period` | 1.0 s | 每秒依目前的位置、電量重算手上任務的分數 |

### 補發

| 常數 | 值 | 說明 |
|---|---|---|
| `resend_gap` | 1.0 s | 同一個 TASK_ANNOUNCE 或 TASK_CLOSE，最短隔多久補一次。鄰居的摘要（round_key）不同時才補；地面上（不參與出價）的鄰居當成什麼都不知道，全部補一次 |
| `replay_period` | 5.0 s | 保底重播：每 5 s 輪流重播一個已知任務。摘要比對漏掉時的最後安全網 |

### 完成確認

| 常數 | 值 | 說明 |
|---|---|---|
| `completion_retry` | 1.0 s | 完成宣告沒收齊確認時，每秒重送一次（每次換新 sequence） |
| `completion_max` | 10 次 | 超過次數就以已收到的確認送出 TASK_CLOSE。宣告後才失聯的飛機不再等 |

### 開機同步

| 常數 | 值 | 說明 |
|---|---|---|
| `join_wait` | 1.5 s | 剛啟動時，這麼久都沒聽到任何鄰居＝單獨一台，可以開始送出自己建立的任務 |
| `join_timeout` | 5.0 s | 有鄰居但摘要一直對不上時最多等這麼久；之後照樣開始 |

摘要和任一鄰居相同時會立刻開始，不用等。目的是重開機後先補回既有任務，BT 重用了開機前的 task_id 時才查得出來
（不送出，log 印「和已知的 task_id 重複」）。

### 其他

| 常數 | 值 | 說明 |
|---|---|---|
| deadline_ms = 0 | 300 s | 收到沒有截止時刻的任務時用 |

### 封包層（`wire_header.hpp`）

| 常數 | 值 | 說明 |
|---|---|---|
| `SessionFilter` 的 `window` | 4096 號 | 每個發送者的滑動視窗：視窗內沒收過的接受（亂序也接受），收過的、比視窗還舊的丟掉。新的 session（重開機）接受，舊 session 的丟掉 |
| `RelayDeduper` 的 `memory` | 60 s | 轉送事件的去重複記多久。鍵是（封包種類, origin_id, origin_session, origin_seq） |

---

## 三、出價公式（`ScoringParams`，程式常數）

| 常數 | 值 | 說明 |
|---|---|---|
| `speed_margin` | 0.8 | 規劃用速度 = `cruise_speed` × 0.8，把到達時間估晚一點，寧可以為會遲到也不要答應了卻趕不上 |
| `min_deadline` | 1.0 s | 剩餘期限的下限，避免除以零。逾期的任務成本很大但還是可以接（軟限制） |
| `score_epsilon` | 1e-3 | 分數相差在這以內視為平手，由機號小的得標。分數用 float32 傳，精度約 1e-5 |
| `rebid_threshold` | 0.1 | 重新評估時，分數變動超過 10% 才更新。避免分數小幅抖動讓任務在兩台之間換手 |

`battery_weight`、`cost_ref`、`max_bundle` 已開成 ROS 參數（見上）。

---

## 四、任務本身的欄位（`swarm_interfaces/Task`）

BT（或 `cbba_task.sh`）建立任務時給的值，也是出價的一部分：

| 欄位 | `cbba_task.sh` 預設 | 說明 |
|---|---|---|
| `task_id` | 機號 × 65536 + 流水號 | 建立者必須是自己。**同一輪不能重複**，重複的會被拒絕 |
| `type` | 1（AIR_RECON） | 無人機只接 AIR_RECON；GROUND_INTERVENTION、PATROL 只有地面載具接 |
| `position` | z = 0 | map ENU。AIR_RECON 的 z 會被拉到 `recon_altitude` |
| `value` | 80 | 任務價值，分數的上限。火點與例行點的建議值見 `cbba_parameters.md` |
| `duration_sec` | 5 s | 現場停留時間，會算進懸停耗電（`hover_energy_per_sec` × 秒數）。停留久的任務很吃電 |
| `deadline_sec` | 300 s（0 也當 300） | 從建立起算的期限，軟限制。TASK_ANNOUNCE 傳的是絕對截止時刻，轉送、補發都不會把期限往後推 |

---

## 五、SITL 的設定（`docker/scripts`）

| 環境變數 | 預設 | 用在哪 | 說明 |
|---|---|---|---|
| `SIM_BAT_DRAIN` | 900 s | `px4_sitl.sh`、`cbba_uav.sh` | PX4 模擬電池的續航，也用來換算能量模型。**sim 和 uav 兩邊要一樣**。PX4 自己的預設是 60 s |
| `SIM_BAT_MIN_PCT` | 20 % | `px4_sitl.sh` | 模擬電池降到這裡就停住（= 安全存量，不會觸發 PX4 的低電量 failsafe）。PX4 預設 50% |
| `CBBA_SPEED` | 5.0 m/s | `cbba_uav.sh` | 同時給 `cbba_node` 的 `cruise_speed` 和 `task_executor` 的 `cruise_speed` |
| `CBBA_ALTITUDE` | 5.0 m | `cbba_uav.sh` | 同時給 `cbba_node` 的 `recon_altitude` 和 `task_executor` 的 `altitude` |
| `UAV_SPAWN_SPACING` | 2.0 m | `.env` | 出生點沿 y 方向的間隔；`px4_sitl.sh`、`px4_state_bridge`、`task_executor` 都讀它 |
| `EXEC_ARGS` | — | `cbba_uav.sh` | 額外給 `task_executor` 的參數，例如分層高度 `-p altitude:=7.0` |

模擬電池只在解鎖時下降，上鎖後回到 100%。

`task_executor`（代替 BT）自己的參數見 `px4_waypoint_node/README.md`：`altitude`、`cruise_speed`、`reach_tolerance`（0.5 m）、`rate_hz`（10 Hz）、`spawn_enu`。

---

## 六、要互相一致的參數

這些不一致的話，不會報錯，但分配和執行會悄悄地對不上：

| 一致的範圍 | 參數 | 不一致會怎樣 |
|---|---|---|
| 全隊（含機器狗） | `udp_group`、`udp_port`、`lost_timeout`、`cancel_authorities`、協定版本 | 收不到彼此、對同一台的失聯判斷不同、有的節點拒絕取消；版本 1 和版本 2 的節點互相拒收 |
| 全隊 | 機號不重複 | 兩台宣布同一個 task_id（log 會印「task_id 衝突」） |
| 每台的 CBBA 與執行者 | `cruise_speed` ↔ 執行者速度 | 到達時間估錯，期限判斷不準 |
| 每台的 CBBA 與執行者 | `recon_altitude` ↔ 執行者飛行高度 | 距離、耗電估錯 |
| 每台的轉接節點與執行者 | 轉接節點 `spawn_enu` ↔ PX4 出生點 ↔ 執行者 `spawn_enu` | 位置差一個出生點的偏移，飛到錯的地方 |
| 每台的轉接節點與 CBBA | `agent_id`、`px4_ns` ↔ `agent_id`、`ns` | 收不到 robot_state（或機號不對被略過），永遠不參與出價 |
| CBBA 與電池 | `energy_per_meter`、`hover_energy_per_sec` ↔ 實際耗電 | 接了做不完，或做得完卻不接 |
| 全隊 | 系統時鐘（chrony） | 時間戳 s 判斷錯誤：誤判失聯或不失聯 |

---

## 七、已知限制

1. **已結束的任務不回收**：一直記著，避免延遲的封包把它加回來。每個任務約 100 B，任務多到上萬才需要考慮。
   協定版本 1 的「每台約 32 個任務」限制已經沒有了。
2. **沒有飛機接得起的任務沒有提示**，會一直留在競標池直到期限。
3. **執行中的任務可能被搶**：別台穩定地出價更高時，任務會換手（CBBA 本來的行為）。`assign_hold` 只擋掉短暫的換手。
4. **能量模型還沒校正**：預設值是舊測試值，SITL 的值只對齊模擬電池，不代表實機。
