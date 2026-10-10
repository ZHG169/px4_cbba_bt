# cbba_core

空地共用的 CBBA（C++17）：CBBA 核心、機間通訊封包、協定層、通用的機上節點 `cbba_node`、離線模擬。

- 機間通訊走 mesh 上的 UDP multicast，**協定版本 2**（2026-10-09）：每筆任務資料都帶 task_id，不相容於「機間通訊封包規格」
  2026-10-06（見 [`doc/protocol.md`](doc/protocol.md)）。DDS 只用在機內（PX4、BT）。
- 核心、封包、協定層都不依賴 ROS，可以單獨編譯和測試。
- `cbba_node` 不分載具、不依賴 PX4：無人機和機器狗跑同一個節點，只吃標準介面（RobotState、TaskResult、Task）。
  無人機的 PX4 話題由 `px4_waypoint_node/px4_state_bridge` 轉成 RobotState；狗的 BT 直接送。

| 文件 | 內容 |
|---|---|
| [`doc/protocol.md`](doc/protocol.md) | 機間通訊協定版本 2：封包總覽、接收端處理、分批、和規格書與建議不同的地方、流量 |
| [`doc/parameters.md`](doc/parameters.md) | 所有參數：意義、預設值、為什麼是這個值、要和誰一致、已知限制 |
| [`doc/sitl_test.md`](doc/sitl_test.md) | PX4 SITL＋Gazebo 的測試步驟（分配、失敗交回、墜毀、晚加入） |
| [`doc/ugv_test.md`](doc/ugv_test.md) | 機器狗端的執行方式與測試資料 T1～T8 |
| [`doc/ugv_tuning.md`](doc/ugv_tuning.md) | 機器狗的能量模型怎麼量、怎麼改 |
| [`cbba_parameters.md`](cbba_parameters.md) | 出價公式各參數的推導與實驗數據 |
| `doc/機間通訊封包規格_20261006.pdf` | 封包規格書（版本 1 照它；版本 2 不相容，原因見 `doc/protocol.md`） |

## 檔案結構

```
cbba_core/
├── include/cbba_core/
│   ├── types.hpp             載具、任務、相容表（無人機只接 AIR_RECON）
│   ├── energy_model.hpp      電池與路徑能量（含回程）
│   ├── scoring.hpp           成本與分數、最佳插入位置
│   ├── cbba_agent.hpp        17 條消解規則、連鎖退標、bundle、出價上限、重新評估
│   ├── wire_io.hpp           封包的位元組讀寫（大端序、不補位）
│   ├── wire_header.hpp       表頭（20 B）、轉送區塊（14 B）、session 過濾（滑動視窗）、轉送與去重複
│   ├── wire_agent_state.hpp  AGENT_STATE：載具狀態（真實的 armed／offboard／landed、位置、電量、鄰居、路徑）
│   ├── wire_cbba_state.hpp   CBBA_STATE：各任務的得標者與得標價（帶 task_id）、時間戳 s、分批
│   ├── wire_task_announce.hpp TASK_ANNOUNCE：任務定義
│   ├── wire_completion.hpp   COMPLETION（宣告）、COMPLETION_ACK（確認）、TASK_CLOSE（結束）
│   ├── cbba_comm.hpp         機間協定：task_id 對照、轉送、補發、完成確認、失聯、開機同步、session、正式指派與鎖定
│   ├── assignment_manager.hpp 正式指派的流程：BT 接受、搶占、排隊（保留）、拒絕、逾時、分段量測（不依賴 ROS）
│   ├── session_registry.hpp  成員與 session 層的介面（只定義，還沒接上）
│   ├── udp_link.hpp          只走 mesh 網卡的 UDP multicast
│   └── network_sim.hpp       離線模擬：丟包網路、場景檔讀取
├── src/                      上面各標頭檔的實作，以及 cbba_node.cpp（通用的 ROS 2 節點）
├── tools/cbba_sim.cpp        離線模擬工具，輸出 CSV
├── scripts/                  plot_results.py（畫圖）、animate_results.py（GIF）、compare_hover.py（電量比較 GIF）
├── scenarios/                離線模擬的測資
│   ├── fire_demo.csv         3 台無人機＋1 隻假的機器狗，11 個任務
│   ├── fire_3uav.csv         同樣的火場，只用 3 台無人機、8 個空中任務（2 個火點）
│   ├── battery_demo.csv      2 台無人機，近的那台電量偏低
│   └── fire_hover*.csv       火場懸停監看：考慮電量／不考慮電量的對照組
├── doc/                      規格書、參數說明、SITL 測試步驟、公式設計等
└── test/                     單元測試與共用的規則案例檔 cbba_rule_cases.csv
```

## 編譯

在 uav 容器裡（所有容器共用編譯結果，不要兩個容器同時編譯）。不依賴 px4_msgs，狗的機上電腦也直接編：

```bash
cd ~/CBBA_BT/ros2_ws
colcon build --symlink-install --packages-select swarm_interfaces cbba_core
source install/setup.bash
```

不用 ROS、直接用 CMake（只編核心、離線模擬和測試；第一次會下載 googletest）：

```bash
cd ~/CBBA_BT/ros2_ws/src/cbba_core
cmake -S . -B /tmp/cbba_core_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/cbba_core_build -j
```

編譯要零警告（`-Wall -Wextra -Wpedantic`）。

## 機上節點：cbba_node

每台載具的機上電腦各跑一個。同一個機號只能跑一個：啟動時對 `<lock_dir>/agentN.lock` 加程序鎖，拿不到就拒絕啟動，
退出才釋放（docker 的所有容器共用 `/run/cbba`）。程式裡固定叫 `cbba_node`，啟動時用 `-r __node:=` 加上載具名稱，
`ros2 node list` 才分得出是哪一台。

```bash
# 無人機（PX4 的狀態由 px4_state_bridge 轉成 /uav1/robot_state）
ros2 run cbba_core cbba_node --ros-args -r __node:=uav1_cbba_node -p agent_id:=1
# 機器狗（狗的 BT 送 /v60/robot_state）
ros2 run cbba_core cbba_node --ros-args -r __node:=v60_cbba_node -p vehicle_type:=ugv -p agent_id:=50
# 不接 PX4／狗、一律參與出價（只測協商）
ros2 run cbba_core cbba_node --ros-args -r __node:=uav1_cbba_node -p agent_id:=1 -p participate:=always \
  -p initial_position:="[0.0, 0.0, 5.0]" -p battery:=80.0
```

PX4 SITL 上用 `docker/scripts/cbba_uav.sh` 一次啟動 XRCE Agent、task_executor（暫代 BT）、px4_state_bridge、cbba_node，
能量模型、速度、巡檢高度會一起對齊模擬電池；任務用 `cbba_task.sh` 建立。步驟見 [`doc/sitl_test.md`](doc/sitl_test.md)。

### 話題（以 uav1 為例；狗是 `/v60/...`）

| 話題 | 型別 | 方向 | 說明 |
|---|---|---|---|
| `/uav1/robot_state` | `swarm_interfaces/RobotState` | 轉接節點或狗的 BT → CBBA | 位置（map ENU）、電量（負值 = 不知道）、飛行狀態（`flight_state_valid`、`armed`、`offboard`、`landed`，狗填 false）。`state_timeout`（1.5 s）內有更新才參與出價；第一筆的位置當返航點 |
| `/uav1/new_task` | `swarm_interfaces/Task` | BT → CBBA | 本機發現的新任務；`task_id` = 機號 × 65536 + 流水號，不能和已知的任務重複（BT 重開機後流水號要接續） |
| `/uav1/task_result` | `swarm_interfaces/TaskResult` | BT → CBBA | 任務結束時一次（reliable）：`success = true` 完成；`false` 執行失敗，任務保留、交回競標池、自己不再接；`detail` 只記 log。**task_id 和 `assignment_version` 都要是目前 assigned_task 上的**，否則拒絕 |
| `/uav1/cancel_task` | `swarm_interfaces/Task` | → CBBA | 取消任務（只看 task_id）。要是建立者或 `cancel_authorities` 裡的機號，全隊標成 CANCELLED |
| `/uav1/assigned_task` | `swarm_interfaces/Task` | CBBA → BT | 目前要執行的任務（正式指派，reliable、transient_local）：同一任務連續 0.6 s 排第一、**BT 接受後**才發，帶 `assignment_version`；沒有任務時 `task_id = 0`、`status = CANCELLED` |
| `/uav1/exec_state` | `swarm_interfaces/ExecState` | BT → CBBA | BT 的執行狀態：狀態、可否中斷、預估剩餘時間、執行中／排隊中的任務。決定能不能接新任務、能不能中斷、能不能排隊 |
| `/uav1/assignment_request` | `swarm_interfaces/AssignmentRequest` | CBBA → BT | 請 BT 接受：ACTIVATE（`preempt` = 要中斷目前的任務）、RESERVE（排隊保留）、RELEASE（作廢）。`require_accept = true` 時才送 |
| `/uav1/assignment_response` | `swarm_interfaces/AssignmentResponse` | BT → CBBA | 接受／拒絕（暫時、永久）＋原因，版本原樣帶回 |

- 巡檢任務（AIR_RECON）的 z 低於 `recon_altitude`（無人機 5 m）時改成巡檢高度：BT 常給地面座標。
- 地面處置任務（GROUND_INTERVENTION）照樣廣播，但無人機不出價，給機器狗。

不接 BT 時可以用指令代替：

```bash
ros2 topic pub --once /uav1/new_task swarm_interfaces/msg/Task \
  "{task_id: 65537, type: 1, position: {x: 2.0, y: 3.0, z: 0.0}, deadline_sec: 60.0, value: 80.0, duration_sec: 10.0}"
ros2 topic echo /uav1/assigned_task --qos-durability transient_local --qos-reliability reliable
ros2 topic pub --once /uav1/task_result swarm_interfaces/msg/TaskResult \
  "{task_id: 65537, success: true, assignment_version: 1}"   # 版本照 assigned_task 上的
ros2 topic pub --once /uav1/cancel_task swarm_interfaces/msg/Task "{task_id: 65537}"
```

### 參數

最常用的幾個；全部的參數和說明見 [`doc/parameters.md`](doc/parameters.md)。

| 參數 | 說明 | 預設 |
|---|---|---|
| `agent_id` | 機號 1～254，全隊不能重複（協定本身是 1～65534，`RobotState.agent_id` 是 uint8） | 必填 |
| `vehicle_type` | `uav` 或 `ugv`：能接哪些任務，以及能量模型、巡檢高度的預設值 | `uav` |
| `ns` | 話題前綴（`/<ns>/...`） | `uav<agent_id>`；ugv 是 `v60` |
| `mesh_ip` | UDP 走哪張網卡 | 環境變數 `MESH_IP` |
| `participate` | `state`：robot_state 有在更新才參與（無人機另外要飛行狀態有效、已解鎖、離地）；`always`：一律參與，位置與電量用 `initial_position`、`battery`（只測協商） | `state` |
| `require_airborne` | 無人機要離地才參與（false = 解鎖就參與） | true |
| `state_timeout` | robot_state 多久沒更新就停止參與出價、釋放任務（秒） | 1.5 |
| `assign_hold` | 同一個任務連續排第一多久才交給 BT（秒） | 0.6 |
| `recon_altitude` | 巡檢任務的高度，要和執行者的飛行高度一致；0 = 不調整 | uav 5.0、ugv 0 |
| `energy_per_meter`、`hover_energy_per_sec` | 能量模型（**未校正**；SITL 用 0.022、0.111） | uav 0.5、0.2；ugv 0.1、0.05 |
| `cruise_speed` | 估到達時間用，要和執行者的速度一致 | uav 5.0、ugv 1.0 |
| `battery_weight`、`cost_ref` | 出價參數 | 1.0、50 |
| `lost_timeout` | 多久沒有某台的新資訊視為失聯（秒），全隊要一樣 | 1.5 |
| `max_packet` | CBBA_STATE 每批（整個 UDP 封包）的上限（B），任務多時分批 | 1200 |
| `cancel_authorities` | 除了建立者，可以取消任務的機號（授權的管理端），全隊要一樣 | `[]` |
| `require_accept` | 要 BT 接受才執行（握手）；false 只跳過握手，可行性、版本、回報檢查照常 | uav false、ugv true |
| `accept_timeout` | BT 多久沒回覆就當成暫時拒絕（故障處理的上限） | 3.0 s |
| `exec_timeout` | ExecState 多久沒更新就不能接新任務（本機單調時間） | 1.5 s |
| `max_queued_fires`、`max_queue_wait` | 執行中任務之後最多排幾個火警；沒有期限時最多等多久（從建立時算） | 1、120 s |
| `lock_dir` | 程序鎖的目錄 | `CBBA_LOCK_DIR` → `$XDG_RUNTIME_DIR/cbba` → `/tmp/cbba` |

## 怎麼分配任務

沒有用最佳化求解器，是 CBBA 標準的兩階段：

1. **每台自己建 bundle（貪婪＋插入）**：對每個還沒接的任務，試著插進目前路徑的每一個位置，
   取總成本增加最少的位置，算出分數；挑分數最高、而且贏得過目前得標價的任務插進去，重複到 bundle 滿（5 個）或沒有能贏的。
2. **共識**：透過 AGENT_STATE 交換每個任務的得標者、得標價、資訊時間，照 Choi 2009 的 17 條規則消解衝突；
   輸掉的任務連同排在後面的一起退出（連鎖退標），再回到第 1 步。反覆到全隊一致。

### 成本與分數

```
每一段     cost = 距離 × (到達時間 / 剩餘期限)                       # 距離是直線距離
整條路徑   C_ST = Σ cost
           C_total = C_ST × (1 + battery_weight × 路徑耗電 / 可用電量)  # 耗電含回程
分數       score = value / (1 + 邊際成本 / cost_ref)                   # 邊際成本 = 插入後 − 插入前
出價       bid = min(score, bundle 裡既有出價的最小值)
```

- 期限是軟限制：來不及時分數降低，但不歸零。
- 做完並返航後電量低於安全存量時才不可行（不出價）。
- 出價上限（bid warping）：確保接的任務越多、出價不會越高，CBBA 才保證收斂。
- 這是貪婪法，不是最佳解；插入時不會重排已經排好的任務。

### 機間通訊

協定版本 2，細節見 [`doc/protocol.md`](doc/protocol.md) 和 `include/cbba_core/cbba_comm.hpp`。

- 封包：AGENT_STATE（狀態）、CBBA_STATE（共識資料）、TASK_ANNOUNCE（任務定義）、COMPLETION、COMPLETION_ACK、TASK_CLOSE。
  20 B 表頭（magic、版本、種類、sender_id、session_id、sequence、payload_length），大端序。
- 每筆任務資料都帶 uint32 task_id，各節點自己維護對照：任何機號都能建立任務，數量不限（只受記憶體、封包大小限制）。
- CBBA_STATE 只放認得、進行中的任務（task_id、得標者、得標價），每批 ≤ 1200 B，每批都帶完整的 s。
  每批直接交給 CBBA 規則；收斂判定、補發只用收齊的那一次。
- `round_key` 是任務清單的 CRC-32 摘要，不同時比對紀錄、補發對方缺的 TASK_ANNOUNCE 或 TASK_CLOSE；
  不參與出價的鄰居（還在地面）沒有附紀錄，全部補一次。另外每 5 s 保底重播。
- 重開機換新的 session_id（隨機），不用存檔；重開機前延遲的封包會被丟掉。
- 完成：宣告 → 確認 → TASK_CLOSE；重送一律換新序號（多跳時才補得回來），宣告後失聯的不再等。
  存活名單是機號清單，機號 50 的狗也會被等待確認。
- **接受、搶占、排隊**（見 [`doc/protocol.md`](doc/protocol.md) 第五節）：得標後先請 BT 接受，接受後才是正式指派；
  已接受（執行中、保留）的任務別台不出價（持有者失聯也照樣鎖著，只標記待確認）。搶占要可以安全中斷、優先級較高
  （火警 ＞ 巡檢，同級不搶占）；執行中任務之後最多保留 1 個火警，要在期限內做得完。每個指派印一行分段量測。
- **正式指派**：發到 assigned_task 時產生新版本（全隊共用、每個任務一個），撤銷時也加一；版本跟著 CBBA_STATE 傳。
  回報、完成宣告、TASK_CLOSE 都帶版本，舊的拒絕（X→Y→X 時 X 第一次的回報無效）。
  取消只能由建立者或 `cancel_authorities` 發起；執行失敗不關閉任務，撤銷指派後由 CBBA 重新分配。
- 某台 1.5 s 沒有新資訊視為失聯，它的任務重新分配；沒起飛的飛機不出價。

## 測試

```bash
cd ~/CBBA_BT/ros2_ws
colcon test --packages-select cbba_core --event-handlers console_direct+
colcon test-result --verbose

# 或直接用 CMake 編譯的版本
ctest --test-dir /tmp/cbba_core_build --output-on-failure
```

共 128 項：

| 測試 | 內容 |
|---|---|
| `test_energy_model` | 能量計算、回程、安全存量 |
| `test_scoring` | 手算結果、異質出價（5 個混合任務）、電池 Cases A/B/C、逾期、排序 |
| `test_rules` | 用 `cbba_rule_cases.csv` 驗證 17 條規則 |
| `test_agent` | 連鎖退標、任務終止、舊訊息過濾、重新評估 |
| `test_lossy_network` | 0%～70% 丟包下的收斂率、100% 斷網 |
| `test_wire_header` | 表頭與轉送區塊的位元組排列（大端序）、拒絕不合法的表頭、轉送後 payload 不變、session 過濾（亂序、重複、視窗、繞回、重開機、退役 session）、去重複的鍵 |
| `test_wire_agent_state` | AGENT_STATE 的大小（52 + 4L + 2K B）、位元組排列、來回編解碼、flags、拒絕不合法的封包 |
| `test_wire_cbba_state` | CBBA_STATE 的大小（36 + 6N + 16R B）、位元組排列（含正式指派）、來回編解碼、分批、拒絕不合法的封包 |
| `test_wire_task_announce` | TASK_ANNOUNCE（61 B）的位元組排列、轉送後不變、截止時刻、拒絕不合法的封包 |
| `test_wire_completion` | COMPLETION、COMPLETION_ACK、TASK_CLOSE 的大小、位元組排列（指派版本、ACK status、actor）、來回編解碼、拒絕不合法的封包 |
| `test_comm` | 機間協定：task_id 擴散、狀態與共識欄位、多跳轉送與完成確認（含掉包重送）、失敗交回、不參與出價、失聯重新分配、漏收補發、晚加入、重開機（新 session、延遲的舊封包、重用 task_id、自己完成的任務）、開機同步、狗 50 和 uav2 同時建立任務、狗也被等待確認、150 個任務分批、缺批、10%／30% 掉包、收斂後流量；正式指派：版本多跳傳遞與正常完成、錯誤回報者、X→Y→X 的舊回報、同時指派時舊的完成作廢、建立者／授權／無權限的取消、取消中止確認中的完成、失敗後以新版本重新分配 |
| `test_exec_constraints` | 核心的限制：搶占、排隊（數量、期限、剩餘時間）、鎖定、暫時撤回、故障（見 `doc/protocol.md` 第七節） |
| `test_assignment` | 指派流程：BT 接受、搶占、排隊與保留、拒絕、逾時、鎖定、失聯（見 `doc/protocol.md` 第七節） |

多節點整合測試：同一個容器用 `participate:=always` 起多個 `cbba_node`（各自 `-r __node:=uavN_cbba_node`），
用 `ros2 topic pub` 代替 BT（走真的 UDP）。停節點要用 `pkill -f "__node:=uavN_cbba_node( |$)"`，
只停 `ros2 run` 外層的話節點會繼續跑。

## 離線模擬：cbba_sim

三個指令，結果都寫成 CSV。下面用 `SIM` 代表執行檔：

```bash
# 容器裡（colcon build 之後）
SIM="ros2 run cbba_core cbba_sim"
# 或 CMake 編譯的版本
SIM=/tmp/cbba_core_build/cbba_sim

SCN=~/CBBA_BT/ros2_ws/src/cbba_core/scenarios
OUT=~/CBBA_BT/ros2_ws/results
```

```bash
# 1. 單次協商：分配結果與協商過程
$SIM run $SCN/fire_demo.csv --loss 0.3 --out $OUT/demo

# 2. 各丟包率（0/10/30/50/70%）重複多次：收斂率、收斂時間、流量
$SIM sweep random --trials 200 --agents 3 --tasks 6 --out $OUT/demo
$SIM sweep $SCN/fire_demo.csv --trials 200 --out $OUT/demo_fixed   # 固定場景，只有網路是隨機的

# 3. 比較不同電池權重（0 ~ 100）的分配結果
$SIM weights $SCN/battery_demo.csv --out $OUT/battery
```

| 選項 | 說明 | 預設 |
|---|---|---|
| `--out DIR` | 輸出目錄 | `cbba_out` |
| `--loss P` | 丟包率 0~1（`run` 用） | 0 |
| `--seed N` | 亂數種子 | 1 |
| `--trials N` | `sweep` 每個丟包率的次數 | 200 |
| `--agents N --tasks M` | `sweep random` 的規模 | 3、6 |
| `--battery-weight W` | 電池權重 | 1.0 |
| `--speed V` | 預設速度（場景檔有指定時以場景檔為準） | 5.0 |
| `--cost-ref C` | 成本轉分數的基準 | 50 |
| `--max-bundle N` | 一次最多接幾個任務 | 5 |

離線模擬用的是核心的預設能量參數（0.5 %/m、0.2 %/s），和 SITL 的值不同。

輸出的檔案：

| 檔案 | 產生的指令 | 內容 |
|---|---|---|
| `agents.csv` | run | 每台載具的位置、電量、做完並返航後的電量、任務數 |
| `tasks.csv` | run | 每個任務的位置、類型、期限、得標者（0 表示無人） |
| `assignment.csv` | run | 每台的執行順序、到達時間、是否逾期、出價 |
| `timeline.csv` | run | 每 10 ms 的不一致任務數與累計廣播次數 |
| `summary.csv` | run | 丟包率、是否收斂、收斂時間等 |
| `bids.csv` | run | 每台 x 每個任務：插入自己最終路徑的邊際成本與出價、只做這一個任務的成本與出價、是否得標 |
| `sweep.csv` | sweep | 每次試驗一列：丟包率、是否收斂、收斂時間、流量 |
| `weights.csv` | weights | 每個權重、每台載具一列：任務數、剩餘電量、到達時間 |

## 畫圖

需要 matplotlib。建議**在 host 上執行**（結果目錄是掛載的，host 看得到）：

```bash
python3 ~/CBBA_BT/ros2_ws/src/cbba_core/scripts/plot_results.py ~/CBBA_BT/ros2_ws/results/demo \
        --scenario ~/CBBA_BT/ros2_ws/src/cbba_core/scenarios/fire_demo.csv   # 選用：標示火點
python3 ~/CBBA_BT/ros2_ws/src/cbba_core/scripts/plot_results.py ~/CBBA_BT/ros2_ws/results/battery --show
```

要在容器裡畫圖的話，先安裝 `sudo apt update && sudo apt install -y python3-matplotlib`（容器重建後要再裝一次）。

目錄裡有哪些 CSV 就畫哪些圖，PNG 存在同一個目錄：

| 圖 | 來源 | 看什麼 |
|---|---|---|
| `allocation.png` | run | 地圖：載具、任務、各台的路徑與到達時間、電量變化、無人認領的任務 |
| `timeline.png` | run | 協商過程：不一致的任務數隨時間下降到 0；累計訊息數 |
| `sweep.png` | sweep | 丟包率對收斂率、收斂時間（中位數與 5%~95% 範圍）、流量 |
| `weights.png` | weights | 電池權重對各台剩餘電量、任務數、平均到達時間 |
| `fire_decision.png` | run（需 `--scenario` 且有 `fire` 任務） | 每個火點：各台的成本、誰成本最低、誰得標、沒得標的原因、CBBA 合作結果 |

GIF 動畫（需要 matplotlib 與 Pillow）：

```bash
# 任務重播：載具沿路徑移動、電量下降；火點紅色光圈；最後 5 秒顯示「誰去火災」
python3 scripts/animate_results.py mission <run 目錄> <場景檔>
# 電池權重逐格比較（每個目錄是用不同 --battery-weight 跑的 run）
python3 scripts/animate_results.py weights out.gif <目錄1> <目錄2> ...
# 火場懸停監看：不考慮電量 vs 考慮電量（用法見檔頭）
python3 scripts/compare_hover.py <未考慮電量 run> <考慮電量 run> scenarios/fire_hover.csv out.gif
```

圖上的文字是英文，避免沒有中文字型時顯示成方框（`compare_hover.py` 需要中文字型 Noto Sans CJK TC）。

## 測資格式

一個 CSV 檔，第一欄是 `agent` 或 `task`，`#` 開頭的是註解：

```
# kind,id,type,x,y,z,battery,safety_reserve,energy_per_meter,hover_energy_per_sec,cruise_speed
agent,1,UAV,-40,-30,5,100,20,0.5,0.2,5.0

# kind,id,type,x,y,z,deadline_sec,value,duration_sec[,tag]
task,1,AIR_RECON,-15,5,5,60,80,10,fire
```

- 載具類型：`UAV`、`UGV`。任務類型：`AIR_RECON`、`GROUND_INTERVENTION`、`PATROL`。
- task 的第 10 欄 `tag` 可省略，只給畫圖用，CBBA 不會讀。目前支援 `fire`（火點）：
  `plot_results.py --scenario 場景檔` 與 `animate_results.py mission` 會用紅色光圈＋紅字 `FIRE` 標示。
- 返航點等於起始位置。`cruise_speed` 可以省略，省略時使用 `--speed`。
- 機器狗（UGV）在離線模擬裡用和無人機相同的直線距離公式計分，只是用來觀察異質分配的替身。

## 還沒做的部分

- PX4 SITL 已驗證：分配、完成確認、失敗交回、墜毀重分配、晚加入、重開機（2026-10-07）。掉包、斷網的 SITL 測試還沒做
- 能量模型的參數校正（用實機 ulog 量；SITL 目前只對齊模擬電池）
- 沒有飛機接得起的任務沒有提示
- AGENT_STATE 的執行進度（`progress`）與「跟隨中」旗標，要等 BT 回報
- 協定版本 2 還沒在 PX4 SITL 跑；和規格書不相容，要和規格作者說明（見 `doc/protocol.md`）
- docker 的 Fast DDS 設定仍讓 DDS 經過 mesh；要讓 DDS 只留在機內，需把 uav 容器的白名單改成只有線材網卡
