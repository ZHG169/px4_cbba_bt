# uav_cbba

無人機端的 CBBA（C++17）：CBBA 核心、機間通訊封包、協定層、機上節點 `cbba_node`、離線模擬。

- 機間通訊依「機間通訊封包規格」2026-10-06，走 mesh 上的 UDP multicast；DDS 只用在機內（PX4、BT）。
- 核心、封包、協定層都不依賴 ROS，可以單獨編譯和測試。
- 機器狗的 CBBA 節點在另一個套件 `ugv_cbba`，共用這裡的核心、協定層與節點骨架（`cbba_node_base`）。

| 文件 | 內容 |
|---|---|
| [`doc/parameters.md`](doc/parameters.md) | 所有參數：意義、預設值、為什麼是這個值、要和誰一致、已知限制 |
| [`doc/sitl_test.md`](doc/sitl_test.md) | PX4 SITL＋Gazebo 的測試步驟（分配、失敗交回、墜毀、晚加入） |
| [`cbba_parameters.md`](cbba_parameters.md) | 出價公式各參數的推導與實驗數據 |
| `doc/機間通訊封包規格_20261006.pdf` | 封包規格書。我們和規格不同的地方，寫在各 `wire_*.hpp`、`cbba_comm.hpp` 的開頭 |

## 檔案結構

```
uav_cbba/
├── include/uav_cbba/
│   ├── types.hpp             載具、任務、相容表（無人機只接 AIR_RECON）
│   ├── energy_model.hpp      電池與路徑能量（含回程）
│   ├── scoring.hpp           成本與分數、最佳插入位置
│   ├── cbba_agent.hpp        17 條消解規則、連鎖退標、bundle、出價上限、重新評估
│   ├── wire_io.hpp           封包的位元組讀寫（小端序、不補位）
│   ├── wire_header.hpp       共用表頭（14 B）、轉送表頭（9 B）、seq 計數與過濾、轉送與去重複
│   ├── wire_agent_state.hpp  AGENT_STATE 的編解碼與 flags
│   ├── wire_task_event.hpp   TASK_EVENT（版本 1 照規格；版本 2 加任務類型、截止時刻、執行時間）
│   ├── wire_completion.hpp   COMPLETION（宣告／證明）與 COMPLETION_ACK
│   ├── cbba_comm.hpp         機間協定：任務編號、轉送、補發、完成確認、失聯、開機同步、seq 預約
│   ├── udp_link.hpp          只走 mesh 網卡的 UDP multicast
│   ├── cbba_node_base.hpp    CBBA 節點的共用骨架（ROS）：UDP 迴圈、交給 BT、seq 紀錄、log
│   └── network_sim.hpp       離線模擬：丟包網路、場景檔讀取
├── src/                      上面各標頭檔的實作，以及 cbba_node.cpp（無人機的 ROS 2 節點）
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

在 uav 容器裡（所有容器共用編譯結果，不要兩個容器同時編譯）：

```bash
cd ~/CBBA_BT/ros2_ws
colcon build --symlink-install --packages-select swarm_interfaces uav_cbba
source install/setup.bash
```

不用 ROS、直接用 CMake（只編核心、離線模擬和測試；第一次會下載 googletest）：

```bash
cd ~/CBBA_BT/ros2_ws/src/uav_cbba
cmake -S . -B /tmp/uav_cbba_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/uav_cbba_build -j
```

編譯要零警告（`-Wall -Wextra -Wpedantic`）。

## 機上節點：cbba_node

每台無人機的機上電腦各跑一個。

```bash
# uav 容器裡：機號、命名空間、mesh 網卡 IP 預設讀環境變數 UAV_ID、UAV_NS、MESH_IP
ros2 run uav_cbba cbba_node

# 不接 PX4、一律參與出價（只測協商）
ros2 run uav_cbba cbba_node --ros-args -p use_px4:=false -p initial_position:="[0.0, 0.0, 5.0]" -p battery:=80.0
```

PX4 SITL 上用 `docker/scripts/cbba_uav.sh` 一次啟動 XRCE Agent、task_executor（暫代 BT）、cbba_node，
能量模型、速度、巡檢高度會一起對齊模擬電池；任務用 `cbba_task.sh` 建立。步驟見 [`doc/sitl_test.md`](doc/sitl_test.md)。

### 話題（以 uav1 為例）

| 話題 | 型別 | 方向 | 說明 |
|---|---|---|---|
| `/uav1/new_task` | `swarm_interfaces/Task` | BT → CBBA | 本機發現的新任務；`task_id` = 機號 × 65536 + 流水號，同一輪不能重複 |
| `/uav1/task_result` | `swarm_interfaces/Task` | BT → CBBA | 任務結束時一次：`status = DONE` 完成；`CANCELLED` 失敗，交回競標池、自己不再接 |
| `/uav1/assigned_task` | `swarm_interfaces/Task` | CBBA → BT | 目前要執行的任務（reliable、transient_local）；同一任務連續 0.6 s 排第一才發；沒有任務時 `task_id = 0`、`status = CANCELLED` |
| `/uav1/fmu/out/*` | `px4_msgs` | PX4 → CBBA | 位置（NED → map ENU，加出生點）、電量、armed、是否離地 |

- 巡檢任務（AIR_RECON）的 z 低於 `recon_altitude`（5 m）時改成巡檢高度：BT 常給地面座標。
- 地面處置任務（GROUND_INTERVENTION）照樣廣播，但無人機不出價，給機器狗。

不接 BT 時可以用指令代替：

```bash
ros2 topic pub --once /uav1/new_task swarm_interfaces/msg/Task \
  "{task_id: 65537, type: 1, position: {x: 2.0, y: 3.0, z: 0.0}, deadline_sec: 60.0, value: 80.0, duration_sec: 10.0}"
ros2 topic echo /uav1/assigned_task --qos-durability transient_local --qos-reliability reliable
ros2 topic pub --once /uav1/task_result swarm_interfaces/msg/Task "{task_id: 65537, status: 1}"   # 1 = DONE
```

### 參數

最常用的幾個；全部的參數和說明見 [`doc/parameters.md`](doc/parameters.md)。

| 參數 | 說明 | 預設 |
|---|---|---|
| `agent_id` | 機號 1～8，全隊不能重複 | 環境變數 `UAV_ID` |
| `px4_ns` | 命名空間（`/uavN/...`） | 環境變數 `UAV_NS` |
| `mesh_ip` | UDP 走哪張網卡 | 環境變數 `MESH_IP` |
| `use_px4` | 從 PX4 讀位置與電量 | `true` |
| `participate` | 何時參與出價：`airborne`（解鎖且離地）、`armed`、`always` | `airborne` |
| `assign_hold` | 同一個任務連續排第一多久才交給 BT（秒） | 0.6 |
| `recon_altitude` | 巡檢任務的高度，要和執行者的飛行高度一致 | 5.0 |
| `energy_per_meter`、`hover_energy_per_sec` | 能量模型（**未校正**；SITL 用 0.022、0.111） | 0.5、0.2 |
| `cruise_speed` | 估到達時間用，要和執行者的速度一致 | 5.0 |
| `battery_weight`、`cost_ref` | 出價參數 | 1.0、50 |
| `lost_timeout` | 多久沒有某台的新資訊視為失聯（秒），全隊要一樣 | 1.5 |
| `seq_file` | seq 預約紀錄，重開機後 seq 接著繼續編 | `~/.cbba/seq_uavN` |

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

細節見 `include/uav_cbba/cbba_comm.hpp`。

- 封包：共用表頭、轉送表頭、AGENT_STATE、TASK_EVENT、COMPLETION、COMPLETION_ACK，格式照規格書。
  TASK_EVENT 另有版本 2，尾端加任務類型、絕對截止時刻、執行時間（7 B）。
- 任務編號各機自編、依機號交錯（uav1：0、8、16…）；`task_id` 以 8 位十六進位寫在 TASK_EVENT 的名稱欄。
- AGENT_STATE 裡不認得或已結束的任務填 y = −1。`round_key` 是任務清單的 CRC-32 摘要，不同時逐格補發缺的任務與完成證明；
  不參與出價的鄰居（還在地面）當成什麼都不知道，全部補一次。另外每 5 s 保底重播。
- 剛啟動的節點先和鄰居同步（摘要相同）才自編任務編號，重開機後不會撞號。
- seq 重開機後接著開機前的繼續編：預約的上限存在 `seq_file`，先寫入才使用；沒有紀錄時用系統時鐘當起點。
- 完成確認照規格的宣告 → 確認 → 證明；重送一律換新序號（多跳時才補得回來），宣告後失聯的飛機不再等。
- 某台 1.5 s 沒有新資訊視為失聯，它的任務重新分配；沒起飛的飛機不出價。

## 測試

```bash
cd ~/CBBA_BT/ros2_ws
colcon test --packages-select uav_cbba --event-handlers console_direct+
colcon test-result --verbose

# 或直接用 CMake 編譯的版本
ctest --test-dir /tmp/uav_cbba_build --output-on-failure
```

共 108 項：

| 測試 | 內容 |
|---|---|
| `test_energy_model` | 能量計算、回程、安全存量 |
| `test_scoring` | 手算結果、異質出價（5 個混合任務）、電池 Cases A/B/C、逾期、排序 |
| `test_rules` | 用 `cbba_rule_cases.csv` 驗證 17 條規則 |
| `test_agent` | 連鎖退標、任務終止、舊訊息過濾、重新評估 |
| `test_lossy_network` | 0%～70% 丟包下的收斂率、100% 斷網 |
| `test_wire_header` | 共用表頭與轉送表頭的位元組排列與規格書一致、拒絕不合法的表頭、seq 過濾與重開機、seq 起點與繞回 0、轉送與去重複 |
| `test_wire_agent_state` | AGENT_STATE 的大小（34／80／85／179 B）與位元組排列、來回編解碼、拒絕不合法的封包、flags |
| `test_wire_task_event` | TASK_EVENT 的大小（42～49 B；版本 2 為 49～56 B）與位元組排列、轉送後內容不變、拒絕不合法的封包、截止時刻 |
| `test_wire_completion` | COMPLETION（39／40+9K B）與 COMPLETION_ACK（39 B）的大小與位元組排列、拒絕不合法的封包、多跳時重送換新序號才補得回來 |
| `test_comm` | 機間協定：任務擴散與編號、多跳轉送與完成確認（含掉包重送）、失敗交回、不參與出價、失聯重新分配、中間編號補發、晚加入（地面上的飛機 1 s 內補齊）、重開機（不撞號、seq 接續）、開機同步、seq 預約、機號 50 的地面載具、10%／30% 掉包、收斂後流量 |

多節點整合測試：同一個容器用 `use_px4:=false` 起多個 `cbba_node`，用 `ros2 topic pub` 代替 BT（走真的 UDP）。
停節點要用 `pkill -f "cbba_node.*agent_id:=N"`，只停 `ros2 run` 外層的話節點會繼續跑。

## 離線模擬：cbba_sim

三個指令，結果都寫成 CSV。下面用 `SIM` 代表執行檔：

```bash
# 容器裡（colcon build 之後）
SIM="ros2 run uav_cbba cbba_sim"
# 或 CMake 編譯的版本
SIM=/tmp/uav_cbba_build/cbba_sim

SCN=~/CBBA_BT/ros2_ws/src/uav_cbba/scenarios
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
python3 ~/CBBA_BT/ros2_ws/src/uav_cbba/scripts/plot_results.py ~/CBBA_BT/ros2_ws/results/demo \
        --scenario ~/CBBA_BT/ros2_ws/src/uav_cbba/scenarios/fire_demo.csv   # 選用：標示火點
python3 ~/CBBA_BT/ros2_ws/src/uav_cbba/scripts/plot_results.py ~/CBBA_BT/ros2_ws/results/battery --show
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
- 每台只能建立約 32 個任務（任務編號 uint8、依機號交錯、已完成的不回收）
- 沒有飛機接得起的任務沒有提示；只關 PX4 時 cbba_node 會繼續出價
- AGENT_STATE 的執行進度（`progress`）與「跟隨中」旗標，要等 BT 回報
- docker 的 Fast DDS 設定仍讓 DDS 經過 mesh；要讓 DDS 只留在機內，需把 uav 容器的白名單改成只有線材網卡
