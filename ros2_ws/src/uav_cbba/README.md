# uav_cbba

無人機端的 CBBA（C++17）。核心函式庫不依賴 ROS，對應「CBBA 介面規格草稿 v0.1」。

## 檔案結構

```
uav_cbba/
├── include/uav_cbba/
│   ├── types.hpp          載具、任務、相容表
│   ├── energy_model.hpp   電池與路徑能量（含回程）
│   ├── scoring.hpp        成本與分數
│   ├── cbba_agent.hpp     17 條消解規則、連鎖退標、bundle、出價上限、重新評估
│   └── network_sim.hpp    離線模擬：丟包網路、場景檔讀取
├── src/                   上面各標頭檔的實作
├── tools/cbba_sim.cpp     離線模擬工具，輸出 CSV
├── scripts/plot_results.py  把 CSV 畫成圖（matplotlib）
├── scenarios/             測資
│   ├── fire_demo.csv      3 台無人機＋1 隻假的機器狗，11 個任務
│   ├── fire_3uav.csv      同樣的火場，只用 3 台無人機、8 個空中任務（2 個火點）
│   └── battery_demo.csv   2 台無人機，近的那台電量偏低
└── test/                  單元測試與共用的規則案例檔 cbba_rule_cases.csv
```

## 成本與分數

```
每個任務   cost = 距離 x (到達時間 / 距離期限剩餘的時間)
整條路徑   C_ST = sum(cost)
           C_total = C_ST x (1 + battery_weight x 路徑耗能 / 可用電量)   # 耗能含回程
分數       score = value / (1 + 邊際成本 / cost_ref)                      # 做不到為 0
```

- 期限是軟限制：來不及時分數降低但不歸零。
- 電量不足以做完並返航時才是不可行（分數 0）。
- 出價上限：新任務的出價不得高於 bundle 裡既有出價的最小值，確保 CBBA 收斂。

## 編譯

在 uav 容器裡：

```bash
cd ~/CBBA_BT/ros2_ws
colcon build --symlink-install --packages-select swarm_interfaces uav_cbba
source install/setup.bash
```

不用 ROS、直接用 CMake（任何有 g++ 和 cmake 的環境，第一次會下載 googletest）：

```bash
cd ~/CBBA_BT/ros2_ws/src/uav_cbba
cmake -S . -B /tmp/uav_cbba_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/uav_cbba_build -j
```

## 測試

```bash
# 容器裡（colcon）
cd ~/CBBA_BT/ros2_ws
colcon --log-base log_uav test --build-base build_uav --install-base install_uav \
       --packages-select uav_cbba --event-handlers console_direct+

# 或直接用 CMake 編譯的版本
ctest --test-dir /tmp/uav_cbba_build --output-on-failure
```

| 測試 | 內容 |
|---|---|
| `test_energy_model` | 能量計算、回程、安全存量 |
| `test_scoring` | 手算結果、異質出價（5 個混合任務）、電池 Cases A/B/C、逾期、排序 |
| `test_rules` | 用 `cbba_rule_cases.csv` 驗證 17 條規則 |
| `test_agent` | 連鎖退標、任務終止、舊訊息過濾、重新評估 |
| `test_lossy_network` | 0%~70% 丟包下的收斂率、100% 斷網 |

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

## 畫圖：plot_results.py

需要 matplotlib。建議**在 host 上執行**（結果目錄是掛載的，host 看得到）：

```bash
python3 ~/CBBA_BT/ros2_ws/src/uav_cbba/scripts/plot_results.py ~/CBBA_BT/ros2_ws/results/demo \
        --scenario ~/CBBA_BT/ros2_ws/src/uav_cbba/scenarios/fire_demo.csv   # 選用：標示火點
python3 ~/CBBA_BT/ros2_ws/src/uav_cbba/scripts/plot_results.py ~/CBBA_BT/ros2_ws/results/battery --show
```

要在容器裡畫圖的話，先安裝 `sudo apt update && sudo apt install -y python3-matplotlib`
（容器重建後要再裝一次）。

目錄裡有哪些 CSV 就畫哪些圖，PNG 存在同一個目錄：

| 圖 | 來源 | 看什麼 |
|---|---|---|
| `allocation.png` | run | 地圖：載具、任務、各台的路徑與到達時間、電量變化、無人認領的任務 |
| `timeline.png` | run | 協商過程：不一致的任務數隨時間下降到 0；累計訊息數 |
| `sweep.png` | sweep | 丟包率對收斂率、收斂時間（中位數與 5%~95% 範圍）、流量 |
| `weights.png` | weights | 電池權重對各台剩餘電量、任務數、平均到達時間 |
| `fire_decision.png` | run（需 `--scenario` 且有 `fire` 任務） | 每個火點：各台的成本、誰成本最低、誰得標、沒得標的原因、CBBA 合作結果 |

GIF 動畫（`animate_results.py`，需要 matplotlib 與 Pillow）：

```bash
# 任務重播：載具沿路徑移動、電量下降；火點紅色光圈；最後 5 秒顯示「誰去火災」
python3 scripts/animate_results.py mission <run 目錄> <場景檔>
# 電池權重逐格比較（每個目錄是用不同 --battery-weight 跑的 run）
python3 scripts/animate_results.py weights out.gif <目錄1> <目錄2> ...
```

圖上的文字是英文，避免沒有中文字型時顯示成方框。

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
- 機器狗（UGV）在這裡用和無人機相同的直線距離公式計分，只是用來觀察異質分配的替身。

## 還沒做的部分

- ROS 2 節點（訂閱 `/swarm/cbba`、`/swarm/tasks`，把 `CbbaAgent` 接上訊息收發與定時重送）
- 從 `/uavN/fmu/out/...` 取得位置與電量，更新 `AgentState`
- 能量模型的參數校正（用 PX4 回報的實際耗電）
