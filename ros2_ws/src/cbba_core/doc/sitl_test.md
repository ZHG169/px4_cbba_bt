# cbba_node＋BT＋PX4 SITL：手動測試步驟

cbba_core · 2026-10-11 改寫（協定版本 2、預設用備用 BT）

一個指令跑完、自動判斷通過與否的版本是 **`docker/itest/itest.py`**（見 `docker/itest/README.md`），
這份是**一步一步看 log** 的手動版，用來看細節、除錯。兩者的項目對照寫在每一節的標題。
各種測試的啟動方式總覽在 `docs/sim_launch_guide.md`。

```
sim 容器：Gazebo＋PX4 #1、#2…（gz_bridge.sh 送下視相機影像）
uavN 容器：XRCE Agent ─ uav_bt_node（BT）──────────────┐
                      ├ fire_detector_node（AprilTag）─┤ 機內 DDS
                      └ px4_state_bridge ─ robot_state ─ cbba_node ═ UDP（協定版本 2）═ mesh_net ═ 其他 uav
```

`cbba_uav.sh` 一次開 XRCE Agent、BT、AprilTag 偵測、px4_state_bridge、cbba_node，輸出前綴
`[bt]`、`[tag]`、`[state]`、`[cbba]`。`EXECUTOR=task_executor cbba_uav.sh` 換成舊的簡易執行者（沒有避碰、沒有火警偵測）。

## 0. 準備（只做一次）

```bash
# 主機
cd ~/CBBA_BT/docker && ./start.sh
# uav1：編譯（所有容器共用，不要兩個容器同時編譯）
colcon build --symlink-install --packages-select swarm_interfaces cbba_core px4_waypoint_node uav_px4_bt apriltag_fire_detector
```

## 1. 啟動（itest U1）

| Terminal | 容器 | 指令 |
|---|---|---|
| 1～3 | `./enter.sh uavN` | `cbba_uav.sh`（每台的 Agent 要比同號的 PX4 先開） |
| 4 | `./enter.sh sim` | `CBBA_WORLD=fire_site px4_sitl.sh 1`，之後 `px4_sitl.sh 2`、`3`（自動加入同一個場景） |
| 5 | `./enter.sh sim` | `gz_bridge.sh`（測火警時才需要） |
| 6 | `./enter.sh uav1` | 發任務用 |

**電量與能量模型**：`px4_sitl.sh` 把模擬電池設成續航 900 s、最低 20%；`cbba_uav.sh` 用同一個續航算能量模型
（懸停 0.111 %/s、每公尺 0.022 %），速度 5 m/s、巡檢高度 5 m 同時給 cbba_node 和 BT。
要改就設 `SIM_BAT_DRAIN`、`SIM_BAT_MIN_PCT`、`CBBA_SPEED`、`CBBA_ALTITUDE`（sim 和同號的 uav 兩邊要一樣）。

**每台依序要看到**：
1. `[state] 開始送 robot_state`、`[state] 飛行狀態有效`
2. `[cbba] 返航點 map (…)`（地面的出生點）
3. `[bt] 解鎖、offboard：爬升到 5.0 m` → `[bt] 到達 5.0 m，等待指派`
4. `[cbba] 開始參與出價`（已解鎖、離地才出現）
5. `[cbba] 收斂：0 個任務，N−1 個鄰居`；別台的 `[cbba] 鄰居 N：參與 1、遙測 1、飛行狀態有效 1、armed 1、offboard 1、landed 0`

## 2. 執行任務（itest U2、U5）

```bash
cbba_task.sh new 1 1 10 5      # uav1 建立任務 00010001 在 (10, 5)；座標是 map ENU（公尺）
```

要看到（得標那台）：
- `[cbba] 路徑：00010001` → 約 0.4 s 後 `[cbba] 指派給 BT：00010001`
- `[bt] 新指派 00010001 v1` → `前往 00010001` → `到達 00010001，停留 5.0 s` → `回報 00010001 v1 完成`
- `[cbba] 任務 00010001 完成，等待全隊確認` → `已確認完成（回報後 0.0x s）`
- **其他每一台**：`任務 00010001 已確認完成（收到完成證明）`

多個任務：每台的 `[cbba] 路徑` 合起來涵蓋全部、不重複；排在後面的，做完前一個後自動指派。
多台交錯飛時 `[bt] 避碰：這次最接近鄰機 x m`，待命的會 `讓路給執行任務的鄰機`。

量測：每個指派印一行 `量測 …：建立→排第一、排第一→請求（assign_hold）…`。10/11 實測建立 → BT 執行約 0.7 s。

## 3. 火警與誘餌（itest U3、U4）

```bash
cbba_task.sh new 1 2 14 -6     # 火警方塊上方（tag 101）
cbba_task.sh new 1 3 -10 -8    # 誘餌（tag 7）
```

- 火警：到達後約 1～2 s，`[tag] tag 101：confidence 1.00` → `[bt] 火警成立：tag 101` → `廣播地面處置任務 0001xxxx`。
  無人機都不出價地面任務，沒開狗時會一直留著沒人接（正常）。
- 誘餌：`[tag]` 看得到 tag 7，但**不能**出現「火警成立」。

## 4. 失敗交回（itest C3）

```bash
cbba_task.sh fail 2 00010004   # 在執行那台的容器；reason 預設 2 TRANSIENT（這台 60 s 內不接），第 4 個參數可改
```

要看到：`任務 … 失敗（MANUAL），交回競標池，這台 60 s 內不接`（括號內是 detail；`cbba_task.sh fail` 送 `MANUAL`），別台接手（`指派給 BT：00010004`）。
reason 給 1（PERMANENT）時這台不再接。

## 5. 回報檢查與取消

```bash
cbba_task.sh watch 1                  # 看 uav1 目前的指派（current_task）
cbba_task.sh done 1 00010005          # 不是 uav1 目前 current_task 上的任務 → 「回報拒絕」，任務狀態不變
cbba_task.sh cancel 2 00010005        # 不是建立者（也不在 cancel_authorities）→ 拒絕
cbba_task.sh cancel 1 00010005        # 建立者 → 全隊「已取消」，執行中的那台放掉
```

## 6. 墜毀、斷線（itest U6、C4、C5）

| 情況 | 怎麼做 | 要看到 |
|---|---|---|
| **只關 PX4**（cbba_node 還開著） | 在 sim 停掉那台的 `px4_sitl.sh` | `[state] PX4 的位置沒有更新，停止送 robot_state` → 1.5 s 後 `[cbba] 停止參與出價` → `執行中的 … 撤銷`；別台以新版本接手。10/11 實測約 2.6 s |
| **整台斷線**（Ctrl+C 停掉 `cbba_uav.sh`） | | 還沒開始的任務約 1.5 s 後釋放、別台接手；**已經在執行的照樣鎖著**，別台印 `持有者 agent N 失聯：待確認，暫不重新指派`。失聯接手（火警 10 s、巡檢 30 s）已定案、還沒實作 |

## 7. 掉包（itest C7、C8）

每台 uav 都要設（tc 只影響送出方向）：

```bash
mesh_tc.sh set 30        # 掉包 30%，延遲 50±30 ms
mesh_tc.sh set 100       # 斷網
mesh_tc.sh clear
```

30% 時照樣收斂、不重複認領，但任務傳播約 2 s、完成確認約 2 s（補發的最短間隔 `resend_gap` 1 s）。

## 8. 巡檢＋隨機插入（patrol_site）

規劃與決定在 `docs/sim_scenario_plan.md`。場景：3 個綠色巡檢點（G1～G3，一開始就有）、5 個紅色插入點（R1～R5，
插入時 tag 才出現），座標在 `docker/gz/worlds/patrol_site_points.csv`。

| Terminal | 容器 | 指令 |
|---|---|---|
| 1～3 | `./enter.sh uavN` | `cbba_uav.sh`（各台要不同電量時：`SIM_BAT_DRAIN=600／300／200 cbba_uav.sh`） |
| 4 | `./enter.sh sim` | `CBBA_WORLD=patrol_site px4_sitl.sh 1`，之後 `px4_sitl.sh 2`、`3`（`SIM_BAT_DRAIN` 和同號的 uav 一樣） |
| 5 | `./enter.sh sim` | `gz_bridge.sh`（只測 CBBA 時可以不開） |
| 6 | `./enter.sh sim` | `patrol_scenario.sh`：輸入 `g` 建立綠色任務，之後輸入 `1`～`5` 插入紅色點 |

- 插入時 Gazebo 裡紅色方塊上出現 tag，`[cbba]` 印出新任務、指派；`[bt]` 印「前往」「到達」「回報」。
- 照離線模擬的同一次試驗重放：`patrol_scenario.sh replay <cbba_sim 輸出>/schedules/trial_007.csv`（含 `g`）。
- 下一次試驗前：`patrol_scenario.sh reset` 清掉紅色 tag，PX4、cbba_uav.sh 都要重開（電池回到 100%、任務清空）。
- 任務由 uav1 建立（`CREATOR=N` 可以改）；流水號存在 sim 容器的 `/tmp/cbba_scenario_seq_uavN`。
- PX4 的模擬電池**都從 100% 開始**，不能設初始電量；要「各台電量不同」只能用不同的 `SIM_BAT_DRAIN`（續航）。
  離線模擬要對照時用 `cbba_sim insert … --batteries 100,100,100 --endurance 600,300,200`。

## 已知限制

- **DDS 會經過 mesh**（`DDS_OVER_MESH` 的修正 patch 還沒套用）：PX4 的話題也會被 tc 影響、吃 mesh 頻寬。
  10/11 量到待命時 mesh 約 69 kB/s，大部分是 DDS。
- 沒有飛機接得起的任務（例如地面任務沒開狗、電量不夠）會一直留在競標池，沒有提示。
- 避碰（CPF）還沒測過刻意交叉的情境。要保守時可以分層高度：每台 `EXEC_ARGS="-p altitude:=5.0／7.0／9.0" cbba_uav.sh`。
- 沒有 stdin 時（例如 `docker exec -d`）PX4 的 shell 會一直重印 `pxh>`，log 暴增；要在背景跑時用 `sleep infinity | px4_sitl.sh N`。
