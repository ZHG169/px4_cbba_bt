# cbba_node＋PX4 SITL 測試步驟

> 2026-10-09 起 `cbba_uav.sh` 預設用備用 BT（`uav_px4_bt`＋`apriltag_fire_detector`，輸出前綴 `[bt]`、`[tag]`）；
> 下面 1～5 節是用 task_executor（`EXECUTOR=task_executor cbba_uav.sh`）寫的。巡檢＋隨機插入的測試見第 6 節。

原本的步驟用 `px4_waypoint_node/task_executor` 代替 BT。它會起飛，飛到 `assigned_task` 指定的位置，
停留 `duration_sec` 秒後對 `task_result` 發 `TaskResult`（`success: true`，帶回 assigned_task 的 `assignment_version`）。
PX4 的狀態由 `px4_state_bridge` 轉成 `robot_state` 給通用的 cbba_node。

```
sim 容器：Gazebo + PX4 #1、#2…
uavN 容器：XRCE Agent ─ task_executor（代替 BT）┐
                       └ px4_state_bridge ─ robot_state ─ cbba_node ─ UDP multicast ─ mesh_net ─ 其他 uav
```

## 0. 準備（只做一次）

```bash
# 主機：要測 A→B→C 時把 docker/.env 的 UAV_COUNT 改成 3，再執行
cd ~/CBBA_BT/docker && ./start.sh

# uav1：編譯（所有容器共用，不要兩個容器同時編譯）
colcon build --symlink-install --packages-select swarm_interfaces cbba_core px4_waypoint_node uav_px4_bt apriltag_fire_detector
# 2026-10-09 套件 uav_cbba 改名成 cbba_core：舊的 install/uav_cbba 不會自動刪掉，但 cbba_uav.sh 只用 cbba_core
```

## 1. 啟動（每台 Agent 要比 PX4 先開）

| Terminal | 容器 | 指令 |
|---|---|---|
| 1 | `./enter.sh uav1` | `cbba_uav.sh` |
| 2 | `./enter.sh uav2` | `cbba_uav.sh` |
| 3 | `./enter.sh sim` | `px4_sitl.sh 1`（開 Gazebo） |
| 4 | `./enter.sh sim` | `px4_sitl.sh 2` |
| 5 | `./enter.sh uav1` | 發任務用 |

`cbba_uav.sh` 會同時開 XRCE Agent、task_executor、px4_state_bridge、cbba_node。輸出前綴 `[exec]`、`[state]`、`[cbba]`。
節點名稱是 `uavN_task_executor`、`uavN_px4_state_bridge`、`uavN_cbba_node`（`ros2 node list` 看得到）。

**電量與能量模型**：`px4_sitl.sh` 把模擬電池設成續航 900 s、最低 20%（PX4 預設是 60 s 就降到 50%）；
`cbba_uav.sh` 用同一個續航算能量模型（懸停 0.111 %/s、每公尺 0.022 %），速度 5 m/s、巡檢高度 5 m
同時給 cbba_node 和 task_executor。要改就設環境變數 `SIM_BAT_DRAIN`、`SIM_BAT_MIN_PCT`、`CBBA_SPEED`、
`CBBA_ALTITUDE`（sim 和 uav 兩邊要一樣）。模擬電池只在解鎖時下降，上鎖後回到 100%。

**確認可以開始**：每台都要看到
- `[exec] 到達 5.0 m，等待指派`
- `[state] 開始送 robot_state`、`[state] 飛行狀態有效`（PX4 一開就送，地面上也送）
- `[cbba] 返航點 map (…)`（地面的出生點），起飛後 `[cbba] 開始參與出價`
- 別台的 `[cbba] 鄰居 N：參與 1、遙測 1、飛行狀態有效 1、armed 1、offboard 1、landed 0`（AGENT_STATE 的真實旗標）

`[cbba] 收斂：0 個任務，1 個鄰居` 的鄰居數應該是 N−1。

## 2. 基本分配

```bash
# terminal 5（uav1）：uav1 建立 3 個任務（map ENU，公尺）
cbba_task.sh new 1 1 10 0
cbba_task.sh new 1 2 0 12
cbba_task.sh new 1 3 -8 6
```

要看到：
- 每台的 `[cbba] 路徑：…` 合起來剛好涵蓋 3 個任務，不重複。
- `[cbba] 指派給 BT：…` 之後，`[exec] 前往 …`，Gazebo 裡的飛機開始移動。
- 停留結束後依序出現 `[exec] 回報 … 完成` → `[cbba] 任務 … 完成，等待全隊確認` →
  `[cbba] 任務 … 已確認完成（回報後 x.xx s）`，接著自動指派下一個任務。
- **其他每一台**都要出現 `[cbba] 任務 … 已確認完成（收到 TASK_CLOSE）`，表示全隊都把它標成 DONE。

座標要在場地內；出生點是 (0, 0)、(0, 2)、(0, 4)…（`UAV_SPAWN_SPACING`）。

## 3. 失敗交回

飛行途中在**執行那台**的容器回報失敗：

```bash
cbba_task.sh fail 2 00010002      # task_id 用 [cbba] 印出來的 8 位十六進位；版本自動讀 assigned_task
```

要看到：這台出現 `任務 … 失敗，交回競標池`，別台在 1 s 內把這個任務排進自己的路徑，正式指派時版本更大
（`指派給 BT：00010002 v3`）。任務保留（不是取消），這台之後不會再接這個任務。

## 3b. 回報檢查與取消

```bash
cbba_task.sh watch 1                  # 看 uav1 目前的指派與版本
cbba_task.sh done 1 00010002 1        # 指定舊的版本 → uav1 的 [cbba] 出現「回報拒絕」，任務狀態不變
cbba_task.sh cancel 2 00010003        # uav2 不是建立者（也不在 cancel_authorities）→「沒有權限」
cbba_task.sh cancel 1 00010003        # 建立者 → 全隊「已取消」，正在執行的那台放掉這個任務
```

## 4. 墜毀（整台消失）

在執行中那台的 terminal 1／2 按 Ctrl+C（停掉 cbba_node、px4_state_bridge 和 task_executor，PX4 會進 failsafe）。

要看到：約 1.5 s 後，別台的 `收斂` 鄰居數減 1，它的任務被重新分配。

## 4b. 只關 PX4（cbba_node 還開著）

在 sim 停掉那台的 PX4（`px4_sitl.sh N` 那個 terminal 按 Ctrl+C）。

要看到：`[state] PX4 的位置沒有更新，停止送 robot_state` → 約 1.5 s 後 `[cbba] 停止參與出價，釋放手上的任務`，別台接手它的任務。（2026-10-09 改成通用節點之前，這種情況 cbba_node 會繼續用最後的位置出價。）

## 5. 加掉包

每台 uav 都要設，因為 tc 只影響送出方向：

```bash
mesh_tc.sh set 30        # 掉包 30%，延遲 50±30 ms
mesh_tc.sh clear
```

重做步驟 2～4，比較收斂時間。要做 A→B→C 鏈狀拓撲，還需要依來源 IP 擋封包的腳本（還沒寫）。

## 6. 巡檢＋隨機插入（patrol_site，2026-10-10）

規劃與決定在 `docs/sim_scenario_plan.md`。場景：3 個綠色巡檢點（G1～G3，一開始就有）、5 個紅色插入點（R1～R5，
插入時 tag 才出現），座標在 `docker/gz/worlds/patrol_site_points.csv`。

| Terminal | 容器 | 指令 |
|---|---|---|
| 1～3 | `./enter.sh uavN` | `cbba_uav.sh`（各台要不同電量時：`SIM_BAT_DRAIN=600／300／200 cbba_uav.sh`） |
| 4 | `./enter.sh sim` | `CBBA_WORLD=patrol_site px4_sitl.sh 1`（之後的台 `px4_sitl.sh 2`、`3`，`SIM_BAT_DRAIN` 和同號的 uav 一樣） |
| 5 | `./enter.sh sim` | `gz_bridge.sh`（下視相機；只測 CBBA 時可以不開） |
| 6 | `./enter.sh sim` | `patrol_scenario.sh`：輸入 `g` 建立綠色任務，之後輸入 `1`～`5` 插入紅色點 |

- 插入時 Gazebo 裡紅色方塊上出現 tag，`[cbba]` 印出新任務、指派；`[bt]` 印「前往」「到達」「回報」。
- 照離線模擬的同一次試驗重放：`patrol_scenario.sh replay <cbba_sim 輸出>/schedules/trial_007.csv`（含 `g`）。
- 下一次試驗前：`patrol_scenario.sh reset` 清掉紅色 tag，PX4、cbba_uav.sh 都要重開（電池回到 100%、任務清空）。
- 任務由 uav1 建立（`CREATOR=N` 可以改）；流水號存在 sim 容器的 `/tmp/cbba_scenario_seq_uavN`。
- PX4 的模擬電池**都從 100% 開始**，不能設初始電量；要「各台電量不同」只能用不同的 `SIM_BAT_DRAIN`（續航）。
  離線模擬要對照時用 `cbba_sim insert … --batteries 100,100,100 --endurance 600,300,200`。

## 已知限制

- **Fast DDS 仍會經過 mesh**：uav 容器的 `LOCAL_IPS` 含 mesh IP，ROS 話題（含 PX4）也會被 tc 影響，
  而且會吃 mesh 頻寬。CBBA 本身走 UDP 不受影響，但掉包測試的結果會受干擾。要改 `gen_compose.py`。
- 沒有飛機接得起的任務（例如電量不夠）會一直留在競標池，目前沒有任何提示。
- **task_executor 沒有避碰，SITL 會撞機**：同高度（5 m）直線飛，出生點只隔 2 m，從出生點往旁邊飛會經過鄰機。
  避碰之後在飛行層加（CPF 之類），和 CBBA 無關。測試時可以暫時分層高度：
  每台 `EXEC_ARGS="-p altitude:=5.0／7.0／9.0" cbba_uav.sh`，並把 `.env` 的 `UAV_SPAWN_SPACING` 拉開到 5。
