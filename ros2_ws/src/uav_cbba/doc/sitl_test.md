# cbba_node＋PX4 SITL 測試步驟

BT 還沒好，先用 `px4_waypoint_node/task_executor` 代替。它會起飛，飛到 `assigned_task` 指定的位置，
停留 `duration_sec` 秒後發 DONE。

```
sim 容器：Gazebo + PX4 #1、#2…
uavN 容器：XRCE Agent ─ task_executor（代替 BT）─ cbba_node ─ UDP multicast ─ mesh_net ─ 其他 uav
```

## 0. 準備（只做一次）

```bash
# 主機：要測 A→B→C 時把 docker/.env 的 UAV_COUNT 改成 3，再執行
cd ~/CBBA_BT/docker && ./start.sh

# uav1：編譯（所有容器共用，不要兩個容器同時編譯）
cbuild --packages-select swarm_interfaces uav_cbba px4_waypoint_node
```

## 1. 啟動（每台 Agent 要比 PX4 先開）

| Terminal | 容器 | 指令 |
|---|---|---|
| 1 | `./enter.sh uav1` | `cbba_uav.sh` |
| 2 | `./enter.sh uav2` | `cbba_uav.sh` |
| 3 | `./enter.sh sim` | `px4_sitl.sh 1`（開 Gazebo） |
| 4 | `./enter.sh sim` | `px4_sitl.sh 2` |
| 5 | `./enter.sh uav1` | 發任務用 |

`cbba_uav.sh` 會同時開 XRCE Agent、task_executor、cbba_node。輸出前綴 `[exec]`、`[cbba]`。

**電量與能量模型**：`px4_sitl.sh` 把模擬電池設成續航 900 s、最低 20%（PX4 預設是 60 s 就降到 50%）；
`cbba_uav.sh` 用同一個續航算能量模型（懸停 0.111 %/s、每公尺 0.022 %），速度 5 m/s、巡檢高度 5 m
同時給 cbba_node 和 task_executor。要改就設環境變數 `SIM_BAT_DRAIN`、`SIM_BAT_MIN_PCT`、`CBBA_SPEED`、
`CBBA_ALTITUDE`（sim 和 uav 兩邊要一樣）。模擬電池只在解鎖時下降，上鎖後回到 100%。

**確認可以開始**：每台都要看到
- `[exec] 到達 5.0 m，等待指派`
- `[cbba] 開始參與出價`

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
- **其他每一台**都要出現 `[cbba] 任務 … 已確認完成（收到完成證明）`，表示全隊都把它標成 DONE。

座標要在場地內；出生點是 (0, 0)、(0, 2)、(0, 4)…（`UAV_SPAWN_SPACING`）。

## 3. 失敗交回

飛行途中在**執行那台**的容器回報失敗：

```bash
cbba_task.sh fail 2 00010002      # task_id 用 [cbba] 印出來的 8 位十六進位
```

要看到：這台出現 `任務 … 失敗，交回競標池`，別台在 1 s 內把這個任務排進自己的路徑。這台之後不會再接這個任務。

## 4. 墜毀（整台消失）

在執行中那台的 terminal 1／2 按 Ctrl+C（停掉 cbba_node 和 task_executor，PX4 會進 failsafe）。

要看到：約 1.5 s 後，別台的 `收斂` 鄰居數減 1，它的任務被重新分配。

> 只關 PX4、cbba_node 還開著的話，cbba_node 會繼續用最後的位置出價（見下面「已知限制」）。

## 5. 加掉包

每台 uav 都要設，因為 tc 只影響送出方向：

```bash
mesh_tc.sh set 30        # 掉包 30%，延遲 50±30 ms
mesh_tc.sh clear
```

重做步驟 2～4，比較收斂時間。要做 A→B→C 鏈狀拓撲，還需要依來源 IP 擋封包的腳本（還沒寫）。

## 已知限制

- **Fast DDS 仍會經過 mesh**：uav 容器的 `LOCAL_IPS` 含 mesh IP，ROS 話題（含 PX4）也會被 tc 影響，
  而且會吃 mesh 頻寬。CBBA 本身走 UDP 不受影響，但掉包測試的結果會受干擾。要改 `gen_compose.py`。
- **cbba_node 沒檢查遙測是否新鮮**：只看 armed、landed，PX4 斷掉時這兩個值會停在最後一次的狀態，
  繼續參與出價。
- 沒有飛機接得起的任務（例如電量不夠）會一直留在競標池，目前沒有任何提示。
- **task_executor 沒有避碰，SITL 會撞機**：同高度（5 m）直線飛，出生點只隔 2 m，從出生點往旁邊飛會經過鄰機。
  避碰之後在飛行層加（CPF 之類），和 CBBA 無關。測試時可以暫時分層高度：
  每台 `EXEC_ARGS="-p altitude:=5.0／7.0／9.0" cbba_uav.sh`，並把 `.env` 的 `UAV_SPAWN_SPACING` 拉開到 5。
