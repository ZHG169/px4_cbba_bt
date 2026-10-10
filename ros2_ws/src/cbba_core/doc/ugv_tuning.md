# 機器狗（V60）的 CBBA 參數調整

通用 `cbba_node`（`vehicle_type:=ugv`）· 2026-10-07，2026-10-09 改成通用節點

狗的出價會用到的參數，以及怎麼量、怎麼改。**目前的能量模型是佔位值，還沒校正**，請伯宇用狗的實測資料換掉。
和無人機共用的參數（協定、出價公式）的完整說明見 [`parameters.md`](parameters.md)。

---

## 先講結論

| 參數 | 現在的值 | 要做什麼 |
|---|---|---|
| `energy_per_meter` | 0.1 %/m（佔位） | 量：走 1 m 掉多少電量 |
| `hover_energy_per_sec` | 0.05 %/s（佔位） | 量：在現場處置時（站著、PTZ 運作）每秒掉多少電量 |
| `cruise_speed` | 1.0 m/s（佔位） | 量：Nav2 實際的平均移動速度 |
| `safety_reserve` | 20 % | 依狗的低電量行為決定（低於多少會強制返航或趴下） |
| 其他 | 和無人機一樣 | `udp_group`、`udp_port`、`index_slots`、`lost_timeout` **全隊要一樣**，不要改 |

改的方法：

```bash
ros2 run cbba_core cbba_node --ros-args -r __node:=v60_cbba_node -p vehicle_type:=ugv -p agent_id:=50 \
  -p energy_per_meter:=0.08 -p hover_energy_per_sec:=0.03 -p cruise_speed:=1.2 -p safety_reserve:=25.0
```

---

## 一、出價怎麼用到這些參數

狗收到地面處置任務（無人機確認火情後建立）時，計算「把這個任務排進自己的路徑，要多花多少成本」：

```
飛（走）過去   距離 = 直線距離（目前沒有用 Nav2 的路徑長度）
到達時間       = 距離 / (cruise_speed × 0.8)
時空成本       = 距離 × 到達時間 / 剩餘期限
耗電           = 距離 × energy_per_meter + 現場時間 × hover_energy_per_sec + 返航距離 × energy_per_meter
可不可行       電量 − 耗電 ≥ safety_reserve，否則不出價
分數           = value / (1 + 邊際成本 / cost_ref)
```

**例子**（現在的佔位值、電量 90%）：狗在 (5, 20)，火點在 (6, 15)，火警任務停留 20 s、期限 120 s

| 項目 | 計算 | 結果 |
|---|---|---|
| 走過去 | 5.1 m × 0.1 | 0.5 % |
| 現場處置 | 20 s × 0.05 | 1.0 % |
| 返航 | 5.1 m × 0.1 | 0.5 % |
| 合計 | | 2.0 %，可用 70%，可行 |
| 到達時間 | 5.1 / 0.8 | 6.4 s，期限 120 s 內 |

---

## 二、每個參數

### `energy_per_meter`（每公尺耗電，%）
- **意義**：移動 1 m 掉多少電量（百分比）。
- **怎麼量**：滿電後在平地直線走一段固定距離（例如 100 m 來回），記錄前後的電量：
  `(開始電量 − 結束電量) / 總距離`。多量幾次取平均；坡地、草地另外量。
- **太小**：狗會接走不完的任務，半路沒電。**太大**：明明走得到卻不接。

### `hover_energy_per_sec`（現場每秒耗電，%）
- **意義**：到了現場執行任務（站著、雲台、灑水等）時每秒掉多少電量。名稱沿用無人機的「懸停」。
- **怎麼量**：原地執行一次典型的處置動作，記錄時間和電量：`電量差 / 秒數`。
- 地面處置任務的現場時間是任務的 `duration_sec`（火警任務預設 20 s）。

### `cruise_speed`（m/s）
- **意義**：估到達時間用，實際用的是 `cruise_speed × 0.8`（預留 20% 的餘裕）。
- **怎麼量**：Nav2 走一段路徑的 `路徑長度 / 實際花的時間`。
- **太大**：以為趕得上期限，其實會遲到。**太小**：成本被高估，可能讓給更遠的載具（目前只有一隻狗，影響不大）。

### `safety_reserve`（%）
- **意義**：做完任務並走回出發點後，至少要剩這麼多電量。
- **怎麼決定**：狗的 BT 在低於多少電量時會強制返航或停機，再多留 5% 給估計誤差。

### 不用改、但要知道的

| 參數 | 值 | 說明 |
|---|---|---|
| `vehicle_type` | `ugv` | 只接地面任務；上面四個能量參數不指定時用狗的預設值 |
| `agent_id` | 50 | 介面規格的機號。無人機完成任務時也會等狗的確認（協定版本 2 起） |
| `ns` | `v60` | 話題前綴：`/v60/robot_state`、`/v60/task_result`、`/v60/assigned_task` |
| `state_timeout` | 1.5 s | `robot_state` 這麼久沒更新就停止參與出價、釋放手上的任務。robot_state 是 2 Hz，等於連掉 3 則 |
| `assign_hold` | 0.6 s | 同一個任務連續排第一這麼久，才發到 `/v60/assigned_task`，避免協商途中來回換 |
| `battery_weight`、`cost_ref`、`max_bundle` | 1.0、50、5 | 出價公式。現在只有一隻狗，誰得標不受影響；之後多隻狗時再一起調 |

---

## 三、火警任務本身的參數（由無人機決定）

無人機確認火情後建立任務，狗不能改這些值，但它們會影響狗的出價：

| 欄位 | 預設（介面規格 v1.0） | 說明 |
|---|---|---|
| `value` | 100 | 分數的上限 |
| `deadline_sec` | 120 s | 從建立起算。走過去的時間要明顯小於它，否則成本很高 |
| `duration_sec` | 20 s | 狗在現場處置的時間，會算進耗電 |

如果狗的處置實際要更久，請告訴無人機端改 `duration_sec`，耗電才估得準。

---

## 四、和狗的 BT 的介面

| 話題 | 型別 | 方向 | 說明 |
|---|---|---|---|
| `/v60/robot_state` | `swarm_interfaces/RobotState` | BT → CBBA | 2 Hz。`agent_id` 要是 50，位置是 map ENU，電量 0～100 |
| `/v60/task_result` | `swarm_interfaces/TaskResult` | BT → CBBA | 任務結束時一次，**請用 reliable 發**，`assignment_version` 原樣帶回 assigned_task 上的版本。`success = false` 時交回競標池、狗不再接；`detail` 記在 log |
| `/v60/assigned_task` | `swarm_interfaces/Task` | CBBA → BT | reliable、transient_local。目前要執行的任務；沒有任務時 `task_id = 0`、`status = CANCELLED` |

狗也能建立任務（`/v60/new_task`，協定版本 2 起）。ManualOverride（平板 → BT）只在狗的機內。

2026-10-09 起：BT 要送 `/v60/exec_state`、回覆 `/v60/assignment_request`（接受後才發 `assigned_task`），
地面端保有最終的接受與中斷權。細節見 [`ugv_test.md`](ugv_test.md) 與 [`protocol.md`](protocol.md) 第五節。

---

## 五、已知限制

1. **距離用直線**：還沒有用 Nav2 的路徑長度（介面規格寫的是由伯宇提供）。有障礙物時，成本和耗電都會低估。
2. **只有一隻狗時**：狗失聯或回報失敗，地面任務會留在競標池，沒有人接，目前也沒有提示。
3. **能量模型是線性的**：坡度、地形、負載都沒有考慮。
