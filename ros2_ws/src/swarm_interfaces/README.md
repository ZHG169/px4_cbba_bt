# swarm_interfaces — 空地 CBBA 介面規格 v1.0

> 狀態：**勝翔定案（2026-10-03）**，待伯宇、宜臻審閱。
> 需要修改請直接在這份文件標註，修改後版本號往上加（v1.1…），並同步改 `package.xml` 的 version。

## 1. 載具編號與命名空間

| 載具 | agent_id（訊息裡的 sender_id / winner_id） | 自己的話題命名空間 | 備註 |
|---|---|---|---|
| 無人機 N（N = 1~9） | N | `/uavN/...`，PX4 話題為 `/uavN/fmu/...` | 支援多台，所以用編號，不用 `/uav` |
| Ghost V60 機器狗 | 50 | `/v60/...` | 之後若有多隻，依序 51、52… |
| 地面站 | —（不參與出價） | — | 只發任務、監控 |

- `agent_id = 0` 保留給「無人得標」。
- 群體共用的話題放在 `/swarm/` 下，不加載具命名空間。

## 2. 話題

| 話題 | 型別 | 誰發布 | 誰訂閱 | QoS | 發布時機 |
|---|---|---|---|---|---|
| `/swarm/tasks` | `swarm_interfaces/msg/Task` | 建立任務者（地面站、無人機的 `ReportFireEvent`）；完成或取消任務的載具 | 所有載具的 CBBA 節點 | **reliable、transient_local、keep_last 100** | 建立時一次；狀態改變（DONE / CANCELLED）時一次 |
| `/swarm/cbba` | `swarm_interfaces/msg/CBBAMessage` | 每台載具的 CBBA 節點 | 所有載具的 CBBA 節點 | **best_effort、volatile、keep_last 10** | 自己的出價表改變時立即送，另外每 **200 ms** 重送 |

- `/swarm/tasks` 用 transient_local，晚啟動的節點也拿得到已發布的任務（核心收到未知任務的出價會直接略過，所以任務必須比出價先到）。
- `/swarm/cbba` 丟了不重傳，靠 200 ms 週期重送補上；弱網測試（tc）量的就是這一條。
- 收斂判定的時限是 **5 秒**（和計畫第 5 週的指標一致）。

## 3. 訊息

### Task.msg（`/swarm/tasks`）

| 欄位 | 型別 | 說明 |
|---|---|---|
| `task_id` | uint32 | **建立者 agent_id × 65536 + 流水號**，各載具自己編，不會撞號 |
| `type` | uint8 | `AIR_RECON=1`、`GROUND_INTERVENTION=2`、`PATROL=3` |
| `position` | geometry_msgs/Point | **map 座標系（ENU），公尺**，見第 4 節 |
| `created` | Time | 任務發現或建立的時間 |
| `deadline_sec` | float32 | 期限，從 `created` 起算（軟限制：逾期分數降低但不歸零） |
| `value` | float32 | 任務價值 |
| `duration_sec` | float32 | 預估的現場執行時間 |
| `status` | uint8 | `OPEN=0`、`DONE=1`、`CANCELLED=2` |
| `status_stamp` | Time | 狀態最後一次改變的時間 |

**相容表**（不相容的組合出價固定為 0）：

| | AIR_RECON | GROUND_INTERVENTION | PATROL |
|---|---|---|---|
| 無人機 | ✅ | 0 | 0 |
| V60 | 0 | ✅ | ✅ |

**火警任務**：無人機確認 AprilTag 火情後，由 `ReportFireEvent` 發布

```
task_id = 自己的 agent_id × 65536 + 流水號
type = GROUND_INTERVENTION      position = 火點的 map 座標
deadline_sec = 120   value = 100   duration_sec = 20   status = OPEN
```

（數值沿用 `uav_cbba/scenarios/fire_demo.csv` 的地面處置任務，之後可依實測調整。）

### CBBAMessage.msg（`/swarm/cbba`）

| 欄位 | 型別 | 說明 |
|---|---|---|
| `header.stamp` | Time | 送出時間 |
| `sender_id` | uint8 | 傳送者 agent_id |
| `sender_type` | uint8 | `UAV=1`、`UGV=2` |
| `seq` | uint32 | 傳送者自己的流水號，用來丟掉舊訊息 |
| `bids[]` | Bid | 每個 OPEN 任務一筆 |
| `stamps[]` | AgentStamp | 傳送者以外、每台已知載具一筆 |

- `Bid`：`task_id`、`score`（float32，傳送者知道的最高分 y）、`winner_id`（uint8，得標者 z，0 = 無人）。
- `AgentStamp`：`agent_id`、`stamp`（傳送者最後一次取得該載具資訊的時間）。

## 4. 座標系

- 所有 swarm 訊息的位置一律用 **map 座標系：ENU，公尺**。模擬時 map = Gazebo 世界座標。
- PX4 的 local position 是 **NED，而且以各自的出生點為原點**。無人機端負責轉換：
  `map_E = 出生點_E + local_y`、`map_N = 出生點_N + local_x`、`map_U = 出生點_U − local_z`。
- V60 使用 Nav2 的 `map`（ENU），只要和 Gazebo 世界座標對齊即可。

## 5. 時間

- 使用系統時鐘（`use_sim_time = false`）。模擬時所有容器在同一台主機上，時鐘一致。
- 實機必須用 chrony／NTP 對時：`created + deadline_sec` 是絕對時間，`stamps` 也要跨機器比較。

## 6. CBBA 共識規則

- 衝突消解採 **Choi 2009 Table 1 的 17 條規則**，實作與測試案例在 `uav_cbba`（`test/cbba_rule_cases.csv`）。
- 計畫中寫的「18 條」視為筆誤；如果有第 18 條（例如任務終止的處理），請宜臻提出規則內容，再加進案例檔。
- 共識核心（bundle、消解規則、連鎖退標、出價上限）**兩邊共用同一份程式庫**，各自只提供自己的出價函數。

## 7. 待伯宇、宜臻確認（不在本規格內定案）

| 項目 | 說明 |
|---|---|
| CBBA 結果寫入黑板的欄位 | `robot_blackboard.hpp` 的 `current_task` 等欄位要放哪些 CBBA 結果（任務 ID、順序、得標分數…） |
| 確認得標才寫入黑板的規則 | 暫時得標就寫入會造成反覆中斷；建議「連續 N 個重送週期（200 ms）都沒被搶走」才寫入 |
| Phase 1 的 300 ms 預算分配 | CBBA 協商、寫入黑板、50 ms 哨兵、Nav2 各占多少 |
| V60 的出價函數 | 地面路徑長度（Nav2）與電量模型，由伯宇提供 |

## 8. 版本紀錄

| 版本 | 日期 | 內容 |
|---|---|---|
| v0.1 | 2026-10-02 | 草稿：四個訊息定義 |
| v1.0 | 2026-10-03 | 勝翔定案：命名空間、agent_id、QoS、座標系、時間、火警任務格式、17 條規則 |
