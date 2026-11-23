# 事件流水線統一：event_bridge → swarm_events（給伯宇，2026-11-23）

> 依據：伯宇 `bt_ws` 快照 `6273c0a`（2026-11-20，分支 `hetero-tasking-integration`）與 `px4_cbba_bt` 分支 `hetero-tasking`（2026-11-23，L5＋L6 C 批）的對照。
> 這份是**提案**，要兩邊對過才算定案；第 7 節是要一起決定的事。

## 0. 一句話

**狗的 BT（`task_manager`）、相機偵測（`intruder_watch.py`）、狗的底層、雲台、Nav2、警戒台畫面都照用你的；
事件判斷、派遣、任務規劃（`event_bridge` 做的 L5、L6）改由 `swarm_events` 的 `event_manager` 處理。**
兩邊之間只靠兩個介面：進去是 `Detection`，出來是 `/gcs/incidents`。

```
你的 intruder_watch（L1 相機）─ Detection ─►  /gcs/detections ─► event_manager（swarm_events：L2～L8）─ inject_task ─► 閘道 ─► CBBA
機器人的偵測（狗、無人機）─────── Detection ─► cbba_node ─► 閘道 ─┘            │
                                                                            ├─► /gcs/incidents（事件狀態）─► 你的警戒台
                                                                            └─► /gcs/operator_notices（值班通知）
閘道的 assignments（含角色進度）、task_reports、task_closed ─────────────────► event_manager（結案、誤報、降級）
狗的 BT（你的 task_manager）◄─ current_task（spec 的 steps）、teammates　　　　──► task_result、task_progress、incident_update
```

## 1. 為什麼統一、為什麼用 swarm_events

1. **分工調整**：11/23 起 2-1 的 L1～L8 由勝翔負責；你專心在狗的 BT（8 個動作、ExecuteSteps、讀 teammates、回報進度）。
   `event_bridge` 是這次調整之前、照當時的交接檔做的，裡面你定案的規則都會搬過去（第 5 節），不會不見。
2. **兩套同時跑會衝突**：同一場火會被兩邊各建一次事件、各注入一組任務，CBBA 會派兩批狗，兩邊還可能互相取消對方的任務。
3. **實機的網路**：`event_bridge` 直接訂每隻狗的 `step_status`、`current_task`、`task_result`，只有 GCS 和狗共用同一個 DDS 時收得到
   （模擬、實驗室）。實機上機器之間只走 CBBA 的 UDP（DDS 只在機內），GCS 看不到狗的話題，卡片會停在 DISPATCHED。
   `swarm_events` 只依賴閘道發的話題（閘道的 `assignments` 11/23 起帶角色進度），實機也收得到。
4. **2-1b 的範本是多角色、空地協同**（例如入侵：空中監看 optional＋前後包夾、會合、等 UAV 定位）。
   `event_bridge` 一個事件派一個角色、只派狗；`swarm_events` 已經支援角色組、precedence、sync、distinct、all-or-nothing、
   optional、降級子事件、改版只動變了的角色。
5. **版本**：`event_bridge` 讀 11/14 的舊註冊表格式（`actions: - name:`）；註冊表 11/16 已照 2-1b 改寫，換新版後它本來就要改。

## 2. 保留你的部分（照用）

| 部分 | 檔案 | 說明 |
|---|---|---|
| 狗的 BT | `src/task_manager`（樹、`step_format.hpp`、`task_manager_node.cpp`） | 讀 `current_task.spec` 的 steps、插隊、急停、平板接管、回報 `failure_code`／`findings_uri`、同 task_id 新 revision 只更新目標（`onTaskUpdate`）都照用。要補的在第 4 節 C |
| `event_bridge` 對應的 BT 行為 | `step_status`、卡片結束時被取消就停下剩下的步驟、回到被打斷的巡檢點 | 照用；`step_status` 另外轉一份成 `task_progress`（第 4 節 C1） |
| L1 相機 | `src/v60_sim/scripts/intruder_watch.py`、`watch_core.py` | YOLO、投影到地圖、禁區時段都照用；只改輸出（第 4 節 A1） |
| 狗的底層 | `ghost_mblink_bridge`、`ghost_bridge`、`ptz_node` | 不動（`cmd_timeout` 見第 4 節 D2） |
| 導航與模擬 | `legged_nav2` 包裝（`nav2_robot.launch.py`）、`v60_sim`（yard、8F）、`lab_dogs` | 不動（禁區見第 4 節 D1） |
| 警戒台、網頁控制台 | `task_manager/web/guard_server.py`、`guard.html`、`server.py` | 畫面照用；資料來源改成 `/gcs/incidents`（第 4 節 A2） |

## 3. 改由 swarm_events 處理的部分

| `event_bridge` 的功能 | `swarm_events` 的對應 |
|---|---|
| 卡片狀態機（CANDIDATE → … → ON_SITE → RESOLVED／FALSE_ALARM／ESCALATED） | `IncidentManager`：同樣的狀態，另外有 EXECUTING（必要角色開始執行）、MERGED（告警風暴）、EXPIRED；狀態依**有效進度**聚合（換人、改版後舊進度不算，會退回） |
| 高／中信心門檻、中信心派 1 隻確認 | θ_low 由 `event_types.yaml` 的 cost（漏報、誤報成本）算、θ_high 是 `dispatch.theta_high`；中信心派範本的 `fallback_template`（`single_dog_verify`） |
| 合併半徑、位置更新、任務改版 | L3 關聯（`associate.dt_s`、`r_m`、track_id）；位移 > 3 m 才重新展開，只改版變了的角色 |
| 範本展開（`rules.expand`） | `RuleExpander`＋`site-config/<場域>/event_rules.yaml`：多角色、觀察點／設備／區域位置、禁區 preflight、TaskSpec 檢查、機隊可行性、續航 |
| 沒人得標、沒有狗能做、過期 → ESCALATED | 閘道的 `task_orphaned`、`fallback_request` → ESCALATED，有 fallback 範本就派降級子事件 |
| 值班的通知（`/operator_alerts`） | `/gcs/operator_notices`（std_msgs/String） |
| 卡片結束時取消名下任務 | 誤報、拒絕時取消還開著的角色任務（`cancel_task`） |
| `alarms.jsonl`、`bridge_log.jsonl` | `<state_dir>/incidents.log`（每次轉換一行）、`event_manager_state.yaml`（重開沿用） |

## 4. 你那邊要改的

### A. 接上 swarm_events（約 1～2 天）

| # | 檔案 | 現在 | 改成 |
|---|---|---|---|
| A1 | `intruder_watch.py` | 軌跡成立後發 Event JSON 到 `/event_bridge/events` | 每一拍（算進去的框，`counted` = true）發 `swarm_interfaces/msg/Detection` 到 `/gcs/detections`（對照表見下）。累積、合併交給 `event_manager` 的 L2、L3；你的 L2、L3 可以留著給畫面用 |
| A2 | `guard_server.py`、`guard.html` | 訂 `/event_bridge/alarms`；按鈕發 `/event_bridge/verdicts`、`/close` | 訂 `/gcs/incidents`（`EventIncident`，transient_local）、`/gcs/operator_notices`；「假的」「結案」呼叫 `/gcs/resolve_incident`（`false_alarm` true／false）；「真的」等我們加（第 5 節 R3） |
| A3 | `demo/8f/host_up.sh`、`host_down.sh` | 起 `event_bridge_node` | 起 `ros2 run swarm_events event_manager --ros-args -r __ns:=/gcs -p registry_dir:=… -p site_dir:=…/site-config/8f -p state_dir:=…`（8F 場域設定由我們做，第 5 節 R6） |
| A4 | `e2e_watch_l5.sh`、`test_watch_core.py`、`replay_boxes.py` | 接 `event_bridge` | 改接 `event_manager`（或只留相機那段的測試） |
| A5 | `src/event_bridge` | — | 停用、不再擴充（情境檔可以改寫成 `Detection` 串流留著當測試資料） |

**A1 的對照**（你的 Event／框 → `Detection`）：

| `Detection` 欄位 | 填什麼 |
|---|---|
| `detector` | `event_types.yaml` 的偵測器名稱：人 `person_yolo_v1`、火和煙 `fire_smoke_yolo`（名稱要和註冊表一致，不認得的會被丟掉） |
| `label` | `person`、`fire`、`smoke` |
| `source` | `fixed_camera:<相機名>`（例如 `fixed_camera:cam4`） |
| `score` | 這一拍框的分數（0～1，不是你累積後的 s） |
| `track_id` | 你的軌跡編號 `t.k`（0 = 沒有） |
| `position_valid`、`position` | true、投影後的 map 座標 |
| `floor` | 樓層（0 = 不分） |
| `t` | 拍攝時刻 |
| `attrs` | JSON：證據參照，例如 `{"snapshot_uri": "file:///…/cam4_123.jpg"}`（會跟著任務送給狗、值班） |

注意：`event_manager` 要「分數 ≥ theta_on 持續 min_duration_s」才派遣、超過 1 s 沒有新偵測開始遺忘，所以要**持續送**（你 5 Hz 的每一拍都送就對了）。

### B. 換到最新版（全隊同一天）

| # | 項目 |
|---|---|
| B1 | 照 `docs/bt_cbba/UPSTREAM.md` 換 `swarm_interfaces`（2.12.0）、`cbba_core`、`swarm_cbba`、`registry/`，加 `swarm_events`；**協定 5 和 4 互相拒收**，無人機、閘道、狗要一起換 |
| B2 | 模擬的註冊表副本（`v60_sim/config/registry/`）照新格式重做；`event_types.yaml` 裡你加的欄位（`confidence_high`、`template`、`standoff_m`…）換成註冊表的欄位（第 7 節 Q3） |

### C. 狗的 BT 要補的（跟 2-1b 的協同有關）

| # | 項目 | 說明 |
|---|---|---|
| C1 | 發 `/<ns>/task_progress`（`TaskProgress`） | 可以由現在的 `step_status` 轉（下表）。沒有它，`teammates` 的 gate、會合、事件的 ON_SITE／EXECUTING 在真狗上都不會動 |
| C2 | 讀 `/<ns>/teammates`（`Teammates`） | 步驟有 `wait_for`、`sync` 時，gate 不是 OPEN 就停在那一步（回報 phase BLOCKED／ARRIVED_WAITING），OPEN 了再做 |
| C3 | 追蹤時發 `/<ns>/incident_update`（`IncidentUpdate`） | 目標移動、已定位時回報新位置（閘道驗證後全隊改版） |
| C4 | 動作參數名稱 | 見第 7 節 Q1 |

C1 的對照（`step_status` → `task_progress`）：

| `step_status` | `phase` | `milestones` |
|---|---|---|
| 第一步 `go_to` running | EN_ROUTE（2） | `en_route` |
| 第一步 `go_to` done | ARRIVED（3） | `arrived` |
| 第一個非移動的步驟 running | EXECUTING（4） | `started` |
| 停在 `wait_for` 沒滿足 | BLOCKED（5） | — |
| 到了 `sync` 那一步、等同伴 | ARRIVED_WAITING（6） | （cbba_node 自動加 `sync_reached`） |
| `inspect` done | EXECUTING（4） | `has_observation` |
| `track` 鎖定目標 | EXECUTING（4） | `target_locked`（定位了另外加 `target_localized`） |

`task_id` 要是目前 `current_task` 上的任務；`accepted`、`completed` 不用報（cbba_node 自己判斷）。訊息定義在 `swarm_interfaces/msg/TaskProgress.msg`。

### D. 實機安全

| # | 項目 | 說明 |
|---|---|---|
| D1 | Nav2 加禁區 | 現在 costmap 只有 static map＋STVL，**狗會穿過高壓區、儲槽區**（`zones.geojson` 只給相機用）。在 `nav2_robot.launch.py` 改寫參數時，global／local costmap 加 `nav2_costmap_2d::KeepoutFilter`，另外起 mask 的 `map_server` 與 `costmap_filter_info_server`；遮罩由 `site-config/<場域>/zones.geojson` 裡 `no_entry` 含 `quadruped` 的區域產生（轉換腳本我們寫，第 5 節 R7） |
| D2 | `ghost_mblink_bridge` 的 `cmd_timeout` | 現在 `-1.0`（關掉逾時，上游停發後狗會一直照最後一筆速度走）；你的 README 也寫真機要設正數（例如 0.3） |

## 5. 我們這邊要做的（你換過來之前先做好）

| # | 項目 | 來源 |
|---|---|---|
| R1 | 誤報後同一處冷卻（預設 5 分鐘）：期間的偵測只記錄、不派 | 你 11/18 定案的規則 |
| R2 | 同一個事件的狗到不了累計兩次（`E_UNREACHABLE`、`E_TIMEOUT`）→ ESCALATED、取消任務、不再派 | 同上 |
| R3 | 值班「判真」：ON_SITE 的事件按「真的」→ 留在現場、通知值班；可設定要不要加派完整範本（預設不加派） | 同上（`/gcs/resolve_incident` 加一種判定，或新服務） |
| R4 | ESCALATED 之後的通知限頻（預設 30 s 一則）；ESCALATED 有狗接手 → 回到 DISPATCHED | 同上 |
| R5 | 範本的站位距離（你的 `standoff_m` → go_to 的 `distance`）：照註冊表的參數名稱放進範本 | 你 11/20 加的 |
| R6 | `site-config/8f`：區域（你的禁區 `always`／`off_hours` 轉成我們的格式）、觀察點、`event_rules.yaml` | 你的 `config/8f_map/zones.geojson` |
| R7 | `zones.geojson` → Nav2 keepout 遮罩的轉換腳本；我們的假 Nav2（`fake_nav_body.py`）也照禁區繞路 | D1 |
| R8 | `Detection` 的轉換範例（從你的 Event JSON 轉），A1 改之前可以先用 | A1 |

## 6. 建議的順序與驗收

1. 我們先做 R1～R8。
2. 你在 8F 模擬裡做 A1～A3（先用 R8 的轉換也行），驗收：`run.sh fire`、`run.sh intruder` 的狗照樣到場、`/gcs/incidents` 的狀態走完 DISPATCHED → ON_SITE → EXECUTING → RESOLVED、警戒台顯示正確。
3. 約一天全隊一起做 B（協定 5），跑雙方的回歸（你的九套、我們的 itest）。
4. C1～C3（BT 協同），驗收：入侵範本的 front／rear 會合、等 UAV 定位後才開始。
5. D1、D2 上真機前做完。

## 7. 要一起決定的

| # | 問題 | 選項 | 建議 |
|---|---|---|---|
| Q1 | 動作參數名稱以誰為準 | A：BT 改成註冊表的（inspect `mode`／`duration_s`、track `duration_s`、go_to `standoff_m`…，照 2-1b 原文）；B：註冊表改成你 BT 的（`snapshots`、`timeout`、`duration`、`distance`） | A（2-1b 原文、無人機也用同一套）；你的 BT 可以兩種名稱都收一陣子 |
| Q2 | 固定攝影機的偵測發到哪 | A：直接發 `/gcs/detections`（攝影機在地面站，不經過機器人）；B：經過某個 cbba_node 送 DETECTION | A |
| Q3 | `event_types.yaml` 的門檻 | 你的 `confidence_high`／`confidence_low` 換成註冊表的 cost（漏報、誤報成本）＋`dispatch.theta_high` | 照註冊表；數值我們先照你的換算一版給你看 |
| Q4 | 場域 | 8F 和 yard 都做成 `site-config/<場域>`，兩邊在同一個場域測 | 兩個都做 |
| Q5 | `event_bridge` 的情境檔、影片 | 留著當紀錄；情境檔改寫成 `Detection` 串流當測試資料 | 看你要不要 |

## 參考

- `swarm_events`：`ros2_ws/src/swarm_events`（設計與決定 `docs/l5_l6_plan.md`）
- 訊息：`swarm_interfaces/msg/Detection.msg`、`EventIncident.msg`、`TaskProgress.msg`、`Teammates.msg`、`IncidentUpdate.msg`、`TaskAssignment.msg`；服務 `ResolveIncident.srv`
- 註冊表格式：`registry/README.md`、`registry/api.md`
- 給狗端的介面清單：`ros2_ws/src/swarm_cbba/README_v60.md` 第 0.1、0.2 節
