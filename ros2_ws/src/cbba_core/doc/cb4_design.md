# CB-4 協同：設計與分批（2026-11-16；第一批已實作）

一件事要多台時，閘道（或外部 task manager）把它拆成角色：同一個 `group_id`，每個角色是一個 CBBA 任務。
M0 已經有：角色組的注入（`inject_task` 的 `role_index`／`role_count`／`optional`／`all_or_nothing`／`skip_satisfies`／
`precedence`／`sync_window_s`）、TASK_ANNOUNCE 轉送這些欄位、各角色各自競標、optional 角色自動關閉（4e）。
**CB-4 要讓這些約束真的有作用**。原則（使用者 11/15、2-1b 第八節）：

- **CBBA 照現在各自分配**（出價、17 條規則、TDR 都不動）；協同在**執行端**做。
- precedence 照原文：後面的角色**可以先出發**，只有「要等的那一步」等前面的角色到某個里程碑；前面的角色是 optional、沒有也不阻塞。
- 判斷寫在 cbba_node（狗和無人機的 BT 共用同一套判斷），BT 只看「這一步可不可以開始」。

使用者 11/16 定的三批：

| 批 | 內容 | 驗收 |
|---|---|---|
| 1 最小可驗證 precedence | ① TASK_PROGRESS 封包 ② `task_progress` 話題（BT → CBBA）③ `teammates` 話題（CBBA → BT）④ 每條 precedence 的 milestone ⑤ step-level gate ⑥ 進度快照與補發 ⑦ 假 BT 測試 | UAV 回報 `target_localized` 後，狗的 gate 解除等待 |
| 2 同步與整組提交 | ⑧ sync_window ⑨ `ARRIVED_WAITING` ⑩ all-or-nothing 暫定指派 ⑪ group commit／timeout | 同步的角色到齊才一起開始；湊不齊不鎖定任何一台 |
| 3 閘道管理 | ⑫ `IncidentUpdate` ⑬ 同組 revision 更新 ⑭ 連鎖撤銷 ⑮ `FallbackRequest` | 無人機更新目標位置 → 狗的目標跟著更新；必要角色失敗 → 依範本撤銷或請求 fallback |

---

## 第一批：最小可驗證 precedence

### 1.1 里程碑（milestone）④

角色執行中會達成的事件，用固定的位元表示（**只增不改**，和能力的 bit 一樣；名稱與 bit 寫在註冊表 `actions.yaml` 的
`milestones`，程式 `cbba_core/milestones.hpp` 要一致，`test_registry` 檢查）：

| bit | 名稱 | 誰回報、什麼時候 | 換手後保留（sticky） |
|---|---|---|---|
| 0 | `accepted` | cbba_node：正式指派、發出 current_task | 否 |
| 1 | `en_route` | BT：開始前往 | 否 |
| 2 | `arrived` | BT：到達該角色第一個 go_to 的位置 | 否 |
| 3 | `started` | BT：開始第一個非移動的 step | 否 |
| 4 | `target_localized` | BT：已定位目標（例如 UAV 看到入侵者、更新了位置） | **是** |
| 5 | `target_locked` | BT：追蹤已鎖定 | 否 |
| 6 | `has_observation` | BT：已取得觀測 | **是** |
| 7 | `completed` | cbba_node：任務完成（TASK_CLOSE reason 1）——視為**所有里程碑都達成** | **是** |

- **sticky**：持有者換了（失聯接手、失敗後重新分配）時，新的指派版本從 0 開始，但 sticky 的里程碑保留（目標已經定位過，
  不用因為換人再等一次）；非 sticky 的（到位、鎖定）跟著執行者。
- BT 回報用名稱（字串），cbba_node 轉成 bit；不認得的名稱印警告、忽略。

### 1.2 每條 precedence 的 milestone ＋ step-level gate ④⑤

A 層的 `precedence`（8 位元遮罩）只說「要等哪幾個角色」；**等到哪個里程碑、在哪一步等**寫在那個角色的 TaskSpec（B 層）裡，
不改 TASK_ANNOUNCE 的格式：

```json
{"steps": [
  {"action": "go_to", "location": {"poi": "OBS_HV_A_FRONT"}},
  {"action": "inspect", "object": "person", "params": {"mode": "rgb"},
   "wait_for": [{"role_index": 0, "milestone": "target_localized"}]},
  {"action": "speak", "params": {"template": "intrusion_warning"}}
]}
```

- `wait_for` 放在**要等的那一步**：前面的步驟照常做（狗先出發、到觀察點），到了這一步才等。這就是 step-level gate。
- 一步可以等好幾個角色（全部滿足才開）。沒寫 `wait_for`、但 A 層 precedence 有某個角色時：等那個角色的 `completed`、
  在第一步等（相容的預設，和原文「A 先、B 後」一致）。
- 檢查：`wait_for` 的 role_index 要在自己的 precedence 遮罩裡、milestone 要認得；不一致時 cbba_node 印警告、照 `wait_for` 做。
  範本（`event_rules.yaml` 的 `coupling.precedence`）由展開器轉成 A 層遮罩＋B 層 `wait_for`。
- cbba_node 解析 spec 的 JSON：用已經連結的 yaml-cpp（JSON 是 YAML 的子集），不加新依賴。

**gate 的狀態**（每個有 `wait_for` 的步驟一個）：

| 狀態 | 條件 |
|---|---|
| `OPEN` | 每個 `wait_for` 都滿足：前面的角色（最新的指派版本）達成該里程碑；或有 sticky 的達成紀錄；或已完成（completed）；或它是 optional、被略過（TASK_CLOSE reason 4）而且它的 `skip_satisfies` = true |
| `WAITING` | 還沒滿足，前面的角色還開著（分配中、執行中、重新競標中） |
| `UNSATISFIABLE` | 前面的角色已經結束、但不會滿足：取消（reason 2）、過期（reason 3）、略過但 `skip_satisfies` = false。第一批只回報、不處理（BT 停在這一步）；第三批由閘道撤銷或 fallback |

gate 是**本機計算**：每台 cbba_node 都看得到同組角色的任務狀態（CBBA_STATE、TASK_CLOSE）與進度（TASK_PROGRESS），
只算自己手上那個角色的 gate，發在 `teammates`。

### 1.3 TASK_PROGRESS 封包 ①

協定版本 **4 不變**，加種類 12（`PacketType::TASK_PROGRESS`，轉送）。沒升級的節點在 `parseHeader` 就丟掉（種類超出範圍），
不會誤解；但沒升級的節點不轉送進度，所以全隊仍要一起重新編譯（和 M0 一樣）。

```
表頭 20 | 轉送區塊 14（origin, origin_session, origin_seq）
| task_id 8 | revision 2 | assign_version 8 | executor 2 | progress_seq 2
| phase 1 | step_index 1 | milestones 2 | flags 1
= 61 B
```

| 欄位 | 說明 |
|---|---|
| executor | 回報的執行者（= origin）；收的那端要是這個任務、這個 assign_version 的正式執行者才接受（和完成回報同一套檢查：版本較舊拒絕） |
| assign_version | 正式指派版本；新的版本（換手）時里程碑重算，sticky 的保留 |
| progress_seq | 同一個 (task, assign_version) 內遞增；舊的（較小）丟掉 |
| phase | 0 UNKNOWN、1 ACCEPTED、2 EN_ROUTE、3 ARRIVED、4 EXECUTING、5 BLOCKED（停在 gate）、6 ARRIVED_WAITING（第二批）、7 DONE |
| step_index | 正在做第幾步（0 起）；255 = 不知道 |
| milestones | 這個指派版本已達成的里程碑（位元 OR，只增不減） |
| flags | bit 0：sticky 的里程碑是之前的指派版本留下的（快照時用） |

- 送出：BT 回報有變化時馬上送，之後 0.2 s、0.6 s 再送兩次（掉包）；執行中每 `progress_period`（2 s）重送目前的狀態。
  每次都換新序號（照協定的「重送一律換新序號」）。
- 收：同一個 (task_id, assign_version) 只留 progress_seq 最大的；里程碑和本機已知的 OR 起來。

### 1.4 進度快照與補發 ⑥

進度是**狀態**、不是事件，最新的一份就夠：

- **定期重送**（上一節）涵蓋大部分：執行者在線時，晚加入的節點 2 s 內就知道。
- **快照**：執行者已經離線（失聯、做完）、別台才加入時，定期重送收不到。所以每台記著同組任務的最新進度，
  **發現新鄰居（新的 session）時**，把自己知道的「還開著的角色組任務」的進度用 SYNC_BATCH 送一次（SYNC_BATCH 的 type 加 12；
  限速，和現有補發共用 `repair_rate`）。sticky 的里程碑標 flags bit 0。
- 結束的任務不用補進度：TASK_CLOSE 已經有補發，completed 就是全部里程碑。

### 1.5 ROS 話題 ②③

**`task_progress`**（BT → CBBA，`swarm_interfaces/msg/TaskProgress.msg`，reliable、keep_last 10）：

```
uint64 task_id           # 要是目前 current_task 上的（和 task_result 一樣檢查）
uint16 revision
uint8 phase              # 同 TASK_PROGRESS 的 phase 常數
uint8 step_index         # 255 = 不知道
string[] milestones      # 這次新達成的里程碑名稱（target_localized…）；cbba_node 累積
string detail            # 只記 log
```

**`teammates`**（CBBA → BT，`swarm_interfaces/msg/Teammates.msg`，reliable、transient_local、depth 1；有變化時＋每 1 s）：

```
uint64 task_id           # 我手上的任務（current_task）；0 = 沒有角色組任務
uint64 group_id
Teammate[] roles         # 同組每個角色（含自己）
StepGate[] gates         # 我這個角色每個有 wait_for 的步驟
```

`Teammate.msg`：`task_id`、`role_index`、`optional`、`status`（OPEN／ASSIGNED／DONE／SKIPPED／CANCELLED／EXPIRED）、
`holder`（機號，0 = 沒人）、`assign_state`、`phase`、`milestones`（string[]）、`position`（持有者在 AGENT_STATE 的位置）、
`position_valid`、`updated`（最後一次進度的時刻）。

`StepGate.msg`：`step_index`、`state`（OPEN／WAITING／UNSATISFIABLE）、`waiting_for`（string[]，例如 `role 0: target_localized`，給 log、畫面）。

BT 的用法：ExecuteSteps 每一步開始前看 `gates` 裡有沒有這一步；有而且不是 OPEN 就停住（回報 phase BLOCKED），
變 OPEN 才開始。gate 的判斷只在 cbba_node，BT 不用懂 precedence。

### 1.6 第一批不做的

- incident 位置更新後同組的目標更新（第三批 ⑫⑬）。第一批 `target_localized` 只是一個里程碑。
- gate UNSATISFIABLE 之後的處理（第三批 ⑭⑮）。
- 真的 BT：狗的 BT（伯宇）、無人機的 ExecuteSteps（M1 第二步）之後再接；第一批用假 BT（`ros2 topic pub`）驗證 cbba_node。

### 1.7 測試 ⑦

| 層 | 測試 |
|---|---|
| 封包 | `test_wire_task_progress`：編解碼、長度、欄位範圍、SYNC_BATCH 裝 type 12 |
| 核心（模擬網路） | `test_group_progress`：里程碑 OR、換人與 revision；舊的 progress_seq／指派版本丟掉；非執行者的回報拒收；gate 的 OPEN／WAITING／UNSATISFIABLE（完成、略過＋skip_satisfies、取消）；新鄰居收到快照；掉包下定期重送收斂 |
| 節點（假 BT） | `swarm_cbba/test/cb4_iface.sh`：閘道＋UAV（1）＋狗（50），注入兩個角色（0：uav optional `skip_satisfies`，steps go_to → track；1：狗，steps go_to → inspect（`wait_for` role 0 `target_localized`））。<br>P1 狗的 `teammates` 顯示 step 1 gate WAITING；P2 假 UAV BT 發 `task_progress`（`target_localized`）→ 狗的 gate 1 s 內 OPEN；P3 狗的節點在 UAV 回報之後才啟動 → 從快照拿到、gate OPEN；P4 沒有 UAV：role 0 被略過（O1 10 s）→ gate OPEN；P5 `skip_satisfies` = false → UNSATISFIABLE；P6 非執行者回報被拒 |

---

## 第二批：同步與整組提交（概要，第一批做完再細化）

| 項 | 做法（草案） |
|---|---|
| ⑧ sync_window | 要同步的角色寫在 spec：步驟的 `sync: {group: "front_rear"}`（同一個 sync 名稱的步驟一起開始）；A 層 `sync_window_s` 是時間窗。gate 種類加 SYNC：同名的步驟的角色都 ARRIVED_WAITING → OPEN；第一個到的開始計時，超過 sync_window_s 還沒到齊 → **待決定**：照樣開始（建議，原文沒說失敗）或 UNSATISFIABLE |
| ⑨ ARRIVED_WAITING | phase 6：已經到 sync 的那一步、等同伴；BT 回報。teammates 顯示各角色是不是已到 |
| ⑩ all-or-nothing 暫定指派 | `all_or_nothing` 的組：角色排第一、assign_hold 已過，但同組必要角色還沒都有得標者 → `WAITING_GROUP`（像 WAITING_SPEC：不建立正式指派、不鎖定、不發 current_task），可以被別台搶走 |
| ⑪ group commit／timeout | 各節點本機判斷：同組每個必要角色都有得標者、穩定 `group_commit_hold`（1 s）→ 各自 commit（建立正式指派）。超過 `group_commit_timeout`（暫定 30 s）湊不齊 → `task_orphaned`（GROUP_INCOMPLETE）通知閘道；閘道在第三批決定取消或 fallback。commit 之後有角色失敗 → 第三批的連鎖撤銷 |

## 第三批：閘道管理（概要）

| 項 | 做法（草案） |
|---|---|
| ⑫ IncidentUpdate | 執行者（例如 UAV）回報新的目標位置：`incident_update` 話題（BT → CBBA，incident_id／group_id、位置、不確定半徑、track_id）→ 新封包送閘道（轉送、ACK、斷線暫存，照 DETECTION 的做法）→ 閘道發 `incident_updates` 給事件流程 |
| ⑬ 同組 revision 更新 | 閘道收到 IncidentUpdate → incident revision + 1 → 對同組**還沒完成**、位置參照 incident 的角色做 `inject_task` 改版（只改位置 → 指派保留，4b 已有）→ 執行中的角色收到新的 current_task（同 task_id、新 revision）。BT 端「同 task_id 新 revision 更新目標、不重做」要和伯宇對 |
| ⑭ 連鎖撤銷 | 閘道依範本：必要角色的 gate UNSATISFIABLE（取消、過期、永久沒人能做）時，`all_or_nothing` 的組 → 取消整組（TASK_CLOSE reason 2）；不是 → 只通知。閘道斷線時各節點不自己撤銷（照 4e 的 O3 規則） |
| ⑮ FallbackRequest | 閘道判定要降級時（非 optional 角色沒人可用、整組取消）發 `fallback_request`（group_id、incident_id、原因、`fallback_template`）給外部 task manager，由它重新 `inject_task`（範本展開是 PLN-1，閘道不自己展開） |

---

## 使用者 11/16 的決定（第一批照這些實作）

1. 協定 v4 新增封包種類 12 TASK_PROGRESS（不升版本）。
2. TaskSpec 的每個 step 可以用 `wait_for` 指定前置角色與 milestone；template-level precedence 沒指定 step gate 時，
   預設由後方角色的**第一步**等前方角色 `completed`。
3. BT 遇到未滿足的 wait_for 時回報 **BLOCKED**（phase 5，原本草案叫 WAITING_GATE）並停在該 step，**保持正式指派、不計入執行逾時**
  （cbba_node：`AssignmentManager::setBlocked`，停住的時間不算 overrun，解除時預估完成時刻往後延）。
   gate 本身的第三種狀態改名 **UNSATISFIABLE**（避免和 BT 的 BLOCKED 混淆）。
4. TASK_PROGRESS 用 progress_seq 合併；狀態變化時送三份錯開的封包（0、0.2、0.6 s）；執行中每 2 s 重送最新快照。
5. TASK_CLOSE(COMPLETED) 同時視為 completed milestone（全部里程碑）。
6. 換人後清除執行者相關的 milestone；事件結果型（target_localized、has_observation）只在**相同 incident revision 而且沒過期**時保留
  （原本草案的「sticky」改成這條；incident revision 用前置角色任務的 revision 代表，第三批的 IncidentUpdate 讓同組一起改版）。
   事件結果型跟著 revision 合併（不跟指派版本）：舊的執行者晚到的定位，同一個 revision 也算。

## 第一批的實作（2026-11-16）

| 部分 | 檔案 |
|---|---|
| 里程碑、phase | `cbba_core/milestones.hpp/.cpp`；註冊表 `registry/actions.yaml` 的 `milestones`（`test_registry` 比對 bit、kind） |
| 封包 | `wire_task_progress.hpp/.cpp`（61 B）；`wire_header`（種類 12、轉送）；`wire_sync`（SYNC_BATCH 可以裝） |
| 合併、gate 的里程碑 | `group_progress.hpp/.cpp`（`mergeProgress`、`effectiveMilestones`、`WaitFor`、`GateState`） |
| 協定層 | `CbbaComm::reportProgress`（只收自己保留或執行中的正式指派）、`sendProgress`（錯開 3 份＋每 2 s）、`onTaskProgress`（轉送、只收正式執行者）、新鄰居時的進度快照（SYNC_BATCH）、`milestones`、`groupTasks`、`roleTask`、`gateState`、`agentPosition`；`CommConfig::progress_period`、`progress_repeat` |
| 執行逾時 | `AssignmentManager::setBlocked` |
| ROS 訊息 | `swarm_interfaces` 2.1.0：`TaskProgress.msg`、`Teammates.msg`、`Teammate.msg`、`StepGate.msg` |
| cbba_node | 訂 `task_progress`（要是目前 current_task）、發 `teammates`（有變化時＋每 1 s）；`swarm_cbba/spec_gates.hpp/.cpp`（spec 的 wait_for → step gate，預設第一步等 completed）；log「gate <任務> step N：WAITING → OPEN」 |
| 測試 | `test_wire_task_progress`（4）、`test_group_progress`（9）、`test_wire_header`（種類 12）、`swarm_cbba/test_spec_gates`（3）、`test_registry`（+1）；節點 `swarm_cbba/test/cb4_iface.sh`（P0～P7） |
