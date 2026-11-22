# CB-4 協同：設計與分批（2026-11-16；第一批已實作）

一件事要多台時，閘道（或外部 task manager）把它拆成角色：同一個 `group_id`，每個角色是一個 CBBA 任務。
M0 已經有：角色組的注入（`inject_task` 的 `role_index`／`role_count`／`optional`／`all_or_nothing`／`skip_satisfies`／
`precedence`／`sync_window_s`）、TASK_ANNOUNCE 轉送這些欄位、各角色各自競標、optional 角色自動關閉（4e）。
**CB-4 要讓這些約束真的有作用**。原則（使用者 11/15、2-1b 第八節）：

- **CBBA 照現在各自分配**（出價、17 條規則、TDR 都不動）；協同在**執行端**做。（2026-11-18 第二階段：出價也看協同等待，見最後一節「協同出價」；17 條規則、協定不變）
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

## 第二批：同步與整組提交（2026-11-16 實作，照 11/16 提出的 4 點建議；使用者之後要改再調）

| 項 | 做法 |
|---|---|
| ⑧ sync_window | spec 步驟的 `"sync": "名稱"` 是會合點；同組裡 spec 有同名會合點的其他角色（還沒收到 TaskSpec 的，有 `sync_window_s` 就當成要等）都到了才 OPEN。第一個到的（自己或別人，這台第一次看到的時刻）開始算，超過自己任務的 A 層 `sync_window_s` 還沒到齊 → **照樣 OPEN**、`StepGate.timed_out = true`、`waiting_for` 列沒到的、log 警告一次（已到的先開始）。`sync_window_s` 0 = 一直等。被略過、取消、過期的角色不等；已完成的算到了。判斷在 cbba_node（`swarm_cbba::evaluateSync`，`syncGate`） |
| ⑨ ARRIVED_WAITING | BT 到了會合點回報 phase 6；cbba_node 自動加里程碑 **`sync_reached`（bit 8，執行者相關）**；和 BLOCKED 一樣保持正式指派、不計入執行逾時 |
| ⑩ all-or-nothing 暫定指派 | `all_or_nothing` 的角色排第一、assign_hold 已過、TaskSpec 收齊，但同組必要角色（不是 optional）還沒都有得標者（`CbbaComm::groupStatus`：每個必要角色有得標者或正式指派、或已完成；role_count 個都認得）→ **WAITING_GROUP**：不建立正式指派、不鎖定、不發 current_task（CBBA 的得標照常，別台仍可以贏走） |
| ⑪ group commit／timeout | 湊齊而且穩定 `group_commit_hold`（1 s）→ 各節點各自提交（建立正式指派；看到的得標者一樣，不另外交換訊息）。等超過 `group_commit_timeout`（30 s）→ `task_orphaned` **GROUP_INCOMPLETE**（每個任務一次，detail 寫缺哪幾個 role），繼續等；必要角色被取消、過期時 `groupStatus.broken`，取消或 fallback 由第三批的閘道決定 |

實作：`milestones.hpp`（`SYNC_REACHED`）、`CbbaComm::groupStatus`、`AssignmentManager`（`WaitingGroup`、`groupReady`、`takeGroupTimeouts`、
`group_commit_hold`／`group_commit_timeout`）、`swarm_interfaces` 2.2.0（`StepGate.sync`、`timed_out`；`TaskOrphaned.GROUP_INCOMPLETE`）、
`swarm_cbba/spec_gates`（`sync`、`evaluateSync`）、cbba_node（參數 `group_commit_hold`、`group_commit_timeout`）、註冊表 `milestones` 加 `sync_reached`。
測試：`test_group_progress` 的 `GroupCommit`（3 項）、`test_spec_gates`（+2）、`test_registry`（9 個里程碑）、節點 `swarm_cbba/test/cb4b_iface.sh`（S0～S4）。

## 第三批：閘道管理（使用者 11/16 定案）

| 項目 | 定案 |
|---|---|
| 1 IncidentUpdate 傳送方式 | 新增獨立的 **INCIDENT_UPDATE** 封包，不借用 DETECTION。執行中的角色向閘道回報 incident 的位置與追蹤更新 |
| 2 同組角色改版 | 閘道接受更新後 **incident_revision 統一加 1**，重新展開同組未完成的角色；**只有 TaskSpec 實際改變的角色才增加 task revision** |
| 3 連鎖撤銷 | 必要角色**永久不可恢復 → 立即處理**；**暫時不完整 → 等 `group_recovery_timeout_s` = 10 s**，期間讓 CBBA 重新分配；是否取消整組由 `all_or_nothing` 決定 |
| 4 fallback_template | 由規劃器透過 **InjectTask 明確帶入**、保存在組資料；閘道不依事件類型猜測。需要降級時只發布**冪等的 FallbackRequest**，由外部 task manager 展開新範本、注入新的角色組 |

### 細節的決議（使用者 11/16）

| 細節 | 決議 |
|---|---|
| A incident_revision 的傳送 | **組層級的 incident state**，INCIDENT_UPDATE 分 **Proposal／Accepted 兩階段**。TASK_PROGRESS **同時保留 task_revision 與 incident_revision** |
| B 重新展開 | **不採用閘道字串代換**：Planner 訂閱 `incident_updates`，自己重新展開範本，用 `inject_task` 改版（只有 TaskSpec 改了的角色） |
| C 回報者自己的定位 | 採用，但 update 要**明確宣告**（例如 `declares_localized`），**閘道驗證通過**後，才在新的 incident revision 建立那個角色的里程碑 |
| 連鎖撤銷 | 採用上面的表；**非 all-or-nothing 也要處理依賴失效**；FallbackRequest 用 **`group_failure_version`** 保證冪等 |

### 實作的設計（照決議展開，2026-11-16）

**INCIDENT_UPDATE（新種類 13，轉送）**
- Proposal（機器人 → 閘道）：group_id、base_incident_revision（提案時知道的）、proposal_seq、回報者的 task_id 與指派版本、位置（map＋樓層＋不確定半徑）、
  track_id、`declares_localized`。沒收到結果每 1 s 重送（同一個 proposal_seq），閘道斷線時暫存（照 DETECTION）。
- 閘道驗證：組還開著；回報者是那個角色**目前的正式執行者**（指派版本一致）；base_incident_revision = 目前的（較舊 → 拒絕，回報者重新看最新的再提）；
  位置有效。通過 → incident_revision + 1，廣播 **Accepted**；不通過 → 廣播 **Rejected**（原因），只給回報者看。
- Accepted（閘道 → 全隊）：group_id、incident_revision、位置、track_id、`localized_by`（宣告了而且通過驗證的回報者 task_id，沒有 = 0）、
  回應的 (提案者, proposal_seq)。開著的組每 2 s 重送、新鄰居時 SYNC_BATCH 補。各節點記 group → incident state；
  `localized_by` 的任務在新 incident revision 記 target_localized（C）；其他角色的事件結果型里程碑在新 revision 失效。
- 閘道發 ROS 話題 `incident_updates`（給 Planner）；Planner 重新展開，對 TaskSpec 改了的角色 `inject_task` 改版（B）。

**TASK_PROGRESS**：加 `incident_revision`（2 B，61 → 63 B）；原本的 `revision` 是 task_revision。事件結果型里程碑看 incident_revision
（沒有角色組的任務 incident_revision = 0）。第一批「用前置任務的 revision 代表」的暫時做法拿掉。

**連鎖撤銷（閘道）**
- 永久不可恢復：必要角色取消、過期、全隊沒有能做的、PERMANENT 失敗而且沒有別台能做 → 立即處理；暫時不完整（沒人得標、撤銷、重新競標）
  → 等 `group_recovery_timeout_s` 10 s，還沒恢復就當成永久。
- all-or-nothing：取消整組其他還開著的角色＋ FallbackRequest。
- 非 all-or-nothing（**依賴失效**）：找出依賴失效角色的角色（A 層 precedence 遮罩，遞移），它們的 gate 永遠不會開 → 一起取消；
  不依賴的角色照做。有被取消的角色時發 FallbackRequest；都不依賴時只發 `task_orphaned`（GROUP_BROKEN）。
- `group_failure_version`：每個組的失效事件計數，每次新的失效（新的角色、新的原因）+ 1；FallbackRequest 的冪等鍵 = (group_id, group_failure_version)，
  重送用同一個。組沒有 fallback_template 時不發。

**InjectTask**：加 `fallback_template`（組資料，同一組的角色要一樣；不一樣時拒絕）、`incident_revision`（Planner 改版時帶，閘道檢查不比目前的新）。

**distinct_agent_set（使用者 11/16 一併凍結）**：同組角色是否必須由不同 agent 執行，由 `distinct_agent_set`（位元 = role_index，
同組一樣）定義；**出價時是硬性可行性限制**（bundle 或已接受的任務裡有同組另一個 distinct 角色 → 不出價，出價表原因 `distinct`），
**Group Commit 再驗證一次**（`groupStatus.duplicates`：同一台得標兩個 distinct 角色 → 不算湊齊）。同一 agent 要依序做幾件事 →
同一個角色的多個 steps。TASK_ANNOUNCE 加 1 B（123 → 124 B）。

### 第三批的實作（2026-11-16）

| 部分 | 檔案 |
|---|---|
| 封包 | `wire_incident_update.hpp/.cpp`（種類 13，90 B，PROPOSAL／ACCEPTED／REJECTED）；`wire_task_progress`（加 incident_revision，63 B）；`wire_task_announce`（distinct_agent_set，124 B）；`wire_sync`（SYNC_BATCH 可以裝 13） |
| 協定層 | `CbbaComm::proposeIncident`、`handleProposal`（閘道驗證：組開著、base 最新、提案者是那個角色目前的正式執行者、位置有效；冪等）、`applyAccepted`（只收組的建立者送的；宣告定位通過 → `markLocalized`）、`sendIncidents`（提案重送、閘道每 2 s 重送 Accepted）、新鄰居的快照、`incidentRevision`、`incident`、`takeIncidentResults`、`takeAcceptedIncidents`；`groupStatus.duplicates` |
| 核心 | `CbbaAgent::distinctConflict`（buildBundle、bidFor） |
| 閘道 | `Gateway::stepGroups`（永久：取消、過期、NO_CAPABLE 3 s；暫時：沒人接 `group_recovery_timeout` 10 s；all_or_nothing 整組取消，否則失效＋依賴的角色；`group_failure_version`）、組資料（fallback_template、incident_id）、InjectTask 的 incident_revision、fallback_template、distinct 檢查 |
| ROS | swarm_interfaces 2.3.0：`IncidentUpdate`（BT → CBBA `incident_update`）、`IncidentState`（閘道 `incident_updates`）、`FallbackRequest`（閘道 `fallback_request`，transient_local）、`TaskOrphaned.GROUP_BROKEN`、`InjectTask`（distinct_agent_set、fallback_template、incident_revision）、`Teammates`（incident revision、位置）；cbba_node 參數 `group_recovery_timeout_s`、`group_no_capable_s` |
| 測試 | `test_wire_incident_update`（3）、`test_incident`（Incident 4、Distinct 3〔使用者的三個案例〕、Cascade 4）、`test_wire_task_progress`、`test_wire_task_announce`、`test_wire_header` 更新；節點 `swarm_cbba/test/cb4c_iface.sh`（I1、D1、D2、C1） |

**修訂（使用者 11/16）**：「目前沒有存活的、平台與能力符合的載具」**不是永久不可恢復**，和沒人得標、執行者失聯、重新競標一樣標成
**暫時不完整**，共用 `group_recovery_timeout_s`（10 s）；gateway_timeout（3 s）只用在存活判斷。恢復期限內恢復（有人得標或重新形成
正式指派、執行者還在）就繼續執行；逾時以 **RECOVERY_TIMEOUT** 收尾，並**保留最初的觸發原因**（`recovery_triggers`：NO_CAPABLE、
NO_WINNER、HOLDER_LOST；中途換成別的原因不改、不重新計時）。只有**合法取消、任務／事件過期**這種確定終止事件立即連鎖撤銷。
初版的 NO_CAPABLE 3 s（`group_no_capable_s`）拿掉；`GroupFailure` 的 3 保留不用。
同時成立時的觸發原因順序：執行者失聯 → 沒有能做的載具 → 沒人得標。

已知限制：「PERMANENT 失敗而且沒有別台能做」閘道看不到各台的排除，會表現成「沒人得標」走 10 s 的恢復等待。

**執行者失聯 vs 失聯接手（做法 A，使用者 11/16）**：原本兩個計時都從「判定失聯」起算，P0 的失聯接手（10 s）和恢復期限（10 s）
同時到，接手後還要重新競標，所以幾乎一定先 RECOVERY_TIMEOUT；P1（15 s）、P2／P3（30 s）更是輪不到接手。改成：

1. 執行者超過 `lost_timeout`（1.5 s）→ 只**標記失聯**（`GroupData::holder_lost`），**不**開始算 `group_recovery_timeout_s`。
2. 各優先級照 `takeover_by_priority`（P0 10、P1 15、P2／P3 30 s）等原執行者恢復；恢復了就取消標記、繼續執行。
3. 接手時限到、`CbbaComm::takeOverLost` 撤銷舊指派並重新開放競標時，**才開始算 10 s**（觸發原因 HOLDER_LOST）。
4. 10 s 內有新的得標者或正式指派 → 組繼續；逾時仍沒人接 → RECOVERY_TIMEOUT，依 all_or_nothing 取消整組，或只取消失效角色＋依賴它的
   角色，再發 FallbackRequest。

例 P1：1.5 s 判定失聯 → 再 15 s 撤銷、重新競標 → 再 10 s 內沒有新指派才 fallback（失聯後最多約 26.5 s）。
測試：`test_incident` 的 `Takeover.SuccessfulTakeoverDoesNotCancel`（接手時限 12 s ＞ 恢復期限 10 s 仍由別台接手、不取消）、
`Takeover.TakeoverTimeoutTriggersFallback`（撤銷後 10 s 才失效、取消依賴角色、發 fallback）、`Takeover.WaitFollowsTaskPriority`
（P0、P1 各用自己的接手時限）；`Cascade.HolderLost*` 照新時序更新。

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

## 協同出價（第二階段，任務欄位補齊 09，2026-11-18）

第一階段（上面三批）出價不看協同：依賴的角色當成「到了就能開始」，實際上要在執行端等。PDF：「這是最佳化精度問題，不代表第一階段不成立」；
第二階段把等待算進 TDR：**有效到達 = max(自身到達、前置角色達成里程碑、會合隊友到齊)**。執行端的 gate、Group Commit、連鎖撤銷都不變。

| 項目 | 做法 |
|---|---|
| 放哪裡 | `CbbaComm::updateCoopEta`（每次 reevaluate 前，`reevaluate_period` 1 s）估別台的時刻，寫進 `Task::coop`（只在本機，封包不帶）；`scoring` 的 `evaluatePath` 把它當成像 release 一樣的等待（`LegEval::wait`），TDR 照開始時刻衰減 |
| 前置角色 | 要等的里程碑：TaskSpec 的 `wait_for`（cbba_node 的 resolver 用 `parseSpecGates` 解析，和 teammates 的 gate 同一套）；沒有 spec 時 A 層 precedence 的每個角色等 completed。gate 已經 OPEN（或不會滿足，交給閘道）的不等 |
| 估別台的時刻 | 前置角色的持有者（正式指派，沒有就用得標者）到它的任務：直線距離 / (`platform_speed`[平台] × speed_margin)；accepted、en_route ≈ 現在，arrived、started、target_*、sync_reached ≈ 到達（已回報 arrived 就是現在），has_observation、completed ≈ 到達 + 執行時間。**不看它路徑上排在前面的任務**（AGENT_STATE 的 path 有，之後需要再加） |
| 會合 | 自己和同組其他 `sync_window_s` > 0 的角色（A 層的近似：spec 的會合點名稱只在執行端分）：隊友最早、最晚的預估到達；開始 = max(自己, min(最晚, 第一個到的 + sync_window_s))，和執行端的 `evaluateSync` 一樣逾時照樣開始。已回報 sync_reached／arrived 的算現在到；已結束、取消、過期的不等 |
| 不知道時 | 還沒有得標者、得標者是自己（自己的路徑順序在出價裡算）、不知道位置或平台速度：不加限制（樂觀，和第一階段一樣） |
| 第一次出價（2026-11-20） | 新任務在 `CbbaComm::addTask` 交給核心之前就算好等待（`coopEtaFor`，和 `updateCoopEta` 共用）。原本只在 reevaluate 更新：第一次出價當成「到了就能開始」，近的空閒載具先得標、`assign_hold` 0.4 s 後就鎖定，之後的預估改不了得標者（itest G17 發現）。測試 `CoopBidding.FirstBidAlreadyWaits` |
| 抖動 | 預估變動超過 `coop_eta_tolerance`（2 s）才更新；各台用同樣的來源（得標者、AGENT_STATE 的位置），同一個任務看到的大致一樣：等待對所有出價者一樣，近的還是比較省電（同分時照舊），主要的差別是**等待期間先做別的任務**、出價不再高估 |
| 收斂 | 等待只依賴「別的角色」的得標者，前置角色自己的出價不受影響；會合的角色互相依賴，但每台只用隊友的位置估，不用隊友的出價；`coop_bidding: false` 退回第一階段 |
| 參數 | cbba_node：`coop_bidding`（true）、`coop_eta_tolerance`（2 s）、`platform_speed`（[uav, quadruped]，預設 [0.5, 1.0]；有註冊表時用 platforms.yaml 的 cruise_speed）。全隊要一樣 |
| 單獨任務的 fallback | 由閘道管理（任務欄位補齊 07 的 P0 看門狗 → `FallbackRequest`），各台不自己展開 |

PDF 的四個驗收項目（第一階段，已經有測試）對照在 `test_coop_bidding.cpp` 開頭。第二階段的測試：`test_coop_bidding`
（`CoopScoring` 2 項：等待時先做別的任務、會合最多等 window；`CoopBidding` 5 項：前置角色的預估、spec 的 wait_for、沒有得標者不加限制、
前置完成後清掉、會合看到隊友的預估到達、第一次出價就有等待）。
