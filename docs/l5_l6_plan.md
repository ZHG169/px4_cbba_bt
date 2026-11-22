# L5 事件狀態機＋L6 任務生成：實作規劃（2026-11-23，使用者已確認第 6 節的決定）

來源：〈2-1 監視事件轉 CBBA 任務〉第一、二、三節與第五節階段 1；〈2-1b〉第六、七節。使用者 2026-11-23：L1～L8 都是使用者負責，
先做 L5＋L6。這兩層把「偵測」變成「CBBA 任務」：目前 itest 裡的假 Planner（寫死在 `itest.py` 的 TEMPLATES）就是要被取代的東西。

## 0. 現在已經有的（不重做）

| 已有 | 在哪裡 |
|---|---|
| 偵測傳到閘道：Detection → DETECTION 封包 → 閘道發 `/gcs/detections`（斷線暫存、2 m 合併） | cbba_node（M0 第 5 步） |
| 注入：`/gcs/inject_task`（冪等鍵 incident_id＋role＋revision、角色組、改版、核准、TaskSpec 檢查、區域檢查） | 閘道 |
| 回饋：`/gcs/incident_updates`（UAV 更新位置）、`/gcs/fallback_request`、`/gcs/task_orphaned`、`/gcs/task_reports`、`/gcs/approvals`、`assignments` | 閘道 |
| 參數：`event_types.yaml` 的 trigger、accumulate、associate、vlm、cost、locate、default_rule、priority、expires_s | 註冊表 |
| 位置：`site-config/yard` 的 zones（屬性 restricted、high_voltage、no_entry、policies）、pois（各區 front／rear／overwatch 觀察點）、devices（設備與觀察姿態）、docks | site-config |
| 格式：Event（`registry/api.md` 第 6 節）、TaskSpec 組（第 4 節：roles、coupling、fallback_template） | api.md |

## 1. 架構

```
機器人／攝影機的偵測 ──► 閘道 cbba_node ──/gcs/detections──►  event_manager（新，GCS 上另一個節點）
                                                              │ L2 時序累積、L3 關聯（同一件事合併）
                                                              │ L4 VLM 驗證（介面＋逾時退化；VLM 本身之後做）
                                                              │ L5 事件狀態機、派遣門檻
                                                              │ L6 範本展開（event_rules.yaml）→ TaskSpec 組 → 檢查
                                                              ▼
                                         /gcs/inject_task（每個角色一個）──► 閘道 ──► CBBA
   /gcs/incident_updates、fallback_request、task_orphaned、task_reports、assignments ──► event_manager（更新狀態、改版、降級）
   event_manager ──► /gcs/incidents（IncidentState，給控制中心、影片）、事件日誌
```

- **新套件 `swarm_events`**（C++，和 `swarm_cbba` 一樣分兩層）：
  - 不依賴 ROS 的函式庫：`IncidentManager`（L2、L3、L5）、`RuleExpander`（L6）、`EventRules`（讀 `event_rules.yaml`）。用 gtest 測，可用模擬時間。
  - 很薄的節點：`event_manager_node`，只負責話題和服務。
- **為什麼不放進閘道的 cbba_node**：閘道是 CBBA 協定節點，只管任務的建立、同步、監控、關閉（11/13 的定義）。事件判斷、範本會常常調，另一個程序可以單獨重開，不影響 CBBA。這也和 2-1 的流水線一致：L5～L6 是任務生成器，L7 才是閘道。
- 註冊表、site-config 的載入沿用 `swarm_cbba/registry.hpp`；TaskSpec 檢查沿用 `spec_validator`。展開完先在本機檢查，閘道注入時會再檢查一次。

## 2. L5 事件狀態機（含 L2、L3 的邏輯）

L2、L3 一起放進來：沒有累積和關聯，就沒有「候選事件」可以進狀態機。這兩層都是純邏輯，參數已經在 `event_types.yaml` 裡。L1 偵測器、L4 VLM 模型之後再做。

### 2.1 狀態（照 2-1 第一節 L5）

```
CANDIDATE ──L2 分數 ≥ θ_on 且持續 min_duration──► VERIFYING ──VLM 否定──► REJECTED
                                                   │ 信心 ≥ θ_high ──► CONFIRMED ──展開完整範本──► DISPATCHED
                                                   │ θ_low ≤ 信心 < θ_high ──► NEEDS_CHECK ──展開確認範本（1 隻）──► DISPATCHED
                                                   │ 信心 < θ_low ──► 只記錄（LOGGED）
DISPATCHED ──有角色到場──► ON_SITE ──必要角色都完成──► RESOLVED
                         │ 確認任務回報「沒有目標」或值班判定 ──► FALSE_ALARM
DISPATCHED／ON_SITE ──P0 逾時沒人得標、整組失效、值班升級──► ESCALATED（發 fallback 或交給值班）
任一狀態 ──expires_s 到了──► EXPIRED
```

### 2.2 各層的做法

| 項目 | 做法（起始值照 `event_types.yaml`） |
|---|---|
| 偵測進來 | 依 label 找偵測器、事件類型；`score < min_score` 丟掉；`trigger` 不符的丟掉，包括 `zone_attr`（偵測位置所在區域要有這個屬性）和 `time_rule`（`off_hours` 等，時段用節點參數） |
| L3 關聯 | 同一事件類型、`associate.dt_s` 內、`associate.r_m` 內（或同 `track_id`）、還沒結案的事件 → 併進去（加來源、更新位置）；否則開新的候選 |
| L2 累積 | **照 2-1 原文用 log-odds**：`s ← λ·s + log(p/(1−p))`，`s ≥ θ_on` 持續 `min_duration_s` 成立，`s < θ_off` 取消候選（遲滯）。`event_types.yaml` 現在的說明寫「加 score」，要改成 log-odds（見第 6 節決定 3） |
| 融合信心 | 沒有 VLM：`p = sigmoid(s)`；有 VLM：用 VLM 校準後的信心（介面先帶 `confidence`，校準之後做） |
| L4 VLM 驗證 | 有 `vlm` 的事件類型：發 `/gcs/vlm_request`，等 `/gcs/vlm_result`（verdict confirmed／rejected／uncertain＋confidence）。`timeout_s` 到了照 2-1 退化：P0（火、煙）直接用 L2 的信心；P2（遺留物）延後再試一次。現在沒有 VLM，測試用假的 VLM 節點回結果 |
| 派遣門檻 | `θ_low = C_false / (C_false + C_miss)`（例如 fire 1/21 ≈ 0.05、left_object 1/4 = 0.25）；`θ_high` 每個事件類型可設，預設 0.8（新欄位 `dispatch.theta_high`） |
| 去重、改版 | 已派遣的事件又收到同一件事的偵測：附加來源；位置移動超過 `retarget_threshold_m`（3 m，和閘道一致）才 `revision + 1` → L6 重新展開，只有 TaskSpec 變了的角色改版（就是 itest G6 的邏輯，從假 Planner 搬過來）。UAV 的 `incident_updates`（位置更新、定位）走同一條路 |
| 升級 | `task_orphaned` P0_UNASSIGNED、GROUP_BROKEN 或 `fallback_request` → ESCALATED。有 `fallback_template` 就展開那個規則（新的子事件 id）並發給值班；沒有就只通知值班 |
| 結案 | 從 `assignments`、`task_reports` 判斷：必要角色都 DONE → RESOLVED；確認任務的回報帶「沒有目標」→ FALSE_ALARM（判斷方式見決定 4）；值班可以用服務 `/gcs/resolve_incident` 結案或標誤報 |
| 過期 | `expires_s`：展開的任務帶同樣的 expires（閘道會發 TASK_CLOSE reason 3）；事件標 EXPIRED |
| 輸出 | `/gcs/incidents`（新訊息 `IncidentState`：event_id、type、state、revision、confidence、位置＋不確定半徑＋區域、來源、group_id、任務、原因；內容變了才發）＋事件日誌 `<state_dir>/incidents.log`（每次轉換一行，同一個 event_id 可重播，照 2-1 第三節） |
| 重開 | 開著的事件存檔（像閘道的 `gateway_state_file`），重開沿用 event_id、group；閘道注入是冪等的，不會重複建任務 |

## 3. L6 任務生成：`event_rules.yaml`＋展開器

### 3.1 規則格式（放 `site-config/<場域>/event_rules.yaml`：觀察點、語音範本跟場域有關）

```yaml
schema_version: "1.0"
rules:
  perimeter_intrusion:
    requires_approval: false            # 沒寫照事件類型
    roles:
      - role: aerial_overwatch
        platform: uav
        required_capabilities: [mobility_air, rgb_camera]
        optional: true
        skip_satisfies: true
        location: {poi: overwatch}       # 事件所在區域、這個平台、role=overwatch 的觀察點；找不到就用 fallback_offset
        fallback_offset: [0, 0]
        steps:
          - {action: go_to}
          - {action: track, object: "person:{incident.track_id}", params: {duration_s: 30}}
      - role: front
        platform: quadruped
        required_capabilities: [mobility_ground, rgb_camera, speaker]
        location: {poi: front}
        fallback_offset: [3, 0]
        steps:
          - {action: go_to}
          - {action: inspect, object: person, params: {duration_s: 3},
             wait_for: [{role: aerial_overwatch, milestone: target_localized}], sync: front_rear}
          - {action: speak, params: {template: intrusion_warning}}
      - role: rear
        ...
    coupling: {all_or_nothing: true, distinct: [front, rear], sync_window_s: 15}
    fallback_template: single_dog_verify
  fire_response: ...
  single_dog_verify: ...                # NEEDS_CHECK、fallback 用的「派 1 隻確認」
  thermal_check: ...                    # 熱異常：location {asset: "{incident.asset}"}（設備的觀察姿態）
  left_object_check: ...
```

- 角色之間用**名稱**互相參照（`wait_for.role`、`distinct`），展開器換成 `role_index`／位元遮罩。現在 itest 的範本是手寫 `role_index` 和遮罩。
- 位置照 api.md 第五節：`{poi: <role>}`（依事件所在區域、平台挑觀察點，多個時取最近）、`{asset: …}`、`{zone: …}`、`{incident}`（事件位置）、`fallback_offset`。
- **展開時的可達性**（2-1 第三節「目標點不可達 → 移到最近的可達觀察點」）：觀察點落在這個平台不能進的區域，就移到最近的安全點，並記在 Event 日誌。這是把 itest 的 preflight 搬進正式的展開器。
- 檢查：展開後跑 `spec_validator`（格式、詞彙、參數、位置、步數、平台、區域、風險 → 核准），**再補 PLN-2 缺的兩層**：
  - 機隊可行性：非 optional 角色要有存活、能力符合的載具；沒有就改展開 fallback 或交給值班。
  - 續航：用 `platforms.yaml` 估往返加作業時間，對每台可能的載具都不夠就標警告。
- 注入：每個角色呼叫一次 `/gcs/inject_task`，帶 incident_id、event_type、task_type、證據、`requires_approval`、`fallback_template`。

## 4. 分三批做（每批單獨可測）

| 批 | 內容 | 測試 |
|---|---|---|
| **A：L6 展開器** | `event_rules.yaml` 格式＋載入檢查；`RuleExpander`（位置、代換、角色名稱 → index／遮罩、可達性、改版只動有變的角色）；PLN-2 補機隊可行性、續航；節點先提供服務 `/gcs/expand_incident`（給一個 Event 就展開注入） | gtest（展開結果、改版、可達性、錯誤規則）；節點腳本：注入 Event → 閘道收到正確的角色組 |
| **B：L5 狀態機＋L2、L3** | `IncidentManager`（關聯、累積、觸發條件、門檻、狀態轉換、去重改版、升級、結案、過期、存檔）；`IncidentState` 訊息、`/gcs/incidents`、事件日誌；VLM 介面＋假 VLM | gtest（每個轉換、遲滯、關聯、門檻、退化、重開）；節點腳本：連續假偵測 → 事件 → 任務 → 結案 |
| **C：換掉 itest 的假 Planner，補 2-1 階段 1 的案例** | itest group 改成「偵測 → event_manager」（G14、G15 不再呼叫 `plan_from_detection`）；新增 2-1 的案例 3（中信心只派 1 隻）、5（兩事件同時、依優先級）、6（小位移不重新拍賣）、9（告警風暴，若做第 5 節的合併） | itest group 全過＋新案例 |

估計：A 約 2～3 天、B 約 3～4 天、C 約 2 天（不含等待測試）。M2（12/18）要求展開器、檢查器、EVT-1，這三批做完就涵蓋了。

## 4.1 A 批的實作（2026-11-23 測過）

- `site-config/yard/event_rules.yaml`：perimeter_intrusion、fire_response、single_dog_verify（**和 itest 寫死的範本一樣**，
  回歸用）、smoke_check、thermal_check、left_object_check。角色加 `retarget`（false：事件位置更新時不跟著改；入侵的空中追蹤、observer）
- `swarm_events`（新套件）：`event_rules.hpp/.cpp`（讀檔、`checkEventRules`）、`expander.hpp/.cpp`（`expand`：位置、preflight、
  代換、wait_for 角色名稱 → role_index、TaskSpec 檢查、機隊可行性 FLEET_INFEASIBLE、續航 ENDURANCE 警告；`changedRoles`；
  `zoneAt`、`forbiddenZone`、`safeAnchor`）、`event_manager_node.cpp`（`/gcs/expand_incident`，**只回傳規劃結果，不注入**）
- `swarm_cbba` 匯出函式庫 `swarm_cbba_registry`（registry、spec_validator；原本的執行檔不變）
- `swarm_interfaces` 2.10.0：`PlannedRole.msg`、`ExpandIncident.srv`
- 測試：`test_expander`（13 項：三個範本和 itest 一樣、poi、preflight、設備觀察姿態、機隊可行性、續航、改版只動 front／rear、
  錯誤的規則）、`swarm_events/test/expand_iface.sh`（X1～X6，X6 確認閘道沒有收到任務）
- itest group 的 G14、G15、G5／G7 **這批不改**（照舊用假 Planner，當回歸）；C 批才換成 event_manager

## 4.2 B 批（2026-11-23 起；分 B1 純邏輯、B2 節點）

**B1（2026-11-23 測過）**：`swarm_events/incident_manager.hpp/.cpp`（`IncidentManager`，不依賴 ROS、不注入）：
- 偵測 → 偵測器、label、min_score → 事件類型 → 觸發條件（zone_attr、time_rule；上班時段 `on_hours` 8～18 時）
- L3 關聯：同類型、沒結案、`associate.dt_s`、`r_m` 內或同一個 track_id → 併進去（位置跟著最新的偵測）
- L2 累積（log-odds）：`s ← λ·s + logit(score)`；沒有新偵測超過 1 s 每秒乘 λ 遺忘；`theta_on` 持續 `min_duration_s` → 驗證／決定；候選期間 `< theta_off` 取消
- L4 介面：有 vlm 且 `vlm_enabled` → VLM_REQUEST；rejected → REJECTED、uncertain 最多派確認；逾時：P0、P1 用 L2 決定，P2、P3 再送一次
- L5：信心 ≥ `dispatch.theta_high`（註冊表新欄位，預設 0.8）→ CONFIRMED → 派預設範本；≥ θ_low（cost）→ NEEDS_CHECK → 派範本的 fallback_template；
  其他 LOGGED。證據變強會升級（LOGGED → 派、確認 → 完整範本）。派遣後：到場 ON_SITE、必要角色都完成 RESOLVED、E_TARGET_LOST 或值班 → FALSE_ALARM、
  升級 ESCALATED（派 fallback 子事件，子事件結案原事件跟著結案）、`expires_s` → EXPIRED；incident revision 或偵測移動 > 3 m → REPLAN
- 輸出動作（DISPATCH、REPLAN、VLM_REQUEST、NOTIFY）＋轉換紀錄；`save`／`load`（開著的事件與編號）
- 測試 `test_incident_manager`（14 項，含 2-1 案例 2、3、10）

**B2（2026-11-23 測過）**：`event_manager_node.cpp` 接上 `IncidentManager`：
- 訂 `detections`、`incident_updates`、`task_closed`（DONE／CANCELLED）、`task_reports`（失敗碼）、`assignments`（ASSIGNED／ACTIVE）、
  `fallback_request`（同一個 failure_version 只處理一次）、`task_orphaned`（只處理沒有 fallback_template 的角色）、`vlm_result`；訂閱一律 volatile
- DISPATCH／REPLAN：展開 → 對角色呼叫 `inject_task`（incident_id = 事件編號、role、revision 每次 + 1）；REPLAN 只注入 `changedRoles`；
  換範本（確認升級成完整範本）時對舊範本多出來的角色發 `cancel_task`；展開失敗、注入被拒 → ESCALATED＋通知值班；`auto_dispatch false` 時只判斷不注入
- 輸出：`incidents`（`EventIncident`，內容變了才發）、`vlm_request`、`operator_notices`（std_msgs/String）；服務 `resolve_incident`
- 存檔：`state_dir` 的 `event_manager_state.yaml`（開著的事件＋角色對應的 task_id）、事件日誌 `incidents.log`；重開讀回
- swarm_interfaces 2.11.0：`EventIncident`、`VlmRequest`、`VlmResult`、`ResolveIncident`
- 測試：`swarm_events/test/incident_iface.sh`（N1～N7）
- **還沒接的**：ON_SITE（到場）沒有資料來源——閘道沒有對外發 task_progress（C 批做，見 4.3）；VLM 服務本身（只有話題）

## 4.3 C 批的決定（使用者 2026-11-23）

| # | 決定 |
|---|---|
| 1 | **閘道統一發布角色／任務進度**（目前閘道收得到 TASK_PROGRESS，但沒有對外發布）；event_manager **依有效版本聚合**（只算目前正式指派版本、目前 revision 的進度，換人、改版後的舊進度不算），事件狀態 **DISPATCHED → ON_SITE → EXECUTING → RESOLVED**（比 B 批多 EXECUTING：有必要角色開始執行第一個非移動的步驟） |
| 2 | C 批補 2-1 階段 1 的 **案例 3（中信心只派 1 隻確認）、5（兩事件同時、依優先級）、6（移動入侵者、小位移不重新拍賣）、9（告警風暴合併成區域巡查）**；案例 9 要在 event_manager 加「同一區同時告警任務設上限、超過就合併成區域巡查並通知值班」 |

C 批的內容（照上面）：閘道發布角色進度 → event_manager 聚合（ON_SITE、EXECUTING）→ itest group 換掉假 Planner
（偵測串流 → 閘道 → event_manager → 展開 → 注入；G5／G7 的降級改由 event_manager 收 fallback_request 派子事件）→ 補案例 3、5、6、9。

## 5. 先不做（之後再說）

- L1 偵測器（人員、煙霧、熱像、遺留物）、L4 的 VLM 模型與信心校準：要有影像、模型、GPU；照 2-1，感知用離線評估集另外驗證。
- 攝影機投影（L3 的座標由偵測直接帶 map 座標；投影是空間對位那一塊）。
- 告警風暴合併成區域巡查、攝影機異常事件、機器人自身遮罩：B 批可以預留，C 批再決定要不要做。
- 開放事件（`unknown`）的 VLM 提案：現在只支援「外部給 steps＋一律要核准」。

## 6. 使用者的決定（2026-11-23）

| # | 問題 | 決定 |
|---|---|---|
| 1 | 放哪裡 | **新套件 `swarm_events`、GCS 上獨立的 `event_manager` 節點**（閘道只管 CBBA 任務） |
| 2 | L2、L3 | **這次一起做，累積照 2-1 原文用 log-odds**：`s ← λ·s + log(p/(1−p))`，`event_types.yaml` 的說明跟著改、`theta_on`／`theta_off` 數值重新對 |
| 3 | 誤報 | **確認任務回報 `E_TARGET_LOST` → FALSE_ALARM，值班也可以用 `/gcs/resolve_incident` 結案或標誤報** |
| 4 | 順序 | **A 展開器 → B 狀態機＋L2、L3 → C 換掉 itest 的假 Planner、補 2-1 階段 1 案例** |
