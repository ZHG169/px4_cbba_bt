# CBBA 的 M0 合約（2-1b，草稿 2026-11-11）

2-1b 的 M0（11/20 凍結）在 CBBA 這邊要定的四件事：

1. 能力過濾：`platform` ＋ `required_capabilities` 對載具的能力
2. 共同的任務資料結構；異質成本與出價
3. 任務注入、版本（revision）、搶占與重新分配
4. 分配結果交給執行端的介面

這份只定**合約**（欄位、規則、誰負責），實作排在 M1 以後。**task_id 改成 64 位元**（使用者 2026-11-11：
建立者 16 ＋ 世代 16 ＋ 流水號 32，見第六節），協定升版本 4（全隊重新編譯，和版本 3 互相拒收）。

來源：2-1 第二節（任務欄位、分數分層）、2-1b 第二、四、八節（動作、能力、異質處理）、2-1a 的 CB-1～CB-6、CB2-0。
已定案的協定行為（指派版本、鎖定、完成確認、失聯接手…）沿用 `protocol.md`、`protocol_v3_sync.md`，這裡只寫要改的。

---

## 一、能力過濾

### 規則

載具 i 對任務 j 可以出價，當且僅當：

```
task.platform ∈ {ANY, agent.platform}
∧ (task.required_caps & ~agent.available_caps) == 0
∧ 其他現有條件（OPEN、參與出價、沒被排除、電量可行、鎖定…）
```

不符合時出價 0（約束，不是成本），取代現在的 `isCompatible(AgentType, TaskType)`。

### 能力的表示

| 項目 | 做法 | 原因 |
|---|---|---|
| 詞彙 | 註冊表 `tools.yaml` 的能力標籤（mobility_ground、mobility_air、indoor_flight、stairs、rgb_camera、zoom、thermal_camera、speaker、spotlight、gas_sensor、manipulator、gripper、comm_relay） | 唯一來源，CBBA 不自己定詞彙 |
| 封包裡 | uint64 位元遮罩；每個標籤的位元位置寫在 `tools.yaml`（`bit:`），**只增不改**：新增標籤用新的位元，刪除的標籤位元保留不重用 | 封包固定大小、比對一個 AND；字串只在 ROS 端和註冊表 |
| 平台 | uint8：0 = ANY、1 = uav、2 = quadruped（`platforms.yaml` 的 `code:`），同樣只增不改 | 和現在的 AgentType（UAV = 1、UGV = 2）數值相同，舊程式的對應不用改 |
| 載具宣告的能力 | cbba_node 啟動時從 `fleet.yaml` 讀自己機號的 platform、capabilities（參數 `fleet_file`）；沒有 fleet 檔時用參數 `capabilities`（字串陣列） | 換狗、換酬載只改 fleet.yaml |
| 可用的能力 | available = 宣告的 − 回報故障的（`E_HW` 時執行端在 task_result 帶 `failed_capability`；或話題 `capability_status`，M1 再定） | 2-1b：E_HW → 標記能力不可用、轉派 |
| 廣播 | AGENT_STATE 加 platform 1 B、available_caps 8 B | 閘道的「機隊可行性」、孤兒任務警告要知道全隊有誰能做 |

### 閘道不是載具（使用者 2026-11-13 定義）

**閘道是非競標的 CBBA 協定節點，負責建立、同步、監控與關閉任務；不屬於任何載具平台，也不得列入任務可行性與 optional 角色的能力統計。**

| 項目 | 做法 |
|---|---|
| 封包 | AGENT_STATE 的 vehicle_type = **0**（閘道）、capabilities = 0、參與出價 = 0；三者不一致的整則當成壞封包（版本 4 的節點都要重新編譯才收得到閘道的 AGENT_STATE） |
| 核心 | `CommConfig::gateway`：永遠不出價（`setParticipating(true)` 不理會）；轉送、補發、完成確認、TASK_CLOSE 照常 |
| 能力統計 | `CbbaComm::capableAgents(task, now)`：自己（不是閘道時）＋聽得到的載具鄰居裡平台、能力符合的；閘道一律不算。機隊可行性、O1、孤兒任務的原因都用它 |
| 節點 | `cbba_node` 的 `participate: never` 就是閘道（`agent_type`、`capabilities` 參數不用） |

平台代碼 0 在**任務**裡是「不限平台」（ANY），在 AGENT_STATE（載具）裡是「閘道、不是載具」，兩個欄位不同、不會混用。

### 舊任務種類的對應（過渡期）

現有的 `TaskType` 先保留，`new_task` 沒有帶 platform、能力時照下表補上，現有的 itest、demo 不用改：

| TaskType | platform | required_caps |
|---|---|---|
| AIR_RECON | uav | mobility_air、rgb_camera |
| GROUND_INTERVENTION | quadruped | mobility_ground |
| PATROL | quadruped | mobility_ground |

### 診斷

- `bidFor`（出價表）的 why 加「平台不符」「缺能力 thermal_camera」。
- 孤兒任務警告（`task_orphaned`）分兩種：「全隊沒有載具具備 X」（能力問題，馬上報）、「有人能做但沒人得標」（現在的 30 s）。

---

## 二、共同的任務資料結構；異質成本與出價

### 欄位分三層

TaskSpec（2-1 第二節）的欄位很多，大部分 CBBA 用不到。分成：

| 層 | 放什麼 | 怎麼傳 |
|---|---|---|
| **A. CBBA 要的**（出價、分配、搶占、協同） | 下表 | TASK_ANNOUNCE（版本 4），固定大小，轉送＋補發 |
| **B. 執行要的**（steps、params、observe_pose、success_criteria、abort_conditions、bt_subtree…） | TaskSpec 的 JSON | 獨立的 TASK_SPEC 封包，各台快取（下面「TaskSpec 的傳送」） |
| **C. 只給人看的**（evidence、description、report、signature 內容） | 留在閘道 | 不上 mesh |

**A 層（CBBA 任務定義，版本 4）**：

| 群組 | 欄位 | 型別 | 說明 |
|---|---|---|---|
| 識別 | task_id | uint64 | 全隊共用的任務識別（第六節） |
| | revision | uint16 | 同一個 task_id 的內容版本，只接受較大的（第三節） |
| | group_id | uint64 | 同一個事件拆出的角色；格式同 task_id（閘道建立時配號）；0 = 單獨任務 |
| | role_index、role_count | uint8、uint8 | 第幾個角色、共幾個（協同約束用） |
| 能力 | platform | uint8 | 第一節 |
| | required_caps | uint64 | 第一節 |
| 位置 | x、y、z | float32 × 3 | map ENU；出價用的「前往點」（UAV 是上空、狗是觀察點，由閘道依平台算好） |
| | floor | uint8 | 樓層編號（0 = 不分） |
| 時間 | created_ms、release_ms | uint32 | 建立、可以開始的時刻（絕對時刻，同現在的 deadline_ms） |
| | deadline_ms、deadline_hard | uint32、旗標 | 軟／硬期限；硬期限做不到 → 不可行 |
| | expires_ms | uint32 | 到了自動失效（第三節） |
| | service_time_s | uint16 | 現場執行時間（現在的 duration_sec） |
| 價值與優先 | priority | uint8 | P0～P3（0 最高） |
| | value | float32 | 同級內的價值 |
| | half_life_s | uint16 | 價值隨時間衰減（0 = 不衰減） |
| 搶占 | can_preempt | uint8 位元遮罩 | 可以搶占哪幾級（預設：比自己低的級） |
| | preempt_margin | uint8（%） | 同級搶占時要好多少（預設不搶占同級） |
| | commit_radius_dm | uint16 | 執行者離目標這麼近之後不可再被搶占（0.1 m） |
| | min_battery_reserve | uint8（%） | 做完還要剩這麼多 |
| 協同 | optional、all_or_nothing | 旗標 | 角色可省略；湊不齊就全部不做 |
| | optional_no_capable_s、optional_unassigned_s | uint16、uint16 | optional 角色自動關閉 O1、O2 的等待時間（第三節「optional 角色」）；0 = 用預設（10 s、15 s） |
| | skip_satisfies | 旗標 | 這個 optional 角色被略過時，等它的角色（precedence）是否視為條件已滿足 |
| | precedence | uint8 位元遮罩 | 要等哪幾個角色先完成（或先到位） |
| | sync_window_s | uint16 | 要同時到位的時間窗；0 = 不用 |
| 型別 | task_type | uint8 | 舊的 TaskType，過渡期保留；新任務 0 |
| TaskSpec | spec_length、spec_crc | uint32、uint32 | 這個 revision 的 TaskSpec（壓縮後）大小與 CRC-32；0 = 沒有 TaskSpec（舊型任務） |

約 120 B（現在 65 B），任務定義只在建立、改版、補發時送，流量影響很小。

### CBBA_STATE 的紀錄（使用者 2026-11-11：只交換 ID、revision、出價、得標者）

CBBA_STATE 不帶任務內容（A 層在 TASK_ANNOUNCE、B 層在 TASK_SPEC）。每筆紀錄：

| 欄位 | 大小 | 說明 |
|---|---|---|
| task_id | 8 | 第六節 |
| revision | 2 | 出價時用的任務版本；比自己知道的新 → 先等新的 TASK_ANNOUNCE（不交給規則），比較舊 → 這筆不交給規則 |
| winner_id、bid | 2、4 | 得標者、得標價（同級分數，見下） |
| assign_version、assignee_id、assign_state | 8、2、1 | **正式指派，保留**（使用者 2026-11-11 確認；鎖定、完成通知的版本檢查、失聯接手都靠它） |

每筆**統一 27 B**（使用者 2026-11-11；原本 21 B：task_id + 4、revision + 2），一批 1200 B 約 42 筆。

### TaskSpec 的傳送（使用者 2026-11-11）

| 項目 | 做法 |
|---|---|
| 封包 | 新種類 TASK_SPEC：task_id、revision、encoding、part_index、part_count、這一段的資料；TaskSpec JSON 切段（每段 ≤ 1100 B）。**encoding 0 = 原文 JSON（M0 只用這個）；1 = zlib 保留、暫不用**（使用者 2026-11-12：先不加 zlib 依賴，等範本展開器做出真的 TaskSpec、看實際大小再決定；加的時候只是多支援一種編碼，協定不用升版，不認得的編碼當成缺 TaskSpec） |
| 驗證 | 收齊後比對 TASK_ANNOUNCE 的 spec_length、spec_crc；不符整份丟掉、重新要。只收建立者送出的（origin = task_id 的建立者） |
| 轉送、補發 | 和 TASK_ANNOUNCE 一樣：建立時、改版時送，轉送；版本 3 的缺口請求點陣圖加一個狀態「有定義、缺 TaskSpec」，回覆時只補缺的段；SYNC_BATCH 可以裝 TASK_SPEC 段，照 token bucket 限速，排在進行中任務定義之後 |
| 快取 | 每台 cbba_node 存（task_id, revision）→ TaskSpec；同一個任務只留最新 revision；任務結束後 `spec_keep_sec`（預設 60 s）丟掉內容、只留 CRC（避免記憶體隨已結束任務成長） |
| 出價 | 沒有 TaskSpec 也可以出價（出價只用 A 層）：TaskSpec 比任務定義大、弱網時晚到，等它才出價會拖慢收斂（使用者 2026-11-11 確認） |
| **WAITING_SPEC** | 排第一、assign_hold 已過，但沒有這個 revision 的 TaskSpec → 本機狀態 `WAITING_SPEC`：**不發 current_task、不建立正式指派**（不鎖定，別台照 CBBA 規則仍可以出價贏走）、持續要缺的段。AGENT_STATE 的 execution_state 報 WAITING_SPEC，閘道看得到 |
| `spec_wait` 超時 | 超過 `spec_wait`（預設 5 s）仍沒收齊：**不當成這台失敗**（不送 task_result、不排除、不打折），只①提高補發（直接向每個鄰居送 GAP_REQUEST、要求建立者重送全部段）②告警：log 警告＋話題 `task_spec_missing`（task_id、revision、缺哪幾段、等了多久），之後每 `spec_wait` 再告警一次。任務一直在 WAITING_SPEC 時由閘道處理（重送、改版或取消）；期限、過期照常生效 |
| 收齊 | 立刻離開 WAITING_SPEC → 建立正式指派、發 current_task（不再等 assign_hold） |
| 改版 | 新的 revision 帶新的 TaskSpec；執行中的那台收齊新的才發新 revision 的 current_task，收齊前照舊版繼續 |
| 舊型任務 | spec_length = 0（BT 的 new_task、itest 的任務）：不需要 TaskSpec，照現在的方式執行 |

### 異質成本與出價

| 項目 | 做法 | 原因 |
|---|---|---|
| **分級**（CB-2，使用者 2026-11-11） | 依序比較：① priority（P0 最高）② 同級分數 ③ 分數相差在 `score_epsilon` 內時機號小的勝（現在的決勝規則）。**priority 是任務的屬性**：同一個任務的所有出價 priority 都一樣，所以封包裡的 bid 只放同級分數（float32），不用把級數編進數字；分級用在每台**組 bundle 時先挑高級的任務**、搶占、排隊 | 2-1：「任何距離或電量差都無法跨級」。編成 tier × B + score 的話 float32 在大數值時精度不夠（4 萬附近約 0.004，比 score_epsilon 粗），會把平手判錯 |
| 同級分數 | **沿用現在的公式**（使用者 2026-11-11 定案，暫時不換）：score = p × value / (1 + 邊際成本 / cost_ref)，成本 = 空間 × 時間（到達時間／剩餘期限）×（1 ＋ 電池因子） | 已經過 17 條規則、弱網收斂的驗證；邊際分數遞減（CBBA 收斂的條件）成立。half_life_s 先只帶在任務定義裡、不用；要換成 2-1 的 R·2^(−τ/T½) − αE 時要重跑收斂實驗 |
| 平台模型（CB2-0） | `platforms.yaml` 每種平台：cruise_speed、speed_margin、energy_per_meter、idle／hover_energy_per_sec、endurance_s、safety_reserve、max_bundle、must_return | 現在這些是 cbba_node 的參數（狗、無人機各一份 yaml），改成從註冊表讀；個別載具可在 fleet.yaml 覆寫 |
| τ（抵達時間） | 各平台自己的速度；狗之後可換成 Nav2 路徑長度（M2 以後） | 2-1b 第八節 |
| E（能耗） | 電量 % 換算；UAV 的 1 % ＝ endurance_s / 100 秒的飛行時間，等同「佔剩餘飛行時間的比例」 | 2-1b：讓 UAV 不會因為快就搶走所有任務 |
| UAV bundle 上限 | `max_bundle` uav 2、quadruped 5 | 2-1b：UAV 1～2 |
| 返航可行性 | 已有：路徑能量含回程、做完要剩 safety_reserve；must_return 的平台回程點是 fleet.yaml 的 dock／pad | 2-1b：每包末端加返航時間 |
| 硬期限 | deadline_hard 時，到達時間超過期限 → 不可行 | 現在期限只是軟限制 |

---

## 三、任務注入、版本、搶占與重新分配

### 注入（CB-1）

```
事件 → 規劃器 → 檢查器 → 閘道（GCS）的 cbba_node（participate: never、agent_id 例如 200）
     → inject_task（ROS service）→ TASK_ANNOUNCE（版本 4）→ mesh → 各台 cbba_node 出價
```

| 項目 | 做法 |
|---|---|
| 介面 | 閘道 cbba_node 的 service `inject_task`：request = A 層欄位＋TaskSpec JSON（B 層）＋冪等鍵；response = accepted、reason、task_id、revision |
| 不出價 | 新參數 `participate: never`：不訂 robot_state、永遠不出價，照常轉送、補發、記任務（現在不送 robot_state 也不出價，但誤送就會開始出價） |
| 冪等 | 冪等鍵 = (incident_id, role, revision)。同一個鍵重送 → 回同一個 task_id、不重發；同一個 (incident_id, role) 更大的 revision → 改版（下面）；更小的 → 拒絕 |
| 過期 | 每個節點到了 expires_ms 就**本機**停止出價、撤銷自己的指派（時刻是絕對的，全隊結果一樣，不靠閘道在線）；建立者看到時送 TASK_CLOSE（reason 新增 3 = 過期；4 = optional 角色略過，見下）讓大家都關掉 |
| 簽章 | M0 只保留欄位（issuer、signature 放在 B 層）；驗證放到 session 層、認證一起做（現在「還沒做」清單第 5 項） |
| 取消 | 照舊：建立者或 `cancel_authorities`；閘道的機號要列進全隊的 `cancel_authorities` |

### 版本（revision，CB-5 的基礎）

移動目標、規劃器更新位置、延長期限時，**同一個 task_id 改內容**，不另開新任務（否則得標者、指派版本全部重來）。

| 項目 | 做法 |
|---|---|
| 誰能改 | 只有建立者（閘道）；別人送的較大 revision 不套用、不轉送 |
| 怎麼傳 | TASK_ANNOUNCE 帶 revision；收到較大的 → 取代定義、轉送；相同 → 重複；較小 → 丟掉 |
| 對分配的影響 | 只改位置、時間、價值 → 指派保留（不換指派版本），得標價下次出價時照新內容重算；改了 platform、required_caps、priority → 視為撤銷（指派版本 + 1、重新競標） |
| 補發 | 版本 3 的摘要要把 revision 算進 CRC，缺口請求的狀態加「認得但版本舊」（2 bit → 改成 2 bit 狀態＋另外比 revision） |
| 執行端 | current_task 用同一個 task_id、新的 revision 再發一次；BT 要能「同一個任務、目標更新」而不是當成換任務（要和伯宇對：現在哨兵只比 task_id） |

### 搶占（CB-2、CB-3）

| 項目 | 現在 | 版本 4 |
|---|---|---|
| 級數 | 2 級（火警＞巡檢） | P0～P3；2-1：煙霧、入侵 P0，熱異常 P1，遺留物 P2，例行巡檢 P3。本機安全處置仍在 BT、最高 |
| 誰能搶誰 | 高級搶低級、同級不搶 | 任務的 `can_preempt` 遮罩（預設 = 比自己低的級）；同級要 `preempt_margin` 而且預設關 |
| 不可再搶 | 執行中可中斷（ExecState 2026-10-12 起拿掉，一律可中斷） | 執行者離目標 < commit_radius 後不可搶：**cbba_node 用 robot_state 的位置自己算**（使用者 2026-11-11；不用改 BT），在 AGENT_STATE 報「已接近」，別台據此不搶；離開 commit_radius × 1.5 才解除（遲滯） |
| 鎖定 | 保留／執行中的任務別台不出價 | 不變 |
| 被搶占的 | 交回 CBBA 重新競標 | 不變；狗的日常巡邏點由 BT 自己恢復（伯宇的骨架已有） |
| 搶占時的評分 | 不算被搶占任務的延後、不受 bundle 上限 | 不變 |

### 重新分配

| 情況 | 現在 | 版本 4 |
|---|---|---|
| 執行失敗 | TaskResult.reason 決定排除時間（PERMANENT 永久、TRANSIENT 60 s＋成功率打折、PREEMPTED 5 s、STOPPED 60 s） | 加 2-1b 的 8 個失敗碼（下表），reason 由失敗碼決定；舊的 detail 前綴照舊 |
| 持有者失聯 | 火警 10 s、巡檢 30 s 後撤銷鎖定 | 依優先級：`takeover_by_priority`（預設 P0 10 s、P1 15 s、P2 30 s、P3 30 s） |
| 停止參與 | 手上的立即釋放 | 不變 |
| 過期 | 沒有 | 上面「過期」 |
| 角色組（all_or_nothing、precedence、sync_window） | 沒有 | M0 只定語意；實作是 CB-4（M1～M2）：湊齊閘門、連鎖撤銷、teammates 話題 |

### optional 角色的自動關閉（使用者 2026-11-11：要有明確的條件）

CBBA 不自己刪減角色，所以 optional 角色什麼時候「不做了」要寫成明確的條件，由**建立者（閘道）**送 TASK_CLOSE
（reason 新增 4 = 略過 OPTIONAL_DROPPED）；條件寫在任務定義裡，各節點、閘道判斷結果一樣。

**`membership_ready`**（O1 開始計時的前提）：閘道知道「現在全隊有誰」之後才判斷「沒有能做的載具」，否則閘道剛啟動、
剛從斷線恢復時，還沒收到各台的 AGENT_STATE 就會把 optional 角色誤關。定義：開機同步完成（摘要和任一鄰居相同，或單獨、
逾時開始），**而且**從啟動或從斷線恢復起已經過 `lost_timeout`（1.5 s）＋ 一個 AGENT_STATE 週期（0.5 s），
期間每個存活鄰居至少收過一則 AGENT_STATE。之後鄰居加入、離開不重置，只有閘道自己斷線又恢復時重置。

| # | 條件 | 判斷依據 | 誰送關閉 |
|---|---|---|---|
| O1 | **全隊沒有能做的載具**：`membership_ready` 之後，連續 `optional_no_capable_s`（預設 **10 s**）存活的載具裡沒有一台的 platform、available_caps 符合 | AGENT_STATE 的 platform、available_caps | 閘道 |
| O2 | **有人能做但沒人接**：釋出（建立或上一位持有者交還）後 `optional_unassigned_s`（預設 **15 s**，比 O1 長：弱網時收斂較慢，不要過早關）內沒有正式指派 | CBBA_STATE 的正式指派 | 閘道 |
| O3 | **同組必要角色都結束了**：非 optional 的角色全部完成或取消 | TASK_CLOSE | **只有閘道送正式 TASK_CLOSE**。閘道斷線時各節點只做**本機判定**：停止自己對它出價，**不產生 TASK_CLOSE**、不轉送任何關閉；閘道恢復後由閘道判定並送正式關閉 |
| O4 | 組被取消、過期 | 同現有 | 同現有 |

| 項目 | 做法 |
|---|---|
| 已經有人在做 | O1、O2 只關「沒有正式指派」的 optional 角色。**一般節點的 O3 本機判定不會中途取消正在執行（或已保留）的 optional 角色**：持有者照常做完並回報，只有閘道的正式 TASK_CLOSE（O3、O4）會讓持有者收到 current_task 清空 |
| 對 precedence 的影響 | 由被略過那個角色的 **`skip_satisfies`** 明確決定（範本設定，沒有預設推論）：true → 等它的角色視為條件已滿足、可以開始（例如入侵的 aerial_overwatch：2-1b「沒有 UAV 也不阻塞」）；false → 等它的角色維持等待，由閘道判斷是否照 `fallback_template` 重建任務組 |
| 等待中的角色 | precedence 的前一個是 optional、還沒結束時，後面的角色照樣可以出價、出發（2-1b：狗可以先出發），只有「開始那一步」要等；最多等到前一個完成，或被略過且 skip_satisfies = true |
| 被關掉之後 | **不自動重開**：要加回來由閘道建新的任務（例如 UAV 恢復後重新加入監看） |
| 告警 | 關掉時 `task_orphaned` 帶原因 O1／O2，給值班看 |

**8 個失敗碼 → reason**（REG-2 由伯宇定介面，這張表由 CBBA 定；也寫進註冊表 actions.yaml 的 `failure_codes`）：

| 失敗碼 | 2-1b 的預設處理 | reason | 另外 |
|---|---|---|---|
| E_PRECONDITION | 規劃錯誤，升級值班 | PERMANENT | 發 `task_orphaned` 通知閘道 |
| E_UNREACHABLE | 換觀察點或轉派 | PERMANENT | |
| E_TIMEOUT | 重試一次後轉派 | TRANSIENT | 成功率打折已涵蓋「重試」 |
| E_TARGET_LOST | 另一台接手 | TRANSIENT | |
| E_LOW_BATTERY | 釋放、返航 | STOPPED | |
| E_SAFETY_STOP | 停止，升級值班 | STOPPED | |
| E_COMM | 自主繼續或安全停止 | TRANSIENT | |
| E_HW | 標記能力不可用，轉派 | PERMANENT | 同時移除 `failed_capability`（第一節） |

---

## 四、分配結果交給執行端的介面

照現在的做法（cbba_node 發 current_task、收 task_result），訊息升 `swarm_interfaces` v2.0，**舊欄位全部保留、只加欄位**，
伯宇的 BT 重新編譯就能跑（不讀新欄位）。

| 話題 | 方向 | 版本 4 的改動 |
|---|---|---|
| `current_task`（Task） | CBBA → BT | 加：revision、group_id、role、priority、platform、required_capabilities（string[]）、floor、release／expires 時刻、`spec`（B 層 TaskSpec JSON，含 steps） |
| `task_result`（TaskResult） | BT → CBBA | 加：failure_code（string，E_*）、failed_capability（string）、findings_uri（只轉給閘道）；reason 照舊（沒帶 failure_code 時照舊用 reason） |
| `teammates`（新） | CBBA → BT | 同一個 group 其他角色的持有者、位置、狀態（未分配／前往／到位／完成），CB-4 的 precedence、sync_window 用 |
| `task_closed`、`task_announced`、`task_orphaned` | CBBA → BT／閘道 | 帶同樣的新欄位；`task_orphaned` 加原因（沒有能力、沒人得標、過期） |
| `assignments`（新，閘道用） | CBBA → 閘道 | 每個任務的得標者、正式指派（版本、執行者、狀態）、revision；取代「GCS 直接聽 UDP 14600」（現在待決定） |
| `robot_state` | BT → CBBA | 不變；commit_radius 的「已接近」由 cbba_node 用它的位置算 |

ExecuteSteps（伯宇）從 `current_task.spec` 取 steps；CBBA 不解析 steps。

**TaskSpec 的格式與 BT 的接口（使用者 2026-11-13）**：外部 task manager（閘道）把每一種突發任務要做的事先固定成一套 tasks，
注入時放進 spec；BT 收到後照它建外部點的 tasks 清單，交給現有的 SplitTask、Switch 依 action 分派（＝2-1b 的 ExecuteSteps 雛形）。
spec 的格式直接沿用狗 BT 任務檔的 `waypoints[].tasks`（伯宇的 `config/MISSION_FORMAT.md`），外面包一層 `tasks`：
`{"tasks": [{"action": "ptz_inspect", "target": {"x": 12, "y": 5, "z": 1.5}, "snapshots": 2}]}`。站的位置用
`current_task.position`（spec 不寫 go_to）；`report_result` 由 BT 補；spec 空的照舊型流程；spec 不能用回 `BAD_SPEC`（PERMANENT）。
狗：`docker/dog/task_manager_spec.patch`；無人機的骨架副本之後照同樣做。cbba_node 只在快取裡有這個 revision 的
TaskSpec（或任務沒有 TaskSpec）時才發 current_task，所以 BT 拿到的任務一定有完整的 steps（第二節「TaskSpec 的傳送」）。

---

## 五、待決定（要使用者決定的）

| # | 項目 | 選項 | 建議 |
|---|---|---|---|
| ~~1~~ | B 層（steps）怎麼到執行端 | **定案（11/11）**：獨立的 TASK_SPEC、各台快取、沒有 TaskSpec 不開始執行（第二節） | — |
| ~~2~~ | 同級分數換不換 | **定案（11/11）**：暫時沿用現在的公式；priority 分層比較，同級才比分數，同分比機號（第二節） | — |
| ~~3~~ | commit_radius 的「已接近」由誰判斷 | **定案（11/11）**：cbba_node 用 robot_state 的位置算 | — |
| ~~4~~ | 舊 TaskType | **定案（11/11）**：保留到 M3 | — |
| ~~5~~ | CBBA_STATE 紀錄裡的正式指派三欄 | **定案（11/11）**：保留，每筆統一 27 B | — |
| ~~6~~ | 沒有 TaskSpec 時 | **定案（11/11）**：可以出價、進 WAITING_SPEC 不執行；`spec_wait` 超時只補發＋告警，不算失敗 | — |
| ~~7~~ | 閘道斷線時 | **定案（11/11）**：機器人不自己展開任務，繼續舊任務、暫存偵測；fallback 只在閘道在線時重建 | — |
| ~~8~~ | optional 角色的自動關閉條件 | **定案（11/11）**：O1 10 s（membership_ready 後計時）、O2 15 s、O3 斷線時只停本機出價、不產生 TASK_CLOSE、不中途取消執行中的；precedence 看 skip_satisfies；不自動重開 | — |

---

## 六、task_id：64 位元（2026-11-11 定案）

### 格式

```
task_id (uint64) = creator (16) | epoch (16) | seq (32)
   creator  建立者機號（1～254；0 保留 = 無人）
   epoch    建立者的「世代」：每次啟動 + 1，存檔（見下）；0 保留給舊型 ID
   seq      這個世代內的流水號，從 1 開始；0 保留
```

**舊型 ID 相容模式**（使用者 2026-11-11 定案）：上半 32 位元是 0 的 id 是舊型 ID，數值和版本 3 完全一樣
（`機號 × 65536 + 流水號`：建立者在 16～31 位元、流水號在 0～15 位元），視為 **epoch = 0**。

| | 舊型 ID（epoch 0） | 版本 4 ID（epoch ≥ 1） |
|---|---|---|
| 誰產生 | BT 自己算（現在的 `new_task`、itest、假狗、伯宇的狗 BT） | **只由 cbba_node／閘道產生**（不新增 `cbba_epoch` 話題，BT 不用知道世代） |
| 判斷 | 上半 32 位元 = 0 | 上半 32 位元 ≠ 0（建立者 ≥ 1，所以不會和舊型混淆） |
| 重開機重用 | 照版本 3：開機同步擋下（`rejected_local_tasks`） | 換世代，不會撞 |
| 期限 | 保留到 M3 | — |

**狗端的通訊由使用者統一負責**（2026-11-11）：第 2 步時狗和無人機一起改，current_task、BT 內部的任務識別（`externalTaskId`、
`current_task_id`、外部點名稱）、task_result **全部改成 uint64**；BT 的 MissionTree、SplitTask、Switch、Action 流程不動。
要改的：我們的無人機 BT（骨架副本、`uav_task_manager_node`、`px4_platform`、備用 BT）、伯宇的狗 BT 的介接層（`dog_ws` 編譯的那份）、
`fake_dog.py`、itest。

| 原本的問題 | 怎麼解決 |
|---|---|
| 號碼用完（每個機號一輩子 65535 個） | 每個世代 40 億個，世代 65535 個 |
| 重開機重用（要存流水號、撞號被開機同步擋下） | 重開就換世代，流水號從 1 重來也不會撞；只要存「世代」一個數字，啟動時讀、+ 1、寫回，**寫回成功才開始建任務** |
| 機號重複（兩台同機號 → 同一個 task_id） | 世代仍可能相同，所以另外**偵測**：同一個機號同時有兩個 session 在講話（AGENT_STATE、CBBA_STATE 的 session_id 不同、時間重疊）→ 兩邊都印錯誤、`task_orphaned` 之外再發 `agent_conflict` 給閘道；較晚出現的那個 session 建立的任務拒收 |

### 世代存在哪裡

| 節點 | 檔案 | 讀不到時 |
|---|---|---|
| 閘道 | `<state_dir>/epoch_<機號>`（參數 `state_dir`，預設 `/var/lib/cbba`） | 拒絕啟動（閘道是主要的建立者，不能冒險） |
| 機器人 | 同上；docker 用主機掛載的 `/run/cbba`（和程序鎖同一個目錄） | 用隨機世代（1～65535）並印警告；機器人很少建任務，撞到的機率很低 |

世代到 65535 後回到 1（實際上不會用到；每天重開 10 次要 18 年）。

### 實作（2026-11-12，第 2c 步）

| 項目 | 程式 |
|---|---|
| 世代 | `cbba_node` 參數 `state_dir`、`epoch_required`；`CommConfig::epoch`；`types.hpp` 的 `nextEpoch` |
| 版本 4 的 task_id | `CbbaComm::newTaskId()`（建立者自己、這次的世代、流水號從 1 起；給第 4 步的閘道注入用） |
| 同機號偵測 | `CbbaComm::trackSession`：同一個機號的兩個 session，較早的在較晚的出現 `conflict_grace`（1 s）之後還在講話 → 衝突，`SessionFilter::pin` 指回較早的，較晚的封包、它建立的任務（含別台轉送、SYNC_BATCH 補來的）都拒收；較早的 `conflict_release`（5 s）沒聲音就解除、改接受較晚的。別台用了自己的機號（loopback 收到別的 session）由 `selfConflict()` 回報。cbba_node 印 ERROR、發 `agent_conflict`（std_msgs/UInt16，衝突的機號） |

### 實作（2026-11-12，第 3 步：TaskSpec）

| 項目 | 程式 |
|---|---|
| 封包 | `wire_task_spec.hpp`：TASK_SPEC（種類 9，轉送）＝ 表頭 20＋轉送 14＋task_id 8、revision 2、encoding 1、第幾段 1、共幾段 1、長度 2＋資料（≤ 1100 B）。encoding 0 原文；1 zlib 保留 |
| 建立 | `CbbaComm::addLocalTask(task, now, spec)`：填任務定義的 spec_length、spec_crc（CRC-32），任務定義之後每段一則 |
| 收 | 只收 origin 是建立者的段；各段到齊、任務定義也到了才比對長度與 CRC（不符整份重收，`spec_bad`）；比已知 revision 舊的丟掉 |
| 補發 | GAP_REQUEST 點陣圖的值 3 = 進行中但缺 TaskSpec；缺的任務第一次發現後每 `spec_pull_period`（1 s）向一個鄰居要（建立者聽得到就問建立者），WAITING_SPEC 超過 `spec_wait` 改每 `gap_request_gap` 向每個鄰居要；回覆用 SYNC_BATCH，段排在進行中任務的定義之後、已結束的之前 |
| WAITING_SPEC | `AssignmentManager`：排第一、assign_hold 已過但 `specReady` 是 false → 不建立正式指派、不發 current_task；`waitingSpec()`；AGENT_STATE exec_flags bit 4；佇列裡的火警（保留）也要收齊才保留 |
| 執行端 | `current_task.spec`（Task.msg 加 `revision`、`spec`）；`new_task` 帶 spec 的話一起送；超過 `spec_wait`（參數，5 s）發 `task_spec_missing`、印警告，每 `spec_wait` 一次 |
| 快取 | 任務結束 `spec_keep_sec`（60 s）後丟掉內容 |

### 實作（2026-11-12，第 4a 步：閘道的注入）

| 項目 | 程式 |
|---|---|
| 注入 | `gateway.hpp/.cpp` 的 `Gateway::inject`（不依賴 ROS）：欄位檢查（平台、P0～P3、角色組、precedence、時間、spec 大小）、冪等鍵 (incident_id, role, revision)、`newTaskId()` 配 task_id；同一個 incident 有角色組時配一個 group_id（格式同 task_id、共用流水號）；交給 `addLocalTask`（還沒和鄰居同步時排隊）。對照表只在記憶體：閘道重開後同一事件再注入是新任務 |
| 節點 | `cbba_node` 的 `participate: never`（不訂 robot_state、永不出價）時提供 `inject_task`（`InjectTask.srv`，時間欄位是從注入起算的秒數）、發 `assignments`（`TaskAssignments.msg`，開著的任務；`publish_assignments`）；`config/cbba_gateway.yaml`（機號 200、`epoch_required`）；`cbba_uav.yaml`、`cbba_v60.yaml` 的 `cancel_authorities: [200]` |
| 還沒做 | 改版（較大的 revision，4b）；過期（4c）；失聯接手依優先級、commit_radius（4d）；optional 關閉 O1～O4（4e）。注入的新型任務（type 0）BT 還不會執行（伯宇的 ExecuteSteps 在 M1～M2） |

### 實作（2026-11-13，第 4b 步：改版）

| 項目 | 程式 |
|---|---|
| 建立者改版 | `CbbaComm::reviseLocalTask(task, now, spec)`：自己建立、還開著、revision 變大才行；新的 origin 送 TASK_ANNOUNCE＋這個 revision 的 TASK_SPEC。閘道：`inject_task` 同 (incident, role)、較大的 revision → `REVISED`（同一個 task_id；期限從原本的建立時刻換算；role_index、role_count 不能改） |
| 收到 | 較新的 revision 只收建立者送的（`rejected_revisions`，不轉送）；比已知舊的丟掉、不轉送。`applyRevision`：換內容、origin 換新版的（保底重播、補發用新版）、`added_at_` 更新（直接推送） |
| 對分配的影響 | 種類、平台、能力、優先級（effective*）都沒變 → `CbbaAgent::updateTask(…, false)`：得標者、得標價、正式指派保留，手上有的照新內容重算（`reevaluate`，做不到就釋放）。有變 → `updateTask(…, true)`：清掉得標者、自己手上的釋放、重新出價；**持有者自己撤銷**指派（`endAssignment`，新版本、執行者 0），別台的鎖等撤銷的紀錄傳到（持有者失聯照失聯接手）（`reauctions`） |
| 補發 | 摘要、round_key 把進行中任務的 revision 算進 CRC；CBBA_STATE 紀錄的 revision 比已知新 → 每 `gap_request_gap` 向那個鄰居送只有那個流水號（標成不認得）的缺口請求，對方回新版的任務定義與 TaskSpec（`revision_pulls`） |
| TaskSpec | 新 revision 沒有 TaskSpec → 丟掉舊的快取（`taskSpec` 回空字串）；有 → 照原本的分段收齊、比對 CRC |
| 執行端 | `AssignmentManager`：執行中的任務改版（指派保留）且新 revision 的 TaskSpec 收齊 → 重發 current_task（同一個 task_id、新的 revision）；收齊前 BT 照舊版繼續 |
| 還沒做 | BT 收到同一個 task_id、新 revision 的 current_task：狗的 BT（`if (t.task_id == cur) return;`）和無人機骨架目前都當成重送忽略、照舊版做完。要換目標時要中斷再插入，和伯宇對 |

### 實作（2026-11-13，第 4c 步：過期）

| 項目 | 程式 |
|---|---|
| 封包 | TASK_CLOSE 的 reason 收 1～4：3 = 過期、4 = optional 略過（4e 用，先一起開放，4e 不用再改協定）。3、4 在本機的狀態是 CANCELLED（`Task.msg` 只有 DONE／CANCELLED），原因用 `CbbaComm::closeReason()` 查。權限同取消（建立者或 `cancel_authorities`） |
| 本機判定 | `CbbaComm::expireTasks`（poll 裡每 0.1 s）：開著、`ext.expires` 到了 → 永久不再出價（`excludeTask`）、撤銷自己的指派（已回報完成、在等確認的不撤）；`expired()`、`takeExpired()`（`expired_local`） |
| 建立者關閉 | 建立者（閘道）送 TASK_CLOSE reason 3（`expired_closes`）；建立者離線期間過期的，回來後看到就送。在那之前任務開著、沒人出價（不當成孤兒任務） |
| 節點 | log「任務 … 過期（expires 到了）」「任務 … 已過期（建立者關閉，reason 3）」；孤兒任務檢查略過過期的 |

### 「統一由閘道建立任務」

使用者 2026-11-11：之後 CBBA 任務**統一由閘道建立**（creator = 閘道的機號）。和 2-1b 第七節一致：機器人鏡頭的偵測
（無人機看到火、狗看到火）也是 Detection，送進事件流程 → 規劃器 → 閘道注入，不再由機器人自己的 cbba_node 建任務。

| 項目 | 做法 |
|---|---|
| 正常 | 機器人的偵測送 `detection` 給閘道；閘道建任務（creator = 閘道） |
| 閘道斷線時（使用者 2026-11-11 定案） | **M0 起機器人不自己展開任務**（也不展開 fallback）：已知的任務照常競標、執行、完成（CBBA 在機器人之間照常運作）；新的偵測**暫存**在機器人的 cbba_node，閘道恢復後依序送出，由閘道決定要不要建任務 |
| 斷線的判定 | 機器人的 cbba_node 超過 `gateway_timeout`（預設 3 s）沒聽到任何閘道（參數 `gateways` 列出的機號）的 AGENT_STATE → 斷線；聽到 → 恢復、送出暫存的偵測 |
| 偵測怎麼送到閘道 | DDS 只在機內，機器人上碰 mesh 的只有 cbba_node → 新封包 DETECTION（機器人 → 閘道，轉送），cbba_node 收機內話題 `detection` 轉成封包；斷線時進暫存佇列（`detection_buffer` 筆數上限，滿了丟最舊的、同一個 track／同一個位置合併）。Detection 的欄位照 2-1b 第七節，和 EVT-1 一起定 |
| 閘道恢復 | 收到暫存的偵測：照時間戳記排進事件狀態機（去重、信心融合），過期的照事件規則丟掉 |
| 簡化的任務組 | 只有**閘道在線時**可以照 `fallback_template` 取消原本的組、重建較簡化的一組（例如 UAV 缺席時入侵改 `single_dog_verify`）；CBBA 不自己刪減角色 |
| 取消 | 閘道的機號列進全隊的 `cancel_authorities` |
| 代價 | 斷線中看到的新事件，要等閘道恢復才有任務（第 5 週的 100% 斷網情境裡「斷網中看到火 → 狗接手」不再成立）。舊型任務（BT 的 `new_task`，TaskType）保留到 M3，期間 itest 的火警流程照舊 |
| 多個閘道 | 各用自己的機號（例如 200、201）；task_id 不會撞 |


### 要跟著改的地方

| 地方 | 改動 |
|---|---|
| `types.hpp` | `TaskId` → `std::uint64_t`；`makeTaskId(creator, epoch, seq)`、`makeLegacyTaskId(creator, seq)`、`taskCreator()`、`taskEpoch()`（舊型 0）、`taskSeq()` |
| CBBA_STATE 紀錄 | task_id 4 → 8 B、加 revision 2 B：每筆統一 **27 B**（第二節） |
| 版本 3 的摘要 | 摘要的鍵從「建立者」改成（建立者, 世代）；每筆固定 **24 B**（使用者 2026-11-11）：creator u16、epoch u16、min_seq u32、max_seq u32、known_count u32、closed_count u32、crc u32，**逐欄序列化（大端序），不用 sizeof(struct)**。舊型 ID 的鍵是（機號, 0）。同一個建立者的舊世代任務全部結束後，摘要就固定不變 |
| GAP_REQUEST | 帶（建立者, 世代）、起始流水號 uint32；點陣圖照舊每個流水號 2 bit |
| TASK_ANNOUNCE、COMPLETION／ACK、TASK_CLOSE、SYNC_BATCH | task_id 欄位 8 B |
| 開機同步 | 「查出 BT 重開機後重用 task_id」的那段（`rejected_local_tasks`）在有世代後不再需要；保留當成最後防線 |
| `swarm_interfaces` | `Task.task_id`、`TaskResult.task_id`、`Bid.task_id` uint32 → **uint64**（v2.0）。**伯宇的 BT 要改**：handoff 的 task_manager 約 12 處 `uint32_t task_id`（`externalTaskId`、`current_task_id`…）；我們的骨架副本、`uav_task_manager_node`、`px4_platform`、itest、`fake_dog.py` 一起改 |
| BT 的外部點名稱 | `cbba_<task_id>` 照舊（字串，位數變長而已） |
| log、出價表、動畫 | task_id 印成 `creator.epoch.seq`（例如 `200.3.17`）比十進位的 64 位元好讀；解析時兩種都收 |
