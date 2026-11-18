# 任務定義的關聯性：載具適合性、航行限制、前後順序（依〈2-1〉第二節補齊）

2026-11-17 · 來源：〈2-1.監視事件轉 CBBA 任務：可靠生成與逐步驗證〉第二節「任務定義」（欄位總表、入侵子任務範例、CBBA 端需要處理的事）。
使用者 11/17：第二節講的是**任務之間、任務和載具之間的關聯**——判斷載具是否適合、航行限制、前後順序；目前只做到淺層，照這份補齊。
任務規劃器本身（狀態機、範本展開）只做初步，放在第 5 節。

---

## 第二節欄位總表與欠缺項目（對照 11/17 的 hetero-tasking）

前四欄照〈2-1〉第二節的欄位總表；「欠缺的項目」是目前實作還沒有或只做到一部分的地方（✅ 已有的不列）。

| 群組 | 欄位 | 說明 | 用在哪裡 | 欠缺的項目 |
|---|---|---|---|---|
| 識別與追溯 | `task_id`、`incident_id`、`group_id`、`revision`、`schema_version`、`source` | revision 單調遞增；同一 task_id 只接受較大的 revision | 冪等注入、追溯 | 已有：task_id（64 位元）、group_id、revision（只收較大的）、閘道以 (incident_id, role, revision) 冪等注入。**欠缺**：任務本身不帶 `incident_id`（只在閘道）、沒有 `source`；`schema_version` 只在 TaskSpec schema、cbba_node 不檢查 |
| 任務語意 | `task_type`、`event_type`、`role`、`description` | task_type 決定 BT 子樹，例如 confirm_intrusion、thermal_recheck | BT | **欠缺**：新型任務的 type 是 NONE，沒有 `task_type`／`event_type`；角色只有 `role_index`，名稱只在注入請求與假 Planner 的 spec；沒有 `description`。BT 目前用 spec 的 steps，不用 task_type 選子樹 |
| 證據 | `evidence`：攝影機 ID、截圖、短片、VLM 結果、信心 | 只放參照，不放影像本身 | 控制中心、結果回傳 | **欠缺**：整欄沒有。合約定在 C 層（留在閘道），還沒做；只有 TaskResult 的 `findings_uri`、Detection 的 `attrs` |
| 空間 | `target`（frame、floor、x、y、z）、`target_kind`（point／area／moving）、`uncertainty_m`、`observe_pose`、`standoff_m` | 移動目標附 track_id 與更新頻道 | CBBA 出價、Nav2 | 已有：位置 x、y、z、floor（A 層）；incident state 有位置、半徑、track_id。**欠缺**：`floor` 只傳不用（不比對載具樓層）；沒有 `target_kind`；任務沒有 `uncertainty_m`、`observe_pose`、`standoff_m`（出價算到目標點而不是觀察點，不檢查安全距離）；移動目標的小位移（< 3 m）不重新拍賣的門檻 |
| 區域限制 | `allowed_zones`、`forbidden_zones` | 高壓區、無人機禁飛區等 | 出價可行性、IsZoneAllowed | **欠缺**：整欄沒有。`site-config/yard/zones.geojson` 已有各區的 `no_entry`（狗不能進高壓區、儲槽區…），但 CBBA 不讀、出價不檢查目標點與路徑、任務不能附帶禁止區域、BT 收不到 |
| 時間 | `release_time`、`deadline`（soft／hard）、`service_time_s`、`expires_at`、`reward_decay` | 過期未完成自動失效 | CBBA 分數 | 已有：release、deadline＋hard、duration、expires（到期本機停止出價、閘道關閉）、`half_life_s`（TDR）。**欠缺**：`reward_decay` 只有指數型（half_life），沒有其他型別（目前也不需要） |
| 優先與搶佔 | `priority`（P0–P3）、`can_preempt`、`preempt_margin`、`commit_radius_m` | 見下方搶佔規則 | CBBA、中斷哨兵 | 已有：P0～P3（先比優先級）、can_preempt 遮罩、commit_radius。**欠缺**：`preempt_margin` 有傳但 `mayPreempt` 沒用（沒有搶占遲滯）；P0 沒人得標的看門狗是 30 s（2-1 要 10 s） |
| 能力需求 | `required_capabilities`、`platform`、`min_battery_reserve` | 例如 thermal、zoom、indoor_flight | 出價可行性 | 已有：能力過濾、平台、任務的電量下限、故障能力扣除。**欠缺**：`indoor_flight` 只是標籤，區域沒有「室內」屬性、不會自動要求；載具的 `max_altitude_m` 不檢查 |
| 協同約束 | `sync_window_s`、`precedence`、`all_or_nothing`、`fallback_template` | 包夾要同時到位；無人機先定位再派狗 | CBBA／執行端同步 | 已有：執行端 gate（precedence、sync）、Group Commit（all-or-nothing）、distinct、optional＋skip_satisfies、角色組的 fallback。**欠缺**：出價不考慮順序與會合（依賴的角色不算等前一個角色的時間、不看和隊友的到達差）；單獨任務沒有 fallback |
| 執行規格 | `steps`、`bt_subtree`、`params`、`success_criteria`、`abort_conditions`、`timeout_s` | 範本事件可直接指定 bt_subtree；開放事件由 steps（動作＋物件＋位置）組成；成功條件要可量測 | BT | 已有：TaskSpec 的 steps（通用動作、物件、位置、params、wait_for、sync），`actions.yaml` 有各動作的 timeout_s。**欠缺**：`bt_subtree`、`success_criteria`、`abort_conditions`、任務層級的 `timeout_s`；參數照 `actions.yaml` 的檢查（檢查器）還沒做 |
| 回報要求 | `report`：是否串流、截圖週期、回報頻道 | | 影像整合、VMS 回寫 | **欠缺**：整欄沒有（TaskResult 只有 findings_uri） |
| 人機與安全 | `requires_approval`、`autonomy_level`、`safety_notes` | 例如煙霧任務只在安全距離外觀察，不進入煙霧區 | 控制中心 | 已有：`actions.yaml` 的 `risk`、`implementations.autonomy`。**欠缺**：任務沒有這三個欄位；沒有「核准前不注入」的流程；「不進入煙霧區」要靠區域限制（也欠缺） |
| 簽章 | `issuer`、`signature` | 只有閘道憑證能發告警任務 | 資安 | 已有：新型任務只由閘道建立、取消要 `cancel_authorities`。**欠缺**：沒有 `issuer` 欄位、沒有密碼學簽章與 TLS |

## TaskCore 最終定義與逐列規劃（2026-11-17 使用者定）

依使用者提供的〈CBBA 任務欄位逐列實作規劃〉（16 頁，13 列；三層：TaskCore＝A 層 TASK_ANNOUNCE、TaskSpec＝B 層、
IncidentRecord＝C 層留在閘道／控制中心）。**只規劃，還沒實作。**

### TaskCore（TASK_ANNOUNCE）最終定義：124 B → 128 B

```
TASK_ANNOUNCE 124 B
+ zone_policy_id         2 B
+ zone_registry_version  2 B
= 128 B
```

CBBA 用這些資訊分配：平台、搭載能力、樓層、禁飛／禁入區、observe pose、任務價值、到達與執行時間、hard deadline、half-life、
電量與返航存量、優先級與搶占、distinct／optional／AON／precedence／sync。

| 項目 | 欄位 | 狀態 |
|---|---|---|
| 平台、能力 | `platform`、`required_caps` | 已在用 |
| 樓層 | 任務 `floor`＋**載具目前樓層** | 任務 floor 只傳不用；載具樓層沒有來源 → 單一樓層先用 `fleet.yaml`，多樓層再改 RobotState |
| 禁飛／禁入區 | **`zone_policy_id`**＋**`zone_registry_version`**＋各節點本機的區域註冊表 | 新增：檢查 observe pose 所在區域對這個平台是否允許、高度上限 |
| observe pose | `x, y, z`（規劃器設成該角色的觀察點；事件位置移到 TaskSpec） | 不改格式 |
| 價值、時間、期限、half-life、電量 | `value`、`release_ms`、`duration_sec`、`deadline_ms`＋hard、`half_life_s`、`min_battery_reserve` | 已在用 |
| 優先級與搶占 | `priority`、`can_preempt`、`preempt_margin_pct`、`commit_radius_dm` | `preempt_margin_pct` 只傳不用，要開始用 |
| 協同 | `group_id`、`role_index`／`role_count`、`distinct_agent_set`、flags（optional、AON、skip_satisfies）、`precedence`、`sync_window_s` | 已在用（precedence、sync 在執行端；出價時考慮是第二階段） |

**決定（使用者 11/17，最終）**

1. **TASK_ANNOUNCE 升為協定版本 5，固定 128 B**：和版本 4 互相拒收；全隊（含狗的 cbba_node）一起重新編譯（BT 不受這 4 B 影響）。
2. **加入 `zone_policy_id` 與 `zone_registry_version`**：`zone_policy_id` 是區域註冊表裡事先定義的政策（例：「uav：禁止 Z_TANKS」
   「煙霧觀察：所有平台禁止 Z_x」），0 = 只套用區域本身的 `no_entry`；執行中才出現的區域（煙霧擴散範圍）由規劃器把觀察點移到安全處。
   `zones.geojson` 加版本欄位，內容改了就遞增。
3. **Zone Registry 版本不符**：第一階段**先告警**（log＋話題）、照常出價；是否禁止出價後續定案。
4. **補上載具樓層的來源**：單一樓層先用 `fleet.yaml`（每台的樓層）；多樓層時再改 RobotState（加 `floor`）。
5. **本次必須實際啟用 `floor` 與 `preempt_margin_pct`**：不接受只序列化而未使用。
   - `floor`：任務樓層 ≠ 0 且 ≠ 載具樓層 → 不可行（WRONG_FLOOR）
   - `preempt_margin_pct`：同級搶占要遮罩允許，而且新分數 > 被搶任務目前的分數 ×（1 + margin）

**不放進 TaskCore（放 TaskSpec／閘道）**：incident_id、source、schema_version、event_type、task_type、role 名稱、事件位置、target_kind、
uncertainty_m、standoff_m、steps、success／abort 條件、timeout、report_policy、requires_approval、evidence、簽章。
閘道重開沿用原 task_id 靠閘道把 (incident_id, role) → task_id 對照存檔，和封包格式無關。

### 13 列的規劃（照 PDF 的順序）

| 批 | 列 | 要做的（摘要） | 放哪裡 |
|---|---|---|---|
| 1 合約骨架 | 01 識別與追溯 | TaskSpec 帶 incident_id、source（0 UNKNOWN、1 ALARM、2 UAV、3 QUADRUPED、4 VMS、5 OPERATOR、6 SENSOR）、schema_version；閘道對照表＋序號存檔，重開沿用 task_id；冪等鍵 (incident_id, role_index)，revision 不換 task_id | TaskSpec、閘道 |
| | 02 任務語意 | event／task／role 的編號註冊表（只增不改）、名稱放 TaskSpec；正式採 steps 驅動，`bt_subtree` 不採用；assignments 與 log 顯示名稱＋編號 | 註冊表、TaskSpec、閘道 |
| | 04 空間資訊 | 規劃器產生 observe pose（安全距離、區域）放進 `x, y, z`；target、target_kind（point／area／moving）、track_id、uncertainty_m、standoff_m 放 TaskSpec；位移小於門檻只更新導航，超過才 revision + 1 | 規劃器、TaskSpec、閘道 |
| 2 安全可行性 | 05 區域限制 | 共用 ZoneRegistry（C++，規劃器讀同一份 geojson）；CBBA 檢查 observe pose 的區域、平台、樓層、高度（不可行原因 ZONE_FORBIDDEN、WRONG_FLOOR、ALTITUDE_LIMIT）；路徑交給 Nav2／UAV 規劃，走不到回報失敗碼；TaskCore 加 zone_policy_id、zone_registry_version（協定版本 5）；載具樓層來源（先 fleet.yaml）；floor 啟用 | cbba_core、swarm_cbba、site-config、swarm_interfaces |
| | 08 能力需求 | 室內區域自動要求 indoor_flight、熱像等由規劃器推導；高度上限檢查；感測器故障撤銷能力並重新分配（已有 E_HW，補能力恢復與測試） | 規劃器、cbba_core |
| | 07 優先級與搶占 | `preempt_margin_pct` 生效（同級：遮罩允許且新分數 > 目前 ×(1 + margin)）；閘道的單一 P0 無人得標看門狗 10 s（和 group_commit_timeout 分開）→ 升級／fallback | cbba_core、閘道 |
| 3 正式執行 | 10 執行規格 | steps 為唯一執行介面；TaskSpecValidator（閘道注入前：動作、參數、location／object、步數上限、平台支援、wait_for 引用）；結構化 success／abort 條件；任務層級 timeout；狗與 UAV 的 8 個動作支援表 | 閘道、BT／adapter |
| | 12 人機與安全 | 閘道的核准狀態 PROPOSED → WAITING_APPROVAL → APPROVED → INJECTED／REJECTED；核准前不建 CBBA 任務；高風險動作強制核准；保存核准紀錄；safety_notes 只顯示，真正限制轉成 zone／standoff／能力／中止條件 | 閘道、TaskSpec |
| 4 外部整合 | 11 回報要求 | TaskSpec 的 report_policy；TaskResult 加 completed_steps、attachments、final_position；結果 outbox | swarm_interfaces、BT、閘道 |
| | 03 證據 | 證據存放與 URI；TaskSpec 只帶 evidence_uri＋confidence | 閘道／控制中心 |
| 5 部署 | 13 簽章與資安 | 簽章信封（issuer、key_id、nonce、payload_hash、signature），PoC HMAC、正式 Ed25519；只簽有權限意義的封包（TASK_ANNOUNCE、TASK_SPEC、TASK_CLOSE、取消、INCIDENT_UPDATE），轉送時簽章只涵蓋原始 payload；nonce 沿用 (session, seq) | cbba_core |
| 整理（✅ 11/18） | 06 時間 | 把現有的絕對時刻（ms，chrony 對時）寫成合約；補時鐘偏差、亂序測試 | 文件、測試 |
| | 09 協同出價（第二階段） | 有效到達時間 = max(自身、前置里程碑、同步)；單獨任務的 fallback 由閘道管理 | cbba_core、閘道 |

每列的完成定義（照 PDF）：合約（型別、單位、預設、版本相容、負責模組）→ 程式（不留只傳不用的欄位）→ 單元、節點介面、至少一個失敗情境
→ itest group G0～G15 不退化、封包大小沒有非預期增加 → schema／enum／錯誤碼／範例同步更新。

### 第一批的實作（2026-11-17，01、02、04；11/18 單元、`meta_iface.sh` M1～M7、回歸腳本、itest group／comm 全過）

| 列 | 做法 | 程式 | 測試 |
|---|---|---|---|
| 01 識別與追溯 | `InjectTask` 加 `source`（固定列舉 0～6）；閘道把 incident_id、source、schema_version（角色 spec `"1.1"`）寫進 TaskSpec 最上層。冪等鍵照舊 (incident_id, role 名稱)（同一個 incident 裡和 role_index 一一對應）。**對照表存檔** `<state_dir>/gateway_<id>.state`（參數 `gateway_state_file`，內容變了才寫）：重開後讀回，任務從鄰居補回來之前同一個鍵拒絕（不建第二個），補回來之後同一個 task_id；`restore_confirm_s`（10 s）還沒補回來 = 全隊都不記得（整隊重開）→ 丟掉那筆、之後是新任務 | `cbba_core/task_meta.*`、`gateway.*`（`saveState`／`loadState`／`confirmRestored`） | `test_gateway` Identity 4 項、`meta_iface.sh` M1、M7 |
| 02 任務語意 | `registry/task_types.yaml`（task_types、roles 的「名稱 → id」）＋ `event_types.yaml` 的 `id`（只增不改，0 = 不知道）；`InjectTask` 加 `event_type`、`task_type`、`description`；閘道有 `registry_dir` 時名稱 → 編號、不認得的拒絕，沒有時編號 0；名稱＋編號寫進 TaskSpec、`TaskAssignment`（只有閘道填）、注入的 log。steps 驅動，`bt_subtree` 不採用 | `registry.*`、`cbba_node.cpp`（`resolveSemantics`、`semanticLabel`）、`TaskAssignment.msg` | `test_registry` SemanticIds、`meta_iface.sh` M1～M3 |
| 04 空間 | `position` ＝ 觀察點（observe pose，出價算到這裡）；`InjectTask` 加 `target_kind`（point／area／moving）、`target`、`track_id`、`uncertainty_m`、`standoff_m`，寫進 TaskSpec（`observe_pose`、`target`…）；閘道檢查觀察點離目標 ≥ standoff（area 從邊緣算）、moving 要 track_id。**移動目標**：只動位置、位移 < `retarget_threshold_m`（3 m，CommConfig，全隊一樣）的改版 → 換成新內容、不重新評估（`CbbaAgent::updateTask` 的 rescore = false，`stats.retargets`）；改版時相對時間沒變就沿用絕對時刻 | `task_meta`、`cbba_comm.cpp`（`smallMove`：比對封包的任務定義）、`gateway.cpp` | `test_task_meta` 6 項、`test_gateway` Identity 2 項＋Retarget 2 項、`meta_iface.sh` M4、M6 |

和原本規劃不同的地方：
- P7「位移 < 3 m 只更新 incident、不改任務 revision」改成「revision 照樣 + 1、CBBA 不重新評估」：單獨任務沒有 incident 頻道，
  BT 要靠新的 revision 拿到新位置；得標者一樣不換人（驗收「小幅移動不換人，大幅移動才重新評估」照樣成立）。
- TaskSpec 的組合在節點層（`swarm_cbba/spec_compose.*`，用 yaml-cpp 檢查規劃器的 spec）；規劃器的 spec 是空的時不產生
  只有 meta 的 spec（狗的 BT 看到沒有 steps 會回 BAD_SPEC），meta 只留在閘道。
- 協定封包沒改（還是版本 4、124 B）；`swarm_interfaces` 2.4.0（InjectTask、TaskAssignment 加欄位，要全隊重編、狗的 BT `dog_bt.sh build`）。

### 第二批的實作（2026-11-18，05、08、07；協定版本 5；程式寫好，**測試由使用者跑**）

| 列 | 做法 | 程式 | 測試 |
|---|---|---|---|
| 05 區域 | **協定版本 5**：TASK_ANNOUNCE 加 `zone_policy_id`、`zone_registry_version`（124 → 128 B；版本 4、5 互相拒收）。`zones.geojson` 加 `version`、`policies`（id、forbid：區域 × 平台）、每區 `floor_id`、`indoor`。核心 `cbba_core/zones.*`（`ZoneRegistry`、`checkZones`）：只看觀察點（task.position）——樓層不同（WRONG_FLOOR）、超過載具的 `max_altitude` 或區域的天花板（ALTITUDE_LIMIT）、區域 no_entry 或政策禁止（ZONE_FORBIDDEN）；`fitness()` 多四種原因，出價表寫 `wrong_floor`／`altitude_limit`／`zone_forbidden`／`indoor`。cbba_node 參數 `site_dir`（或 `CBBA_SITE_DIR`），全隊讀同一份；載具樓層、高度上限從 `fleet.yaml`（`floor`、`limits.max_altitude_m`；參數 `floor`、`max_altitude_m` 可覆蓋）。閘道：任務帶自己的區域註冊表版本；不認得的 `zone_policy_id` 拒絕；指定平台時觀察點在該平台不能進的區域拒絕。版本不同、不認得的政策：`zone_warning` 話題＋log，**照常出價**（使用者 11/17 決定 3） | `zones.*`、`capabilities.*`、`scoring`、`wire_task_announce`、`gateway`、`registry.*`（`toZoneRegistry`）、`cbba_node` | `test_zones`、`test_wire_task_announce`、`test_gateway` ZoneGateway 2 項、`test_registry` SiteZonesForCbba、`zone_iface.sh` Z1～Z4 |
| 08 能力 | 室內區域（`indoor`）在 CBBA 的可行性：UAV 沒有 `indoor_flight`（或故障）不出價（不靠規劃器推導，平台不限的任務也對）；高度上限同 05。E_HW 拿掉的能力：新話題 `capability_restored`（std_msgs/String，能力名稱）放回來 | `zones.cpp`、`cbba_node` | `test_zones` Failed…Restore、`zone_iface.sh` Z5 |
| 07 搶占 | `preempt_margin_pct` 生效：同級（遮罩允許）要出價 > 被搶任務目前的分數 ×（1 + margin/100）才排到它前面（`InsertionLimits::ahead_min_gain`）；跨級不看分數。閘道的**單獨 P0 看門狗**（`p0_unassigned_timeout_s` 10 s，和角色組的計時分開）：沒有得標者也沒有正式指派 → `task_orphaned` P0_UNASSIGNED（10）；有 `fallback_template`（單獨任務現在也能給）時發 `FallbackRequest` P0_UNASSIGNED（5，新欄位 `task_id`） | `cbba_agent.cpp`、`scoring.cpp`、`gateway`、`cbba_node`、swarm_interfaces 2.5.0 | `test_zones` PreemptMargin 2 項、`test_gateway` Watchdog 2 項、`zone_iface.sh` Z6 |

和原本規劃不同的地方：
- 「室內區域自動要求 indoor_flight」放在 CBBA 的可行性（看觀察點所在區域），不是規劃器加到 required_capabilities：平台不限的任務
  若被加上 indoor_flight，狗也會被排除。
- 「升級」只做告警（task_orphaned＋log）＋FallbackRequest，沒有自動提高優先級。
- 鄰居的樓層、高度上限不在 AGENT_STATE 裡：閘道的「有沒有能做的載具」（capableAgents）只看平台的區域限制。

### 第三批的實作（2026-11-18，10、12；單元、`exec_iface.sh`、回歸全過）

| 列 | 做法 | 程式 | 測試 |
|---|---|---|---|
| 10 執行規格 | steps 是唯一的執行介面（不用 bt_subtree）。閘道注入前的 **TaskSpec 檢查**（`swarm_cbba/spec_validator.*`，要 `registry_dir`；`site_dir` 時另外查位置、設備實例）：動作存在且可規劃開放、參數型別範圍 enum required、location／object（required、參照存在、沒代換的範本字串、物件類別）、步數 ≤ 20、平台支援（not_started 不算）、wait_for 引用、成功／中止條件格式、timeout_s。有錯不注入，警告回 `InjectTask.warnings`。**任務層級 `timeout_s`**：執行端 cbba_node 強制（`AssignmentManager::setTaskTimeout`，停住的時間不算）→ 當成 E_TIMEOUT 失敗。8 個動作的支援表在 `registry/api.md` 4.2 | `spec_validator.*`、`assignment_manager.*`、`cbba_node` | `test_spec_validator` 7 項、`exec_iface.sh` E1～E4 |
| 12 人機與安全 | 核准狀態 PROPOSED → WAITING_APPROVAL → APPROVED → INJECTED／REJECTED（EXPIRED），核准前不建 CBBA 任務（`cbba_core/approval.*`）；誰要核准：`InjectTask.requires_approval`、事件類型 requires_approval、風險 high 的動作（`approval_policy` none／risk／strict）；`approve_task` 服務（單號或整個事件、操作者必填）、`approvals` 話題、稽核紀錄檔；safety_notes 只顯示 | `approval.*`、`cbba_node`、`ApproveTask.srv`、`ApprovalState.msg`、swarm_interfaces 2.6.0 | `test_approval` 4 項、`exec_iface.sh` E5、E6 |

還沒做的：前置條件鏈（preconditions／effects 模擬）、續航、機隊可行性（`/validate` 的另外幾層）；BT 端照 success_criteria、abort_conditions
判斷（狗、無人機的 BT）；在等核准的單子不存檔（閘道重開要規劃器重送）。

### 第四批的實作（2026-11-18，11、03；單元、`report_iface.sh`、回歸全過）

| 列 | 做法 | 程式 | 測試 |
|---|---|---|---|
| 11 回報 | TaskSpec `report_policy`（stream、snapshot_period_s、channels gcs／vms／operator、required_findings）注入前檢查；`TaskResult` 加 completed_steps、attachments、final_position；**TASK_REPORT／TASK_REPORT_ACK**（協定版本 5 的種類 14、15，轉送）：執行者把被接受的結果送閘道，**結果 outbox**（閘道斷線時暫存、恢復後補送、沒確認重送、限速、滿了丟最舊的）；閘道發 `task_reports`（channels、missing_findings）、紀錄檔 | `task_report.*`、`wire_task_report.*`、`cbba_comm`、`spec_validator`、`cbba_node`、`TaskReport.msg` | `test_wire_task_report` 3 項、`test_task_report` 4 項、`test_spec_validator` ReportPolicy、`report_iface.sh` R2～R5 |
| 03 證據 | `InjectTask` 帶事件的證據（URI＋信心值＋攝影機）；URI 規則 `evidence://<incident>/<種類>/<檔名>`；TaskSpec 只帶 `evidence` 的 uri、confidence；整筆留在閘道（`incident_evidence`、紀錄檔）；CBBA 封包不帶證據 | `task_meta`、`cbba_node`、`IncidentEvidence.msg` | `test_task_meta` Evidence、`report_iface.sh` R1、R5 |

還沒做的：BT／adapter 照 report_policy 控制相機（串流、定期截圖）；GCS／VMS 端實際的回寫；證據庫本身（保存期限、權限、刪除）在控制中心。
swarm_interfaces 2.7.0：**TaskResult 加欄位，狗的 BT 要 `dog_bt.sh build`**（型別雜湊變了，舊的 BT 會和新的 cbba_node 對不上）。

### 第五批的實作（2026-11-18，13 簽章與資安；單元、`sec_iface.sh`、回歸全過）

| 做法 | 程式 | 測試 |
|---|---|---|
| 簽 TASK_ANNOUNCE、TASK_SPEC、TASK_CLOSE、INCIDENT_UPDATE：封包最後 31 B 的簽章（HMAC-SHA256 的 PoC；alg 留給 Ed25519），簽轉送區塊＋內容（nonce ＝ origin_session、origin_seq），不簽表頭；驗過的存起來，非 issuer 的轉送、補發原樣帶上；`task_issuers` 白名單；`security_mode` none（off）／sign／require；金鑰檔與輪換、撤銷；細節在 `cbba_core/doc/protocol.md` 第八節 | `crypto.*`、`wire_signature.*`、`cbba_comm`（`signOutgoing`、`verifyIncoming`）、`cbba_node`（參數、金鑰檔） | `test_crypto` 3 項（FIPS、RFC 4231 向量）、`test_security` 7 項（4 個驗收：改位置簽章失效、未授權 issuer 拒收不轉送、同 nonce 判定重放、未授權取消 P0）、`sec_iface.sh` S1～S4 |

### 整理批的實作（2026-11-18，06 時間、09 協同出價第二階段；單元、節點腳本、itest 回歸全過）

| 列 | 做法 | 程式 | 測試 |
|---|---|---|---|
| 06 時間 | 合約寫進 `cbba_core/doc/protocol.md` 第九節：wall time（chrony）而不是 monotonic offset、ms 低 32 位元循環、各欄位的意義與單位、`reward_decay` 只有 EXPONENTIAL。**先後**：release < 硬期限 ≤ expires（閘道注入時檢查；軟期限不檢查）。**expires 一律是硬限制**：完成晚於 expires 不出價（失效時執行中的也撤銷；執行中的不檢查）。**時鐘偏差**：CBBA_STATE 的 s 帶傳送者自己的送出時刻，**不參與出價的閘道也帶**（多 6 B）；收的那端超過 `clock_skew_warn`（0.5 s）告警（log），只告警、不修正 | `scoring.cpp`、`gateway.cpp`（check）、`cbba_comm`（`cbbaStateBody`、`clockSkew`、`takeSkewWarnings`）、`cbba_node`（`handleClockSkew`） | `test_time_contract` 9 項（PDF 驗收 1～4 已有的測試對照在檔案開頭）；`test_comm` 的不參與節點改成帶 1 筆 s；`test_gateway` 的過期夾具執行時間改 0 |
| 09 協同出價 | 有效到達 = max(自身到達、前置角色達成里程碑、會合隊友到齊〔最多等 sync_window_s〕)。`CbbaComm::updateCoopEta`（每次 reevaluate 前）用前置角色、隊友的持有者（或得標者）位置與平台速度估時刻 → `Task::coop`（只在本機）；`evaluatePath` 當成像 release 的等待。里程碑照 spec 的 `wait_for`（cbba_node 的 resolver 用 `parseSpecGates`），沒有 spec 時 precedence 等 completed。不知道時不加限制。單獨任務的 fallback 由閘道管理（07 的 P0 看門狗，已有） | `types.hpp`（`CoopEta`）、`scoring`（`LegEval::wait`）、`cbba_agent`（`setCoopEta`）、`cbba_comm`（`travelTime`、`milestoneEta`、`arrivalEta`、`waitsOf`）、`cbba_node`（參數 `coop_bidding`、`platform_speed`、`coop_eta_tolerance`；resolver） | `test_coop_bidding` 7 項（PDF 驗收 1～4 是第一階段，已有的測試對照在檔案開頭）；設計 `cbba_core/doc/cb4_design.md`「協同出價」 |

和原本規劃不同的地方：
- 時鐘偏差只告警、不修正（不估各台的 offset 去改時刻）：chrony 本來就要開，修正會讓同一個時刻在各台算出不同的值。
- 會合用 A 層的近似：同組 `sync_window_s` > 0 的角色都算同一個會合點（spec 的會合點名稱只在執行端分）。
- 別台的預估不看它路徑上排在前面的任務（直線距離 / 平台速度）；之後需要時用 AGENT_STATE 的 path 加。
- 封包格式沒改（還是協定版本 5）；只有不參與出價的節點 CBBA_STATE 多帶自己的 1 筆 s。swarm_interfaces 不變（狗的 BT 不用重編）。

> 下面第 0～6 節是 11/17 稍早的分析（使用者提供 PDF 之前），以上一節為準：路徑繞行估算（F6、N3）不做（路徑交給 Nav2）；
> 檢查器與核准放在閘道；第 4 節的實作順序由上表取代。

## 0. 現況：哪裡是「淺層」

| 面向 | 目前的判斷 | 淺在哪裡 |
|---|---|---|
| 載具是否適合 | 平台 ∈ {不限, 我的平台}、需要的能力 ⊆ 我的能力、電量做完＋直線返航後 ≥ 存量、硬期限 | 不看**樓層、區域、室內能不能飛、高度上限、安全距離、觀察點**；路程一律直線 |
| 航行限制 | 沒有 | `site-config/yard/zones.geojson` 已有 `no_entry`（狗不能進高壓區、儲槽區…）、`fleet.yaml` 有 `max_altitude_m`、`max_flight_s`，**CBBA 都沒讀**；任務也不能帶「這個任務額外禁止的區域」 |
| 前後順序 | CB-4：precedence／sync／all-or-nothing／distinct／optional 都在**執行端**（gate），出價照各自的到達時間 | 出價**不知道要等前一個角色**：狗的到達時間算成「到了就能開始」，等無人機定位的時間沒算進分數；`preempt_margin` 有帶沒用；P0 沒人得標的看門狗是 30 s |

---

## 1. 載具是否適合（出價可行性）

2-1 欄位：`platform`、`required_capabilities`、`min_battery_reserve`、`target.floor`、`allowed_zones`／`forbidden_zones`、`standoff_m`、`observe_pose`、`deadline`（hard）。
原則不變：**可行性是條件、不進分數**；不可行 = 出價 0（出價表寫原因）。

| 編號 | 檢查 | 做法 | 資料來源 | 出價表原因 |
|---|---|---|---|---|
| F1 | 平台、能力 | ✅ 已有 | 任務 A 層、`fleet.yaml` | `platform`、`capability` |
| F2 | 電量、返航 | ✅ 已有（含任務的 `min_battery_reserve`） | | `infeasible` |
| F3 | 硬期限、release | ✅ 已有 | | `infeasible` |
| **F4** | **樓層** | 任務的 `floor` ≠ 0 而且 ≠ 載具目前樓層 → 不可行；狗跨樓層要 `stairs` 能力（先只擋，不算爬樓梯的時間） | robot_state 加 `floor`（或 fleet 的預設樓層） | `floor` |
| **F5** | **區域：目標點** | 目標點落在「這個平台不能進」的區域（`zones.geojson` 的 `no_entry`，加上任務自己的 `forbidden_zones`）→ 不可行；有 `allowed_zones` 時目標點要在其中 | 各節點啟動時讀 `site-config/<場域>/zones.geojson`；任務帶區域 ID | `zone` |
| **F6** | **區域：路徑** | 直線路徑（目前位置 → 各任務 → 返航）穿過這個平台不能進的區域 → 先繞多邊形外緣估算繞行距離（不規劃路徑，只算長度），繞不過去才不可行；距離進 TDR 的到達時間與耗電 | 同上 | `zone_path` |
| **F7** | **室內飛行、高度** | UAV：目標所在區域標 `indoor` → 要 `indoor_flight` 能力；任務高度 > `limits.max_altitude_m` → 不可行；`cruise_altitude_m` 用區域的值 | `zones.geojson` 加 `indoor`、`max_altitude_m`；`fleet.yaml` 的 `limits` | `indoor`、`altitude` |
| **F8** | **觀察點與安全距離** | 2-1：任務帶 `target`（目標）＋`observe_pose`（站的位置）＋`standoff_m`。出價的到達時間算到**這個平台的觀察點**，不是目標本身；觀察點離目標 < 物件的安全距離（`objects.yaml` 的 `safety_distance_m`，依平台）→ 由規劃器往外推到合法距離 | A 層加 `observe_offset`（或規劃器直接把 A 層位置設成觀察點，見 §6 決定） | — |
| **F9** | **續航時間** | UAV：路徑時間（含懸停）＋返航 > `max_flight_s` × 剩餘電量比例 → 不可行（目前用電量換算，等價；確認一致即可） | `fleet.yaml` | `infeasible` |

---

## 2. 航行限制（區域、空間）

| 編號 | 內容 | 做法 |
|---|---|---|
| N1 | **區域資料進 CBBA** | 新的 `cbba_core/zones.hpp`：多邊形、屬性（`no_entry` 平台清單、`indoor`、`high_voltage`、`floor`、`cruise_altitude_m`）；`swarm_cbba` 從 `site_dir` 參數讀 `zones.geojson`（和 `registry_dir` 一樣，沒給就不檢查）。**全隊要用同一份**：AGENT_STATE 或摘要帶區域檔的 CRC，不一致時告警 |
| N2 | **任務附帶的區域限制** | A 層加 `forbidden_zones`、`allowed_zones`（區域編號，最多各 4 個，u8 索引；TASK_ANNOUNCE 多 10 B）。例如煙霧任務：煙霧所在區域禁止進入、只在區域外觀察（2-1 的 safety_notes） |
| N3 | **路徑估算** | 直線段與多邊形相交 → 沿多邊形外緣取較短的一側繞行的長度（凸包近似）；只給出價用，實際路徑由 Nav2／PX4 規劃。2-1 的「生成時查 costmap」由規劃器做（§5） |
| N4 | **無人機禁飛區** | `no_entry` 裡有 `uav` 的區域（例如之後的室內、人員密集區）同樣處理；高度限制 F7 |
| N5 | **BT 端** | `forbidden_zones` 也寫進 TaskSpec，給 BT 的 `IsZoneAllowed`（2-1 欄位表的「用在哪裡」）；CBBA 只做可行性 |

---

## 3. 前後順序與協同（任務之間）

2-1 欄位：`sync_window_s`、`precedence`、`all_or_nothing`、`fallback_template`；「包夾要同時到位；無人機先定位再派狗」；「先用角色子任務＋執行端同步，不夠時再在出價加入協同約束」。

| 編號 | 內容 | 現況 | 補齊做法 |
|---|---|---|---|
| P1 | precedence、sync、all-or-nothing、distinct、optional＋skip_satisfies | ✅ CB-4（執行端 gate＋Group Commit） | 不變 |
| **P2** | **出價看前一個角色**：依賴角色的開始時間 = max(自己到達觀察點, 前一個角色預估完成里程碑的時刻)；TDR 的時間折扣與 `service_time` 從這個時刻算 | ❌ 狗的出價當成到了就能開始 | 前一個角色的得標者在 CBBA 紀錄裡就有；它的預估到達＋該步驟的 `service_time` 從 TASK_PROGRESS／AGENT_STATE（BT 的預估剩餘時間）或用它的位置、速度估算。只影響**依賴角色**的分數，前一個角色不受影響（不破壞 DMG） |
| **P3** | **sync 的等待成本**：同一個會合點的角色，預估到達差超過 `sync_window_s` 的那台分數打折（等太久的不划算） | ❌ | 用同組其他角色目前得標者的預估到達時間；先只做「超過窗口 → 不可行或打折」二選一（§6 決定） |
| **P4** | **條件式順序**：無人機參與時狗等 `target_localized`，無人機被略過時 `skip_satisfies` 放行 | ✅（G14） | 加 itest：無人機缺席時狗直接開始 |
| **P5** | **搶占遲滯**：新任務分數要比目前任務高出 `preempt_margin` 才搶；進入 `commit_radius_m` 後鎖定 | ⚠️ commit_radius ✅、margin 沒用 | `mayPreempt` 加 margin：搶占的出價 ≥ 被搶任務目前分數 ×（1 + margin） |
| **P6** | **指派看門狗**：P0 在 10 s 內沒有得標者 → 升級、`fallback_template` 降級重發 | ⚠️ 角色組有（恢復期限 10 s）；單獨任務只有 30 s 的 `task_orphaned` | `orphan_warn_sec` 依優先級（P0 10 s）；閘道對單獨任務也發 FallbackRequest（有 `fallback_template` 時） |
| **P7** | **移動目標**：位置更新只改 revision 與 target；位移 < 門檻（3 m）不重新拍賣，只讓得標者更新導航目標 | ⚠️ 改位置一律保留指派（不看距離） | 位移 < 3 m：只更新 incident（不改任務 revision）；≥ 3 m：改版位置、保留指派、重算分數（得標者明顯變差時才釋放，用 `rebid_threshold`） |
| P8 | 被搶占的巡檢恢復 | ✅ P3 交回重新競標（G13）；「從最近的巡檢點接續」是 BT | 不變 |

---

## 4. 實作順序（建議）

| 步驟 | 內容 | 改哪裡 | 測試 |
|---|---|---|---|
| 1 | N1 區域資料、F5 目標點、F4 樓層、F7 室內與高度 | `cbba_core/zones.*`、`capabilities`／`scoring` 的可行性、`swarm_cbba`（`site_dir`）、`site-config` 加屬性 | 單元：每種原因一個案例；itest group 加 G16（狗不出價高壓區內的任務、UAV 接） |
| 2 | N2 任務附帶的區域、N5 TaskSpec | 協定 A 層（TASK_ANNOUNCE ＋10 B）、`InjectTask`、`Task.msg` | wire 測試、G17（煙霧任務禁止進入煙霧區） |
| 3 | F6、N3 路徑繞行估算 | `scoring`（到達時間、耗電用繞行距離） | 單元：穿過禁區的任務出價下降或不可行 |
| 4 | P5 搶占 margin、P6 看門狗、P7 移動目標 | `types.hpp` 的 `mayPreempt`、`cbba_node`、閘道 | G18（P0 10 s 沒人得標 → fallback）、G19（小位移不重新拍賣） |
| 5 | P2 出價看前一個角色、P3 sync 等待 | `scoring`／`cbba_agent`（依賴角色的開始時間）、TASK_PROGRESS 帶預估完成時刻 | 單元＋G20（兩隻狗搶同一個依賴角色：近但要等很久的 vs 遠但剛好的） |
| 6 | F8 觀察點與安全距離 | 規劃器（§5）＋可能的 A 層欄位 | |

每一步都只加可行性與時間估算，不動 17 條消解規則；協定改了要全隊重編（含狗）。

---

## 5. 初步任務規劃器（只做到初步）

把 itest 裡寫死的假 Planner 換成節點 `swarm_planner`（閘道旁邊，Python）：

| 編號 | 內容 |
|---|---|
| PL-1 | TaskSpec schema 1.1：第二節 B 層欄位（incident_id、task_type、event_type、role、target_kind、uncertainty_m、observe_pose、standoff_m、zones、success_criteria、abort_conditions、timeout_s、report、requires_approval、evidence_ref） |
| PL-2 | `registry/event_rules.yaml`：入侵、煙霧／火、熱異常、遺留物＋`single_dog_verify` |
| PL-3 | 事件狀態機簡化版：去重（Δt、r 內同類型 → revision + 1）、派遣門檻（高信心派完整範本、中信心派 1 隻確認） |
| PL-4 | 展開＋檢查：格式、詞彙、參數、目標／觀察點不在禁區（不可達 → 移到最近的可達點或標 unreachable）、安全距離 |
| PL-5 | 注入、`requires_approval` 閘門、看門狗、incident 更新重新展開、結案回寫 |

驗證用 2-1 第五節「階段 1」的 12 個情境（目前 1、4、7 已涵蓋，2、3、5、6、8、10 要新增，9 先只做上限，11、12 要 VLM 不做）。

---

## 6. 要先決定的事

| 項目 | 選項（建議在前） |
|---|---|
| 觀察點怎麼進出價 | 規劃器把 A 層位置直接設成**該角色的觀察點**（不改協定）／A 層加 observe_offset（同一個任務不同平台有不同觀察點時才需要） |
| 路徑穿越禁區 | 繞行距離估算進分數，繞不過去才不可行／一律不可行（簡單，但會漏掉可以繞的） |
| sync 等待 | 超過 sync_window_s 的不可行／分數打折 |
| 區域檔一致性 | 各節點讀同一份檔，AGENT_STATE 帶 CRC 告警（建議）／由閘道用 TASK_SPEC 發區域（之後） |
| 樓層來源 | robot_state 加 floor／fleet.yaml 的預設樓層（單一樓層實驗時兩者相同） |
