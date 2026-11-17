# 註冊表的對外格式：`/registry`、`/validate`、`/test_run`、TaskSpec、ExecuteStep（2-1b M0，2026-11-16 草案）

2-1b 第九節的 M0（11/20）：「六份註冊表 schema v1.0 凍結；通用 Action 定義與 8 個失敗碼定版；`/registry`、`/validate`、
`/test_run` 格式與羅凡確認」。分工：`/registry`、`/validate` 是勝翔（REG-1、PLN-2），`/test_run`、Action 介面是伯宇（BT-3、REG-2）。
**這份是草案**：`/registry`、`/validate` 照這裡做；`/test_run`、ExecuteStep 是給伯宇的提案，對過才定。

- 傳輸：HTTP＋JSON（羅凡的 Node-RED 積木編輯器直接呼叫），由地面站（閘道那台）提供；之後需要時再加同內容的 ROS 2 service。
- 所有回應都帶 `schema_version`（註冊表的版本，目前 `"1.0"`）；版本不同的客戶端要拒絕。
- 錯誤：HTTP 4xx／5xx 時 body 是 `{"error": {"code": "...", "message": "..."}}`。

## 1. `GET /registry`

回傳整份註冊表＋場域設定（規劃器、檢查器、VLM 詞彙、積木編輯器的唯一來源）。內容就是 `registry/*.yaml` 與
`site-config/<site>/` 轉成 JSON，鍵名不改。

```json
{
  "schema_version": "1.0",
  "revision": "beb06c1+3f2a",
  "site": "yard",
  "tools":       [{"name": "rgb_camera", "bit": 4, "description": "可見光相機"}],
  "platforms":   [{"name": "uav", "code": 1, "cruise_speed": 0.5, "endurance_s": 900, "max_bundle": 2}],
  "fleet":       {"robots": [{"id": 1, "name": "uav1", "platform": "uav", "ns": "/uav1",
                              "capabilities": ["mobility_air", "rgb_camera"],
                              "limits": {"max_altitude_m": 10.0, "max_flight_s": 900, "landing_pads": ["PAD_1"]}}],
                  "gateways": [{"id": 200, "name": "gcs", "ns": "/gcs"}]},
  "actions":     {"conditions": [{"name": "at_location"}], "common_failure_codes": ["E_PRECONDITION"],
                  "actions": {"inspect": {"class": "plannable", "params": {"mode": {"type": "enum"}}}}},
  "objects":     {"classes": {"person": {"allowed_actions": ["inspect", "track", "speak"]}}},
  "event_types": {"detectors": {}, "event_types": {}},
  "location":    {"zones": {"type": "FeatureCollection", "features": []},
                  "devices": [{"id": "machine:TA1", "class": "machine", "x": -30.0, "y": 15.0, "z": 1.5}],
                  "pois": [], "docks": []},
  "vocabulary":  {"actions": ["go_to", "patrol", "inspect", "track", "speak", "wait", "return_to_dock", "alert_operator"],
                  "objects": ["person", "bag", "machine", "gauge", "door", "smoke_region", "puddle", "fire"],
                  "capabilities": ["mobility_ground", "mobility_air", "..."]}
}
```

- `revision`：註冊表內容的版本（git commit＋內容雜湊前 4 碼），客戶端用來判斷要不要重抓。
- `vocabulary`：給 VLM 提示詞與積木編輯器的詞彙，由註冊表自動產生：只列 `class: plannable` 而且 `status: open` 的動作
  （2-1b 第六節：「只列可規劃且已開放的動作」）；物件是類別名稱。
- 查詢參數：`?part=actions`（只回一部分，值是上面的鍵）、`?site=yard`。

## 2. `POST /validate`（PLN-2 九層檢查器）

輸入一份 TaskSpec（第 4 節），回傳每一層的結果。範本展開與 VLM 提案都要通過；積木編輯器組好任務也先呼叫這個。

請求：

```json
{
  "task_spec": { "...": "第 4 節" },
  "fleet_state": [{"id": 1, "available_capabilities": ["mobility_air", "rgb_camera"], "battery": 0.82, "airborne": false}],
  "options": {"layers": ["format", "vocabulary", "params", "preconditions", "zones", "fleet", "endurance", "risk", "scale"]}
}
```

- `fleet_state` 可省略：省略時用 `fleet.yaml` 宣告的能力、電量當滿電（「機隊可行性」「續航」兩層只能做靜態檢查）。
- `options.layers` 可省略（= 全部）。

回應：

```json
{
  "schema_version": "1.0",
  "ok": false,
  "requires_approval": false,
  "use_fallback": false,
  "layers": [
    {"layer": "format",        "ok": true},
    {"layer": "vocabulary",    "ok": true},
    {"layer": "params",        "ok": false},
    {"layer": "preconditions", "ok": true},
    {"layer": "zones",         "ok": true},
    {"layer": "fleet",         "ok": true},
    {"layer": "endurance",     "ok": true},
    {"layer": "risk",          "ok": true},
    {"layer": "scale",         "ok": true}
  ],
  "errors": [
    {"layer": "params", "code": "PARAM_OUT_OF_RANGE", "path": "roles[1].steps[1].params.duration_s",
     "message": "inspect 的 duration_s 是 90，範圍 2～60"}
  ],
  "warnings": [
    {"layer": "fleet", "code": "OPTIONAL_ROLE_NO_ROBOT", "path": "roles[0]",
     "message": "aerial_overwatch 目前沒有可用的無人機（optional，會略過）"}
  ]
}
```

| 層 | 檢查內容（2-1b 第六節） | 錯誤碼 |
|---|---|---|
| `format` | 符合 TaskSpec JSON Schema（`registry/schema/task_spec.schema.json`） | `SCHEMA` |
| `vocabulary` | action 在可規劃且已開放清單中；object 的類別在該 action 的 objects；location 存在（zone、poi、asset、dock、pose） | `UNKNOWN_ACTION`、`ACTION_NOT_OPEN`、`OBJECT_NOT_ALLOWED`、`UNKNOWN_LOCATION` |
| `params` | 型別與範圍；安全距離（standoff_m）不小於物件要求；UAV 高度不超過區域天花板與 `max_altitude_m` | `PARAM_TYPE`、`PARAM_OUT_OF_RANGE`、`PARAM_MISSING`、`BELOW_SAFETY_DISTANCE`、`ABOVE_CEILING` |
| `preconditions` | 用各動作的 preconditions／effects 模擬整串步驟（例如 inspect 前要有 at_location）；UAV 的任務結束必須回到 landed | `PRECONDITION_UNMET`、`UAV_NOT_LANDED_AT_END` |
| `zones` | 狗不進禁入區；UAV 不進禁飛區、不從人員上方飛過；位置可達 | `NO_ENTRY_ZONE`、`NO_FLY_ZONE`、`OVERFLY_PERSON`、`UNREACHABLE` |
| `fleet` | 至少一台可用機器人具備每個角色的能力；非 optional 角色無人可用 → `use_fallback: true`（改用 fallback_template） | `ROLE_NO_ROBOT`（錯誤）、`OPTIONAL_ROLE_NO_ROBOT`（警告） |
| `endurance` | UAV 角色的預估飛行時間＋返航時間不超過剩餘飛行時間 | `ENDURANCE_EXCEEDED` |
| `risk` | 含 risk: high 的動作，或實作的 autonomy 未到 field_verified → `requires_approval: true`（警告，不是錯誤） | `HIGH_RISK`、`AUTONOMY_NOT_VERIFIED` |
| `scale` | 步驟數、角色數不超過上限（暫定每角色 20 步、每組 8 個角色） | `TOO_MANY_STEPS`、`TOO_MANY_ROLES` |

- `ok` = 沒有錯誤（警告不影響）；`requires_approval`、`use_fallback` 給規劃器決定下一步。
- `path` 是 JSON Pointer 風格的點路徑，積木編輯器用來標在出錯的積木上。
- M2 前先做前四層（2-1b 第九節的預案：uav_adapter 延誤時「續航」「機隊可行性」移到 M3）。

## 3. `POST /test_run`（BT-3，伯宇；提案）

讓積木編輯器把一組 steps 直接交給一台機器人試跑（不經 CBBA），看每一步的回饋與結果。

```json
POST /test_run
{"robot": "dog1", "mode": "sim", "steps": [{"action": "go_to", "location": "OBS_HV_B_FRONT"},
                                           {"action": "inspect", "object": "machine:TB3", "params": {"mode": "rgb"}}],
 "validate": true}
→ {"run_id": "tr_0007", "status": "accepted"}            # validate 失敗時 400，body 同 /validate 的回應

GET /test_run/tr_0007
→ {"run_id": "tr_0007", "robot": "dog1", "status": "running",   # accepted／running／succeeded／failed／canceled
   "current_step": 1,
   "steps": [{"step_id": 0, "action": "go_to", "status": "succeeded", "started": "...", "finished": "..."},
             {"step_id": 1, "action": "inspect", "status": "running", "progress": 0.4, "phase": "aiming"}],
   "failure_code": "", "findings": []}

POST /test_run/tr_0007/cancel → {"status": "canceling"}
```

- 機器人正在執行 CBBA 的任務時拒絕（409），避免和正式流程互搶（同 2-1b 的「手動接管」鎖）。
- `mode`：sim（Gazebo）／real。

## 4. TaskSpec（規劃器的輸出、`/validate` 的輸入）

2-1b 第六節：一組 TaskSpec（同一 `group_id`），每個角色有平台限制、需要的能力、steps、協同約束。閘道照角色拆成 CBBA 任務
（每個角色一個任務、同一個 group_id；`current_task.spec` 是那個角色的 `{"steps": [...]}`）。JSON Schema：
`registry/schema/task_spec.schema.json`。

```json
{
  "schema_version": "1.0",
  "group_id": "",
  "incident": {"event_id": "ev_0042", "event_type": "perimeter_intrusion", "revision": 1,
               "location": {"frame": "map", "floor": "1F", "x": -10.0, "y": 15.0, "z": 0.0, "radius_m": 3.0, "zone": "Z_HV_A"},
               "track_id": "trk_88"},
  "rule": "perimeter_intrusion",
  "priority": "P0",
  "expires_s": 600,
  "requires_approval": false,
  "roles": [
    {"role": "aerial_overwatch", "platform": "uav", "required_capabilities": ["mobility_air", "rgb_camera"], "optional": true,
     "steps": [{"action": "go_to", "location": {"zone": "Z_HV_A"}, "params": {"altitude_m": 3.0}},
               {"action": "track", "object": "person:trk_88", "params": {"distance_m": 2.0, "duration_s": 30}},
               {"action": "return_to_dock"}]},
    {"role": "front", "platform": "quadruped", "required_capabilities": ["mobility_ground", "rgb_camera", "speaker"],
     "steps": [{"action": "go_to", "location": {"poi": "OBS_HV_A_FRONT"}},
               {"action": "inspect", "object": "person", "params": {"mode": "rgb", "duration_s": 5}},
               {"action": "speak", "params": {"template": "intrusion_warning"}}]},
    {"role": "rear", "platform": "quadruped", "required_capabilities": ["mobility_ground", "rgb_camera"],
     "steps": [{"action": "go_to", "location": {"poi": "OBS_HV_A_REAR"}},
               {"action": "inspect", "object": "person"}]}
  ],
  "coupling": {"precedence": [["aerial_overwatch", "front"]], "sync_window_s": 15},
  "fallback_template": "single_dog_verify"
}
```

- `group_id`：空字串 = 閘道注入時產生（64 位元 task_id 同格式，見 `cbba_core/doc/cbba_m0_contract.md`）。
- `platform`：`any`（狗和 UAV 都可以競標）或 `platforms.yaml` 的 name。
- `location`（第五節的位置標準），五選一：`{"zone": id}`、`{"poi": id}`、`{"asset": "machine:TA1"}`、`{"dock": id}`、
  `{"frame": "map", "floor": "1F", "x": .., "y": .., "z": ..}`。範本裡的 `{incident.zone}`、`{incident.observe_front}` 由展開器代換成這些。
  閘道交給 CBBA 前會把 zone／poi／asset／dock 換成座標（CBBA 的任務位置），原本的參照留在 spec 裡給 BT。
- `object`：類別（`person`）或「類別:實例」（`machine:TB3`、`person:trk_88`）。
- `coupling.precedence`：`[[A, B], ...]` = A 先、B 後。2-1b 第八節：**執行端同步**（B 可以先出發，A 更新 incident 位置
  revision + 1 後 B 的目標自動更新；A 是 optional 時沒有也不阻塞）。原文範例寫成一維 `[aerial_overwatch, front]`，
  這裡統一成二維，兩種都收（一維當成一組）。

### 4.1 角色 TaskSpec（`current_task.spec`）的最上層（任務欄位補齊第一批，2026-11-17）

閘道把每個角色拆成一個 CBBA 任務時，把**識別、語意、空間**寫在那個角色的 spec 最上層（`cbba_core/task_meta.hpp`；
規劃器經 `InjectTask` 的欄位給，不要自己寫進 spec——寫了會被拒絕，`role`、`schema_version` 一樣的話例外）。
這些不進 TASK_ANNOUNCE（TaskCore 只放出價、可行性、協同要的）；規劃器的 spec 是空的時不產生（只留在閘道，`assignments` 看得到）。

```json
{"schema_version": "1.1", "incident_id": "ev_0042", "source": "VMS", "source_id": 4,
 "event_type": "perimeter_intrusion", "event_type_id": 1, "task_type": "confirm_intrusion", "task_type_id": 4,
 "role": "front", "role_id": 2, "description": "入侵：前方確認",
 "observe_pose": {"frame": "map", "floor_id": 1, "x": 12.0, "y": 0.0, "z": 0.0},
 "target_kind": "moving", "target": {"frame": "map", "floor_id": 1, "x": 17.0, "y": 0.0, "z": 0.0},
 "track_id": "trk_88", "uncertainty_m": 2.0, "standoff_m": 5.0,
 "steps": [{"action": "go_to"}, {"action": "inspect", "object": "person:trk_88"}]}
```

| 欄位 | 說明 |
|---|---|
| `schema_version` | 角色 spec 的版本；BT 看主版本（1），不認得的主版本不執行。1.0 = 只有 steps |
| `incident_id` | 事件識別（＝閘道的冪等鍵；閘道重開後同一個 (incident_id, role) 沿用原 task_id） |
| `source`／`source_id` | 固定列舉：0 UNKNOWN、1 ALARM、2 UAV、3 QUADRUPED、4 VMS、5 OPERATOR、6 SENSOR（只增不改） |
| `event_type`、`task_type`、`role`（＋`_id`） | 名稱給人看、編號給程式比（`event_types.yaml` 的 id、`task_types.yaml`）；執行照 steps，`bt_subtree` 不採用 |
| `observe_pose` | 這個角色真正要到的位置 ＝ TASK_ANNOUNCE 的 x、y、z（出價、TDR 都算到這裡）。`floor_id` 是 A 層的樓層編號 |
| `target_kind`、`target` | 事件本身：point（固定）、area（`uncertainty_m` 是半徑）、moving（要 `track_id`）；沒有另外的目標時不寫 |
| `standoff_m` | 觀察點至少離目標多遠（area 從邊緣算）；閘道注入時檢查，不夠就拒絕（觀察點由規劃器往外推，閘道不推） |

- **移動目標**：規劃器改版（revision + 1）只動了位置、觀察點位移 < `retarget_threshold_m`（3 m，全隊一樣）→ CBBA 換成新內容但
  **不重新評估**（得標者照舊，BT 收到新的 revision 更新導航）；≥ 3 m 或同時改了別的欄位 → 照一般改版重新評估。
  規劃器改版時相對時間（deadline_sec 等）和上一版一樣 → 閘道沿用原本的絕對時刻（期限不會因改版往後推）。

## 5. ExecuteStep（REG-2 的通用 Action，給伯宇的提案）

使用者 11/15 定：**一個通用的 `ExecuteStep.action`**（狗和無人機共用；BT 的 `ExecuteSteps` 逐步呼叫 `/<ns>/execute_step`），
欄位照 2-1b 第二節的「通用介面」。和伯宇對好之後放進 `swarm_interfaces/action/ExecuteStep.action`。

```
# Goal
uint64 task_id                 # CBBA 任務（版本 4 的 64 位元 task_id）；/test_run 時是 0
uint16 revision                # 任務的 revision
uint32 step_id                 # 第幾步（0 起）
string action                  # actions.yaml 的名稱
string object                  # 類別或「類別:實例」（machine:TR-01、person:trk_88）；空 = 沒有
string location_json           # 第 4 節的 location（JSON）；空 = 沒有
string params_json             # 參數（JSON 物件）
builtin_interfaces/Time deadline   # 0 = 用 actions.yaml 的 timeout_s
---
# Result
uint8 SUCCEEDED=0
uint8 FAILED=1
uint8 CANCELED=2
uint8 TIMEOUT=3
uint8 status
string failure_code            # 8 個 E_*；成功時空字串
string failed_capability       # E_HW 時哪個能力壞了
string findings_json           # 截圖 URI、讀值、偵測結果（JSON 陣列）
---
# Feedback
string state                   # running
float32 progress               # 0～1
geometry_msgs/PoseStamped pose # 目前位姿
string phase                   # 子階段說明（例如 takeoff、aiming、capturing）
```

- 必須可取消（對應 BT 的 `halt()` 與搶占）。取消後 Result 的 status = CANCELED。
- 平台內部步驟（UAV 的 takeoff、set_altitude）由 adapter 依平台狀態機自動補，不出現在 steps 裡；進度回饋照常送（phase）。
- 失敗碼到 CBBA 的 reason：見 `cbba_core/failure_codes.hpp`（`task_result.failure_code`）。

## 6. 統一的 Event（事件狀態機的輸出，2-1b 第七節）

```json
{"schema_version": "1.0", "event_id": "ev_0042", "event_type": "perimeter_intrusion", "category": "template",
 "sources": [{"kind": "fixed_camera", "id": "CAM-07"}, {"kind": "robot_camera", "id": "uav1/down_cam"}],
 "location": {"frame": "map", "floor": "1F", "x": -10.0, "y": 15.0, "z": 0.0, "radius_m": 3.0, "zone": "Z_HV_A"},
 "confidence": 0.93, "evidence": ["file:///.../frame_0001.jpg"], "vlm_result": {"verdict": "intrusion", "reason": "..."},
 "vendor_raw": {}, "revision": 1}
```

- `Detection.msg` 的 `source` 是字串：寫成 `kind:id`（例如 `fixed_camera:CAM-07`、`robot_camera:uav1/down_cam`），
  事件流程拆成原文的 `{kind, id}`。
