# 註冊表（2-1b REG-1，schema 1.0；2026-11-16 照 2-1b 原文改寫）

2-1b 的原則：**一份模組註冊表是唯一來源**，規劃器、檢查器、VLM 詞彙、CBBA 的可行性、BT 分派都讀它；
新增動作或機器人只改註冊表與 adapter。原文第一節的六份都有（schema 1.0）；位置標準（第五節）在 `site-config/<site>/`。
對外格式（`/registry`、`/validate`、`/test_run`、TaskSpec、ExecuteStep、Event）：[`api.md`](api.md)；
TaskSpec 的 JSON Schema：[`schema/task_spec.schema.json`](schema/task_spec.schema.json)。

| 檔案 | 內容（原文第一節） | 誰讀 |
|---|---|---|
| `tools.yaml` | 工具／酬載對應的能力標籤：name、**bit**（CBBA 封包裡能力遮罩的位元位置）、description、deprecated | cbba_node（和程式一致）、規劃器、檢查器 |
| `platforms.yaml` | 平台類型 quadruped／uav 的運動與能耗模型：**code**（封包裡的平台代碼）、速度、耗電、存量、續航、max_bundle、must_return | cbba_node（出價） |
| `fleet.yaml` | 實體機隊：每台的機號、平台、`ns`、能力、`limits`（UAV：max_altitude_m、max_flight_s、landing_pads；狗：docks）；gateways | cbba_node（自己的平台與能力、閘道的機號）、檢查器 |
| `actions.yaml` | 動作：參數、前置條件、效果、平台實作、風險、自主狀態（原文第二節） | 規劃器、檢查器、BT／adapter；cbba_node 只檢查 |
| `objects.yaml` | 物件類別：偵測方式（locate）、允許的動作、安全距離（原文第三節） | 規劃器、VLM 詞彙、檢查器；cbba_node 只檢查 |
| `event_types.yaml` | 偵測器與事件類型：觸發、累積、關聯、VLM、代價、預設範本（原文第七節） | 閘道的事件流程（EVT-1）；cbba_node 只檢查 |
| `site-config/<site>/` | 位置：`zones.geojson`（區域與屬性）、`devices.csv`（設備實例＋觀察姿態）、`pois.csv`（觀察點）、`docks.csv`（充電樁、起降點） | 規劃器（代換 location）、檢查器（區域層）；`test_registry` 檢查和註冊表一致 |

## 規則

- 每份 YAML 開頭 `schema_version: "1.0"`；版本不對、缺欄位、型別不對時 cbba_node 拒絕啟動。
- **bit、code 只增不改**：已經用過的不改、不重用；不要的標籤標 `deprecated: true` 留著。bit 0～63、code 1～255（0 保留給
  「任務不限平台」和「閘道」）。`tools.yaml` 要包含 `cbba_core/capabilities.hpp` 認得的全部標籤、bit 一樣；註冊表可以比程式新
  （多出的標籤 cbba_node 印警告）。
- 機號 1～254，載具和閘道之間也不能重複。
- 名稱（動作、物件類別、事件類型、偵測器、設備 id）用過之後不改：任務、偵測、紀錄都用名稱。
- 能力故障（E_HW）在執行期由 cbba_node 扣掉，不改這份。

## actions.yaml

格式照原文的註冊表範例：`actions:` 底下「動作名稱 → 內容」，欄位 `class`、`version`、`params`、`objects`、`requires.any_of`、
`preconditions`、`effects`、`timeout_s`、`cancelable`、`risk`、`implementations`（每個平台的 `backend`、`interface` 或
`sequence`、`extra_preconditions`、`autonomy`）。我們加的欄位：`status`（open／closed）、`location`、`object`
（required／optional／none）、`failure_codes`；原文的範例沒寫這幾個也能載入（有預設）。前置條件與效果的詞彙在 `conditions`。

| 動作 | 分級 | 參數（原文） | 需要的能力 | 狗 | 無人機 |
|---|---|---|---|---|---|
| `go_to` | 可規劃 | location、standoff_m、altitude_m（UAV） | mobility | Nav2 NavigateToPose（模擬驗證過） | takeoff → set_altitude → fly_to（自動補） |
| `patrol` | 可規劃 | waypoints 或 area、loops | mobility | Nav2 航點序列 | `{ns}/patrol`（要改成可傳航點） |
| `inspect` | 可規劃 | object、mode（rgb／thermal／zoom）、duration_s、standoff_m | 依 mode：rgb_camera／thermal_camera／rgb_camera＋zoom | 雲台 `/ptz_inspect`（模擬驗證過） | hover → capture |
| `track` | 可規劃 | object（person:trk_88）或 bbox、distance_m、duration_s | rgb_camera | follow_person | `{ns}/track` |
| `speak` | 可規劃 | template（語音範本 ID）、volume | speaker | 機載喇叭 | 不支援 |
| `wait` | 可規劃 | duration_s | — | 原地等待 | hover |
| `return_to_dock` | 可規劃 | dock（可省略） | mobility | 回充電樁 | fly_to → land |
| `alert_operator` | 可規劃 | severity、message、attachments | — | 經閘道送控制中心 | 同左 |
| `report_state`、`emergency_stop`、`emergency_land` | 系統 | — | — | 狀態 topic、急停 | `{ns}/state`、`{ns}/land` |
| `takeoff`、`set_altitude`、`fly_to`、`hover`、`capture`、`land` | 平台內部 | — | — | — | UAV 的 sequence 用；實機 `/MAV1/api/*` |
| `open_door`、`close_door`、`pick_up`、`drop_off`、`use_tool` | 可規劃（未開放） | — | manipulator（、gripper） | 未開發 | 不支援 |

- `autonomy`：not_started／in_development／sim_verified／field_verified（M5 的「自主狀態標記」）；檢查器的風險層：
  risk high 或 autonomy 未到 field_verified → `requires_approval`。
- **和 11/14 初版的差異**（BT 要跟著改的）：inspect 的參數 `snapshots`、`timeout` → `mode`、`duration_s`、`standoff_m`
  （狗的 `task_manager_spec.patch` 照抄 params，目前還讀 snapshots，沒寫時拍 1 張；`duration_s` 要和伯宇對怎麼用）；
  speak 的 `text`、`repeat` → `template`、`volume`；alert_operator 的 `level` → `severity`，加 `attachments`；
  track 的 `distance`、`duration` → `distance_m`、`duration_s`；go_to 的 `tolerance`、`timeout` → `standoff_m`、`altitude_m`
  （逾時改用動作的 `timeout_s`）；patrol 的 `pattern`、`size` → `waypoints` 或 `area`。

## objects.yaml

只放**類別**（原文第三節的 8 類：person、bag、machine、gauge、door、smoke_region、puddle、extinguisher，加我們模擬用的 fire）。
**實例**放在場域設定的 `devices.csv`（`machine:TA1`），或來自追蹤（`person:trk_88`）；steps 的 object 寫「類別:實例」。
每類：`locate`、`allowed_actions`（和動作的 `objects` 要互相一致，載入器檢查）、`safety_distance_m`（{quadruped, uav}，
null = 原文的「—」，`per_instance` = 依設備）、`rules`（no_overfly、no_contact、no_enter、avoid_ground、
approach_needs_approval、high_voltage_zone）、`findings`、`note`。

## event_types.yaml

格式照原文第七節：`event_types:` 底下「名稱 → 內容」，欄位 `category`（template／open）、`detectors`、`trigger`（zone_attr、
time_rule）、`accumulate`（lambda、theta_on、theta_off、min_duration_s）、`associate`（dt_s、r_m）、`vlm`（prompt_id、samples、
timeout_s）、`cost`（miss、false）、`locate`、`default_rule`、`priority`（P0～P3）、`requires_approval`。
我們加的：最上面的 `detectors:` 表（每個偵測器的 labels、source_kinds、min_score、status），事件的 `object_class`、`expires_s`、`status`。

| 事件 | 來源 | 優先級 | 偵測器 | 範本 |
|---|---|---|---|---|
| `perimeter_intrusion` | 原文 | P0 | person_yolo_v1、vms_line_crossing（planned） | perimeter_intrusion |
| `unknown` | 原文（開放事件） | P2 | —（VLM 判讀，requires_approval） | — |
| `fire` | 我們 | P0 | **apriltag_fire（已有）**、fire_smoke_yolo | fire_response |
| `smoke` | 我們 | P0 | fire_smoke_yolo | smoke_check |
| `thermal_anomaly` | 我們 | P1 | thermal_hotspot | thermal_check |
| `left_object` | 我們 | P2 | left_object_yolo | left_object_check |

累積的意思（原文只給參數，這裡定義）：同一個關聯群組每來一筆偵測 E ← λ·E + score，E ≥ theta_on 而且持續 min_duration_s
才成立，E < theta_off 解除；派遣門檻：事件機率 ≥ false / (miss + false)。範本本身在 `site-config/<site>/event_rules.yaml`（PLN-1，M2）。

## site-config（位置標準，原文第五節）

| 類型 | 檔案 | 例子（yard） |
|---|---|---|
| 區域 zone | `zones.geojson`（map 座標的多邊形；properties：restricted、high_voltage、no_entry〔哪些平台不能進，uav = 禁飛〕、floor、cruise_altitude_m、ceiling_m） | Z_YARD、Z_HV_A～C、Z_HV_TC6、Z_TANKS、Z_CONTROL、Z_PADS |
| 觀察點 poi | `pois.csv`（zone、role〔front／rear／overwatch〕、platform、座標、朝向） | OBS_HV_A_FRONT… |
| 設備點位 asset | `devices.csv`（「類別:實例」、座標、zone、安全距離、狗與 UAV 的觀察姿態） | machine:TA1～TC6、tank_1… |
| 充電／起降 dock／pad | `docks.csv` | PAD_1～3、DOCK_1～3 |
| 座標 pose | `{frame: map, floor, x, y, z}` | — |

狗與無人機共用同一個 map 座標系與樓層編號；無人機的高度由動作參數或區域屬性決定，不由 VLM 決定。
檢查（`checkSite`）：狗的觀察姿態、觀察點、充電樁不在狗的禁入區；UAV 的觀察點不在禁飛區、不高於天花板；
fleet.yaml 的 landing_pads、docks 都在 docks.csv。

## 怎麼新增（我們或伯宇都可以）

| 要加的 | 改哪裡 | 注意 |
|---|---|---|
| 事件類型 | `event_types.yaml` 的 `event_types` 加一筆 | 偵測器還沒有就在 `detectors` 加一筆、標 `status: planned`；label 要和偵測器發的 `Detection` 一樣（≤ 64 B） |
| 物件類別 | `objects.yaml` 加一筆 | 同時在用到的動作的 `objects` 加上（兩邊要一致） |
| 設備 | `site-config/<site>/devices.csv` 加一行 | id 寫「類別:實例」；machine 要填安全距離 |
| 動作 | `actions.yaml` 加一筆 | 名稱照 2-1b；BT 那邊同時加 Switch 的 case |
| 能力 | `tools.yaml` 加一筆（新的 bit）＋ `cbba_core/capabilities.hpp` 一起加 | bit 只增不改 |
| 平台 | `platforms.yaml` 加一筆（新的 code） | CBBA 出價要另外對應（目前只認 uav、quadruped） |
| 載具 | `fleet.yaml` 加一筆 | 機號 1～254 不重複；三處機號要一致（cbba_node、BT、這裡） |

加完跑檢查（隔離編譯後）：`/out/build/swarm_cbba/test_registry`；或直接用 `registry_dir` 開 cbba_node，有錯會印出哪一份、哪一筆。

## cbba_node 怎麼用

```bash
ros2 run swarm_cbba cbba_node --ros-args ... -p registry_dir:=$HOME/CBBA_BT/registry
# 或環境變數 CBBA_REGISTRY_DIR=$HOME/CBBA_BT/registry
```

- 載具：從 `fleet.yaml` 找自己的機號，平台要和 `agent_type` 一致（uav = UAV、quadruped = UGV），能力取代 `capabilities` 參數；
  平台的成本參數從 `platforms.yaml`。不在清單裡的機號照參數（印警告）。
- 閘道：全隊的 `gateways` 參數改用 `fleet.yaml` 的。
- 沒給 `registry_dir`：照舊用參數，行為和之前一樣。cbba_node 不讀 site-config。

## 目前的內容

模擬的機隊（docker：uav1～3、dog1～3、gcs）與 yard 場景。能力照節點原本的預設（UAV：mobility_air、rgb_camera；狗：mobility_ground、
rgb_camera），所以給不給 `registry_dir` 出價都一樣。實機（V60 的雲台變焦、熱像、喇叭；室內無人機的 indoor_flight、高度 3 m、
起降板）待確認（2-1b 第十節）。
