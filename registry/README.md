# 註冊表（2-1b REG-1，schema 1.0，2026-11-14）

2-1b 的原則：**一份模組註冊表是唯一來源**，規劃器、檢查器、VLM 詞彙、CBBA 的可行性、BT 分派都讀它；
新增動作或機器人只改註冊表與 adapter。2-1b 的六份都有了（schema 1.0）；任務範本 `event_rules.yaml`（PLN-1）之後加。

| 檔案 | 內容 | 誰讀 |
|---|---|---|
| `tools.yaml` | 能力標籤：name、**bit**（CBBA 封包裡能力遮罩的位元位置）、description、deprecated | cbba_node（檢查和程式一致）、規劃器、檢查器 |
| `platforms.yaml` | 平台：name、**code**（封包裡的平台代碼）、成本模型（速度、耗電、存量、續航、max_bundle、must_return） | cbba_node（M0 只檢查；M1 拿來出價） |
| `fleet.yaml` | 機隊：每台的 id（機號）、name、platform、namespace、capabilities；gateways（閘道的機號） | cbba_node（自己的平台與能力、閘道的機號） |
| `actions.yaml` | 動作（steps 的 action）：2-1b 第二節的 8 個可規劃動作，每個的平台、需要的能力、location／object、參數、前置條件、失敗碼 | 規劃器、檢查器（展開、檢查 steps）、BT（葉節點收的參數）；cbba_node 只檢查格式 |
| `objects.yaml` | 對象：種類（人、車、變壓器、儲槽、火、煙…，可不可以 track／inspect）與場地上的設備（yard 的 TA1～TC6、儲槽、管架…，id 與位置） | 規劃器、VLM 詞彙、steps 的 object；cbba_node 只檢查格式 |
| `event_types.yaml` | 事件種類：偵測器與類別、累積門檻、VLM 確認、派遣代價、預設範本、優先級（先自訂 5 種：fire、smoke、intrusion、thermal_anomaly、left_object） | 閘道的事件流程（EVT-1）；cbba_node 只檢查格式 |

## 規則

- 每份檔案開頭 `schema_version: "1.0"`；版本不對、缺欄位、型別不對時 cbba_node 拒絕啟動。
- **bit、code 只增不改**：已經用過的不改、不重用；不要的標籤標 `deprecated: true` 留著。bit 0～63、code 1～255（0 保留給
  「任務不限平台」和「閘道」）。
- `tools.yaml` 要包含程式（`ros2_ws/src/cbba_core/include/cbba_core/capabilities.hpp`）認得的全部標籤、bit 一樣；
  註冊表可以比程式新（多出程式不認得的標籤，cbba_node 印警告、宣告了也用不到）。新增標籤時兩邊一起加。
- 機號 1～254，載具和閘道之間也不能重複。能力要在 `tools.yaml`、不能是 deprecated；平台要在 `platforms.yaml`。
- 能力故障（E_HW）在執行期由 cbba_node 扣掉，不改這份。

## actions.yaml（2026-11-14 初版，**參數暫定**）

| 動作 | 做什麼 | 需要的能力 | location | object | 參數（預設） |
|---|---|---|---|---|---|
| `go_to` | 走（飛）到一個點 | — | 必填 | — | `tolerance` 0.5 m、`timeout` 0（自動） |
| `patrol` | 照一串點巡邏 | — | 選填（正方形的中心） | — | `pattern` waypoints／square、`waypoints`、`size` 10 m、`loops` 1 |
| `inspect` | 到點後看、拍照 | rgb_camera | 必填（要看的點） | 選填（設備） | `snapshots` 1、`timeout` 30 s |
| `track` | 跟著目標 | rgb_camera | 選填（目標最後位置） | **必填**（目標） | `distance` 2 m、`duration` 30 s |
| `speak` | 喊話 | speaker | — | — | `text`（必填）、`repeat` 1 |
| `wait` | 原地等 | — | — | — | `duration`（必填） |
| `return_to_dock` | 回充電點（無人機是起降點） | — | 選填（沒給用自己的） | — | — |
| `alert_operator` | 通知值班的人 | — | — | 選填 | `message`（必填）、`level` warning |

- 全部都是 `plannable`、狗和無人機都能做、可以取消；每個動作都可能回 `common_failure_codes`（E_PRECONDITION、E_SAFETY_STOP、
  E_LOW_BATTERY、E_COMM、E_HW），另外各自的（go_to：E_UNREACHABLE、E_TIMEOUT；track：E_TARGET_LOST…）。
- 參數名稱、預設值是初版：`inspect` 照狗 BT 已經在用的（`snapshots`、`timeout`），`patrol` 的 10 m、`track` 的 2 m／30 s
  照無人機實機 API，其他是提案。**動作的 Action 介面（REG-2）由伯宇定，對過之後才凍結**。
- `inspect` 的前置條件 `at_location`：載具要已經在 location 附近（通常前一步是 `go_to`），檢查器（PLN-2）用。
- 還沒放的：系統動作（emergency_stop、emergency_land）、平台內部動作（UAV 的 takeoff、set_altitude，adapter 自動補）、
  report_state、未開放的（open_door、close_door、pick_up、use_tool）。

各平台的實作：

| 動作 | 狗（伯宇的 task_manager） | 無人機（uav_adapter，M1） |
|---|---|---|
| `go_to` | ✅ steps 的第一步 = 站的位置（外部點） | 待做（SITL：`NavigateTo`） |
| `inspect` | ✅ 雲台（`/ptz_inspect`） | 待做（SITL：`AirRecon`） |
| `patrol`、`track`、`speak`、`wait`、`return_to_dock`、`alert_operator` | 待做：Switch 加 case、`kValidActions` 加名字（spec patch 會照傳參數） | 待做 |

## objects.yaml、event_types.yaml（2026-11-14 初版）

- `objects.yaml`：12 種對象；設備照 yard 場景（`docker/gz/worlds/yard_env.sdf.inc`）的 21 個，steps 的 `object` 填設備 id（例如
  `{"action": "inspect", "object": "TB3"}`）。換場地時加另一個 `site` 的設備。
- `event_types.yaml`：2-1b 的事件種類之後會再增加，**目前是我們先自訂的 5 種**：

| 事件 | 優先級 | 主體 | 偵測器（類別） | 累積門檻 | VLM | 範本 |
|---|---|---|---|---|---|---|
| `fire` | P0 | fire | `apriltag_fire`（fire，≥ 0.85，**已有**）、`fire_smoke_yolo`（規劃中） | 2 s、2 m 內 3 筆 | 不用 | `fire_response` |
| `smoke` | P0 | smoke | `fire_smoke_yolo`（smoke） | 3 s、3 m 內 5 筆 | 要 | `smoke_check` |
| `intrusion` | P0 | person | `person_yolo`（person） | 2 s、3 m 內 5 筆 | 要 | `intrusion_response` |
| `thermal_anomaly` | P1 | hot_spot | `thermal_hotspot`（hot_spot） | 5 s、2 m 內 3 筆 | 不用 | `thermal_check` |
| `left_object` | P2 | left_object | `left_object_yolo`（bag、box、left_object） | 30 s、1 m 內 10 筆 | 要 | `left_object_check` |

  門檻、範本名稱都是初版；範本本身（要派哪些角色、steps）在 `event_rules.yaml`，之後做範本展開（PLN-1）時加。

## 怎麼新增（我們或伯宇都可以）

每份都是「一筆一個條目」，新增就是在清單後面加一筆，**不用改程式**：

| 要加的 | 改哪裡 | 注意 |
|---|---|---|
| 事件種類 | `event_types.yaml` 加一筆 | 名稱用小寫加底線、用過之後不改；偵測器還沒做就標 `status: planned`；`detector`、`label` 要和偵測器發的 `Detection` 一樣（≤ 64 B） |
| 對象種類、場地的設備 | `objects.yaml` 的 `classes`／`instances` 加一筆 | 設備 id 不重複；事件的 `object_class` 要在這裡 |
| 動作 | `actions.yaml` 加一筆 | 名稱照 2-1b；BT 那邊同時加 Switch 的 case 和 `kValidActions` |
| 能力 | `tools.yaml` 加一筆（新的 bit）＋ `cbba_core/capabilities.hpp` 一起加 | bit 只增不改；只加 YAML 不改程式時 CBBA 用不到（會警告） |
| 平台 | `platforms.yaml` 加一筆（新的 code） | code 只增不改；CBBA 出價要另外對應（目前只認 uav、quadruped） |
| 載具 | `fleet.yaml` 加一筆 | 機號 1～254 不重複；三處機號要一致（cbba_node、BT、這裡） |

加完跑檢查（隔離編譯後）：`/out/build/swarm_cbba/test_registry`；或直接用 `registry_dir` 開 cbba_node，有錯會印出哪一份、哪一筆，並拒絕啟動。
多人同時改用 git：各自加在清單後面，衝突通常只是相鄰行，照兩邊都留的方式合併。

## cbba_node 怎麼用

```bash
ros2 run swarm_cbba cbba_node --ros-args ... -p registry_dir:=$HOME/CBBA_BT/registry
# 或環境變數 CBBA_REGISTRY_DIR=$HOME/CBBA_BT/registry
```

- 載具：從 `fleet.yaml` 找自己的機號，平台要和 `agent_type` 一致（uav = UAV、quadruped = UGV），能力取代 `capabilities` 參數；
  不在清單裡的機號照參數（印警告）。
- 閘道（`participate: never`）：自己的機號應該在 `gateways`；全隊的 `gateways` 參數（判斷閘道在不在線）改用 `fleet.yaml` 的。
- 沒給 `registry_dir`：照舊用參數（`capabilities`、`gateways`），行為和之前一樣。

檢查：`swarm_cbba` 的單元測試 `test_registry` 讀這個目錄，和 `capabilities.hpp` 比對。

## 目前的內容

模擬的機隊（docker：uav1～3、dog1～3、gcs）。能力照節點原本的預設（UAV：mobility_air、rgb_camera；狗：mobility_ground、
rgb_camera），所以給不給 `registry_dir` 出價都一樣；實機（V60 的雲台變焦、熱像等）的能力待和伯宇確認。
`platforms.yaml` 的成本參數和現在 `swarm_cbba/config/cbba_uav.yaml`、`cbba_v60.yaml`（及程式預設）一樣，M1 改成從這裡讀之後，
那兩份的對應參數就拿掉。
