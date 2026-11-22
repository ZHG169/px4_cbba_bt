# 執行端與「能做的動作」：決定紀錄（2026-11-19～20）

來源：使用者 11/19 的〈CBBA 模擬先行架構（BT 後端可替換）〉與〈CBBA G0～G21 三個模擬場景〉PDF。11/19～20 照它試做了一套
模擬執行端（TaskExecutionBridge、SimExecutionBackend、profile、Gazebo 替身、itest `--exec-backend sim`），
**使用者 11/20 決定拿掉**：那是另外加的一套流程，不是 CBBA 驗證的必要元件。程式備份在 session 的暫存目錄
（`execution_bridge_backup_20261120.tar.gz`），沒有 commit。

## 保留的

| 項目 | 內容 |
|---|---|
| `Task.assignment_version` | current_task 帶正式指派的版本（swarm_interfaces 2.9.0）；執行端（真的 BT）用來擋舊的指令、判斷同一個任務換了一次指派 |
| 能做的動作（`unsupported_action`） | **由 cbba_node 用註冊表推導，不由執行端另外宣告**（使用者 11/20）：`actions.yaml` 裡 status open、class 不是 internal、有這台平台的實作，而且 `requires.any_of` 有一組能力全部在這台的可用能力裡（`fleet.yaml` 宣告的、扣掉執行中故障的）。實作的 autonomy（not_started…）不看，那是核准與檢查器警告用的。TaskSpec 有不能做的動作 → 不出價（出價表原因 `unsupported_action`），手上的交出去；TaskSpec 一收齊就判斷。沒有 `registry_dir` 時不限制。程式：`swarm_cbba::supportedActions`（registry）、cbba_node 的 `updateSupportedActions`、`CbbaComm::setSupportedActions`／`setActionLister`、`Task::unsupported` |
| 執行中故障 | 沿用既有機制：TaskResult 帶 `failure_code: E_HW`＋`failed_capability` → 那個能力暫時不可用（`failed_caps`），到 `capability_restored` 或重開為止；推導出的動作清單跟著重算 |
| 閘道的 TaskSpec 檢查 | 平台還沒實作的動作只警告（`PLATFORM_NOT_SUPPORTED` 當警告），照收；各台由推導出的清單決定能不能出價，沒人能做時由孤兒任務、P0 看門狗告警或 fallback。不認得的動作、參數、location 等照樣拒絕 |
| 範本 | itest group 的入侵、火警範本照 PDF 第 6 頁（協同等待寫在 coupling，不寫死 wait；alert_operator 改 report_policy 的 operator 頻道） |
| `fleet.yaml` | 三隻狗加 `speaker`（入侵範本的 speak 要它；V60 的喇叭待伯宇確認） |

## 拿掉的

`SupportedActions.msg`（執行端宣告）、`execution_bridge` 節點（Bridge、SimExecutionBackend、KinematicPlatform、FakeGcs）、
`config/simulation.yaml`、`real_uav.yaml`、`test_execution`、`bridge_iface.sh`、`docker/scripts/gz_world.sh`、`gz_body_sync.py`、
itest 的 `--exec-backend`、`--gazebo`。itest group 照原本的做法（測試代替 BT 回報進度與完成）。

## 真的 BT

照 2-1b 文件：BT 用通用的 `ExecuteSteps` 依序呼叫 `ExecuteStep.action`（`registry/api.md` 第 5 節），或依任務用專用子樹；
看 `teammates` 的 gate、回報 `task_progress`、`task_result`（失敗碼、`failed_capability`）。`fake_adapter`（ExecuteStep 草案）保留。
