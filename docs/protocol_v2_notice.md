# 機間通訊協定版本 2：變更通知

2026-10-09 · 勝翔（無人機端）· 詳細設計見 `ros2_ws/src/cbba_core/doc/protocol.md`

給三種人：狗端（BT、cbba_node 的使用者）、規格書作者、全隊。

---

## 一、全隊：要一起升級

- 協定版本 1（照「機間通訊封包規格」2026-10-06）和版本 2 的節點**互相拒收**（magic、version 不同），
  不能一部分升級。所有跑 cbba_node 的機器要用同一版的 `cbba_core`。
- `swarm_interfaces` 升到 **v1.4**，`RobotState`、`Task`、`TaskResult` 都改了、另外新增三個訊息，型別雜湊變了：
  **用到它的套件全部要重新編譯**，否則舊版的節點收不到新版的訊息。
- `cancel_authorities`（可以取消任何任務的機號）全隊要設成一樣。
- 每台只能跑一個 cbba_node（程序鎖 `agentN.lock`）；docker 的容器共用主機的 `/tmp/cbba_locks`。
- 無人機端還沒在 PX4 SITL 重跑版本 2，步驟在 `cbba_core/doc/sitl_test.md`（第 1～5 步、3b、4b）。

## 二、狗端：要改的地方

| 項目 | 以前 | 現在 |
|---|---|---|
| 編譯 | `ugv_cbba` 套件，要有 px4_msgs | `colcon build --packages-select swarm_interfaces cbba_core`，**不用 px4_msgs** |
| 啟動 | `ros2 run ugv_cbba ugv_cbba_node` | `ros2 run cbba_core cbba_node --ros-args -r __node:=v60_cbba_node -p vehicle_type:=ugv -p agent_id:=50`（全隊有設 `cancel_authorities` 的話也要加） |
| `/v60/robot_state` | 位置、電量 | 多 `flight_state_valid`、`armed`、`offboard`、`landed`：**狗填 false 就好**（不填就是 false） |
| `/v60/assigned_task` | Task | 多 `assignment_version`：這次正式指派的版本。**同一個任務也可能收到新版本**（重新指派），BT 要記最新的 |
| `/v60/task_result` | task_id、success、detail | 多 `assignment_version`：**原樣帶回** assigned_task 上的版本。task_id 或版本不是目前的正式指派就拒絕（log：`回報拒絕`），任務狀態不變 |
| 執行失敗（`success: false`） | 交回競標池 | 一樣：任務保留、交回競標池、狗不再接；**不是取消** |
| `/v60/new_task` | 狗不能建立任務 | 狗**也能建立任務**（task_id = 50 × 65536 + 流水號） |
| `/v60/cancel_task`（新） | — | 取消任務（只看 task_id）。狗是建立者、或在 `cancel_authorities` 裡才有效 |
| 無人機完成任務 | 不等狗的確認 | 也等狗的確認 |
| `/v60/exec_state`（新） | — | **BT 要送**（約 2 Hz）：執行狀態、可否中斷、預估剩餘時間、執行中／排隊中的任務。1.5 s 沒更新就不能接新任務 |
| `/v60/assignment_request`、`assignment_response`（新） | 得標就直接發 assigned_task | **得標後先請 BT 接受**：ACTIVATE（要中斷時先停止舊的、交接完再回覆）、RESERVE（排隊）、RELEASE（作廢）。拒絕要帶原因（暫時／永久）；3 s 沒回覆當成暫時拒絕 |
| 版本型別 | uint32 | `assignment_version` 是 **uint64** |
| 火警插單 | 新火點進來時執行中的任務會被換掉 | 只有「可以安全中斷、而且優先級較高（火警 ＞ 巡檢）」才中斷；同級的火警排在後面（最多 1 個，要在期限內做得完） |

測試資料（T1～T8 的指令已經加上版本）在 `cbba_core/doc/ugv_test.md`。

## 三、規格書作者：版本 2 和規格書不相容

規格書用 AGENT_STATE 裡 z、y 陣列的位置（uint8 任務編號）當全隊共用的任務識別。這在我們的情況造成：

1. 機號超過 8 的載具（狗 50）不能建立任務：任務編號依機號交錯切份，50 的份會和 uav2 撞。
2. 每台一輩子只能建立約 32 個任務（uint8、切成 8 份、已完成的不回收）。
3. 鄰居、存活名單是 8 個位元，完成確認不會等機號超過 8 的載具。
4. 重開機要把 seq 存在檔案裡（從 0 開始會被鄰居的去重複擋掉；只用時鐘的話沒有 RTC 時會往回跳）。

版本 2 的改法（逐位元組格式在各 `wire_*.hpp`）：

| 項目 | 版本 2 |
|---|---|
| 任務識別 | 每筆任務資料都帶 uint32 task_id（建立者機號 × 65536 + 流水號），各節點自己維護對照 |
| 表頭 | 20 B：magic 0x4342、version 2、type、sender_id uint16、session_id uint64、sequence uint32、payload_length；**大端序** |
| 重開機 | session_id 每次啟動隨機產生，取代 seq 存檔 |
| 封包 | AGENT_STATE（狀態）和 CBBA_STATE（共識資料，可分批 ≤ 1200 B）分開；TASK_ANNOUNCE、COMPLETION、COMPLETION_ACK、TASK_CLOSE |
| 正式指派 | 每個任務一個指派版本，跟著 CBBA_STATE 傳；回報、完成、TASK_CLOSE 都帶版本，舊的拒絕 |
| 權限 | 完成由有效的執行者回報；取消由建立者或授權的管理端發起 |

想請規格書作者決定的：

- 規格書要不要改成以 task_id 識別任務（其他模組，例如 FORMATION，是不是也有同樣的需求）。
- 重開機時序號怎麼處理，最好全規格統一（我們用 session_id）。
- 原本就要確認的：座標系（我們用 map ENU）、拒絕確認的處理、重送 10 次之後的處理。
- 取消任務的「授權管理端」是誰（地面站？）、地面站要不要跑 cbba_node。
