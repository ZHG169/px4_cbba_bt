# swarm_cbba：統一版 CBBA 節點

無人機和機器狗跑同一支 `cbba_node`，`agent_type`（UAV／UGV）與能量模型由 params yaml 決定。
2026-10-12 起**照機器狗端（伯宇，spec v1.1 §2）的套件與介面**：BT 端（狗的 `task_manager`、無人機的 BT）不用改。
核心、機間協定（UDP，協定版本 2）、正式指派流程在 `cbba_core`。
**機器狗端（伯宇）請看 [`README_v60.md`](README_v60.md)**：換哪些套件、編譯、啟動、驗收、和 10/07 版本的差異、常見問題。

## 啟動

```bash
# 狗（預設 yaml 就是 cbba_v60.yaml）
ros2 launch swarm_cbba cbba.launch.py namespace:=v60
# 無人機
ros2 launch swarm_cbba cbba.launch.py namespace:=uav1 params:=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config/cbba_uav.yaml
# 不用 launch：命令列的 -p 蓋過 yaml（第二隻狗不用另開 yaml）
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 --params-file <cbba_v60.yaml> -p agent_id:=51
```

無人機在 docker 模擬環境用 `docker/scripts/cbba_uav.sh`（會一起開 PX4 轉接、BT，並傳入對齊 PX4 模擬電池的能量模型）。

## 和 BT 之間的介面（話題在 launch 的 namespace 底下）

| 話題 | 方向 | 型別 | QoS | 說明 |
|---|---|---|---|---|
| `current_task` | CBBA → BT | `swarm_interfaces/Task` | reliable、transient_local、depth 1 | 目前要執行的任務；`task_id = 0` 表示沒有任務（取消）。同一任務連續 `assign_hold` 秒排第一才發 |
| `task_closed` | CBBA → BT | `swarm_interfaces/Task` | reliable、transient_local、depth 100 | 任務結束（`status` DONE／CANCELLED），每個任務一次。2026-10-13 我們加的，狗端不用訂；無人機發布火警後懸停監控，等地面處置完成才走 |
| `task_announced` | CBBA → BT | `swarm_interfaces/Task` | reliable、transient_local、depth 256 | 全隊已知的任務（自己建立或收到別台的 TASK_ANNOUNCE），每個任務一次。2026-10-14 我們加的，**只有 `agent_type: UAV` 才發**（狗的 cbba_node 沒有這個話題）；無人機的火點去重複要算進別台報過的火 |
| `robot_state` | BT → CBBA | `swarm_interfaces/RobotState` | BT 發 reliable；CBBA 收 best_effort | 約 2 Hz：位置（map ENU）、電量、`mode` |
| `task_result` | BT → CBBA | `swarm_interfaces/TaskResult` | reliable | 任務結束時一次：`success`、`detail`（給人看）、`reason` |
| `new_task` | BT → CBBA | `swarm_interfaces/Task` | reliable | 本機發現的任務（無人機：火點）；`task_id` = 機號 × 65536 + 流水號 |
| `cancel_task` | → CBBA | `swarm_interfaces/Task` | reliable | 取消任務（只看 task_id）：要是建立者或 `cancel_authorities`。機器狗端沒用到 |

規則：

- **參與出價**：`robot_state` 在 `state_timeout`（1.5 s）內有更新，而且 `mode == AUTO`（spec §2.3 規則 2、4）。
  `MANUAL`（平板接管）、`ESTOP`、沒有 `robot_state` 時對所有任務出價 0，手上的任務立刻釋放（`current_task` 變 0）。
  無人機的 `mode` 由 `px4_waypoint_node/px4_state_bridge` 決定：已解鎖、在空中，而且這次起飛後爬到過 `auto_min_altitude`（`cbba_uav.sh` 給巡檢高度 − 0.5 m）= AUTO，起飛完成才出價（2026-10-12）。
- **回報**：`task_result` 的 `task_id` 要是目前 `current_task` 上的任務，不是就拒絕（log 會寫），任務狀態不變。
  BT 不用帶指派版本；版本只在 CBBA 節點之間（機間封包）用，擋掉別台的舊回報。
- **失敗**（`success = false`）：任務交回競標池，這台依 `reason` 不再對它出價：
  `PERMANENT` 永久；`TRANSIENT`、`PREEMPTED`、`STOPPED` `failure_exclude_sec`（60 s）後恢復（spec §2.3 規則 3）。
- **不握手**：CBBA 直接發 `current_task`，BT 的哨兵負責中斷舊的任務。
- **執行中的任務鎖定**：已經發到 `current_task` 的任務，別台不出價；只有失敗、取消、更高優先級（火警 > 巡檢）搶占才換。
- **AIR_RECON** 的 z 低於 `recon_altitude` 時抬到巡檢高度（地面任務不改）。

## 參數（yaml 或 `-p`）

| 參數 | 預設 | 說明 |
|---|---|---|
| `agent_id` | 環境變數 `AGENT_ID`，沒有就 50 | 1～254 |
| `agent_type` | `UGV` | `UAV`／`UGV`：能接哪些任務、能量模型的預設值 |
| `ns` | 空字串 | 空 = 相對名稱（由 launch 的 namespace 決定）；給了就用 `/<ns>/…` |
| `state_timeout` | 1.5 s | `robot_state` 多久沒更新就不出價 |
| `assign_hold` | 0.4 s | 排第一多久才發 `current_task`（spec §2.3 規則 1：2 個廣播週期） |
| `failure_exclude_sec` | 60 s | 失敗（不是 PERMANENT）後多久不再對同一任務出價 |
| `takeover_fire`、`takeover_patrol` | 10 s、30 s | 失聯接手：已接受任務的持有者失聯後多久重新競標（火警／其他，全隊要一樣） |
| `recon_altitude` | 5.0 m | AIR_RECON 的巡檢高度 |
| `cruise_speed`、`energy_per_meter`、`hover_energy_per_sec`、`safety_reserve` | UAV：5、0.5、0.2、20；UGV：1、0.1、0.05、20 | 能量模型（狗的是佔位值，見 `cbba_core/doc/ugv_tuning.md`） |
| `battery_weight`、`cost_ref`、`max_bundle` | 1.0、50、5 | 出價公式，**全隊要一樣**（見 `cbba_core/doc/parameters.md`） |
| `mesh_ip` | 環境變數 `MESH_IP` | 機間 UDP 走哪張網卡；空 = 系統預設 |
| `udp_group`、`udp_port` | 239.255.42.99、14600 | 全隊要一樣 |
| `cancel_authorities` | [] | 除了建立者，可以取消任務的機號（全隊要一樣） |
| `lock_dir` | `CBBA_LOCK_DIR` → `$XDG_RUNTIME_DIR/cbba` → `/tmp/cbba` | 程序鎖：同一機號只能開一個 |
| `participate` | `state` | `always`＝不看 `robot_state`、一律參與（只用來測協商），位置與電量用 `initial_position`、`battery` |

## 測試

```bash
# uav 容器裡、編譯完；不用 PX4、BT（用 ros2 topic pub 假裝 BT），約 3 分鐘
bash src/swarm_cbba/test/g2_e2e.sh      # log 在 /tmp/cbba_g2（OUT= 可改）
```

| 組 | 內容 |
|---|---|
| G2 | 機器狗端（伯宇）的 T1 火點、T2 完成、T6 BT 停掉再恢復、T8 機號不對、T9 MANUAL、T10 相對名稱；log 字串照他的腳本 grep |
| E1～E4 | 和他 10/07 版本不同的行為：暫時失敗 60 s 後回來、PERMANENT 不回來、回報不是手上的任務被拒、處理火警時不排第二個 |
| DEMO3 | 三隻狗（50／51／52）＋uav1 搶一個火點：只有一台得標、別台站到火點上也搶不走（鎖定）、全隊確認完成 |

和正在跑的節點隔離：`ROS_DOMAIN_ID` 93、UDP port 14693、程序鎖 `/tmp/cbba_g2_lock`；節點名稱 `g2t_*`，結束時只關自己開的。
假的 robot_state 用一支 python publisher、mode 從檔案讀（切 mode 不能重開 `ros2 topic pub`：重開要 1～2 s，
超過 `state_timeout`，會變成逾時釋放而不是 MANUAL 釋放）。2026-10-12：58 項全過。

PX4 SITL 的整串（起飛、指派、火警、PX4 斷線）在 `docker/itest`（`python3 docker/itest/itest.py uav`）。

## 和機器狗端 handoff（2026-10-11，對照 10/07 的 `uav_cbba`）的差別

格式（套件、launch、yaml、話題、訊息、QoS）**完全一樣**。底下換成協定版本 2 的核心，所以分配的行為有幾點不同：
執行中的任務不會被別台搶走；處理火警時不會被第二個火警打斷（交給別台，沒有別台就等做完）；
持有者斷線時還沒開始的任務 1.5 s 後釋放、執行中的先鎖著，火警 10 s、巡檢 30 s 沒恢復才重新競標（失聯接手）；
只接受目前 `current_task` 上的任務的回報；任務數量、建立者機號沒有限制（舊版每台約 32 個、機號 > 8 不能建立）。
