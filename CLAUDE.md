# CLAUDE.md

給在這個 repo 工作的 Claude 的說明。使用者負責**無人機端**（PX4、BT、AprilTag、空中 CBBA、mesh＋tc）；
機器狗（Ghost V60）端由隊友負責。回覆與文件一律用繁體中文。

## 專案

空地異質系統的無人機端：PX4 旋翼機以 CBBA 分散式拍賣分配任務，交給行為樹（BT）執行。

```
docs/                       跨套件的文件（目錄在 docs/README.md）：sim_launch_guide.md（**模擬測試的開啟方式**）、
                            progress.md（**目前進度與缺失清單**，持續更新）、sim_scenario_plan.md（巡檢＋隨機插入、電池權重）、
                            protocol_v2_notice.md（協定版本 2 給狗端與規格作者的變更通知）、reports/（進度快照 PDF）、archive/
docker/                     模擬環境（sim、uavN、gcs 容器；mesh_net 可用 mesh_tc.sh 加掉包）
                            gz/：Gazebo 場景 fire_site.sdf（火警 tag 101、誘餌 tag 7、巡檢點）、patrol_site.sdf（3 綠＋5 紅，
                            由 gen_patrol_site.py 從 patrol_site_points.csv 產生）、demo_site.sdf、yard.sdf（伯宇的變電所，
                            yard_env.sdf.inc＋yard_points.csv 產生，2026-10-24）與 tag 0～7、101、mono_cam 模型
registry/                   2-1b 註冊表（schema 1.0）：tools.yaml（能力＋bit）、platforms.yaml、fleet.yaml；cbba_node 的 registry_dir
ros2_ws/src/
├── swarm_interfaces/       ROS 訊息 v1.6（＝機器狗端的 v1.2＋FireDetection；Task、RobotState、TaskResult、ManualOverride.srv…）
├── cbba_core/              CBBA 核心、機間封包、協定層、指派流程、離線模擬、測試（不依賴 ROS）；
│                           狗的文件在 doc/ugv_test.md、doc/ugv_tuning.md
├── swarm_cbba/             統一版 cbba_node（無人機和狗同一支，agent_type）、launch、yaml：照機器狗端的套件與介面（2026-10-12）
├── px4_waypoint_node/      PX4 端：takeoff_hover 起飛懸停、task_executor（簡易執行器）、px4_state_bridge（PX4 → RobotState）
├── uav_px4_bt/             無人機的 BT（正式 uav_task_manager_node＝伯宇骨架、備用 uav_bt_node）、避碰（avoidance：路權＋VO＋二階 CBF）、
│                           flight_beacon 位置廣播
├── apriltag_fire_detector/ 下視相機 AprilTag 火情偵測 → /uavN/fire_detection（FireDetection）
└── px4_msgs/               由 docker/scripts/setup_px4_msgs.sh 下載（PX4 v1.17.0）
```

## 編譯與測試

主機沒有 g++／colcon，用 `cbba-robot:jazzy` 映像。**不要編到共用的 `ros2_ws/build`、`install`**
（使用者在容器裡用，兩邊同時編譯會壞），用另外的目錄：

```bash
OUT=<暫存目錄>
docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp \
  -v /home/zhg/CBBA_BT:/home/ncrl/CBBA_BT -v $OUT:/out cbba-robot:jazzy bash -c '
  source /opt/ros/jazzy/setup.bash && source /home/ncrl/CBBA_BT/ros2_ws/install/setup.bash &&
  cd /home/ncrl/CBBA_BT/ros2_ws &&
  python3 -m colcon --log-base /out/log build --build-base /out/build --install-base /out/install \
    --packages-select swarm_interfaces cbba_core swarm_cbba px4_waypoint_node uav_px4_bt apriltag_fire_detector --cmake-args -DCMAKE_BUILD_TYPE=Release &&
  cd /out/build/cbba_core && for t in test_*; do ./$t; done'
```

- 掛載路徑要是 `/home/ncrl/CBBA_BT`（共用 install 裡的 px4_msgs 用絕對路徑）。
- 容器裡沒有 `colcon` 指令，用 `python3 -m colcon`。
- 編譯要零警告（`-Wall -Wextra -Wpedantic`）。
- 使用者自己在 uav 容器編譯：`colcon build --symlink-install --packages-select swarm_interfaces cbba_core swarm_cbba px4_waypoint_node uav_px4_bt apriltag_fire_detector`。
- 跑 `uav_px4_bt` 的測試前要 `source` ROS 環境（`test_tree` 要 ament_index）。
- PX4 SITL 測試步驟：`cbba_core/doc/sitl_test.md`（`cbba_uav.sh`、`cbba_task.sh` 在 `docker/scripts`）。
- 參數說明：`cbba_core/doc/parameters.md`（全部參數、要互相一致的參數、已知限制）；出價公式的實驗在 `cbba_core/cbba_parameters.md`。
- 無人機 BT 的飛行參數：`uav_px4_bt/doc/parameters.md`（每個參數的作用、症狀 → 調哪個）；值在 `uav_px4_bt/config/uav_bt.yaml`，
  `cbba_uav.sh` 用 `--params-file` 載入（install 裡是連結，改完重開就生效）；速度、高度用 `CBBA_SPEED`、`CBBA_ALTITUDE`（cbba_node 一起改）。
- 自動整合測試 `docker/itest/itest.py`（comm、uav 兩組，只看話題與網路）；要在隔離環境跑時另起一套 compose（不同 project 名稱、網段、domain、鎖目錄）。
- 多節點整合測試：同一個容器起多個 `swarm_cbba/cbba_node`（各自 `-r __ns:=/uavN -r __node:=uavN_cbba_node`，假的 robot_state 用 `mode: 0`，或 `participate:=always`；程序鎖用 `CBBA_LOCK_DIR` 指到暫存目錄），
  用 `ros2 topic pub` 代替 BT；要測轉接節點時對 `/uavN/fmu/out/*` 發假的 PX4 話題。設 `ROS_DOMAIN_ID` 避免干擾使用者的容器。
  **停節點要用 `pkill -f "__node:=uavN_cbba_node( |$)"`**，只 kill `ros2 run` 外層，節點還會繼續跑。

## cbba_core 的分層

| 層 | 檔案 | 說明 |
|---|---|---|
| ROS 節點（在 `swarm_cbba` 套件） | `swarm_cbba/src/cbba_node.cpp`（統一版，只吃 RobotState／TaskResult／Task；PX4 由 `px4_waypoint_node/px4_state_bridge` 轉接；程序鎖） | 接感測與 BT；不寫邏輯 |
| 指派流程 | `assignment_manager.*` | 正式指派：搶占、排隊、拒絕、逾時、分段量測（BT 接受握手的程式還在，節點 2026-10-12 起固定不用）；不依賴 ROS，用模擬網路測（`test_assignment`） |
| UDP | `udp_link.*` | multicast，綁 mesh 網卡（`MESH_IP`） |
| 協定層 | `cbba_comm.*` | CBBA ⇄ 封包；不依賴 ROS 與 socket，用模擬網路測（`test_comm`） |
| 封包格式 | `wire_io`、`wire_header`、`wire_agent_state`、`wire_cbba_state`、`wire_task_announce`、`wire_completion` | 只管位元組格式 |
| CBBA 核心 | `cbba_agent`、`scoring`、`energy_model`、`types` | 17 條消解規則、bundle、出價 |

## 機間通訊：協定版本 3（2026-10-28；版本 2 是 2026-10-09，使用者要求「每筆任務資料帶 task_id」）

> **2026-11-11 起協定版本 4**（2-1b M0）：64 位元 task_id、紀錄 27 B、摘要 24 B、TASK_ANNOUNCE 帶 A 層、TASK_SPEC，和版本 3 互相拒收。
> 變更見「2-1b 的 CBBA 合約」「2-1b M0 的 CBBA 實作進度」；本節的其他規則照舊。

**版本 3 只改補發**（使用者 2026-10-28 決定，設計在 `cbba_core/doc/protocol_v3_sync.md`）：版本 2 的補發是「round_key 不同就把所有
已結束的任務各補一次」，流量和已結束的任務數成正比（弱網測試 70%：重開節點 13 kB/s、累積 1350 個任務 78 kB/s）。版本 3：
CBBA_STATE 帶壓縮摘要（每個建立者一筆 14 B）→ 比對出不同的建立者送 GAP_REQUEST（流水號點陣圖）→ 對方只補缺的；同時直接推送最近
`push_recent_sec` 內新增、結束的任務 → 用 SYNC_BATCH 分批、token bucket 限速（`repair_rate` 8 kB/s）。版本 2、3 互相拒收，
全隊（含狗的 cbba_node）要一起重新編譯；cbba_node 的 ROS 話題、BT 都不受影響。下表其他項目照舊。

整體說明在 `cbba_core/doc/protocol.md`，逐位元組格式在各 `wire_*.hpp` 開頭。**不相容於規格書**
（`cbba_core/doc/spec/機間通訊封包規格_20261006.pdf`）：規格用 AGENT_STATE 的 z、y 陣列位置（uint8 任務編號）當全隊共用的
任務識別，造成機號 > 8 不能建立任務、每台約 32 個任務、鄰居位元只放得下 1～8。版本 1、2 的節點互相拒收，全隊要一起升級。
**DDS 只用在機內**（PX4、BT），機間全走 UDP。

| 封包 | 內容 |
|---|---|
| 表頭 20 B | magic 0x4342、version 3（2026-10-28 前是 2）、type、sender_id u16、session_id u64、sequence u32、payload_length u16；**大端序**、逐欄位寫 |
| AGENT_STATE（1） | 參與、遙測、飛行狀態有效、**真實的 armed／offboard／landed**、位置、電量、BT 的執行狀態（狀態、可否中斷、預估剩餘時間）、執行中／保留的任務與版本（有無用旗標）、路徑、鄰居清單；74 + 4L + 2K B；每 0.5 s |
| CBBA_STATE（2） | round_key、exchange_step、stable_steps、snapshot_id／part_index／part_count、s（機號＋時間）、紀錄（task_id 4、winner_id 2、bid 4、assign_version 8、assignee_id 2、assign_state 1 共 **21 B**）；每 200 ms＋變動時；每批 ≤ 1200 B |
| TASK_ANNOUNCE（3） | 任務定義（61 B），轉送 |
| COMPLETION（4）／COMPLETION_ACK（5）／TASK_CLOSE（6） | 宣告 → 確認（status 0 不認得／1 接受／2 指派已過期）→ 結束（reason 1 完成、2 取消；actor），都帶指派版本，轉送；存活名單是機號清單 |

已定案的決定（改之前先和使用者確認）：

| 項目 | 做法 | 原因 |
|---|---|---|
| 任務識別 | 每筆任務資料帶 uint32 task_id（機號 × 65536 + 流水號），各節點自己維護對照 | 不用先對「第 i 格是哪個任務」有共識；任何機號都能建立任務、數量不限 |
| 機號 | 協定 uint16（1～65534），0 = 無人，拒收；節點限制 1～254 | `RobotState.agent_id` 是 uint8 |
| 無人得標 | winner_id = 0 | 和現有程式一致 |
| 紀錄 | 只放認得、進行中的任務；不認得、已結束的省略 | 版本 1 用 y = −1 區分；省略就不會有「z = 0 兩種意思」的問題 |
| 重開機 | **session_id**（每次啟動隨機）取代 seq_file；新 session 接受，退役 session 的封包丟掉 | (origin, session, seq) 不會和開機前撞，不用存檔、不受時鐘往回跳影響 |
| 舊封包 | 每個發送者一個**滑動視窗**（4096 號）：視窗內沒收過的接受（亂序也收）；狀態的新舊看 snapshot_id（CBBA_STATE）、sequence（AGENT_STATE） | 只收「比較新的」會把一次送出、亂序到達的大量封包丟掉（測試發現） |
| 去重複的鍵 | （種類, origin_id, origin_session, origin_seq） | 轉送事件只處理一次 |
| 重送 | **一律換新序號**；確認者「原樣回送」指決定不變 | 沿用舊序號時，多跳下第一次轉送掉了就會被轉送者當重複擋掉，永遠補不回來 |
| 分批 | CBBA_STATE 每批帶完整的 s，**每批直接交給規則**；收斂判定、補發只用收齊的那一次；缺批等下一次 | 規則逐任務判斷；缺少的紀錄不能當成任務被刪 |
| 沒有任務定義的紀錄 | 不暫存，靠補發（摘要比對出缺口） | 200 ms 後下一次會再帶 |
| 已停止參與的載具 | 收到的紀錄裡得標者是「已知停止參與」的鄰居時略過 | 晚到的訊息會把它寫回得標者，等 1.5 s 失聯才釋放（測試發現，版本 1 也有） |
| 截止時刻 | 絕對時刻（ms，約 49.7 天循環） | 轉送／補發原封不動，期限不會被往後推 |
| round_key | 已知任務（task_id）與是否結束的 CRC-32 摘要 | 相同就不用比對摘要；開機同步也用它 |
| exchange_step | 本機分配表變動次數 | 我們是非同步 CBBA，沒有「步」 |
| s | 機號 + 系統時鐘 ms 低 32 位元 | 需要各機對時（chrony） |
| 補發 | **版本 3（2026-10-28）**：壓縮摘要 → 缺口請求 → 限速的分批回覆（上面一段）；每 5 s 保底重播一個任務。**不參與出價的節點照樣送紀錄（不送 s）**，收的那端只合併正式指派、不交給規則（2026-10-11 起，原本不送紀錄） | 晚起飛、重開機、飛出範圍再回來；停止參與時自己撤銷的指派要傳得出去（SITL 測試發現：飛行中關掉 PX4，任務卡住沒人接） |
| 開機同步 | 摘要和任一鄰居相同才送出自己建立的任務；1.5 s 沒鄰居＝單獨；5 s 對不上就開始 | 查出 BT 重開機後重用 task_id（不送出、記 `rejected_local_tasks`） |
| 先到的 TASK_CLOSE | 任務還不認得時先暫存；執行機是自己也套用 | 補發時可能比任務先到；重開機後要知道自己完成過的任務 |
| 已結束的任務 | 一直記著，不回收 | 忘記的話，斷線的那台補發時會把它加回來 |
| 完成確認 | 要確認的 = 宣告當下存活名單 ∩ 現在還活著的（機號清單，狗也算） | 宣告後失聯的不用等 10 次重送 |
| 失聯 | s 超過 1.5 s 沒更新就釋放它的任務；變成不參與時立即釋放 | 規格的 1.5 s |
| 非同步 CBBA | 只重送目前狀態 | 規格是同步式，「重送上一步」對非同步沒意義 |
| 正式指派版本（2026-10-09） | 每個任務一個 uint64 版本、全隊共用：**請 BT 接受時預先分配**（已知最新 + 1，狀態待接受），接受後同版本變保留或執行中；拒絕、逾時、撤銷（換掉、搶占、失敗、之後的 constraint）時 + 1（執行者 0、狀態無），作廢的版本不重用；放在 CBBA_STATE 紀錄裡，取（版本, 狀態, 執行者）較大的 | 區分每次正式指派；其他節點經 gossip 取得正式指派紀錄，才能驗證 UDP 的完成通知 |
| 回報檢查 | task_result 的 task_id 要是目前 current_task 上的任務（版本由節點自己補，BT 不帶，2026-10-12 起）；COMPLETION／TASK_CLOSE 的版本比已知的舊就拒絕（ACK status 2、不結束） | 擋掉任務換掉、X→Y→X 之後、別台的舊回報 |
| 同時指派 | 同版本時已接受的勝過待接受的，同狀態才比機號 | 剛加入、還不知道鎖定的那台不能蓋掉已接受的 |
| 鎖定（容量承諾） | 別台「保留」或「執行中」的任務不出價；持有者的核心不讓別台的得標資訊拿走它；待接受不鎖。撤銷、失敗 → 重新開放；完成、取消 → 永久關閉；**持有者失聯 → 照樣鎖著、只標記待確認**，到「失聯接手」的時限才撤銷 | 2026-10-09 使用者確認 |
| 失聯接手（2026-10-11 定案，2026-10-12 實作：`CbbaComm::takeOverLost`、參數 `takeover_fire`／`takeover_patrol`） | 失聯的那台把手上的任務做完（「不接新的」沒有另外實作：各機分不出自己是不是失聯的那台）；別台在持有者失聯（s 超過 lost_timeout 沒更新）後，**火警（GROUND_INTERVENTION）10 s、巡檢 30 s** 才撤銷鎖定、重新競標（撤銷 = 新版本，執行者 0）；恢復連線後舊持有者的回報版本較舊，照現有的回報檢查拒絕（ACK status 2） | 「通訊斷但還在做」和「墜毀」從外面分不出來：火警寧可重複、巡檢寧可等。時限之後依第 5 週弱網的量測調整；狗照同一套規則，要和伯宇、宜臻對齊 |
| BT 接受（握手） | **2026-10-12 起不用**（照機器狗端）：排第一 `assign_hold` 0.4 s 就發 current_task，BT 的哨兵負責中斷。核心的握手程式和測試保留（`require_accept` 固定 false，之後要地面端最終接受權時再打開） | 使用者 2026-10-12 決定照伯宇的介面 |
| 失敗後的排除（2026-10-12；10-17 加打折） | TaskResult.reason：PERMANENT → 這台永久不再對它出價；PREEMPTED → `preempt_exclude_sec`（狗、無人機的 yaml 都 5 s，2026-10-17：只擋 mode = MANUAL 晚到的空檔；接管期間靠 MANUAL 不出價，做完回到 AUTO 就能接回）；TRANSIENT／STOPPED → `failure_exclude_sec`（60 s）後恢復。**TRANSIENT 另外累計失敗次數 f**，之後出價用成功率 p = 1/(1+f)：價值 × p、到達時間加 (1−p)/p ×（前往＋執行），只在本機（`Task::success_prob`、`CbbaAgent::countFailure`）；`max_failures`（0 = 不啟用）達到時永久排除 | spec §2.3 規則 3（機器狗端）；打折是宜臻的實作說明（需求 1）：只排除 60 s 時，時間到又派回同一台、再失敗 |
| 搶占與排隊 | 優先級：BT 本機安全處置 ＞ 火警（GROUND_INTERVENTION）＞ 巡檢；同級不搶占；已接受的任務釘在路徑前面。2026-10-12 起沒有 ExecState：一律當可中斷、剩餘時間不知道 → **執行中的火警之後不排隊第二個火警**（交給別台，或做完再接）；搶占時評分不算被搶占任務的延後、出價不受 bundle 上限（被搶占的會交回重新競標，2026-10-12），巡檢中的 V60 才搶得到身邊的火警 | Accept = F ∧ [Idle ∨ (P_j > P_k ∧ Preemptible)] |
| TASK_CLOSE 權限 | 完成：actor = origin = 執行者、版本不舊；取消：actor 是建立者或 `cancel_authorities`（全隊一樣）；不合的不套用、不轉送 | 完成由有效執行者回報；取消由建立者或授權管理端發起 |
| 失敗與關閉分開 | 執行失敗不送 TASK_CLOSE：撤銷指派、任務保留、自己依 reason 暫時或永久不出價，由 CBBA 重新分配 | 之後的 constraint 撤銷也走同一條路 |

## 統一版 cbba_node 與 BT 的介面（2026-10-12 起照機器狗端，無人機和狗相同）

2026-10-11 伯宇給了 handoff（`docs/handoff_uav_2026-10-11/`）：他用 GitHub `dog-integration` 上 10/07 的 `uav_cbba`（協定版本 1）
包了一個統一版 `swarm_cbba` 和介面 v1.2。**使用者 2026-10-12 決定：BT 之間的格式完全照他的，機間協定和分配行為照 main（協定版本 2）**，
所以他的 BT、task_manager、launch、yaml 都不用改，只要換成 main 的 `swarm_interfaces`、`cbba_core`、`swarm_cbba` 重新編譯。
詳細在 `swarm_cbba/README.md`；他的 README 的 T1、T2、T9 在我們的節點上照跑過，結果一致。

| 話題（launch 的 namespace 底下） | 方向 | 用法 |
|---|---|---|
| `current_task`（Task） | CBBA → BT | 目前要執行的任務（reliable、transient_local）；排第一 `assign_hold` 0.4 s 就發；沒有任務時 task_id = 0、status = CANCELLED |
| `task_closed`（Task） | CBBA → BT | 任務結束（status DONE／CANCELLED），每個任務一次（transient_local、depth 100）；**我們加的**（2026-10-13），狗端不用訂；無人機火警監控用 |
| `task_announced`（Task） | CBBA → BT | 全隊已知的任務，每個任務一次（transient_local、depth 256）；**我們加的**（2026-10-14），預設只有 agent_type = UAV 才發（參數 `publish_announced`，itest 的 `--dogs` 給狗 true 量送達，2026-11-09）；無人機的火點去重複（別台報過的火不再發布、不轉向、不搶著監控） |
| `task_orphaned`（Task） | CBBA → 地面站等 | 孤兒任務（2026-10-17，我們加的，狗、無人機都發）：開著但 `orphan_warn_sec`（30 s）內沒有人得標、也沒有正式指派，發一次＋log 警告；有人得標時印一行。只通知、不處理 |
| `robot_state`（RobotState） | BT → CBBA | 位置（map ENU）、電量（負值 = 不知道）、`mode`（AUTO／MANUAL／ESTOP）；`state_timeout` 1.5 s 內有更新**而且 mode = AUTO** 才出價，否則手上的任務立刻釋放。無人機由 `px4_state_bridge` 發；**mode 由正式 BT 決定**（2026-10-17，照機器狗端）：BT 送 `/uavN/bt_mode`（std_msgs/UInt8），起飛完成、BT 在控制 = AUTO，地上、爬升中、被接管、PX4 失效保護、返航 = MANUAL，急停 = ESTOP；備用 BT 時 bridge 自己判斷（在空中、爬到巡檢高度 − 0.5 m、offboard）；第一筆當返航點 |
| `task_result`（TaskResult） | BT → CBBA | `success`、`detail`（只記 log）、`reason`（失敗後的排除時間）；task_id 要是目前 current_task 上的，否則拒絕 |
| `new_task`（Task） | BT → CBBA | task_id 建立者必須是自己，不能和已知的重複；佇列深度 256；AIR_RECON 的 z 低於 `recon_altitude` 時改成巡檢高度 |
| `cancel_task`（Task） | → CBBA | 只看 task_id；要是建立者或 `cancel_authorities`（我們多的，狗端沒用） |

| 項目 | 做法 |
|---|---|
| 啟動 | `ros2 launch swarm_cbba cbba.launch.py namespace:=v60`（無人機加 `params:=…/cbba_uav.yaml`）；`cbba_uav.sh` 用 `ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/uavN -r __node:=uavN_cbba_node --params-file cbba_uav.yaml -p …` |
| 參數 | `agent_id`（預設環境變數 AGENT_ID 或 50）、`agent_type`（UAV／UGV，預設 UGV）、`ns`（空 = 相對名稱）、`failure_exclude_sec` 60 |
| 測協商 | 假的 robot_state 用 `mode: 0`；或 `participate:=always`（位置與電量用參數） |
| AGENT_STATE 旗標 | robot_state 沒有飛行狀態了：flight_state_valid、armed、offboard、landed 送 false；鄰居開始／停止參與時印 log |
| 指派版本 | 只在 cbba_node 之間（CBBA_STATE、COMPLETION、TASK_CLOSE）；BT 的話題都不帶 |
| 程序鎖 | cbba_node 啟動時對 `<lock_dir>/agentN.lock` 加 flock，拿不到拒絕啟動，退出才釋放。docker 共用主機 `/tmp/cbba_locks`（掛 `/run/cbba`）；沒設時 `/tmp/cbba` |
| 分段量測 | 每個指派印「建立 → 排第一 → 發布」；2026-10-12 量到約 0.4 s（assign_hold 0.4 s），**不能宣稱符合 300 ms** |

`cbba_core/doc/spec/ugv_interface.png` 是**機器狗端**的介面規格圖（RobotState、TaskResult、ManualOverride.srv 的定義來源）。

### 狗

| 項目 | 做法 |
|---|---|
| 程式 | 伯宇的 task_manager（BT）＋我們的 `swarm_cbba/cbba_node`（`cbba_v60.yaml`）；不用裝 px4_msgs。給他的使用說明：`swarm_cbba/README_v60.md` |
| 機號 | **50**（介面規格）。協定版本 2 起和其他機號沒有差別 |
| 參與出價 | robot_state 1.5 s 內有更新且 mode = AUTO（平板接管 MANUAL、急停 ESTOP 時不出價）。狗只接地面任務 |
| 平板接管 | `ManualOverride.srv`（平板 → 狗的 BT，不經過 CBBA）；狗的 BT 先回報手上的 CBBA 任務失敗（PREEMPTED，60 s 內不接），再發 mode = MANUAL |
| 能量模型 | 佔位值（0.1 %/m、0.05 %/s、1 m/s），待伯宇實測，見 `cbba_core/doc/ugv_tuning.md` |
| 和他 10/07 版本的行為差異 | 執行中的任務不會被搶走（鎖定）、處理火警時不排隊第二個、持有者斷線時執行中的先鎖著、只接受目前指派的回報、任務數量與機號不再受限（`swarm_cbba/README.md` 最後一節） |
| 測試 | 他 README 的 T1、T2、T9（2026-10-12 在我們的節點上照跑，結果一致）；`cbba_core/doc/ugv_test.md` 的 T1～T8 是舊介面的紀錄 |

## 分支（2026-10-12）

| 分支 | 內容 |
|---|---|
| `main` | 2026-10-10 起包含全部（通用 cbba_node、協定版本 2、BT 接受流程、uav_px4_bt、apriltag_fire_detector、模擬測試場景），以 `12e36bb` 為基礎直接 commit 目前的工作目錄 |
| `hetero-tasking` | 2026-11-14 起：異質機隊的任務分派（多種載具分工、同一事件拆成多個角色），2-1b M0 全部（協定版本 4 的後半、閘道、介面 v2.0、DETECTION、註冊表、給伯宇的清單）。GitHub 的 main 停在 `2553ca8`（11/12），伯宇 pull main 不會拿到；約好一起換時再合進 main。本機的 main 也指到 `a4c716b`（沒 push），之後在這個分支上做 |
| `dog-integration` | GitHub 上是 `e802fbb`（10/07 的 `uav_cbba`＋`ugv_cbba`，協定版本 1；伯宇的 handoff 就是對照這版），不再用 |

- 兩個分支共用 `ros2_ws/build`、`install`：**切換分支後要重新編譯**，否則 install 裡是另一個分支的程式。
  `uav_cbba` 改名成 `cbba_core` 後，舊的 `install/uav_cbba`、`install/ugv_cbba` 不會自動刪掉。
- push 要用 GitHub 的 Personal Access Token（主機和容器都沒有存憑證）。
- 通用節點的規劃與決定見 `docs/archive/generic_cbba_node_plan.md`（已實作）；協定版本 2 見 `cbba_core/doc/protocol.md`。

## 週計畫（2026-10-05 ～ 11-13，5 週開發＋1 週緩衝）

分工：**勝翔＝無人機**（PX4 SITL、無人機 BT、AprilTag 火情確認、空中 CBBA 出價、mesh＋`tc`）；
**伯宇＝機器狗 V60**（Gazebo 模型、Nav2、狗的 BT 與 50 ms 中斷哨兵、機械臂／開關復位、地面 CBBA 出價）；
伯宇＋宜臻：黑板資料契約（`robot_state`、`current_task`、`new_targets`）、雙方 CBBA 消解規則一致。

閉環情境：無人機飛抵巡檢點 → 下視相機看到 AprilTag 101、confidence ≥ 0.85 → 建立 GROUND_INTERVENTION（火點座標）
→ 無人機對地面任務不出價、狗得標 → 狗的 BT 在 50 ms 內中斷 Nav2、掉頭前往。出價可行性：無人機只做 AIR_RECON，
狗只做 GROUND_INTERVENTION／PATROL（`types.hpp` 的能力檢查）。

| 週 | 日期 | 勝翔（無人機） | 伯宇（狗） | 里程碑 |
|---|---|---|---|---|
| 1 | 10/05～ | PX4 SITL＋XRCE-DDS、起飛懸停；CBBA 訊息定義；mesh＋`tc` 弱網環境 | V60 URDF 進 Gazebo；黑板標頭 | 兩種載具同一個世界、話題命名空間清楚 |
| 2 | 10/12～ | PX4 3D 航點巡邏；空中出價（地面任務 0） | Nav2 平滑調校；地面出價（空中任務 0） | 各飛／走完 3 個點；5 個混合任務各自只對專長出價 |
| 3 | 10/19～ | AprilTag 火情確認（> 0.85）；`ReportFireEvent` 注入地面任務 | `IsTaskValidAndUnchanged` 中斷哨兵、Mock 插單 | 無人機辨識後自動發任務；狗被插單後平滑掉頭 |
| 4 | 10/26～ | **Phase 1**：空地全流程聯調 | 同左 | 黑板覆寫 → cancel_goal ≤ 50 ms；火情發布 → 狗開始轉向 ≤ 300 ms；cmd_vel 平滑 |
| 5 | 11/02～ | **Phase 2**：`tc` 掉包 0／10／30／50／70 %、延遲 50±30 ms，注入多源任務；100 % 斷網 | 監看高掉包下的狀態轉移、連鎖退標 | 5 s 內收斂率（30 % 掉包 ≥ 95 %）；掉包率對收斂時間、頻寬曲線 |
| 6 | 11/09～ | 一鍵啟動、數據報表、容器化（Jetson Orin） | 同左 | 驗收報告、展示影片 |

週計畫和實作不同的地方（照實作；要對外說明）：
- 交付物 `mesh_bridge_node` 沒有獨立節點：機間 UDP 在 `cbba_core`（`udp_link`、`cbba_comm`），弱網用 `docker/scripts/mesh_tc.sh`。
- `CBBAMessage.msg`（y、z 向量）已被協定版本 2 取代：機間走 UDP 的 AGENT_STATE／CBBA_STATE，每筆帶 task_id；DDS 只在機內。
- 週計畫的「18 條消解規則」：核心實作 17 條（`cbba_rule_cases.csv`）。實驗規劃 PDF 也寫 18 條，使用者 2026-10-23 確認是誤植，以 17 條為準。
- 「黑板」對我們是 `cbba_node` 的話題（`current_task` 等）；狗的 `robot_blackboard.hpp` 是狗的機內實作。
- 命名空間是 `/uavN`（多台），不是 `/uav`。
- 無人機 BT 和週計畫 XML 的差異（加 TakeOff、頂層 ReactiveFallback、HoverAndMonitor 有期限並回報 task_result、
  FlyToWaypoint 逾時回報失敗）寫在 `uav_px4_bt/include/uav_px4_bt/bt_nodes.hpp` 開頭。
- 300 ms 全流程：2026-10-09 量到指派約 0.8 s（`assign_hold` 0.6 s 佔大部分），目前達不到，要和伯宇、宜臻分預算。

### 前三週（無人機端）的進度：**都已在 PX4 SITL 驗證**（2026-10-11～12，`docker/itest`）

| 週 | 項目 | 程式 | 驗證 |
|---|---|---|---|
| 1 | PX4 SITL、XRCE-DDS、起飛懸停 | `docker/`、`takeoff_hover` | 完成（itest U1：3 台起飛） |
| 1 | 機間訊息、mesh＋`tc` | 協定版本 2、`mesh_tc.sh` | 通訊測試、30%／100% 掉包通過；DDS 只在機內（2026-10-12） |
| 2 | 3D 航點、空中出價 | 備用 BT 的 FlyToWaypoint（CPF）、相容表 | itest U2、U5（三台分配；避碰交叉情境還沒測） |
| 2 | 混合任務只對專長出價 | `types.hpp` 能力檢查 | 真 UDP（假狗）、伯宇的 T1 |
| 3 | AprilTag 火情確認 | `apriltag_fire_detector`、`fire_site` | itest U3（約 1.6～1.9 s、誤差 < 0.5 m）、U4（誘餌不觸發） |
| 3 | `ReportFireEvent` 注入地面任務 | 備用 BT | itest U3；狗（假狗）得標 |
| 2～3 | 巡檢＋隨機插入、電池權重 | `patrol_site`、`patrol_scenario.sh`、`mission_sim`＋`cbba_sim insert` | 離線 100 次×5 權重已跑；Gazebo 抽驗還沒跑 |

正式的 UAV BT 改用伯宇 2026-10-12 的骨架（`docs/handoff_uav_2026-10-12/`，見「無人機 BT：改用伯宇的骨架」），寫好後用同一套 itest 重驗第 2、3 週。

### 第 5 週：弱網、100% 斷網（2026-11-09 完成；空地 3 架＋3 隻狗）

| 項目 | 做法 | 結果 |
|---|---|---|
| 狗的容器 | `docker/.env` 的 `DOG_COUNT=3` → `gen_compose.py` 產生 `cbba_dog1～3`（同一個 robot 映像、沒有 PX4；機號 50～52、`/v60`、`/v60_1`、`/v60_2`；mesh `172.30.0.50～52`）。**線材所有狗共用一條 `sim_dog_net`（172.30.20.0/24，sim .20.2、dogK .20.(10+K)）**：sim 的線材網卡要 ≤ 4 張（有狗時 `UAV_COUNT` ≤ 3，gen_compose 會擋） | 空地每條機間連線都經過 tc；demo 的狗（cbba_node＋BT）也搬進去（`config.json` 的 `dog.units[].container`） |
| Fast DDS 的坑 | ① 遠端位址預設只記 4 個：`fastdds/profile.xml.template` 加 `remote_locators` 16；② **PX4 的 XRCE agent 用 PX4 自己送的 XML 建參與者，不吃這份設定**（仍 4 個）→ sim 網卡 > 4 張時 agent 可能丟掉自己那條線、sim 收不到那台的 PX4 話題（11/09 demo：uav1、uav2 沒有位置） | 所以狗共用一條線；`itest.py` 的 `dds_diag`（U1 失敗時寫 `dds_diag.txt`：sim 看到幾個發布者、收不收得到） |
| itest `--dogs N`（comm、loss） | 狗加進 agent 清單（`Env.add_dogs`；容器、namespace、tc、頻寬照機號對應，名稱 dog1～dogN）；loss 任務空地混合：每台 2 個 → 空中 6＋地面 6（每隻狗 1 火警〔建立者是無人機〕＋1 巡檢〔CBBA 的 PATROL 任務，不是狗 BT 的日常巡邏點〕）；跨能力檢查（應為 0）；C1～C8 只測無人機，C9 含狗 | — |
| cbba_node 新參數／話題 | `publish_announced`（預設只有 UAV 發 `task_announced`；測試的狗給 true 量送達）；`cbba_tx`（UInt64MultiArray，[全部, 重傳, 轉送] 送出位元組累計，每 `tx_stats_sec` 0.2 s，送進 UDP 之前、tc 丟掉的也算）；`CommStats` 的 `TxKind`：重傳＝CBBA_STATE 快速重送、任務定義／結束的重送、完成宣告第 2 次以後、GAP_REQUEST、SYNC_BATCH | 頻寬改量「每台 TX」：任務注入後固定 10 s 區間（loss 每次試驗至少 10 s）；只算 CBBA（UDP 14600）；**模擬沒有限速、沒有共用頻寬**（各自、無上限；單一廣播域、一跳全連通；掉包在送出端，同一封包對所有接收者同時成立或遺失），圖上是傳送需求 |
| 主要數據 `out/20261109_173015_loss`（簡報第 7 頁，`plot_loss.py` → `loss_sweep.png`；備用 `loss_backup.png`） | 收斂時間＝每個任務只剩一位持有者 − 任務建立；Median／P90 用全部 20 次、逾時記成 10 s；5 s 收斂率要求「≤ 5 s 而且沒有重複認領」 | 30%：**100%**、0.75／1.10 s、每台 TX 3.12 kB/s（重傳 0.40）；70%：90%（18/20：1 次逾時、1 次 1.96 s 但有重複認領）、2.31／4.27 s、TX 5.18（重傳 2.18）；跨能力 0。舊的 `20261109_124416_loss`（沒有 TX）70% 是 85%，簡報統一用新的 |
| 100% 斷網 | C9（`comm --dogs 3 --only C9`，`out/20261109_124315_comm` PASS）；demo 情境 `scenarios/blackout.json`（等到狗和無人機手上都有任務才斷，全部容器 `mesh_tc.sh set 100` 20 s；報告 B1～B4；`plot_blackout.py` → `blackout_timeline.png`，itest 自動畫） | `20261109_170653_demo`（B1、B3 是修正前的量法，數字以這裡為準）：失聯偵測 ≈ 0.9 s；手上任務 2/2 由原本那台斷網中做完；火警斷網 10.9 s 後 dog2、dog3 都接手（設計：火警寧可重複）→ dog3 多做一次、dog2 恢復時取消；dog1 的完成版本較舊被拒收，火警恢復後 22 s 才全隊結束。斷網 8 s vs 20 s 的對照還沒跑 |
| 影片 | 狗的偵測範圍（`demo.dog_detect.radius` 10 m，`demo_plan.json` 記 `dog_detect`）：2D 動畫、`gazebo_labeled.mp4` 淡色虛線圓，發現火的前後變紅標「dogN sees Fk」；demo 動畫中途不停頓（`--freeze` 預設 0）；做完的任務 Gazebo 刪標記（`patrol_scenario.sh remove`）、2D 旗子變淺灰、`gazebo_labeled` 標籤拿掉；狗的日常巡邏點 A1～／B1～／C1～（每隻狗一組字母，紫色 `task_marker_p`，做完變灰／消失、下一圈再出現，`dog_patrol.py` 讀狗 BT 的 log） | — |

## 下一階段：2-1b 標準化模組、任務規劃器與事件偵測（2026-11-16 ～ 2027-01-15，9 週）

來源：使用者 2026-11-10 給的〈2-1b.標準化模組、任務規劃器與事件偵測（多狗＋無人機）〉PDF（Notion 頁；上層 2-1 監視事件轉 CBBA 任務、2-1a 整合實作計畫、2-1c 任務積木編輯器〔羅凡〕）。只分析過、還沒開始做。

| 項目 | 規格 |
|---|---|
| 核心 | **一份模組註冊表是唯一來源**：`registry/` 的 actions、objects、tools、platforms、fleet、event_types 六份 YAML；規劃器、檢查器、VLM 詞彙、CBBA 可行性、BT 分派都讀它。新增動作或機器人只改註冊表與 adapter |
| 動作 | ROS 2 Action（Goal：task_id、step_id、action、object、location、params、deadline；Feedback；Result：status、failure_code、findings；必須可取消）。8 個失敗碼：E_PRECONDITION、E_UNREACHABLE、E_TIMEOUT、E_TARGET_LOST、E_LOW_BATTERY、E_SAFETY_STOP、E_COMM、E_HW。分級：可規劃、系統（急停、緊急降落）、平台內部（UAV 的 takeoff、set_altitude 由 adapter 自動補）。清單：go_to、patrol、inspect、track、speak、wait、return_to_dock、alert_operator、report_state、emergency_stop／land（open／close_door、pick_up、use_tool 未開放） |
| 無人機 | 實機是 Node-RED 的 `/MAV1/api/*`（takeoff、set_altitude 3 m、hover、patrol 10 m 正方形、track 2 m／30 s、land、state／status），要包成標準 Action：`uav_adapter`（回饋、取消、平台狀態機 LANDED／AIRBORNE／EMERGENCY）；多台用 `/MAV2`…；室內、高度 3 m、續航 900 s、起降板 |
| 能力 | `fleet.yaml` 每台的能力標籤（mobility_ground／air、indoor_flight、stairs、rgb_camera、zoom、thermal_camera、speaker、spotlight、gas_sensor、manipulator、comm_relay） |
| 規劃器 | 事件 → TaskSpec（group_id；每個 role 有 platform、required_capabilities、steps；coupling：precedence、sync_window_s）。範本展開（`event_rules.yaml`）＋ VLM 提案；**九層檢查器**（格式、詞彙、參數、前置條件鏈、區域、機隊可行性、續航、風險、規模）。BT 通用 `ExecuteSteps` |
| 事件偵測 | 偵測器外掛輸出統一 Detection（detector、source、t、label、score、bbox、track_id）；`event_types.yaml`（偵測器、觸發、累積門檻、VLM、派遣代價、預設範本） |
| CBBA 異質（CB-6；2-1a 寫 CB1 宜蓁、CB2 學弟，**使用者 2026-11-11：全部 CB 項目由使用者負責**） | 能力過濾、各平台自己的成本模型、UAV bundle 上限 1～2、precedence、optional 角色 |

**勝翔（文件寫：三人中負荷最重）**：REG-1 註冊表 schema＋載入器、UAV-1 `uav_adapter`＋平台狀態機、UAV-2 SITL 接 Gazebo、PLN-1 範本展開器、PLN-2 九層檢查器、EVT-1 偵測器外掛介面＋`event_types.yaml`、UAV 端協同與 CBBA 介接。伯宇：Action 介面與失敗碼、狗 8 個動作、`ExecuteSteps`、搶佔取消、追蹤交接。羅凡：2-1c 積木編輯器（Node-RED）。
里程碑：M0 **11/20** schema v1.0 凍結、`/registry` 格式；M1 12/4 UAV 在 SITL 經 uav_adapter 完成 go_to、patrol、inspect、track、return_to_dock（可取消、有回饋）；M2 12/18 展開器、檢查器、EVT-1、`ExecuteSteps`；M3 12/31 Gazebo 2 狗＋1 UAV：入侵範本、空中先到定位、追蹤交接、UAV 缺席降級、CBBA 異質介接；M4 1/8 實機；M5 1/15 驗收。文件的預案：uav_adapter 延誤時，檢查器的「續航」「機隊可行性」移到 M3。

**和現有架構對照（2026-11-10 分析）**：
- 可沿用：UAV SITL＋Gazebo（UAV-2 幾乎已完成）、cbba_node＋協定版本 3、伯宇骨架的哨兵（＝文件的 IsTaskStillMine）、`NavigateTo`≈go_to、`AirRecon`≈inspect、回起飛點≈return_to_dock（但現在不降落）、`apriltag_fire_detector` 當第一個偵測器外掛、itest。
- 角色交給 CBBA ＝ 之前分析的「角色拆分」（每個 role 一個任務，核心與 17 條規則不改；要加湊齊閘門、連鎖撤銷、teammates 話題）。
- 要先釐清：① **無人機控制有兩套**（SITL 是 PX4 offboard、實機是 `/MAV1/api/*`）→ uav_adapter 要同一個 Action 介面、兩種後端（SITL 接 `Px4Platform`）；② 無人機上跑 cbba_node 還是地面站代理（文件待確認問題；我們現在是機上直接跑）；③ ~~steps 各台用註冊表在本機展開~~（11/11 改成獨立的 TASK_SPEC 封包，見下一節）；④ `isCompatible` 從載具種類改成能力集合；⑤ 8 個失敗碼對應現在的 TaskResult.reason（PERMANENT／TRANSIENT／PREEMPTED／STOPPED）；⑥ 室內 3 m、起降板 vs 現在的戶外 yard 8 m。
- 建議順序：M0 先定 actions、fleet、platforms 三份 → uav_adapter 介面（SITL 後端用 `Px4Platform`）→ 範本展開器 → 檢查器先做格式、詞彙、參數、前置條件鏈 → EVT-1 用火警偵測。
- **使用者 2026-11-11 的順序：先做 CBBA（CB 全部），再做無人機（UAV-0／1／2）。**

### 2-1b 的 CBBA 合約（2026-11-11 定案；全文 `cbba_core/doc/cbba_m0_contract.md`）

**原則（使用者 2026-11-11）：以我們的 CBBA 架構為主。** 2-1／2-1b 的規格目前都還在調整，CBBA 端照這份合約做；之後要對齊別人
（閘道、規劃器、伯宇的 BT、羅凡的編輯器）時，**只調整通訊的話題、訊息欄位**（誰送、怎麼收到），BT 的邏輯理論上不受影響、
話題本身影響也不大。所以：不要為了配合尚未定案的外部介面去改 CBBA 核心、協定的行為；介面差異放在 cbba_node 的話題層或轉接節點。

| 項目 | 定案 |
|---|---|
| 協定 | 版本 4（和版本 3 互相拒收，全隊一起重新編譯）。M0 只定合約，實作從 M1 起 |
| task_id | **uint64 = 建立者 16 ｜ 世代 16 ｜ 流水號 32**；**舊型 ID 相容模式**：上半 32 位元 = 0 的是舊型（`機號 × 65536 + 流水號`，數值同版本 3，epoch = 0，BT 自己算，保留到 M3）；版本 4 ID 只由 cbba_node／閘道產生（不加 `cbba_epoch` 話題）；**狗端通訊由使用者統一負責**：第 2 步時狗和無人機的 current_task、BT 內部任務識別、task_result 一起改成 uint64（BT 流程不動）；摘要每筆固定 24 B（known、closed 為 uint32，逐欄序列化）；世代每次啟動 + 1 並存檔（閘道讀不到就拒絕啟動；機器人讀不到用隨機世代＋警告）；同一機號同時兩個 session → 告警、拒收較晚的。group_id 同格式。log 印 `creator.epoch.seq`。`swarm_interfaces` 的 Task／TaskResult／Bid 的 task_id 改 uint64（伯宇的 task_manager 約 12 處要改） |
| 閘道（2026-11-13 使用者定義） | **非競標的 CBBA 協定節點，負責建立、同步、監控與關閉任務；不屬於任何載具平台，不列入任務可行性與 optional 角色的能力統計**。AGENT_STATE 的 vehicle_type = 0、能力 0、不參與（不一致的當壞封包）；`CommConfig::gateway`；`CbbaComm::capableAgents` 不算閘道 |
| 誰建任務 | **統一由閘道（GCS）建**：閘道跑不出價的 cbba_node（`participate: never`，機號暫定 200，列進全隊 `cancel_authorities`），`inject_task` service，冪等鍵 (incident_id, role, revision)。機器人的偵測（無人機看到火等）送 DETECTION 給閘道，不自己建任務；舊型 `new_task`（TaskType）保留到 M3 |
| 閘道斷線 | 機器人**不自己展開任務**（也不展開 fallback）：已知任務照常競標、執行；新的偵測暫存在 cbba_node，恢復後送出。斷線判定 `gateway_timeout` 3 s。fallback_template 只在閘道在線時由閘道重建較簡化的任務組；CBBA 不自己刪減角色。代價：斷網中看到的新火警要等閘道恢復才有任務 |
| 能力過濾 | 可出價 ⇔ task.platform ∈ {ANY, agent.platform} ∧ required_caps ⊆ available_caps；取代 `isCompatible`。能力 uint64 位元遮罩、平台 uint8（0 ANY、1 uav、2 quadruped），位元位置寫在註冊表 `tools.yaml`／`platforms.yaml`、只增不改；載具能力從 `fleet.yaml`；E_HW 拿掉故障的能力；AGENT_STATE 廣播 platform、available_caps。舊 TaskType 自動對應（AIR_RECON → uav＋mobility_air、rgb_camera；GROUND_INTERVENTION／PATROL → quadruped＋mobility_ground） |
| 任務資料三層 | A（CBBA 要的，TASK_ANNOUNCE 約 120 B：id、revision、group／role、platform、caps、位置、樓層、時間、priority、value、搶占、協同、spec 長度與 CRC）；B（steps 等執行要的，**獨立 TASK_SPEC 封包**、分段（encoding 0 原文；zlib 保留、暫不用，2026-11-12）、轉送＋補發、各台快取）；C（證據、說明，留在閘道） |
| CBBA_STATE 紀錄 | 每筆**統一 27 B**：task_id 8、revision 2、winner 2、bid 4、assign_version 8、assignee 2、assign_state 1（正式指派三欄保留） |
| TaskSpec 未到 | 可以出價；排第一但沒有該 revision 的 TaskSpec → `WAITING_SPEC`：不發 current_task、不建立正式指派（不鎖定）。`spec_wait` 5 s 超時**不算失敗**，只加強補發＋告警（`task_spec_missing`）；收齊立刻發 |
| 出價比較 | priority（P0～P3，0 最高）→ 同級分數 → 分數差在 score_epsilon 內比機號（小的勝）。priority 是任務屬性，封包的 bid 只放同級分數；分級用在組 bundle 先挑高級、搶占、排隊。**同級分數暫時沿用現在的公式**（half_life_s 只帶不用） |
| 異質成本 | 平台模型從 `platforms.yaml`（速度、耗能、續航、safety_reserve、max_bundle uav 2／quadruped 5、must_return）；UAV 能耗以剩餘飛行時間比例計；硬期限不可行 |
| revision | 只有建立者能改；只改位置／時間／價值 → 指派保留；改 platform／caps／priority → 撤銷重新競標。BT 要能處理「同 task_id、新 revision」（要和伯宇對） |
| 搶占 | P0～P3；任務的 `can_preempt` 遮罩（預設比自己低的級），同級預設不搶；**commit_radius 由 cbba_node 用 robot_state 位置算**（離開 1.5 倍才解除），接近後不可搶 |
| 過期 | expires_ms 到了各節點本機停止出價、撤銷自己的指派；建立者送 TASK_CLOSE（reason 3 = 過期） |
| 失聯接手 | 依 priority（預設 P0 10 s、P1 15 s、P2／P3 30 s） |
| 失敗碼 → reason | E_PRECONDITION、E_UNREACHABLE、E_HW → PERMANENT（E_HW 另外拿掉能力）；E_TIMEOUT、E_TARGET_LOST、E_COMM → TRANSIENT；E_LOW_BATTERY、E_SAFETY_STOP → STOPPED |
| optional 角色關閉 | 閘道送 TASK_CLOSE（reason 4 = 略過）：O1 全隊沒有能做的載具，`membership_ready` 後計時 10 s；O2 有人能做沒人接 15 s；O3 同組必要角色都結束——**斷線時各節點只停本機出價、不產生 TASK_CLOSE、不中途取消執行中（或已保留）的**；O4 組取消／過期。precedence 是否視為滿足看該角色的 `skip_satisfies`（範本明確設定）；關閉後不自動重開 |
| 執行端介面 | `swarm_interfaces` v2.0 只加欄位：current_task 加 revision、group、role、priority、platform、caps、`spec`（TaskSpec JSON）；task_result 加 failure_code、failed_capability；新話題 `teammates`、`assignments`（閘道）、`task_spec_missing` |

### 2-1b M0 的 CBBA 實作進度（2026-11-11～14）

照合約分步做，每步都能單獨編譯、測試；測試由使用者跑（隔離編譯＋單元測試、容器重編、itest）。協定、訊息改了要在 uav 容器重編 6 個套件，
訊息改了狗的 BT 也要 `dog_bt.sh build`。

| 步驟 | 內容 | 主要程式 | 狀態 |
|---|---|---|---|
| 1 能力過濾、P0～P3 | `fitness()`：平台 ∈ {ANY, 載具平台} ∧ 需要的能力 ⊆ 可用能力（取代 `isCompatible`）；13 個能力標籤（位元位置只增不改）；舊 TaskType 自動對應；組 bundle 先挑高級（P0 最高）、同級比分數；出價表原因 `platform`／`capability`；cbba_node 參數 `capabilities` | `capabilities.hpp/.cpp`、`types.hpp`、`cbba_agent.cpp`、`scoring.cpp` | ✅ 11/11 單元＋itest |
| 2a 協定版本 4 封包 | task_id 64 位元（舊型 ID 相容：上半 32 位元 0、epoch 0）；CBBA_STATE 紀錄 27 B（加 revision；revision 不同的那筆不交給規則）、摘要 24 B 鍵（建立者, 世代）；TASK_ANNOUNCE 123 B（A 層全部欄位，存在 `Task::ext`，M0 只傳送保存）；AGENT_STATE 帶能力、task_id 8 B；GAP_REQUEST 帶世代、流水號 4 B | `wire_*`、`cbba_comm.cpp`、`types.hpp` | ✅ 11/12 |
| 2b 機內 task_id uint64 | Task／TaskResult／Bid 的 task_id uint64、`cbba_path` UInt64MultiArray；無人機 BT（`TaskId`、`taskHex`）、`task_executor`、**伯宇狗 BT 介接層**（直接改 handoff；patch `docker/dog/task_manager_uint64.patch`，原檔備份 `docs/handoff_uav_2026-10-12/task_manager.orig_20261112`，不能放在 `bt/` 底下）；Python 不用改 | `swarm_interfaces`、`uav_px4_bt`、`cbba_node.cpp`、itest `config.json` | ✅ 11/12 itest comm --dogs 3（C0～C9）、uav |
| 2c 世代、同機號 | 世代檔 `<state_dir>/epoch_<機號>`（+ 1 寫回成功才用；`epoch_required` 讀寫失敗就拒絕啟動，給閘道）；`CbbaComm::newTaskId()`（版本 4 ID，給閘道）；同機號偵測：較早的 session 在較晚的出現 1 s 後還在講話 → 留較早的（`SessionFilter::pin`）、拒收較晚的封包與任務，較早的 5 s 沒聲音就解除；別台用了自己的機號 `selfConflict()`；`agent_conflict` 話題 | `cbba_comm.cpp`、`wire_header.cpp`、`cbba_node.cpp` | ✅ 11/12 單元＋itest comm |
| 3 TaskSpec | TASK_SPEC（種類 9，轉送）分段 ≤ 1100 B；encoding 0 原文、**1 zlib 保留暫不用**（使用者 11/12：先不加依賴）；收齊比對長度＋CRC；缺口請求的值 3「缺 TaskSpec」、缺的任務 1 s 後開始要；WAITING_SPEC（不建立正式指派、不發 current_task、AGENT_STATE exec_flags bit 4）；`spec_wait` 5 s 後向每個鄰居要＋`task_spec_missing`（不算失敗）；Task.msg 加 `revision`、`spec`，`current_task.spec` 一定是收齊的；`new_task` 帶 spec 會一起送；結束 60 s 後丟內容 | `wire_task_spec.hpp/.cpp`、`cbba_comm.cpp`、`assignment_manager.cpp`、`cbba_node.cpp` | ✅ 11/12（`2553ca8`；通訊測試通過） |
| 4a 閘道的注入 | `participate: never`（不訂 robot_state、永不出價）；`inject_task`（`InjectTask.srv`：A 層＋spec＋冪等鍵 (incident_id, role, revision)，同鍵回同一個 task_id、不重發；較舊拒絕；時間欄位是從注入起算的秒數）；group_id（同一 incident 的角色組共用）；`assignments`（`TaskAssignments.msg`）；`cbba_gateway.yaml`（200、`epoch_required`）；uav／v60 yaml 的 `cancel_authorities: [200]`。11/13：閘道不是載具（AGENT_STATE 平台 0、能力 0，`CommConfig::gateway`、`capableAgents`） | `gateway.hpp/.cpp`、`wire_agent_state`、`cbba_comm`、`cbba_node.cpp`、`swarm_interfaces`、`test_gateway`（5 項）、`test_wire_agent_state` | ✅ 11/13 單元＋手動測試（閘道 + 假狗） |
| 4b 改版 | `inject_task` 較大的 revision → `REVISED`（同一個 task_id）；`CbbaComm::reviseLocalTask`；只收建立者的改版；只改位置／時間／價值保留指派（`CbbaAgent::updateTask` 重算），改種類／平台／能力／優先級重新競標、持有者自己撤銷；摘要、round_key 算 revision，紀錄較新時向鄰居要（`revision_pulls`）；執行中改版、TaskSpec 收齊後重發 current_task。BT 端收到同 task_id 新 revision 還是忽略（要和伯宇對） | `cbba_agent`、`cbba_comm`、`assignment_manager`、`gateway`、`test_gateway`（+4 項 Revision） | ✅ 11/13 單元＋手動（閘道＋狗：改位置保留指派、改平台撤銷、重複、舊版拒絕） |
| 4c 過期 | expires 到了各節點本機永久不出價、撤銷自己的指派（`expireTasks`）；建立者（閘道）送 TASK_CLOSE reason 3，離線期間過期的回來再送；TASK_CLOSE 的 reason 開放 1～4（4 = optional 略過，4e 用）、3／4 本機狀態 CANCELLED、`closeReason()`；過期的不算孤兒任務 | `wire_completion`、`cbba_comm`、`cbba_node`、`test_gateway`（+3 項 Expiry）、`test_wire_completion` | ✅ 11/13 單元＋手動（閘道＋狗：10 s 過期、閘道送 reason 3） |
| 4d 搶占與接手 | 失聯接手依優先級（`takeover_by_priority` [10, 15, 30, 30] s；舊參數 `takeover_fire`／`takeover_patrol` 當預設）；commit_radius（`AssignmentManager::updateCommitted`：水平距離進半徑內不可被搶、1.5 倍解除；AGENT_STATE bit 5）；can_preempt 遮罩（`mayPreempt`，同級要遮罩明確開；preempt_margin 還沒用） | `types.hpp`、`cbba_agent`、`assignment_manager`、`cbba_comm`、`wire_agent_state`、`cbba_node`、`test_gateway`（+4 項） | ✅ 11/13 單元＋手動（閘道＋狗：半徑內 P0 搶不走、P0 10 s／P1 15 s 接手） |
| 4e optional 關閉 | 閘道 `Gateway::step`：O3（同組必要角色都結束，`groupMandatoryDone`）馬上關；沒有正式指派時 O1（沒有能做的載具，`membership_ready` 後 10 s）、O2（有人能做沒人接，15 s）；TASK_CLOSE reason 4（`dropOptional`）、cbba_node 印原因＋`task_orphaned`；`membershipReady`（`NeighborInfo::last_status`）；非閘道節點本機 O3 只停出價（`dropOptionalLocally`）。O4 不用另外做；`skip_satisfies` 只帶著（CB-4）；orphan 的原因欄位等 v2.0 | `gateway`、`cbba_comm`、`cbba_node`、`test_gateway`（+5 項 Optional） | ✅ 11/13 單元＋手動（閘道＋狗：O1 10 s 略過、O3 取消必要角色後馬上略過） |
| BT 接口（狗） | 外部 task manager 用 2-1b 的**通用 steps**（`step.action`，狗和無人機同一套名稱，11/13 使用者定）寫突發任務 → `current_task.spec`；狗 BT 的 `parseSpecSteps` 翻成外部點的 tasks（第一步 go_to 是站的位置、inspect 的 location → 雲台 target、沒寫 snapshots 拍 1 張），交給 SplitTask＋Switch；spec 空的照舊、不支援的回 `BAD_SPEC`（PERMANENT）。**`ptz_inspect` 改名 `inspect`**（雲台的 ROS action `/ptz_inspect` 不變）；itest 的狗巡邏任務檔跟著改 | `docker/dog/task_manager_spec.patch`（mission_loader、external_task、node、test_tree、任務檔、說明） | ✅ 11/13 端到端（閘道 → CBBA → 狗 BT：go_to 當站的位置、兩次 inspect、舊名 ptz_inspect 回 BAD_SPEC）；無人機（uav_adapter，air_recon → inspect）之後照做 |
| 4f 異質成本（第一批） | 公式不變，只加可行性：硬期限（`deadline_hard` 來不及 → 不可行，插入不能擠到期限之後；執行中的 `exempt` 不檢查）、release（到達 = max(到達, release)；`AssignmentManager::dispatchTime` 等到 release − 前往時間才發 current_task）、任務的 `min_battery_reserve`（存量取較大的）、`cbba_uav.yaml` 的 `max_bundle: 2`（comm／loss 沒讀，仍 5）。第二批（無人機續航比例、平台參數讀註冊表）放 M1 | `scoring`、`cbba_agent`、`assignment_manager`、`cbba_uav.yaml`、`test_scoring`（+4）、`test_agent`（+1）、`test_gateway`（+1 Release） | ✅ 11/14 單元＋手動（閘道＋狗：硬期限來不及不出價、release 前不發，注入後約 15 s 才發） |
| 介面 v2.0 | `Task` 加 priority、platform、required_capabilities、group_id、role_index／count、optional、floor、deadline_hard、release、expires（舊型填對應值）；`TaskResult` 加 failure_code（`failure_codes.hpp`：有認得的碼 → reason 照合約的表，BT 的 reason 不看；空的照舊）、failed_capability（E_HW → `failed_caps`，到重開）、findings_uri（只記 log）；`task_orphaned` 改型別 `TaskOrphaned`（原因 NO_WINNER、NO_CAPABLE〔`orphan_no_capable_sec` 3 s〕、EXPIRED、PRECONDITION、OPTIONAL_*）；swarm_interfaces 2.0.0 | `swarm_interfaces`、`failure_codes.hpp/.cpp`、`capabilities`（`capabilityNames`）、`cbba_node.cpp`、`test_failure_codes`（4 項）、`swarm_cbba/test/v2_iface.sh`（V1～V6：閘道＋狗，假 BT） | ✅ 11/14 單元＋`v2_iface.sh`＋`g2_e2e.sh`（EXPIRED 改在收到 reason 3 的 TASK_CLOSE 時發；沒有能做的載具時不重複報 NO_WINNER） |
| 註冊表 | `<repo>/registry/` 的 `tools.yaml`（13 個能力＋bit）、`platforms.yaml`（uav 1、quadruped 2＋成本模型，M0 只檢查）、`fleet.yaml`（uav1～3、狗 50～52、閘道 200），schema 1.0（`registry/README.md`）；`swarm_cbba/registry.hpp`（yaml-cpp）載入＋檢查（bit 和 `capabilities.hpp` 一致等）；cbba_node 參數 `registry_dir`（或 `CBBA_REGISTRY_DIR`）：平台、能力、閘道機號從註冊表，有錯拒絕啟動 | `registry/`、`swarm_cbba/registry.hpp/.cpp`、`cbba_node.cpp`、`test_registry`（5） | ✅ 11/14 單元＋節點載入 R1～R7＋回歸 |
| 5 DETECTION | 機器人的偵測（`detection` 話題，2-1b 第七節的欄位＋位置、attrs）→ DETECTION 封包（種類 10，轉送）→ 閘道確認（DETECTION_ACK，種類 11）、發 `detections`；沒確認每 1 s 重送；閘道斷線（`gateway_timeout` 3 s）時暫存（64 筆、同 track／2 m 內合併、限速 5 則/s），恢復後依序送出、標「暫存過」；閘道在線也看多跳（`gateways` [200]）。無人機 BT 的火警照舊 `new_task`（M3 前） | `detection.hpp/.cpp`、`wire_detection`、`cbba_comm`、`cbba_node.cpp`、`Detection.msg`、`test_wire_detection`（5）、`test_detection`（11）、`swarm_cbba/test/det_iface.sh` | ✅ 11/14 單元＋`det_iface.sh`（D1～D4）＋回歸（`v2_iface.sh`、`g2_e2e.sh`） |

第 4 步的手動測試都用「閘道＋狗的 cbba_node」（`participate:=always`、domain 77、獨立的 `CBBA_LOCK_DIR`），需要時加狗的 BT
（`dog_bt.sh start`）＋ sim 的 `fake_nav_body.py`。itest 還不會建新型任務（閘道注入、TaskSpec、角色組）。
節點的 log 除了終端機，也寫在容器的 `~/.ros/log/cbba_node_<pid>_*.log`（使用者 Ctrl+C 之前可以從這裡查）。
CBBA 的 UDP 不受 `ROS_DOMAIN_ID` 隔離：只重開一個節點時，任務會從還在跑的節點補回來（設計如此）。

**M0（11/20）還剩下的（依順序）**：
1. ~~介面 v2.0~~（11/14 寫好，見上表；`teammates` 是 CB-4）。改訊息 → 全隊（含狗的 BT）重編，BT 不讀新欄位就不用改程式
2. ~~5 DETECTION~~（11/14 寫好，見上表；欄位照 2-1b 第七節＋位置、attrs，EVT-1 只加欄位）
3. ~~註冊表~~（11/14 寫好，見上表；`registry_dir` 預設不用，要在 `cbba_uav.sh`、itest 打開再定）
4. ~~給伯宇的介面變更清單~~（11/14：`README_v60.md` 第 0.1 節改成 A 必做 6 項、B 要知道 9 項、C 要對的 6 項＋驗收步驟）
5. ~~全隊重編、itest 重跑~~（11/14 全過）：`20261114_173818_comm`（`--dogs 3`，C0～C9 PASS；待命 10.5 kB/s，和 11/12 版本 4 相同，11/09 版本 3 是 8.5）、
   `20261114_174034_loss`（`--dogs 3 --levels 30,70`：30% 5 s 收斂 100%、0.75／1.10 s；70% 100%、2.46／3.06 s〔第 5 週 90%、2.31／4.27 s〕；重複認領 0）、
   `20261114_175714_uav`（U1～U6 PASS，`max_bundle: 2` 第一次在 SITL：U5 uav2 拿 2 個）、
   `20261114_180303_demo`（yard `assign_random --seed 7 --dog bt`：全部 PASS，狗 BT 套兩個 patch，F_M1、F_T2 由狗 50、51 處理；CBF 最近 1.83 m）。
   **M0 的 CBBA 部分完成**；還沒 commit：註冊表、給伯宇的清單、這段紀錄

M1 以後：4f 第二批（無人機耗電改續航比例、平台參數讀註冊表）、uav_adapter（無人機的 action 也用通用名稱：`air_recon` → `inspect`）、
CB-4 協同（precedence、sync_window、all_or_nothing、teammates）。

## 機間 UDP：現在有兩條（2026-10-13 整理）

DDS 不走 mesh（`.env` 的 `DDS_OVER_MESH=0`）：ROS 話題（robot_state、current_task、PX4…）只在機內。機間只有這兩條 UDP：

| | CBBA 協定（版本 2） | 飛行層位置廣播（避碰） |
|---|---|---|
| 誰送 | `cbba_node`（無人機、V60 都有） | 無人機的 BT（`uav_px4_bt` 的 `flight_beacon`） |
| 位址 | multicast `239.255.42.99:14600`，magic `0x4342` | 同 group，port `14601`，magic `0x4346` |
| 內容 | AGENT_STATE（0.5 s）、CBBA_STATE（200 ms＋變動時，不一致時 50 ms 快速重送）、TASK_ANNOUNCE、COMPLETION／ACK／TASK_CLOSE（格式見上節、`wire_*.hpp`） | 42 B、10 Hz：機號、session、序號、旗標（在空中、位置有效、執行任務中）、位置、速度 |
| 轉送 | TASK_ANNOUNCE、完成三種會轉送（多跳） | 不轉送（避碰只需要直接聽得到的） |

- 實測流量（每台 mesh 收＋送）：待命 3.5 kB/s（原本 DDS 漏到 mesh 時 69 kB/s）；有任務 0% 掉包 6 kB/s；30% 36 kB/s；50% 51 kB/s；
  位置廣播每台再多約 0.4 kB/s 送、1.3 kB/s 收（3 台）。
- **伯宇的 BT 沒有任何 UDP**（handoff 2026-10-12 全包搜過）：狗的 BT 只走機內 ROS（current_task、task_result、robot_state、
  Nav2、雲台 action），機間全靠 cbba_node。
- **無人機 BT 為什麼有 UDP**：CPF 避碰要鄰機的即時位置與速度（機間資料）。AGENT_STATE 0.5 s 一次、沒有速度（5 m/s 時差 2.5 m）
  不夠用；跨機走 DDS 又會回到 DDS 漏到 mesh 的問題，所以 10/09 在 BT 裡另開一條。
- **使用者 2026-10-13 提議過、決定先不做（維持在 BT）**：比照伯宇，BT 只走機內 ROS。做法：位置廣播搬到 **cbba_node**
  （無人機的 cbba_node 依 `agent_type` 送、收，狗不送），整理成機內話題 `<ns>/neighbors` 給 BT 的 CPF；cbba_node 成為機上唯一碰
  mesh 的程式，協定版本 2 與 14601 的格式不變。代價：多一次機內 ROS（幾 ms）、要定 neighbors 話題、位置與速度的來源
  （PX4 或 px4_state_bridge）。

## GCS ↔ 載具：Zenoh（2026-11-04 拔掉）

2026-10-21 依實驗規劃 PDF 做過 `swarm_zenoh`（`zenoh-bridge-ros2dds` 1.10.1 的白名單、GCS = router 的星狀、selftest、
`docker/scripts/zenoh_sim.sh`、`robot.Dockerfile` 裝 bridge），10/26 selftest PASS。**使用者 2026-11-04：用不到，全部拔掉**
（套件、腳本、Dockerfile 那段、README 說明）。要再用時從 git `ec23b01` 拿回（`git checkout ec23b01 -- ros2_ws/src/swarm_zenoh
docker/scripts/zenoh_sim.sh`，Dockerfile 那段也在那個 commit）。GCS 插任務、CBBA 監控、全體急停仍見「待決定」。

## 無人機 BT：改用伯宇的骨架（2026-10-13，完成；SITL itest uav 6／6、demo 6／6）

伯宇 2026-10-12 的 handoff 在 `docs/handoff_uav_2026-10-12/`（**這份才是要用的 BT**；`UAV_BT_GUIDE.md` v2、整個 `task_manager`
套件、測資）。骨架：`ReactiveSequence`（急停 → 哨兵 `IsTaskValidAndUnchanged` → `PatrolAllWaypoints`），外部任務（cbba_）
插在佇列頭、callback 寫黑板後立刻 tick（中斷 0.1 ms）、回報恰好一次、`HandleWaypointFailure` 看 `last_error` 前綴、五條規則（§5）。
狗和無人機只差葉節點（狗 `GoTo`／`PtzInspect`，無人機 `NavigateTo`／`AirRecon`）。

我們的映像**沒有 `nav2_msgs`**，他的 `task_manager` 照原樣編不起來，所以放進 `uav_px4_bt`（和備用 BT 並存）：

| 檔案 | 內容 | 狀態 |
|---|---|---|
| `include/uav_px4_bt/px4_platform.hpp`、`src/px4_platform.cpp` | 從 `uav_bt_node.cpp` 抽出的 PX4＋ROS 介面（`Px4Platform`：PX4 話題、setpoint、位置廣播、task_result、new_task、fire_detection、參數），兩個執行檔共用 | 寫好 |
| `src/uav_bt_node.cpp` | 備用 BT 改用 `Px4Platform`（行為不變） | 寫好 |
| `include/uav_px4_bt/skeleton/bt_nodes.hpp` | 伯宇 `bt_nodes.hpp` **原樣切出**通用部分，只拿掉 `GoTo`、`PtzInspect`、`fillPtzGoal`、Nav2／雲台 include（開頭註明來源與改動，他更新時照清單重切） | 寫好 |
| `include/uav_px4_bt/skeleton/external_task.hpp` | 伯宇 `external_task.hpp`，只改 `makeExternalWaypoint`：目標正上方、tasks = [air_recon（target、duration_sec）, report_result]；`failureReason` 加 `BATTERY_RTL`、`NOT_AIRBORNE` → STOPPED | 寫好 |
| `include/uav_px4_bt/skeleton/uav_leaves.hpp` | `NavigateTo`（CPF＋運動平滑、飛行途中看到不是目的地的新火點 → 發布地面任務、繼續飛；逾時 `NAV_FAILED(TIMEOUT)`、沒位置／掉 offboard `NAV_NO_ACK`）、`AirRecon`（懸停 duration、穩定後認 `view_radius` 6 m 內的 tag 101 → 地面任務；**週計畫：懸停監控到那個地面任務結束（`task_closed`）或 `fire_task.monitor_timeout` 120 s 才完成**；路上已發布過的同一個火點改成監控那個任務） | 寫好 |
| `trees/uav_mission.xml` | 伯宇 `mission.xml` 照 GUIDE §3 改：GoTo → NavigateTo、ptz_inspect → air_recon；沒有 DefaultBehavior | 寫好 |
| `include/uav_px4_bt/skeleton/uav_flight.hpp` | 樹外的飛行 `UavFlight`（不依賴 ROS）：起飛（備用 BT 的 TakeOff）、待命盤旋與讓路（LandOrLoiter）、**做完 `return_home_delay` 2 s 後飛回起飛點上空**、低電量返航（RTL 最多送 3 次）、降落後重新起飛 | 寫好 |
| `src/uav_task_manager_node.cpp` | 照伯宇 `task_manager_node.cpp` 的 node 層（onCurrentTask、kick、finish、releaseExternal、abortNow、waypointInHand、關機先交還）；起飛完成才 tick 樹；返航 → `BATTERY_RTL`、降落 → `NOT_AIRBORNE` 交還，返航中收到的任務直接回失敗；被接管 → `PREEMPTED_BY_MANUAL`、PX4 失效保護 → `PX4_FAILSAFE`（2026-10-17）；發 `/<ns>/bt_mode`；急停話題 `/<ns>/emergency_stop`（各台分開，急停 = 原地懸停）；robot_state 照舊由 `px4_state_bridge` 發；沒有巡檢 yaml、/start_mission、TF、markers、manual_override、Nav2 reaper | 寫好 |
| CMake（`px4_platform.cpp` 編進兩個執行檔）、`test/test_skeleton.cpp`（19 項：插隊一拍內 halt、換任務不回報舊的、重送冪等、取消、回報後的取消、路過發現、目的地火警、逾時交還、掉 offboard、急停、外部點幾何、reason、起飛、返航、降落、讓路）、`test/fake_platform.hpp`（和 test_tree 共用）、`cbba_uav.sh` 的 `EXECUTOR=task_manager`（節點 `uavN_task_manager`）、`itest.py --executor task_manager` | | 寫好；隔離編譯零警告、單元測試全過（2026-10-13） |

`UavContext` 加了 `newFire()`、`fire_tasks`／`fireTaskNear()`（火點 → 地面任務）；`UavPlatform::taskClosed()`（`Px4Platform` 訂 `<ns>/task_closed`）。
使用者 2026-10-13：**UDP 維持在 BT**（位置廣播不搬到 cbba_node）；**依照週計畫**：巡檢點看到火 → 發布地面任務 → 懸停監控到狗處理完 →
完成 → 回起飛點（10/12 的「路過發現只發布、繼續飛」仍保留在 `NavigateTo`）。

2026-10-13 SITL 發現、已修（比對 PX4 的 ulog：PX4 追得很準，是 BT 送的 setpoint 在擺）：
- **起飛、懸停晃**（高度 3.2～6.8 m、水平 1.5～2.7 m）：CPF 的排斥是 3D 方向（高度差一點就互相上下推）、待命時「正在靠近」就在 6 m 內排斥＋環流
  （漂移就觸發、互繞），safe_radius 1.8 m 和出生間隔 2 m 太接近。改成排斥只取水平、待命／起飛時只對執行中的鄰機提早反應、
  `closing_speed` 0.5 m/s、safe_radius 1.2 m、垂直剎車曲線 `brake_vertical_accel` 0.5 m/s²、平滑層參考位置不越過目標。
  修後懸停水平 4～7 cm、起飛最高 5.02 m。
- **U6 沒人接**：懸停監控中的無人機，CBBA 以為停留 5 s 就做完，把新任務排在它後面、出價贏過閒著的。`AssignmentManager` 加
  `overrun_margin`（10 s）：執行中超過預估就不排在後面（`ExecConstraints::queue_behind_active`），搶占照常。
- **Gazebo 錄影**：視窗的 VideoRecorder 按鈕要 `/gui/record_video`，gz-sim 8＋MinimalScene 沒人提供；改成場景裡的俯瞰攝影機
  （`overview_camera`＋`CameraVideoRecorder`，`gen_patrol_site.py` 產生；`/<world>/overview/record_video`），伺服器自己錄。
- **itest 起火條件**：航線模式要三台同時符合，窗口不到 1 s；改成任一台符合就起火。
- 注意：容器的主程序是 `sleep infinity`，**不要 `pkill -f "sleep infinity"`**（會把容器停掉，10/13 停過 sim 一次）；
  `px4_start` 改用 `tail -f /dev/null | px4_sitl.sh`，itest 結束時清掉。

## 目前狀態（2026-11-14）

整體進度與缺失清單：`docs/progress.md`（10/12 更新，舊）；機間通訊的整理（做簡報用）：`docs/cbba_comm_status_20261012.md`（舊）。
第 5 週的細節見「週計畫」底下的「第 5 週：弱網、100% 斷網」；下一階段見「下一階段：2-1b」。

### 現況

| 部分 | 狀態 |
|---|---|
| 機間通訊（**協定版本 4**，11/12；版本 3 是 11/04） | 版本 4（2-1b M0）：64 位元 task_id、能力過濾、P0～P3、TASK_SPEC、閘道（平台 0）、改版、過期（TASK_CLOSE reason 3）、optional 略過（reason 4）、AGENT_STATE 的「已接近」，見「2-1b M0 的 CBBA 實作進度」。失聯接手 11/13 起依優先級（P0 10 s、P1 15 s、P2／P3 30 s；舊型火警 P0、巡檢 P3，結果同原本）。以下是版本 3 起的主體：擴散、分配、完成確認、失敗交回（reason 排除）、停止參與後接手、失聯接手（火警 10 s、巡檢 30 s）、重開機、鎖定、指派版本、權限；版本 3 的補發（壓縮摘要＋缺口請求＋限速分批）。第 5 週量測（空地 6 台、12 個任務，`20261109_173015_loss`）：30% 掉包 5 s 內收斂 100%、P90 1.10 s；70% 90%；跨能力 0 |
| CBBA 搶占與出價 | 舊型：只有地面處置任務是火警（10/09 定案）；搶占時不算被搶占任務的延後、出價不受 bundle 上限 → 巡檢中、離火點近的 V60 搶得到。新型（11/13）：照任務的 `can_preempt` 遮罩（`mayPreempt`）、commit_radius 內不可被搶。出價公式不變，11/14 加可行性：硬期限、release、任務的電量下限；`cbba_uav.yaml` 的 `max_bundle: 2` |
| 閘道（GCS） | 11/13：非競標的 CBBA 協定節點（`participate: never`、機號 200、`cbba_gateway.yaml`）；`inject_task`（冪等、角色組、改版）、`assignments`、過期與 optional 角色的關閉。規劃器、範本、偵測接入還沒做 |
| cbba_node ↔ BT 介面 | 照機器狗端（伯宇）：`swarm_cbba`、swarm_interfaces v1.6、current_task／mode／reason、不握手、assign_hold 0.4 s。**11/12 起 task_id uint64、Task.msg 加 `revision`、`spec`**。我們加的話題：`task_closed`、`task_announced`（預設只有 UAV；參數 `publish_announced`）、`task_orphaned`、`cbba_path`（UInt64MultiArray）、`cbba_tx`（11/09，送出位元組累計）、`agent_conflict`（11/12）、`task_spec_missing`（11/12）；閘道才有的 `inject_task`（service）、`assignments`（11/12）。**`current_task.spec` 是 2-1b 的通用 steps**（`{"steps": [{action, location, object, params}]}`，狗和無人機同一套動作名稱，11/13） |
| 無人機 BT | 正式：`uav_task_manager_node`（伯宇骨架）；備用 `uav_bt_node`（`EXECUTOR=bt`）；避碰、運動平滑層兩者共用 |
| 無人機避碰 | 預測衝突 → 路權 → VO 繞行 → 二階 CBF 保底 → 緊急退讓（10/24 定案）；**路權 11/07 最終規格**：只看移動意圖與機號（一台動一台停住 → 動的讓；都在動 → 機號大的讓；都停住 → 只靠 CBF），見「無人機避碰」一節 |
| `px4_state_bridge` | 發 robot_state；mode 照正式 BT 的 `bt_mode`；備用 BT 時自己判斷 |
| AprilTag 火警 | 到達後 1.2～1.9 s 判定 tag 101、誤差 < 0.5 m、誘餌不觸發；巡檢點火警 → 懸停監控 → 狗處理完 → 完成；路過、待命時發現也會處理 |
| 狗（模擬） | `--dog bt`：伯宇的 task_manager（原樣編譯，`dog_ws`）＋方塊身體 `fake_nav_body.py`（假 Nav2、假雲台），接我們的 cbba_node（協定版本 4；介接層套 `docker/dog/` 的兩個 patch：uint64、讀 spec＋`ptz_inspect` 改名 `inspect`）；`--dog fake`：`fake_dog.py`。11/09 起每隻在自己的容器 `cbba_dogK`（線材共用 `sim_dog_net`）。日常巡邏 A1～／B1～／C1～（到點 `inspect` 看最近的設備，loop；11/13 前叫 `ptz_inspect`）；模擬偵測（`demo.dog_detect` 半徑 10 m、持續 1 s → 用狗的身分發布地面任務） |
| 模擬環境 | docker：sim、uav1～3、**dog1～3**（`DOG_COUNT`）、gcs；DDS 只在機內；sim 的線材網卡 ≤ 4 張（Fast DDS／XRCE agent 的位址上限）。場景 `yard`（主要）、`demo_site`、`patrol_site`、`fire_site`；任務標記 `models/task_marker_g／m／t／f／p`（G 綠、M 藍、T 黃、F 紅、狗巡邏點紫），做完刪掉（`patrol_scenario.sh remove`）；設備頂上起火時設備變紅（`fire-on`） |
| 自動測試 | `docker/itest`：comm（C0～C9，`--dogs`）、uav（U1～U6）、loss（掉包梯度、`--dogs`、每台 TX／重傳 TX，`plot_loss.py`、`plot_tasks.py`）、demo（D1～D7；情境 `assign_random`、`manual_fire`、**`blackout`**〔B1～B4、`plot_blackout.py`〕）；`swarm_cbba/test/g2_e2e.sh`（61 項）；U1 失敗時寫 `dds_diag.txt` |
| 影片 | `demo.mp4`（2D 動畫，中途不停頓、做完的旗子變淺灰、狗的偵測範圍圓）、`gazebo.mp4`／`gazebo_labeled.mp4`（標籤、偵測範圍、做完的標籤拿掉）、`demo_cameras.mp4`（下視相機） |
| GCS ↔ 載具 | Zenoh 11/04 拔掉；插任務、監控改由閘道的 cbba_node（`inject_task`、`assignments`，11/12～13）；全體急停還沒做 |
| 真的 V60 | **還沒在真的狗上測**；伯宇的 BT 在我們的模擬裡已經接協定版本 4（兩個 patch），他自己的部署要換成 main 並照 `README_v60.md` 第 0.1 節套 patch；他 11/11 的新版（Switch9：Track、Wait、Speak…）還沒給，patch 可能要手動合 |

最近的結果：
- 2026-11-09 `out/20261109_173015_loss`（空地 6 台、有 TX）：見上面的「第 5 週」；簡報第 7 頁用 `loss_sweep.png`。
- 2026-11-09 `out/20261109_124315_comm`：C9 100% 斷網（6 台含狗）PASS。
- 2026-11-09 `out/20261109_170653_demo`（`--scenario blackout --dog bt`）：U1、D1～D3、D5、D7、B1～B3 PASS；B1、B3 是修正前的量法，正確數字見「第 5 週」；`blackout_timeline.png`。
- 2026-11-08 `out/20261108_162805_demo`：dog1 模擬偵測發現 F2；影片已補畫偵測範圍（舊版留成 `*_before_detect.mp4`）。
- 2026-10-24 `out/20261024_235929_demo`（`assign_random --seed 7`，3 m/s）：全過，X1 VO 繞過懸停的 uav2、最近 2.25 m。單元測試（10/24）：test_avoidance 15、test_cbf 16、test_flight_beacon 5、test_motion 8、test_skeleton 41、test_tree 13 全過、零警告。

**commit**：分支 `hetero-tasking`（11/14）：`41e52cf`（註冊表、清單、itest 結果）、`a4c716b`（DETECTION）、`867ea76`（介面 v2.0）；之前 `70ad844`（4f 第一批）。M0 依序：`b9d3649`（第 1、2a、2b 步）、`9e4eea9`（2c）、`2553ca8`（3 TaskSpec）、
`a6dd8a8`（4a 閘道）、`dd9336e`（狗 BT 讀 spec）、`4b20ad5`（4b 改版＋README 版本 4）、`afe3ea3`（4c 過期）、`f754ac8`（4d）、
`c1f7c17`（通用 steps、`inspect` 改名）、`e52f159`（4e＋inspect 拍 1 張）、`70ad844`（4f 第一批）；之前 `2edd074`（第 5 週收尾＋M0 合約）。
**還沒 push**：分支 `hetero-tasking`（`a6dd8a8` 之後的 commit 都在這個分支；GitHub 的 main 停在 `2553ca8`）。push：`git push -u origin hetero-tasking`（要 GitHub 的 Personal Access Token）。
**還沒 commit**：無。
不要進 repo：`graphify-out/`、`docs/reports/82fd836a-…實驗規劃.pdf`（使用者的檔案，要不要收先問）、`swarm_zenoh` 的 `__pycache__`、`ros2_ws/src/world/`（伯宇的原檔）、
隊友的資料（`docs/handoff_uav_*`、`docs/messageImage_*.jpg`、`swarm_cbba/testdata_for_uav_2026-10-12/`、宜臻的實作說明 .md，`.gitignore`）。
共用的 `ros2_ws/install` 改 C++ 後要在 uav 容器重新編譯 6 個套件；改了 `docker/.env`、`gen_compose.py`、Fast DDS 範本後要 `./stop.sh && ./start.sh`。

### 還沒做的（依建議順序）

1. **2-1b 的 M0（11/20）**：照「2-1b M0 的 CBBA 實作進度」表下的「M0 還剩下的」：介面 v2.0 → DETECTION → 註冊表 → 給伯宇的清單 → 全隊重編、itest。
   使用者的順序：先做 CBBA（CB 全部），再做無人機（uav_adapter，SITL 後端用 `Px4Platform`）。
2. 第 5 週收尾（選做）：
   - 重跑 `demo --scenario blackout`，讓報告的 B1～B3 是修正後的量法；斷網 8 s vs 20 s 的對照；C9 重複 5～10 次。
   - 追 `20261109_173015_loss` 70% 第 5 次的重複認領（哪個任務、哪兩台）。
   - 讓 loss 記每台實際分到的任務種類（才能說「每隻狗實際接到一火警＋一巡檢」）。
3. 避碰選做（使用者：合理就好）：回起飛點最後一段限速；約 6 m/s 對飛時緊急退讓的抽動；CBBA 讓同一台順路接多個任務（使用者還沒決定）。
4. 真的 V60：等伯宇的部署換成協定版本 4（main＋兩個 patch）；巡檢中被火警搶占在真狗上跑。
5. session 層、認證；新版 CBBA 的其他 constraints；多跳、實機 mesh、Jetson、chrony、實體相機、能量模型校正、itest ssh 版；共用頻寬的模擬（現在各自、無上限）。
6. 對外：`protocol_v2_notice.md` 改成版本 3 的現況再寄規格作者；給伯宇的說明；架構圖。

### 待決定

| 項目 | 狀況 |
|---|---|
| 300 ms 全流程 | 目前指派約 0.4 s（assign_hold）；建議只在有競爭、未收斂時才等；預算要和伯宇、宜臻分 |
| BT 的 UDP | 2026-10-13 決定先維持在 BT（`flight_beacon`） |
| 平板／GCS 的角色 | 要不要跑 cbba_node、能否建任務；部分保留可用容量 Q（討論過、沒實作） |
| 急停話題 | 狗 `/emergency_stop`（全域）、無人機 `/<ns>/emergency_stop`；PDF 要「全體急停」＋單機返航，要和伯宇統一 |
| GCS 的 CBBA 監控 | 閘道的 cbba_node 發 `assignments`（11/12 做了）；要不要做監控畫面還沒定 |
| 狗 BT 收到同 task_id、新 revision | CBBA 會重發（4b），但狗的 `onCurrentTask` 當成重送、照舊版做完。A：我出 patch（先取消再插入）；B：等伯宇新版。使用者還沒決定 |
| 電池權重測試的設定 | 續航、插入時段、耗電誤差要多緊（`docs/sim_scenario_plan.md` 第七節） |
| 失聯的那台「不接新的」 | 各機分不出自己是不是失聯的那台，沒有另外實作 |
| 斷網後舊持有者的完成被拒收 | 斷網 > 10 s 時火警被接手、原持有者其實已做完卻因版本較舊被拒收，任務被重做（`20261109_170653_demo`）；目前是設計（火警寧可重複），要不要改先和使用者確認 |
| 2-1b：無人機的 CBBA | 機上直接跑 cbba_node（現在）還是地面站代理（文件待確認問題） |
| 2-1b：無人機控制與場地 | 實機 `/MAV1/api/*`（Node-RED）vs SITL 的 PX4 offboard；室內 3 m、起降板 vs 模擬的戶外 8 m |

### 歷史紀錄（濃縮）

- 2026-10-07：協定版本 1、task_executor 的第一階段 SITL；SITL 電量對齊（PX4 續航 900 s、最低 20%）。
- 2026-10-09：通用 cbba_node、協定版本 2、正式指派版本、TASK_CLOSE 權限、程序鎖、BT 接受／搶占／排隊、鎖定；備用 BT、AprilTag 偵測、`fire_site`。
- 2026-10-10：`patrol_site`、`patrol_scenario.sh`、`mission_sim`＋`cbba_sim insert`。
- 2026-10-11：SITL 實測協定版本 2＋備用 BT；修 A1、`px4_sitl.sh` 自動加入場景；`docker/itest`；失聯接手規則定案；收到伯宇的 handoff（10-11 版）。
- 2026-10-12：照伯宇的介面改（swarm_cbba、v1.6）；`status_timeout` 2 → 5 s；log 對齊伯宇；`g2_e2e.sh`；DDS 只在機內；失聯接手實作；
  itest `loss`、`demo`；CBBA 搶占評分修正；`px4_state_bridge` 起飛完成才 AUTO；備用 BT 運動改寫（CPF 剎車曲線、懸停不互推、
  運動平滑層、位置＋速度前饋）、火警看位置去重、路過發現；`demo_site`、Gazebo 視窗設定、起飛／CBBA 閘門。
  **使用者定的流程**：多個固定巡檢點 → CBBA 分給 UAV → 飛行途中相機持續偵測 → 路徑經過火點看到 tag 101 → 確認火警 →
  發布地面處置任務 → V60 競標 → 得標的中斷巡檢前往（路過發現只發布地面任務、繼續飛；`fire_task.divert_on_sight` 預設 false）。
  試過又退回：「價值 ≥ 100 的空中任務算火警」、無人機發現後改去懸停。
- 2026-10-13：收到伯宇 handoff 2026-10-12（BT 骨架），整合完成（`uav_task_manager_node`）；整理機間 UDP（維持在 BT）；
  週計畫流程（巡檢點火警 → 懸停監控 → 狗處理完 → 回起飛點）、`task_closed` 話題、假狗、俯瞰攝影機錄影；修起飛／懸停晃、U6 接手。
- 2026-10-14：無人機避碰 **CPF → CBF**（`uav_px4_bt/src/cbf.cpp`，參數 `cbf.*`，公式在 `uav_px4_bt/doc/parameters.md` 第五節；CPF 備份在 `docs/archive/cpf_20261014/`）；`fly_timeout_abort`（逾時只警告、繼續飛）；Nav2 導入規劃 `docs/nav2_integration_plan.md`；**路過發現改成轉向**（使用者：週計畫與 BT 規定看到火點要到上空懸停等待）：`NavigateTo` 發布地面任務 → 飛到火點上空懸停監控到 task_closed 或 monitor_timeout → 回航線（`divert="false"` 恢復只發布、繼續飛）；`AirRecon` 看到火也移到火點上空；`demo_site` 的點重擺（G1、G2 同一條線看 CBF，R1 在 G3 航線旁、R2 在 G2 旁 2.5 m）。本文件其他段落的「CPF」指 10/13 以前的做法。
- 2026-10-15：commit 到 `f161680`（10/12 之後的全部工作分三個 commit）。
- 2026-10-17：宜臻的「CBBA 任務失敗排除與人工接管：實作說明」（需求 1：TRANSIENT 失敗後用成功率 p = 1/(1+f) 降低出價；
  需求 2：人工接管不出價、平板指定 CBBA 任務時鎖定、latch、無人機的 MANUAL 來源）。**需求 1 已實作**（必做項目＋`max_failures`；
  `scoring.cpp`、`CbbaAgent::countFailure`、`CbbaComm::countFailure`、`cbba_node` 的 `preempt_exclude_sec`／`max_failures`、
  `formula_design.md` 第 5 行；test_scoring 4 項、test_agent 6 項、`g2_e2e.sh` E1 加 1 項）。和說明不同：期望重試時間不加在
  有 BT 剩餘時間的執行中任務；PREEMPTED 的排除時間照說明 5 s（狗、無人機都是；使用者一度改回 60 s，釐清「接管期間靠 MANUAL」後改回 5 s）。
  孤兒任務警告也做了（`cbba_node` 的 `checkOrphans`、參數 `orphan_warn_sec` 30 s、話題 `task_orphaned`；`g2_e2e.sh` 加 2 項 → 61 項）。
  還沒做：平板指定 CBBA 任務時的鎖定、兩隻狗 TRANSIENT 後由另一隻得標的整合測試。無人機 mode 的討論：狗是 BT 自己發 robot_state（2 Hz，
  `manual_active`／`emergency`），接管時先回報 `PREEMPTED_BY_MANUAL` 再改 mode，解除接管直接放行（不 latch）；
  伯宇的 GUIDE 規定無人機的 robot_state 由 `px4_state_bridge` 發。使用者傾向 **B**（BT 決定 mode、透過話題給 bridge，
  bridge 照舊發 robot_state；備用 BT 照舊由 bridge 判斷、加 offboard 條件），正在和伯宇、宜臻討論；
  先在 `UavFlight` 加警告（只印 log、`platform.log` 是 INFO 等級、字首「警告：」）：爬升 20 s（`kClimbWarnSec`）沒到
  altitude ± `takeoff_tolerance`、在空中被切出 offboard、切回 offboard 時高度差超過 `takeoff_tolerance`。
- 2026-10-17（續）：**無人機的接管照狗的方法做了**（使用者選 B）：`UavFlight` 加 `OVERRIDE`（起飛完成後 offboard 掉了、不是自己返航）、
  事件 TAKEOVER／FAILSAFE／RESUMED；`Px4Platform` 多傳 `failsafe`、`user_took_over`；node 層交還（`PREEMPTED_BY_MANUAL`／`PX4_FAILSAFE`）、
  發 `/<ns>/bt_mode`；bridge 參數 `bt_mode_topic`、`bt_mode_timeout`；`preempt_exclude_sec` 狗、無人機都 5 s（使用者討論後決定）；切回 offboard 時
  待命點改成目前位置、運動平滑層重來。只改無人機機內，狗、cbba_node、訊息、協定都不變。備用 BT 的 `FIRE_FOUND`（PREEMPTED）只在
  `fire_task.divert_on_sight = true`（預設關）時才會發生。
- 2026-10-17（續 2）：SITL 驗證接管整串（被接管 → PREEMPTED_BY_MANUAL、5 s → 切回重新得標完成；懸停補爬到 5.0 m；接管 40 s 時孤兒任務
  警告、切回解除）。修兩個顯示問題：node 層 `abortNow()` 清掉 kick 留下的中斷延遲起點（閒置開跑的任務被中止時印出 6350 ms 這種假延遲，
  狗端骨架也有同樣問題）；骨架副本的 `printReport` 沒列出的結束原因改印「中止（原因）」，MISSION_TIMEOUT 才是「逾時中止」。
- 2026-10-19：Gazebo 視角說明 `docs/gazebo_view_guide.md`；demo_site 錄影鏡頭拉高到 35 m、場景加地面網格（`gen_patrol_site.py` 的 `GRID`，
  錄影看得到；視窗的 GridConfig 錄不到）。`cbba_overrun` 的 SITL：一般懸停誤差平均 0.1 m、到達時最多衝過 0.46 m；uav1 在 G1 看起來偏 2 m
  是移到火點上空監控（設計如此）。**修待命讓路的範圍**：原本執行中的鄰機在 `cbf.influence_radius`（demo 12 m）內就停下，uav2 回起飛點
  途中停了 12 s；改成 2 × `cbf.yield_radius`（4 m），`test_skeleton` 加 1 項。新增手動測試 `docs/failure_takeover_tests.md`
  （地面端 A1～A7：gcs 兩隻假狗、`docker/scripts/fake_robot_state.py`；無人機端 B1～B5），還沒跑。
  **待命時也看火**（`UavFlight::loiter` 的 `idleFire`，參數 `fire_task.idle_watch_tag` 101、0 = 不看）：`f R1` 的 SITL 裡火生成得太晚，
  飛 G3 的那台出發時沒看到，回程飛回起飛點時清楚看到（16/16）卻不理（原本只有 NavigateTo 看火）→ 待命看到新火點也發布地面任務、
  到火點上空監控、結束後回到待命；監控中被派任務時 `leaveHold` 結束監控。`test_skeleton` 加 3 項（37 項）。
  動畫 `animate_demo.py` 改全程 1 倍速（原本沒事件時 3 倍速）、電量 0～100%；錄影鏡頭 150 m＋視角 0.389 rad（35 m 時透視視差讓無人機
  看起來偏 1 m）；itest D5 等停好（0.3 m 內，最多 5 s）再記位置。
  itest D4：先發現火的是別台（例如待命時看到）時，改成檢查發現的那台監控到狗處理完。itest demo 加 **`demo_cameras.mp4`**：
  sim 容器的 `docker/itest/record_cameras.py` 錄 `/uavN/camera/image_raw`（每秒 5 張 JPEG，牆上時間當檔名），結束後
  `compose_cameras.py` 照 gazebo.mp4 的開始、結束時間拼成三台直排，主機 ffmpeg 和 gazebo.mp4 左右並排（`--no-record` 時不錄）。
  懸停：`docker/itest/hover_error.py` 量出**沒有衝過頭**（0.00～0.01 m）、穩定後 0.07～0.10 m，問題是 `reach_tolerance` 0.5 m 就算到達，
  懸停的前 1.2～1.5 s 還在靠近 → `reach_tolerance` 0.25；`brake_accel` 試過 0.15 沒差、改回 0.2；`hold_radius` 0.2（要 < reach_tolerance）。
- 2026-10-20：`AirRecon` 為了盯火離開巡檢點（火點離巡檢點 > reach_tolerance）時，監控結束（狗處理完或 monitor_timeout）後
  **先飛回巡檢點補完剩下的停留時間再回報完成**（`cbba_overrun` 裡 uav1 在 G1 只停 1 s 就去盯 R4，狗處理完直接回報 G1 完成）；
  `test_skeleton` 加 `FireBesidePatrolPointReturnsToFinishHover`（38 項）。`demo_cameras.mp4` 在 `cbba_overrun` 合成成功。
  公式 PDF：`docs/cbba_cbf_formulas.html` → `docs/reports/cbba_cbf_formulas_20261019.pdf`（Chrome headless 轉）。
  `animate_demo.py`：任務名稱改成先認場景的點（G1 旁 2 m 起火時原本叫「F_R4 fire」，看不出 G1 完成）、火點任務要在 0.5 m 內；
  發現火的事件依資料寫「at <點>／en route to <點>／while idle」（原本照模式猜，懸停時看到也寫 en route）。
  itest demo 的假狗改成 **3 隻**（`config.json` 的 `dog.units`：v60／50 (−4, 0)、v60b／51 (10, 6)、v60c／52 (−18, −8)），
  ros_bridge 記每隻的位置（`dog` 帶機號）和 `current_task`（`dog_assigned`），D3 列出哪隻狗處理；2D 動畫無人機用四旋翼圖示＋`uavN`、
  狗用四足圖示＋`dogN`，事件加「dogN <- ground task」；`demo_cameras.mp4` 縮成總寬 1280（Gazebo 1024 × 576＋相機 256 × 576）。
- 2026-10-20（續）：**人工指派**（模擬地面站／平板）：`insert.py` 的 `m uav2 x y d=秒`／`m dog51 R3`／`m uav2 clear`，itest 在 sim 呼叫
  `/<ns>/manual_override`（`ManualOverride.srv`）。無人機 BT（`uav_task_manager_node::onManualOverride`，照伯宇的同名函式）、
  `fake_dog.py` 都實作：交還手上的 CBBA 任務（PREEMPTED_BY_MANUAL）→ mode MANUAL、忽略 current_task → 去指定點停留 → 做完自動
  回到 AUTO（不 latch）；`bt_mode` 在 `manual_active` 時是 MANUAL。2D 動畫畫指定點（X）和事件。yaml：`cbf.brake_accel` 0.2 → 0.5、
  `return_home_delay` 2 → 1 s（uav3 的停頓感：最後 6 m 約 8 s、做完多停 2 s；另外 21、41 s 的減速是 cbba_overrun 刻意的 CBF 交會）。
  `fake_dog.py` 節點名稱改成 `<ns>_fake_dog`（3 隻同名會衝突）。
- 2026-10-20（續 2）：**出價表**：`CbbaAgent::bidFor`（這台現在對任務的出價＋沒出價的原因，只給 log 用、不影響分配；
  test_agent 加 1 項）；cbba_node 在任務第一次知道、得標者改變、這台的出價狀態改變時印「出價表 task=… agent=… bid=… why=…」；
  `docker/itest/bid_table.py` 收集無人機、狗的 log → `bids.md`、`bids.svg`（長條圖：橫軸任務、縱軸每一台的出價）、`bids.csv`（itest demo 結束時自動做）。2D 動畫：任務點改成旗子、
  航跡每 4 m 一個方向箭頭、無人機與狗的圖示縮小。
- 2026-10-21：itest demo `manual_fire` SITL 跑通（全過，M1～M3 照預期）。依實驗規劃 PDF 加 `swarm_zenoh`（GCS ↔ 載具白名單，
  `zenoh-bridge-ros2dds` 1.10.1、`robot.Dockerfile`、`zenoh_sim.sh`、selftest；見「GCS ↔ 載具：Zenoh」）。
- 2026-10-23：分析實驗規劃 PDF：Zenoh 只用在 GCS ↔ 載具（DDS 風暴已用自訂 UDP 解決）；PDF 的「18 條規則」是誤植，實作 17 條。
  itest demo 的 `--speed` 預設 3 → 0.5 m/s（使用者：3 太快），拿掉 `influence_radius` 12 m 的覆蓋，時限放寬。
  CBF 停頓的分析（10/21 影片的軌跡）：約束只看徑向靠近速度、往旁邊繞要等違反才開始、對懸停鄰機只負責一半 →
  路線本來會錯開 2.36 m 也從 2.8 減到 0.6 m/s。定案見「無人機避碰」一節。
- 2026-10-24：**yard 場景**（`docker/gz/worlds/yard.sdf`，`gen_patrol_site.py yard`；伯宇的障礙物在 `yard_env.sdf.inc`，拿掉固定的
  tag 101、102，物理 ode 4 ms；`yard_points.csv` G1～G3、R1～R5）；出生原點 `UAV_SPAWN_ORIGIN`（`cbba_uav.sh` 用 `spawn_enu` 給各節點、
  `px4_sitl.sh` 的 `PX4_GZ_MODEL_POSE`；yard 要 `-40,-30`，沒設對拒絕啟動）、出生間隔 yard 5 m（3 m/s 剎停約 3.9 m，2 m 時回程互相讓不開、
  相撞過一次）；itest `--world yard`（`config.json` 的 `worlds.yard`：原點、間隔、高度 8 m、模擬電池 1800 s、時限 900 s、`keep_out`、
  假狗出生點）；`cbba_uav.sh` 預設 0.5 m/s（itest uav 照舊 5）、速度高度一律寫成小數（`CBBA_ALTITUDE=8` 整數時 BT 拒絕啟動）；
  ros_bridge 換算位置加出生原點；itest 開頭檢查容器；假狗改名 `v60`、`v60_1`、`v60_2`（照伯宇）。
  **無人機避碰定案**（見「無人機避碰」一節；過程：一階 CBF 一直開著時錯開也減速 → 第一版提前繞行在 3 m/s 起降板相撞 → 使用者定
  「預測衝突＋最小路權＋VO＋CBF 保底」→ 二階 CBF）。
  **itest 情境**：`assign_random`（G、M、T、兩個火點）；情境檔加 `count`、`at_range`、tasks 的 `"fire": true`（任務點同時起火）、
  `"behind": "manual"`（放在人工指派懸停那台的後方，`behind_spot`）、manuals 的 `pair`（對調，留著但情境不用：挑到盯火的、兩台搶同一點）；
  人工指派拆成 `manual_prepare`／`manual_send`／`manual_record`、編號 M1、M2…（terminal、動畫、Gazebo 標記不寫座標）；
  `cbf_speed_args`（3 m/s：miss 2.1、exit 2.6）；`--speed` 預設 3 m/s。2D 動畫：圖示照地圖比例（`ICON_M` 1.6 m）、地圖加大、
  箭頭間隔跟著放大、狗叫 dog1～dog3（依機號排序）。使用者 10/24：避碰過程不在動畫上標註。
  CBBA 看距離，兩台在飛行中交叉的分配幾乎不會出現（交叉時對調一定更近）→ 用「航線經過不會讓開的懸停無人機」製造交會。
- 2026-11-03～04：協定版本 3 的弱網量測（`docker/itest/out/20261103_163735_loss`，3 台、每次 9 個任務、每個掉包率 20 次）：
  0～70% 每次都收斂（5 s 內 100%）、重複 0、任務覆蓋 100%；每台平均 3 個，掉包越高越不平均（70% 時 1～5 個，13／20 次不是 3/3/3）——
  建立者先搶＋出價看邊際成本（順路的任務幾乎不多花）＋ assign_hold 0.4 s 後鎖定，**不是 bug**；報告的「完全正確」改稱「和最近分配一致」。
  完成 → 全隊確認 70% 時 11 s ≈ `completion_retry` × `completion_max`。mesh 13.7 → 4.3 kB/s（0 → 70%）。
  週計畫、實驗規劃都沒有「每台最少分到幾個」的標準（實驗規劃的是任務完成覆蓋率 100%）；實驗規劃的階段 1 不是使用者負責。
  加：`plot_tasks.py`（每台分到幾個的長條圖）、`plot_loss.py` 第四張圖（完成 → 全隊確認）與依 `node_params` 分線、
  itest `--node-param`、comm 的 **C9 100% 斷網**、cbba_node 參數 `completion_retry`、`completion_max`。
- 2026-11-05：**下週報告的影片**（使用者：Gazebo 視角照舊、後製加標籤、2D 動畫不改）：
  隨機挑點考慮設備（`config.json` 的 `worlds.yard.demo.equipment`／`placement`：從 `yard_env.sdf.inc` 讀設備；
  所有任務種類（M、T、火）一樣：40% 設備旁、30% 設備頂上（標記、火點生成在頂面）、30% 空地；不放在設備本體裡、鋼構棚頂底下；`itest.py` 的
  `placement_spot`、`blocked`；挑到的類別印成「TB3 旁」「tank_1 頂上」「空地」）；外觀：G 綠／M 藍／T 黃的貼地圓盤
  （`models/task_marker_g／m／t`，直徑 3 m、沒有碰撞）、火點 `models/task_marker_f`（同一種單層紅色圓盤，tag 101 在 0.032 m；
  設備頂上時生成在頂面；任務點同時起火時只放火的圓盤。使用者：不要紅色疊黃色、簡單一個平面）、yard 不放 R 灰色方塊、網格留著（使用者）；`patrol_scenario.sh mark-at … [M|T|G]`、`fire-at … [高度]`；
  `label_gazebo.py`（gazebo.mp4 → gazebo_labeled.mp4，ros_bridge 的 `clock` 事件對齊模擬時間、`demo_plan.json` 的
  `record`）。狗照舊直線穿過設備（使用者：測試用）。伯宇的狗 BT 分析：任務座標當成要看的目標，停在目標前 1.5 m
  （只看直線方向）、雲台高度固定 1.0 m；真 Nav2 去不了時 204／208 → PERMANENT。
  **同一點懸停衝突**（`20261105_025031_demo`）：T2 和火同點，uav2 去 M1 路上先看到、停在火點上空監控，CBBA 又把 T2 派給 uav3 →
  兩台搶同一點、避碰來回晃，最近 0.45 m。使用者：先記著、模擬避免（任務點同時起火時避開別台的航線與位置，`uav_routes`），
  BT 的修法（目的地被懸停鄰機佔住 → 在旁邊 2.5 m 外執行）之後再定。
  **狗也看得到火（模擬）**：`demo.dog_detect`（半徑 10 m、持續 1 s）→ itest 的 `dog_watch` 用狗的身分發布地面任務、寫一筆
  `new_task` 事件（uav = "dogN"），之後照常由 CBBA 在狗之間競標；誰先看到誰報，無人機看 task_announced 不重複發布。
  原因：火只有無人機偵測時，就算狗比較近也要等無人機看到（F2：狗 16 m、等了 22 s）——不是 CBBA 寫死，是只有無人機有偵測。
  同日再改（使用者）：**火和任務點（G、M、T）至少 `demo.fire_task_gap`（預設 view_keep 9.6 m）**——任務點離火 6 m（AirRecon
  的 view_radius）內時，做任務的那台會移到火點上空，和正在監控的那台搶同一點、繞圈；T2 不再同一點起火（改成另一個隨機火）；
  M（位置、火）完全照原本的做法，不檢查 fire_task_gap；這個距離只用在隨機插入的任務 T ↔ 火。`gazebo_labeled.mp4`：載具一律黑圈＋白底黑字（顏色只留給任務），事件字幕移到左上
  （下方會被播放器的進度列擋住），連線用任務的顏色。`dog_watch` 只在 itest 裡，不影響狗端。
  **設備頂上起火 → 那個設備變紅**（使用者；照伯宇 README_WORLD.md「TC6_fire 紅色＝起火的那塊、tag 101 在旁邊地上」的表示法）：
  `patrol_scenario.sh fire-on`（設備外蓋大 2 cm 的紅色外殼、tag 101 貼頂面），只有 placement 的 top 類；設備旁、空地的火照舊紅色圓盤。
  `TC6_fire` 改成灰藍色（`yard_env.sdf.inc` 開頭註明；原檔紅色），平常場景裡沒有紅色設備。T3～T5 加進 `assign_random`；
  影片拿掉載具到任務的連線。
  **D7 0.41 m（`20261105_144909_demo`，回起飛點）**：uav3 已停在起飛點上空待命，uav2 回 (−40, −25) 的航線穿過它（起飛點南北一列）。
  「任務中懸停」優先只在 busy 時成立，待命懸停沒有 → 比機號，uav2 直行、uav3 要讓；停著的那台只在對方航線 < 1.5 m 而且 5 s 內
  才讓，剩 3.8 m 才反應，3 m/s 剎停要約 3.9 m → 兩台緊急退讓。使用者：不要一直加狀態、不要改進場路線這種繞道。**11/06 查出真正原因是預測的 bug**（路權照 10/24：執行中 ＞ 機號小，uav3 該讓
  也判對了）：預測窗口 T = min(ttc_horizon, 到自己目標的時間)，uav3 停在起飛點上空還在修正幾公分 → t_goal 約 1 s，只看 1 s
  （log「預測 1.0 s 後…現在 3.79 m」）。改成 T 一律 ttc_horizon、自己到目標後停住、鄰機照樣飛，兩段一起算（`predictCpaStop`）；
  停住之後鄰機飛過來的要求 miss_clearance。（11/07 使用者定最終規格：路權只看移動意圖與機號（不看 busy、任務類型），停住＝已到目標或 BT 沒在控制，不看瞬間速度；兩台都停住只靠 CBF——見「無人機避碰」一節。起因：20261107_032737_demo 回起飛點時停著的 UAV2、UAV3 被機號 1 的 UAV1 推開 4 m；用速度判斷時 DepartureTowardPassingNeighbor 沒到目標）。
  `test_avoidance` 加 NeighborArrivesAfterSelfStops、HomingPassesNeighborParkedAtHome（uav3 目標每秒左右 0.12 m 切換，模擬修正）。
  單元測試第一次跑：緊急退讓觸發在違反量 0.0018 m/s²（門檻 1e-3）→ `cbf.cpp` 的有解門檻放寬成 `kFeasibleSlack` 0.02（數值誤差，不是規則）；
  測試失敗時印出每台的避碰事件（診斷用，留著）。
  `animate_demo.py`（使用者：demo_final.png 太亂）：任務旗子標名字＋狀態（T3 ✓、G1 -> uav2）、火 F1／F2（處理完 ✓，事件列表同名）、
  只畫用到的場景點（yard 的 R 不畫）、狗的軌跡只畫最近 30 s 細線不畫箭頭（巡邏來回）、箭頭間隔 4 → 10 m；`--final-only` 只重畫最後一格。
  同日再改成使用者的「CBBA 動畫顯示需求」：左邊載具同圖示＋名字（UAV 有機頭朝移動方向）、播放中不畫路徑、任務全用旗子（灰 T、綠 G、紅 F、黑 M）只寫編號；
  右邊 Live Agent Status（Agent／Task／Status，從事件推算）、Battery (%) 同色，不顯示任務數量；最後一格加 UAV 完整航跡（實線／虛線／點線）、Makespan、Final Battery。`--at 秒` 只畫一格。
  同日拿掉 Gazebo 視窗的 GridConfig 網格（`cbba_gui.config`），只留場景裡的網格。
  獨立的隨機火落在 tank_2 頂上（場地角落，整場最近 44.8 m）沒人經過、沒人發現（`20261105_135112_demo` D2、D3 FAIL）。
  使用者：火要靠**移動中的無人機、狗看到**才會被發現（不是一出現就變任務）→ ① **狗巡邏**：`--dog bt` 時 itest 照伯宇
  README_WORLD.md 的巡檢線（`worlds.yard.dog_patrol`）產生任務檔（`out/<run>/dog_mission_<ns>.yaml`，`policy.loop: true`），
  `dog_bt.sh start <ns> <機號> [任務檔]`；CBBA 任務插隊、做完接回巡邏（伯宇的 BT 本來就會）。`--dog fake` 照舊不巡邏。
  ② **火只放在有人會經過的範圍**（`demo.fire_in_coverage`）：狗巡檢線 `dog_detect.radius` 內，或無人機現在航線的下視相機範圍內；
  M 的火照原本。試過又拿掉：T 的火固定放 T 旁 6.5～8.5 m（使用者：只是把 T2 拉開，不是要的）。
- 2026-11-07：2D 動畫版面定案（見「2D 動畫的版面」）；避碰路權最終規格（只看移動意圖與機號）。
- 2026-11-08～09（影片）：demo 動畫中途不停頓；做完的任務 Gazebo 刪標記、2D 旗子變淺灰、`gazebo_labeled` 標籤拿掉（最後一個任務做完就離開迴圈、標記沒刪的問題已修）；
  狗的日常巡邏加 `ptz_inspect`、巡邏點 A1～／B1～／C1～（紫色、做完變灰／消失、下一圈再出現，讀狗 BT 的 log 才和實際同步）；狗的偵測範圍圓。
- 2026-11-09（第 5 週）：狗的容器（`DOG_COUNT`）；Fast DDS 位址上限兩個坑（`remote_locators` 16；XRCE agent 不吃設定 → 狗共用一條線、sim 網卡 ≤ 4）；
  `publish_announced`；itest `--dogs`；loss 空地混合任務；C9 含狗 PASS；demo 斷網情境（`blackout`、B1～B4）；`cbba_tx` 與每台 TX／重傳 TX；
  `plot_loss.py` 照週計畫重畫（5 s 收斂率｜收斂時間 Median／P90，逾時記 10 s｜每台 TX、重傳 TX，Retry 含在 Total 裡）；`plot_blackout.py`。
  使用者的範圍確認：掉包測試只負責 tc 梯度＋多源任務注入（V60 的狀態轉移、連鎖退標是伯宇的）。
- 2026-11-10：分析〈7. 以多機為核心的任務分配〉（合作任務：建議「角色拆分」，不採團隊投標，只分析、沒做）與〈2-1b〉（整理成「下一階段：2-1b」一節）。
- 2026-11-11～12：2-1b M0 合約定案；第 1、2a、2b、2c、3 步（能力過濾、協定版本 4、uint64、世代、TaskSpec）。
- 2026-11-13：第 4a～4e 步。使用者的定義與決定：**閘道是非競標的 CBBA 協定節點，不屬於任何載具平台、不列入能力統計**（AGENT_STATE 平台 0）；
  外部 task manager 固定每種突發任務的步驟、BT 只需要一個接口（`current_task.spec`）；spec 用 **2-1b 的通用 steps**（狗和無人機同一套名稱，
  方便同時下命令、搭配 Node-RED），狗的 `ptz_inspect` 統一改名 `inspect`、spec 的 inspect 沒寫 snapshots 拍 1 張（PDF 沒規定張數，
  伯宇對 CBBA 任務原本就拍 1 張）；CBBA 分批做（M0 只加可行性、M1 才動公式）。手動測試的方式見 M0 進度表下。
- 2026-11-14：4f 第一批（硬期限、release、任務的電量下限、UAV `max_bundle: 2`）。

## 新的模擬端測試：指派＋火警＋插入（2026-10-20；10/21 SITL 跑通，狀態監控還沒做）

使用者 2026-10-20 定的流程（itest demo 的新情境，取代以 `cbba_overrun` 為主的展示）：

| 步驟 | 內容 | 預期行為 | 現況 |
|---|---|---|---|
| 1 | 確認無人機全部起飛 | 三台 `AIRBORNE`（巡檢高度）後才往下 | itest demo 已有（全部起飛完成才啟動 cbba_node）；要在報告列成一項 |
| 2 | 開始出價控制 | cbba_node 啟動、收斂 | 已有 |
| 2-1 | 固定任務 G1、G2、G3 | CBBA 分給三台 | 已有（情境檔 `tasks`） |
| 2-2a | 突發人工指派 M：正在做 G 的那台轉去 M | 那台交還 G（`PREEMPTED_BY_MANUAL`，5 s 內不接）→ mode MANUAL、去 M；**G 重新競標，由別台（閒著或做完的）接手** | `m` 指令已有（只能互動輸入）；情境檔要能自動下、目標指定成「正在做 G? 的那台」 |
| 2-2b | 承 a：去 M 的路上（或 M 附近）看到火警 | **照火警 BT**：發布地面任務 → 轉到火點上空監控到狗處理完（最多 120 s）→ 回 M 完成；**輸出「M 延遲」訊息** | 火警 BT 對 `manual_` 點本來就會做（NavigateTo／AirRecon 不分前綴）；延遲訊息還沒有 |
| 3 | 插入任務 T | 由**沒被指派、或已做完**的那台執行（不是在做 M、盯火的那台）；還在做 G 的那台也可以接，排在 G 後面、做完才開始（G 鎖定中、不搶占） | 情境檔 `tasks` 的 `at`／`after_done` 已有；要加檢查 |
| ＊ | 同時調整 CBF | 讓路時不要停頓，至少能**順利轉彎**繞過 | 見下 |

### 要做的

| 項目 | 檔案 | 做法 |
|---|---|---|
| 情境檔的人工指派 | `itest.py`（`scenario_items`、`insert_due`）、`scenarios/manual_fire.json`（新） | `manuals`：`target`（`uav2`，或 `{"holder": "G3"}` = 觸發時拿著 G3 的那台）、`point`／`x y`、`duration`；觸發照 tasks（`at`、`after_arrival`、`after_done`、`delay`）；用現有的 `manual_cmd` 呼叫 `/<ns>/manual_override` |
| 火放在 M 的路上或附近 | 同上 | `fires` 加 `near`（點名＋偏移）或 `on_route`（被指派那台到 M 的線段上某比例）；起火時間要在指派之後、那台飛到之前 |
| M 延遲訊息 | `uav_leaves.hpp`（NavigateTo 轉向、AirRecon 監控） | 目前的點是 `manual_` 時：開始監控印「警告：人工指派 manual_N 延遲：發現火警，先監控地面任務 XXXX（最多 120 s）」，回 M 時印「繼續（延遲 N s）」，做完印總延遲（只印 log） |
| 地面站的狀態監控 | `itest.py`、`insert.py` | itest 每秒把各台狀態寫到輸出資料夾的 `status.txt`（從 ros_bridge 的事件整理）；insert.py 加 `s`（印一次）和 `insert.py --watch`（另一個 terminal 每秒更新） |
| 檢查（情境專用，S1～S6） | `itest.py` | S1 全部起飛才開始出價；S2 G1～G3 都完成；S3 M 被接受、被搶的 G 回報 `PREEMPTED_BY_MANUAL` 且由**另一台**完成；S4 去 M 的那台發布地面任務、狗處理完、有延遲訊息、M 在地面任務結束後才完成；S5 T 的執行者在插入時沒有任務（閒著或做完）；S6 CBF 最近距離 ≥ 0.9 × safe_radius、回程沒有停住超過 1 s |
| CBF：讓路不停頓 | `uav_flight.hpp` 的 `loiter()`、`test_skeleton.cpp` | **已改（2026-10-20；10/21 SITL 的 manual_fire D7 最近 1.74 m）**：飛回起飛點途中不停下讓路，交給 CBF 的 `k_tan` 往右繞；停在被推開的位置只用在原地待命（還沒出發，或離起飛點 ≤ `yield_radius`）。測試 `HomingTurnsAroundHoveringBusyNeighborWithoutStopping`（航線正前方有懸停的執行中鄰機：不讓路、距離 ≥ 0.9 × safe_radius、途中沒有停住 > 1 s、回到起飛點）。SITL 繞得不夠順再調 `cbf.k_tan`（0.8）、`cbf.gamma`（1.0） |
| 情境安排交會 | `scenarios/manual_fire.json` | 被搶的 G 由別台接手時的路線，經過盯火的那台附近，順便看 CBF 轉彎 |

### 已實作（2026-10-20；10/21 SITL 跑通：`docker/itest/out/20261021_094443_demo`，結果見「目前狀態」）

- 情境檔 `docker/itest/scenarios/manual_fire.json`：`python3 docker/itest/itest.py demo --scenario manual_fire --seed 7`。
  新欄位：`manuals`（`target` busy／uavN、`random`、`at`、`delay_range`、`fire` 的 route／near／route_prob），
  tasks 的 `random`、`after_manual`、`delay_range`。`itest.py`：`busy_uavs`、`random_spot`、`manual_now`、`manual_log`、
  `manual_finished`（看 BT log 的「平板任務 N 完成」）、`check_manual`（報告 M1～M3，**都是 INFO**：使用者說延遲、交還是要通知的現象，不是錯誤）、`ground_station_watch`（跑的時候在 terminal 即時印「⚠ 地面站通知」：交還 CBBA 任務、改由誰接手、BT 的人工指派延遲／繼續／完成；`--interactive` 時也寫給 insert.py；互動的 `m` 指令一樣有）。火放好時離其他無人機、其他任務點、
  起飛點 ≥ 6.5 m（只有被指派的那台看得到）。順便修：互動模式結束時 `plan["manual"]` 被蓋成名稱清單（2D 動畫畫不出指派點）。
- BT：`uav_leaves.hpp` 的 `isManualPoint`，NavigateTo 轉向／回航線、AirRecon 監控／完成時印「警告：人工指派 manual_N 延遲／繼續／完成」；
  `test_skeleton` 加 `ManualPointFireFollowsFireFlowAndLogsDelay`、`ManualPointFireEnRouteLogsDelayAndResumes`。
- 讓路：`loiter()` 回起飛點途中不停（上面的表）；`test_skeleton` 加 `HomingTurnsAroundHoveringBusyNeighborWithoutStopping`。
- M3（2026-10-20 使用者）：T 給還在做 G 的那台也可以——G 鎖定中、同級不搶占，T 排在後面、G 做完才開始，算符合 3-1；M3 檢查 T 開始時前一個任務已完成、不在人工指派中。

### 使用者 2026-10-20 的決定

| 項目 | 決定 |
|---|---|
| 位置 | **隨機**（才測得到 CBBA）：M、火、T 的位置在範圍內隨機（離其他點 ≥ min_gap），用 `--seed` 重現、記在 demo_plan.json |
| 火 | 在被指派那台飛往 M 的**路上或 M 附近**（隨機：線段 40～80 % 處，或 M 旁 ≤ 3 m）；目的是確認照火警 BT 執行 |
| 插入 T | 位置、時間隨機（指派 M 之後的一段時間內） |
| 被指派 M 的那台 | 觸發時「正在做 G 的那幾台」裡隨機挑一台、時間隨機（使用者：要先測隨機） |
| M 延遲訊息 | **先只印 log**（BT 的 log，字首「警告：」），不發話題 |
| 地面站的監控 | **插入的那一端（insert.py／itest 的 terminal）顯示無人機和狗的狀態**，內容對齊 2D 動畫：每台的 mode（AUTO／MANUAL）、現在的任務、位置、電量、在做什麼（前往／懸停／盯火／待命／回起飛點），狗也一樣；資料用 ros_bridge 已經記的事件（pos、assigned、dog、dog_assigned、battery、result、closed） |

## 無人機避碰（2026-10-24 定案；`uav_px4_bt/include/uav_px4_bt/avoidance.hpp`）

目標（使用者）：路線交叉時不碰撞、不互相卡住（不要「停下來等人過」），各自繼續往目標。純 CBF 做不到「不停頓」
（對飛、交叉時只會減速），所以要有一套**每種交會都有唯一結果**的路權規則，但只用最小的一套，不是把所有依據都加進來。

```
每台鄰機：預測會不會碰（u_nom 的有限線段 CPA）
├─ 不會：照原路飛，不加 CBF 約束（距離 < guard_radius 例外）
└─ 會：路權（2026-11-07）：一台動、一台停住 → 移動中的讓；都在動 → 機號大的讓；都停住 → 不繞，只靠 CBF
       （停住＝沒有移動意圖：已到目標或 BT 沒在控制，不看瞬間速度、busy、任務類型）
       → 讓的那台用 VO 找離 u_nom 最近的安全速度（先只轉方向；左右一樣時往右；鎖定那一側；找不到才降速）
       → 角色、方向鎖定到已經通過且距離變大，或預測不衝突且距離 > yield_exit
最後 CBF 保底（預設二階 cbf2Filter：讓的那台 ρ = 1；其他只在 < guard_radius 或剎車距離不夠時 ρ = 0.5）
距離 < safe_radius 或 CBF 無解 → 緊急退讓（不用投影的結果）
```

| 項目 | 做法 |
|---|---|
| 模組 | `avoidance.hpp/.cpp`：`predictCpa`（ConflictPredictor）、`hasRightOfWay`（RightOfWayPolicy）、`CollisionAvoider`（EncounterState、VO、CBF、緊急）；`cbf.hpp/.cpp` 拆出 `nominalVelocity`、`cbfFilter`（指定鄰機與 λ、ρ）；舊的 `cbfVelocity`（約束一直開著＋違反後往右繞）保留給 `test_cbf` 對照 |
| 呼叫 | `UavContext::moveTo`（正式 BT、備用 BT 都經過它）；`yield_to_busy` 參數不用了 |
| 停住（2026-11-07 使用者定的規格） | **只看移動意圖與機號，不看 busy、任務類型**：一台移動、一台停住 → 移動中的用 VO 側向繞開；兩台都在動 → 機號大的讓；兩台都停住 → 不繞行，只由 CBF 維持距離。停住＝已到自己的目標（< max(2 × reach_tolerance, 0.5 m)）或 BT 沒在控制（offboard 掉了），**不看瞬間速度**（11/06 試過速度 < 0.3 m/s：0.5 m/s 巡航時讓路減速的那台被當成停著）。由位置廣播 bit 3 送出（欄位名 `task_hover` 沿用）。預測固定看滿 5 s；通過、確認遠離（連續 3 週期距離變大）就解除。目前每台目標不同，暫不考慮兩台前往同一座標 |
| 參數 | `cbf.safe_radius` 1.0（程式預設也改成 1.0，全專案統一）、`ttc_horizon` 5 s、`miss_clearance` 1.5、`guard_radius` 1.3、`yield_exit` 2.0、`second_order` true、`response_delay` 0.3 s、`emergency_speed` 0.3（一階用）；itest `--speed` 不是 0.5 時自動給 `cbf_speed_args`（3 m/s → miss 2.1、exit 2.6；二階時剎車距離由 CBF 自己處理，guard、γ 不用改） |
| log | 「避碰：預測 t* s 後和 uavN 最近 d* m（現在 x m）→ 我讓行（VO 繞）｜對方讓行，我直行」、「避碰：和 uavN 解除（…往右繞，現在 x m）」、「避碰：保持速度繞不開，降到 …」、「警告：避碰緊急退讓 …」 |
| 單元測試 | `test_avoidance`（15 項：路權鍵唯一、CPA、錯開 2 m 不減速、0.88 m 的重現（照 SITL 的順序：uav3 先懸停、uav2 繞開、uav3 再起步）、對飛、交叉、任務懸停優先、待命讓執行中、起降板不動、3 m/s 回起飛點對調（停機點 5 m）、3 m/s 交叉、角色鎖定、夾擊緊急、`Cbf2.BrakesInTimeAtSpeed`、`Cbf2.WidePassNotBraked`）；`test_flight_beacon` 加 bit 3 |
| 不影響狗 | 位置廣播（UDP 14601）、CBF 只有 `uav_px4_bt` 用；伯宇的狗 BT 沒有 UDP，cbba_node、協定、介面都不變 |
| 停機點間隔 | yard 5 m（`worlds.yard.spawn_spacing`）：3 m/s 剎停約 3.9 m（含反應延遲），2 m 時讓的那台被夾住、CBF 無解。是場景的設定，不是避碰規則 |
| 選做（使用者 10/24） | 回起飛點最後一段限速（例如離家 5 m 內 ≤ 1 m/s）：只改名目速度的上限，不加避碰規則。目標是合理、好解釋的結果，不是最佳解 |
| 已知現象 | 兩台約 6 m/s 對飛時，讓的那台（假設只有自己剎）在 14 m 外就判成剎不住，觸發 1～2 s 的緊急退讓（VO 已經讓航線錯開，不危險，影片裡可能像抽動一下） |
| 不做（之後需要再加，只改 RightOfWayPolicy） | 火警＞手動＞固定＞插入、電量、離目標遠近、協商封包、完整 ICAO、任務代價、高度分層 |
| 報告（10/25） | `docs/reports/cbf_nonlinear_control_20261025.pdf`（原始檔 `docs/cbf_nonlinear_report.html`，Chrome headless 轉）：以非線性控制的觀點寫——系統模型、安全集合與 CBF、距離平方的相對階數 2、剎車距離障礙函數與推導、責任分配 ρ、CBF-QP 與可行性、延遲餘量、監督式名目控制（路權打破對稱、遲滯、VO）、參數來源、單元測試與 SITL 結果、限制；文獻依記憶整理，引用前要核對 |
| 二階 CBF（10/24） | `cbf.second_order: true`（預設；false = 一階對照）。模型 ṗ = v、v̇ = a、\|a\| ≤ accel_max（＝ `motion.max_accel`，UavContext 自動同步）；h = √(2A(d − r)) + ḋ（A = accel_max / ρ），ḣ ≥ −γ h；安全計算用實際量到的速度，VO 的速度換成加速度（從平滑層的參考速度 `MotionShaper::referenceVelocity` 起算），過濾後積分回速度——平滑層不會再改掉 CBF 的結果；剎車距離的餘裕 < 0.5 m 時也加約束；安全距離加上 `cbf.response_delay`（0.3 s）× 靠近速度；對方沒在靠近（< 0.2 m/s）時直行的也全額負責（ρ = 1）——單元測試 3 m/s 起降板對調時靠到 0.57 m 才加的。緊急＝最大加速度、對所有鄰機約束違反最少的方向（`cbf2Escape`；只往遠離最近那台時，會被推向另一台高速飛來的，單元測試 0.31 m）。itest 3 m/s 只改 miss 2.1、exit 2.6（一階時要 guard 4、miss 4.5）。`test_avoidance` 加 `Cbf2.BrakesInTimeAtSpeed`、`Cbf2.WidePassNotBraked`，緊急退讓測一階、二階 |

二階 CBF 的三個補丁（`response_delay`、對方沒在靠近時 ρ = 1、`cbf2Escape`）是為了 3 m/s、停機點 2 m 的回程對調單元測試加的，
測試最後是靠停機點改成 5 m 才通過；補丁留著（都有道理，也有單元測試），但之後不要再為極端情況疊規則，先看場景、參數合不合理。

SITL 驗證（yard、3 m/s、`assign_random --seed 7`）：10/24 23:59 全過，T1 經過懸停的 uav2 時 uav1 用 VO 繞過、最近 2.25 m、不停頓；
懸停讓路的速度是「剛好來得及讓開」（1.5 ×（要側移的距離）/ 交會前的時間，最低 0.2 m/s），原本 max(0.3 V, 對方速度) 時會衝出去 4.6 m 再繞回。

## 2D 動畫的版面（`docker/itest/animate_demo.py`，2026-11-07 定）

使用者的設計原則：影片途中看載具現在在哪、在做什麼；最後一格才看 UAV 完整的飛行結果。任務靠旗子顏色＋編號，載具靠相同圖示＋名字。
輸出 1440 × 720。`--at 秒` 只畫一格（`demo_at.png`）、`--final-only` 只重畫最後一格（`demo_final.png`）。

| 部分 | 影片途中（`demo.mp4`） | 最後一格（`demo_final.png`） |
|---|---|---|
| 標題列 | 不放（地圖用整個高度） | 深藍標題列「CBBA Multi-UAV Task Allocation Result」、副標「PX4 SITL (場景) \| N UAVs \| Seed」 |
| UAV | 相同的四旋翼圖示＋UAV1～3，不畫路線；移動中旁邊一個紅色小箭頭指移動方向 | 不畫圖示；完整航跡用各台的顏色（藍、橘、綠）＋方向箭頭 |
| 狗 | 狗圖示＋dog1～3，不畫路線；移動中有小箭頭 | 最終位置（不畫路線） |
| 任務 | 全部用旗子、旁邊只寫編號，顏色同 Gazebo：黃 T（UAV 一般任務）、綠 G（固定任務）、紅 F（火災／地面處置，地面任務建立時才出現）、藍 M（人工指派） | 同左 |
| 右上卡片 | 「Live Agent Status」：Agent／Task／Status 三欄直接放文字（**不畫表格線、不放底色框**）。Status 只有三種：En Route（前往）、Executing（到點執行中）、Done（最後一個任務做完，直到接到下一個）；還沒有任務時是 — | 「Live Agent Status — Snapshot」：每台做完的任務（照完成順序，例如 `T1, T3`）、✓ Done；UAV 都列，狗只列做過任務的 |
| 右中卡片 | Battery (%)：各台顏色的長條＋百分比 | Final Battery (%) |
| 右下卡片 | 圖例：左半「Agents」（UAV 圖示、狗圖示、移動方向箭頭）、右半 Task Flags | 左半換成 UAV Trajectories（各台線色） |
| 不顯示 | 任務數量（Tasks Completed）、Makespan（使用者 11/06、11/07 拿掉） | 同左 |

狀態從事件推算：有任務時離任務點 1 m 內（狗 2.5 m：伯宇的 BT 停在目標前 1.5 m）算 Executing；人工指派 M 的結束 = 到點、停滿 duration 後離開 1.5 m。
地圖上的時間在右下角圓角框（最後一格加「(end)」）。不要開圖檢查（耗 token）：圖由使用者自己看；要確認時序用 `--status` 之類的文字輸出。

**11/07 改好**（使用者看過新版 demo 後提出）：
1. **旗子顏色和 Gazebo 的地面標記一樣**：G 綠、M 藍、T 黃、F 紅（`FLAG_COLOR`，同 `label_gazebo.py`），上表的灰 T、黑 M 已作廢。
2. **Live Agent Status 跟著實際在做的事切換**：Task 平常是被分派的任務（CBBA 的 current_task 或人工指派 M）；**監控火時切成那個火**
   （發現的那台：回報後還沒到火點上空 = F En Route、到了 2 m 內 = F Executing；別台停在火點上空 2 m 內、速度 < 0.5 m/s 也算），
   到地面任務結束（最多 120 s）再切回原本的任務。**M 的完成時間用 BT 的實際紀錄**：itest 把 log「平板任務 N 完成」記進
   demo_plan.json 的 `manual[].finished_t`；舊的輸出由動畫直接讀 `stack_uavN.log`（原本用距離猜，火在 M 旁時會誤判）。
   `--status`：只印每台 Task／Status 的變化（文字，檢查時段用，不用看圖）。
   用 `20261107_032737_demo` 驗證：UAV3 去 G3 途中 23.5 s 看到 F1 → F1 En Route → 27.5 s F1 Executing → 46.5 s 狗處理完、切回 G3；
   UAV2 的 M1 49.5 s Done（BT 實際完成）。

## C/C++ 格式

沒有 `.clang-format`，照現有程式碼（ROS 2 的慣例）：

| 項目 | 寫法 |
|---|---|
| 標準 | C++17；`-Wall -Wextra -Wpedantic` 零警告 |
| 縮排、行寬 | 2 個空白，不用 tab；一行約 100 字元內，續行再縮 2 或 4 格對齊 |
| 大括號 | 函式、類別、namespace 的 `{` 換行；`if`／`for`／`switch` 的 `{` 同一行；`if` 一律加大括號 |
| 參考與指標 | 兩邊都留空白：`const Task & task`、`const char * name` |
| 命名 | 型別 `PascalCase`；函式 `camelCase`；變數、參數 `snake_case`；成員變數 `snake_case_`（尾端底線）；常數 `kName`；列舉 `enum class`，值用全大寫 |
| 標頭檔 | `#pragma once`；開頭註解寫這個檔案做什麼、和規格不同的地方與原因 |
| include 順序 | 自己的標頭 → C 系統標頭（`<fcntl.h>`）→ C++ 標準庫 → 其他函式庫（rclcpp、px4_msgs）→ 本專案；各組之間空一行，組內照字母排 |
| namespace | 核心、協定層、節點在 `cbba_core`，封包格式在 `cbba_core::wire`；檔案內的輔助函式放匿名 namespace |
| 短函式 | 一行的 getter 寫在標頭：`AgentId id() const {return id_;}`（大括號內不留空白） |
| 錯誤處理 | 解析、查詢失敗回傳 `std::optional`／`bool`，不丟例外；建構時參數不合法才丟 `std::invalid_argument`／`std::runtime_error` |
| 註解 | 繁體中文；寫「為什麼」，不重述程式碼；規格沒寫或不照規格的地方一定要註明原因；長檔案用 `// ===` 分段 |
| 測試 | gtest，`TEST(類別, 行為)`；每個測試開頭一行註解寫情境；整合測試走真的 UDP |
| 分層 | 協定層、封包格式、核心不依賴 ROS、socket、檔案；需要時由節點層用參數或 `std::function` 注入（例如 seq 紀錄） |

## 工作習慣

- 使用者指定輸出位置時直接寫到那裡，不另存副本。
- 大改、刪除前先說明影響；刪除未 commit 的檔案前先備份。
- 先做格式與測試，再做邏輯；每一層都要有測試，整合測試走真的 UDP。
