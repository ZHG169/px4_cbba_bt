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
                            由 gen_patrol_site.py 從 patrol_site_points.csv 產生）與 tag 0～7、101、mono_cam 模型
ros2_ws/src/
├── swarm_interfaces/       ROS 訊息 v1.6（＝機器狗端的 v1.2＋FireDetection；Task、RobotState、TaskResult、ManualOverride.srv…）
├── cbba_core/              CBBA 核心、機間封包、協定層、指派流程、離線模擬、測試（不依賴 ROS）；
│                           狗的文件在 doc/ugv_test.md、doc/ugv_tuning.md
├── swarm_cbba/             統一版 cbba_node（無人機和狗同一支，agent_type）、launch、yaml：照機器狗端的套件與介面（2026-10-12）
├── px4_waypoint_node/      PX4 端：takeoff_hover 起飛懸停、task_executor（簡易執行器）、px4_state_bridge（PX4 → RobotState）
├── uav_px4_bt/             無人機的備用 BT（BehaviorTree.CPP v4，trees/main_uav_tree.xml）、CPF 避碰、flight_beacon 位置廣播
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

## 機間通訊：協定版本 2（2026-10-09，使用者要求「每筆任務資料帶 task_id」）

整體說明在 `cbba_core/doc/protocol.md`，逐位元組格式在各 `wire_*.hpp` 開頭。**不相容於規格書**
（`cbba_core/doc/spec/機間通訊封包規格_20261006.pdf`）：規格用 AGENT_STATE 的 z、y 陣列位置（uint8 任務編號）當全隊共用的
任務識別，造成機號 > 8 不能建立任務、每台約 32 個任務、鄰居位元只放得下 1～8。版本 1、2 的節點互相拒收，全隊要一起升級。
**DDS 只用在機內**（PX4、BT），機間全走 UDP。

| 封包 | 內容 |
|---|---|
| 表頭 20 B | magic 0x4342、version 2、type、sender_id u16、session_id u64、sequence u32、payload_length u16；**大端序**、逐欄位寫 |
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
| 沒有任務定義的紀錄 | 不暫存，靠對方補發（它看得到我的 round_key 和紀錄） | 200 ms 後下一次會再帶 |
| 已停止參與的載具 | 收到的紀錄裡得標者是「已知停止參與」的鄰居時略過 | 晚到的訊息會把它寫回得標者，等 1.5 s 失聯才釋放（測試發現，版本 1 也有） |
| 截止時刻 | 絕對時刻（ms，約 49.7 天循環） | 轉送／補發原封不動，期限不會被往後推 |
| round_key | 已知任務（task_id）與是否結束的 CRC-32 摘要 | 不同就比對紀錄補發 |
| exchange_step | 本機分配表變動次數 | 我們是非同步 CBBA，沒有「步」 |
| s | 機號 + 系統時鐘 ms 低 32 位元 | 需要各機對時（chrony） |
| 補發 | 摘要不同時補缺的 TASK_ANNOUNCE／TASK_CLOSE；每 5 s 保底重播一個任務。**不參與出價的節點照樣送紀錄（不送 s）**，收的那端只合併正式指派、不交給規則（2026-10-11 起，原本不送紀錄） | 晚起飛、重開機、飛出範圍再回來；停止參與時自己撤銷的指派要傳得出去（SITL 測試發現：飛行中關掉 PX4，任務卡住沒人接） |
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
| `task_announced`（Task） | CBBA → BT | 全隊已知的任務，每個任務一次（transient_local、depth 256）；**我們加的**（2026-10-14），只有 agent_type = UAV 才發；無人機的火點去重複（別台報過的火不再發布、不轉向、不搶著監控） |
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
- 週計畫的「18 條消解規則」：核心實作 17 條（`cbba_rule_cases.csv`）。
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

## 目前狀態（2026-10-17）

整體進度與缺失清單：`docs/progress.md`（10/12 更新）；機間通訊的整理（做簡報用）：`docs/cbba_comm_status_20261012.md`。

### 現況

| 部分 | 狀態 |
|---|---|
| 機間通訊（協定版本 2） | 主體完成：擴散、分配、完成確認、失敗交回（reason 排除）、停止參與後接手、**失聯接手**（火警 10 s、巡檢 30 s，10/12）、重開機、鎖定、指派版本、權限、30% 掉包 5 s 內收斂 95% |
| CBBA 搶占 | 只有地面處置任務是火警（10/09 定案）；10/12 修評分：搶占時不算被搶占任務的延後、出價不受 bundle 上限 → 巡檢中、離火點近的 V60 搶得到（`test_assignment` 驗證） |
| cbba_node ↔ BT 介面 | 照機器狗端（伯宇）：`swarm_cbba`、swarm_interfaces v1.6、current_task／mode／reason、不握手、assign_hold 0.4 s；log 字串照他的 e2e grep |
| 無人機 BT | **正式：`uav_task_manager_node`（伯宇骨架，上一節），`cbba_uav.sh` 的 `EXECUTOR=task_manager`、itest 預設**；備用 `uav_bt_node`（`EXECUTOR=bt`，預設仍是 bt）；CPF、運動平滑層兩者共用 |
| `px4_state_bridge` | 發 robot_state；mode 照正式 BT 的 `bt_mode`（2026-10-17）；備用 BT 時自己判斷：爬到巡檢高度（`auto_min_altitude`，高度 − 0.5 m）而且 offboard 才 AUTO |
| AprilTag 火警 | 到達後 1.2～1.9 s 判定 tag 101、誤差 < 0.5 m、誘餌不觸發；巡檢點火警 → 懸停監控 → 假狗處理完 → 完成（demo D2～D4）；路過發現 SITL 也看過（10/13） |
| 模擬環境 | docker（sim、uavN、gcs），DDS 只在機內；`fire_site`、`patrol_site`、`demo_site`（±22 m 展示用）；Gazebo 視窗設定 `docker/gz/gui/cbba_gui.config`（網格、關陰影、俯瞰鏡頭）；錄影用場景的俯瞰攝影機；假狗 `docker/scripts/fake_dog.py`（sim，Gazebo 橘色方塊替身）；火點 `models/fire_block`＋tag 101（`patrol_scenario.sh fire-at`） |
| 自動測試 | `docker/itest`：comm（C0～C8）、uav（U1～U6）、loss（掉包梯度＋`plot_loss.py`）、demo（週計畫流程＋假狗，D1～D6；Gazebo 錄影＋俯視動畫 `animate_demo.py` → MP4，要主機 ffmpeg；`--executor` 預設 task_manager）；`swarm_cbba/test/g2_e2e.sh`（伯宇的 T1～T10＋E1～E4＋DEMO3，58 項） |
| 機器狗端 | 假狗（真 UDP；demo 有 cbba_node＋`fake_dog.py` 執行者）正確；**還沒在真的狗上測**，伯宇還在用協定版本 1（互相拒收），要等他換成 main |

最近的結果：2026-10-17 晚上（需求 1＋無人機接管 B）使用者回報全過：單元測試（`test_skeleton` 32 項）、`g2_e2e.sh`、itest uav 6／6。
2026-10-17 下午 `test_skeleton` 29 項全過（加高度與接管警告 3 項）。
2026-10-13：單元測試 cbba_core 145、uav_px4_bt 60（cpf 12、motion 8、tree 13、skeleton 23、beacon 4）全過、零警告；
itest（task_manager）uav 6／6（U6 1.0 s 接手）、demo 6／6（種子 7、21）。
2026-10-12：單元測試 cbba_core 144、uav_px4_bt 33、apriltag 6；itest comm 9／9（C5 32 s 接手）、
uav 6／6；loss 30% 時 5 s 內收斂 95%（中位數 0.65 s）、50% P90 2.9 s（`resend_gap` 1 s 一階）、70% 超出設計範圍；
待命 mesh 3.5 kB/s；demo 的 D1、D2 通過（10/12 19:47 那次；之後的運動、起飛閘門、路過發現版本由使用者測試中）。

**commit**：GitHub main 在 `db56cf9`（2026-10-17，使用者自己 push；之後還有 README、文件的修正）。`uav_bt.yaml` 的 `cbf.brake_vertical_accel`
0.2 → 0.1（使用者調的，10/17 commit）。宜臻的「CBBA 任務失敗排除與人工接管：實作說明.md」在 repo 根目錄、沒進 repo（隊友的資料）。
共用的 `ros2_ws/install` 改 C++ 後要在 uav 容器重新編譯 6 個套件。
隊友的資料（`docs/handoff_uav_*`、`docs/messageImage_*.jpg`、`swarm_cbba/testdata_for_uav_2026-10-12/`）不進 repo（`.gitignore`）。

### 還沒做的（依建議順序）

1. 被接管、返航時 mode（2026-10-17 已改，**待 SITL 重測**）：選 B——正式 BT 決定 mode、bridge 照抄；offboard 掉了時用 PX4 的
   `failsafe`、`failsafe_and_user_took_over` 分辨人接管（`PREEMPTED_BY_MANUAL`）和 PX4 失效保護（`PX4_FAILSAFE`），SITL 已確認分得出來
   （`docker/scripts/watch_px4_status.py`；模擬沒有遙控器，`posctl` 會被拒，用 `commander mode auto:loiter` 模擬接管）。
   懸停高度：起飛完成那一刻的高度當待命點（實測停在 4.71 m，不是 5.0 m），要不要固定用巡檢高度待定。
   `cbba_uav.sh` 的預設執行者還是 `bt`，要不要改成 task_manager 待定。
2. 殘留的 Gazebo 伺服器會不理 SIGTERM（itest 已改成 SIGKILL 後備、`px4_sitl.sh` 會提示 `pkill -9`）。
3. 第 5 週：`resend_gap` 開成 cbba_node 參數、量 1.0／0.5／0.3 s；30% 再量 100 次。
4. 真的 V60：假狗（`fake_dog.py`）已經完成「發布地面任務 → 狗競標 → 走過去 → 處理完」；巡檢中被火警搶占（狗有自己的巡檢）
   還沒在 SITL 跑；等伯宇換成 main（協定版本 2）。
5. session 層、認證；新版 CBBA 的其他 constraints；多跳腳本、實機 mesh、Jetson、chrony、實體相機、能量模型校正、itest ssh 版。
6. 對外：`protocol_v2_notice.md` 的介面那段要改成現況再寄規格作者；給伯宇的說明；架構圖。

### 待決定

| 項目 | 狀況 |
|---|---|
| 300 ms 全流程 | 目前指派約 0.4 s（assign_hold）；建議只在有競爭、未收斂時才等；預算要和伯宇、宜臻分 |
| BT 的 UDP | 2026-10-13 決定先維持在 BT（`flight_beacon`） |
| 平板／GCS 的角色 | 要不要跑 cbba_node、能否建任務；部分保留可用容量 Q（討論過、沒實作） |
| 取消的授權管理端 | `cancel_authorities` 目前是空的 |
| 電池權重測試的設定 | 續航、插入時段、耗電誤差要多緊（`docs/sim_scenario_plan.md` 第七節） |
| 失聯的那台「不接新的」 | 各機分不出自己是不是失聯的那台，沒有另外實作 |

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
  還沒做：孤兒任務警告（`orphan_warn_sec`）、需求 2、兩隻狗 TRANSIENT 後由另一隻得標的整合測試。無人機 mode 的討論：狗是 BT 自己發 robot_state（2 Hz，
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
