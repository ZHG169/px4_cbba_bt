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
| 失聯接手（2026-10-11 定案，**還沒實作**） | 失聯的那台把手上的任務做完（不接新的）；別台在持有者失聯（s 超過 lost_timeout 沒更新）後，**火警（GROUND_INTERVENTION）10 s、巡檢 30 s** 才撤銷鎖定、重新競標（撤銷 = 新版本，執行者 0）；恢復連線後舊持有者的回報版本較舊，照現有的回報檢查拒絕（ACK status 2） | 「通訊斷但還在做」和「墜毀」從外面分不出來：火警寧可重複、巡檢寧可等。時限之後依第 5 週弱網的量測調整；狗照同一套規則，要和伯宇、宜臻對齊 |
| BT 接受（握手） | **2026-10-12 起不用**（照機器狗端）：排第一 `assign_hold` 0.4 s 就發 current_task，BT 的哨兵負責中斷。核心的握手程式和測試保留（`require_accept` 固定 false，之後要地面端最終接受權時再打開） | 使用者 2026-10-12 決定照伯宇的介面 |
| 失敗後的排除（2026-10-12） | TaskResult.reason：PERMANENT → 這台永久不再對它出價；TRANSIENT／PREEMPTED／STOPPED → `failure_exclude_sec`（60 s）後恢復 | spec §2.3 規則 3（機器狗端） |
| 搶占與排隊 | 優先級：BT 本機安全處置 ＞ 火警（GROUND_INTERVENTION）＞ 巡檢；同級不搶占；已接受的任務釘在路徑前面。2026-10-12 起沒有 ExecState：一律當可中斷、剩餘時間不知道 → **執行中的火警之後不排隊第二個火警**（交給別台，或做完再接） | Accept = F ∧ [Idle ∨ (P_j > P_k ∧ Preemptible)] |
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
| `robot_state`（RobotState） | BT → CBBA | 位置（map ENU）、電量（負值 = 不知道）、`mode`（AUTO／MANUAL／ESTOP）；`state_timeout` 1.5 s 內有更新**而且 mode = AUTO** 才出價，否則手上的任務立刻釋放。無人機由 `px4_state_bridge` 發（在空中 = AUTO）；第一筆當返航點 |
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
| 1 | 機間訊息、mesh＋`tc` | 協定版本 2、`mesh_tc.sh` | 通訊測試、30%／100% 掉包通過；DDS 漏到 mesh 待修 |
| 2 | 3D 航點、空中出價 | 備用 BT 的 FlyToWaypoint（CPF）、相容表 | itest U2、U5（三台分配；避碰交叉情境還沒測） |
| 2 | 混合任務只對專長出價 | `types.hpp` 能力檢查 | 真 UDP（假狗）、伯宇的 T1 |
| 3 | AprilTag 火情確認 | `apriltag_fire_detector`、`fire_site` | itest U3（約 1.6～1.9 s、誤差 < 0.5 m）、U4（誘餌不觸發） |
| 3 | `ReportFireEvent` 注入地面任務 | 備用 BT | itest U3；狗（假狗）得標 |
| 2～3 | 巡檢＋隨機插入、電池權重 | `patrol_site`、`patrol_scenario.sh`、`mission_sim`＋`cbba_sim insert` | 離線 100 次×5 權重已跑；Gazebo 抽驗還沒跑 |

正式的 UAV BT 要照伯宇的範本重寫（`docs/handoff_uav_2026-10-11/pkg2_bt_template`），寫好後用同一套 itest 重驗第 2、3 週。

## 目前狀態（2026-10-12）

整體進度與缺失清單：`docs/progress.md`；機間通訊的整理（做簡報用）：`docs/cbba_comm_status_20261012.md`。

### 現況

| 部分 | 狀態 |
|---|---|
| 機間通訊（協定版本 2） | 主體完成：擴散、分配、完成確認、失敗交回（reason 排除）、停止參與後接手、重開機、鎖定、指派版本、權限、30% 掉包容錯 |
| cbba_node ↔ BT 介面 | 2026-10-12 照機器狗端（伯宇）：`swarm_cbba`、swarm_interfaces v1.6、current_task／mode／reason、不握手、assign_hold 0.4 s。伯宇的 BT 不用改，換三個套件重新編譯即可 |
| 無人機 BT | 備用 BT `uav_px4_bt`（CPF 避碰、火警判定、低電量返航）可用；**正式的 UAV BT 要照伯宇的範本寫**（`docs/handoff_uav_2026-10-11/pkg2_bt_template`：NavigateTo、AirRecon、DefaultBehavior、IsEmergency，內容從 uav_px4_bt 搬） |
| AprilTag 火警 | `apriltag_fire_detector`，SITL：到達後 1.6～1.9 s 判定 tag 101、誤差 < 0.5 m、誘餌不觸發 |
| 模擬環境 | docker（sim、uavN、gcs）、`fire_site`／`patrol_site` 場景、`px4_sitl.sh` 多台自動加入同一場景 |
| 自動測試 | `docker/itest`（只看話題與網路，介面在 config.json）：comm C0～C8、uav U1～U6 |
| 機器狗端 | 假狗（真 UDP）正確；伯宇的 T1、T2、T9 在我們的節點照跑一致；**還沒在真的狗上測**，等伯宇換成 main 的程式 |

最近一次測試（2026-10-12，隔離的 docker 環境，3 台）：167 項單元測試全過、零警告（test_comm、test_assignment 各連跑 20 次）；
`itest comm` 8／9（C5 失聯接手還沒實作）；`itest uav` 6／6；指派約 0.4 s，PX4 斷線後約 2.4 s 別台接手；
30% 掉包時任務傳播、完成確認約 0.5～2.8 s（`resend_gap` 1 s）。

**還沒 commit**（2026-10-11～12 的修改：A1、px4_sitl.sh、itest、文件整理、介面對齊、status_timeout）；
共用的 `ros2_ws/install` 也要重新編譯 6 個套件（swarm_interfaces 變了）。
`docs/handoff_uav_2026-10-11/`、`docs/messageImage_….jpg`、`swarm_cbba/testdata_for_uav_2026-10-12/`（伯宇的測資）
**不進 repo**（2026-10-12 使用者決定，已加進 `.gitignore`，只留在本機）。
cbba_node 的回歸測試：`swarm_cbba/test/g2_e2e.sh`（伯宇的 T1～T10＋E1～E4＋DEMO3，58 項，2026-10-12 全過）；
log 字串已照伯宇的 e2e 腳本 grep 對齊（`mode 0 → 1`、`reason 2，detail「…」`、`失敗（detail）`、`收到完成證明`）。

### 還沒做的（依建議順序）

1. **DDS 漏到 mesh**：patch 在 scratchpad（`dds_wire_only.patch`：`.env` 加 `DDS_OVER_MESH=0`，uav 的 DDS 只綁線材網卡），
   隔離環境驗證過，使用者 2026-10-11 說「先分析」、還沒套用。待命時 mesh 約 69 kB/s 大多是 DDS，第 5 週掉包數據的前提。
2. **失聯接手**：已定案（火警 10 s、巡檢 30 s，見協定的定案表），還沒實作；`itest` C5 驗收。
3. **第 5 週量測**：itest 加掉包梯度 0／10／30／50／70%、協定版本 2 的收斂率統計、收斂時間與頻寬紀錄、畫圖；之後調 `resend_gap`。
4. **正式的 UAV BT**（照伯宇的範本）；避碰的刻意交叉測試（itest U5 的航線沒有交錯）。
5. session 層、認證（只有 `session_registry.hpp` 介面）；新版 CBBA 的其他 constraints（競標前、正式指派前、執行期間）。
6. 多跳（A→B→C）的 docker 腳本；實機 mesh（multicast 轉送、MTU > 1200 B）；Jetson（arm64）映像、chrony、實體相機與
   `kCameraToBody`、能量模型校正、共同座標系（實機沒有出生點間隔）、`itest` 的 ssh 版。
7. 對外：把 `protocol_v2_notice.md` 送給規格作者（版本 2 不相容於規格書；另外要確認座標系、拒絕確認、重送 10 次後、重開機）；
   給伯宇的說明（main 已照他的格式、`dog-integration` 不用了、行為差異見 `swarm_cbba/README.md`）；架構圖要更新。

### 待決定

| 項目 | 狀況 |
|---|---|
| 300 ms 全流程 | 目前指派約 0.4 s（assign_hold）；建議只在有競爭、未收斂時才等；預算要和伯宇、宜臻分 |
| 平板／GCS 的角色 | 要不要跑 cbba_node、能否建任務；被指派的載具不再接任務：已有 `mode = MANUAL`（整台），部分保留可用容量 Q（公式的可行性限制，討論過、沒實作） |
| 取消的授權管理端 | `cancel_authorities` 目前是空的 |
| 電池權重測試的設定 | 續航、插入時段、耗電誤差要多緊（`docs/sim_scenario_plan.md` 第七節） |
| AirRecon 看到火之後 | 立刻算完成（無人機去下一點）或懸停監控到停留時間（週計畫的「持續監控」，目前的做法） |

### 歷史紀錄（濃縮）

- 2026-10-07：協定版本 1、task_executor 的第一階段 SITL（分配、完成確認、失敗交回、墜毀、晚加入、重開機）；
  SITL 電量對齊（PX4 續航 900 s、最低 20%；能量模型懸停 0.111 %/s、每公尺 0.022 %，`cbba_uav.sh` 傳入）。
- 2026-10-09：通用 cbba_node、協定版本 2、正式指派版本、TASK_CLOSE 權限、程序鎖、BT 接受／搶占／排隊、鎖定；
  備用 BT、AprilTag 偵測、`fire_site` 場景。
- 2026-10-10：`patrol_site`、`patrol_scenario.sh`、`mission_sim`＋`cbba_sim insert`（800 個任務全完成，接錯 21 → 15 隨權重下降）。
- 2026-10-11：SITL 實測協定版本 2＋備用 BT；修 A1（停止參與時照樣送紀錄）、`px4_sitl.sh` 自動加入場景；`docker/itest`；
  文件整理（docs、cbba_core/doc）；失聯接手規則定案；收到伯宇的 handoff。
- 2026-10-12：照伯宇的介面改（swarm_cbba、v1.6）；修 `px4_state_bridge` 的 `status_timeout`（2 → 5 s，`vehicle_status`
  約 1.4～2.6 s 一則，原本一直誤判、反覆撤銷任務）。

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
