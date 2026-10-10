# CLAUDE.md

給在這個 repo 工作的 Claude 的說明。使用者負責**無人機端**（PX4、BT、AprilTag、空中 CBBA、mesh＋tc）；
機器狗（Ghost V60）端由隊友負責。回覆與文件一律用繁體中文。

## 專案

空地異質系統的無人機端：PX4 旋翼機以 CBBA 分散式拍賣分配任務，交給行為樹（BT）執行。

```
docs/                       跨套件的規劃：sim_launch_guide.md（**模擬測試的開啟方式**）、week4_6_status.md（第 4～6 週和程式的差異）、
                            sim_scenario_plan.md（巡檢＋隨機插入、電池權重）、generic_cbba_node_plan.md（通用節點）、
                            protocol_v2_notice.md（協定版本 2 給狗端與規格作者的變更通知）
docker/                     模擬環境（sim、uavN、gcs 容器；mesh_net 可用 mesh_tc.sh 加掉包）
                            gz/：Gazebo 場景 fire_site.sdf（火警 tag 101、誘餌 tag 7、巡檢點）、patrol_site.sdf（3 綠＋5 紅，
                            由 gen_patrol_site.py 從 patrol_site_points.csv 產生）與 tag 0～7、101、mono_cam 模型
ros2_ws/src/
├── swarm_interfaces/       ROS 訊息（Task、CBBAMessage、Bid、AgentStamp）
├── cbba_core/              CBBA 核心、機間封包、協定層、通用 cbba_node（無人機和狗共用、不依賴 PX4）、離線模擬、測試；
│                           狗的文件在 doc/ugv_test.md、doc/ugv_tuning.md
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
    --packages-select swarm_interfaces cbba_core px4_waypoint_node --cmake-args -DCMAKE_BUILD_TYPE=Release &&
  cd /out/build/cbba_core && for t in test_*; do ./$t; done'
```

- 掛載路徑要是 `/home/ncrl/CBBA_BT`（共用 install 裡的 px4_msgs 用絕對路徑）。
- 容器裡沒有 `colcon` 指令，用 `python3 -m colcon`。
- 編譯要零警告（`-Wall -Wextra -Wpedantic`）。
- 使用者自己在 uav 容器編譯：`colcon build --symlink-install --packages-select swarm_interfaces cbba_core px4_waypoint_node`。
- PX4 SITL 測試步驟：`cbba_core/doc/sitl_test.md`（`cbba_uav.sh`、`cbba_task.sh` 在 `docker/scripts`）。
- 參數說明：`cbba_core/doc/parameters.md`（全部參數、要互相一致的參數、已知限制）；出價公式的實驗在 `cbba_core/cbba_parameters.md`。
- 多節點整合測試：同一個容器用 `participate:=always` 起多個 `cbba_node`（各自 `-r __node:=uavN_cbba_node`；程序鎖用 `CBBA_LOCK_DIR` 指到暫存目錄；狗要接受時用 Python 假 BT 回覆 assignment_request），
  用 `ros2 topic pub` 代替 BT；要測轉接節點時對 `/uavN/fmu/out/*` 發假的 PX4 話題。設 `ROS_DOMAIN_ID` 避免干擾使用者的容器。
  **停節點要用 `pkill -f "__node:=uavN_cbba_node( |$)"`**，只 kill `ros2 run` 外層，節點還會繼續跑。

## cbba_core 的分層

| 層 | 檔案 | 說明 |
|---|---|---|
| ROS 節點 | `cbba_node.cpp`（通用，只吃 RobotState／ExecState／TaskResult／Task／AssignmentResponse；PX4 由 `px4_waypoint_node/px4_state_bridge` 轉接；程序鎖） | 接感測與 BT；不寫邏輯 |
| 指派流程 | `assignment_manager.*` | 正式指派：BT 接受、搶占、排隊、拒絕、逾時、分段量測；不依賴 ROS，用模擬網路測（`test_assignment`） |
| UDP | `udp_link.*` | multicast，綁 mesh 網卡（`MESH_IP`） |
| 協定層 | `cbba_comm.*` | CBBA ⇄ 封包；不依賴 ROS 與 socket，用模擬網路測（`test_comm`） |
| 封包格式 | `wire_io`、`wire_header`、`wire_agent_state`、`wire_cbba_state`、`wire_task_announce`、`wire_completion` | 只管位元組格式 |
| CBBA 核心 | `cbba_agent`、`scoring`、`energy_model`、`types` | 17 條消解規則、bundle、出價 |

## 機間通訊：協定版本 2（2026-10-09，使用者要求「每筆任務資料帶 task_id」）

整體說明在 `cbba_core/doc/protocol.md`，逐位元組格式在各 `wire_*.hpp` 開頭。**不相容於規格書**
（`cbba_core/doc/機間通訊封包規格_20261006.pdf`）：規格用 AGENT_STATE 的 z、y 陣列位置（uint8 任務編號）當全隊共用的
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
| 補發 | 摘要不同時補缺的 TASK_ANNOUNCE／TASK_CLOSE；**不參與出價的鄰居當成什麼都不知道，全部補**；每 5 s 保底重播一個任務 | 晚起飛、重開機、飛出範圍再回來 |
| 開機同步 | 摘要和任一鄰居相同才送出自己建立的任務；1.5 s 沒鄰居＝單獨；5 s 對不上就開始 | 查出 BT 重開機後重用 task_id（不送出、記 `rejected_local_tasks`） |
| 先到的 TASK_CLOSE | 任務還不認得時先暫存；執行機是自己也套用 | 補發時可能比任務先到；重開機後要知道自己完成過的任務 |
| 已結束的任務 | 一直記著，不回收 | 忘記的話，斷線的那台補發時會把它加回來 |
| 完成確認 | 要確認的 = 宣告當下存活名單 ∩ 現在還活著的（機號清單，狗也算） | 宣告後失聯的不用等 10 次重送 |
| 失聯 | s 超過 1.5 s 沒更新就釋放它的任務；變成不參與時立即釋放 | 規格的 1.5 s |
| 非同步 CBBA | 只重送目前狀態 | 規格是同步式，「重送上一步」對非同步沒意義 |
| 正式指派版本（2026-10-09） | 每個任務一個 uint64 版本、全隊共用：**請 BT 接受時預先分配**（已知最新 + 1，狀態待接受），接受後同版本變保留或執行中；拒絕、逾時、撤銷（換掉、搶占、失敗、之後的 constraint）時 + 1（執行者 0、狀態無），作廢的版本不重用；放在 CBBA_STATE 紀錄裡，取（版本, 狀態, 執行者）較大的 | 區分每次正式指派；其他節點經 gossip 取得正式指派紀錄，才能驗證 UDP 的完成通知 |
| 回報檢查 | task_result 的 task_id、assignment_version 要是目前的正式指派；COMPLETION／TASK_CLOSE 的版本比已知的舊就拒絕（ACK status 2、不結束） | 擋掉任務換掉、X→Y→X 之後的舊回報 |
| 同時指派 | 同版本時已接受的勝過待接受的，同狀態才比機號 | 剛加入、還不知道鎖定的那台不能蓋掉已接受的 |
| 鎖定（容量承諾） | 別台「保留」或「執行中」的任務不出價；持有者的核心不讓別台的得標資訊拿走它；待接受不鎖。撤銷、失敗 → 重新開放；完成、取消 → 永久關閉；**持有者失聯 → 照樣鎖著、只標記待確認**（失聯接手另做） | 2026-10-09 使用者確認 |
| BT 接受（`require_accept`，狗 true、無人機 false） | 排第一 0.6 s → ACTIVATE／RESERVE 請求（帶預先分配的版本）→ BT 接受才發 assigned_task；拒絕：暫時 → 撤回、BT 狀態改變時再評估，永久 → 不再出價；3 s 沒回覆當暫時拒絕、送 RELEASE、之後的回覆無效。false 只跳過握手 | 地面端保有最終的接受與中斷權 |
| 搶占與排隊 | 優先級：BT 本機安全處置 ＞ 火警（GROUND_INTERVENTION）＞ 巡檢；同級不搶占。搶占要 `preemptible` 且優先級較高；已接受的任務釘在路徑前面。執行中之後最多保留 1 個火警，要 now + 剩餘 + 前往 + 處置 ≤ 期限（沒期限 = 建立時刻 + `max_queue_wait` 120 s），剩餘時間不知道不排隊。故障或 ExecState 逾時：不接新任務、撤回還沒接受的出價 | Accept = F ∧ [Idle ∨ (P_j > P_k ∧ Preemptible)] |
| TASK_CLOSE 權限 | 完成：actor = origin = 執行者、版本不舊；取消：actor 是建立者或 `cancel_authorities`（全隊一樣）；不合的不套用、不轉送 | 完成由有效執行者回報；取消由建立者或授權管理端發起 |
| 失敗與關閉分開 | 執行失敗不送 TASK_CLOSE：撤銷指派、任務保留、自己不再出價，由 CBBA 重新分配 | 之後的 constraint 撤銷也走同一條路 |

## 通用 cbba_node 與 BT 的介面（2026-10-09 起，無人機和狗相同）

每台跑同一個 `cbba_core/cbba_node`，話題在 `/<ns>` 底下（無人機 `/uavN`、狗 `/v60`）。節點名稱啟動時用
`-r __node:=uavN_cbba_node`／`v60_cbba_node` 區分（`cbba_uav.sh` 會自動加，轉接節點、task_executor 也一樣）。

| 話題 | 方向 | 用法 |
|---|---|---|
| `/<ns>/robot_state`（RobotState v1.2） | → CBBA | 位置（map ENU）、電量（負值 = 不知道）、`flight_state_valid`／`armed`／`offboard`／`landed`（狗填 false）。`state_timeout` 1.5 s 內有更新才參與出價；**無人機另外要飛行狀態有效、已解鎖、沒有 landed**（`require_airborne`）。第一筆當返航點。無人機由 `px4_state_bridge` 從 PX4 轉換（NED → ENU、加出生點），PX4 位置有在更新就一直送（地面上也送），PX4 斷線時停送 |
| `/<ns>/new_task`（Task） | BT → CBBA | 新任務；task_id 建立者必須是自己，不能和已知的重複（BT 重開機後流水號要接續）；佇列深度 256。AIR_RECON 的 z 低於 `recon_altitude`（無人機 5 m）時改成巡檢高度 |
| `/<ns>/task_result`（TaskResult v1.3） | BT → CBBA | `success` true = 完成；false = 執行失敗，任務保留、交回競標池、自己不再接；`detail` 只記 log。**task_id 和 `assignment_version` 要是目前 assigned_task 上的**，否則拒絕、狀態不變。2026-10-09 前無人機用 Task 的 status |
| `/<ns>/cancel_task`（Task） | → CBBA | 取消任務（只看 task_id）；要是建立者或 `cancel_authorities` |
| `/<ns>/assigned_task`（Task v1.4） | CBBA → BT | 目前要執行的任務＝正式指派（reliable、transient_local）；BT 接受後才發，帶 `assignment_version`（uint64）；沒有任務時 task_id = 0、status = CANCELLED |
| `/<ns>/exec_state`（ExecState） | BT → CBBA | 執行狀態（0 閒置…3 不可中斷、5 故障）、`preemptible`、`estimated_remaining_time`、有無執行中／排隊中的任務（旗標＋id＋版本）；逾時用本機單調時間（1.5 s） |
| `/<ns>/assignment_request`、`assignment_response` | CBBA ⇄ BT | ACTIVATE（`preempt`）／RESERVE／RELEASE；回覆帶 task_id、版本、接受／拒絕、原因（暫時／永久） |

| 項目 | 做法 |
|---|---|
| `vehicle_type` | `uav`／`ugv`：AGENT_STATE 的載具種類、參與出價的條件，以及能量模型、`recon_altitude`、`ns` 的預設值 |
| 測協商 | `participate:=always`（取代原本的 `use_px4:=false`），位置與電量用參數 |
| 返航點 | 第一筆 robot_state，也就是地面的出生點（只用在估返航耗電） |
| AGENT_STATE 旗標 | robot_state 的真實值；只在 `flight_state_valid` 時送 armed、offboard、landed。節點在鄰居的旗標改變時印 log |
| 指派版本 | `assignment_version` 從 assignment_request／assigned_task 一路帶到 BT 的 task_result（task_executor、`cbba_task.sh done/fail` 會自動帶） |
| 程序鎖 | cbba_node 啟動時對 `<lock_dir>/agentN.lock` 加 flock，拿不到拒絕啟動，退出才釋放；不因任務狀態釋放，BT 等其他節點不碰。docker 共用主機 `/tmp/cbba_locks`（掛 `/run/cbba`） |
| 分段量測 | 每個開始執行的指派印「建立 → 排第一 → 請求 → 接受 → 發布 → BT 執行」；2026-10-09 量到約 0.8 s（assign_hold 0.6 s 佔大部分），**不能宣稱符合 300 ms** |

`cbba_core/doc/image.png` 是**機器狗端**的介面（RobotState、TaskResult、ManualOverride）；2026-10-09 起無人機也用 RobotState、TaskResult。

### 狗（2026-10-07 和使用者確認）

| 項目 | 做法 |
|---|---|
| 啟動 | `ros2 run cbba_core cbba_node --ros-args -r __node:=v60_cbba_node -p vehicle_type:=ugv -p agent_id:=50`；不用裝 px4_msgs |
| 訊息 | RobotState、TaskResult 照 image.png 加進 `swarm_interfaces`（v1.1）；v1.2 的 RobotState 多 4 個飛行狀態欄位，**狗端要重新編譯 swarm_interfaces** |
| 機號 | **50**（介面規格）。協定版本 2 起和其他機號沒有差別：無人機完成任務也等狗的確認 |
| 參與出價 | robot_state 1.5 s 內有更新。狗只接地面任務，等於「收到無人機的火點才出價」 |
| 建立任務 | 協定版本 2 起狗也能建立（`/v60/new_task`）；ManualOverride 只在狗的機內 |
| 回報 | `task_result` 要原樣帶回 `assignment_version`（v1.3）；變更通知在 `docs/protocol_v2_notice.md` |
| 能量模型 | 佔位值（0.1 %/m、0.05 %/s、1 m/s），待伯宇實測，見 `cbba_core/doc/ugv_tuning.md` |
| 測試資料 | `cbba_core/doc/ugv_test.md` 的 T1～T8（假無人機＋假 robot_state；2026-10-09 改通用節點後重跑 T1、T2、T5、T6、T8，結果相同） |

狗端測試發現（寫在 `ugv_test.md`）：新火點進來時路徑重排，**執行中的任務會被換掉**；排序不一定近的先
（狗走得慢、每個任務停 20 s，後面的成本被放大）。要不要加「執行中不換手」、改出價公式，待討論。
回報檢查 2026-10-09 已補：只接受目前 assigned_task 上的任務的回報。

## 分支（2026-10-10）

| 分支 | 內容 |
|---|---|
| `main` | 2026-10-10 起包含全部（通用 cbba_node、協定版本 2、BT 接受流程、uav_px4_bt、apriltag_fire_detector、模擬測試場景），以 `12e36bb` 為基礎直接 commit 目前的工作目錄 |
| `dog-integration` | 留在 `cdb1156`（舊的 `ugv_cbba`）當備份，之後不再用 |

- 兩個分支共用 `ros2_ws/build`、`install`：**切換分支後要重新編譯**，否則 install 裡是另一個分支的程式。
  `uav_cbba` 改名成 `cbba_core` 後，舊的 `install/uav_cbba`、`install/ugv_cbba` 不會自動刪掉。
- push 要用 GitHub 的 Personal Access Token（主機和容器都沒有存憑證）。
- 通用節點的規劃與決定見 `docs/generic_cbba_node_plan.md`；協定版本 2 見 `cbba_core/doc/protocol.md`。

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
- 「黑板」對我們是 `cbba_node` 的話題（`assigned_task` 等）；狗的 `robot_blackboard.hpp` 是狗的機內實作。
- 命名空間是 `/uavN`（多台），不是 `/uav`。
- 無人機 BT 和週計畫 XML 的差異（加 TakeOff、頂層 ReactiveFallback、HoverAndMonitor 有期限並回報 task_result、
  FlyToWaypoint 逾時回報失敗）寫在 `uav_px4_bt/include/uav_px4_bt/bt_nodes.hpp` 開頭。
- 300 ms 全流程：2026-10-09 量到指派約 0.8 s（`assign_hold` 0.6 s 佔大部分），目前達不到，要和伯宇、宜臻分預算。

### 前三週（無人機端）的進度：程式都寫好了，**目前在 Gazebo／PX4 SITL 做模擬測試**

| 週 | 項目 | 程式 | 模擬測試 |
|---|---|---|---|
| 1 | PX4 SITL、XRCE-DDS、起飛懸停 | `docker/`、`px4_waypoint_node/takeoff_hover` | 完成（2 台，`test/results/takeoff_hover/2uav_1003`） |
| 1 | 機間訊息、mesh＋`tc` | 協定版本 2、`mesh_tc.sh` | 協定版本 2 的單元測試＋真 UDP 多節點測試通過；`tc` 掉包還沒測 |
| 2 | 3D 航點、空中出價 | `task_executor`、`uav_px4_bt` 的 FlyToWaypoint（CPF） | task_executor 的第一階段 SITL 完成（協定版本 1）；**BT 版本、協定版本 2 還在測** |
| 2 | 混合任務只對專長出價 | `types.hpp` 能力檢查 | 真 UDP 測試正確（狗 T1～T8）；SITL 裡 5 個混合任務還沒跑 |
| 3 | AprilTag 火情確認 | `apriltag_fire_detector`、`docker/gz/worlds/fire_site.sdf` | **測試中**：要確認 tag 101 觸發、誘餌 tag 7 不觸發 |
| 3 | `ReportFireEvent` 注入地面任務 | `uav_px4_bt`（同一個 tag 只報一次） | **測試中**：要確認 `/uavN/new_task` 發出、狗（或假狗）得標 |
| 2～3 | 巡檢＋隨機插入、電池權重（`docs/sim_scenario_plan.md`） | `patrol_site`、`patrol_scenario.sh`（terminal 輸入編號插入）、`mission_sim`＋`cbba_sim insert` | 離線 100 次×5 權重已跑（2026-10-10）；**Gazebo 抽驗還沒跑** |

## 目前狀態（2026-10-09）

- 完成：封包格式、核心補強（停止出價、釋放、失聯、排除失敗任務）、協定層、UDP、cbba_node。
  108 項單元測試通過（協定版本 1）；3 個節點走 UDP 的整合測試（分配、完成、失敗交回、墜毀重分配）正確。
- 2026-10-09 改成通用 cbba_node：4 個節點走真 UDP 的整合測試正確（狗 T1、T2、T5、T6、T8、只關 PX4 時停止出價並由別台接手）。
- 2026-10-09 協定版本 2：95 項單元測試通過、零警告（test_comm 連跑 20 次都過）；4 個節點走真 UDP 的整合測試正確
  （真實旗標跟著 PX4 變、狀態逾時標無效、uav2 和狗 50 同時建立任務、回報檢查、改派後舊執行者被拒、130 個任務分批）。
- 2026-10-09 正式指派版本、TASK_CLOSE 權限、失敗與關閉分開：101 項單元測試、零警告（test_comm 連跑 20 次都過）；
  真 UDP 整合測試通過（正常完成、錯誤回報者、X→Y→X 的舊回報、合法／非法取消、失敗後以新版本重新分配）。
- 2026-10-09 程序鎖、BT 接受／搶占／排隊、鎖定（容量承諾）：128 項單元測試、零警告（test_comm、test_assignment 各連跑 20 次都過）；
  真 UDP＋假狗 BT 的整合測試 R1～R7 全過（閒置、巡檢中搶占、處理火警時排隊與升級、不可中斷等到安全點、暫時拒絕、逾時、故障）。
  測試中修掉的：持有者被剛加入（不知道鎖定）的那台用共識拿走；BT 變故障時逾時撤回的任務又被恢復請求。
  session 層只定義介面（`session_registry.hpp`），還沒接上。
  **還沒在 SITL 跑**，共用工作區也還沒重新編譯（swarm_interfaces v1.4，狗端也要重新編譯，而且狗的 BT 要實作握手）。
- 進行中：PX4 SITL＋Gazebo 測試（`sitl_test.md`）。2026-10-09 起有備用 BT（`uav_px4_bt`，`cbba_uav.sh` 預設 `bt` 模式，
  搭配 `apriltag_fire_detector`）；`px4_waypoint_node/task_executor` 保留作簡易執行器。下面的第一階段結果是用 task_executor 跑的。
  **第一階段 SITL 驗證完成**：分配、完成確認（20 ms 內傳到全隊）、失敗交回（0.6 s 接手）、
  墜毀重分配（1.5 s 判定、2 s 接手）、晚加入（20 ms 補齊）、重開機（seq 接續）。掉包、斷網的測試之後再做。
  （以上是協定版本 1、2026-10-07 的結果）
  SITL 的電量對齊（2026-10-07）：PX4 模擬電池續航 900 s、最低 20%；能量模型改成懸停 0.111 %/s、
  每公尺 0.022 %（`cbba_uav.sh` 傳入，核心預設 0.5／0.2 不變）；速度 5 m/s、巡檢高度 5 m 兩邊一致。
  之前 PX4 預設 60 s 就降到 50%，加上 0.5 %/m，遠一點、停留久的任務會被誤判成電量不夠。
- 尚未：docker 的 Fast DDS 仍讓 DDS 經過 mesh（要改 `gen_compose.py` 讓 uav 容器只走線材網卡）；
  AGENT_STATE 的 progress 與「跟隨中」旗標要等 BT；
  移動時的避碰（之後在飛行層加 CPF 之類的機制，和 CBBA 無關；目前 task_executor 同高度直線飛，SITL 會撞機）。
  機器狗端：通用 cbba_node（`vehicle_type:=ugv`）走真 UDP 的整合測試正確；還沒在真的狗上測。
  `cbba_task.sh ground` 建立地面處置任務。
- 模擬測試環境（3 綠＋5 紅方塊、AprilTag、電池權重 100 次，`docs/sim_scenario_plan.md`）：2026-10-10 使用者決定
  terminal 輸入編號插入、離線統計＋Gazebo 抽驗、5 個全插隨機時刻、權重 0／1／5／20／100、各台電量不同。已做：
  `patrol_site` 場景、`patrol_scenario.sh`（在隔離的 Gazebo 測過生成、刪除 tag、重放）、`cbba_core/mission_sim`
  （真的 CbbaComm＋AssignmentManager＋假 BT＋PX4 式依時間耗電；`test_mission_sim` 8 項）、`cbba_sim insert`。
  第一次結果：800 個任務全部完成，接錯（撤銷）21 → 15 隨權重下降；場地對續航太小，要更緊的電池設定才分得出四類，
  **值待使用者決定**。PX4 的模擬電池不能設初始電量，Gazebo 只能用不同的 `SIM_BAT_DRAIN`。
- 要和規格作者說明：**協定版本 2 不相容於規格書**（task_id 取代 uint8 任務編號、20 B 表頭、session_id、大端序、
  AGENT_STATE／CBBA_STATE 分開）；另外原本就要確認的：座標系（我們用 map ENU）、拒絕確認的處理、重送 10 次後的處理、
  重開機（FORMATION 等其他模組和狗端也會遇到，最好由規格統一規定）。
- 下一步（使用者 2026-10-09 的順序）：session 層（明確註冊、衝突偵測、序號檢查；mesh 形式與認證方式待確認），
  再加新版 CBBA 的其他 constraints（競標前、正式指派前、執行期間；撤銷 = 新版本，回報與權限檢查沿用）。
- 待決定：取消的「授權管理端」是誰（地面站要不要跑 cbba_node）；失聯接手機制；`assign_hold` 依量測調整（要驗證不震盪）；
  300 ms 預算怎麼分（和伯宇、宜臻）。

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
