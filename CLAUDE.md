# CLAUDE.md

給在這個 repo 工作的 Claude 的說明。使用者負責**無人機端**（PX4、BT、AprilTag、空中 CBBA、mesh＋tc）；
機器狗（Ghost V60）端由隊友負責。回覆與文件一律用繁體中文。

## 專案

空地異質系統的無人機端：PX4 旋翼機以 CBBA 分散式拍賣分配任務，交給行為樹（BT）執行。

```
docker/                     模擬環境（sim、uavN、gcs 容器；mesh_net 可用 mesh_tc.sh 加掉包）
ros2_ws/src/
├── swarm_interfaces/       ROS 訊息（Task、CBBAMessage、Bid、AgentStamp）
├── uav_cbba/               CBBA 核心、機間封包、協定層、cbba_node、離線模擬、測試
├── px4_waypoint_node/      PX4 offboard：takeoff_hover 起飛懸停、task_executor 代替 BT
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
    --packages-select swarm_interfaces uav_cbba --cmake-args -DCMAKE_BUILD_TYPE=Release &&
  cd /out/build/uav_cbba && for t in test_*; do ./$t; done'
```

- 掛載路徑要是 `/home/ncrl/CBBA_BT`（共用 install 裡的 px4_msgs 用絕對路徑）。
- 容器裡沒有 `colcon` 指令，用 `python3 -m colcon`。
- 編譯要零警告（`-Wall -Wextra -Wpedantic`）。
- 使用者自己在 uav 容器編譯：`colcon build --symlink-install --packages-select swarm_interfaces uav_cbba`。
- PX4 SITL 測試步驟：`uav_cbba/doc/sitl_test.md`（`cbba_uav.sh`、`cbba_task.sh` 在 `docker/scripts`）。
- 參數說明：`uav_cbba/doc/parameters.md`（全部參數、要互相一致的參數、已知限制）；出價公式的實驗在 `uav_cbba/cbba_parameters.md`。
- 多節點整合測試：同一個容器用 `use_px4:=false` 起多個 `cbba_node`，用 `ros2 topic pub` 代替 BT。
  **停節點要用 `pkill -f "cbba_node.*agent_id:=N"`**，只 kill `ros2 run` 外層，節點還會繼續跑。

## uav_cbba 的分層

| 層 | 檔案 | 說明 |
|---|---|---|
| ROS 節點 | `src/cbba_node.cpp` | 接 PX4 與 BT；不寫邏輯 |
| UDP | `udp_link.*` | multicast，綁 mesh 網卡（`MESH_IP`） |
| 協定層 | `cbba_comm.*` | CBBA ⇄ 封包；不依賴 ROS 與 socket，用模擬網路測（`test_comm`） |
| 封包格式 | `wire_io`、`wire_header`、`wire_agent_state`、`wire_task_event`、`wire_completion` | 只管位元組格式 |
| CBBA 核心 | `cbba_agent`、`scoring`、`energy_model`、`types` | 17 條消解規則、bundle、出價 |

## 機間通訊：依「機間通訊封包規格」2026-10-06

規格書在 `uav_cbba/doc/機間通訊封包規格_20261006.pdf`。CBBA 只用：共用表頭、轉送表頭、AGENT_STATE、
TASK_EVENT、COMPLETION、COMPLETION_ACK。**DDS 只用在機內**（PX4、BT），機間全走 UDP。

原則：**格式照規格書；規格沒寫或不符合我們情況的，以我們的為主，並在程式註解寫明原因。**
已定案的決定（改之前先和使用者確認）：

| 項目 | 做法 | 原因 |
|---|---|---|
| 去重複的鍵 | （封包種類, origin_id, origin_seq） | 規格的 seq 每種封包各自遞增，只看 origin 會撞號（TASK_EVENT 第 1 則和 COMPLETION 第 1 則） |
| 重送 | **一律換新序號**；確認者「原樣回送」指決定不變 | 沿用舊序號時，多跳下第一次轉送掉了就會被轉送者當重複擋掉，永遠補不回來 |
| agent_id、origin_id = 0 | 拒收 | 0 保留為「無人」 |
| seq 起點 | **區塊預約**：上限存在 `seq_file`（預設 `~/.cbba/seq_uavN`），先寫入上限 +10000 才使用；開機從上次的上限之後開始；沒有紀錄時用系統時鐘 ms 當起點。`SeqFilter` 用循環比較（RFC 1982） | 從 0 開始的話重開機後 (origin_id, origin_seq) 和開機前一樣，被鄰居去重複（記 60 s）當舊的丟掉；多跳時遠端看不出重開機。只用時鐘的話，樹莓派沒有 RTC、還沒對時時會往回跳（2026-10-07） |
| 重開機 | `SeqFilter`：3 s 沒聽到某台某種封包就接受較小的 seq（時鐘異常時的備援） | 規格沒寫 |
| TASK_EVENT | 版本 2（type 位元組 0x44）尾端 +7 B：type 1、**絕對截止時刻** 4（ms，約 49.7 天循環）、duration 2 | 出價需要；絕對時刻讓轉送／補發原封不動，期限不會被往後推 |
| 任務編號 | uint8，各機自編、依機號交錯：(k−1) + 8j | 沒有 leader；交錯讓 M 不會暴增 |
| task_id | ROS／核心用 uint32（機號 × 65536 + 流水號），寫成 8 位十六進位放 TASK_EVENT 名稱欄 | 各機還原出同一個 task_id |
| AGENT_STATE y | 不認得或已結束填 **−1** | z = 0 有兩種意思，不分開會依規則 16 洗掉別人的得標者 |
| round_key | 已知任務與狀態的 CRC-32 摘要 | 不同就逐格補發 |
| exchange_step | 本機分配表變動次數 | 我們是非同步 CBBA，沒有「步」 |
| s | 系統時鐘 ms 低 32 位元 | 需要各機對時（chrony） |
| 補發 | 摘要不同時逐格補缺的 TASK_EVENT／證明；**不參與出價的鄰居當成什麼都不知道，全部補**；每 5 s 保底重播一個任務 | 規格沒處理晚起飛、重開機、飛出範圍再回來；地面上的飛機沒有附 y（2026-10-07 前只能等保底重播） |
| 開機同步 | 摘要和任一鄰居相同才自編任務編號；1.5 s 沒聽到鄰居＝單獨一台；有鄰居但 5 s 對不上就開始，跳過鄰居 M 以下的編號 | 先追上再上線，避免重開機後撞號 |
| 先到的證明 | 任務還不認得時先暫存 | 補發時證明可能比任務先到 |
| 完成確認 | 要確認的 = 宣告當下存活名單 ∩ 現在還活著的 | 宣告後失聯的不用等 10 次重送 |
| 失聯 | s 超過 1.5 s 沒更新就釋放它的任務；變成不參與時立即釋放 | 規格的 1.5 s |
| 參與出價 | 預設解鎖且離地；不參與時 M=N=Lt=0（34 B） | 沒起飛的不搶任務 |
| 非同步 CBBA | 只重送目前狀態 | 規格是同步式，「重送上一步」對非同步沒意義 |

## cbba_node 與 BT 的介面（只用現有的 `swarm_interfaces/Task`）

| 話題 | 方向 | 用法 |
|---|---|---|
| `/uavN/new_task` | BT → CBBA | 新任務；task_id 建立者必須是自己。AIR_RECON 的 z 低於 `recon_altitude`（5 m）時改成巡檢高度 |
| `/uavN/task_result` | BT → CBBA | `status` DONE = 完成；CANCELLED = 失敗，交回競標池、自己不再接 |
| `/uavN/assigned_task` | CBBA → BT | 目前要執行的任務（reliable、transient_local）；同一任務連續 0.6 s 排第一才發；沒有任務時 task_id = 0、status = CANCELLED |
| `/uavN/fmu/out/*` | PX4 → CBBA | 位置（NED → map ENU，加出生點）、電量、armed、是否離地 |

`uav_cbba/doc/image.png` 是**機器狗端**的介面（RobotState、TaskResult、ManualOverride），不是無人機用的。

## 目前狀態（2026-10-07）

- 完成：封包格式、核心補強（停止出價、釋放、失聯、排除失敗任務）、協定層、UDP、cbba_node。
  106 項單元測試通過；3 個節點走 UDP 的整合測試（分配、完成、失敗交回、墜毀重分配）正確。
- 共用工作區已編出 cbba_node（使用者 2026-10-07 在容器裡 colcon build 完成）。
- 進行中：PX4 SITL＋Gazebo 測試（`sitl_test.md`）。BT 還沒好，先用 `px4_waypoint_node/task_executor` 代替
  （起飛、飛到指派的任務、停留 duration 後回報 DONE）。
  **第一階段 SITL 驗證完成**：分配、完成確認（20 ms 內傳到全隊）、失敗交回（0.6 s 接手）、
  墜毀重分配（1.5 s 判定、2 s 接手）、晚加入（20 ms 補齊）、重開機（seq 接續）。掉包、斷網的測試之後再做。
  SITL 的電量對齊（2026-10-07）：PX4 模擬電池續航 900 s、最低 20%；能量模型改成懸停 0.111 %/s、
  每公尺 0.022 %（`cbba_uav.sh` 傳入，核心預設 0.5／0.2 不變）；速度 5 m/s、巡檢高度 5 m 兩邊一致。
  之前 PX4 預設 60 s 就降到 50%，加上 0.5 %/m，遠一點、停留久的任務會被誤判成電量不夠。
- 尚未：改動未 commit；
  docker 的 Fast DDS 仍讓 DDS 經過 mesh（要改 `gen_compose.py` 讓 uav 容器只走線材網卡）；
  AGENT_STATE 的 progress 與「跟隨中」旗標要等 BT；
  移動時的避碰（之後在飛行層加 CPF 之類的機制，和 CBBA 無關；目前 task_executor 同高度直線飛，SITL 會撞機）。
  每台只能建立約 32 個任務（任務編號 uint8、依機號交錯、已完成的不回收）。
  機器狗端：要另寫 ugv 版節點（共用協定層）；狗在 ROS 的機號 50 要對應到網路上的 1～8，CBBA → BT 的話題待確認。
- 要和規格作者確認：去重複鍵含封包種類、重送換新序號、座標系（我們用 map ENU）、拒絕確認的處理、
  重送 10 次後的處理、證明的 K 含不含執行機、**seq 起點與重開機**（規格只寫「遞增」，
  FORMATION 等其他模組和狗端也會遇到重開機撞號，最好由規格統一規定）。

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
| namespace | 核心與協定層在 `uav_cbba`，封包格式在 `uav_cbba::wire`；檔案內的輔助函式放匿名 namespace |
| 短函式 | 一行的 getter 寫在標頭：`AgentId id() const {return id_;}`（大括號內不留空白） |
| 錯誤處理 | 解析、查詢失敗回傳 `std::optional`／`bool`，不丟例外；建構時參數不合法才丟 `std::invalid_argument`／`std::runtime_error` |
| 註解 | 繁體中文；寫「為什麼」，不重述程式碼；規格沒寫或不照規格的地方一定要註明原因；長檔案用 `// ===` 分段 |
| 測試 | gtest，`TEST(類別, 行為)`；每個測試開頭一行註解寫情境；整合測試走真的 UDP |
| 分層 | 協定層、封包格式、核心不依賴 ROS、socket、檔案；需要時由節點層用參數或 `std::function` 注入（例如 seq 紀錄） |

## 工作習慣

- 使用者指定輸出位置時直接寫到那裡，不另存副本。
- 大改、刪除前先說明影響；刪除未 commit 的檔案前先備份。
- 先做格式與測試，再做邏輯；每一層都要有測試，整合測試走真的 UDP。
