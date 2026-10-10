# 第 4～6 週：週計畫和目前程式的差異

2026-10-10 · 無人機端（勝翔）的觀點。週計畫全文在 `CLAUDE.md` 的「週計畫」；啟動方式在 `docs/sim_launch_guide.md`。
狀態的依據：程式碼、單元測試（2026-10-10 重新編譯，5 個套件零警告、164 項通過）、`cbba_core/doc/protocol.md` 的整合測試紀錄。

圖例：✅ 有程式且測過　🟡 有程式、還沒在 SITL／整合環境驗證　❌ 還沒有　🔀 做法和週計畫不同

---

## 第 4 週：Phase 1 空地即時中斷與重規劃

週計畫流程：狗巡檢中 → 無人機飛往火情區 → AprilTag 成立 → 廣播 GROUND_INTERVENTION → 無人機出價 0、狗得標
→ 狗寫入黑板 → 狗的 BT 哨兵 50 ms 內中斷 Nav2、掉頭。

| 環節 | 週計畫 | 目前 | 狀態 |
|---|---|---|---|
| 無人機飛往火情區 | PX4 航點 | `uav_px4_bt` 的 FlyToWaypoint，CPF 避碰（`test_cpf` 9 項） | 🟡 |
| AprilTag 成立 | 信心度 > 0.85 | `apriltag_fire_detector`：confidence = 最近 1 s 影像中看到的比例；BT 懸停穩定後觀察、≥ 0.85 成立（`test_fire_detector`、`test_tree`） | 🟡 `fire_site` 還沒在 SITL 跑 |
| 廣播地面任務 | 「向全網 Mesh 注入」 | BT 對 `/uavN/new_task` 發 GROUND_INTERVENTION（value 100、停留 20 s、期限 120 s），cbba_node 用 UDP 的 TASK_ANNOUNCE 轉送 | 🟡 |
| 無人機出價 0 | 可行性檢查 | `types.hpp` 的 `isCompatible`：無人機只做 AIR_RECON | ✅ |
| 狗得標 | 地面出價最高 | 通用 cbba_node（`vehicle_type:=ugv`）；狗的能量模型是佔位值（`ugv_tuning.md`） | ✅ 真 UDP＋假狗 BT 的整合測試 |
| 狗中斷舊任務 | 「寫入黑板 → 哨兵 50 ms 中斷」 | 🔀 **不是直接覆寫黑板**：cbba_node 先送 `assignment_request`（ACTIVATE，`preempt` = 要中斷），狗的 BT 停掉舊的、交接完回覆接受，cbba_node 才發 `assigned_task`。搶占只在「可以安全中斷、優先級較高」時（火警 ＞ 巡檢）。R1～R7 整合測試通過（`test_assignment` 13 項） | ✅ CBBA 端；50 ms 的中斷是伯宇的 BT |
| Gazebo 同一個世界 | V60＋PX4 | 我們的 sim 容器只有 PX4；狗的模型、Nav2 在伯宇那邊 | ❌ 要決定怎麼接（見最後） |

**量化指標**

| 指標 | 週計畫 | 目前 | 差異 |
|---|---|---|---|
| 中斷時延 | 黑板覆寫 → `cancel_goal` ≤ 50 ms | 狗的 BT 才量得到。對應到我們的介面是「收到 `assignment_request`（preempt）→ `cancel_goal`」 | 量測起點要和伯宇重新定義 |
| 全流程 | 無人機發布火情 → 狗開始轉向 ≤ 300 ms | cbba_node 有分段量測：2026-10-09 量到**建立 → BT 執行約 790～870 ms**，其中 `assign_hold` 600 ms（另外：UDP＋共識約 15 ms、假 BT 回覆 64～134 ms、ExecState 10 Hz 約 105 ms） | ❌ **目前不能宣稱符合**。`assign_hold` 是為了擋掉協商途中的來回換手，調低要先驗證不會震盪；300 ms 怎麼分給 CBBA／BT／Nav2 要和伯宇、宜臻談 |
| 運動平滑度 | cmd_vel 平滑、無 jerk | 狗端 | — |

還要注意：火情的「判定」本身要時間（懸停穩定 `settle_sec`＋觀察 `observe_sec`＋confidence 的 1 s 視窗），
週計畫的 300 ms 從「發布火情」開始算，不含這段。

---

## 第 5 週：Phase 2 弱網 CBBA 收斂壓測

| 項目 | 週計畫 | 目前 | 狀態 |
|---|---|---|---|
| tc 網橋 | 掉包 0／10／30／50／70 %、延遲 50±30 ms | `mesh_tc.sh set <掉包> [延遲] [抖動]`，每台 uav（和 gcs）都要設 | ✅ 指令有；🟡 還沒在 SITL 跑過梯度 |
| tc 只影響機間 | — | **Fast DDS 也走 mesh**：PX4 的 DDS 話題也會被 tc 影響，結果會受干擾 | ❌ 要改 `gen_compose.py` 讓 DDS 只走線材網卡（這週一定要先修） |
| 注入多源任務 | 無人機火警＋地面巡檢點 | 無人機：`patrol_scenario.sh`（G／R 點）、BT 的火警；狗也能建立任務（`/v60/new_task`）；`cbba_task.sh ground` 建地面任務 | ✅ |
| 收斂率（5 s、30 % 掉包 ≥ 95 %） | 各掉包率統計 | 兩套離線測試，**都還不是正式數據**：(1) `test_lossy_network`／`cbba_sim sweep`：200 次、0～70 %，但用的是**舊的核心層訊息**（不是協定版本 2 的封包）；(2) `test_comm` 的 `ConvergesUnderLoss`：協定版本 2 的封包，但只有 10 次、10／30 %，只檢查 5 s 後一致 | 🔀 要用協定版本 2 重做統計（可以接在 `mission_sim` 或測試的 `Swarm` 上） |
| 收斂時間、頻寬曲線 | 繪圖 | 離線：舊的 sweep 有收斂時間、估計流量。實機：cbba_node 只在「變成收斂」時印一行 log，`CommStats` 有各封包的送出位元組，但**沒有輸出** | ❌ 要加：定期把收斂時間、`CommStats` 寫成 CSV（或發一個 ROS 話題），再寫畫圖腳本 |
| 連鎖退標（Cascade Release） | 監控高掉包下不重複認領、不遺失 | 核心有（`test_agent` 的 `CascadeReleaseWhenOutbidInTheMiddle`）；`Swarm::consistent()` 檢查不重複持有 | ✅ 單元測試；🟡 整合 |
| 100 % 斷網 → 單機貪婪 | 安全完成手頭任務 | 失聯 1.5 s 後釋放失聯者「沒鎖定」的任務，各自重新出價（`TotalBlackoutFallsBackToLocalGreedy`：會重複認領，這是預期）。🔀 **已接受（保留／執行中）的任務照樣鎖著、只標記待確認，不會被接手**（失聯接手機制待決定）；手上執行中的任務照常做完；完成確認只等「宣告時存活、現在還活著」的鄰居，失聯的不用等，斷線的那台之後由補發（TASK_CLOSE）得知 | 🟡 |
| A→B→C 鏈狀拓撲 | （情境需要時） | 協定支援多跳轉送（`MultiHopRelayAndCompletion`）；docker 裡依來源 IP 擋封包的腳本還沒寫 | ❌ |

---

## 第 6 週：展示、數據結案、部署準備

| 項目 | 週計畫 | 目前 | 狀態 |
|---|---|---|---|
| 一鍵啟動 | `swarm_inspection_gazebo.launch.py` | 🔀 用 shell 腳本：`start.sh`、`cbba_uav.sh`（每台一次開 Agent＋BT＋偵測＋轉接＋cbba_node）、`px4_sitl.sh`、`gz_bridge.sh`、`patrol_scenario.sh`。跨容器（sim、uavN）所以不是單一 launch 檔 | 🟡 可以補一個 tmux 腳本一次開好 |
| 數據報表 | 中斷延遲、收斂輪數、丟包容錯 | 有：分段量測 log、`cbba_sim insert`（電池權重，CSV）、舊的 `cbba_sim sweep`、`plot_results.py`（舊格式）。缺：協定版本 2 的收斂統計、實機的收斂／頻寬紀錄、insert 結果的畫圖 | 🟡 |
| 容器化 | Docker | ✅ `docker/`（sim、uavN、gcs，mesh_net＋線材網路） | ✅ |
| Jetson Orin 部署 | 交叉編譯 | 沒有 arm64 映像、沒試過；cbba_core 不依賴 PX4、只用標準 C++17＋rclcpp，移植風險低。實機的 mesh 網卡、chrony 對時、`MESH_IP` 要設定（`docker/README.md`「換成實機」） | ❌ |

---

## 和週計畫不同、要對外說明的地方（彙整）

| 項目 | 週計畫 | 目前 | 原因 |
|---|---|---|---|
| 指派給 BT | CBBA 寫黑板，BT 哨兵偵測覆寫 | 先請 BT 接受（`assignment_request`），接受後才發 `assigned_task`（狗 `require_accept` true、無人機 false） | 地面端保有最終的接受與中斷權（2026-10-09 確認） |
| 300 ms | 全流程 ≤ 300 ms | 量到約 0.8 s | `assign_hold` 0.6 s 擋換手震盪；預算待分 |
| 機間訊息 | `CBBAMessage.msg`（y、z 向量）、18 條規則 | 協定版本 2（UDP、task_id、session）、17 條規則 | 見 `protocol_v2_notice.md` |
| mesh 節點 | `mesh_bridge_node` | 沒有獨立節點：cbba_node 直接走 UDP multicast | DDS 只在機內 |
| 黑板 | `robot_blackboard.hpp`（共同凍結） | 無人機和狗的 cbba_node 用同一組話題（`robot_state`、`assigned_task`、`task_result`、`exec_state`、`assignment_*`） | 黑板是各自 BT 的內部實作 |
| 斷網退化 | 單機貪婪、安全完成手頭任務 | 未鎖定的任務各自貪婪；**已接受的任務照樣鎖著** | 避免兩台同時做同一個已接受的任務；失聯接手待決定 |
| 一鍵啟動 | launch.py | shell 腳本（跨容器） | 模擬環境是多容器 |

## 建議接下來的順序

1. **修 DDS 走 mesh 的問題**（`gen_compose.py`）：不修的話第 5 週的 tc 數據不可信。
2. **SITL 跑通第 3 週**：`fire_site`（tag 101 觸發、tag 7 不觸發）＋ gcs 的假狗接到 GROUND_INTERVENTION（`sim_launch_guide.md` B）。
3. **第 5 週的量測工具**：協定版本 2 的離線收斂統計（0～70 %、100 次以上），cbba_node 定期輸出收斂時間與 `CommStats`，畫圖腳本。
4. **和伯宇、宜臻對齊**：50 ms 的量測起點（`assignment_request` 收到時）、300 ms 預算與 `assign_hold`、狗在 Gazebo 怎麼和我們的 sim 容器接
   （同一個 Gazebo 世界，還是各自模擬、只透過 mesh 連 cbba_node）。
5. 決定失聯接手機制、取消的授權管理端（gcs 要不要跑 cbba_node）。
