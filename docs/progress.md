# 目前進度（無人機端）

最後更新：2026-10-12 · 勝翔 · 這份會持續更新；某一天的快照另外存在 `reports/`（例如 `reports/progress_20261011.pdf`）。

狀態：**SITL** 在 PX4 SITL＋Gazebo 實測通過　**測試** 單元或真 UDP 整合測試通過、還沒上 SITL　
**進行中**　**未達成**　**超前**＝週計畫之後的週次或週計畫沒有的項目　**狗端**＝要等伯宇

週計畫：2026-10-05 ～ 11-13（5 週開發＋1 週緩衝），現在是第 2 週開頭；第 1～3 週的無人機端項目都已在 SITL 驗證，
第 4～6 週有一部分已經先做。

---

## 一、週計畫對照

| 週 | 週計畫項目 | 目前 | 驗證 | 狀態 |
|---|---|---|---|---|
| 1 | PX4 SITL＋XRCE-DDS、起飛懸停 | 多台 SITL（docker：sim、uavN、gcs），`px4_sitl.sh` 自動加入同一個場景 | itest U1（3 台起飛） | SITL |
| 1 | CBBA 通訊訊息 | 協定版本 2（UDP，每筆帶 task_id、session、正式指派版本），取代週計畫的 `CBBAMessage` | 單元、真 UDP、itest comm | SITL |
| 1 | mesh＋`tc` 弱網環境 | `mesh_tc.sh`；**DDS 只在機內**（`.env` 的 `DDS_OVER_MESH=0`，10/12），mesh 上只有 CBBA 的 UDP | itest M：待命 69 → **3.5 kB/s** | SITL |
| 2 | PX4 3D 航點巡邏 | 備用 BT `uav_px4_bt`：offboard 速度控制＋CPF 避碰 | itest U2、U5 | SITL |
| 2 | 空中出價（地面任務 0） | 統一版 cbba_node 的相容表（無人機只接 AIR_RECON） | itest、G2 T1、DEMO3 | SITL |
| 2 | 飛完 3 個點；5 個混合任務各自只對專長出價 | 三台分配三個任務；混合任務用真 UDP＋假狗 | itest U5、G2 | SITL |
| 3 | AprilTag 火情確認（> 0.85） | `apriltag_fire_detector`＋BT 的 `DetectAprilTagFire`；誘餌 tag 7 不觸發 | itest U3（到達後 1.2～1.9 s、誤差 0.28～0.41 m）、U4 | SITL |
| 3 | `ReportFireEvent` 注入地面任務 | BT 建立 GROUND_INTERVENTION，cbba_node 廣播；無人機不出價、假狗得標 | itest U3、G2 | SITL |
| 4 | 火警 → 狗得標、中斷、掉頭 | CBBA 端完成；介面已照伯宇的格式（`swarm_cbba`、v1.6），他的 BT 不用改 | G2 58 項（伯宇的 T1～T10＋DEMO3） | 測試／狗端 |
| 4 | 黑板覆寫 → cancel ≤ 50 ms | 狗的 BT 才量得到 | 伯宇 G3：0.1～0.3 ms | 狗端 |
| 4 | 火情發布 → 狗開始轉向 ≤ 300 ms | 指派約 0.4 s（`assign_hold` 0.4 s 佔大部分） | 分段量測 | 未達成 |
| 4 | Gazebo 兩種載具同一個世界 | sim 容器只有 PX4，狗的模型在伯宇那邊 | — | 狗端 |
| 5 | `tc` 掉包 0／10／30／50／70%、延遲 50±30 ms，注入多源任務 | `itest.py loss`：每個掉包率 20 次、每次三台各建一個任務 | 見「二、測試結果」 | SITL |
| 5 | 5 s 內收斂率（30% 掉包 ≥ 95%） | **30%：95%（19／20），中位數 0.65 s** | itest loss L30 PASS | SITL（剛好達標） |
| 5 | 掉包率對收斂時間、頻寬曲線 | `plot_loss.py` 畫圖；頻寬 6 → 51 kB/s（0 → 50%） | itest loss | SITL |
| 5 | 100% 斷網 | 斷網時只有建立者拿到；**失聯接手**（火警 10 s、巡檢 30 s）10/12 實作 | itest C8、C5 | SITL |
| 5 | 連鎖退標、不重複認領 | 單元測試；itest 檢查不重複 | C1、C7、loss | SITL |
| 6 | 一鍵啟動 | `cbba_uav.sh`、`docker/itest`（comm、uav、loss、demo） | — | 可用 |
| 6 | 容器化 | docker 環境 | — | 完成 |
| 6 | Jetson Orin 部署 | 還沒開始 | — | 待做 |
| 6 | 數據報表、展示影片 | `itest.py demo`（Gazebo 錄影＋俯視動畫 MP4）做好，**Gazebo 視窗開不起來，查原因中** | — | 進行中 |

週計畫和實作不同的地方（要對外說明）：機間通訊走 UDP 的協定版本 2（沒有 `mesh_bridge_node`、`CBBAMessage`）；
消解規則實作 17 條；「黑板」對我們是 cbba_node 的話題；命名空間是 `/uavN`；300 ms 目前達不到。

---

## 二、最近的測試結果（2026-10-12，docker 環境，3 台）

### 單元測試：174 項全過、零警告

| 套件 | 項數 |
|---|---|
| `cbba_core` | 143（含失聯接手 4 項；comm、assignment、lossy 各連跑 20 次） |
| `uav_px4_bt` | 25（含火警看位置去重、路過發現、目的地就是火點） |
| `apriltag_fire_detector` | 6 |

### 整合測試

| 測試 | 結果 | 重點 |
|---|---|---|
| itest comm | **9／9** | 指派 0.40 s、完成撤下 50 ms、失敗交回 0.45 s、停送狀態 2.0 s 接手、**C5 節點停止 32.0 s 接手（巡檢 30 s 規則）**、重開機、30% 掉包、斷網 |
| itest uav | **6／6** | 起飛、單一任務、火警（1.24 s、誤差 0.28 m）、誘餌、三台分配（機間最近 15.8 m）、PX4 斷線 2.4 s 接手 |
| G2（`swarm_cbba/test/g2_e2e.sh`） | **58／58** | 伯宇的 T1、T2、T6、T8、T9、T10；E1～E4（60 s 排除、PERMANENT、回報檢查、不排第二個火警）；DEMO3（三隻狗＋uav1、鎖定） |
| 失聯接手（真 UDP） | 通過 | 關掉持有火警的節點 → 1.3 s 判定失聯 → 11.3 s 撤銷 → 11.8 s 別台接手 |

### 第 5 週弱網量測（itest loss，每個掉包率 20 次，延遲 50±30 ms）

| 掉包率 | 計入 | 5 s 內收斂 | 收斂中位數／P90 | mesh 頻寬（每台） |
|---|---|---|---|---|
| 0% | 20 | 100% | 0.55／0.55 s | 6.3 kB/s |
| 10% | 20 | 100% | 0.60／0.65 s | 29 kB/s |
| **30%** | 20 | **95%** | 0.65／0.85 s | 36 kB/s |
| 50% | 20 | 95% | 0.65／2.91 s | 51 kB/s |
| 70% | 5 | 60% | 3.1／5.7 s | 49 kB/s |

- 30% 剛好達標（19／20），報告前建議 `--levels 30 --trials 100` 再量一次。
- 50% 的長尾以 1 s 為一階：任務定義掉了要等 `resend_gap`（1 s）補發。
- 70% 已超出設計範圍：誤判失聯（1.5 s 內收不到的機率約 7%）、完成確認要約 11 s（重送 10 次），開始時常常還有上一次的任務，只計入 5 次。
- 第一次量測（同一天）30% 以上全部 0%，是測試的收尾漏掉排隊的任務、連環卡住，已修正（`itest.py` loss 的收尾、試驗間等全隊閒下來）。

---

## 三、2026-10-11～12 完成的事項

### 介面與機器狗端對齊
- 統一版 `swarm_cbba/cbba_node`（無人機、狗同一支），`swarm_interfaces` v1.6：`current_task`、`RobotState.mode`、
  `TaskResult.reason`，不握手、`assign_hold` 0.4 s；伯宇的 BT 換套件重新編譯即可。
- 失敗依 reason 排除：PERMANENT 永久不接，其他 60 s 後恢復。
- log 字串照伯宇 e2e 腳本的 grep（`mode 0 → 1`、`reason 2，detail「…」`、`失敗（detail）`、`收到完成證明`）。
- 伯宇的測資整理成 `g2_e2e.sh`（58 項），可以在 uav 容器直接跑、不干擾正在跑的節點。

### 機間通訊（UDP）
- **DDS 只在機內**（`DDS_OVER_MESH=0`）：待命時 mesh 流量 69 → 3.5 kB/s，第 5 週的 tc 數據才可信。
- **失聯接手**：已接受的任務在持有者失聯後先鎖著，火警 10 s、巡檢 30 s 後各機自己撤銷（新版本、執行者 0）、重新競標；
  舊持有者回來後指派被取代、舊版本回報被拒。參數 `takeover_fire`、`takeover_patrol`（全隊要一樣）。
- 解除鎖定的任務馬上重新出價（不等 1 s 的重新評估）。

### 無人機（PX4、BT）
- `px4_state_bridge`：**爬到巡檢高度才 mode AUTO**（`auto_min_altitude`，`cbba_uav.sh` 給高度 − 0.5 m），起飛完成才出價；
  `status_timeout` 2 → 5 s（原本一直誤判、反覆撤銷任務）。
- 備用 BT：
  - 火警「只報一次」改成看位置（3 m 內同一個火點），多個火點都貼 tag 101 也能各報一次；
  - **路過發現**：飛行途中看到別的火點 → 地面處置任務（給狗）＋火點監控任務（懸停 20 s），手上的任務回報
    `FIRE_FOUND`（PREEMPTED）交回 CBBA；火點就是自己的目的地時到達後才判定。

### 模擬環境與測試工具
- `docker/itest`：`comm`、`uav`、**`loss`**（第 5 週量測＋`plot_loss.py`）、**`demo`**（展示）；`config.json` 的路徑修正。
- `demo`：patrol_site 3 個固定任務（G1～G3），各台到達時 R1～R5 隨機起火（紅色方塊＋tag 101＋火警任務），
  CBBA 指派一台飛過去、丟出地面任務、懸停監控；全部起飛完成、懸停 8 s 後才建任務；Gazebo 錄影 `gazebo.mp4`＋
  俯視動畫 `demo.mp4`（`animate_demo.py`，事件附近 1 倍速、其他 3 倍速）。
- Gazebo 視窗設定 `docker/gz/gui/cbba_gui.config`：地面網格、關陰影、俯瞰鏡頭、錄影外掛；R 方塊改灰色，火點方塊
  `models/fire_block` 起火時才生成；`patrol_scenario.sh fire`、`fire-at`。

### 文件
- `docs/sim_launch_guide.md` 依步驟重寫（準備 → 共通規則：指令在哪個容器下 → A～F）。
- `sitl_test.md`、`parameters.md`、`protocol.md`、各套件 README、CLAUDE.md 跟著更新；編譯指令補上 `swarm_cbba`。
- 隊友的資料（handoff、測資、圖片）不進 repo（`.gitignore`）。

---

## 四、缺失與待辦（依優先順序）

**程式、設定**

| # | 項目 | 狀況 | 優先 |
|---|---|---|---|
| 1 | demo 的 Gazebo 視窗開不起來 | 15:57 的 Gazebo 伺服器殘留，之後的 demo 加入舊場景（只有自己開場景才開視窗）；新的視窗設定是否讓 GUI 開不起來還要測（預設設定和 `cbba_gui.config` 各開一次比較）；demo 開始前要檢查殘留的 Gazebo | 高（展示） |
| 2 | 火警反應約 0.4 s（目標 300 ms） | `assign_hold` 0.4 s；建議只在有競爭、未收斂時才等 | 高（要和伯宇、宜臻定預算） |
| 3 | `resend_gap` 調整 | 50% 掉包的長尾以 1 s 為一階；要先開成 cbba_node 參數，量 1.0／0.5／0.3 s 的收斂時間和頻寬 | 中 |
| 4 | 30% 掉包的統計量 | 20 次剛好 95%，再量 100 次 | 中 |
| 5 | AGENT_STATE 的飛行狀態、BT 執行狀態送空的 | 改介面後 robot_state 沒有這些欄位；決策用不到，地面站監看看不到 | 低 |
| 6 | 沒有認證 | 任何人都能冒用機號；session 層只有介面 | 中（實機前） |
| 7 | 新版 CBBA 的其他限制條件 | 競標前、正式指派前、執行期間 | 中 |
| 8 | 取消的授權管理端是空的 | 和 GCS 角色一起定 | 低 |

**測試**

| # | 缺口 |
|---|---|
| 9 | 狗端沒整合測過：伯宇還在用協定版本 1（互相拒收），要等他換成 main；他的 G3 跑時加 `-p failure_exclude_sec:=1e9` |
| 10 | 避碰沒測過刻意交叉的情境 |
| 11 | 「路過發現」只有單元測試，還沒在 Gazebo 跑過；`px4_state_bridge` 的起飛高度判斷還沒有單元測試 |
| 12 | 電池「接了沒飛到」在 Gazebo 做不出來（PX4 模擬電池都從 100% 開始） |
| 13 | 多跳（A→B→C）、實機 mesh（multicast 轉送、MTU > 1200 B） |

**要決定**

| # | 決定 | 和誰 |
|---|---|---|
| 14 | 300 ms 怎麼分給 CBBA、BT、Nav2；`assign_hold` 怎麼改 | 伯宇、宜臻 |
| 15 | 平板／GCS 的角色（跑不跑 cbba_node、能否建任務、容量參數 Q） | 伯宇、宜臻 |
| 16 | 火警任務要不要搶占同類任務（現在被指派的那台要先做完手上的固定任務停留） | 勝翔 |
| 17 | 「失聯的那台不接新的」：各機分不出自己是不是失聯的那台，沒有另外實作 | 勝翔 |
| 18 | 正式的 UAV BT 照伯宇的範本重寫；看到火之後懸停監控多久、CPF 要不要搬進去 | 勝翔 |

**文件、對外**

| # | 項目 |
|---|---|
| 19 | `docs/protocol_v2_notice.md` 的介面那一段還是 10/09 的（`cbba_core/cbba_node`、`vehicle_type`），寄給規格作者前要改 |
| 20 | 給伯宇的說明（main 已照他的格式、`dog-integration` 不用了、行為差異見 `swarm_cbba/README_v60.md`） |
| 21 | 架構圖要更新；BT 介面合約 |

**實機部署**

| # | 項目 |
|---|---|
| 22 | 共同座標系（模擬靠出生點間隔換算） |
| 23 | Jetson（arm64）的 docker 映像、各機對時（chrony） |
| 24 | 實體相機的驅動、內參；`kCameraToBody` |
| 25 | 能量模型校正（無人機用實機 ulog；狗是佔位值） |
| 26 | 安全：RC 接管、地理圍欄、offboard 斷線的 failsafe |
| 27 | `itest` 改成 ssh 版（實機沒有 docker exec） |

---

## 五、還沒 commit

2026-10-12 下午之後的修改都還沒 commit（上一個 commit 是 `b744f1a`）：DDS 只在機內、失聯接手、`itest` 的 loss／demo、
`animate_demo.py`、`plot_loss.py`、BT 的火警修改、`px4_state_bridge` 的起飛高度、Gazebo 視窗設定與場景、文件。
容器裡共用的 `ros2_ws/install` 要重新編譯 6 個套件才是最新版。
