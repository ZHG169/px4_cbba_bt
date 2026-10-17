# 目前進度（無人機端）

最後更新：2026-10-17 · 勝翔 · 這份會持續更新；某一天的快照另外存在 `reports/`（例如 `reports/progress_20261011.pdf`）。

狀態：**SITL** 在 PX4 SITL＋Gazebo 實測通過　**測試** 單元或真 UDP 整合測試通過、還沒上 SITL　
**進行中**　**未達成**　**超前**＝週計畫之後的週次或週計畫沒有的項目　**狗端**＝要等伯宇

週計畫：2026-10-05 ～ 11-13（5 週開發＋1 週緩衝），現在是第 2 週結尾；第 1～3 週的無人機端項目都已在 SITL 驗證，
第 4～6 週有一部分已經先做。

---

## 一、週計畫對照

| 週 | 週計畫項目 | 目前 | 驗證 | 狀態 |
|---|---|---|---|---|
| 1 | PX4 SITL＋XRCE-DDS、起飛懸停 | 多台 SITL（docker：sim、uavN、gcs），`px4_sitl.sh` 自動加入同一個場景 | itest U1（3 台起飛） | SITL |
| 1 | CBBA 通訊訊息 | 協定版本 2（UDP，每筆帶 task_id、session、正式指派版本），取代週計畫的 `CBBAMessage` | 單元、真 UDP、itest comm | SITL |
| 1 | mesh＋`tc` 弱網環境 | `mesh_tc.sh`；**DDS 只在機內**（`.env` 的 `DDS_OVER_MESH=0`），mesh 上只有 CBBA 的 UDP | itest M：待命 69 → **3.5 kB/s** | SITL |
| 2 | PX4 3D 航點巡邏 | **正式 BT**：照伯宇骨架的 `uav_task_manager_node`（10/13）；**CBF 避碰**（10/14 取代 CPF）＋運動平滑層 | itest uav 6／6（10/17） | SITL |
| 2 | 空中出價（地面任務 0） | 統一版 cbba_node 的相容表（無人機只接 AIR_RECON） | itest、G2 T1、DEMO3 | SITL |
| 2 | 飛完 3 個點；5 個混合任務各自只對專長出價 | 三台分配三個任務；混合任務用真 UDP＋假狗 | itest U5、G2 | SITL |
| 3 | AprilTag 火情確認（> 0.85） | `apriltag_fire_detector`＋BT 的 `AirRecon`；誘餌 tag 7 不觸發 | itest U3（到達後 1.2～1.9 s、誤差 < 0.5 m）、U4 | SITL |
| 3 | `ReportFireEvent` 注入地面任務 | 巡檢點看到火 → 發布地面任務 → **懸停監控到狗處理完**（`task_closed`）→ 回起飛點；路上看到火 → 轉向火點上空監控 → 回航線（10/14）；全隊火點去重複（`task_announced`） | itest demo（假狗） | SITL |
| 4 | 火警 → 狗得標、中斷、掉頭 | CBBA 端完成；介面照伯宇的格式（`swarm_cbba`、v1.6），他的 BT 不用改 | G2 61 項（伯宇的 T1～T10＋E1～E4＋DEMO3＋孤兒任務）；假狗 | 測試／狗端 |
| 4 | 黑板覆寫 → cancel ≤ 50 ms | 狗的 BT 才量得到 | 伯宇 G3：0.1～0.3 ms | 狗端 |
| 4 | 火情發布 → 狗開始轉向 ≤ 300 ms | 指派約 0.4 s（`assign_hold` 0.4 s 佔大部分） | 分段量測 | 未達成 |
| 4 | Gazebo 兩種載具同一個世界 | sim 只有 PX4；狗在 Gazebo 用橘色方塊替身（`fake_dog.py`） | — | 狗端 |
| 5 | `tc` 掉包 0／10／30／50／70%、延遲 50±30 ms，注入多源任務 | `itest.py loss`：每個掉包率 20 次、每次三台各建一個任務 | 見「二、測試結果」 | SITL |
| 5 | 5 s 內收斂率（30% 掉包 ≥ 95%） | **30%：95%（19／20），中位數 0.65 s** | itest loss L30 PASS（10/12） | SITL（剛好達標） |
| 5 | 掉包率對收斂時間、頻寬曲線 | `plot_loss.py` 畫圖；頻寬 6 → 51 kB/s（0 → 50%） | itest loss | SITL |
| 5 | 100% 斷網 | 斷網時只有建立者拿到；**失聯接手**（火警 10 s、巡檢 30 s） | itest C8、C5 | SITL |
| 5 | 連鎖退標、不重複認領 | 單元測試；itest 檢查不重複 | C1、C7、loss | SITL |
| 6 | 一鍵啟動 | `cbba_uav.sh`、`docker/itest`（comm、uav、loss、demo）、情境檔、手動插入（`insert.py`） | — | 可用 |
| 6 | 容器化 | docker 環境 | — | 完成 |
| 6 | Jetson Orin 部署 | 還沒開始 | — | 待做 |
| 6 | 數據報表、展示影片 | `itest.py demo`：場景的俯瞰攝影機錄影（`gazebo.mp4`）＋俯視動畫（`demo.mp4`，含電量） | demo 6／6（10/13） | 可用 |

週計畫和實作不同的地方（要對外說明）：機間通訊走 UDP 的協定版本 2（沒有 `mesh_bridge_node`、`CBBAMessage`）；
消解規則實作 17 條；「黑板」對我們是 cbba_node 的話題；命名空間是 `/uavN`；300 ms 目前達不到。

---

## 二、最近的測試結果

### 2026-10-17（失敗後出價打折＋無人機被接管，使用者回報全過）

| 測試 | 結果 |
|---|---|
| 單元測試 `cbba_core` | 全過（`test_scoring` 加 4 項、`test_agent` 加 6 項） |
| 單元測試 `uav_px4_bt` 的 `test_skeleton` | 32 項全過；之後加懸停高度 → 33 項全過 |
| G2（`swarm_cbba/test/g2_e2e.sh`） | 59 項（E1 加「失敗 1 次，估計成功率 0.50」）；之後加孤兒任務警告 2 項 → 61 項全過 |
| itest uav | 6／6 |
| SITL 手動：分辨人接管和 PX4 失效保護 | `commander mode auto:loiter` → `user_intention` 跟著變、`failsafe` 0；停掉 BT → `nav_state` AUTO_RTL、`user_intention` 仍是 OFFBOARD、`failsafe` 1（`docker/scripts/watch_px4_status.py`）。模擬停用了遙控器，`posctl` 在空中會被拒 |

### 2026-10-13

單元測試 cbba_core 145、uav_px4_bt 60 全過、零警告；itest（task_manager）uav 6／6（U6 1.0 s 接手）、demo 6／6（種子 7、21）。

### 2026-10-12：第 5 週弱網量測（itest loss，每個掉包率 20 次，延遲 50±30 ms）

| 掉包率 | 計入 | 5 s 內收斂 | 收斂中位數／P90 | mesh 頻寬（每台） |
|---|---|---|---|---|
| 0% | 20 | 100% | 0.55／0.55 s | 6.3 kB/s |
| 10% | 20 | 100% | 0.60／0.65 s | 29 kB/s |
| **30%** | 20 | **95%** | 0.65／0.85 s | 36 kB/s |
| 50% | 20 | 95% | 0.65／2.91 s | 51 kB/s |
| 70% | 5 | 60% | 3.1／5.7 s | 49 kB/s |

- 30% 剛好達標（19／20），報告前建議 `--levels 30 --trials 100` 再量一次。
- 50% 的長尾以 1 s 為一階：任務定義掉了要等 `resend_gap`（1 s）補發。
- 70% 已超出設計範圍：誤判失聯、完成確認要約 11 s（重送 10 次）。
- 之後協定沒有改，但 BT、cbba_node 改了不少，正式報告前要重量。

---

## 三、2026-10-13～17 完成的事項

### 無人機 BT
- **正式 BT 改用伯宇的骨架**（`uav_task_manager_node`，10/13）：急停 → 哨兵 → 巡檢佇列；葉節點 `NavigateTo`、`AirRecon`；
  起飛、待命、做完回起飛點、低電量返航在樹外（`UavFlight`）。備用 BT（`uav_bt_node`）保留。
- **CBF 避碰取代 CPF**（10/14），運動平滑層（加速度上限、剎車曲線）；修好起飛、懸停晃動（懸停水平 4～7 cm）。
- 火警照週計畫：巡檢點看到火 → 發布地面任務 → 懸停監控到狗處理完 → 回起飛點；路上看到火 → 轉向火點上空監控 → 回航線；
  別台報過的火不再發布、不轉向。
- **被接管**（10/17，照機器狗端的平板接管）：起飛完成後 offboard 掉了，用 PX4 的 `failsafe` 分辨——人接管 → 任務以
  `PREEMPTED_BY_MANUAL` 交還（5 s 不接、不算失敗次數）；PX4 失效保護 → `PX4_FAILSAFE`（STOPPED）。切回 offboard 從目前位置接回。
- **mode 改由 BT 決定**（10/17）：BT 送 `/uavN/bt_mode`，`px4_state_bridge` 照抄進 robot_state。被接管、失效保護、返航時 MANUAL、不出價
  （原本 bridge 只看高度，接管、返航時仍送 AUTO）；「起飛完成」只剩 BT 一套標準。
- 高度警告：爬升 20 s 沒到巡檢高度、切回 offboard 時高度和巡檢高度差太多。
- `cbf.brake_vertical_accel` 0.2 → 0.1（起飛、換高度不衝過頭）。

### CBBA
- **TRANSIENT 失敗後出價打折**（10/17，宜臻的實作說明「需求 1」）：失敗 f 次 → 成功率 1/(1+f)，價值乘成功率、到達時間多算預期的重試；
  60 s 後通常由別台接手，不會一直派回同一台再失敗。公式與理由在 `cbba_core/doc/formula/formula_design.md` 第 5 行。
- **孤兒任務警告**：任務開著但 30 s 沒有人得標（能做的都被永久排除、不在 AUTO 或失聯）→ log 警告＋`task_orphaned`，只通知、不處理。
- PREEMPTED 的排除時間 60 → 5 s（狗、無人機都是；接管期間靠 mode = MANUAL 不出價）。新參數 `preempt_exclude_sec`、`max_failures`。
- 執行超時交還（`overrun_margin`）：懸停監控火點的那台不再被排新任務（修 U6）。
- `task_closed`、`task_announced` 話題（無人機的火警監控、火點去重複）。

### 模擬環境與工具
- `demo_site`（±22 m 展示場景）、場景的俯瞰攝影機錄影、假狗 `fake_dog.py`、火點方塊與任務平台模型。
- itest demo D1～D7、情境檔（`docker/itest/scenarios/`）、兩個 terminal 手動插入任務（`insert.py`）、俯視動畫加電量。
- `watch_px4_status.py`：看 PX4 的模式、失效保護欄位。
- Nav2 導入規劃：`docs/nav2_integration_plan.md`。

### 文件、commit
- 10/15 起全部 commit（GitHub main）；README、各套件文件、參數文件、CLAUDE.md 跟著更新。
- 給伯宇的說明 `swarm_cbba/README_v60.md` 已更新（失敗打折、PREEMPTED 5 s、他的 G3 要加 `preempt_exclude_sec: 1e9`）。

---

## 四、缺失與待辦（依優先順序）

**程式、設定**

| # | 項目 | 狀況 | 優先 |
|---|---|---|---|
| 1 | 火警反應約 0.4 s（目標 300 ms） | `assign_hold` 0.4 s；建議只在有競爭、未收斂時才等 | 高（要和伯宇、宜臻定預算） |
| 2 | 被接管的整串 SITL 測試 | 10/17 SITL 通過：飛行途中接管 → PREEMPTED_BY_MANUAL 交還 → 切回 → 重新得標完成；接管 40 s 時孤兒任務警告、切回解除 | 完成 |
| 3 | 懸停高度停在 4.71 m | 10/17 改：待命點的高度一律是巡檢高度（被接管後切回也回到巡檢高度）；單元測試、SITL 通過 | 完成 |
| 4 | 需求 2 剩下的 | 平板指定的點剛好是 CBBA 任務時要鎖住（別台不出價）。孤兒任務警告已加（10/17，`orphan_warn_sec` 30 s、`task_orphaned`；只通知，之後怎麼處理再看） | 中 |
| 5 | `resend_gap` 調整 | 50% 掉包的長尾以 1 s 為一階；要先開成 cbba_node 參數，量 1.0／0.5／0.3 s | 中 |
| 6 | 30% 掉包的統計量 | 20 次剛好 95%，再量 100 次 | 中 |
| 7 | `cbba_uav.sh` 的預設執行者 | 10/17 改成正式 BT（`task_manager`），`EXECUTOR=bt` 換備用 BT | 完成 |
| 8 | AGENT_STATE 的飛行狀態、BT 執行狀態送空的 | 決策用不到，地面站監看看不到 | 低 |
| 9 | 沒有認證 | 任何人都能冒用機號；session 層只有介面 | 中（實機前） |
| 10 | 新版 CBBA 的其他限制條件 | 競標前、正式指派前、執行期間 | 中 |
| 11 | 取消的授權管理端是空的 | 和 GCS 角色一起定 | 低 |
| 12 | 殘留的 Gazebo 伺服器不理 SIGTERM | itest 已改成 SIGKILL 後備 | 低 |

**測試**

| # | 缺口 |
|---|---|
| 13 | 狗端沒整合測過：伯宇還在用協定版本 1（互相拒收），要等他換成 main；他的 G3 跑時加 `failure_exclude_sec`、`preempt_exclude_sec` 都設 1e9 |
| 14 | 兩隻狗：A 回報 TRANSIENT 後由 B 得標的整合測試（目前只有單元測試） |
| 15 | 巡檢中的狗被火警搶占（狗有自己的巡檢）還沒在 SITL 跑 |
| 16 | 電池「接了沒飛到」在 Gazebo 做不出來（PX4 模擬電池都從 100% 開始） |
| 17 | 多跳（A→B→C）、實機 mesh（multicast 轉送、MTU > 1200 B） |
| 18 | 失效保護中人接手（`failsafe_and_user_took_over`）還沒在 SITL 看過 |

**要決定**

| # | 決定 | 和誰 |
|---|---|---|
| 19 | 300 ms 怎麼分給 CBBA、BT、Nav2；`assign_hold` 怎麼改 | 伯宇、宜臻 |
| 20 | 平板／GCS 的角色（跑不跑 cbba_node、能否建任務、容量參數 Q） | 伯宇、宜臻 |
| 21 | 任務層級的 PERMANENT（地圖外、沒路）要不要另外分類、通知地面站 | 伯宇、宜臻 |
| 22 | 「失聯的那台不接新的」：各機分不出自己是不是失聯的那台，沒有另外實作 | 勝翔 |

**文件、對外**

| # | 項目 |
|---|---|
| 23 | 傳訊息給伯宇（失敗打折、PREEMPTED 5 s、G3 要加的參數、換成 main） |
| 24 | `docs/protocol_v2_notice.md` 的介面那一段還是 10/09 的，寄給規格作者前要改 |
| 25 | 架構圖要更新；BT 介面合約 |

**實機部署**

| # | 項目 |
|---|---|
| 26 | 共同座標系（模擬靠出生點間隔換算） |
| 27 | Jetson（arm64）的 docker 映像、各機對時（chrony） |
| 28 | 實體相機的驅動、內參；`kCameraToBody` |
| 29 | 能量模型校正（無人機用實機 ulog；狗是佔位值） |
| 30 | 安全：遙控器接管、地理圍欄、offboard 斷線的 failsafe（模擬關掉的 `NAV_DLL_ACT`、`COM_RC_IN_MODE`、`COM_RCL_EXCEPT` 實機要開回來） |
| 31 | `itest` 改成 ssh 版（實機沒有 docker exec） |

---

## 五、commit

GitHub main 已包含到 2026-10-17 的所有改動（`799fdf6`）。宜臻的「CBBA 任務失敗排除與人工接管：實作說明.md」在 repo 根目錄，
不進 repo（隊友的資料）。容器裡共用的 `ros2_ws/install` 改 C++ 後要在 uav 容器重新編譯。
