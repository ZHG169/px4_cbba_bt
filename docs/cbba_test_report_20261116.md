# CBBA 測試報告與指令測試項目（2026-11-16）

整理今天的成果（CB-4 整合測試、失聯接手時限、TDR 參數實驗）和每個測試指令的項目、指標，給報告與情境解釋用。
實驗規劃見 `docs/tdr_experiment_plan.md`；公式見 `ros2_ws/src/cbba_core/doc/formula/tdr_design.md`。

---

## 一、報告內容

### 1. CB-4 多台協同：整合測試通過

| 測試 | 結果 |
|---|---|
| itest `group`（`20261116_162150_group`） | G0～G7 共 8 項 PASS（閘道注入角色組＋假 BT、假 Planner） |
| itest `comm --dogs 3`（`20261116_162304_comm`） | C0～C9 共 10 項 PASS |
| itest `loss --dogs 3`（`20261116_162504_loss`，3 架無人機＋3 隻狗、12 個任務、各 20 次） | 30 %：5 s 收斂 100 %、0.75／1.31 s；70 %：100 %、2.31／3.66 s；重複認領 0、跨能力 0 |

和 11/15 比沒有退步（CB-4 改了封包：TASK_ANNOUNCE 124 B、種類 12、13）：70 % 的 P90 4.41 → 3.66 s、每台 TX 7.25 → 6.94 kB/s。

### 2. 執行者失聯 vs 失聯接手：做法 A（已實作、單元測試通過）

- **問題**：原本兩個計時都從「判定失聯」開始。P0 的接手時限 10 s 和組的恢復期限 10 s 同時到，接手後還要重新競標，
  所以組幾乎一定先被取消；P1（15 s）、P2／P3（30 s）更輪不到接手。
- **規則**：失聯只標記、不算恢復期限 → 依優先級等原執行者（P0 10、P1 15、P2／P3 30 s）→ 時限到撤銷舊指派、重新開放競標，
  **從這時才開始算 10 s** → 10 s 內有新指派就繼續；逾時才 RECOVERY_TIMEOUT（all_or_nothing 取消整組，否則取消失效角色＋依賴它的），發 fallback。
- **例 P1**：1.5 s 判定失聯 → 再 15 s 撤銷、重新競標 → 再 10 s 沒人接才 fallback。
- **測試**（`test_incident`）：接手成功不取消（接手時限 12 s ＞ 恢復期限仍由別台接手）、接手逾時才 fallback、P0／P1 各用自己的接手時限。

### 3. TDR 參數實驗：`energy_weight` 50 → 25，half_life 不改

**方法**（依〈TDR 參數設定與測試計畫〉）：w_E {0, 25, 50, 100, 200} × half_life 倍率 γ {0.5, 1, 2}（基準 60/120/300/600 s）= 15 組；
情境 S1～S6；每格 100 次、同一組種子配對比較。指標用模型外的（P0／P1 到達、耗電、返航餘量、完成率、固定 half_life 的折扣價值）＋ CBBA 穩定性。

**階段 C（不掉包，9000 次）**

| 區域 | 現象 | 解讀 |
|---|---|---|
| **w_E 25～50、γ 1～2** | S1～S3 全部選對、完成率 100 %、5 s 收斂 100 %；彼此差 1～4 % | 合理範圍 |
| w_E = 0 | S2、S3_P3 選對 0 %；返航餘量 −6.7 % | 不看電量：派出電量不足的載具 |
| w_E ≥ 100、γ ≥ 1 | S3_P0 選對 26 % → 0 %；P0 到達 +12 % | 省電壓過急迫度：緊急任務給了慢的 |
| w_E = 200 | 完成率 90～97 %；耗電 +55～128 % | 遠的任務出價變負、沒人接；任務拖久、待命也在耗電 |
| γ = 0.5 | 25／0.5 的 S3_P3 選錯；50／0.5 耗電 +22 % | half_life 太短，連 P3 都變急 |

注意：200／0.5 的 P0 到達 −21 % 是倖存者偏差（只算完成的，完成率 90 %），不是比較好。

**階段 D（弱網，候選 4 組 × 0/30/50/70 %，9600 次，各情境合併）**

| 掉包 | 25 / 1 | 50 / 1（原暫定值） | 25 / 2 | 50 / 2 |
|---|---|---|---|---|
| 30 % 5 s 收斂 | 100 % | 100 % | 100 % | 100 % |
| 50 % 5 s 收斂 | 99.3 % | 99.3 % | 99.3 % | 99.2 % |
| 70 % 5 s 收斂 | **84.2 %** | 84.2 % | 83.5 % | 82.5 % |
| 70 % 重複認領（600 次） | **51** | 53 | 60 | 61 |
| 70 % 收斂 P95（最差情境） | 8.29 s | 8.96 s | 7.86 s | 8.27 s |

- 弱網表現由網路決定，參數組之間幾乎沒有差別；30 % 掉包符合週計畫（≥ 95 %）。
- 載具越少越受掉包影響：70 % 時 S6（6 台）97 %，S1～S3（2 台）85 %，S4 插隊、S5 異質約 73 %（2 台只有一條連線，多台時有多條轉送路徑）。
- 離線模擬比 itest 嚴格（每個接收者各自掉包、收斂要全隊一致持續 0.5 s），用來比參數，不代表實際網路的絕對值。

**定案**

| 參數 | 正式值 | 備選 | 依據 |
|---|---|---|---|
| `energy_weight` | **25**（程式預設已改） | 50 | 和 50 比：P0 到達 −3.8 %、P0 P95 128 → 119 s、耗電 −2.9 %；S1～S3 全對；弱網持平或略好 |
| `half_life_by_priority` | **60/120/300/600 s（不改）** | 120/240/600/1200 s | γ 2 平手較多、70 % 掉包重複認領較多 |
| 狗的耗電模型 | 待伯宇實測 | 0.1 %/m＋0.05 %/s | 階段 E 重跑 |

**結論句**：在不造成期限違反與返航存量不足的前提下，參數組（w_E = 25，P0～P3 half-life = 60/120/300/600 s）相較原暫定值
使總耗電下降 2.9 %，P0 到達時間縮短 3.8 %（P95 128 → 119 s），30 % 掉包下 5 s 收斂率 100 %，70 % 掉包下 84.2 %（與原暫定值相同）。
w_E 在 25～50、γ 在 1～2 結果穩定；w_E = 0 會派出電量不足的載具，w_E ≥ 100 會犧牲緊急任務換省電。

### 4. 下一步

1. 重新編譯（預設值改了）、跑單元測試與 itest 回歸（comm、loss）
2. commit（CB-4 第二、三批、itest group、TDR 實驗工具、做法 A、w_E 25）
3. 階段 E：伯宇實測狗的速度、耗電後重跑 25／1、50／1 與邊界
4. 階段 F：BT、Planner、Registry 對接後在 Gazebo 驗證角色組

---

## 二、用指令測試的項目

### A. `cbba_sim tdr`：離線參數實驗

```bash
cbba_sim tdr <S1,…,S6|all> --weights 0,25,50,100,200 --gammas 0.5,1,2 --losses 0,0.3,0.5,0.7 --trials 100
python3 tdr_report.py <輸出目錄>   # 報告＋圖
```

**情境**

| 情境 | 設定 | 要說明的事 | 應該給誰 |
|---|---|---|---|
| S1 時間差 | 兩隻狗離任務一樣遠、耗電相同，速度 0.7 vs 1.5 m/s；P0／P1 | half_life 讓緊急任務給先到的 | 快的 |
| S2 能量差 | 兩隻狗只差電量 35 % vs 90 %；P2 | w_E 避開低電量的載具 | 電量多的（w_E 0 時平手比機號會給錯） |
| S3 時能取捨 | A 1.5 m/s、0.3 %/m；B 0.7 m/s、0.08 %/m；P0／P3 | 急的看時間、不急的看電量 | P0 → A、P3 → B |
| S4 路徑插入 | 狗前往 P3 途中，旁邊插入 P0 | 緊急任務插隊、舊任務延後多少 | — |
| S5 異質平台 | 1 架無人機＋1 隻狗、3 個平台不限的任務 | 平台成本反映在分配上 | — |
| S6 多任務機隊 | 3 架無人機＋3 隻狗、12 個任務＋1 組 3 個角色 | 工作量分散、不頻繁換手、角色由不同台做 | — |

**任務品質指標**

| 指標 | 定義 | 怎麼看 |
|---|---|---|
| P0／P1 到達時間 | 插入 → 執行者到達（中位、P95） | 緊急任務有沒有被省電拖慢；只算完成的（注意倖存者偏差） |
| 耗電 | 全部載具耗電總和，和基準組配對比較 | 含無人機待命耗電（PX4 模擬電池依時間），任務拖久也會變高 |
| 實際折扣價值 | v × 2^(−到達 / h)，h 固定用 60/120/300/600 s | 完成得及不及時；h 固定，不同 γ 才能比 |
| 完成率 | 完成數 / 任務數 | w_E 太大會沒人出價 |
| 期限違反 | 有期限的任務沒完成或晚到 | 必須 0 |
| 最低返航餘量 | 電量 − 返航耗電 − 存量 | ≥ −0.5 %（模型不含待命耗電、實際耗電 ±10 %） |
| 選對比例 | S1～S3 第一位執行者 = 應該給誰 | ≥ 90 %：公式的取捨方向對不對 |
| 分配均衡 | 每台完成數的最大、最小 | 有沒有集中在一台 |
| 舊任務延後（S4） | 實際到達 − 沒被插隊時的到達 | 搶占的代價 |

**CBBA 穩定性指標**

| 指標 | 定義 | 怎麼看 |
|---|---|---|
| 收斂時間 | 插入後全部存活節點對得標者一致、得標者路徑上有它，持續 0.5 s（取開始時刻） | 中位、P95 |
| 5 s 收斂率 | 全部任務 5 s 內收斂且沒有重複認領的比例 | 30 % 掉包 ≥ 95 % |
| 重複認領 | 同一任務同時兩台以上執行 | 應為 0 |
| 換手 | 執行者換人；分被迫（失敗、返航、搶占）與非被迫（重新競標） | 非被迫越少越穩 |
| 分數差距 | 收斂當下第一、二名出價的相對差；< 1 % 算平手 | 平手多，高掉包時可能互搶 |
| bundle 重建 | 路徑上還開著的任務被拿掉的次數 | 太高代表競價太敏感 |
| 出價明細（`bids.csv`） | 每台的到達估計、時間報酬、能量懲罰、邊際出價、插入位置、是否得標 | 解釋為什麼是這台得標 |

**入選順序**：底線（期限違反 0、返航餘量 ≥ −0.5 %、完成率不低於 w_E 0 超過 1 點、選對 ≥ 90 %、不掉包 5 s 收斂 ≥ 95 %）
→ P0 到達在最好的 10 % 內 → 耗電最低 → 弱網。

### B. `itest loss`：弱網量測（真實節點＋tc）

```bash
python3 docker/itest/itest.py loss --dogs 3 --levels 30,70
```

情境：3 架無人機＋3 隻狗、一次注入 12 個任務（空中 6、地面 6）、延遲 50±30 ms、每個掉包率 20 次。

| 指標 | 定義 | 怎麼看 |
|---|---|---|
| 送達（A） | 每台知道幾個任務、送達時間 | 任務擴散速度 |
| 分配（B） | 每台分到幾個；和最近那台一致的比例 | 不一致不是錯（CBBA 不保證最近），高掉包時建立者先搶 |
| 收斂 | 每個任務只剩一位持有者的時間（Median、P90，逾時記 10 s） | 判 PASS 的主指標 |
| 5 s 收斂率 | 5 s 內收斂且沒有重複認領 | 30 % ≥ 95 % |
| 重複認領、跨能力、完成後又派 | — | 都應為 0 |
| 完成 → 全隊確認 | 回報完成到全隊知道結束 | 70 % 約 11 s（重送間隔 × 次數） |
| 每台 TX、重傳 TX | 注入後 10 s 每台送出的 kB/s | 頻寬需求 |

### C. `itest comm`：協定功能（C0～C9）

| 項目 | 情境 | 要說明的事 |
|---|---|---|
| C1 | 注入一個任務 | 全隊收到、只有一台得標 |
| C2 | 回報完成 | 全隊不再派 |
| C3 | 執行失敗 | 交回、別台接手 |
| C4 | 停送狀態（PX4 斷線） | 不再參與、別台接手 |
| C5 | 節點停止 | 偵測失聯、別台接手 |
| C6 | 重開機 | 重新加入、不重用 task_id |
| C7 | 30 % 掉包 | 10 s 內分配完成 |
| C8 | 斷網 | 只有建立者知道（協調確實經過 mesh）；恢復後不重複 |
| C9 | 100 % 斷網 | 各自做完手上的、自己建的自己接；恢復後一致 |

### D. `itest group`：多台協同（G0～G7）

| 項目 | 情境 | 要說明的事 |
|---|---|---|
| G1 precedence | 空中先定位 → 地面才開始 | 狗的 gate 等無人機定位才打開 |
| G2 incident | 執行者回報新位置 | 閘道驗證、revision + 1、全隊同步 |
| G3 會合點 | 兩隻狗都要到位 | 都到了才一起開始 |
| G4 distinct | 角色要不同載具 | 靠近的狗只拿一個角色 |
| G5 all-or-nothing | 角色湊不齊 | 不發任務，10 s 後整組取消、FallbackRequest |
| G6 假 Planner 改版 | 事件位置更新 | 只改版 TaskSpec 有變的角色 |
| G7 假 Planner fallback | 收到 fallback_request | 注入降級的組、有狗接 |

### E. 單元測試：失聯接手時序（`test_incident` 的 Takeover）

| 情境 | 時間線 | 結果 |
|---|---|---|
| 接手成功 | 失聯 1.5 s → 等接手時限 → 撤銷 → 10 s 內別台得標 | 組繼續 |
| 接手逾時 | 失聯 1.5 s → 等 3 s（測試設定）→ 撤銷 → 10 s 沒人接 | RECOVERY_TIMEOUT、取消失效＋依賴角色、fallback |
| 優先級不同 | 同一台持有 P0、P1 角色後失聯 | P0 約 14.5 s、P1 約 19.5 s 各自失效 |

---

## 三、重跑方式

```bash
OUT=/tmp/cbba_tdr2   # 已存在；新目錄要先 mkdir，否則 docker 建成 root 的

# 1. 編譯 cbba_core＋單元測試（隔離目錄，不碰共用的 ros2_ws/build）
docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp -v /home/zhg/CBBA_BT:/home/ncrl/CBBA_BT -v $OUT:/out \
  cbba-robot:jazzy bash -c 'source /opt/ros/jazzy/setup.bash && source /home/ncrl/CBBA_BT/ros2_ws/install/setup.bash &&
  cd /home/ncrl/CBBA_BT/ros2_ws && python3 -m colcon --log-base /out/log build --build-base /out/build \
  --install-base /out/install --packages-select cbba_core --cmake-args -DCMAKE_BUILD_TYPE=Release &&
  cd /out/build/cbba_core && for t in test_*; do ./$t > /dev/null || echo "FAIL $t"; done; echo done'

# 2. 參數實驗（C：15 組不掉包；D：候選組＋弱網）
docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp -v /home/zhg/CBBA_BT:/home/ncrl/CBBA_BT -v $OUT:/out \
  cbba-robot:jazzy bash -c '/out/build/cbba_core/cbba_sim tdr all --trials 100 --losses 0 --out /out/tdr_c'
docker run --rm --user $(id -u):$(id -g) -e HOME=/tmp -v /home/zhg/CBBA_BT:/home/ncrl/CBBA_BT -v $OUT:/out \
  cbba-robot:jazzy bash -c '/out/build/cbba_core/cbba_sim tdr all --weights 25,50 --gammas 1,2 \
  --losses 0,0.3,0.5,0.7 --trials 100 --bids 0 --out /out/tdr_d'

# 3. 報告＋圖（主機沒有 matplotlib，用 cbba-sim 映像；不用重新模擬）
for d in tdr_c tdr_d; do docker run --rm --user $(id -u):$(id -g) -e MPLCONFIGDIR=/tmp -v /home/zhg/CBBA_BT:/repo \
  -v $OUT:/data --entrypoint python3 cbba-sim:jazzy /repo/ros2_ws/src/cbba_core/scripts/tdr_report.py /data/$d; done

# 4. 共用 install 重新編譯（uav 容器裡）＋ itest 回歸
colcon build --symlink-install --packages-select swarm_interfaces cbba_core swarm_cbba px4_waypoint_node uav_px4_bt apriltag_fire_detector
python3 docker/itest/itest.py comm --dogs 3 --kill-existing
python3 docker/itest/itest.py loss --dogs 3 --levels 30,70 --kill-existing
python3 docker/itest/itest.py group --dogs 3 --kill-existing
```

報告的參數：`--baseline 50,1`（比較的基準組，預設是原暫定值）、`--eta-tol 0.10`、`--margin-tol 0.5`、`--choice-min 0.9`、`--no-plots`。
