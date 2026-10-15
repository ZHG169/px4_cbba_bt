# 插入任務點的使用方法（2026-10-14）

在模擬裡加入新的任務點或起火點，看 CBBA 怎麼分配、無人機怎麼處理。三種方式：

| 方式 | 什麼時候用 | 會進報告、動畫、錄影嗎 |
|---|---|---|
| [一、手動插入（`--interactive`）](#一手動插入--interactive) | 邊看邊決定要插什麼、插在哪 | 會 |
| [二、情境檔（`--scenario`）](#二情境檔--scenario) | 事先排好，每次跑都一樣（可重現、可比較） | 會 |
| [三、不經過 itest](#三不經過-itest手動開-sitl) | 自己開 SITL、手動下指令除錯 | 不會 |

場景都是 `demo_site`（點在 `docker/gz/worlds/demo_site_points.csv`）。座標是 map ENU（公尺，x 往東、y 往北），
無人機出生點在 (0, 0)、(0, 2)、(0, 4)。

---

## 一、手動插入（`--interactive`）

### 啟動：兩個 terminal（主機）

| terminal | 做什麼 | 指令 |
|---|---|---|
| 1 | **開環境**：Gazebo、三台 PX4、起飛、cbba_node、假狗；跑完產生報告、錄影 | `python3 docker/itest/itest.py demo --interactive --area -15 15 -12 8` |
| 2 | **輸入資料**：插入任務點、起火點 | `python3 docker/itest/insert.py` |

```bash
# terminal 1
cd ~/CBBA_BT
python3 docker/itest/itest.py demo --interactive --area -15 15 -12 8 --seed 42 --kill-existing

# terminal 2（terminal 1 開始跑之後）
cd ~/CBBA_BT
python3 docker/itest/insert.py              # 互動：insert> 提示後輸入指令；exit 或 Ctrl-D 離開（不影響 terminal 1）
python3 docker/itest/insert.py t 12 -4      # 或一次只送一個指令
```

- **原本的巡檢任務 G1～G3 一開始就會建立**（`config.json` 的 `demo.patrol`，停留時間照 `demo.durations`／`demo.duration`，
  預設都是 4 s），也要做完；你插入的任務點是加在它們之上，CBBA 會把新點插進各台正在做的路徑裡重新分配。
  不要原本的任務、全部手動插入時加 `--no-patrol`；搭配 `--scenario` 時改照情境檔，不另外建 G1～G3。
- `--area x0 x1 y0 y1`：插入範圍（建議限制在 30 m × 20 m 左右，畫面看得清楚）；不給就用 `config.json` 的 `demo.area`。
- terminal 2 在環境還沒準備好（起飛、CBBA 上線前）送的指令會先排著，上線後才處理。
- 回覆（插入了什麼、範圍外、列表）會同時印在兩個 terminal。
- 兩個 terminal 之間只靠 `docker/itest/out/interactive/` 的檔案（`commands.txt`、`replies.txt`、`session.json`），
  terminal 1 結束時 `session.json` 會刪掉，terminal 2 再送會提示「沒有在跑的 itest demo --interactive」。
- 也可以直接在 terminal 1 輸入（相容），但 itest 的輸出會一直捲動，不好打字。

### 指令

| 指令 | 作用 | 例子 |
|---|---|---|
| `t` | 範圍內**隨機**一個任務點：生成藍色平台＋建立任務 | `t` |
| `t x y` | 指定位置的任務點 | `t 12 -4` |
| `t 點名` | 用場景的點（G1～G3、R1～R5），不生成平台；那裡還有沒做完的任務時會拒絕（例如 G1～G3 還在做） | `t R3` |
| `tf` / `tf x y` | **任務點而且那裡起火**：飛到 → 看到火 → 發布地面任務 → 移到火點上空監控 → 狗處理完才完成 | `tf 8 6` |
| `f` / `f x y` | **只起火、不建任務**：無人機路過（下視相機範圍內）或在附近懸停時發現 → 轉向火點上空監控 | `f -9 -2` |
| `d=秒數` | 停留時間（預設 4 s），接在 `t`、`tf` 後面 | `t 12 -4 d=15`、`t d=20` |
| `l` | 列出已插入的任務（完成／uavN 執行中／等待分配）和起火點 | |
| `a` | 顯示範圍 | |
| `h` | 說明 | |
| `q` | 不再插入；目前的任務做完、狗處理完 → 回起飛點 → 產生報告 | |

### 範圍與隨機點

- 預設範圍 x −22～24、y −14～11（俯視錄影拍得到的地方）；`--area` 或 `docker/itest/config.json` 的 `demo.area` 改。
- 手動給的座標超出範圍會拒絕。
- 隨機點離場景的點、已插入的點、起飛點都 ≥ 3 m（`demo.min_gap`），避免被當成同一個點；位置由 `--seed` 決定，
  同一個種子、同樣的輸入順序會得到同樣的位置（種子會印在開頭，也寫在 `demo_plan.json`）。

### 要知道的規則

| 規則 | 影響 |
|---|---|
| 起火點離無人機的目的地 ≤ 3 m（`fire_task.dedup_radius`）算「目的地的火」 | 路上不轉向，到了懸停時才判定（`tf` 就是這種） |
| 起火點離航線在下視相機範圍內（5 m 高約 12 m × 8 m）、離目的地 > 3 m | 飛行途中看到 → 轉向火點上空監控 → 狗處理完 → 回航線繼續原本的任務 |
| 同一個火點（3 m 內）全隊只發布一次地面任務 | 別台已經報過的火，看到也不再發布、不轉向（`task_announced`） |
| 無人機只做空中任務、狗只做地面任務 | 地面任務一定給狗（假狗從 (−4, 0) 出發，1.5 m/s） |
| 監控最多 `fire_task.monitor_timeout`（yaml 預設 120 s） | 狗太慢時無人機會先離開 |
| CBBA 依「距離 × 到達時間」出價 | 停留時間長的點，CBBA 比較不會把其他任務排在同一台後面 |

### 例子：原本的巡檢＋路上的火＋臨時插入

G1～G3 一開始就建立了。在 terminal 2：

```
f -9 -2        # R1 旁起火：飛 G3 的那台路上看到 → 轉向監控 → 狗處理完 → 回去 G3
t d=10         # 範圍內隨機插一個任務點：CBBA 把它排進某台的路徑（可能是做完的、也可能是排在後面）
tf 8 4         # 再插一個任務點、那裡起火：飛到的那台看到火 → 監控到狗處理完
l              # 看 G1～G3 和插入點的分配、進度
q              # 不再插入；全部做完（含 G1～G3）就結束
```

### 結果

輸出在 `docker/itest/out/<時間>_demo/`：

| 檔案 | 內容 |
|---|---|
| `report.md` | D1 你插入的任務都完成；有起火時 D2 發現火警、D3 每個火點都由狗處理完；D5 回起飛點；D7 機間最近距離 |
| `gazebo.mp4` | 俯視錄影：藍色平台 = 你插入的任務點、紅色方塊 = 火點 |
| `demo.mp4` | 俯視動畫：任務、指派、火警事件、電量長條 |
| `demo_plan.json` | 種子、範圍、你輸入過的指令（`manual`）、每個任務的 task_id 和位置 |

---

## 二、情境檔（`--scenario`）

事先寫好 JSON，放在 `docker/itest/scenarios/`：

```bash
python3 docker/itest/itest.py demo --scenario cbba_fire      # 檔名不用加 .json；也可以給完整路徑
```

### 格式

```json
{
  "tasks": [
    {"name": "A", "point": "G1", "duration": 20},
    {"name": "B", "x": 12, "y": -4, "at": 5},
    {"name": "C", "x": -6, "y": 10, "after_done": "A", "delay": 2}
  ],
  "fires": [
    {"name": "F1", "point": "R1", "at": 0}
  ],
  "timeout": 200,
  "exec_args": "-p cbf.gamma:=2.0"
}
```

| 欄位 | 說明 |
|---|---|
| `point` 或 `x`、`y` | 位置：場景的點名，或任意座標（任意座標的任務點會生成藍色平台） |
| `at` | 開始後幾秒插入（預設 0） |
| `after_arrival` | 某個任務的執行者到點（1 m 內）後才插入 |
| `after_done` | 某個任務回報完成後才插入 |
| `delay` | 條件達成後再等幾秒 |
| `duration` | 停留秒數（預設 5） |
| `creator` | 由哪台建立（預設輪流；建立者不影響誰得標） |
| `fires` | 起火點（不建任務），一樣可以用 `at`、`after_*` |
| `timeout` | 整個情境最長幾秒 |
| `exec_args` | 額外給無人機 BT 的參數（接在 `config.json` 的 `demo.exec_args` 後面） |

### 現有的情境

| 檔案 | 內容 |
|---|---|
| `cbba_overrun.json` | **G1～G3 都停 4 s、R4 (0, −8) 起火**：CBBA 照常把 G1、G2 給同一台，那台在 G1 看到火、監控到狗處理完，超時後 G2 交回、別台正對它飛過去（CBF）。手動插入時輸入 `f R4` 也是同樣的效果 |
| `cbba_fire.json` | A（G1，停 20 s）、B（G2）、C（G3）同時建立，R2 起火：CBBA 出價自然產生交會（CBF）＋目的地旁的火警 |
| `cbf_pass.json` | A 到點後才插入 B：保證交會，沒有火 |
| `cbf_fire.json` | 同上＋R2 起火 |
| `example.json` | 格式範例：指定座標、`at`、`after_done`、R1 起火 |

---

## 三、不經過 itest（手動開 SITL）

自己開 SITL（`docs/sim_launch_guide.md`）時，直接用腳本；**不會**進 itest 的報告、動畫。

| 要做的事 | 在哪個容器 | 指令 |
|---|---|---|
| 建立任務 | uavN（建立者那台） | `cbba_task.sh new N 流水號 x y [value] [停留秒數] [期限秒數]`，例：`cbba_task.sh new 1 1 12 -4 100 10 300` |
| 起火（不建任務） | sim | `CBBA_SCENARIO=demo_site patrol_scenario.sh fire-at x y 名稱` |
| 藍色平台（只標位置） | sim | `CBBA_SCENARIO=demo_site patrol_scenario.sh mark-at x y 名稱` |
| 清掉火點、平台、插入的 tag | sim | `CBBA_SCENARIO=demo_site patrol_scenario.sh reset` |
| 互動插入 R 點 | sim | `CBBA_SCENARIO=demo_site patrol_scenario.sh`（輸入 1～5、f1～f5…） |

`cbba_task.sh` 的流水號要自己避免重複（同一台、同一次 cbba_node 啟動期間）。

---

## 常見問題

| 狀況 | 原因 | 處理 |
|---|---|---|
| 輸入後沒反應 | CBBA 還沒上線前送的指令會先排著，上線後才處理 | 等 terminal 1 印出「CBBA 上線，開始接受指令」 |
| terminal 2 說「沒有在跑的 itest demo --interactive」 | terminal 1 沒加 `--interactive`、還沒開始，或已經結束 | 確認 terminal 1 的指令 |
| 「在範圍外」 | 座標超出 `area` | `a` 看範圍，或用 `--area` 放大（要在俯視錄影範圍內才拍得到） |
| 「範圍內找不到…的位置」 | 範圍太小、點太多 | 放大範圍，或手動給座標 |
| 任務一直「等待分配」 | 三台都在忙（例如懸停監控火點） | 等它們做完；或縮短 `fire_task.monitor_timeout` |
| 火點沒被發現 | 不在任何一台的航線旁、也不在懸停點附近 | 起火後插一個 `t` 在火點附近（> 3 m），或用 `tf` |
| 藍色平台沒出現 | Gazebo 是改程式前開的，找不到 `task_pad` 模型 | 重開 Gazebo（itest 會自己開；手動時重開 `px4_sitl.sh 1`） |
| `q` 之後一直沒結束 | 還有任務沒做完或狗還沒處理完 | 等完成，最長 `demo.interactive_timeout`（600 s）；沒被發現的火只會多等 `demo.fire_grace`（15 s） |
| 「已經有還沒做完的任務」 | 插入的位置 3 m 內有任務還沒完成 | 換個位置，或等它做完再插 |
| 做到一半無人機返航 | 模擬電池約 12 分鐘從 100% 降到 20%，BT 返航 | 一次手動測試控制在 10 分鐘內 |
