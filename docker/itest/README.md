# 整合測試（itest）

判定只看**話題和網路**（例外：demo 的 D7 另外讀 BT 的 log，列出每次接近鄰機的距離），所以換成別人寫的 CBBA Node／BT 也能用：
介面（話題名稱、訊息型別、欄位、QoS、啟動指令）都寫在 `config.json`，對方的介面不同時只改這份。

```
主機：itest.py ──docker exec──► 各容器啟動／關掉節點、PX4、tc
         │  commands.jsonl ▲ events.jsonl（docker/itest/out/<時間>/，主機和容器都掛載）
         ▼                 │
   sim 容器：ros_bridge.py（經線材網路連得到每一台的 DDS）：收各台的話題、幫主機發話題
```

## 用法

容器要先開好（`docker/start.sh`）、要測的套件要先編譯好，在**主機**執行：

```bash
cd ~/CBBA_BT
python3 docker/itest/itest.py comm     # CBBA 之間的通訊（不開 PX4、BT），約 2 分鐘
python3 docker/itest/itest.py uav      # UAV 整串（PX4 SITL＋Gazebo＋CBBA Node＋BT），約 8～10 分鐘
python3 docker/itest/itest.py uav --executor bt   # 同一套 U1～U6，BT 換成備用的 uav_bt_node（2026-10-13 起預設 task_manager）
python3 docker/itest/itest.py comm --only C1,C3     # 只跑某幾項
python3 docker/itest/itest.py demo     # 展示（週計畫）：開 Gazebo 畫面，巡檢點起火 → 無人機發布地面任務、懸停監控 → 假狗處理 → 回起飛點，錄影＋俯視動畫 MP4
python3 docker/itest/itest.py loss     # 第 5 週弱網量測：掉包 0／10／30／50／70% 各 20 次，約 15～25 分鐘
python3 docker/itest/itest.py loss --levels 30 --trials 50     # 只量 30%、50 次
```

容器裡已經有在跑的節點、PX4、Gazebo 時會停下來，避免打斷你正在做的事；加 `--kill-existing` 先關掉再跑。
結果在 `docker/itest/out/<時間>_<組>/report.md`，各節點的輸出（`node_uavN.log`、`stack_uavN.log`、`px4_N.log`）也在那裡。

## 測試項目

**comm：CBBA 之間的通訊**（每台在自己的容器啟動 CBBA Node，位置由測試發假的 robot_state）

| 項目 | 測什麼 | 通過的條件 |
|---|---|---|
| C0 | 節點上線 | 每台的 assigned 話題都收得到 |
| M | mesh 流量 | 只記錄（待命時每秒多少） |
| C1 | 任務傳到全隊、只有一台得標 | 5 s 內恰好一台拿到，之後 2 s 沒有別台拿過 |
| C2 | 回報完成 | 3 s 內撤下，之後不再派給別台 |
| C3 | 失敗交回 | 5 s 內別台接手 |
| C4 | 停止送 robot_state（等同 PX4 斷線） | 10 s 內別台接手 |
| C5 | 節點整個停止 | 40 s 內別台接手（失聯接手：火警 10 s、巡檢 30 s） |
| C6 | 重開機後重新加入 | 重開的那台拿到身邊的新任務，不重複 |
| C7 | 掉包 30%、延遲 50±30 ms | 10 s 內恰好一台拿到；完成後撤下 |
| C8 | 斷網 | 斷網時只有建立者拿到（證明協調走 mesh）；恢復後不重複 |

**uav：UAV 整串**（`fire_site` 場景）

| 項目 | 測什麼 | 通過的條件 |
|---|---|---|
| U1 | 起飛 | 每台 armed、到巡檢高度（150 s 內） |
| U2 | 單一任務 | 指派 → 飛到 1.5 m 內 → task_result 成功 |
| U3 | 火警 | 到達火警點後出現地面處置任務（new_task），位置誤差 ≤ 1.5 m |
| U4 | 誘餌 | 到達誘餌點後**沒有**地面任務 |
| U5 | 多台分配與避碰 | 3 個任務全部成功、機間最近距離 ≥ 1.5 m |
| U6 | 飛行中 PX4 斷線 | 15 s 內別台接手並完成 |

**demo：展示**（`demo_site`，**開 Gazebo 畫面**；2026-10-12，週計畫流程 2026-10-13）

```bash
python3 docker/itest/itest.py demo                 # 隨機種子（報告和終端機會印出來）
python3 docker/itest/itest.py demo --seed 42       # 同一個種子 = 同樣的火點
python3 docker/itest/itest.py demo --fire-mode side  # 航線旁起火（R1 在 G3 航線旁）：路過發現 → 轉向火點上空監控 → 假狗處理完 → 回航線完成原任務（D4 檢查這個順序）
python3 docker/itest/itest.py demo --scenario cbf_pass   # CBF：A 在 G1 懸停 25 s，到點後插入 G2，飛 G2 的那台去程、回程都經過它（D7 看最近距離）
python3 docker/itest/itest.py demo --scenario example    # 自訂情境的範例：自己排任務點、起火點、插入時機（格式見 docker/itest/scenarios/example.json 的 _說明）
python3 docker/itest/itest.py demo --interactive --area -15 15 -12 8   # 手動插入（terminal 1 開環境）；terminal 2：python3 docker/itest/insert.py 輸入 t [x y] [d=秒]、tf、f、l、q（說明見 docs/task_insertion_guide.md）
#   --speed 3（無人機巡航 m/s，給 CBBA 出價和 BT）  --hold 10（完成後停幾秒）  --no-record（不錄 Gazebo）
#   --executor bt（用備用 BT；沒有懸停監控、回起飛點，D4、D5 會失敗）
```

- 場景 `demo_site`：展示用的小場景（點在 ±22 m 內，`docker/gz/worlds/demo_site_points.csv`），鏡頭拉近，無人機看得清楚；
  `patrol_site`（±40 m）不動，離線的電池權重實驗用它的座標。
- 載入：第 1 台 PX4 開場景，場景一開好就接著開第 2、3 台（間隔 2 s），三台一起出現在畫面上。
- 起飛：全部 PX4 都在送位置後才同時讓 BT 啟動（`cbba_uav.sh` 的 `CBBA_TAKEOFF_GATE`，沒有 BT 就不會解鎖），**三台一起起飛**。
- CBBA：**三台都起飛到巡檢高度後**，懸停 3 s，才同時啟動三台的 cbba_node 和假狗（`CBBA_START_GATE`），連上後再等
  `start_delay`（6 s）才建立任務。
- **流程（`fire_mode` point，週計畫，預設）**：依 `--seed` 挑一個 R 點（`fire_points`）起火：紅色方塊＋tag 101
  （`CBBA_SCENARIO=demo_site patrol_scenario.sh fire-at`），和 G1～G3 一起建立巡檢任務（停留 4 s），CBBA 分給三台。
  飛到 R 點的那台懸停時看到 tag 101 → 發布地面處置任務 → **假狗**得標、走過去、處理 20 s、回報 → 那台**懸停監控到地面任務結束**
  （cbba_node 的 `task_closed`）才完成 R 點 → 全部做完後**各自飛回起飛點上空**（BT 的 `return_home_delay` 2 s）。
- `fire_mode` route（2026-10-12 的版本，改 `config.json`）：各台出發後在某台前方航線上起火，看飛行途中發現、發布地面任務、繼續飛。
- **假狗**（`config.json` 的 `dog`）：cbba_node（機號 50、UGV、`/v60`，`cbba_v60.yaml`）借 uav1 容器跑（要在 mesh 上）；
  執行者 `docker/scripts/fake_dog.py` 在 sim（收 `/v60/current_task`、直線走過去、停留、回報 `task_result`；
  Gazebo 放一個橘色方塊當替身）。速度 1.5 m/s（兩邊一致）。紀錄在 `fake_dog.log`、`dog_cbba.log`。
- **Gazebo 錄影**（2026-10-13 改）：場景裡放一台固定的俯瞰攝影機（`overview_camera`，`CameraVideoRecorder` 系統，
  `gen_patrol_site.py` 產生），由 Gazebo **伺服器**錄，不用開視窗、不用按按鈕；場景開好就開始，完成後停 `--hold` 秒再停止，
  存成 `gazebo.mp4`（1280×720、15 fps）。原本視窗的 VideoRecorder 按鈕要呼叫 `/gui/record_video`，
  gz-sim 8＋MinimalScene 沒有人提供這個服務，一直錄不到，已經從 `cbba_gui.config` 拿掉。手動錄：
  `gz service -s /demo_site/overview/record_video --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 --req 'start: true, format: "mp4", save_filename: "/home/ncrl/CBBA_BT/x.mp4"'`（停止：`--req 'stop: true'`）。
- 俯視動畫：sim 容器畫每一格（`animate_demo.py --frames`），**主機的 ffmpeg**（`sudo apt install ffmpeg`）合成 `demo.mp4`；
  沒有 ffmpeg 時每一格留在 `frames/`，終端機會印出合成指令。

| 項目 | 通過的條件 |
|---|---|
| U1 | 每台起飛到巡檢高度 |
| D1 | 巡檢任務（G1～G3＋起火的 R 點）都完成（`timeout` 240 s 內） |
| D2 | 看到火警、發布一個地面處置任務（位置誤差 ≤ 2 m）；地面任務總數 = 火點數 |
| D3 | 假狗處理完（地面任務的 `task_closed` status DONE），而且沒有無人機被指派地面任務 |
| D4 | 發現的那台在地面任務結束之後才回報 R 點完成，監控期間離火點 < 2 m |
| D5 | 全部做完後每台離起飛點 < 1 m、高度在巡檢高度 ±0.8 m |
| D6 | `demo.mp4`（俯視動畫）、`gazebo.mp4`（Gazebo 俯瞰攝影機）、`demo_final.png`（最後一格） |
| D7 | CBF：任務開始到回起飛點，空中兩台的最近水平距離 ≥ 0.9 × `cbf.safe_radius`（從 `exec_args` 或 `uav_bt.yaml` 讀）；列出 BT 記到的每次接近（「避碰：這次最接近鄰機」） |

俯視動畫：一開始就畫出固定任務點（綠色）；起火時火點才出現（紅色方塊＋光圈）；各台的航跡、
目前要去的任務（虛線）；任務圓圈未分配是白的、有人拿著時填那台的顏色、完成後變灰；地面處置任務是紅色方塊。
右邊是事件時間軸。事件前後 2 s 用 1 倍速、其他 3 倍速，起火和丟出地面任務時停 1.5 s。自己重畫：

```bash
./docker/enter.sh sim
python3 /home/ncrl/CBBA_BT/docker/itest/animate_demo.py /home/ncrl/CBBA_BT/docker/itest/out/<時間>_demo --frames /home/ncrl/CBBA_BT/docker/itest/out/<時間>_demo/frames
#   不給 --frames 時存 demo.gif；--speedup 3 --slow 1 --window 2 --freeze 1.5 --fps 15
```

**loss：第 5 週弱網量測**（不開 PX4、BT；2026-10-12）

每個掉包率（`--levels`，預設 0,10,30,50,70；延遲 `--delay` 50 ± `--jitter` 30 ms，每台 uav 的 mesh 送出方向）
試驗 `--trials` 次（預設 20）。每次三台**各建立一個任務**（多源插單），任務放在**別台**身邊（要靠協商），每次輪換：

| 指標 | 定義 |
|---|---|
| 收斂時間 | 建立 → 每個任務恰好一台持有、各台持有的不同（current_task 只看得到路徑第一個，所以任務數 = 台數），之後 1 s 不變 |
| 重複認領 | 這次試驗裡同一個任務被兩台以上拿過 |
| 5 s 收斂 | 收斂時間 ≤ 5 s 而且沒有重複認領。**L30 的通過條件：≥ 95%**（週計畫）；其他掉包率只記錄（INFO） |
| 完成 → 撤下 | 持有者回報完成到全部撤下；「完成後又派」是之後又派給別台的次數（應該是 0） |
| mesh 頻寬 | 每個掉包率從第一次到最後一次試驗，各台 mesh 網卡收＋送的平均 kB/s |

每次試驗從「三台都沒有任務、再等 3 s」開始（掉包時全隊確認完成要 1～3 s），收尾把這次的任務**全部**做完
（含排在路徑後面、之後才輪到的）。開始時還有任務的試驗不計（`trials.csv` 的 `clean = 0`）；
收尾 30 s 內做不完時（`leftover > 0`）停止量測、報告 FAIL，因為留下的任務會卡住之後的每一次試驗
（2026-10-12 第一次量測就是這樣：30% 以上全部「未收斂」，是測試的收尾漏掉排隊的任務，不是協定的問題）。

輸出：`report.md`（每個掉包率一列）、`trials.csv`（每次試驗一列）、`levels.csv`（每個掉包率的統計）。畫圖要 matplotlib，
主機沒有時在 sim 容器裡跑（沒有中文字型時自動用英文標籤）：

```bash
./docker/enter.sh sim
python3 /home/ncrl/CBBA_BT/docker/itest/plot_loss.py /home/ncrl/CBBA_BT/docker/itest/out/<時間>_loss   # → loss_sweep.png
```

可以給多個目錄合併（例如分幾次量完）。

## 換成別人的 CBBA Node／BT 時

| config.json | 改什麼 |
|---|---|
| `topics` | 話題名稱、訊息型別、QoS（`{ns}` = 命名空間） |
| `fields` | assigned 訊息裡任務編號、指派版本的欄位名稱；沒有版本就把 `version` 設成 `""` |
| `messages` | 測試發的 new_task、task_result、robot_state 的內容（`{task_id}` 等會換成數值） |
| `comm.start`、`comm.stop_pattern` | 在 uav 容器啟動 CBBA Node 的指令、關掉時 `pkill -f` 比對的字串 |
| `uav.start`、`uav.stop_pattern` | 每台的 CBBA Node＋BT＋轉接（要在背景一直跑） |
| `paths.ros_setup` | 容器裡要 source 的環境 |

訊息型別要有 `swarm_interfaces` 的定義才收得到；對方用別的訊息套件時，`ros_setup` 要包含那個套件。
