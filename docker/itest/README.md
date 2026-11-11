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
python3 docker/itest/itest.py loss --tasks 3                   # 每次 3 個任務（每台 1 個，等於 10/27 以前的量法）
python3 docker/itest/itest.py loss --dogs 3                    # 空地（2026-11-09）：3 架無人機＋3 隻狗，每次 12 個任務（空中 6、火警＋巡檢 6）
python3 docker/itest/itest.py comm --dogs 3 --only C9          # 100% 斷網，空地各自退化成單機
python3 docker/itest/itest.py demo --world yard --scenario blackout --seed 7 --dog bt   # 100% 斷網的任務退化（真的 BT，報告 B1～B4）
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
| C9 | 100% 斷網（週計畫第 5 週） | 各台拿著身邊的任務 → 全部斷網 6 s：失聯後手上的照樣拿著、斷網中回報完成本機就確認結束、斷網中自己建的任務自己接（單機貪婪）；恢復後全隊都知道那些完成、知道新任務而且在原本那台，沒有重複認領、完成的沒再派。只跑這項：`--only C9` |

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
python3 docker/itest/itest.py demo --world yard           # 伯宇的變電所場景（100 × 80 m；出生在起降板 (−40, −30)、巡檢高度 8 m，2026-10-24）
python3 docker/itest/itest.py demo --world yard --scenario manual_fire --seed 7   # yard 上跑人工指派＋火警＋插入
python3 docker/itest/itest.py demo --world yard --scenario assign_random --seed 7 # G1～G3 固定任務＋M1 指定做 G 的那台（路上或旁邊隨機起火）＋T1 放在 M1 懸停那台的後方（閒著的那台航線經過它，看避碰；報告 X1）＋T2 隨機插入（那裡也起火）；人工指派畫面上用 M1、M2… 表示（2026-10-24）
python3 docker/itest/itest.py demo --scenario example    # 自訂情境的範例：自己排任務點、起火點、插入時機（格式見 docker/itest/scenarios/example.json 的 _說明）
python3 docker/itest/itest.py demo --interactive --area -15 15 -12 8   # 手動插入（terminal 1 開環境）；terminal 2：python3 docker/itest/insert.py 輸入 t [x y] [d=秒]、tf、f、m（人工指派某台）、l、q（說明見 docs/task_insertion_guide.md）
#   --speed 3（無人機巡航 m/s，給 CBBA 出價和 BT；預設 3，2026-10-24。不是 0.5 時 itest 照速度算 cbf.gamma、
#              cbf.influence_radius、cbf.miss_clearance 給 BT（itest.py 的 cbf_speed_args；3 m/s → 1.5、8.6 m、4.4 m））  --hold 10（完成後停幾秒）  --no-record（不錄 Gazebo）
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
- **假狗**（`config.json` 的 `dog`；2026-10-20 起 **3 隻**，`dog.units`；2026-10-24 起名稱照伯宇的狗）：v60（機號 50）在 (−4, 0)、v60_1（51）在 (10, 6)、
  v60_2（52）在 (−18, −8)（yard 場景照伯宇的出生點：(−35, 7.5)、(−35, −7.5)、(−15, 25)），分散在場地三側，看狗之間對火警的競標。每隻的 cbba_node（UGV、`/<ns>`，`cbba_v60.yaml`）借 uav1 容器跑
  （要在 mesh 上）；執行者 `docker/scripts/fake_dog.py` 在 sim（收 `/<ns>/current_task`、直線走過去、停留、回報 `task_result`；
  Gazebo 放一個橘色方塊 `<ns>_standin` 當替身）。速度 1.5 m/s（兩邊一致）。紀錄在 `fake_dog_<ns>.log`、`dog_cbba_<ns>.log`。
- **`--dog bt`：伯宇的狗 BT＋方塊身體**（2026-10-27）：執行者換成伯宇的 `task_manager`（handoff 原檔，不改；uav1 容器，`dog_bt.sh start`），
  sim 只放方塊身體 `docker/scripts/fake_nav_body.py`：假的 Nav2（`/<ns>/navigate_to_pose`，直線走、cancel 立刻停）、
  假雲台（`/<ns>/ptz_inspect`，等 `dog.ptz_sec` 5 s 回 OK）、TF `map → <ns>/base_link`。驗證他的 BT 本身（哨兵中斷、
  `★ halt → cancel Nav2 goal` 的中斷延遲、回報恰好一次、平板接管、急停）；Nav2 的規劃、繞障礙不在範圍內。
  和假狗不同：停在目標前 1.5 m（standoff）、處置時間是雲台的 5 s（不看 `duration_sec`）、robot_state 2 Hz。
  **巡邏**（2026-11-05）：場景有 `worlds.<場景>.dog_patrol`（yard：伯宇 README_WORLD.md 的三條巡檢線）時，itest 產生
  `dog_mission_<ns>.yaml`（他的任務檔格式、`policy.loop: true`）給 `dog_bt.sh start <ns> <機號> <任務檔>`：狗一直巡邏、
  邊走邊看火（`demo.dog_detect`），CBBA 派地面任務時插隊、做完接回巡邏。隨機的火只放在狗巡檢線或無人機航線看得到的範圍
  （`demo.fire_in_coverage`）。
  第一次要先在 uav1 容器 `dog_bt.sh build`（編到 `dog_ws/`，不碰共用的 `ros2_ws/build`、`install`）；映像要重建
  （`robot.Dockerfile`、`sim.Dockerfile` 加了 `ros-jazzy-nav2-msgs`）。紀錄在 `dog_body_<ns>.log`、`dog_bt_<ns>.log`、`dog_cbba_<ns>.log`。
  報告 D3 會寫每個火點由哪隻狗處理；`units` 只留一隻就是原本的單隻狗。
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
| D3 | 假狗處理完（地面任務的 `task_closed` status DONE），而且沒有無人機被指派地面任務；列出由哪隻狗處理 |
| D4 | 發現的那台在地面任務結束之後才回報 R 點完成，監控期間離火點 < 2 m。先發現的是別台時（例如待命、回程途中看到，2026-10-19）：發現的那台監控到地面任務結束（離火點 < 2 m），R 點有完成即可（被派去的那台照規則不重複監控） |
| D5 | 全部做完後每台離起飛點 < 1 m、高度在巡檢高度 ±0.8 m |
| D6 | `demo.mp4`（俯視動畫）、`gazebo.mp4`（Gazebo 俯瞰攝影機）、`gazebo_labeled.mp4`（2026-11-05，`label_gazebo.py`：gazebo.mp4 加上 uav1～N、dog1～N、任務 G 綠／M 藍／T 黃／F 紅（做完變灰打勾）、載具到任務的連線、模擬時間、事件字幕、圖例；照 ros_bridge 的 clock 事件對齊模擬時間；舊的輸出可以 `python3 docker/itest/label_gazebo.py <目錄> --t0 … --t1 …` 手動補）、`demo_final.png`（最後一格）、`demo_cameras.mp4`（左：gazebo.mp4 縮成 1024 × 576；右：三台的下視相機各 256 × 192，掃 AprilTag 的那台；總寬 1280，2026-10-20） |
| D7 | CBF：任務開始到回起飛點，空中兩台的最近水平距離 ≥ 0.9 × `cbf.safe_radius`（從 `exec_args` 或 `uav_bt.yaml` 讀）；列出 BT 記到的每次接近（「避碰：這次最接近鄰機」） |

俯視動畫：一開始就畫出固定任務點（綠色）；起火時火點才出現（紅色方塊＋光圈）；各台的航跡（每 4 m 一個方向箭頭）、
目前要去的任務（虛線）；任務旗子未分配是白的、有人拿著時填那台的顏色、完成後變灰；地面處置任務是紅色方塊。
右邊是事件時間軸和電量長條（0～100 %，長條長度就是電量，旁邊是數值）。全程 1 倍速（2026-10-19 起，原本沒有事件時 3 倍速），起火、丟出地面任務、地面任務結束時停 1.5 s。自己重畫：

```bash
./docker/enter.sh sim
python3 /home/ncrl/CBBA_BT/docker/itest/animate_demo.py /home/ncrl/CBBA_BT/docker/itest/out/<時間>_demo --frames /home/ncrl/CBBA_BT/docker/itest/out/<時間>_demo/frames
#   不給 --frames 時存 demo.gif；--freeze 1.5 --fps 15；預設全程 1 倍速，要快轉給 --speedup 3（事件前後 --window 2 秒維持 --slow 1 倍速）
```

新的模擬端測試（2026-10-20，`--scenario manual_fire`）：起飛 → 出價 → G1～G3 → 隨機挑一台正在做 G 的無人機人工指派到隨機的 M
（交還 G、別台接手）→ 同時在它飛往 M 的路上或 M 旁隨機起火（照火警 BT 處理，M 延遲）→ 之後隨機插入 T（由沒任務或做完的那台做）。
跑的時候 terminal 即時印「⚠ 地面站通知」（交還 G、改由誰接手、M 因火警延遲／繼續／完成，給地面站或平板使用者看）。報告多了 M1（指派、G 交還與接手）、M2（看到火 → 地面任務 → 狗處理完 → M 延遲多久）、M3（T 由誰做、當時在做什麼），都是 INFO 紀錄，不判對錯。
換 `--seed` 換一組位置與時間。

出價表（2026-10-20）：demo 結束時 `bid_table.py` 收集各台 cbba_node（無人機 `stack_uavN.log`、狗 `dog_cbba_<ns>.log`）的
「出價表」log，輸出 `bids.md`（一、每個任務 × 每一台第一次知道時的出價、最後得標者；二、每台得到、失去任務的時刻）、`bids.svg`（長條圖：橫軸任務、縱軸每一台的初次出價，黑框＋★ = 最後得標，瀏覽器開）和
`bids.csv`（全部紀錄，Excel 可直接開）。cbba_node 只在任務第一次知道、得標者改變、這台的出價狀態改變時印，飛行中出價隨位置變動不印。
沒出價時寫原因：不能做（能力不符、來不及、電量不夠）、失敗後排除中、別台已接受、不參與出價…。自己重做：
`python3 docker/itest/bid_table.py docker/itest/out/<時間>_demo`（主機，只用標準庫）。

**loss：第 5 週弱網量測**（不開 PX4、BT；2026-10-12；2026-10-27 改成多任務、比對理論值）

**`--dogs N`（comm、loss，2026-11-09）**：加 N 隻狗（機號 50～，`/v60`…），每隻在自己的容器 `cbba_dogK`（`docker/.env` 的 `DOG_COUNT`，`gen_compose.py` 產生；同一個 robot 映像、沒有 PX4，只跑 `cbba_v60.yaml` 的 cbba_node），有自己的 mesh 網卡，`mesh_tc.sh` 也套在狗的容器上，空地之間每條連線都經過 tc。位置在 `config.json` 的 `comm.dogs`。loss 的任務改成混合：無人機身邊空中任務（1）、狗身邊輪流火警（2，建立者是無人機）和巡檢點（3）；理論執行者＝能力相符的最近那台；多記「跨能力」（空中任務出現在狗、地面任務出現在無人機的路徑或 current_task，應該永遠 0，30% 時不是 0 算 FAIL）。comm 的 C1～C8 照舊只測無人機（狗是旁觀的鄰居），C9 的 100% 斷網包含狗（狗身邊是火警、斷網中自己建巡檢）。`trials.csv` 多 `agents`、`cross_type`、`ground_tasks` 欄，`plot_tasks.py` 照 `agents` 標名稱。

每個掉包率（`--levels`，預設 0,10,30,50,70；延遲 `--delay` 50 ± `--jitter` 30 ms，每台 uav 的 mesh 送出方向）
試驗 `--trials` 次（預設 20）。每次建立 `--tasks` 個任務（預設 9 個）：第 j 個放在第 j % 3 台身邊 2.5 m（角度每次輪換），
由**別台**建立（多源插單，要靠協商）。別台的任務遠 20 m，所以**理論上每台分到身邊的 3 個、而且每台都知道全部 9 個**：

| 指標 | 定義 |
|---|---|
| A 送達 | 每台知道幾個任務（cbba_node 的 `task_announced`）；「全部送到」= 每台都知道全部，記時間；`known_per_agent` 是每台知道幾個 |
| B 分配 | 每台路徑（cbba_node 的 `cbba_path`，2026-10-27 加的話題）裡有幾個這次的任務，和理論值比（`expected_per_agent` 3/3/3、`got_per_agent`）；`wrong` = 不在理論執行者路徑裡、或在兩台以上路徑裡的任務數；「正確時間」= 建立 → 每個任務都在理論執行者的路徑裡 |
| 收斂時間（無衝突） | 建立 → 每個任務恰好在一台的路徑裡（**不管分得對不對**，對不對看 B），之後 1 s 不變 |
| 重複認領 | 這次試驗裡同一個任務被兩台以上當成 current_task 拿過（真的派給兩台的 BT） |
| 5 s 收斂 | 收斂時間 ≤ 5 s 而且沒有重複認領。**L30 的通過條件：≥ 95%**（週計畫）；其他掉包率、A、B 只記錄（INFO） |
| 完成 → 本機撤下 | 每個任務：測試回報完成 → 執行那台的 `current_task` 不再是它（每次試驗取中位數） |
| 完成 → 全隊確認 | 每個任務：回報完成 → **三台都發了 `task_closed`**（COMPLETION → ACK → TASK_CLOSE 走完；中位數、P90）。這才是「完成」 |
| 做不完 | 時限（30 s ＋ 每個任務 5 s）內沒有被全隊確認結束的任務數；有的話這個掉包率停止 |
| 完成後又派 | 回報完成的任務之後又出現在某台 `current_task` 的次數（應該是 0；持有者當掉、失聯接手時會有） |
| mesh 頻寬 | 每個掉包率從第一次到最後一次試驗，各台 mesh 網卡收＋送的平均 kB/s |

每次試驗等到「全部送到而且分配正確」或 10 s。每台最多接 `max_bundle`（5）個，`--tasks` 超過 15 時理論值不成立。

試驗隔離（2026-10-27）：
- 下一次試驗開始前，等上一次的任務**全隊都確認結束**、三台的 `current_task` 和路徑都空，再等 3 s（最多 30 s，等不到的試驗 `clean = 0` 不計）。
  原本只等 `current_task` 空，70% 時 19 次有 11 次開始時還有上一次的任務。
- **每個掉包率重新啟動三台 cbba_node**（已結束的任務會一直記著，不帶到下一個掉包率）；log 分開：`node_uavN.log`（第一個掉包率）、
  `node_uavN_L30.log`…。`--no-restart` 恢復原本整輪不重開（要重現長時間累積的問題時用）。
- 每次試驗後檢查三台的 cbba_node 還在；不在時停止這個掉包率、報告 `LX` FAIL（2026-10-27 遇過 70% 時 Segmentation fault）。

追當機（2026-10-28）：cbba_node 自己裝了當機處理，收到 SIGSEGV 等訊號時把呼叫堆疊印到 log
（`===== cbba_node 當掉`），平常不影響時序，**不用加 `--gdb`**（gdb 會改變時序，10/28 在 gdb 底下跑 100 次沒重現）。
位址換成行號：`addr2line -f -C -e $(ros2 pkg prefix swarm_cbba)/lib/swarm_cbba/cbba_node <+0x 位移>`，
要用 `--cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` 編 `cbba_core`、`swarm_cbba` 才有行號。
`--gdb` 留著（節點在 gdb 底下跑），當機處理拿不到堆疊時再用。

每次試驗從「三台都沒有任務、再等 3 s」開始（掉包時全隊確認完成要 1～3 s），收尾把這次的任務**全部**做完
（含排在路徑後面、之後才輪到的）。開始時還有任務的試驗不計（`trials.csv` 的 `clean = 0`）；
收尾 30 s ＋ 每個任務 5 s 內做不完時（`leftover > 0`）停止量測、報告 FAIL，因為留下的任務會卡住之後的每一次試驗
（2026-10-12 第一次量測就是這樣：30% 以上全部「未收斂」，是測試的收尾漏掉排隊的任務，不是協定的問題）。

輸出：`report.md`（每個掉包率一列）、`trials.csv`（每次試驗一列）、`levels.csv`（每個掉包率的統計）。畫圖要 matplotlib，
主機沒有時在 sim 容器裡跑（沒有中文字型時自動用英文標籤）：

```bash
./docker/enter.sh sim
python3 /home/ncrl/CBBA_BT/docker/itest/plot_loss.py /home/ncrl/CBBA_BT/docker/itest/out/<時間>_loss   # → loss_sweep.png
```

四張圖：5 s 內收斂率、收斂時間、完成 → 全隊確認、mesh 頻寬。可以給多個目錄合併（例如分幾次量完）。
另外 `plot_tasks.py`（同樣的用法 → `loss_tasks.png`）：每台平均分到幾個任務（誤差線＝最少～最多）、每次試驗的分配結果
（和理論相同／不平均但全部做完／有任務沒做完）。

### 比較 cbba_node 參數（`--node-param`，2026-11-04）

`--node-param KEY=VALUE`（可以給多次）會加到每台 cbba_node 的 `-p KEY:=VALUE`，記在 `trials.csv`、`levels.csv` 的
`node_params`。每組參數各跑一次，再把目錄一起給 `plot_loss.py`：參數不同的各畫一條線（最多 4 組）。
例如第 5 週的補發、完成確認的間隔（`resend_gap`、`completion_retry` 都是 1.0 s；高掉包時
「完成 → 全隊確認」約等於 `completion_retry × completion_max`）：

```bash
python3 docker/itest/itest.py loss --levels 30,50,70                                              # 預設 1.0 s
python3 docker/itest/itest.py loss --levels 30,50,70 --node-param resend_gap=0.5 --node-param completion_retry=0.5
python3 docker/itest/itest.py loss --levels 30,50,70 --node-param resend_gap=0.3 --node-param completion_retry=0.3
# sim 容器：python3 …/plot_loss.py <目錄1> <目錄2> <目錄3>
```

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
