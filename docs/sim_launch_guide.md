# 模擬測試的開啟方式

2026-10-12 更新 · 無人機端（勝翔）。把散在 `docker/README.md`、`cbba_core/doc/sitl_test.md`、`ugv_test.md`、
`docs/sim_scenario_plan.md` 的步驟整理在一起；各測試的細節、完整的預期 log 還是看原本的文件。

| 測試 | 用途（週計畫） | 要開的東西 |
|---|---|---|
| [A. 基本分配](#a-基本分配task_executor) | CBBA 分配、完成、失敗、墜毀（第 1～2 週） | Gazebo 預設場景＋task_executor |
| [B. 火情偵測](#b-火情偵測fire_site) | AprilTag 101 → 地面處置任務（第 3 週） | `fire_site`＋BT＋相機＋（可選）假狗 |
| [C. 巡檢＋隨機插入](#c-巡檢隨機插入patrol_site) | terminal 輸入編號插入、電池權重抽驗 | `patrol_site`＋BT |
| [D. 弱網](#d-弱網mesh_tc) | 手動加掉包（第 5 週） | 任一個上面的測試＋`mesh_tc.sh` |
| [E. 離線模擬](#e-離線模擬不開-gazebo) | 電池權重 100 次 | 只要 `cbba-robot` 映像 |
| [F. 自動整合測試](#f-自動整合測試itest) | 通訊（C0～C8）、UAV 整串（U1～U6）、掉包梯度量測（loss），自動判斷 | 主機執行 `docker/itest/itest.py` |

---

## 0. 準備

### 第一次，或改了 `.env`、docker 設定

```bash
# 主機
cd ~/CBBA_BT/docker
vi .env                 # UAV_COUNT（預設 3）、PX4_SIM_MODEL=gz_x500_mono_cam_down（有下視相機）、DDS_OVER_MESH（預設 0）
./start.sh              # 產生 docker-compose.yaml，重建、啟動 sim、uav1～N、gcs（容器裡在跑的東西會中斷）
```

### 第一次，或改了 C++、訊息、yaml、launch

```bash
./enter.sh uav1         # 進容器：./enter.sh sim | uav1 | uav2 | … | gcs
# uav1 容器（所有容器共用同一個 ros2_ws，只要編一次；不要兩個容器同時編譯）
cd ~/CBBA_BT/ros2_ws
colcon build --symlink-install --packages-select \
  swarm_interfaces cbba_core swarm_cbba px4_waypoint_node uav_px4_bt apriltag_fire_detector
source install/setup.bash     # 已經開著的 terminal 都要重新 source（或重開）
```

- 6 個套件要一起編（`swarm_interfaces` 變了時只編一部分，DDS 會靜默收不到）。
- 切換 git 分支後一定要重新編譯（各分支共用 `build/`、`install/`）。
- 只改了 Python、shell 腳本（`itest.py`、`plot_loss.py`、`docker/scripts/*.sh`）不用編譯、不用 `start.sh`。

---

## 共通規則

**指令在哪個容器下**（DDS 只在機內，`.env` 的 `DDS_OVER_MESH=0`，2026-10-12 起）：

| 指令 | 在哪裡 |
|---|---|
| `cbba_uav.sh` | 每台各自的 `uavN` 容器 |
| `cbba_task.sh <cmd> N …`（new、ground、done、fail、cancel、watch）、`ros2 topic echo /uavN/…` | **uavN 自己的容器**（或 sim）。在別台的容器下會一直等訂閱者，看起來像沒反應 |
| `px4_sitl.sh N`、`gz_bridge.sh`、`patrol_scenario.sh` | `sim`（sim 經線材網路連得到每一台） |
| 假狗（狗的 cbba_node＋假 BT＋echo） | `gcs`（全部在 gcs 裡；和無人機之間只走 mesh 的 UDP） |
| `mesh_tc.sh` | 每台 `uavN`（和 gcs，有開假狗時） |
| `itest.py`、`./start.sh`、`./stop.sh` | **主機** |

從 gcs 看不到各台無人機的話題。真的要看時 `.env` 設 `DDS_OVER_MESH=1` 再 `./start.sh`（PX4 的話題會被 tc 影響）。

**啟動順序**：每台的 `cbba_uav.sh`（含 XRCE Agent）要比同號的 `px4_sitl.sh N` 先開；`px4_sitl.sh 1` 負責開 Gazebo，
之後的台自動加入同一個場景；`gz_bridge.sh` 要在 Gazebo 開好之後。

**共用的環境變數**（sim 和同號的 uav 兩邊要一樣）：

| 變數 | 預設 | 用在 |
|---|---|---|
| `SIM_BAT_DRAIN` | 900 | PX4 模擬電池的續航（秒）；`cbba_uav.sh` 用它算能量模型。要各台電量不同就各台設不同值 |
| `SIM_BAT_MIN_PCT` | 20 | PX4 模擬電池最低停在這裡（= BT 返航門檻） |
| `CBBA_SPEED`、`CBBA_ALTITUDE` | 5 m/s、5 m | cbba_node 的出價和執行者的飛行一致 |
| `CBBA_WORLD` | （PX4 預設場景） | `fire_site`、`patrol_site`：只有第 1 台要設 |
| `EXECUTOR` | `bt` | `bt` = `uav_px4_bt`＋`apriltag_fire_detector`；`task_executor` = 舊的簡易執行者 |

**每台開好之後要看到**（細節見 `sitl_test.md` 第 1 節）：`[bt] 到達 5.0 m，等待指派` →
`[state] 起飛完成（4.5 m）：mode AUTO（可以接任務）` → `[cbba] 開始參與出價`（爬到巡檢高度才出價） → `[cbba] 收斂：0 個任務，N−1 個鄰居`。

---

## A. 基本分配（task_executor）

1. `uav1`～`uavN` 各開一個 terminal：`EXECUTOR=task_executor cbba_uav.sh`
2. `sim`：`px4_sitl.sh 1`；再開 terminal `px4_sitl.sh 2`、`3`…
3. 等每台「開始參與出價」。
4. `uav1`：`cbba_task.sh new 1 1 10 5`（uav1 建立任務 00010001 在 (10, 5)）

要看到：得標那台 `[cbba] 指派給 BT：00010001` → 飛到、停留後回報 → `已確認完成（回報後 x s）`；其他台
`已確認完成（收到完成證明）`。之後照 `sitl_test.md` 第 4～6 節（`cbba_task.sh fail`、取消、關掉一台 PX4 或整個容器）；
`fail`、`done` 要在**執行那台**的容器下。

task_executor 沒有避碰：同高度會撞機，可以每台 `EXEC_ARGS="-p altitude:=5.0／7.0／9.0"` 分層。

## B. 火情偵測（fire_site）

場景：火警紅色方塊 (14, −6) 貼 tag 101、誘餌 (−10, −8) 貼 tag 7、巡檢點 (12, 12)、(−12, 10)、(0, −16)。

1. `uav1`～`uavN`：`cbba_uav.sh`（預設 BT；輸出 `[bt]`、`[tag]`、`[state]`、`[cbba]`）
2. `sim`：`CBBA_WORLD=fire_site px4_sitl.sh 1`；之後 `px4_sitl.sh 2`、`3`…
3. `sim`（另一個 terminal）：`gz_bridge.sh`（`/clock`＋每台的 `/uavN/camera/image_raw`、`camera_info`）

Gazebo 視窗（`px4_sitl.sh` 開的）用 `docker/gz/gui/cbba_gui.config`：地面網格（分得出地面和空中）、沒有陰影、
依場景的俯瞰鏡頭、工具列有錄影按鈕（存成 MP4）。
4. （可選）`gcs`：開假狗，見下面。
5. 等每台「開始參與出價」。
6. `uav1`：`cbba_task.sh new 1 1 14 -6`（巡檢火警點）；`cbba_task.sh new 1 2 -10 -8`（誘餌，不能觸發）

要看到：
- `[bt]` 依序印「新指派 → 前往 → 到達」，懸停穩定後 `[tag]` 有 tag 101 的 confidence；≥ 0.85 時 `[bt] 火警成立：tag 101`
  → `廣播地面處置任務`（task_id 流水號 0x8000 以上，同一個 tag 只報一次），各台 `[cbba]` 印出新任務。
- 誘餌 tag 7 看得到、但不觸發。
- 看偵測結果：在**那台的 uav 容器**（或 sim）`ros2 topic echo /uav1/fire_detection`。
- 有假狗時，狗的 `current_task` 收到那個 GROUND_INTERVENTION；無人機都不出價（沒開狗時任務一直留著，正常）。

**假狗**（沒有真的狗、只看地面任務會不會分給狗），全部在 `gcs`：

```bash
# gcs terminal 1：狗的 CBBA（預設 yaml 就是 cbba_v60.yaml，機號 50）
ros2 launch swarm_cbba cbba.launch.py namespace:=v60
# gcs terminal 2：假裝狗的 BT 送狀態（2 Hz，位置用 map ENU，mode 0 = AUTO 才出價）
ros2 topic pub -r 2 /v60/robot_state swarm_interfaces/msg/RobotState \
  "{agent_id: 50, position: {x: 0.0, y: -10.0, z: 0.0}, battery: 90.0, mode: 0}"
# gcs terminal 3：看指派給狗的任務
ros2 topic echo /v60/current_task --qos-durability transient_local --qos-reliability reliable
# 回報完成（task_id 照 current_task 上的）
ros2 topic pub --once -w 1 /v60/task_result swarm_interfaces/msg/TaskResult "{task_id: <id>, success: true, reason: 0}"
```

## C. 巡檢＋隨機插入（patrol_site）

場景：綠色 G1 (20, 25)、G2 (−25, 20)、G3 (10, −30)（tag 0～2，一開始就有）；灰色方塊（2026-10-12 前是紅色）R1 (35, 5)、R2 (−35, −10)、
R3 (25, −25)、R4 (−10, 38)、R5 (40, 35)（tag 3～7，插入時才出現）。座標在 `docker/gz/worlds/patrol_site_points.csv`，
改了要執行 `python3 docker/gz/gen_patrol_site.py` 重新產生場景。

1. `uav1`～`uav3`：`cbba_uav.sh`；各台電量不同時 `SIM_BAT_DRAIN=600`／`300`／`200 cbba_uav.sh`
2. `sim`：`CBBA_WORLD=patrol_site px4_sitl.sh 1`，之後 `px4_sitl.sh 2`、`3`（`SIM_BAT_DRAIN` 和同號 uav 一樣）
3. `sim`：`gz_bridge.sh`（只測 CBBA 可以不開）
4. 等每台「開始參與出價」。
5. `sim`：`patrol_scenario.sh`，輸入 `g` 建立綠色巡檢任務，之後輸入 `1`～`5` 插入紅色點

| 輸入 | 動作 |
|---|---|
| `g` | 建立 G1～G3 三個巡檢任務 |
| `1`～`5`（可一次多個，例如 `2 4`） | 插入紅色點：tag 出現＋建立任務 |
| `f1`～`f5` | R 點起火：灰色方塊變紅＋tag 101、建立偵巡任務，飛過去的那台丟出地面處置任務 |
| `r` | 清掉紅色 tag |
| `l`、`q` | 列出任務點、結束 |

其他用法：`patrol_scenario.sh init`、`insert 3 5`、`fire 2`（R2 起火）、`fire-at 12.5 -3`（任意位置起火、不建任務，
等無人機路過發現）、`reset`、`list`、
`replay <離線輸出>/schedules/trial_007.csv`（照離線第 7 次試驗的時刻插入，含 `g`）。
環境變數 `CREATOR`（建立者，預設 1）、`VALUE`、`DURATION`、`DEADLINE`（預設 80、10 s、300 s，和離線模擬一致）。

**下一次試驗前**：`patrol_scenario.sh reset`，PX4（`px4_sitl.sh`）和每台 `cbba_uav.sh` 都重開（電池回到 100 %、任務清空）。

注意：
- PX4 的模擬電池都從 100 % 開始，不能設初始電量；Gazebo 的「電量不同」只能用不同的 `SIM_BAT_DRAIN`。
  離線對照：`cbba_sim insert … --batteries 100,100,100 --endurance 600,300,200`。
- `ros2 topic pub` 啟動約 1 s，replay 的插入時刻整體晚約 1 s。

## D. 弱網（mesh_tc）

在 A～C 任一個測試開著時，**每台 uav（和開了假狗的 gcs）都要設**（只影響 mesh 網卡的送出方向）：

```bash
mesh_tc.sh set 30          # 掉包 30 %、延遲 50±30 ms（週計畫的條件）
mesh_tc.sh set 70 80 20    # 掉包 70 %、延遲 80±20 ms
mesh_tc.sh set 100         # 斷網
mesh_tc.sh show
mesh_tc.sh clear           # 結束後每台都要清
```

看 `[cbba] 收斂：…`、完成確認的時間（`已確認完成（回報後 x s）`）。tc 只影響 mesh 上的 CBBA UDP 和避碰廣播，
PX4 的話題不受影響。要統計數字用 F 的 `itest.py loss`。

## E. 離線模擬（不開 Gazebo）

在 uav 容器（或主機用 `cbba-robot:jazzy` 映像，見 CLAUDE.md 的編譯方式）：

```bash
# 電池權重：100 次 × 權重 0／1／5／20／100，配對比較；輸出 summary、trials、tasks、agents.csv 和 schedules/
ros2 run cbba_core cbba_sim insert ~/CBBA_BT/docker/gz/worlds/patrol_site_points.csv --out ~/cbba_out/insert
#   選項：--batteries 100,70,45  --endurance 300（可每台一個值）  --drain-error 0.1  --window 120
#         --weights 0,1,5,20,100  --trials 100  --loss 0.3（網路掉包）  --seed 1
# 舊的協商統計（核心層的訊息，不是協定版本 2 的封包）
ros2 run cbba_core cbba_sim sweep random --trials 200 --out ~/cbba_out/sweep
```

`insert` 每台跑真的 CbbaComm＋AssignmentManager，電池依時間下降；「接了沒做完」分撤銷、返航兩種，見
`docs/sim_scenario_plan.md` 第七節。

---

## F. 自動整合測試（itest）

只看話題和網路、不看 log，換成別人寫的 CBBA Node／BT 時只改 `docker/itest/config.json`。說明在 `docker/itest/README.md`。

1. 做完「0. 準備」（容器開好、6 個套件編好）。
2. 在**主機**依序跑（前一個跑完再跑下一個，三組共用同一批容器）：

```bash
cd ~/CBBA_BT
python3 docker/itest/itest.py comm   # CBBA 之間的通訊 C0～C8，約 2 分鐘
python3 docker/itest/itest.py uav    # UAV 整串 U1～U6（fire_site），約 8～10 分鐘
python3 docker/itest/itest.py demo   # 展示：demo_site（小場景）開 Gazebo 畫面，全部起飛後才開始 CBBA，3 個固定任務；火點出現在某台的航線上、飛行途中發現、發布地面任務，
                                     # 錄 gazebo.mp4、畫俯視動畫 demo.mp4（要主機的 ffmpeg），約 6～8 分鐘；--seed N 重現
python3 docker/itest/itest.py loss   # 第 5 週掉包梯度 0／10／30／50／70%，各 20 次，約 25～35 分鐘
#   只量某些：--levels 30 --trials 100
```

3. 結果在 `docker/itest/out/<時間>_<組>/report.md`。loss 要畫圖時（主機沒有 matplotlib，在 sim 容器畫）：

```bash
./docker/enter.sh sim
python3 /home/ncrl/CBBA_BT/docker/itest/plot_loss.py /home/ncrl/CBBA_BT/docker/itest/out/<時間>_loss   # → loss_sweep.png
```

容器裡已經有在跑的節點、PX4、Gazebo 時會停下來，加 `--kill-existing` 先關掉。

2026-10-12 的結果：comm 9／9、uav 6／6；loss 30% 時 5 s 內收斂 95%（中位數 0.65 s），待命時 mesh 約 3.5 kB/s。

## 結束與清理

- 各 terminal Ctrl+C（`cbba_uav.sh` 會一起停掉自己開的節點；停掉後 PX4 沒有 setpoint 會觸發 failsafe）。
- 單獨停一個節點要用 `pkill -f "__node:=uavN_cbba_node( |$)"`（只 kill `ros2 run` 外層，節點會繼續跑）。
- 有設 `mesh_tc.sh` 的每台都要 `mesh_tc.sh clear`（itest 結束時會自己清）。
- 全部關掉：主機 `cd ~/CBBA_BT/docker && ./stop.sh`。
- 程序鎖在主機 `/tmp/cbba_locks`：同一個機號的 cbba_node 只能開一個，舊的沒停乾淨時新的會拒絕啟動。
