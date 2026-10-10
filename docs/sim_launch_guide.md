# 模擬測試的開啟方式

2026-10-10 · 無人機端（勝翔）。把散在 `docker/README.md`、`cbba_core/doc/sitl_test.md`、`ugv_test.md`、
`docs/sim_scenario_plan.md` 的步驟整理在一起；各測試的預期結果、細節還是看原本的文件。

| 測試 | 用途（週計畫） | 要開的東西 |
|---|---|---|
| [A. 基本分配](#a-基本分配task_executor) | CBBA 分配、完成、失敗、墜毀（第 1～2 週） | Gazebo 預設場景＋task_executor |
| [B. 火情偵測](#b-火情偵測fire_site) | AprilTag 101 → 地面處置任務（第 3 週） | `fire_site`＋BT＋相機＋（可選）假狗 |
| [C. 巡檢＋隨機插入](#c-巡檢隨機插入patrol_site) | terminal 輸入編號插入、電池權重抽驗 | `patrol_site`＋BT |
| [D. 弱網](#d-弱網mesh_tc) | 掉包 0～70 %（第 5 週） | 任一個上面的測試＋`mesh_tc.sh` |
| [E. 離線模擬](#e-離線模擬不開-gazebo) | 電池權重 100 次、收斂統計 | 只要 `cbba-robot` 映像 |

---

## 0. 準備（只做一次，或改程式後）

```bash
# 主機
cd ~/CBBA_BT/docker
vi .env                 # UAV_COUNT（預設 3）、PX4_SIM_MODEL=gz_x500_mono_cam_down（有下視相機）
./start.sh              # 產生 docker-compose.yaml，啟動 sim、uav1～N、gcs
./enter.sh uav1         # 進容器：./enter.sh sim | uav1 | uav2 | … | gcs

# uav1 容器：編譯（所有容器共用同一個 ros2_ws，不要兩個容器同時編譯）
cd ~/CBBA_BT/ros2_ws
colcon build --symlink-install --packages-select \
  swarm_interfaces cbba_core px4_waypoint_node uav_px4_bt apriltag_fire_detector
source install/setup.bash     # 已經開著的 terminal 都要重新 source
```

- 切換 git 分支後一定要重新編譯（`main` 和 `dog-integration` 共用 `build/`、`install/`）。
- 舊的 `install/uav_cbba`、`install/ugv_cbba` 不會自動刪掉，`cbba_uav.sh` 只用 `cbba_core`。

**順序規則**：每台的 `cbba_uav.sh`（含 XRCE Agent）要比同號的 `px4_sitl.sh N` 先開；`px4_sitl.sh 1` 負責開 Gazebo，
之後的台加入；`gz_bridge.sh` 要在 Gazebo 開好之後。

**共用的環境變數**（sim 和同號的 uav 兩邊要一樣）：

| 變數 | 預設 | 用在 |
|---|---|---|
| `SIM_BAT_DRAIN` | 900 | PX4 模擬電池的續航（秒）；`cbba_uav.sh` 用它算能量模型。要各台電量不同就各台設不同值 |
| `SIM_BAT_MIN_PCT` | 20 | PX4 模擬電池最低停在這裡（= BT 返航門檻） |
| `CBBA_SPEED`、`CBBA_ALTITUDE` | 5 m/s、5 m | cbba_node 的出價和執行者的飛行一致 |
| `CBBA_WORLD` | （PX4 預設場景） | `fire_site`、`patrol_site`（只對 `px4_sitl.sh 1` 有效） |
| `EXECUTOR` | `bt` | `bt` = `uav_px4_bt`＋`apriltag_fire_detector`；`task_executor` = 舊的簡易執行者 |

---

## A. 基本分配（task_executor）

| Terminal | 容器 | 指令 |
|---|---|---|
| 1～N | `uavN` | `EXECUTOR=task_executor cbba_uav.sh` |
| N+1 | `sim` | `px4_sitl.sh 1`，再開一個 terminal `px4_sitl.sh 2`… |
| 最後 | `uav1` | `cbba_task.sh new 1 1 10 5`（uav1 建立任務 00010001 在 (10, 5)） |

確認：每台 `[cbba] 開始參與出價`、`收斂：0 個任務，N−1 個鄰居`。之後照 `sitl_test.md` 第 2～4 節
（分配、`cbba_task.sh fail`、取消、關掉一台 PX4 或整個容器）。
task_executor 沒有避碰：同高度會撞機，可以每台 `EXEC_ARGS="-p altitude:=5.0／7.0／9.0"` 分層。

## B. 火情偵測（fire_site）

場景：火警紅色方塊 (14, −6) 貼 tag 101、誘餌 (−10, −8) 貼 tag 7、巡檢點 (12, 12)、(−12, 10)、(0, −16)。

| Terminal | 容器 | 指令 |
|---|---|---|
| 1～N | `uavN` | `cbba_uav.sh`（預設 BT；輸出 `[bt]`、`[tag]`、`[state]`、`[cbba]`） |
| N+1 | `sim` | `CBBA_WORLD=fire_site px4_sitl.sh 1`，之後 `px4_sitl.sh 2`… |
| N+2 | `sim` | `gz_bridge.sh`（`/clock`＋每台的 `/uavN/camera/image_raw`、`camera_info`） |
| N+3 | `uav1` | `cbba_task.sh new 1 1 14 -6`（巡檢火警點）；`cbba_task.sh new 1 2 -10 -8`（誘餌，不能觸發） |
| （可選）假狗 | `gcs` | 見下面 |

確認：
- `[bt]` 依序印「新指派 → 前往 → 到達」，懸停穩定後 `[tag]` 有 tag 101 的 confidence；≥ 0.85 時 BT 建立
  GROUND_INTERVENTION（task_id 流水號 0x8000 以上，同一個 tag 只報一次），`[cbba]` 印出新任務。
- 誘餌 tag 7 看得到、但不觸發。
- `ros2 topic echo /uav1/fire_detection` 看偵測結果（tag_id、confidence、frames、position）。

**假狗**（沒有真的狗、只看地面任務會不會分給狗）：gcs 容器在 mesh 上，可以跑狗的 cbba_node。
沒有狗的 BT，所以關掉握手：

```bash
# gcs terminal 1
ros2 run cbba_core cbba_node --ros-args -r __node:=v60_cbba_node \
  -p vehicle_type:=ugv -p agent_id:=50 -p require_accept:=false
# gcs terminal 2：狗的狀態（2 Hz，位置用 map ENU）
ros2 topic pub -r 2 /v60/robot_state swarm_interfaces/msg/RobotState \
  "{agent_id: 50, position: {x: 0.0, y: -10.0, z: 0.0}, battery: 90.0}"
# gcs terminal 3：看指派給狗的任務
ros2 topic echo /v60/assigned_task --qos-durability transient_local --qos-reliability reliable
```

火警成立後，狗的 `assigned_task` 應該收到那個 GROUND_INTERVENTION；回報完成照 `ugv_test.md`（要帶 `assignment_version`）。
要測握手（狗預設 `require_accept` true）就去掉 `-p require_accept:=false`，用 `ugv_test.md` 的假 BT 回覆 `assignment_request`。

## C. 巡檢＋隨機插入（patrol_site）

場景：綠色 G1 (20, 25)、G2 (−25, 20)、G3 (10, −30)（tag 0～2，一開始就有）；紅色 R1 (35, 5)、R2 (−35, −10)、
R3 (25, −25)、R4 (−10, 38)、R5 (40, 35)（tag 3～7，插入時才出現）。座標在 `docker/gz/worlds/patrol_site_points.csv`，
改了要執行 `python3 docker/gz/gen_patrol_site.py` 重新產生場景。

| Terminal | 容器 | 指令 |
|---|---|---|
| 1～3 | `uavN` | `cbba_uav.sh`；各台電量不同時 `SIM_BAT_DRAIN=600`／`300`／`200 cbba_uav.sh` |
| 4 | `sim` | `CBBA_WORLD=patrol_site px4_sitl.sh 1`，之後 `px4_sitl.sh 2`、`3`（`SIM_BAT_DRAIN` 和同號 uav 一樣） |
| 5 | `sim` | `gz_bridge.sh`（只測 CBBA 可以不開） |
| 6 | `sim` | `patrol_scenario.sh` |

`patrol_scenario.sh` 的互動輸入：

| 輸入 | 動作 |
|---|---|
| `g` | 建立 G1～G3 三個巡檢任務 |
| `1`～`5`（可一次多個，例如 `2 4`） | 插入紅色點：tag 出現＋建立任務 |
| `r` | 清掉紅色 tag |
| `l`、`q` | 列出任務點、結束 |

其他用法：`patrol_scenario.sh init`、`insert 3 5`、`reset`、`list`、
`replay <離線輸出>/schedules/trial_007.csv`（照離線第 7 次試驗的時刻插入，含 `g`）。
環境變數 `CREATOR`（建立者，預設 1）、`VALUE`、`DURATION`、`DEADLINE`（預設 80、10 s、300 s，和離線模擬一致）。

**下一次試驗前**：`patrol_scenario.sh reset`，PX4（`px4_sitl.sh`）和每台 `cbba_uav.sh` 都重開（電池回到 100 %、任務清空）。

注意：
- PX4 的模擬電池都從 100 % 開始，不能設初始電量；Gazebo 的「電量不同」只能用不同的 `SIM_BAT_DRAIN`。
  離線對照：`cbba_sim insert … --batteries 100,100,100 --endurance 600,300,200`。
- `ros2 topic pub` 啟動約 1 s，replay 的插入時刻整體晚約 1 s。

## D. 弱網（mesh_tc）

只影響 mesh 網卡的送出方向，所以**每台 uav（和 gcs）都要設**：

```bash
mesh_tc.sh set 30          # 掉包 30 %、延遲 50±30 ms（週計畫的條件）
mesh_tc.sh set 70 80 20    # 掉包 70 %、延遲 80±20 ms
mesh_tc.sh show
mesh_tc.sh clear
```

看 `[cbba] 收斂：…`、完成確認的時間（`已確認完成（回報後 x s）`）。
**已知問題**：Fast DDS 也走 mesh（uav 容器的 `LOCAL_IPS` 含 mesh IP），PX4 的 DDS 話題也會被 tc 影響，結果會受干擾；
要改 `gen_compose.py` 才能分開。100 % 斷網可以用 `mesh_tc.sh set 100`。

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

## 結束與清理

- 各 terminal Ctrl+C（`cbba_uav.sh` 會一起停掉自己開的節點；停掉後 PX4 沒有 setpoint 會觸發 failsafe）。
- 單獨停一個節點要用 `pkill -f "__node:=uavN_cbba_node( |$)"`（只 kill `ros2 run` 外層，節點會繼續跑）。
- 全部關掉：主機 `cd ~/CBBA_BT/docker && ./stop.sh`。
- 程序鎖在主機 `/tmp/cbba_locks`：同一個機號的 cbba_node 只能開一個，舊的沒停乾淨時新的會拒絕啟動。
