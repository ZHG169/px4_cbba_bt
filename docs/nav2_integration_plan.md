# 導入 Nav2 的規劃（2026-10-14 草案，同日更新：無人機端的改動對狗端的影響）

目的：模擬環境裡的狗從「假狗」（`docker/scripts/fake_dog.py`：直線移動、Gazebo 橘色方塊）換成
**伯宇的 `task_manager`＋Nav2**，讓 BT 照原樣編譯、照原樣跑。驗證 Phase 1 的「cancel_goal ≤ 50 ms」「火情發布 → 狗開始轉向 ≤ 300 ms」，
以及 demo 的全流程。

本文件只是規劃，Nav2 的部分還沒有改任何程式。

## 〇、10/14 無人機端的改動對導入 Nav2 的影響

| 改動 | 對狗端／Nav2 的影響 | 導入時要注意 |
|---|---|---|
| 火點去重複算進別台（cbba_node 新話題 `task_announced`，**只有 agent_type = UAV 才發**） | 狗的 cbba_node 不變、伯宇的 BT 不用改；同一個火點只會有一個 GROUND_INTERVENTION（原本兩台都看到時會有兩個，狗要白跑第二次） | 狗的 cbba_node 用 `cbba_v60.yaml`（agent_type UGV），不會有 `/v60/task_announced`，不用處理 |
| 無人機看到火點會轉向到火點上空懸停，**等地面任務結束（task_closed）才走**，最多 `fire_task.monitor_timeout` 120 s | 地面任務結束 = 狗的 BT 回報 `task_result` 成功 → cbba_node 送 COMPLETION／TASK_CLOSE。換成真的 task_manager＋Nav2 後，狗慢（Nav2 規劃、繞障礙），無人機就懸停得久 | 假狗（`fake_dog.py`，1.5 m/s 直線）在 demo 量到「發布 → 地面任務結束」30 s；換成 Nav2 後拿這個當比較基準。超過 120 s 無人機會先走（D4 會 FAIL） |
| `demo_site` 重擺（10/14 第二版）：G1 (0, −6)、G2 (0, −14)、G3 (−14, −12)；R1 (−9, −2)、R2 (2.5, −14)、R3 (−12, 12)、R4 (0, −8)、R5 (−20, 4)；狗的起點 (−4, 0)（itest config 的 `dog.start_xy`），要往南走過 G1 平台旁才到 R2 | 狗要繞過的東西：綠色平台（G，1.5 m 見方、高 0.05 m）、灰色方塊（R，1.4 m、高 0.3 m）、火點方塊（`fire_block` 1.6 m、高 0.32 m） | **都比一般 2D 雷達的掃描高度低**，雷達看不到 → 階段 4 的地圖要用 `gen_map.py` 從 CSV 把方塊畫進靜態地圖，不能只靠空白地圖＋雷達；R2 離 G2 平台只有 2.5 m，站位（`external_standoff_m` 1.5 m）可能落在平台旁的窄縫 |
| 火點在方塊上（有碰撞） | 任務位置 = 火點中心，在障礙物裡，Nav2 直接去會回 206（occupied） | 伯宇的 `makeExternalWaypoint` 已經停在目標前 `external_standoff_m` 1.5 m；方塊半寬 0.8 m＋狗半長約 0.45 m，1.5 m 剛好，costmap 的膨脹半徑要 ≤ 0.2 m 左右，否則站位也算佔用 |
| itest demo 有 `--fire-mode side`、`--scenario`（自訂任務點、起火點、插入時機）、D7（CBF 最近距離） | 階段 6 的 `--dog nav2` 可以直接用這些情境（例如 `--scenario example` 有起火點，會開狗） | D3（狗處理完）、D4（無人機監控到狗處理完）的判定不用改，只換狗的實作 |
| 無人機的避碰換成 CBF（只在無人機之間，看位置廣播） | 狗不送位置廣播，無人機的 CBF 不會閃狗（高度不同，不需要） | 不用處理 |

---

## 一、範圍：Nav2 只給狗用，無人機不用

| | 狗（V60） | 無人機 |
|---|---|---|
| 移動 | 2D 平面、要繞障礙 → Nav2（`NavigateToPose`、costmap、規劃器、控制器） | 3D、PX4 offboard 位置＋速度 setpoint |
| 避碰 | Nav2 的 costmap（雷達） | CPF＋位置廣播（`flight_beacon`） |
| BT 葉節點 | `GoTo`（Nav2 action client） | `NavigateTo`（`UavContext::moveTo`） |

無人機**不建議**接 Nav2：
- Nav2 的 costmap、規劃器、控制器都是 2D，送出的是 `cmd_vel`。PX4 offboard 要的是位置或速度 setpoint，
  中間還要再加一層轉接，高度、起降都在 Nav2 外面。
- 空中沒有需要繞的靜態障礙；機間避碰已經由 CPF 處理（10/13 SITL 調過）。
- 伯宇的骨架本來就讓兩邊只差葉節點（UAV_BT_GUIDE §3），不需要為了統一而把無人機也接 Nav2。

所以「導入 Nav2」= 讓**狗那一端**在我們的 docker／Gazebo 裡完整跑起來。無人機端的程式不動。

---

## 二、伯宇的 task_manager 需要什麼

依 `docs/handoff_uav_2026-10-12/bt/task_manager` 的 `package.xml`、`CMakeLists.txt`、`task_manager_node.cpp`：

| 依賴 | 用途 | 我們現在有沒有 |
|---|---|---|
| `nav2_msgs` | `GoTo` 的 `NavigateToPose` action、reaper | **沒有**（所以 10/13 把骨架切進 `uav_px4_bt`） |
| `task_manager_interfaces` | `PtzInspect.action` | 在 handoff 裡，可以直接放進 `ros2_ws/src` |
| `swarm_interfaces` | current_task、robot_state、task_result | 有（v1.6，和他的 v1.2 相容） |
| `behaviortree_cpp`、`yaml-cpp`、`tf2_ros`、`visualization_msgs` | BT、任務檔、TF、RViz | `behaviortree_cpp` 有；`yaml-cpp`、tf2 多半已有，要確認 |
| Nav2 本體（`navigate_to_pose` server） | 實際走路 | **沒有** |
| TF `map → base_link` | robot_state 的位置（`lookupTransform(frame_id, base_frame)`） | **沒有**（要由 Gazebo 的 odom＋靜態 TF 或 AMCL 提供） |
| `ptz_node`（`/ptz_inspect` action server） | 雲台拍照 | **沒有**，不在 handoff 裡 |
| `ghost_bridge`（急停） | 真機的急停鈕 | 模擬不用：`estop_bridge:=false` |
| `nav_ws`（地圖、Nav2 參數、`sim.launch.py`、V60 模型） | 他的 Nav2 設定 | **沒有**，要跟他拿 |

他的參數重點：
- `nav_action` 預設 `/navigate_to_pose`（絕對名稱，Nav2 沒放在 namespace 下）；只有一隻狗時不衝突。
- `namespace` 預設 `v60`、`agent_id` 50。
- `frame_id` 來自任務檔（`map`）、`base_frame` 為 `base_link`。
- 模擬時要 `use_sim_time:=true`。

---

## 三、架構（模擬）

```
sim 容器（Gazebo Harmonic，同一個世界）
  ├─ PX4 SITL ×N（無人機，不變）
  ├─ V60 模型：VelocityControl（或 DiffDrive）＋ OdometryPublisher ＋ gpu_lidar（2D）＋ IMU
  └─ gz_bridge.sh：/clock、/uavN/camera/*（不變）
                   ＋ /v60/cmd_vel（ROS → GZ）、/v60/odom、/v60/scan、/tf（odom → base_link）
        │ 線材網路（DDS，機內）
dog 容器（新，cbba-dog:jazzy）
  ├─ Nav2（map_server、planner、controller、bt_navigator、costmap）use_sim_time
  ├─ 靜態 TF map → odom（= 狗的出生點）或 AMCL
  ├─ 假的 ptz_node（/ptz_inspect：等 N 秒回成功）
  ├─ task_manager（伯宇，原樣編譯）
  └─ swarm_cbba/cbba_node（cbba_v60.yaml，機號 50，協定版本 2）
        │ mesh_net（UDP，可加 tc）
uav1～N 容器（不變）
```

- 狗和無人機之間仍然只走 mesh 的 UDP（CBBA 協定版本 2），DDS 只在機內。這和現在的規則一致。
- 狗的 DDS 需要看得到 sim 容器的 Gazebo bridge（`/clock`、`/v60/*`、`/tf`）。所以 dog 容器要像 uavN 一樣有一條線材網路接 sim。

---

## 四、分階段

### 階段 1：編譯通過（不跑 Nav2）

1. `robot.Dockerfile` 加 `ros-jazzy-nav2-msgs`（小，只有訊息）、`libyaml-cpp-dev`。
2. 把 `task_manager`、`task_manager_interfaces` 從 handoff 放進 `ros2_ws/src`（或子模組、另一個 workspace）。
   隊友的資料不進 repo（`.gitignore`）。用另一個 workspace（`~/dog_ws`）overlay 在 `ros2_ws/install` 上最乾淨。
3. 隔離編譯：`task_manager`＋`test_tree` 照他的 README 跑過。

完成條件：零警告、他的 `test_tree` 全過。

到這一步，**無人機的 `uav_px4_bt/skeleton` 可以考慮改成直接引用他的標頭**（不再切一份）。但他的 `bt_nodes.hpp` 會連 Nav2、雲台一起
include，要和他談是否拆成「通用」與「狗葉節點」兩個標頭。先不做，列在第七節。

### 階段 2：dog 容器與 Nav2 本體

1. 新映像 `cbba-dog:jazzy`（`FROM cbba-robot:jazzy`），安裝：
   `ros-jazzy-navigation2`、`ros-jazzy-nav2-bringup`、`ros-jazzy-tf2-ros`；要看地圖時另加 RViz（`WITH_GUI=1`）。
   不要裝進 cbba-robot：Nav2 很大，無人機容器（之後的 Jetson）用不到。
2. `gen_compose.py` 加 `dog` 服務：`.env` 加 `DOG_ENABLE=1`（預設 0）、`DOG_SPAWN=-4,0`；
   兩條網路：`sim_dog_net`（線材，DDS 看得到 sim）＋ `mesh_net`（`MESH_IP`，cbba_node 的 UDP）；
   `ROS_DOMAIN_ID`、FastDDS 的設定照 uavN 的寫法（只走線材）。
3. `enter.sh dog`、`start.sh` 跟著支援。

完成條件：`./enter.sh dog` 後 `ros2 pkg list | grep nav2` 有；容器之間 DDS 和 UDP 的隔離和 uavN 一樣。

### 階段 3：Gazebo 裡的 V60

1. 模型：優先用伯宇的 V60 URDF（週計畫第 1 週「V60 URDF 進 Gazebo」）。四足步態在 Gazebo 模擬很慢、很不穩，**建議模擬時用平面移動**：
   - `gz-sim-velocity-control-system`（收 `cmd_vel` 直接平移、轉向），或 `gz-sim-diff-drive-system`；
   - `gz-sim-odometry-publisher-system`（odom → base_link）；
   - `gpu_lidar` 2D（Nav2 的 obstacle layer）＋ IMU（可選）。
   只是看起來像 V60（外觀）。步態、機械臂不在這次範圍。
2. 場景：`fire_site`、`patrol_site`、`demo_site` 加 `<include>` V60（或用 `gz service /world/<w>/create` 在 `px4_sitl.sh` 之後生成，和 `fake_dog.py` 的方塊同一個做法）。
3. `gz_bridge.sh` 的 config 加 `/v60/cmd_vel`（ROS → GZ）、`/v60/odom`、`/v60/scan`、`/model/v60/tf` → `/tf`。

完成條件：在 dog 容器裡 `ros2 topic pub /v60/cmd_vel …`，Gazebo 裡的 V60 會動；`ros2 run tf2_ros tf2_echo odom base_link` 有值。

### 階段 4：座標與地圖

**狗的 `map` frame 一定要和無人機的 map ENU 相同。** CBBA 的任務座標、火點座標都在這個座標系；`px4_state_bridge` 的 map ENU
= Gazebo 世界座標（出生點加上去）。

| 項目 | 做法 | 原因 |
|---|---|---|
| map → odom | 先用**靜態 TF = 狗的出生點**（Gazebo 的 odom 沒有誤差）；之後要測定位時再換 AMCL | 少一個會飄的元件，先把流程跑通 |
| 地圖 | **`gen_map.py` 從 `<world>_points.csv` 產生 PGM**（平台、灰色方塊、和 `gen_patrol_site.py` 同一份座標；0.05 m 解析度）＋雷達的 obstacle layer；起火時生成的 `fire_block` 不在靜態地圖裡 → 用深度相機或 3D 雷達的 voxel layer，或起火時由測試把方塊加進 keepout | 10/14 更新：場景裡的方塊只有 0.05～0.32 m 高，2D 雷達（通常裝在 0.3 m 以上）看不到，空白地圖＋雷達會直接撞上 |
| 地圖原點 | `origin` 設成讓 Gazebo (0,0) 對到 map (0,0) | 和無人機一致 |
| 火點方塊 | `fire_block` 有碰撞時，任務點要停在旁邊：伯宇已經有 `external_standoff_m` 1.5 m | 目標在障礙物裡 Nav2 會回 206（occupied） |

完成條件：`ros2 action send_goal /navigate_to_pose …` 走到正確的 Gazebo 座標；`task_manager` 的 robot_state 位置和 Gazebo 一致（誤差 < 0.2 m）。

### 階段 5：雲台

`ptz_node` 不在 handoff 裡。二選一：
- **寫一個假的 `/ptz_inspect` action server**（Python，等 `snapshots × 1 s` 回成功、不拍照）。最快，建議先用這個。
- 跟伯宇拿 `ptz_node` 的模擬模式（他的 `bt.sh sim` 有「假相機」）。

### 階段 6：接上 CBBA，取代假狗

1. dog 容器裡啟動：
   ```
   ros2 launch swarm_cbba cbba.launch.py namespace:=v60                       # 協定版本 2（main）
   ros2 launch <nav2 bringup> use_sim_time:=true map:=<空白地圖>
   假 ptz_node
   ros2 launch task_manager task_manager.launch.py namespace:=v60 agent_id:=50 use_sim_time:=true
   ```
   包成 `docker/scripts/dog_sim.sh`（和 `cbba_uav.sh` 同風格），有 tmux 分頁。
2. **時間**：Nav2、task_manager 用 sim time；`cbba_node` 的 s 用系統時鐘（協定規定）。這和無人機端一樣，不用改；
   但要確認 task_manager 的逾時（`mission_timeout`、`server_timeout`）在 sim time 下合理（Gazebo 慢於即時時會變長）。
3. itest：`itest.py demo --dog nav2`（預設仍是 `fake`）。D1～D7 照跑，`--fire-mode`、`--scenario` 都能搭配；另外加：
   - `cancel_goal` 延遲：task_manager 的 log「halt → cancel Nav2 goal」到 Nav2 的 `_action/status` 變 CANCELED；
   - 火情發布 → 狗的 `cmd_vel` 方向改變的時間（週計畫 Phase 1 的 300 ms）。

完成條件：demo 的 D1～D6 用真的 task_manager＋Nav2 通過；兩個延遲有量測值。

### 階段 7：真的 V60

真機上 Nav2、定位、ptz、ghost_bridge 都是伯宇的。我們要做的只有一件事：**請他把 `swarm_cbba`、`cbba_core`、`swarm_interfaces` 換成 main（協定版本 2）**。
模擬這一套跑通之後，可以把 dog 容器的啟動順序整理成他的檢查清單。

---

## 五、要改或新增的檔案（預估）

| 檔案 | 改什麼 | 階段 |
|---|---|---|
| `docker/robot.Dockerfile` | 加 `ros-jazzy-nav2-msgs`、`libyaml-cpp-dev` | 1 |
| `docker/dog.Dockerfile`（新） | `FROM cbba-robot`，加 `navigation2`、`nav2-bringup` | 2 |
| `docker/build.sh`、`start.sh`、`enter.sh` | 建立、啟動、進入 dog 容器 | 2 |
| `docker/gen_compose.py`、`.env` | `dog` 服務、`DOG_ENABLE`、`DOG_SPAWN`、網路 | 2 |
| `docker/gz/models/v60/`（新） | 模型（伯宇的 URDF 轉 SDF，或簡化外觀）＋移動、odom、雷達 plugin | 3 |
| `docker/gz/worlds/*.sdf`、`gen_patrol_site.py` | 放 V60 | 3 |
| `docker/scripts/gz_bridge.sh` 的 config | `/v60/*`、`/tf` | 3 |
| `docker/dog/`（新）：`nav2_params.yaml`、`gen_map.py`（從 `<world>_points.csv` 把平台、方塊畫進地圖；方塊比雷達低，不能只用空白地圖） | Nav2 設定、地圖 | 4 |
| `docker/scripts/fake_ptz.py`（新） | 假的 `/ptz_inspect` | 5 |
| `docker/scripts/dog_sim.sh`（新） | 一次開 Nav2、ptz、task_manager、cbba_node | 6 |
| `docker/itest/itest.py` | `--dog nav2`、延遲量測 | 6 |
| `docs/sim_launch_guide.md`、`CLAUDE.md` | 開啟方式、分工 | 6 |

不動的：`cbba_core`、`swarm_cbba`、`uav_px4_bt`、`px4_waypoint_node`、`apriltag_fire_detector`。

---

## 六、風險與注意

| 風險 | 影響 | 對策 |
|---|---|---|
| sim 容器的 CPU（3 台 PX4＋3 個相機＋Nav2＋雷達） | Gazebo 慢於即時，PX4、Nav2 的逾時都會被拉長 | 雷達用低解析度（360 點、10 Hz）、`DOG_ENABLE` 預設關；先在 `UAV_COUNT=2` 跑 |
| map frame 對不上 | 狗走到錯的位置，demo 看起來像 CBBA 錯 | 階段 4 的完成條件要先過，再接 CBBA |
| Nav2 的 action 名稱是絕對的 `/navigate_to_pose` | 多隻狗時衝突 | 現在只有一隻；之後 Nav2 放進 `/v60` namespace，task_manager 設 `nav_action:=/v60/navigate_to_pose` |
| use_sim_time 混用 | TF 的時間戳對不上，Nav2 報 extrapolation | dog 容器裡除了 cbba_node 以外全部 sim time；`/clock` 由 gz_bridge 提供 |
| 伯宇的 handoff 更新 | 我們用的版本落後 | 不 fork 他的程式；用 overlay workspace 放他的原檔，更新時直接覆蓋 |
| 四足步態 | 平面移動模擬不到步態的震動、轉向限制 | 模擬只驗流程與延遲；步態留給真機 |

---

## 七、要先和伯宇確認的

1. `nav_ws` 能不能給我們（`sim.launch.py`、`nav2_params.yaml`、V60 的 URDF／SDF、地圖）？他用的是 Gazebo Harmonic 還是 Classic？
2. 他模擬時狗怎麼移動（步態外掛還是平面移動）？`cmd_vel` 的話題名稱、`base_link`／`odom` 的 frame 名稱？
3. `ptz_node` 有沒有模擬模式可以單獨給我們？
4. `bt_nodes.hpp` 能不能拆成「通用骨架」和「狗的葉節點（GoTo、PtzInspect）」兩個標頭？拆了以後無人機可以直接 include 他的骨架，不用每次重切。
5. 他什麼時候換成 main（協定版本 2）？

## 八、建議順序與預估

| 順序 | 階段 | 預估 | 依賴 |
|---|---|---|---|
| 1 | 階段 1 編譯 | 0.5 天 | 無 |
| 2 | 階段 2 容器 | 0.5～1 天 | 無 |
| 3 | 階段 3＋4 Gazebo、座標 | 1～2 天 | 伯宇的模型（沒有就先用簡化外觀） |
| 4 | 階段 5 假雲台 | 0.5 天 | 無 |
| 5 | 階段 6 接 CBBA、itest | 1～2 天 | 1～4 |
| 6 | 階段 7 真機 | — | 伯宇換 main |

週計畫上屬於第 4 週（Phase 1 空地聯調，10/26～）：建議第 3 週後半先做階段 1、2，第 4 週開頭完成 3～6。
