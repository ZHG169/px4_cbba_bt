# 機器狗（V60）端使用說明

給伯宇（狗端）。2026-10-12 · 勝翔（2026-11-14 更新：2-1b M0 的變更清單，見第 0.1 節）

**一句話**：main 上的 CBBA 節點照你 handoff（2026-10-11）的格式做好了。`mission.xml`、launch 指令、yaml 不用改；
**換掉三個套件、你的 `task_manager` 套兩個 patch、任務檔的 `ptz_inspect` 改 `inspect`，再全部重新編譯**（第 0.1 節 A）。

## 0.1 2-1b M0（2026-11-11～14）的變更清單

協定升到**版本 4**、訊息升到 **`swarm_interfaces` v2.0**。分三類：A 部署前一定要做；B 不用改程式、但行為要知道；C 要和你對的。

### A. 部署前一定要做

| # | 項目 | 為什麼 | 怎麼做 |
|---|---|---|---|
| A1 | 換掉三個套件：分支 `2-1b-m0` 的 `swarm_interfaces`、`cbba_core`、`swarm_cbba`（第 1 節） | 版本 3 和版本 4 **互相拒收**；全隊（無人機、狗、閘道）要同一版 | 第 1 節 |
| A2 | 用到 `swarm_interfaces` 的**全部重編**（含 `task_manager`） | `Task`、`TaskResult` 加了欄位、新增 `TaskOrphaned`、`Detection`；DDS 型別雜湊變了，沒重編的程式**靜默收不到** | 第 2 節（最後有確認的指令） |
| A3 | `task_manager` 套 **uint64 patch** | task_id 改 64 位元（建立者｜世代｜流水號）；你的 `uint32_t task_id` 約 12 處（`externalTaskId`、`current_task_id`、task_result…）會把閘道建的任務截斷 | `docker/dog/task_manager_uint64.patch` |
| A4 | `task_manager` 套 **spec patch** | 閘道建的任務用 `current_task.spec`（2-1b 的通用 steps：`{"steps": [{"action": "go_to"…}, {"action": "inspect"…}]}`）告訴你要做什麼；不套的話 spec 被忽略、照舊固定做一次雲台 | `docker/dog/task_manager_spec.patch`（`parseSpecSteps`：第一步 go_to 是站的位置、inspect 的 location → 雲台 target、沒寫 snapshots 拍 1 張；`BAD_SPEC` → PERMANENT；說明在你的 `config/MISSION_FORMAT.md` 最後一節） |
| A5 | 動作名稱 `ptz_inspect` 改叫 **`inspect`** | 狗和無人機用同一套動作名稱（2-1b 的 actions），閘道、Node-RED 才能同時下命令 | 同 A4 的 patch（Switch 的 case、`kValidActions`）；**你的任務檔**也要改 `action: inspect`。雲台的 ROS action（`/ptz_inspect`、`PtzInspect.action`）不變 |
| A6 | 同一隻狗的**機號三處一致** | cbba_node 收到的 robot_state 機號不對會略過 → 這隻狗永遠不出價 | `cbba_v60.yaml` 的 `agent_id`、你 BT 的 `agent_id` 參數、`registry/fleet.yaml` 的 `id`（有用註冊表時）：dog1 = 50、dog2 = 51、dog3 = 52 |

套 patch（照順序；你更新 `task_manager` 後也照這樣重套，有衝突時對照 patch 開頭的說明手動改）：

```bash
cd <你的 task_manager 的上一層>
patch -p1 -d task_manager < <repo>/docker/dog/task_manager_uint64.patch
patch -p1 -d task_manager < <repo>/docker/dog/task_manager_spec.patch
```

### B. 不用改程式，但行為要知道

| # | 項目 | 對你的影響 |
|---|---|---|
| B1 | 任務改由**閘道**（GCS，機號 200，不出價的 cbba_node）建立 | `current_task` 的 task_id 會是版本 4 的大數字（log 印成 `00C8.0001.00000003` = 建立者 200、世代 1、流水號 3）；舊的 `new_task`（`50 × 65536 + 流水號`）保留到 M3。`cbba_v60.yaml` 已經有 `cancel_authorities: [200]`（閘道可以取消任務） |
| B2 | **WAITING_SPEC**：閘道建的任務，TaskSpec 還沒收齊前**不發 `current_task`** | 弱網時可能晚一點才收到任務（`spec_wait` 5 s 後 cbba_node 會加強要、發 `task_spec_missing` 告警，不算失敗）；你收到的 `spec` 一定是完整的 |
| B3 | 介面 v2.0：`Task` 加 `revision`、`spec`、`priority`、`platform`、`required_capabilities`、`group_id`、`role_index`、`role_count`、`optional`、`floor`、`deadline_hard`、`release`、`expires` | 不讀也能跑。要用時直接讀 `current_task` 的欄位；舊型任務填對應後的值（火警 P0、quadruped、mobility_ground） |
| B4 | 介面 v2.0：`TaskResult` 加 `failure_code`、`failed_capability`、`findings_uri` | 不填照舊用 `reason`。填了 `failure_code` 時 cbba_node 照下表決定排除（你填的 `reason` 不看）：E_PRECONDITION、E_UNREACHABLE、E_HW → PERMANENT；E_TIMEOUT、E_TARGET_LOST、E_COMM → TRANSIENT；E_LOW_BATTERY、E_SAFETY_STOP → STOPPED。E_HW 時 `failed_capability` 填故障的能力（例如 `rgb_camera`），這隻之後不接需要它的任務（到 cbba_node 重開）；E_PRECONDITION 另外通知值班（`task_orphaned`） |
| B5 | `task_orphaned` 的型別從 `Task` 改成 **`TaskOrphaned`**（task、reason、waited_sec、detail） | 你的 BT 沒訂這個話題，不影響；地面站、監控程式有訂的要改型別 |
| B6 | 新話題 `/v60/detection`（`swarm_interfaces/Detection`：detector、source、t、label、score、bbox、track_id、position、attrs） | 現在不用做。之後狗的偵測（看到火、看到人）**不自己建任務**，發這個話題；cbba_node 送給閘道（閘道斷線時暫存、恢復後送出），由閘道決定要不要建任務 |
| B7 | 能力過濾：任務帶 `platform`、`required_capabilities`，載具要平台符合、能力都有才出價 | 狗預設能力 `mobility_ground`、`rgb_camera`；V60 實際有的（雲台變焦 `zoom`、熱像 `thermal_camera`…）請告訴我，加進 `registry/fleet.yaml` |
| B8 | 註冊表（`<repo>/registry/`：`tools.yaml`、`platforms.yaml`、`fleet.yaml`） | 選用：cbba_node 給 `registry_dir`（或環境變數 `CBBA_REGISTRY_DIR`）時從 `fleet.yaml` 讀平台與能力；沒給照參數。`swarm_cbba` 多了 yaml-cpp 的依賴（Jazzy 本來就有，不用另外裝） |
| B9 | 搶占與接手照任務的優先級 P0～P3：接近目標（`commit_radius` 內）後不可被搶；持有者失聯 P0 10 s、P1 15 s、P2／P3 30 s 後別台接手 | 舊型任務結果和原本相同（火警 = P0 10 s、巡檢 = P3 30 s） |

### C. 要和你對的

| # | 項目 | 狀況 |
|---|---|---|
| C1 | **改版**：同一個 task_id、較大的 `revision`（例如目標移動），cbba_node 會重發 `current_task` | 你的 `onCurrentTask` 的 `if (t.task_id == cur) return;` 會當成重送、照舊版做完。選項：我出 patch（revision 變大時先取消再插入），或你的新版處理 |
| C2 | 你 11/11 的新版（Switch9：Track、Wait、Speak…） | 還沒拿到；兩個 patch 可能要手動合。新的 case 請用通用名稱（`go_to`、`inspect`、`track`、`wait`、`speak`、`return_to_dock`、`alert_operator`），同時加進 `kValidActions` |
| C3 | 失敗碼（REG-2，你定 Action 介面）→ `failure_code` | 你的 Action 有失敗碼後，`finish()` 一起填 B4 的欄位 |
| C4 | 機號只寫一處 | 現在要手動對齊 A6 的三處；之後可以讓 launch 從 `fleet.yaml` 用 namespace 查機號再給 BT 和 cbba_node |
| C5 | V60 的實際能力 | B7 |
| C6 | 急停話題 | 狗是全域 `/emergency_stop`、無人機是 `/<ns>/emergency_stop`；2-1b 要「全體急停」＋單機，要統一 |

### 驗收（換完之後）

1. 第 2 節的確認指令（`ros2 interface show …`）全部對。
2. 第 5 節你 README 的 T1、T2、T9 照跑。
3. 第 0.2 節：用閘道下帶 steps 的任務，測你的 BT 照動作名稱分派。

## 0.2 用 spec 測 BT 的動作名稱（2026-11-14）

閘道建的任務把「要做什麼」寫在 `current_task.spec` 的 **steps**（2-1b 的通用動作，狗和無人機同一套名稱）。
spec patch 讓你的 BT 把 steps 翻成外部點的 tasks，交給 `mission.xml` 的 Switch 照 `action` 分派。
這一節不需要無人機、PX4、閘道以外的機器：**閘道＋你的狗（cbba_node＋BT）**就能測。

### 現在能測、還沒定的

| | 項目 |
|---|---|
| 能測 | steps 的解析與分派：`go_to`、`inspect` 已經接好；**你新加的動作**（`track`、`wait`、`speak`、`return_to_dock`、`alert_operator`…）只要 Switch 加 case、`kValidActions` 加名字就能用，解析的程式不用改；不支援的名稱回 `BAD_SPEC` |
| 能測 | 完成、失敗的回報：做完回 `success`，閘道的 log 印「已確認完成」；失敗照 `reason`（之後可以填 `failure_code`，第 0.1 節 B4） |
| 還沒定 | **每個動作的參數格式**（`wait` 是秒數還是條件、`track` 要哪些 params…）：要寫進註冊表的 `actions.yaml`，你定 Action 介面（REG-2）時一起定。下面例子的參數名稱只是暫定 |
| 還沒定 | 同一個任務改版（revision 變大）時 BT 怎麼處理（第 0.1 節 C1） |

### steps 的格式

```json
{"steps": [
  {"action": "go_to",   "location": {"x": 10.0, "y": 5.0}},
  {"action": "wait",    "params": {"duration": 5}},
  {"action": "inspect", "location": {"x": 11.0, "y": 5.0, "z": 1.5}, "params": {"snapshots": 2, "timeout": 30}}
]}
```

| step 的欄位 | 意思 | 翻成你的 Task |
|---|---|---|
| `action` | 動作名稱（通用名稱） | `action`（要在 `kValidActions`、Switch 有那個 case） |
| `location` | 動作的位置（map ENU） | **第一步的 `go_to`**：站的位置（外部點的 x、y）；`inspect`：雲台的 `target.x／y／z`（沒給看任務位置、沒給 z 用 `target_z`）；其他動作：`params` 的 `location.x`、`location.y`、`location.z` |
| `object` | 動作的對象（例如 `track` 的目標 id） | `params` 的 `object` |
| `params` | 其他參數 | 攤平放進 `params`（字串）；`inspect` 沒寫 `snapshots` 時拍 1 張 |

做完最後一步由 BT 自己補 `report_result`，steps 裡不用寫。spec 是空字串時照舊（舊型任務：固定做一次雲台）。

### 加一個新動作（例如 `wait`）

1. `bt/mission.xml` 的 Switch 加一個 case，名稱用通用名稱（`wait`），接你的葉節點。
2. `kValidActions` 加 `"wait"`。
3. 葉節點從 `params` 讀參數（例如 `duration`）。

`parseSpecSteps` 不用改。名稱不在 `kValidActions` 時，整個任務回 `BAD_SPEC`（PERMANENT，CBBA 不再派給這隻）。

### 啟動（4 個 terminal，都設同一個 `ROS_DOMAIN_ID`，都 source 好你的工作區）

```bash
# terminal 1：閘道（機號 200，不出價；提供 /gcs/inject_task）
ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/gcs -r __node:=gcs_cbba_node \
  --params-file $(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config/cbba_gateway.yaml

# terminal 2：你的狗的 cbba_node（機號 50）
ros2 launch swarm_cbba cbba.launch.py namespace:=v60

# terminal 3：你的 BT（agent_id 50；用你平常的 launch，或下面這行）
ros2 run task_manager task_manager_node --ros-args -r __ns:=/v60 -r __node:=v60_task_manager \
  -p mission_file:=<沒有巡檢點的任務檔> -p bt_file:=$(ros2 pkg prefix task_manager)/share/task_manager/bt/mission.xml \
  -p autostart:=true -p agent_id:=50 -p nav_action:=/v60/navigate_to_pose -p ptz_action:=/v60/ptz_inspect \
  -p base_frame:=v60/base_link -p battery_topic:=/v60/battery

# terminal 4（沒有 Nav2、雲台時才要）：假的身體（直線走、雲台等幾秒回 OK、發 TF），不用 Gazebo
python3 <repo>/docker/scripts/fake_nav_body.py --ns v60 --start 0 0
```

- 沒有巡檢點的任務檔可以用 `<repo>/docker/dog/idle_mission.yaml`。
- 你的 BT 要發 `robot_state`（mode = AUTO）cbba_node 才會出價；只想先看分派時，terminal 2 改用
  `ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 --params-file <cbba_v60.yaml> -p participate:=always`（不看 robot_state）。

### 測試項目（從第 5 個 terminal 下任務）

下任務的指令（`STEPS` 換成下表每一項的 steps；`incident_id` 每次換一個，同一個會被當成重送）：

```bash
STEPS='{"steps":[{"action":"go_to","location":{"x":10,"y":5}},{"action":"inspect","location":{"x":11,"y":5,"z":1.5}}]}'
ros2 service call /gcs/inject_task swarm_interfaces/srv/InjectTask \
  "{incident_id: 'bt-test-1', revision: 1, platform: 2, required_capabilities: [mobility_ground], priority: 1,
    position: {x: 10.0, y: 5.0, z: 0.0}, value: 100.0, service_time_s: 10.0, spec: '$STEPS'}"
```

| # | steps | 預期 |
|---|---|---|
| S1 | `go_to` (10, 5) → `inspect` (11, 5, 1.5) | BT 走到 (10, 5)、雲台看 (11, 5, 1.5) 拍 1 張、回 `success`；閘道 log「任務 00C8.… 已確認完成」 |
| S2 | `go_to` → `inspect`（`params: {"snapshots": 3}`）→ `inspect`（另一個 location） | 兩次雲台，第一次拍 3 張、第二次 1 張 |
| S3 | `go_to` → `wait`（`params: {"duration": 5}`），**還沒加 `wait` 的 case 時** | BT 回 `BAD_SPEC: …`（PERMANENT），任務不執行；cbba_node log「這台不再接」 |
| S4 | 同 S3，**加了 `wait` 的 case 之後** | 走到後停 5 s、回 `success` |
| S5 | 你其他新動作（`track`、`speak`…），參數照你葉節點要的 | 走到對的葉節點；參數名稱和你的葉節點對不上時告訴我，一起寫進 `actions.yaml` |
| S6 | `spec` 給空字串 | 照舊型流程（走到任務位置前、固定做一次雲台） |

看的地方：你 BT 的 log（外部點的 tasks、Switch 走到哪個 case）、`ros2 topic echo /v60/current_task`（`spec` 欄位）、
`ros2 topic echo /v60/task_result`、閘道 terminal 的「已確認完成」。

---

## 0. 為什麼要換

你的 handoff 是對照 GitHub `dog-integration` 上 10/07 的程式（`uav_cbba`，機間協定版本 1）。之後 main 上已經改成：

- `uav_cbba` 改名 `cbba_core`，節點是統一版（無人機和狗同一支），放在 `swarm_cbba`。
- 機間協定改成**版本 2**（每筆任務帶 task_id、session 取代 seq_file、機號不限 1～8），2026-10-28 再升到**版本 3**
  （只改補發：壓縮摘要 → 缺口請求 → 限速分批，弱網時流量不再隨已結束的任務數增加，`cbba_core/doc/protocol_v3_sync.md`）。
  **版本 1、2、3 互相拒收**，全隊（無人機、狗）要用同一版的 `cbba_core`、`swarm_cbba` 編譯。
  2026-11-11 起是**版本 4**（第 0.1 節）：task_id 改 uint64、`Task.msg` 加欄位，BT 要套 patch。
- 你 handoff 的 patch 套不上 main（`uav_cbba` 已經不存在），**不用套**。

格式（套件名、launch、yaml、話題、訊息、QoS）完全照你的；底下換成協定版本 4 的核心，分配行為有幾點不同（第 6 節）。

## 1. 取得程式

從 GitHub 的分支 **`2-1b-m0`** 拿這三個套件（2026-11-14 起：2-1b M0 的改動先放在這個分支，main 停在 11/12 的版本，
等我們約好一起換時再合進 main；不是 `dog-integration`）：

| 套件 | 是什麼 |
|---|---|
| `ros2_ws/src/swarm_interfaces` | 訊息 v2.0＝你的 v1.2（RobotState 的 `mode`、TaskResult 的 `reason`）＋ `ManualOverride.srv`＋`FireDetection`（無人機用）＋ 2-1b 的欄位與 `TaskOrphaned`、`Detection`、`InjectTask.srv`（第 0.1 節） |
| `ros2_ws/src/cbba_core` | CBBA 核心、機間協定版本 4、指派流程、閘道（不依賴 ROS） |
| `ros2_ws/src/swarm_cbba` | 統一版 `cbba_node`、`launch/cbba.launch.py`、`config/cbba_v60.yaml`、`config/cbba_uav.yaml` |

```bash
git clone -b 2-1b-m0 https://github.com/ZHG169/px4_cbba_bt.git
cp -r px4_cbba_bt/ros2_ws/src/{swarm_interfaces,cbba_core,swarm_cbba} <你的工作區>/src/
```

**你工作區裡舊的要拿掉**（名稱一樣的套件不能同時存在）：`swarm_interfaces`、`swarm_cbba`、`uav_cbba`、`ugv_cbba`
——刪掉，或在資料夾裡放一個空的 `COLCON_IGNORE`。

## 2. 編譯

訊息定義變了，**用到 `swarm_interfaces` 的套件全部要重編**（只編一半，DDS 會因為型別雜湊不同而靜默收不到）：

```bash
cd <你的工作區>
colcon build --symlink-install --packages-select swarm_interfaces cbba_core swarm_cbba task_manager
source install/setup.bash
```

確認：

```bash
ros2 interface show swarm_interfaces/msg/RobotState | grep mode          # 有 AUTO / MANUAL / ESTOP / mode
ros2 interface show swarm_interfaces/msg/TaskResult | grep reason        # 有 NONE / PERMANENT / … / reason
ros2 interface show swarm_interfaces/srv/ManualOverride                  # Task task、bool clear / bool accepted、string message
ros2 interface show swarm_interfaces/msg/Task | grep assignment_version  # 應該沒有輸出
ros2 interface show swarm_interfaces/msg/Task | grep -E "uint64 task_id|revision|spec"   # 版本 4：三行都要有
ros2 interface show swarm_interfaces/msg/Task | grep -E "required_capabilities|expires"  # 介面 v2.0：兩行都要有
ros2 interface show swarm_interfaces/msg/TaskResult | grep failure_code                 # 介面 v2.0
ros2 interface show swarm_interfaces/msg/Detection | head -3                            # 2026-11-14 新增
```

`ManualOverride.srv` 是照介面規格圖（`cbba_core/doc/spec/ugv_interface.png`）放的，欄位和你 `task_manager_node.cpp` 的用法一致；
如果和你手上的原檔不同，請告訴我。

## 3. 啟動

和你原本一樣：

```bash
ros2 launch swarm_cbba cbba.launch.py namespace:=v60          # 預設 params 就是 config/cbba_v60.yaml（機號 50）
```

- 第二隻狗：`ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v61 --params-file <cbba_v60.yaml> -p agent_id:=51`（命令列的 `-p` 蓋過 yaml）。
- **mesh 網卡**：機間 UDP 走哪張網卡由 `mesh_ip` 參數或環境變數 `MESH_IP` 決定；沒設時用系統預設網卡。實機要設成 mesh 電台那張。
- **程序鎖（新的）**：節點啟動時在 `/tmp/cbba/agent50.lock`（或環境變數 `CBBA_LOCK_DIR`）加鎖，**同一個機號只能開一個**；
  舊的沒停乾淨時，新的會拒絕啟動並印出佔用的 PID。同一台機器開多個容器時，`CBBA_LOCK_DIR` 要指到共用的目錄。
- **對時**：各機要用 chrony／NTP 對時（任務期限、時間戳都是系統時鐘）。

啟動時會印一行（確認話題都在 `/v60/` 底下）：

```
agent 50（UGV）：UDP 239.255.42.99:14600（網卡 系統預設），session …；話題 /v60/current_task /v60/robot_state /v60/task_result /v60/new_task；參與出價：robot_state 在 state_timeout 內有更新且 mode = AUTO
```

## 4. 和你的 BT 之間的介面（和你的 handoff 相同）

| 話題 | 方向 | 型別 | 你的 BT 要做的 |
|---|---|---|---|
| `/v60/current_task` | CBBA → BT | `Task`（reliable、transient_local、depth 1） | 收到就插隊執行；`task_id = 0` 是取消 |
| `/v60/robot_state` | BT → CBBA | `RobotState` | 約 2 Hz；`agent_id` 要是 50；位置 map ENU；`mode`：AUTO 才會被派任務，MANUAL／ESTOP 時 CBBA 立刻釋放手上的任務 |
| `/v60/task_result` | BT → CBBA | `TaskResult`（reliable） | 一個任務一次；**task_id 要是目前 `current_task` 上的**；不用帶指派版本 |
| `/v60/new_task` | BT → CBBA | `Task` | 狗也可以建立任務（task_id = 50 × 65536 + 流水號，重開機後流水號要接續） |

- **失敗的 `reason`**：`PERMANENT` → 這隻狗永久不再接這個任務；`PREEMPTED`（`PREEMPTED_BY_MANUAL`）→ **5 s** 內不接
  （`preempt_exclude_sec`，2026-10-17 起：只擋「回報先到、mode = MANUAL 晚到」；接管期間靠 MANUAL 不出價，做完回到 AUTO 就能接回）；
  `TRANSIENT`、`STOPPED` → **60 s** 內不接（`failure_exclude_sec`），之後恢復出價。
- **`TRANSIENT` 會讓之後的出價打折**（2026-10-17）：失敗 f 次，這隻狗對這個任務的出價用成功率 1/(1+f)（價值減半、到達時間多算重試），
  60 s 後通常由別隻接手，不會一直派回同一隻；只有你能做時仍會派回來重試。卡住、逾時請回 `TRANSIENT`，
  被平板叫走回 `PREEMPTED_BY_MANUAL`（不算失敗次數）。
- **assign_hold 0.4 s**：同一個任務在路徑第一位穩定 0.4 s 才發 `current_task`。
- **不握手**：CBBA 直接發 `current_task`，你的哨兵負責中斷。
- `/v60/cancel_task`（取消任務）是我們多的，你不用也沒關係。

## 5. 驗收：你 README 的 T1、T2、T9

5 個 terminal、同一個 `ROS_DOMAIN_ID`，都先 `source install/setup.bash`。**假無人機也要用 main 的 `swarm_cbba`**（舊的會被拒收）。

| T | 指令 |
|---|---|
| 1 狗 | `ros2 launch swarm_cbba cbba.launch.py namespace:=v60` |
| 2 假無人機 | `ros2 launch swarm_cbba cbba.launch.py namespace:=uav1 params:=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config/cbba_uav.yaml` |
| 3 狗的假 BT | `ros2 topic pub -r 2 /v60/robot_state swarm_interfaces/msg/RobotState "{agent_id: 50, position: {x: 5.0, y: 20.0, z: 0.0}, battery: 90.0, mode: 0}"` |
| 3' 無人機的假 BT | `ros2 topic pub -r 2 /uav1/robot_state swarm_interfaces/msg/RobotState "{agent_id: 1, position: {x: 0.0, y: 0.0, z: 5.0}, battery: 90.0, mode: 0}"` |
| 4 看指派 | `ros2 topic echo /v60/current_task --qos-durability transient_local --qos-reliability reliable` |

2026-10-12 在我們這邊照跑的結果（狗 node 的 log）：

| # | 做法（terminal 5） | 狗印出的（實測） |
|---|---|---|
| T1 收到火點 | `ros2 topic pub --once -w 1 /uav1/new_task swarm_interfaces/msg/Task "{task_id: 65537, type: 2, position: {x: 6.0, y: 15.0, z: 0.0}, value: 100.0, duration_sec: 20.0, deadline_sec: 120.0}"` | `路徑：00010001` → 約 0.4 s 後 `指派給 BT：00010001 (6.0, 15.0, 0.0)`、`量測 00010001：…建立→發布 408 ms`；T4 收到 `task_id: 65537`、`type: 2`、`value: 100`、`duration_sec: 20`、`deadline_sec: 120.0`；無人機不接 |
| T2 回報完成 | `ros2 topic pub --once -w 1 /v60/task_result swarm_interfaces/msg/TaskResult "{task_id: 65537, success: true, reason: 0}"` | `task_result 00010001：success 1，reason 0，detail「」（NONE）` → `任務 00010001 完成，等待全隊確認` → `指派給 BT：無` → `已確認完成（回報後 0.01 s）`；T4 收到 `task_id: 0`；無人機印 `已確認完成（收到完成證明）` |
| T9 mode 接管 | 再發一個火點（`task_id: 65538`）等到指派 → T3 改 `mode: 1` 重發 → 等 2 s → 改回 `mode: 0` | `mode 0 → 1（AUTO → MANUAL）` → `停止參與出價，釋放手上的任務` → `指派給 BT：無`；改回後 `mode 1 → 0（MANUAL → AUTO）` → `開始參與出價` → `指派給 BT：00010002` |

和你 README 的預期**行為一樣**，log 也照你腳本 grep 的字串（2026-10-12 對齊）：
`mode 0 → 1`、`task_result …：success 0，reason 2，detail「NAV_NO_ACK」`、`失敗（NAV_NO_ACK），交回競標池`、
`已確認完成（收到完成證明）` 都一樣，只是後面多印一些資訊（`（AUTO → MANUAL）`、`（TRANSIENT）`、`這台 60 s 內不接`）。
其他不同的地方：
- `deadline_sec` 是 `120.0`（你預期 `119.98`）：新協定不再把時間取整到 ms。
- `指派給 BT` 後面多印座標。

你的 T1～T10 和 DEMO3 我們寫成了一支腳本 `test/g2_e2e.sh`（不用 BT，約 3 分鐘，61 項；2026-10-17 加失敗次數、孤兒任務警告的檢查），另外加了 E1～E4 驗下一節的差異：
`bash src/swarm_cbba/test/g2_e2e.sh`（ROS_DOMAIN_ID 93、UDP port 14693，不會干擾正在跑的節點）。

**跑你的 `e2e_cbba_node.sh`（G3）時**：狗的 node 要加 `-p failure_exclude_sec:=1e9 -p preempt_exclude_sec:=1e9`（或寫進 yaml；
2026-10-17 起 `cbba_v60.yaml` 的 `preempt_exclude_sec` 是 5 s，第 3 幕平板接管交還的任務 5 s 後就會派回來）。
失敗後只排除 60 s（你的規則 3），腳本約 4 分鐘，第 2、3、5 幕失敗的任務 60 s 後會派回來，
閒置的樹會自己開跑、打亂後面幾幕的計數。加了之後行為和你 10/07 跑的一樣（失敗的永遠不回來）。

## 6. 和 10/07 版本的行為差異（格式不變，CBBA 怎麼分配變了）

| # | 情境 | 10/07 的版本 | 現在 |
|---|---|---|---|
| 1 | 狗做到一半，別台出價比較高 | 任務被換走，`current_task` 變、哨兵中斷 | **執行中的任務鎖住**，只有失敗、取消、更高優先級（火警 > 巡檢）才換。你 ugv_test 記的「執行中的任務被換掉」就是這樣解決的 |
| 2 | 處理火警時又出現第二個火警 | 分數高的可能取代手上的 | 手上的做完前**不換、也不預先排隊**；第二個交給別的狗，沒有別的狗就等做完再接 |
| 3 | 狗短暫斷線 | 1.5 s 後它的任務全部改派 | 還沒開始的 1.5 s 後釋放；**執行中的先鎖著**，失聯後火警 10 s、巡檢 30 s 沒恢復才重新競標（失聯接手，參數 `takeover_fire`、`takeover_patrol`，全隊要一樣）；恢復連線時已被接手的話，它的指派撤下（`current_task` 變 0）、舊的回報被拒 |
| 4 | 回報不是目前指派的任務 | 只要任務還沒結束都接受 | **拒絕**（log 會寫），避免把已改派給別台的任務關掉。照你範本的寫法（任務被換掉時不回報）很少遇到 |
| 5 | 任務數量、建立者 | 每台約 32 個；機號 > 8（狗）不能建立 | 沒有限制，狗也能建立任務 |

另外：平板接管時，你的 BT 會先回報手上的 CBBA 任務失敗（`PREEMPTED`）。接管期間 mode = MANUAL 不出價；**2026-10-17 起 PREEMPTED 只排除 5 s**（`preempt_exclude_sec`），接管做完、回到 AUTO 後，那個任務還沒人接的話就能接回（原本 60 s）。PREEMPTED 不算失敗次數。

## 7. 常見問題

| 現象 | 檢查 |
|---|---|
| BT 收不到 `current_task`、或 CBBA 收不到 `robot_state` | `swarm_interfaces` 兩邊沒用同一版編譯（重編 task_manager）；`ROS_DOMAIN_ID` 不同；namespace 不同 |
| 狗一直不出價 | `robot_state` 有沒有 2 Hz 在送、`agent_id` 是不是 50（不是會印「agent_id 是 …：略過」）、`mode` 是不是 0（AUTO） |
| 看不到無人機（`收斂：…，0 個鄰居`） | 對方也要是 main 的 `swarm_cbba`（協定版本不同會被拒收，全隊要同一版）；`mesh_ip`／`MESH_IP` 網卡對不對；multicast 能不能過 |
| 回報被拒（`回報拒絕：不是目前 current_task 上的任務`） | task_id 要是 `current_task` 最新那一則 |
| 節點拒絕啟動（`already has a running cbba_node`） | 同機號的舊節點還在跑：`pkill -f cbba_node`，或換 `agent_id` |
| 地面任務一直沒人接 | 狗的 `mode` 不是 AUTO、或這個任務在 60 s 排除期間（之前失敗過） |

## 8. 參數（`config/cbba_v60.yaml` 或 `-p`）

| 參數 | 預設 | 說明 |
|---|---|---|
| `agent_id` | 50 | |
| `agent_type` | `UGV` | |
| `state_timeout` | 1.5 s | `robot_state` 多久沒更新就停止出價 |
| `assign_hold` | 0.4 s | |
| `failure_exclude_sec` | 60 s | 失敗（TRANSIENT、STOPPED）後多久不接同一個任務 |
| `preempt_exclude_sec` | 5 s（yaml） | PREEMPTED 後多久不接 |
| `max_failures` | 0 | 同一個任務 TRANSIENT 失敗幾次後永久不接；0 = 不啟用（只打折） |
| `orphan_warn_sec` | 30 s | 任務開著但這麼久沒有人得標（例如只有你能做、你回報了 PERMANENT）就印警告、發 `/v60/task_orphaned`（2026-11-14 起型別 `TaskOrphaned`，帶原因）；你的 BT 不用訂 |
| `cruise_speed`、`energy_per_meter`、`hover_energy_per_sec`、`safety_reserve` | 1.0、0.1、0.05、20 | **佔位值，請用實測換掉**（`cbba_core/doc/ugv_tuning.md`） |
| `battery_weight`、`cost_ref`、`max_bundle` | 1.0、50、5 | 出價公式，**全隊要一樣**，不要只改狗的 |
| `mesh_ip` | 環境變數 `MESH_IP` | |
| `udp_group`、`udp_port` | 239.255.42.99、14600 | 全隊要一樣 |

完整的參數與規則：`swarm_cbba/README.md`；機間協定：`cbba_core/doc/protocol.md`。
