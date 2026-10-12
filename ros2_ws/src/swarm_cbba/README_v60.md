# 機器狗（V60）端使用說明

給伯宇（狗端）。2026-10-12 · 勝翔

**一句話**：main 上的 CBBA 節點已經照你 handoff（2026-10-11）的格式做好了。你的 `task_manager`、`mission.xml`、
launch 指令、yaml 都不用改，只要**換掉三個套件、重新編譯**。

---

## 0. 為什麼要換

你的 handoff 是對照 GitHub `dog-integration` 上 10/07 的程式（`uav_cbba`，機間協定版本 1）。之後 main 上已經改成：

- `uav_cbba` 改名 `cbba_core`，節點是統一版（無人機和狗同一支），放在 `swarm_cbba`。
- 機間協定改成**版本 2**（每筆任務帶 task_id、session 取代 seq_file、機號不限 1～8）。**版本 1 和 2 互相拒收**，全隊要一起換。
- 你 handoff 的 patch 套不上 main（`uav_cbba` 已經不存在），**不用套**。

格式（套件名、launch、yaml、話題、訊息、QoS）完全照你的；底下換成協定版本 2 的核心，分配行為有幾點不同（第 6 節）。

## 1. 取得程式

從 GitHub 的 **main**（不是 `dog-integration`）拿這三個套件：

| 套件 | 是什麼 |
|---|---|
| `ros2_ws/src/swarm_interfaces` | 訊息 v1.6＝你的 v1.2（RobotState 的 `mode`、TaskResult 的 `reason`）＋ `ManualOverride.srv`＋`FireDetection`（無人機用，你用不到） |
| `ros2_ws/src/cbba_core` | CBBA 核心、機間協定版本 2、指派流程（不依賴 ROS） |
| `ros2_ws/src/swarm_cbba` | 統一版 `cbba_node`、`launch/cbba.launch.py`、`config/cbba_v60.yaml`、`config/cbba_uav.yaml` |

```bash
git clone -b main https://github.com/ZHG169/px4_cbba_bt.git
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

- **失敗的 `reason`**：`PERMANENT` → 這隻狗永久不再接這個任務；`TRANSIENT`、`PREEMPTED`、`STOPPED` → **60 s** 內不接（`failure_exclude_sec`），之後恢復出價。
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

你的 T1～T10 和 DEMO3 我們寫成了一支腳本 `test/g2_e2e.sh`（不用 BT，約 3 分鐘，58 項），另外加了 E1～E4 驗下一節的差異：
`bash src/swarm_cbba/test/g2_e2e.sh`（ROS_DOMAIN_ID 93、UDP port 14693，不會干擾正在跑的節點）。

**跑你的 `e2e_cbba_node.sh`（G3）時**：狗的 node 要加 `-p failure_exclude_sec:=1e9`（或寫進 yaml）。
失敗後只排除 60 s（你的規則 3），腳本約 4 分鐘，第 2、3、5 幕失敗的任務 60 s 後會派回來，
閒置的樹會自己開跑、打亂後面幾幕的計數。加了之後行為和你 10/07 跑的一樣（失敗的永遠不回來）。

## 6. 和 10/07 版本的行為差異（格式不變，CBBA 怎麼分配變了）

| # | 情境 | 10/07 的版本 | 現在 |
|---|---|---|---|
| 1 | 狗做到一半，別台出價比較高 | 任務被換走，`current_task` 變、哨兵中斷 | **執行中的任務鎖住**，只有失敗、取消、更高優先級（火警 > 巡檢）才換。你 ugv_test 記的「執行中的任務被換掉」就是這樣解決的 |
| 2 | 處理火警時又出現第二個火警 | 分數高的可能取代手上的 | 手上的做完前**不換、也不預先排隊**；第二個交給別的狗，沒有別的狗就等做完再接 |
| 3 | 狗短暫斷線 | 1.5 s 後它的任務全部改派 | 還沒開始的 1.5 s 後釋放；**執行中的先鎖著**（失聯接手：火警 10 s、巡檢 30 s，還在實作） |
| 4 | 回報不是目前指派的任務 | 只要任務還沒結束都接受 | **拒絕**（log 會寫），避免把已改派給別台的任務關掉。照你範本的寫法（任務被換掉時不回報）很少遇到 |
| 5 | 任務數量、建立者 | 每台約 32 個；機號 > 8（狗）不能建立 | 沒有限制，狗也能建立任務 |

另外：平板接管時，你的 BT 會先回報手上的 CBBA 任務失敗（`PREEMPTED`），所以**解除接管後 60 s 內，狗不會接回同一個任務**（你的規則 3）。

## 7. 常見問題

| 現象 | 檢查 |
|---|---|
| BT 收不到 `current_task`、或 CBBA 收不到 `robot_state` | `swarm_interfaces` 兩邊沒用同一版編譯（重編 task_manager）；`ROS_DOMAIN_ID` 不同；namespace 不同 |
| 狗一直不出價 | `robot_state` 有沒有 2 Hz 在送、`agent_id` 是不是 50（不是會印「agent_id 是 …：略過」）、`mode` 是不是 0（AUTO） |
| 看不到無人機（`收斂：…，0 個鄰居`） | 對方也要是 main 的 `swarm_cbba`（版本 1 會被拒收）；`mesh_ip`／`MESH_IP` 網卡對不對；multicast 能不能過 |
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
| `failure_exclude_sec` | 60 s | 失敗（不是 PERMANENT）後多久不接同一個任務 |
| `cruise_speed`、`energy_per_meter`、`hover_energy_per_sec`、`safety_reserve` | 1.0、0.1、0.05、20 | **佔位值，請用實測換掉**（`cbba_core/doc/ugv_tuning.md`） |
| `battery_weight`、`cost_ref`、`max_bundle` | 1.0、50、5 | 出價公式，**全隊要一樣**，不要只改狗的 |
| `mesh_ip` | 環境變數 `MESH_IP` | |
| `udp_group`、`udp_port` | 239.255.42.99、14600 | 全隊要一樣 |

完整的參數與規則：`swarm_cbba/README.md`；機間協定：`cbba_core/doc/protocol.md`。
