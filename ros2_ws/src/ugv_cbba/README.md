# ugv_cbba

機器狗（Ghost V60）端的 CBBA 節點 `ugv_cbba_node`。

CBBA 核心、機間通訊封包、協定層、節點骨架都用 `uav_cbba` 的同一份程式（`find_package(uav_cbba)`），
所以狗和無人機在網路上講的一定是同一套協定；這個套件只有接狗的 BT 的那一層。

## 編譯

```bash
cd ~/CBBA_BT/ros2_ws
colcon build --symlink-install --packages-select swarm_interfaces uav_cbba ugv_cbba
source install/setup.bash
```

`uav_cbba` 依賴 `px4_msgs`，所以狗的機上電腦也要先有 px4_msgs（只有編譯時需要，執行時不會用到 PX4）：
執行 `docker/scripts/setup_px4_msgs.sh` 下載 PX4 v1.17.0 的 px4_msgs，再 `colcon build --packages-select px4_msgs`。

## 執行

狗的機上電腦跑一個：

```bash
ros2 run ugv_cbba ugv_cbba_node      # 機號 50、話題 /v60/...；機號和命名空間可用環境變數 UGV_ID、UGV_NS 改
```

| 話題 | 型別 | 方向 | 說明 |
|---|---|---|---|
| `/v60/robot_state` | `swarm_interfaces/RobotState` | 狗的 BT → CBBA | 位置（map ENU）、電量，2 Hz；1.5 s 內有更新才參與出價 |
| `/v60/task_result` | `swarm_interfaces/TaskResult` | 狗的 BT → CBBA | `success = false` 時交回競標池；`detail` 記在 log |
| `/v60/assigned_task` | `swarm_interfaces/Task` | CBBA → 狗的 BT | 同無人機 |

- 狗只接 GROUND_INTERVENTION、PATROL；這些任務由無人機確認火情後建立（測試時用 `cbba_task.sh ground`）。
- 狗不建立任務。機號 50 沒有 AGENT_STATE 的鄰居位元，無人機完成任務時不等狗的確認（狗照樣收得到證明）。
- 能量模型是佔位值，調整方法見 [`doc/ugv_tuning.md`](doc/ugv_tuning.md)。

不接狗、只測協商時，用指令假裝狗的 BT：

```bash
ros2 topic pub -r 2 /v60/robot_state swarm_interfaces/msg/RobotState \
  "{agent_id: 50, position: {x: 5.0, y: 20.0, z: 0.0}, battery: 90.0}"
ros2 topic pub --once /v60/task_result swarm_interfaces/msg/TaskResult "{task_id: 65537, success: true}"
```

## 測試資料

給狗端 BT 的測試：用一台「假無人機」（不接 PX4 的 cbba_node）建立火點，看狗的 CBBA 怎麼分配、BT 怎麼回報。
下面每個情境的預期結果都是實際跑出來的（2026-10-07，dog-integration 分支）。

### 準備

都在同一台機器、同一個 `ROS_DOMAIN_ID` 下開 5 個 terminal（先 `source install/setup.bash`）：

| Terminal | 用途 | 指令 |
|---|---|---|
| 1 | 狗的 CBBA | `ros2 run ugv_cbba ugv_cbba_node` |
| 2 | 假無人機（只負責建立火點，自己不接地面任務） | `ros2 run uav_cbba cbba_node --ros-args -p agent_id:=1 -p use_px4:=false -p px4_ns:=uav1` |
| 3 | 狗的狀態（真的狗 BT 有送就不用） | 見下面 |
| 4 | 看指派給狗的任務 | `ros2 topic echo /v60/assigned_task --qos-durability transient_local --qos-reliability reliable` |
| 5 | 建立火點、回報結果 | 見下面的各情境 |

Terminal 3（假裝狗的 BT，2 Hz）：

```bash
ros2 topic pub -r 2 /v60/robot_state swarm_interfaces/msg/RobotState \
  "{agent_id: 50, position: {x: 5.0, y: 20.0, z: 0.0}, battery: 90.0}"
```

建立火點、回報結果的兩種寫法（`cbba_task.sh` 在 repo 的 `docker/scripts/`）：

```bash
# 建立火點：由 uav1 建立，task_id = 1 × 65536 + 流水號
cbba_task.sh ground 1 1 6 15
#   等同
ros2 topic pub --once -w 1 /uav1/new_task swarm_interfaces/msg/Task \
  "{task_id: 65537, type: 2, position: {x: 6.0, y: 15.0, z: 0.0}, value: 100.0, duration_sec: 20.0, deadline_sec: 120.0}"

# 狗的 BT 回報結果（請用 reliable，ros2 topic pub 預設就是）
ros2 topic pub --once -w 1 /v60/task_result swarm_interfaces/msg/TaskResult "{task_id: 65537, success: true}"
ros2 topic pub --once -w 1 /v60/task_result swarm_interfaces/msg/TaskResult "{task_id: 65539, success: false, detail: 'NAV2_ABORTED'}"
```

log 裡的 task_id 是 8 位十六進位：`00010001` = 65537、`00010002` = 65538……`00010008` = 65544。

### 情境

流水號在同一輪不能重複。全部重開後可以從 1 重新開始。

| # | 測什麼 | 做法 | 預期結果（狗的 log） |
|---|---|---|---|
| T1 | 收到火點 | Terminal 3 開始送 → `cbba_task.sh ground 1 1 6 15` | `開始參與出價` → `路徑：00010001` → 約 0.6 s 後 `指派給 BT：00010001 (6.0, 15.0, 0.0)`。Terminal 4 收到 `task_id: 65537`、`type: 2`、`value: 100`、`duration_sec: 20`、`deadline_sec: 120` |
| T2 | 回報完成 | `task_result` 送 `{task_id: 65537, success: true}` | `任務 00010001 已確認完成（回報後 0.01 s）` → `指派給 BT：無`（Terminal 4 收到 `task_id: 0`）。假無人機印 `已確認完成（收到完成證明）` |
| T3 | 回報失敗 | `cbba_task.sh ground 1 3 8 25` → 指派後送 `{task_id: 65539, success: false, detail: 'NAV2_ABORTED'}` | `任務 00010003 失敗（NAV2_ABORTED），交回競標池` → `指派給 BT：無`。狗不會再接這個任務；沒有別的地面載具，任務留在競標池 |
| T4 | 連續來好幾個火點 | 依序 `ground 1 4 20 20`、`ground 1 5 6 15`、`ground 1 6 -10 30`（間隔約 1 s），再依 Terminal 4 收到的順序逐一回報完成 | 路徑隨新火點重排：`00010004` → `00010005 → 00010004` → `00010006 → 00010005 → 00010004`，**BT 的指派在 2 s 內被換了兩次**。做完一個就指派下一個，三個都 `已確認完成` |
| T5 | 空中任務不會給狗 | `cbba_task.sh new 1 2 10 0` | 狗的路徑不變；假無人機 `路徑：00010002` |
| T6 | 狗的 BT 停掉、再恢復 | `cbba_task.sh ground 1 7 2 16`，指派後停掉 Terminal 3，等 3 s 再開 | 停掉約 1.5 s 後 `停止參與出價，釋放手上的任務` → `指派給 BT：無`；恢復後 `開始參與出價` → 又接回 `00010007` |
| T7 | 電量不足 | Terminal 3 的 `battery` 改成 21 → `cbba_task.sh ground 1 8 -20 -10` → 再把 `battery` 改回 90 | 21% 時不出價（可用電量只剩 1%）；改回 90% 後 `路徑：00010008`、`指派給 BT：00010008` |
| T8 | robot_state 的機號不對 | Terminal 3 的 `agent_id` 改成 51 | `robot_state 的 agent_id 是 51，不是 50：略過`，不會開始參與出價 |

### 要注意的（給狗端 BT）

1. **執行中的任務可能被換掉**（T4）。新火點進來時，CBBA 會重新排路徑，`/v60/assigned_task` 會換成別的任務，BT 要能中途切換。
   0.6 s 的保持時間只擋掉協商途中的短暫來回，擋不住「新的排法穩定下來」的情況。
2. **排序不一定是近的先**（T4 最後是最遠的 (−10, 30) 排第一）。這是目前的成本公式造成的：狗走得慢、每個任務停 20 s，
   排在後面的任務到達時間拉長，成本放大，所以把新任務插在最前面反而比較便宜。如果狗端希望「近的先」或「手上的做完再換」，
   要回頭調出價公式或加「執行中不換手」的規則，請提出來。
3. **回報的 task_id 不檢查是不是指派給狗的**：任何還開著的任務都會被接受，甚至不是狗得標的也會。BT 請只回報 `/v60/assigned_task` 目前給的那個任務。
4. 這些測試只有一隻狗、一台假無人機；假無人機的 CBBA 不接地面任務，所以失敗或狗斷線時，任務會留在競標池沒人接。

## 參數

能量模型、出價、協定、UDP、`seq_file`、`assign_hold`、`tick_ms` 和無人機的 cbba_node 一樣（同一個骨架宣告的），
說明見 [`uav_cbba/doc/parameters.md`](../uav_cbba/doc/parameters.md)。狗不同的只有：

| 參數 | 預設 | 說明 |
|---|---|---|
| `agent_id` | 環境變數 `UGV_ID`，沒有時 50 | 介面規格的機號。超過 8 沒有鄰居位元 |
| `ns` | 環境變數 `UGV_NS`，沒有時 `v60` | 話題前綴 |
| `state_timeout` | 1.5 s | `robot_state` 這麼久沒更新就停止參與出價、釋放手上的任務 |
| 能量模型 | 0.1 %/m、0.05 %/s、1.0 m/s、保留 20% | **佔位值，未校正**，怎麼量見 [`doc/ugv_tuning.md`](doc/ugv_tuning.md) |
| `seq_file` | `~/.cbba/seq_agent50` | seq 預約紀錄 |

`udp_group`、`udp_port`、`index_slots`、`lost_timeout` **要和無人機一樣**。

## 還沒做的部分

- 還沒在真的狗上測（已和 2 台無人機走真 UDP 測過：地面任務只有狗出價、完成、失敗交回、robot_state 斷掉時釋放）
- 能量模型的實測值、Nav2 的路徑長度（目前用直線距離），待伯宇提供
