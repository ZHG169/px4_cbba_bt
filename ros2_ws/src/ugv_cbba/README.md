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
