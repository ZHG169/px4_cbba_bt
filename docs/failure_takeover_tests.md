# 任務失敗、平板接管、地面端的手動測試（2026-10-19）

驗 10/17 加的 CBBA 行為，全部由使用者手動跑。兩組：

| 組 | 環境 | 要多久 | 測什麼 |
|---|---|---|---|
| **A. 地面端（狗、平板）** | `gcs` 容器，只開 cbba_node＋假的狗狀態，**不用 PX4、Gazebo** | 約 15 分鐘 | 兩隻狗之間的失敗交接、打折、孤兒任務、平板接管、急停、取消 |
| **B. 無人機端** | `itest.py demo --interactive`（PX4＋Gazebo，三台） | 約 10 分鐘 | 被遙控器接管、TRANSIENT／PERMANENT 失敗、急停、取消 |

規則的定義在 `CLAUDE.md`「失敗後的排除」、`swarm_cbba/README.md`；這份只寫怎麼測、要看到什麼。

| reason | 排除 | 失敗次數 | 誰會回報 |
|---|---|---|---|
| 1 PERMANENT | 這台永久不接 | 不算 | 地圖外、沒路（`NAV_FAILED(204)`、`(208)`） |
| 2 TRANSIENT | 60 s（`failure_exclude_sec`） | **+1 → 之後出價打折**（成功率 1/(1+f)） | 卡住、逾時、`NAV_NO_ACK` |
| 3 PREEMPTED | 5 s（`preempt_exclude_sec`） | 不算 | 平板接管 `PREEMPTED_BY_MANUAL`、無人機被遙控器接管 |
| 4 STOPPED | 60 s | 不算 | 低電量返航、降落、PX4 失效保護、急停 |

孤兒任務：任務開著、30 s（`orphan_warn_sec`）沒有人得標 → 警告＋`task_orphaned`。

---

## A. 地面端（gcs 容器）

### A0. 準備（開 4 個 terminal，全部在 gcs）

和正在跑的模擬隔離：另一個 ROS domain、另一個 UDP port、另一個程序鎖目錄。**每個 terminal 都先貼這三行**：

```bash
./docker/enter.sh gcs          # 在主機的 ~/CBBA_BT
source ~/CBBA_BT/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=95 CBBA_LOCK_DIR=/tmp/cbba_fail_lock CFG=$(ros2 pkg prefix swarm_cbba)/share/swarm_cbba/config
```

| terminal | 做什麼 | 指令 |
|---|---|---|
| 1 | 狗 A（機號 50，`/v60`） | `ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60 -r __node:=t_v60 --params-file $CFG/cbba_v60.yaml -p agent_id:=50 -p udp_port:=14695` |
| 2 | 狗 B（機號 51，`/v60_1`） | `ros2 run swarm_cbba cbba_node --ros-args -r __ns:=/v60_1 -r __node:=t_v60_1 --params-file $CFG/cbba_v60.yaml -p agent_id:=51 -p udp_port:=14695` |
| 3 | 兩隻狗的狀態（代替狗的 BT） | `python3 ~/CBBA_BT/docker/scripts/fake_robot_state.py --ns v60 --id 50 --pos 0 -10 & python3 ~/CBBA_BT/docker/scripts/fake_robot_state.py --ns v60_1 --id 51 --pos 0 -30` |
| 4 | 下指令（建任務、回報、切 mode） | 下面各項 |

- 狗 A 在 (0, −10)、狗 B 在 (0, −30)；任務放在誰旁邊，誰就先得標。
- 切 mode：`echo 1 > /tmp/fake_mode_v60`（MANUAL）、`echo 2 > …`（ESTOP）、`echo 0 > …`（AUTO）；狗 B 的檔案是 `/tmp/fake_mode_v60_1`。
- 兩隻都印出「開始參與出價」、「收斂：0 個任務，1 個鄰居」再開始。

terminal 4 先定義幾個指令（之後直接用）：

```bash
# 狗 A 建立地面任務：task_id = 50 × 65536 + 流水號（log 裡是 8 位十六進位，例如流水號 1 → 00320001）
ground() { ros2 topic pub --once -w 1 /v60/new_task swarm_interfaces/msg/Task \
  "{task_id: $((50 * 65536 + $1)), type: 2, position: {x: $2, y: $3, z: 0.0}, value: 100.0, duration_sec: 20.0, deadline_sec: 300.0}"; }
# 回報：result <v60|v60_1> <流水號> <true|false> <reason> [detail]
result() { ros2 topic pub --once -w 1 /$1/task_result swarm_interfaces/msg/TaskResult \
  "{task_id: $((50 * 65536 + $2)), success: $3, reason: $4, detail: '${5:-}'}"; }
# 取消：cancel <v60|v60_1> <流水號>
cancel() { ros2 topic pub --once -w 1 /$1/cancel_task swarm_interfaces/msg/Task "{task_id: $((50 * 65536 + $2))}"; }
```

### A1. TRANSIENT 失敗 → 另一隻接手（打折）

```bash
ground 1 0 -12                                   # 靠近狗 A
# 等狗 A（terminal 1）印「指派給 BT：00320001」
result v60 1 false 2 'NAV_FAILED(TIMEOUT)'
```

| 要看到 | 在哪 |
|---|---|
| `任務 00320001 失敗（NAV_FAILED(TIMEOUT)），交回競標池，這台 60 s 內不接` | terminal 1 |
| `任務 00320001 這台失敗 1 次，估計成功率 0.50，之後的出價打折` | terminal 1 |
| 約 0.5 s 內 `指派給 BT：00320001` | terminal 2（狗 B 接手） |

收尾：`result v60_1 1 true 0` → 兩邊都印「已確認完成」。

### A2. 只剩一隻能做 → 60 s 後打折重試＋孤兒任務

```bash
echo 1 > /tmp/fake_mode_v60_1                     # 狗 B 平板接管中（不出價）
ground 2 0 -12
# 狗 A 得標後
result v60 2 false 2 'NAV_FAILED(TIMEOUT)'
```

| 時間（從回報算） | 要看到（terminal 1） |
|---|---|
| 0 s | `這台 60 s 內不接`、`失敗 1 次，估計成功率 0.50` |
| 約 30 s | `孤兒任務 00320002 …：開著但 30 s 沒有人得標`（terminal 2 也會印） |
| 約 60 s | `路徑：00320002`、`任務 00320002 有人得標了（agent 50）`、`指派給 BT：00320002`（只有它能做，打折了還是會派回來） |

收尾：`result v60 2 true 0`、`echo 0 > /tmp/fake_mode_v60_1`。

### A3. PERMANENT → 永久不接、孤兒任務、別隻回來才接

```bash
echo 1 > /tmp/fake_mode_v60_1
ground 3 0 -12
# 狗 A 得標後
result v60 3 false 1 'NAV_FAILED(204)'
# 等孤兒任務警告出現後
echo 0 > /tmp/fake_mode_v60_1                     # 狗 B 回到 AUTO
```

| 要看到 | 在哪 |
|---|---|
| `任務 00320003 失敗（NAV_FAILED(204)），交回競標池，這台不再接`；**沒有**「失敗次數」那行 | terminal 1 |
| 約 30 s：`孤兒任務 00320003` | terminal 1、2 |
| 狗 B 回 AUTO 後：`有人得標了（agent 51）`、terminal 2 `指派給 BT：00320003` | terminal 1、2 |
| 之後狗 A **一直不再**對 00320003 出價（等 60 s 以上也一樣） | terminal 1 的「路徑」不會出現它 |

收尾：`result v60_1 3 true 0`。

### A4. 平板接管 → 交給別隻

照伯宇 BT 的順序：先回報手上的任務 `PREEMPTED_BY_MANUAL`，再把 mode 改成 MANUAL。

```bash
ground 4 0 -12
# 狗 A 得標後，兩行連著貼
result v60 4 false 3 'PREEMPTED_BY_MANUAL'
echo 1 > /tmp/fake_mode_v60
```

| 要看到 | 在哪 |
|---|---|
| `任務 00320004 失敗（PREEMPTED_BY_MANUAL），交回競標池，這台 5 s 內不接`；**沒有**「失敗次數」那行 | terminal 1 |
| `mode 0 → 1（AUTO → MANUAL）`、`停止參與出價，釋放手上的任務` | terminal 1 |
| 狗 B：`指派給 BT：00320004` | terminal 2 |

收尾：`result v60_1 4 true 0`、`echo 0 > /tmp/fake_mode_v60`。

### A5. 平板接管做完 → 回來接回原本的任務（5 s 的意義）

```bash
echo 1 > /tmp/fake_mode_v60_1                     # 狗 B 不能接，任務只能等狗 A
ground 5 0 -12
# 狗 A 得標後
result v60 5 false 3 'PREEMPTED_BY_MANUAL'
echo 1 > /tmp/fake_mode_v60
# 等 15 s（模擬平板指定的事做完）
echo 0 > /tmp/fake_mode_v60
```

| 要看到 | 在哪 |
|---|---|
| 接管期間沒有人接 00320005（15 s 內不會有孤兒任務警告，30 s 才會） | terminal 1、2 |
| 回 AUTO 後**馬上**（1 s 內）`指派給 BT：00320005`：5 s 的排除早就過了，不用等到 60 s | terminal 1 |

收尾：`result v60 5 true 0`、`echo 0 > /tmp/fake_mode_v60_1`。

### A6. 急停（ESTOP）

```bash
ground 6 0 -12
# 狗 A 得標後
echo 2 > /tmp/fake_mode_v60
# 10 s 後
echo 0 > /tmp/fake_mode_v60
```

| 要看到 | 在哪 |
|---|---|
| `mode 0 → 2`、`停止參與出價，釋放手上的任務`、`指派給 BT：無`（不需要 task_result） | terminal 1 |
| 狗 B 接手 `指派給 BT：00320006` | terminal 2 |
| 狗 A 回 AUTO 後開始參與出價，但 00320006 已經被狗 B 鎖住，不會搶回來 | terminal 1 |

收尾：`result v60_1 6 true 0`。

### A7. 地面端取消（建立者才可以）

```bash
ground 7 0 -28                                   # 靠近狗 B，由狗 B 執行
# 狗 B 得標後
cancel v60_1 7                                    # 不是建立者（建立者是狗 A，機號 50）
cancel v60 7                                     # 建立者
```

| 要看到 | 在哪 |
|---|---|
| 第一次：`取消任務 00320007 拒絕：…`（`cancel_authorities` 是空的，只有建立者能取消） | terminal 2 |
| 第二次：`取消任務 00320007，通知全隊`；兩邊都印 `任務 00320007 已取消`；狗 B `指派給 BT：無` | terminal 1、2 |

### A8. 收尾

terminal 1～3 按 Ctrl-C。這組用的是 domain 95、port 14695、`/tmp/cbba_fail_lock`，不影響正在跑的模擬。

---

## B. 無人機端（itest demo --interactive）

### B0. 準備

```bash
# terminal 1（主機）：開環境，等「CBBA 上線，開始接受指令」
cd ~/CBBA_BT
python3 docker/itest/itest.py demo --interactive --no-patrol --seed 7 --kill-existing
# terminal 2（主機）：插入任務
python3 docker/itest/insert.py
# terminal 3（主機）：看 log
cd ~/CBBA_BT/docker/itest/out && tail -f $(ls -td *_demo | head -1)/stack_uav*.log | \
  grep -E "警告|task_result|失敗|成功率|孤兒|有人得標|指派給 BT|mode|取消|急停"
```

需要時再開：`./docker/enter.sh sim`（切 PX4 模式）、`./docker/enter.sh uavN`（回報、急停）。
在 insert.py 輸入 `l` 看每個任務的 task_id 和執行者。

PX4 的模式在 sim 容器用命令列工具切（itest 在背景開 PX4，沒有 `pxh>`）：

```bash
PX4CMD=/opt/PX4-Autopilot/build/px4_sitl_default/bin/px4-commander
$PX4CMD --instance 1 mode auto:loiter       # uav1 被接管（模擬沒有遙控器，posctl 會被拒）
$PX4CMD --instance 1 mode offboard          # 交回
```

### B1. 被遙控器接管 → 交給別台

1. insert.py：`t 18 -10 d=10`，`l` 看執行者（假設 uav1）。
2. 飛到一半：`$PX4CMD --instance 1 mode auto:loiter`；10 s 後 `mode offboard`。

| 要看到 |
|---|
| uav1：`警告：被接管（…failsafe 0）`、`reason 3，detail「PREEMPTED_BY_MANUAL」`、`這台 5 s 內不接`、`mode → MANUAL`；**沒有**「失敗次數」 |
| 別台很快 `指派給 BT：…`、把任務做完 |
| uav1 交回後：`切回 offboard…`、`mode → AUTO`，原地爬回 5.0 m；巡檢報告寫 `中止（PREEMPTED_BY_MANUAL）`、沒有「中斷延遲」 |

### B2. TRANSIENT 失敗 → 打折、別台接手

1. insert.py：`t -15 6 d=10`，`l` 看執行者和 task_id（假設 uav2、`0002xxxx`）。
2. 飛行途中，在 **uav2 容器**：`cbba_task.sh fail 2 0002xxxx`（reason 預設 2）。

| 要看到 |
|---|
| uav2：`失敗（MANUAL），交回競標池，這台 60 s 內不接`、`這台失敗 1 次，估計成功率 0.50，之後的出價打折` |
| 別台接手完成 |

### B3. PERMANENT 失敗

同 B2，但 `cbba_task.sh fail 2 0002xxxx 1`。

| 要看到 |
|---|
| uav2：`交回競標池，這台不再接`；沒有「失敗次數」；別台接手 |

### B4. 急停

在 **uav1 容器**：

```bash
ros2 topic pub --once /uav1/emergency_stop std_msgs/msg/Bool "{data: true}"
# 10 s 後
ros2 topic pub --once /uav1/emergency_stop std_msgs/msg/Bool "{data: false}"
```

| 要看到 |
|---|
| uav1：`緊急停止：TRUE`、`mode → ESTOP`、原地懸停；cbba `停止參與出價，釋放手上的任務`；手上有任務時別台接手 |
| `FALSE` 之後：`mode → AUTO`、`開始參與出價` |

### B5. 取消

1. insert.py：`t 10 -10`（insert.py 的任務由某台建立，`l` 看 task_id，例如 `0003xxxx` → 建立者是 uav3）。
2. 在**建立者**的容器（uav3）：`cbba_task.sh cancel 3 0003xxxx`。

| 要看到 |
|---|
| `取消任務 … 通知全隊`；全部 `已取消`；執行的那台 `CBBA 取消任務 → 哨兵下一拍 halt`、停在原地 |
| 在**非建立者**的容器取消：`取消任務 … 拒絕` |

### B6. 結束

insert.py 輸入 `q`。被取消的任務不算進 D1；其他手動插入的任務都應該完成。

---

## 結果紀錄

| 項目 | 結果 | 備註 |
|---|---|---|
| A1 TRANSIENT → 另一隻接手 | | |
| A2 只剩一隻 → 60 s 後重試＋孤兒 | | |
| A3 PERMANENT → 孤兒 → 別隻接 | | |
| A4 平板接管 → 交給別隻 | | |
| A5 接管做完 → 馬上接回 | | |
| A6 急停 | | |
| A7 取消（建立者／非建立者） | | |
| B1 被遙控器接管 | | |
| B2 TRANSIENT | | |
| B3 PERMANENT | | |
| B4 急停 | | |
| B5 取消 | | |

有不符合的，把對應 terminal 的 log（或 `docker/itest/out/<時間>_demo/stack_uav*.log`）貼過來。
