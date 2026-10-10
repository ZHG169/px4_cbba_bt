# 通用 CBBA 節點（做法三，規劃）

2026-10-07 · 狀態：**2026-10-09 已實作**（dog-integration 分支，未 push）。下面保留當時的規劃；和規劃不同或後來決定的：

| 項目 | 決定 |
|---|---|
| 無人機的任務結果 | 改用 `TaskResult` |
| 返航點 | 用第一筆 robot_state（離地後、空中 5 m）；只用在估返航耗電，差約 0.1% |
| 套件名稱 | `cbba_core`（namespace 也改成 `cbba_core`）；`ugv_cbba` 拿掉，狗的文件移到 `cbba_core/doc/ugv_*.md` |
| 轉接節點 | `px4_waypoint_node/px4_state_bridge` |
| 節點名稱 | 程式裡固定 `cbba_node`，啟動時 `-r __node:=uav1_cbba_node`、`v60_cbba_node` |
| 測協商（不接 PX4） | 原本的 `use_px4:=false` 改成 `participate:=always` |
| AGENT_STATE 旗標 | 一開始因為 RobotState 沒有欄位，armed 用「有在更新」代替、offboard 固定 0。**同日改成**：RobotState 加 `flight_state_valid`、`armed`、`offboard`、`landed`，轉接節點填 PX4 的真實值，AGENT_STATE 直接用 |
| 機號超過 8 | 一開始不接受 `new_task`（任務編號會和 1～8 撞）。**同日改成協定版本 2**（task_id 識別任務，見 `cbba_core/doc/protocol.md`），任何機號都能建立任務 |
| 返航點 | 改成轉接節點在地面就送之後，返航點是地面的出生點（上面「第一筆在空中 5 m」的取捨不再需要） |
| 回報檢查 | task_result 的 task_id 要是目前 assigned_task 上的那一個，否則拒絕 |

把 cbba_node 改成不分載具的通用節點：只吃一套標準的 ROS 介面，換載具時把話題接上（remap）就好，
像插件一樣。載具格式不同時，才另外寫一個小的轉接節點。

---

## 一、為什麼要改

dog-integration 目前的分法：

```
ugv_cbba ──依賴──→ uav_cbba ──依賴──→ px4_msgs
（狗的節點）        （共用的協定、骨架，加上無人機的節點）
```

| 問題 | 說明 |
|---|---|
| 名稱誤導 | 狗要依賴一個叫「uav」的套件，看不出哪些是共用的 |
| 狗要裝 px4_msgs 才能編譯 | `uav_cbba` 依賴 `px4_msgs`，狗根本不用 PX4 |
| 每種載具都要寫一個子類別 | 新增載具就要寫 C++、動編譯設定 |

## 二、輸入差在哪

ROS 的 remap 只能改話題名稱，**不能轉型別**。現在兩邊的輸入型別都不一樣：

| 輸入 | 無人機 | 狗 | 型別一樣 |
|---|---|---|---|
| 位置、電量 | PX4 的 4 個話題（NED、以出生點為原點，要轉換） | `RobotState`（已經是 map ENU） | ✗ |
| 參與出價的條件 | 解鎖且離地 | robot_state 有在更新 | ✗ |
| 任務結果 | `Task`（看 `status`） | `TaskResult`（看 `success`，有 `detail`） | ✗ |
| 指派的任務 | `Task` | `Task` | ✓ |
| 建立任務 | `Task` | 不建立 | ✓ |

## 三、考慮過的做法

| | 現在（狗繼承骨架） | 做法 1：參數切換 | 做法 2：pluginlib | **做法 3：標準介面＋轉接** | 拆法二：三個套件 |
|---|---|---|---|---|---|
| 怎麼做 | 每種載具一個子類別 | 同一個節點 `vehicle:=uav/ugv` | 執行時載入載具插件 | 通用節點只吃標準介面，格式不同的寫轉接節點 | 共用的獨立成 `cbba_core`，兩邊各一個節點 |
| 共用節點完全不知道載具 | ✗ | ✗ | ✓ | ✓ | ✗ |
| 狗不用裝 px4_msgs | ✗ | ✗ | ✓ | ✓ | ✓ |
| 新增載具要做什麼 | 寫子類別 | 改共用節點 | 寫插件 | **接話題**；格式不同才寫轉接 | 寫一個節點 |
| 協定只有一份 | ✓ | ✓ | ✓ | ✓ | ✓ |
| 複雜度 | 低 | 低 | 高（插件類別、XML、pluginlib） | 低 | 中（搬檔案） |
| 額外成本 | — | 兩邊程式混在一起 | 插件設定 | 無人機多一個程序、多一跳話題（約 1 ms） | — |

另一種「兩份完全獨立（各自複製協定）」不考慮：協定必須兩邊一樣，複製兩份時某一邊沒跟上不會報錯，只會在網路上悄悄對不上；
今天修的補發、開機同步、seq 撞號就要修兩次。

**選做法 3。**

## 四、做法 3 的設計

```
                      標準介面（大家都送一樣的型別）
狗的 BT ──────────────── RobotState ─────┐
                         TaskResult ─────┤
                                         ├──→ cbba_node（通用）──→ assigned_task（Task）
PX4 ──→ px4_state_bridge ── RobotState ──┤
        （無人機的轉接節點）               │
無人機的 BT ──────────── TaskResult ──────┤
                         new_task（Task）─┘
```

| 部分 | 做法 |
|---|---|
| 通用 cbba_node | 只訂閱 `state`（RobotState）、`task_result`（TaskResult）、`new_task`（Task），發 `assigned_task`（Task）。用參數設定機號、載具類型（UAV／UGV）、命名空間；話題用 remap 接 |
| 參與出價 | 統一成「`state` 在 `state_timeout` 內有更新」。無人機的轉接節點只在解鎖且離地時才送，**順便修好「只關 PX4 時會繼續出價」** |
| 無人機的轉接節點 `px4_state_bridge` | PX4 的 `vehicle_local_position`、`battery_status`、`vehicle_status`、`vehicle_land_detected` → RobotState。NED → map ENU、加出生點都在這裡 |
| 狗 | **什麼都不用寫**：狗端的 BT 本來就送 RobotState、TaskResult，直接接上。`ugv_cbba` 套件可以拿掉 |
| 巡檢高度 | 變成通用節點的參數 `recon_altitude`（0 = 不調整），無人機 5、狗 0 |
| 依賴 | 通用節點和核心不依賴 PX4；只有轉接節點依賴 px4_msgs |

### 套件怎麼放

| 套件 | 內容 |
|---|---|
| `cbba_core`（名稱待定） | CBBA 核心、封包、協定層、UDP、通用 cbba_node、離線模擬、單元測試、參數說明、規格書 |
| 無人機端（`px4_waypoint_node` 或新開 `uav_bridge`） | `px4_state_bridge`、`task_executor` |
| `ugv_cbba` | 拿掉（或只留狗的文件 `ugv_tuning.md`、測試資料） |

## 五、會改到哪些介面

| 層 | 會改嗎 |
|---|---|
| **機間通訊**（UDP 封包、協定、定案表） | **完全不會**。無人機和狗在網路上講的話一模一樣 |
| 狗的機內介面（RobotState、TaskResult、Task） | **不會**，狗的介面就是標準 |
| 無人機的機內介面 | **會改一項**：`/uavN/task_result` 從 `Task`（status）改成 `TaskResult`（success、detail）。無人機的 BT 還沒寫，現在改成本最低。這會改到 CLAUDE.md 定案的「cbba_node 與 BT 的介面」 |
| `cbba_task.sh`、`task_executor`、`cbba_uav.sh` | 跟著改：回報結果改送 TaskResult；`cbba_uav.sh` 多啟動轉接節點 |

## 六、要先決定的事

1. **無人機的任務結果改用 `TaskResult`**：同意嗎？
2. **返航點**：轉接節點離地後才送 RobotState，通用節點收到的第一個位置是空中 5 m。
   選項：轉接節點另外提供起飛點，或直接用第一筆（差 5 m 高度，對耗電影響很小）。
3. **套件名稱**：共用的套件叫 `cbba_core`？`swarm_cbba`？
4. **時機**：等狗端在 dog-integration 上測完再做（建議），還是現在就做。

## 七、要怎麼驗證

- 108 項單元測試照舊通過（協定層、核心都不動）。
- 通用節點＋轉接節點在 PX4 SITL 重跑 `uav_cbba/doc/sitl_test.md` 的全部步驟，結果和現在一樣。
- 通用節點接狗的話題，重跑 `ugv_cbba/README.md` 的 T1～T8，結果和現在一樣。
- 新增：只關 PX4（不關 cbba_node）時，1.5 s 後停止參與出價、釋放任務。
