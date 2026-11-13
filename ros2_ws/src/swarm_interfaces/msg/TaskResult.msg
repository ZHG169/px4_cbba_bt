# 任務執行結果（/<ns>/task_result，reliable、keep_last 10）
# BT 端在任務結束時發一次。CBBA node 收到後：
#   success = true  → completeTask()，並在 /swarm/tasks 發布 status = DONE
#   success = false → 從本機 bundle / path 釋放該任務，讓它回到競標池（不可標 DONE）
uint64 task_id                  # 2026-11-11 起 uint64（協定版本 4，同 Task.task_id）
bool success
string detail                   # 失敗原因，例如 "NAV2_ABORTED"、"TIMEOUT"、"PTZ_HW_ERROR"、"PREEMPTED_BY_MANUAL"；成功時可留空
# v1.2：失敗分類，給 CBBA 決定這隻還能不能再出價（spec 2.3 規則 3）。detail 留給人看
uint8 NONE = 0                  # 成功
uint8 PERMANENT = 1             # 這隻對這個點做不到：NAV_FAILED(204) 地圖外、NAV_FAILED(208) 沒路、PTZ_OUT_OF_RANGE → 整趟不再出價
uint8 TRANSIENT = 2             # 卡住、逾時、沒回應等其他失敗 → 加 cost，可再試
uint8 PREEMPTED = 3             # 被平板搶走（PREEMPTED_BY_MANUAL）
uint8 STOPPED = 4               # 載具本身停了：OPERATOR_STOP / EMERGENCY / MISSION_TIMEOUT / SHUTDOWN / POLICY_ABORT / INTERNAL_ERROR
uint8 reason
# 2026-11-14 介面 v2.0（2-1b 的失敗碼，合約第三節的表）：有帶 failure_code 時 cbba_node 依它決定排除，reason 不看；
# 空字串 = 舊的 BT，照舊用 reason。success = true 時這三個不看
#   E_PRECONDITION、E_UNREACHABLE、E_HW → PERMANENT（E_PRECONDITION 另外發 task_orphaned；E_HW 另外拿掉 failed_capability）
#   E_TIMEOUT、E_TARGET_LOST、E_COMM   → TRANSIENT
#   E_LOW_BATTERY、E_SAFETY_STOP       → STOPPED
string failure_code
string failed_capability        # E_HW 時故障的能力標籤（例如 thermal_camera）：這台之後不再對需要它的任務出價，到 cbba_node 重開
string findings_uri             # 執行結果（照片等）的位置；M0 只記 log，之後轉給閘道
