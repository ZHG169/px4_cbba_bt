# 角色的執行進度（/<ns>/task_progress，BT → CBBA，reliable、keep_last 10；CB-4 第一批，2026-11-16，
# cbba_core/doc/cb4_design.md）。BT 在階段、步驟改變或達成里程碑時發；cbba_node 轉成 TASK_PROGRESS 封包送全隊
# （同組的其他角色用 teammates 看到）。task_id 要是目前 current_task 上的任務（和 task_result 一樣檢查），否則忽略
uint64 task_id
uint16 revision                  # current_task 的 revision（只記 log；cbba_node 用自己知道的）

uint8 PHASE_UNKNOWN = 0
uint8 PHASE_ACCEPTED = 1
uint8 PHASE_EN_ROUTE = 2
uint8 PHASE_ARRIVED = 3
uint8 PHASE_EXECUTING = 4
uint8 PHASE_BLOCKED = 5          # 停在 wait_for 還沒滿足的 step：保持正式指派，不計入執行逾時
uint8 PHASE_ARRIVED_WAITING = 6  # 第二批（sync_window）
uint8 PHASE_DONE = 7
uint8 phase

uint8 STEP_UNKNOWN = 255
uint8 step_index                 # 正在做 spec 的第幾步（0 起）

# 這次新達成的里程碑（cbba_node 累積，只增不減）：en_route、arrived、started、target_localized、target_locked、
# has_observation。accepted、completed 由 cbba_node 判定，不用回報；不認得的名稱印警告、忽略
string[] milestones
string detail                    # 只記 log
