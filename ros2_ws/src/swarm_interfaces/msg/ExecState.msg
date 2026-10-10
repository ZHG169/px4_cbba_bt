# 載具 BT 的執行狀態（/<ns>/exec_state，BT → CBBA，約 2 Hz）
# CBBA 依它決定能不能出價、能不能中斷目前的任務、能不能排隊。逾時判斷用 CBBA 本機收到的時間；
# header.stamp 是資料時間，留作量測。
std_msgs/Header header

uint8 IDLE = 0                       # 閒置
uint8 NAVIGATING = 1                 # 前往任務
uint8 EXECUTING = 2                  # 處置中
uint8 NON_INTERRUPTIBLE = 3          # 不可中斷階段（例如操作閥門、機械臂）
uint8 STOPPING = 4                   # 停止、控制交接中
uint8 FAULT = 5                      # 故障：不能接任務
uint8 execution_state

bool preemptible                     # 現在可以安全中斷目前的任務
float32 estimated_remaining_time     # 目前任務的預估剩餘時間（秒）；負值 = 不知道（不知道時不接受排隊）

bool has_active                      # 有沒有執行中的任務（不用 task_id = 0 表示沒有）
uint32 active_task_id
uint64 active_assignment_version

bool has_queued                      # 有沒有已接受、排隊中的任務
uint32 queued_task_id
uint64 queued_assignment_version
