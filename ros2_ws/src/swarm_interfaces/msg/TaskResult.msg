# 機器狗的任務結果（/v60/task_result，狗的 BT → CBBA，任務結束時一次）
uint32 task_id
bool success                       # false = 釋放回競標池
string detail                      # NAV2_ABORTED / TIMEOUT / PTZ_HW_ERROR / PREEMPTED_BY_MANUAL
