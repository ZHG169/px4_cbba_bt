# 載具的任務結果（/<ns>/task_result，BT → CBBA，任務結束時一次）
uint32 task_id
bool success                       # true = 完成；false = 執行失敗：任務保留、交回競標池，自己不再接
string detail                      # NAV2_ABORTED / TIMEOUT / PTZ_HW_ERROR / PREEMPTED_BY_MANUAL
uint64 assignment_version          # 收到的 assigned_task 的 assignment_version，原樣帶回；不是目前的正式指派就拒絕
