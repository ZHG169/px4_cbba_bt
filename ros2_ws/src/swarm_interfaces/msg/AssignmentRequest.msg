# CBBA 請 BT 接受一個指派（/<ns>/assignment_request，CBBA → BT）。只有 require_accept = true 時才送
# task.assignment_version 是這次請求的識別碼：BT 回覆時原樣帶回。逾時、拒絕、撤銷之後這個版本就作廢，
# 不會再用；作廢之後才到的接受回覆不會啟動任務。
uint8 ACTIVATE = 1                   # 現在就執行（preempt = true 時要先停止目前的任務、完成交接再回覆接受）
uint8 RESERVE = 2                    # 排在目前任務之後（保留）
uint8 RELEASE = 3                    # 收回：先前的請求或保留作廢（不用回覆）
uint8 kind

bool preempt                         # ACTIVATE 時：會中斷目前的任務
Task task
