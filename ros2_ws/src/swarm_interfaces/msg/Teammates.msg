# 同組角色的狀況（/<ns>/teammates，CBBA → BT，reliable、transient_local、depth 1；有變化時＋每 1 s；CB-4 第一批，2026-11-16）
# 我手上的任務（current_task）屬於角色組時才有內容；沒有時 task_id = 0、roles、gates 空
uint64 task_id                   # 我手上的任務
uint64 group_id
uint8 role_index                 # 我是第幾個角色
Teammate[] roles                 # 同組每個角色（含自己），照 role_index
StepGate[] gates                 # 我這個角色每個有 wait_for 的步驟；沒列的步驟不用等
