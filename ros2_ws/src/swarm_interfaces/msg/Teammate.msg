# 同組（group_id）的一個角色（Teammates.roles；CB-4 第一批，2026-11-16）
uint64 task_id
uint8 role_index
bool optional

uint8 STATUS_OPEN = 0            # 還沒有正式指派
uint8 STATUS_ASSIGNED = 1        # 有正式指派（保留、執行中）
uint8 STATUS_DONE = 2
uint8 STATUS_SKIPPED = 3         # optional 被略過（TASK_CLOSE reason 4）
uint8 STATUS_CANCELLED = 4
uint8 STATUS_EXPIRED = 5
uint8 status

uint16 holder                    # 正式指派的執行者；0 = 沒有
uint8 phase                      # TaskProgress 的 PHASE_*
uint8 step_index                 # 255 = 不知道
string[] milestones              # gate 用的里程碑（目前的指派版本、revision；完成 = 全部）
bool position_valid
geometry_msgs/Point position     # 執行者的位置（AGENT_STATE），map ENU
builtin_interfaces/Time updated  # 最後一次進度變化
