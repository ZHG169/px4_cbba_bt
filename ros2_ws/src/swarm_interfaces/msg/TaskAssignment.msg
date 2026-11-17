# 一個任務的分配狀態（TaskAssignments 的一筆；2-1b M0 第 4a 步，2026-11-12）
uint8 NONE = 0                     # assign_state：沒有正式指派（或已撤銷）
uint8 PENDING = 1
uint8 RESERVED = 2
uint8 ACTIVE = 3

uint64 task_id
uint16 revision
uint64 group_id                    # 0 = 單獨任務
uint8 role_index
uint8 priority                     # P0～P3
uint16 winner_id                   # CBBA 目前的得標者；0 = 沒有
float32 winner_bid                 # 得標價（同級分數）
uint64 assign_version              # 正式指派：版本、執行者（0 = 沒有）、狀態
uint16 assignee_id
uint8 assign_state
bool spec_ready                    # 這個節點有收齊的 TaskSpec（或任務不需要）
# 任務語意（任務欄位補齊 02，2026-11-17）：只有閘道填（它記得注入的內容）；機器人發的是空的、0
string incident_id
uint8 source                       # InjectTask 的 SOURCE_*
string event_type
uint16 event_type_id
string task_type
uint16 task_type_id
string role
uint16 role_id
