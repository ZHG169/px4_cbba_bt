# 核准單的狀態變化（閘道的 /<ns>/approvals，reliable、transient_local、keep_last 50；任務欄位補齊 12，2026-11-18）
# PROPOSED → WAITING_APPROVAL → APPROVED → INJECTED／REJECTED；等核准時過期 → EXPIRED。核准前不建 CBBA 任務
uint8 PROPOSED = 0
uint8 WAITING_APPROVAL = 1
uint8 APPROVED = 2
uint8 INJECTED = 3
uint8 REJECTED = 4                 # 值班拒絕，或核准後注入失敗（note 寫原因）
uint8 EXPIRED = 5

uint32 approval_id
uint8 state
string incident_id
string role
uint16 revision
string[] reasons                   # 為什麼要核准（規劃器要求、事件類型、高風險動作…）
string operator_name
string note
uint64 task_id                     # INJECTED 之後
uint64 group_id
builtin_interfaces/Time stamp
