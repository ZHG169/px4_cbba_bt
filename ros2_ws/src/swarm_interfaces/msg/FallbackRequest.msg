# 降級請求（/<ns>/fallback_request，閘道 → 外部 task manager，reliable、transient_local、keep_last 20；CB-4 第三批，2026-11-16）
# 角色組的必要角色永久不可恢復、閘道取消了角色（all_or_nothing 整組；否則失效的角色＋依賴它的）而且組有 fallback_template 時發。
# 冪等：(group_id, failure_version) 相同的是同一個請求（task manager 收到重複的要忽略）；task manager 展開 fallback_template、
# 用新的 incident_id／角色注入新的角色組（閘道不自己展開）
uint64 group_id
string incident_id
uint32 failure_version            # group_failure_version：這個組第幾次失效
string fallback_template          # 規劃器注入時帶的（InjectTask.fallback_template）

# 確定終止（立即）：合法取消、過期；其他都是暫時不完整，等 group_recovery_timeout_s（10 s）還沒恢復才以 RECOVERY_TIMEOUT 收尾
uint8 CANCELLED = 1               # 必要角色被取消
uint8 EXPIRED = 2                 # 必要角色（任務／事件）過期
uint8 RECOVERY_TIMEOUT = 4        # 暫時不完整等不到恢復（3 保留，未使用）
uint8 reason
string reason_text

uint8[] failed_roles              # 這次失效的必要角色（role_index）
# 和 failed_roles 對應：RECOVERY_TIMEOUT 的最初觸發原因（恢復前不改）；取消、過期是 TRIGGER_NONE
uint8 TRIGGER_NONE = 0
uint8 TRIGGER_NO_CAPABLE = 1      # 目前沒有存活的、平台與能力符合的載具
uint8 TRIGGER_NO_WINNER = 2       # 沒有得標者也沒有正式指派（撤銷、重新競標中）
uint8 TRIGGER_HOLDER_LOST = 3     # 執行者失聯
uint8[] recovery_triggers
uint8[] cancelled_roles           # 閘道因此取消的角色
bool all_or_nothing
uint16 incident_revision
bool position_valid
geometry_msgs/Point position      # 目前的 incident 位置
builtin_interfaces/Time stamp
