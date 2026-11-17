# 組層級的 incident state（CB-4 第三批，2026-11-16）
# 閘道：/<ns>/incident_updates，每次接受一個更新發一則（給外部 Planner：依新位置重新展開範本，對 TaskSpec 改了的角色 inject_task 改版）
# reliable、keep_last 50
uint64 group_id
string incident_id                # 閘道才填（規劃器給的事件識別）
uint16 incident_revision          # 接受後的 revision
bool position_valid
geometry_msgs/Point position
uint8 floor
float32 radius
uint32 track_id
uint64 localized_by               # 宣告了定位而且通過驗證的角色任務；0 = 沒有
uint16 proposer                   # 提案的機器人
builtin_interfaces/Time stamp
