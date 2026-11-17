# 執行中的角色回報 incident 的位置與追蹤更新（/<ns>/incident_update，BT → CBBA，reliable、keep_last 10；CB-4 第三批，2026-11-16，
# cbba_core/doc/cb4_design.md）。cbba_node 送 INCIDENT_UPDATE 的 Proposal 給建立這個組的閘道；閘道驗證（是不是這個角色目前的
# 正式執行者、base incident revision 是不是最新）後廣播 Accepted（incident revision + 1）或 Rejected。
# task_id 要是目前 current_task 上、屬於角色組的任務，否則忽略
uint64 task_id
bool position_valid
geometry_msgs/Point position      # map ENU
uint8 floor                       # 0 = 不分
float32 radius                    # 不確定半徑（m）
uint32 track_id                   # 追蹤編號；0 = 沒有
bool declares_localized           # 明確宣告「已定位目標」：通過閘道驗證才在新的 incident revision 記這個角色的 target_localized
string detail                     # 只記 log
