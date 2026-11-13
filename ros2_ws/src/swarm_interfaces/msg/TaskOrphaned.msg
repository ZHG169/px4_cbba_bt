# 孤兒任務通知（/<ns>/task_orphaned；2026-10-17 起是 Task，2026-11-14 介面 v2.0 改成這個，加原因）
# 只通知（給地面站、值班），cbba_node 不另外處理。每個任務每種原因最多一次。reliable、keep_last 20
uint8 NO_WINNER = 1             # 有載具能做（沒有能做的載具時報 NO_CAPABLE、不報這個），但 orphan_warn_sec（30 s）內沒有人得標、也沒有正式指派
uint8 NO_CAPABLE = 2            # 全隊（聽得到的載具）沒有一台的平台、能力符合；持續 orphan_no_capable_sec（3 s）才報
uint8 EXPIRED = 3               # expires 到了還沒完成：收到建立者的 TASK_CLOSE reason 3 時發（建立者自己也是）
uint8 PRECONDITION = 4          # 執行端回報 E_PRECONDITION：規劃錯誤，要值班處理
uint8 OPTIONAL_NO_CAPABLE = 5   # optional 角色略過 O1（閘道）
uint8 OPTIONAL_UNASSIGNED = 6   # optional 角色略過 O2（閘道）
uint8 OPTIONAL_GROUP_DONE = 7   # optional 角色略過 O3：同組必要角色都結束了（閘道）

Task task
uint8 reason
float32 waited_sec              # 條件成立了多久（NO_WINNER、NO_CAPABLE、O1、O2）；其他 0
string detail                   # 給人看的說明
