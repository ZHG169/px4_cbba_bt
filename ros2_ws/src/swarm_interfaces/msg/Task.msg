# 任務定義（在 /swarm/tasks 上廣播：建立時、狀態改變時各一次）
# QoS：reliable、transient_local、keep_last 100（介面規格 v1.0）
uint8 AIR_RECON = 1
uint8 GROUND_INTERVENTION = 2
uint8 PATROL = 3

uint8 OPEN = 0
uint8 DONE = 1
uint8 CANCELLED = 2

uint32 task_id                        # 建立者編號 * 65536 + 流水號
uint8 type
geometry_msgs/Point position          # map 座標系 (ENU)，公尺
builtin_interfaces/Time created       # 發現或建立的時間
float32 deadline_sec                  # 期限，從 created 起算的秒數（軟限制）
float32 value                         # 任務價值
float32 duration_sec                  # 預估的現場執行時間
uint8 status
builtin_interfaces/Time status_stamp  # 狀態最後一次改變的時間
uint64 assignment_version              # 正式指派的版本（cbba_node 發 assigned_task 時填；BT 回報 task_result 時原樣帶回；其他用途填 0）
