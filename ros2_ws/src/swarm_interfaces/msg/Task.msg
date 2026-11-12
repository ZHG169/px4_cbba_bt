# 任務定義（在 /swarm/tasks 上廣播：建立時、狀態改變時各一次）
# QoS：reliable、transient_local、keep_last 100（介面規格 v1.0）
uint8 AIR_RECON = 1
uint8 GROUND_INTERVENTION = 2
uint8 PATROL = 3

uint8 OPEN = 0
uint8 DONE = 1
uint8 CANCELLED = 2

uint64 task_id                        # 協定版本 4（2026-11-11）：64 位元。舊型 = 建立者編號 * 65536 + 流水號（上半 32 位元是 0）；
                                      # 版本 4 = 建立者 16 ｜ 世代 16 ｜ 流水號 32，只由 cbba_node／閘道產生
uint8 type
geometry_msgs/Point position          # map 座標系 (ENU)，公尺
builtin_interfaces/Time created       # 發現或建立的時間
float32 deadline_sec                  # 期限，從 created 起算的秒數（軟限制）
float32 value                         # 任務價值
float32 duration_sec                  # 預估的現場執行時間
uint8 status
builtin_interfaces/Time status_stamp  # 狀態最後一次改變的時間
