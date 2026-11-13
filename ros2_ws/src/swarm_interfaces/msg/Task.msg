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
# 2026-11-12（協定版本 4，2-1b M0 合約第四節）：舊的 BT 不讀這兩個，重新編譯就能跑
uint16 revision                       # 任務定義的版本（建立者改內容時 + 1）
string spec                           # TaskSpec（JSON：steps 等）。只有 current_task 帶，而且一定是收齊的；
                                      # 沒有 TaskSpec 的任務是空字串。new_task 帶的話一起送給全隊（TASK_SPEC）
# 2026-11-14 介面 v2.0（2-1b M0 合約第四節）：只加欄位，舊的 BT 不讀這些，重新編譯就能跑。
# cbba_node 發的 current_task、task_closed、task_announced、task_orphaned 都填；new_task、cancel_task 不讀這些
# （新型任務由閘道的 inject_task 建立）。舊型任務（type 1～3）填的是對應後的值（AIR_RECON → uav、P3；
# GROUND_INTERVENTION → quadruped、P0；PATROL → quadruped、P3），新型任務 type = 0
uint8 PLATFORM_ANY = 0
uint8 PLATFORM_UAV = 1
uint8 PLATFORM_QUADRUPED = 2
uint8 priority                        # P0～P3，0 最高
uint8 platform                        # PLATFORM_*
string[] required_capabilities        # 能力標籤（註冊表 tools.yaml 的名稱）；不認得的位元是 "bitN"
uint64 group_id                       # 同一個事件拆出的角色組；0 = 單獨任務
uint8 role_index                      # 第幾個角色（0 起）；角色名稱在 spec
uint8 role_count                      # 共幾個角色；0 = 不屬於角色組
bool optional                         # 可略過的角色
uint8 floor                           # 樓層；0 = 不分
bool deadline_hard                    # deadline_sec 是硬期限（來不及就不出價）
builtin_interfaces/Time release       # 可以開始的時刻；0 = 立即
builtin_interfaces/Time expires       # 失效的時刻；0 = 不失效
