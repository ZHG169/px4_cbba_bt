# 我這個角色某一步的 gate（Teammates.gates；CB-4 第一批，2026-11-16）
# spec 步驟的 wait_for（前置角色＋里程碑）都滿足才 OPEN；BT 到這一步前看它，不是 OPEN 就停住、回報 PHASE_BLOCKED
uint8 OPEN = 0
uint8 WAITING = 1                # 還沒滿足，前面的角色還開著
uint8 UNSATISFIABLE = 2          # 前面的角色已結束但不會滿足（取消、過期、略過而 skip_satisfies = false）；第三批由閘道處理
uint8 step_index
uint8 state
string[] waiting_for             # 還沒滿足的條件，例如 "role 0: target_localized"、"sync front_rear: role 2"（給 log、畫面）
# 同步（CB-4 第二批，2026-11-16）：這一步是 spec 的 sync 會合點時，同名的角色都到了（ARRIVED_WAITING）才 OPEN；
# 第一個到的開始算，超過任務的 sync_window_s 還沒到齊 → 照樣 OPEN、timed_out = true（已到的先開始，waiting_for 是沒到的）
string sync                      # 會合點的名稱；空 = 這一步不是會合點
bool timed_out
