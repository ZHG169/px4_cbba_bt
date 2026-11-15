# 我這個角色某一步的 gate（Teammates.gates；CB-4 第一批，2026-11-16）
# spec 步驟的 wait_for（前置角色＋里程碑）都滿足才 OPEN；BT 到這一步前看它，不是 OPEN 就停住、回報 PHASE_BLOCKED
uint8 OPEN = 0
uint8 WAITING = 1                # 還沒滿足，前面的角色還開著
uint8 UNSATISFIABLE = 2          # 前面的角色已結束但不會滿足（取消、過期、略過而 skip_satisfies = false）；第三批由閘道處理
uint8 step_index
uint8 state
string[] waiting_for             # 還沒滿足的條件，例如 "role 0: target_localized"（給 log、畫面）
