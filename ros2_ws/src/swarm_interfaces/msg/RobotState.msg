# 載具的狀態（/<ns>/robot_state → CBBA）。狗的 BT 2 Hz；無人機由 px4_state_bridge 從 PX4 轉換
std_msgs/Header header             # stamp 採樣時間；frame_id = "map"
uint8 agent_id                     # 機號（V60 = 50）
geometry_msgs/Point position       # map ENU，公尺
float32 battery                    # 0~100 %；負值 = 不知道
# 飛行狀態（2026-10-09 加）：只有 flight_state_valid = true 時其他三個才有意義。機器狗一律填 false
bool flight_state_valid            # 有收到飛控的狀態，而且還在更新
bool armed                         # 已解鎖
bool offboard                      # 在 offboard 模式
bool landed                        # 在地面上
