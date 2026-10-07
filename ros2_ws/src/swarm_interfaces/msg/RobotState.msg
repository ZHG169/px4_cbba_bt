# 機器狗的狀態（/v60/robot_state，狗的 BT → CBBA，2 Hz）
std_msgs/Header header             # stamp 採樣時間；frame_id = "map"
uint8 agent_id                     # V60 = 50
geometry_msgs/Point position       # map ENU，公尺
float32 battery                    # 0~100 %
