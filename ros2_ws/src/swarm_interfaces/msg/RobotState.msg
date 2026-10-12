# 載具即時狀態（/<ns>/robot_state，reliable、keep_last 1，2 Hz）
# BT 端發布，CBBA node 訂閱後覆寫 AgentState 的 position / battery。
# 不變的參數（home、cruise_speed、energy_per_meter、safety_reserve…）走 CBBA node 的 params yaml，不在這裡。
std_msgs/Header header          # stamp 為採樣時間；frame_id = "map"
uint8 agent_id                  # 與 CBBAMessage.sender_id 相同（V60 = 50）
geometry_msgs/Point position    # map 座標系 (ENU)，公尺，與 Task.position 同座標系
float32 battery                 # 0~100 (%)
# v1.2：載具能不能接 CBBA 任務。CBBA 的可行性檢查看到 MANUAL / ESTOP → 對所有任務出價 0（約束，不是成本）
uint8 AUTO = 0                  # 自主巡檢或閒置，可接任務
uint8 MANUAL = 1                # 平板接管中（/<ns>/manual_override），不接 CBBA 任務
uint8 ESTOP = 2                 # 急停中
uint8 mode
