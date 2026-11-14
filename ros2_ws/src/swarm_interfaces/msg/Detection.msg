# 偵測（2-1b 第七節的 Detection；2-1b M0 第 5 步，2026-11-14）。EVT-1 之後只加欄位
# 機器人：偵測器（或 BT）發 /<ns>/detection → cbba_node 送閘道（DETECTION 封包，閘道斷線時暫存、恢復後送出）
# 閘道：cbba_node 發 /<ns>/detections（收到的新偵測，每筆一次；同一個 track、同一個位置的合併、信心融合由事件流程做）
# reliable、keep_last 100
builtin_interfaces/Time t         # 偵測的時刻（系統時鐘，各機對時）
string detector                   # 偵測器（event_types.yaml 的名稱，例如 apriltag_fire）；≤ 64 B
string source                     # 感測器（例如 uav1/down_cam）；≤ 64 B
string label                      # 類別（fire、person…）；≤ 64 B
float32 score                     # 0～1
float32[4] bbox                   # 影像上的 x、y、w、h（px）；全 0 = 沒有
uint32 track_id                   # 偵測器的追蹤編號；0 = 沒有（沒有時用位置合併）
bool position_valid
geometry_msgs/Point position      # map ENU
uint8 floor                       # 0 = 不分
string attrs                      # 其他屬性（JSON）；cbba_node 不解析；≤ 900 B
# 下面由閘道的 cbba_node 填（機器人發 detection 時不用填）
uint16 agent_id                   # 偵測的機器人
uint32 detection_seq              # 那台機器人這次啟動的偵測流水號（1 起）
bool buffered                     # 閘道斷線時暫存、恢復後才送到的
builtin_interfaces/Time received  # 閘道收到的時刻
