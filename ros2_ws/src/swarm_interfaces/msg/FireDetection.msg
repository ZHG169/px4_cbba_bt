# AprilTag 火情偵測結果（/<ns>/fire_detection，apriltag_fire_detector → BT，機內）
# 每處理一張影像、看到的每個 tag 各送一則（看不到時不送）。BT 依 confidence 判定火警是否成立
std_msgs/Header header             # stamp = 影像時間；frame_id = "map"
uint32 tag_id                      # tag36h11 的編號
float32 confidence                 # 0～1：最近 window_sec 內，處理過的影像中有偵測到這個 tag 的比例
float32 decision_margin            # 這一張的 decision_margin（apriltag 的解碼品質，越大越可靠）
uint16 frames                      # 計算 confidence 用了幾張影像（太少時 confidence 不可靠）
geometry_msgs/Point position       # tag 中心，map ENU（公尺）：視窗內的平均
float32 distance                   # 相機到 tag 的距離（公尺）
