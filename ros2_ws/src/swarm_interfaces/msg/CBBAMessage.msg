# CBBA 出價訊息（/swarm/cbba，best effort、volatile、keep_last 10；改變時立即送，另每 200 ms 重送）
# sender_id：無人機 N = N，V60 = 50，0 保留為「無人」（介面規格 v1.0）
uint8 UAV = 1
uint8 UGV = 2

std_msgs/Header header   # stamp 為送出時間
uint8 sender_id
uint8 sender_type
uint32 seq               # 傳送者自己的訊息流水號
Bid[] bids               # 每個 OPEN 任務一筆；無人得標時 score 為 0、winner_id 為 0
AgentStamp[] stamps      # 傳送者以外、每台已知的載具一筆
