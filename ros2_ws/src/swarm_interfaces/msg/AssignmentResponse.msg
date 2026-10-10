# BT 對指派請求的回覆（/<ns>/assignment_response，BT → CBBA）
uint32 task_id
uint64 assignment_version            # 請求裡的版本，原樣帶回；不是目前在等的那一個就忽略

bool accepted

uint8 REASON_NONE = 0
uint8 REASON_TEMPORARY = 1           # 暫時不能（不可中斷階段等）：狀態改變後 CBBA 會重新評估
uint8 REASON_PERMANENT = 2           # 不能做（能力、故障、不可達）：CBBA 不再對它出價
uint8 reason
string detail                        # 只記 log
