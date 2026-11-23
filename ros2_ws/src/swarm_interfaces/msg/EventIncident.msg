# 事件（L5 事件狀態機的輸出，2026-11-23；swarm_events 的 event_manager 發 /gcs/incidents，內容變了才發）。
# 名稱和 CB-4 的 IncidentState（角色組的 incident 位置、revision）不同：這裡是「事件」本身的狀態與派遣
string event_id
string event_type
string state                       # CANDIDATE、VERIFYING、CONFIRMED、NEEDS_CHECK、LOGGED、REJECTED、DISPATCHED、ON_SITE、
                                   # EXECUTING、RESOLVED、FALSE_ALARM、ESCALATED、MERGED、EXPIRED（EXECUTING、MERGED：2.12.0）
string reason                      # 最近一次轉換的原因
string rule                        # 派的範本（event_rules.yaml）
uint16 incident_revision           # 閘道接受的 incident revision
uint16 plan_version                # 展開、注入過幾次
float32 confidence                 # 融合信心（決定派遣用的）
float32 evidence_score             # L2 的 log-odds 分數
geometry_msgs/Point position
float32 radius_m
string zone
uint32 track_id
string[] sources                   # 偵測器:來源
uint32 detections
string parent                      # 降級的子事件：原本的事件；MERGED（告警風暴）：併進的區域巡查
string[] children                  # 降級派出的子事件；區域巡查：併進來的事件
string[] roles                     # 角色名稱
string[] role_states               # 和 roles 對應：PLANNED、ASSIGNED、ACTIVE、ARRIVED、EXECUTING、DONE、FAILED、CANCELLED、SKIPPED
uint64[] task_ids                  # 和 roles 對應；0 = 還沒注入（等核准）
builtin_interfaces/Time stamp
