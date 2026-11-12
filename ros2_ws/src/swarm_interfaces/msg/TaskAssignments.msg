# 全隊開著的任務的分配狀態（/<ns>/assignments；2-1b M0 第 4a 步，2026-11-12）
# 閘道的 cbba_node 發（publish_assignments，預設只有 participate: never）：取代「GCS 直接聽 UDP 14600」。
# 內容是發布者本機看到的表（經 CBBA_STATE gossip，弱網時可能落後幾百 ms）；已結束的任務不列（看 task_closed）。
# 變動時發（最快每 assignments_period 一次），reliable、transient_local、depth 1
builtin_interfaces/Time stamp
uint16 agent_id                    # 發布者的機號
TaskAssignment[] tasks
