# 任務的執行結果（閘道的 /<ns>/task_reports，reliable、transient_local、keep_last 100；任務欄位補齊 11，2026-11-18）
# 執行者的 task_result 經 TASK_REPORT 送到閘道（閘道斷線時暫存在執行者，恢復後補送）。GCS／VMS 從這裡拿 findings 與附件；
# channels 是 TaskSpec report_policy 的回寫頻道（gcs、vms、operator），required_findings 有寫但成功時沒有任何 findings、附件 →
# missing_findings。閘道同時附加在 <state_dir>/gateway_<id>_reports.log
uint64 task_id
uint16 revision
uint64 assign_version
uint16 agent_id                    # 執行者
string incident_id                 # 閘道記得的（不是閘道注入的任務是空的）
bool success
string failure_code
string detail
uint16 completed_steps
string findings_uri
string[] attachments
bool final_position_valid
geometry_msgs/Point final_position
bool buffered                      # 閘道斷線時產生、之後才送到
string[] channels                  # report_policy.channels
bool missing_findings              # report_policy.required_findings 有寫，但成功卻沒有 findings、附件
builtin_interfaces/Time reported   # 執行者回報的時刻
builtin_interfaces/Time received   # 閘道收到的時刻
