# 事件的證據紀錄（IncidentRecord；閘道的 /<ns>/incident_evidence，reliable、transient_local、keep_last 100；
# 任務欄位補齊 03，2026-11-18）。規劃器在 InjectTask 帶進來，閘道只記參照、不複製影像；TaskSpec 只帶 evidence_uri 與信心值，
# CBBA 封包不帶。內容變了（第一次、或規劃器改版時換了）才發；同時附加在 <state_dir>/gateway_<id>_evidence.log。
# URI：evidence://<incident_id>/<種類>/<檔名>（控制中心的證據庫）或 https://、http://、s3://、file://；保存期限、權限、刪除由控制中心決定
string incident_id
string[] camera_ids
float32 confidence
string evidence_uri                # 給 TaskSpec 的總參照
string snapshot_uri
string clip_uri
string vlm_result_uri
builtin_interfaces/Time stamp
