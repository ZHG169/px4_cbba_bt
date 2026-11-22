# L4 VLM 驗證的請求（2026-11-23；event_manager 發 /gcs/vlm_request）：VLM 服務看事件的證據，回 /gcs/vlm_result
string event_id
string event_type
string prompt_id                   # event_types.yaml 的 vlm.prompt_id
uint8 samples                      # 取幾張
float32 timeout_s                  # 超過就退化（P0、P1 用 L2 的信心決定；P2、P3 再送一次）
geometry_msgs/Point position
string[] sources
