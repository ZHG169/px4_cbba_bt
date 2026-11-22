# L4 VLM 驗證的結果（2026-11-23；VLM 服務發 /gcs/vlm_result）
string event_id
string verdict                     # confirmed、rejected、uncertain
float32 confidence                 # 0～1（校準後）
string evidence                    # VLM 描述的證據（只記 log）
