# 範本展開的一個角色（L6 任務生成，2026-11-23；swarm_events 的 RuleExpander、/gcs/expand_incident 的回應）。
# 欄位對應 InjectTask.srv：呼叫的人照這些欄位呼叫 /gcs/inject_task（展開服務本身不注入）
string role                        # task_types.yaml 的 roles
uint8 role_index
uint8 role_count
string platform                    # uav、quadruped、any
uint8 platform_code                # InjectTask 的 platform（0 ANY、1 uav、2 quadruped）
string[] required_capabilities
uint8 priority                     # P0～P3
geometry_msgs/Point position       # 觀察點（CBBA 的任務位置，map ENU）
string location_source             # incident、poi:<id>、asset:<id>、zone:<id>、dock:<id>；preflight 移過時加「-> anchor (x, y) out of <區域>」
bool optional
bool skip_satisfies
bool all_or_nothing
uint8 precedence                   # 要等的角色（位元 = role_index）
uint8 distinct_agent_set
uint16 sync_window_s
string fallback_template
bool requires_approval
float32 expires_s                  # 0 = 不失效
float32 value
float32 service_time_s
bool retarget                      # false：事件位置更新時不跟著改這個角色
string spec                        # 角色 TaskSpec（JSON）
