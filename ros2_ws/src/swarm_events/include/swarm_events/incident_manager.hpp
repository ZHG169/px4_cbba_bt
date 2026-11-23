// 事件管理（L2 時序累積、L3 關聯、L5 事件狀態機；2026-11-23，docs/l5_l6_plan.md B 批）
//
// 偵測（Detection）進來 → 依偵測器與 label 找事件類型（event_types.yaml）→ 觸發條件（zone_attr、time_rule）→
// L3 關聯：同類型、還沒結案、dt_s 內且 r_m 內（或同一個 track_id）的事件 → 併進去，否則開新的候選 →
// L2 累積（照 2-1 原文用 log-odds）：每一筆 s ← λ·s + log(p/(1−p))；沒有新偵測超過 1 s 時每秒乘 λ 遺忘 →
// s ≥ theta_on 持續 min_duration_s → 驗證（有 vlm 且啟用：VLM_REQUEST，逾時依優先級退化）→ 決定：
//   信心 ≥ theta_high（dispatch.theta_high，預設 0.8）→ CONFIRMED → DISPATCH 預設範本
//   θ_low（cost：false/(miss+false)）≤ 信心 < theta_high → NEEDS_CHECK → DISPATCH 範本的 fallback_template（派 1 隻確認）
//   信心 < θ_low → LOGGED（只記錄；之後證據變強會再決定）
// 候選期間 s < theta_off → 取消（REJECTED，原因「證據消退」）。
// 派遣之後（C 批，2026-11-23 使用者決定 1）：依角色的有效進度聚合（閘道的 assignments 帶進度，節點只算目前正式指派版本、
//   目前 revision 的）：DISPATCHED → 有角色到場 ON_SITE → 有必要角色開始第一個非移動的步驟 EXECUTING → 必要角色都完成 RESOLVED；
//   換人、改版後舊進度不算，事件狀態跟著退回（例如 EXECUTING → DISPATCHED）。確認角色回報 E_TARGET_LOST 或值班判定 → FALSE_ALARM；
// 閘道 P0 沒人接、整組失效、fallback_request → ESCALATED（有 fallback 範本就派子事件）；expires_s 到了 → EXPIRED；
// 值班拒絕核准（閘道的 approvals）→ 節點當成值班判定誤報（resolve）。
// 閘道接受的 incident 更新（UAV 定位、位置）、已派遣事件的新偵測移動超過 retarget_m → REPLAN（重新展開，只注入改了的角色）。
// 告警風暴（2-1 案例 9，C 批）：同一區已有 storm_max_per_zone 件派遣中的事件時，新決定要派遣的事件不各自派，
//   改成合併進那一區的區域巡查（storm_rule，一個新的事件）→ MERGED，並通知值班；巡查結案時合併的事件一起結案。
//
// 這個類別不依賴 ROS、不注入：每個 on*／step 回傳要做的事（Action），由 event_manager 節點執行
// （DISPATCH／REPLAN：expand → inject_task；VLM_REQUEST：發請求；NOTIFY：通知值班），結果再用 onDispatchResult 告訴它。
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "swarm_cbba/registry.hpp"
#include "swarm_events/event_rules.hpp"

namespace swarm_events
{

enum class EventState : std::uint8_t
{
  CANDIDATE, VERIFYING, CONFIRMED, NEEDS_CHECK, LOGGED, REJECTED, DISPATCHED, ON_SITE, EXECUTING, RESOLVED,
  FALSE_ALARM, ESCALATED, MERGED, EXPIRED
};
const char * stateName(EventState state);
std::optional<EventState> stateFromName(const std::string & name);
bool isTerminal(EventState state);   // REJECTED、RESOLVED、FALSE_ALARM、EXPIRED
bool isDispatched(EventState state);  // DISPATCHED、ON_SITE、EXECUTING（有角色在處理）

// ARRIVED：有效進度有 arrived；EXECUTING：有效進度有 started（開始第一個非移動的步驟）
enum class RoleState : std::uint8_t
{
  PLANNED, ASSIGNED, ACTIVE, ARRIVED, EXECUTING, DONE, FAILED, CANCELLED, SKIPPED
};
const char * roleStateName(RoleState state);
std::optional<RoleState> roleStateFromName(const std::string & name);

// 一筆偵測（swarm_interfaces/Detection 裡事件流程要的部分）
struct DetectionIn
{
  std::uint16_t agent_id{0};
  std::uint32_t seq{0};
  std::string detector;
  std::string label;
  std::string source;          // kind:id
  double score{0.0};
  std::uint32_t track_id{0};   // 0 = 沒有
  bool position_valid{false};
  double x{0.0}, y{0.0}, z{0.0};
  // 證據（偵測的 attrs 帶的參照，C 批）：注入時帶給閘道（InjectTask 的 evidence_uri、snapshot_uri、clip_uri）
  std::string snapshot_uri;
  std::string clip_uri;
  std::string evidence_uri;
};

// 事件的證據：分數最高、有參照的那筆偵測＋各來源的相機
struct Evidence
{
  std::string snapshot_uri;
  std::string clip_uri;
  std::string evidence_uri;     // 空 = 用 snapshot_uri
  double score{0.0};
  std::set<std::string> cameras;   // 偵測的 source
};

struct RoleStatus
{
  bool optional{false};
  RoleState state{RoleState::PLANNED};
  std::string failure_code;
};

struct EventRecord
{
  std::string event_id;
  std::string event_type;
  EventState state{EventState::CANDIDATE};
  std::uint16_t incident_revision{0};   // 閘道接受的 incident revision（UAV 的更新）
  std::uint16_t plan_version{0};        // 這個事件展開、注入過幾次（每次 REPLAN + 1）
  double x{0.0}, y{0.0}, z{0.0};
  double radius_m{0.0};
  std::string zone;
  std::uint32_t track_id{0};
  std::set<std::string> sources;        // 偵測器:來源
  std::size_t detections{0};
  double first_t{0.0};
  double last_t{0.0};                   // 最後一筆偵測
  double s{0.0};                        // L2 的 log-odds 分數
  double s_t{0.0};                      // s 上次更新（遺忘）的時刻
  std::optional<double> above_since;    // s 第一次 ≥ theta_on 的時刻（之後掉下去就清掉）
  double confidence{0.0};               // 融合信心（決定派遣用的）
  bool vlm_pending{false};
  double vlm_deadline{0.0};
  int vlm_retries{0};
  double decided_t{0.0};                // 決定（CONFIRMED／NEEDS_CHECK／LOGGED）的時刻，過期從這裡算
  std::string rule;                     // 派的規則
  double dispatched_x{0.0}, dispatched_y{0.0};   // 上次展開時的位置
  std::map<std::string, RoleStatus> roles;
  std::string parent;                   // fallback 派出的子事件：原本的事件
  std::vector<std::string> children;
  bool storm{false};                    // 告警風暴合併出來的區域巡查
  std::string merged_into;              // MERGED：併進哪一個區域巡查
  std::vector<std::string> merged;      // 區域巡查：併進來的事件
  Evidence evidence;
  std::string reason;                   // 最近一次轉換的原因
};

struct Action
{
  enum class Kind : std::uint8_t {DISPATCH, REPLAN, VLM_REQUEST, NOTIFY};
  Kind kind{Kind::NOTIFY};
  std::string event_id;
  std::string rule;                     // DISPATCH、REPLAN：要展開的規則
  std::uint16_t incident_revision{0};
  std::uint16_t plan_version{0};
  std::string text;                     // NOTIFY：給值班的說明；VLM_REQUEST：prompt_id
};
const char * actionName(Action::Kind kind);

struct ManagerConfig
{
  double retarget_m{3.0};               // 已派遣事件的位置變動超過這個才重新展開（和閘道一致）
  bool vlm_enabled{false};              // 沒有 VLM 時 false：有 vlm 的事件類型直接用 L2 的信心決定
  int on_hours_start{8};                // time_rule 的「上班時間」[start, end)（當地時間的小時）
  int on_hours_end{18};
  std::function<int(double)> hour_of;   // 時刻 → 當地小時；空 = localtime
  std::string id_prefix{"ev"};          // 事件編號的前綴（節點用啟動時刻區分重開前後）
  // 告警風暴（2-1 案例 9）：同一區（zoneAt 的最小區域）同時派遣中的事件上限；0 = 不合併
  int storm_max_per_zone{3};
  std::string storm_rule{"zone_patrol"};   // 合併成的區域巡查範本（event_rules.yaml）
};

class IncidentManager
{
public:
  IncidentManager(const swarm_cbba::Registry & registry, const swarm_cbba::Site & site, const EventRules & rules,
    ManagerConfig config = ManagerConfig{});

  std::vector<Action> onDetection(const DetectionIn & det, double now);
  // VLM 的判讀：verdict confirmed／rejected／uncertain，confidence 0～1
  std::vector<Action> onVlmResult(const std::string & event_id, const std::string & verdict, double confidence,
    double now);
  // 節點執行 DISPATCH／REPLAN 的結果：ok 時 roles 是（角色, optional）
  std::vector<Action> onDispatchResult(const std::string & event_id, bool ok,
    const std::vector<std::pair<std::string, bool>> & roles, const std::string & reason, double now);
  // 閘道接受的 incident 更新（/gcs/incident_updates）：revision 變大就重新展開
  std::vector<Action> onIncidentUpdate(const std::string & event_id, std::uint16_t revision, bool position_valid,
    double x, double y, double now);
  // 角色的狀態（閘道的 assignments〔含有效進度〕、task_closed、task_reports）；FAILED 帶 failure_code。
  // 事件的 DISPATCHED／ON_SITE／EXECUTING 由全部角色重新聚合（可以退回：換人、改版後舊進度不算）
  std::vector<Action> onRoleState(const std::string & event_id, const std::string & role, RoleState state,
    const std::string & failure_code, double now);
  // 升級：閘道的 P0 看門狗、整組失效、fallback_request。fallback_rule 空 = 用派遣規則的 fallback_template
  std::vector<Action> onEscalate(const std::string & event_id, const std::string & reason,
    const std::string & fallback_rule, double now);
  // 值班結案（false_alarm = 標誤報）
  std::vector<Action> resolve(const std::string & event_id, bool false_alarm, const std::string & who, double now);
  // 計時：遺忘、min_duration、VLM 逾時、過期
  std::vector<Action> step(double now);

  const std::map<std::string, EventRecord> & events() const {return events_;}
  const EventRecord * find(const std::string & event_id) const;
  // 狀態轉換的紀錄（每次一行：「時刻 事件 舊狀態 → 新狀態 原因」），拿走後清空
  std::vector<std::string> takeLog();

  // 存檔與讀回（重開沿用；只存沒結案的事件與編號計數）
  std::string save() const;
  void load(const std::string & yaml);

  // 這個事件類型的派遣門檻
  double thetaLow(const swarm_cbba::EventType & et) const;

private:
  const swarm_cbba::EventType * typeOf(const EventRecord & rec) const;
  bool triggered(const swarm_cbba::EventType & et, const DetectionIn & det, double now) const;
  void setState(EventRecord & rec, EventState state, const std::string & reason, double now);
  void decay(EventRecord & rec, const swarm_cbba::EventType & et, double now);
  // s ≥ theta_on 持續 min_duration 了沒；到了就往下驗證或決定
  void advance(EventRecord & rec, const swarm_cbba::EventType & et, double now, std::vector<Action> & out);
  void decide(EventRecord & rec, const swarm_cbba::EventType & et, double p, double now, std::vector<Action> & out);
  void dispatch(EventRecord & rec, const std::string & rule, double now, std::vector<Action> & out);
  void finish(EventRecord & rec, EventState state, const std::string & reason, double now, std::vector<Action> & out);
  // 角色狀態 → 事件的 DISPATCHED／ON_SITE／EXECUTING
  void aggregate(EventRecord & rec, const std::string & role, double now);
  // 告警風暴：要派遣的事件改併進區域巡查；true = 已處理（不要各自派）
  bool absorbIntoStorm(EventRecord & rec, double now, std::vector<Action> & out);
  std::string newId() {return config_.id_prefix + "_" + std::to_string(next_id_++);}

  const swarm_cbba::Registry & registry_;
  const swarm_cbba::Site & site_;
  const EventRules & rules_;
  ManagerConfig config_;
  std::map<std::string, EventRecord> events_;
  std::uint64_t next_id_{1};
  std::vector<std::string> log_;
};

}  // namespace swarm_events
