#include "swarm_events/incident_manager.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <sstream>

#include <yaml-cpp/yaml.h>

#include "swarm_events/expander.hpp"

namespace swarm_events
{

namespace
{

constexpr double kForgetAfter = 1.0;   // 沒有新偵測超過這麼久（s）才開始遺忘

double logit(double p)
{
  p = std::clamp(p, 0.01, 0.99);
  return std::log(p / (1.0 - p));
}

double sigmoid(double s)
{
  return 1.0 / (1.0 + std::exp(-s));
}

std::string fmt(double v, int digits = 2)
{
  std::ostringstream os;
  os.setf(std::ios::fixed);
  os.precision(digits);
  os << v;
  return os.str();
}

Action action(Action::Kind kind, const EventRecord & rec, const std::string & rule = "", const std::string & text = "")
{
  Action a;
  a.kind = kind;
  a.event_id = rec.event_id;
  a.rule = rule;
  a.incident_revision = rec.incident_revision;
  a.plan_version = rec.plan_version;
  a.text = text;
  return a;
}

}  // namespace

const char * stateName(EventState state)
{
  switch (state) {
    case EventState::CANDIDATE: return "CANDIDATE";
    case EventState::VERIFYING: return "VERIFYING";
    case EventState::CONFIRMED: return "CONFIRMED";
    case EventState::NEEDS_CHECK: return "NEEDS_CHECK";
    case EventState::LOGGED: return "LOGGED";
    case EventState::REJECTED: return "REJECTED";
    case EventState::DISPATCHED: return "DISPATCHED";
    case EventState::ON_SITE: return "ON_SITE";
    case EventState::EXECUTING: return "EXECUTING";
    case EventState::RESOLVED: return "RESOLVED";
    case EventState::FALSE_ALARM: return "FALSE_ALARM";
    case EventState::ESCALATED: return "ESCALATED";
    case EventState::MERGED: return "MERGED";
    case EventState::EXPIRED: return "EXPIRED";
  }
  return "?";
}

std::optional<EventState> stateFromName(const std::string & name)
{
  for (int i = 0; i <= static_cast<int>(EventState::EXPIRED); ++i) {
    const auto s = static_cast<EventState>(i);
    if (name == stateName(s)) {
      return s;
    }
  }
  return std::nullopt;
}

bool isTerminal(EventState state)
{
  return state == EventState::REJECTED || state == EventState::RESOLVED || state == EventState::FALSE_ALARM ||
         state == EventState::EXPIRED;
}

bool isDispatched(EventState state)
{
  return state == EventState::DISPATCHED || state == EventState::ON_SITE || state == EventState::EXECUTING;
}

const char * roleStateName(RoleState state)
{
  switch (state) {
    case RoleState::PLANNED: return "PLANNED";
    case RoleState::ASSIGNED: return "ASSIGNED";
    case RoleState::ACTIVE: return "ACTIVE";
    case RoleState::ARRIVED: return "ARRIVED";
    case RoleState::EXECUTING: return "EXECUTING";
    case RoleState::DONE: return "DONE";
    case RoleState::FAILED: return "FAILED";
    case RoleState::CANCELLED: return "CANCELLED";
    case RoleState::SKIPPED: return "SKIPPED";
  }
  return "?";
}

std::optional<RoleState> roleStateFromName(const std::string & name)
{
  for (int i = 0; i <= static_cast<int>(RoleState::SKIPPED); ++i) {
    const auto s = static_cast<RoleState>(i);
    if (name == roleStateName(s)) {
      return s;
    }
  }
  return std::nullopt;
}

const char * actionName(Action::Kind kind)
{
  switch (kind) {
    case Action::Kind::DISPATCH: return "DISPATCH";
    case Action::Kind::REPLAN: return "REPLAN";
    case Action::Kind::VLM_REQUEST: return "VLM_REQUEST";
    case Action::Kind::NOTIFY: return "NOTIFY";
  }
  return "?";
}

IncidentManager::IncidentManager(const swarm_cbba::Registry & registry, const swarm_cbba::Site & site,
  const EventRules & rules, ManagerConfig config)
: registry_(registry), site_(site), rules_(rules), config_(std::move(config))
{
  if (!config_.hour_of) {
    config_.hour_of = [](double t) {
        const std::time_t tt = static_cast<std::time_t>(t);
        std::tm local{};
        localtime_r(&tt, &local);
        return local.tm_hour;
      };
  }
}

const EventRecord * IncidentManager::find(const std::string & event_id) const
{
  const auto it = events_.find(event_id);
  return it == events_.end() ? nullptr : &it->second;
}

std::vector<std::string> IncidentManager::takeLog()
{
  std::vector<std::string> out;
  out.swap(log_);
  return out;
}

const swarm_cbba::EventType * IncidentManager::typeOf(const EventRecord & rec) const
{
  return registry_.eventType(rec.event_type);
}

double IncidentManager::thetaLow(const swarm_cbba::EventType & et) const
{
  return et.has_cost ? et.cost_false / (et.cost_false + et.cost_miss) : 0.5;
}

bool IncidentManager::triggered(const swarm_cbba::EventType & et, const DetectionIn & det, double now) const
{
  if (et.zone_attr != "any") {
    if (!det.position_valid) {
      return false;
    }
    bool ok = false;
    for (const auto & z : site_.zones) {
      if (z.contains(det.x, det.y) &&
        ((et.zone_attr == "restricted" && z.restricted) || (et.zone_attr == "high_voltage" && z.high_voltage)))
      {
        ok = true;
      }
    }
    if (!ok) {
      return false;
    }
  }
  if (et.time_rule != "always") {
    const int h = config_.hour_of(now);
    const bool on = h >= config_.on_hours_start && h < config_.on_hours_end;
    if ((et.time_rule == "off_hours" && on) || (et.time_rule == "on_hours" && !on)) {
      return false;
    }
  }
  return true;
}

void IncidentManager::setState(EventRecord & rec, EventState state, const std::string & reason, double now)
{
  if (rec.state != state) {
    log_.push_back(fmt(now, 3) + " " + rec.event_id + " " + stateName(rec.state) + " -> " + stateName(state) +
      (reason.empty() ? "" : "（" + reason + "）"));
  }
  rec.state = state;
  rec.reason = reason;
}

void IncidentManager::decay(EventRecord & rec, const swarm_cbba::EventType & et, double now)
{
  // 沒有新偵測超過 kForgetAfter 才遺忘：每秒乘 λ（偵測之間的 λ 已經在 onDetection 乘過）
  const double start = std::max(rec.s_t, rec.last_t + kForgetAfter);
  if (et.has_accumulate && now > start) {
    rec.s *= std::pow(et.lambda, now - start);
    rec.s_t = now;
  }
}

void IncidentManager::advance(EventRecord & rec, const swarm_cbba::EventType & et, double now,
  std::vector<Action> & out)
{
  if (rec.state != EventState::CANDIDATE) {
    return;
  }
  const double on = et.has_accumulate ? et.theta_on : 0.0;
  const double off = et.has_accumulate ? et.theta_off : -1e9;
  if (rec.s >= on) {
    if (!rec.above_since) {
      rec.above_since = now;
    }
  } else {
    rec.above_since.reset();
    if (rec.s < off) {
      setState(rec, EventState::REJECTED, "證據消退（s " + fmt(rec.s) + " < theta_off " + fmt(off) + "）", now);
      return;
    }
  }
  const double need = et.has_accumulate ? et.min_duration_s : 0.0;
  if (!rec.above_since || now - *rec.above_since < need) {
    return;
  }
  if (et.has_vlm && config_.vlm_enabled) {
    rec.vlm_pending = true;
    rec.vlm_deadline = now + et.vlm_timeout_s;
    setState(rec, EventState::VERIFYING, "送 VLM 驗證（" + et.vlm_prompt_id + "）", now);
    out.push_back(action(Action::Kind::VLM_REQUEST, rec, "", et.vlm_prompt_id));
    return;
  }
  decide(rec, et, sigmoid(rec.s), now, out);
}

void IncidentManager::decide(EventRecord & rec, const swarm_cbba::EventType & et, double p, double now,
  std::vector<Action> & out)
{
  rec.confidence = p;
  rec.decided_t = now;
  const double low = thetaLow(et);
  const std::string why = "信心 " + fmt(p) + "（θ_low " + fmt(low) + "、θ_high " + fmt(et.theta_high) + "）";
  if (p >= et.theta_high) {
    setState(rec, EventState::CONFIRMED, why, now);
    if (absorbIntoStorm(rec, now, out)) {
      return;
    }
    out.push_back(action(Action::Kind::NOTIFY, rec, "", et.name + " 確認，派完整範本 " + et.default_rule));
    dispatch(rec, et.default_rule, now, out);
    return;
  }
  if (p >= low) {
    setState(rec, EventState::NEEDS_CHECK, why, now);
    const Rule * full = rules_.rule(et.default_rule);
    const std::string verify = full ? full->fallback_template : std::string();
    if (verify.empty()) {
      out.push_back(action(Action::Kind::NOTIFY, rec, "", et.name + " 中信心、範本沒有確認任務：請值班判斷"));
      return;
    }
    if (absorbIntoStorm(rec, now, out)) {
      return;
    }
    out.push_back(action(Action::Kind::NOTIFY, rec, "", et.name + " 中信心，先派 1 隻確認（" + verify + "）"));
    dispatch(rec, verify, now, out);
    return;
  }
  setState(rec, EventState::LOGGED, why, now);
}

void IncidentManager::dispatch(EventRecord & rec, const std::string & rule, double now, std::vector<Action> & out)
{
  rec.rule = rule;
  rec.plan_version = static_cast<std::uint16_t>(rec.plan_version + 1);
  rec.dispatched_x = rec.x;
  rec.dispatched_y = rec.y;
  setState(rec, EventState::DISPATCHED, "展開 " + rule, now);
  out.push_back(action(Action::Kind::DISPATCH, rec, rule));
}

void IncidentManager::finish(EventRecord & rec, EventState state, const std::string & reason, double now,
  std::vector<Action> & out)
{
  setState(rec, state, reason, now);
  out.push_back(action(Action::Kind::NOTIFY, rec, "", rec.event_id + " " + stateName(state) + "：" + reason));
  // 區域巡查結案 → 併進來的事件跟著結案
  for (const auto & id : rec.merged) {
    const auto it = events_.find(id);
    if (it != events_.end() && !isTerminal(it->second.state)) {
      finish(it->second, state, "合併的區域巡查 " + rec.event_id + " " + stateName(state), now, out);
    }
  }
  // fallback 子事件結案 → 原本的事件跟著結案
  if (!rec.parent.empty()) {
    const auto it = events_.find(rec.parent);
    if (it != events_.end() && !isTerminal(it->second.state)) {
      finish(it->second, state, "降級的 " + rec.event_id + " " + stateName(state), now, out);
    }
  }
}

void IncidentManager::aggregate(EventRecord & rec, const std::string & role, double now)
{
  if (!isDispatched(rec.state)) {
    return;
  }
  bool on_site = false;
  bool executing = false;
  for (const auto & [name, st] : rec.roles) {
    const bool started = st.state == RoleState::EXECUTING || st.state == RoleState::DONE;
    on_site = on_site || started || st.state == RoleState::ARRIVED;
    executing = executing || (!st.optional && started);
  }
  const EventState next = executing ? EventState::EXECUTING : on_site ? EventState::ON_SITE : EventState::DISPATCHED;
  if (next == rec.state) {
    return;
  }
  const auto rank = [](EventState s) {
      return s == EventState::EXECUTING ? 2 : s == EventState::ON_SITE ? 1 : 0;
    };
  const auto it = rec.roles.find(role);
  const std::string rs = it != rec.roles.end() ? roleStateName(it->second.state) : "?";
  setState(rec, next, rank(next) > rank(rec.state) ? "角色 " + role + " " + rs :
    "角色 " + role + " " + rs + "：換人或改版，舊進度不算", now);
}

bool IncidentManager::absorbIntoStorm(EventRecord & rec, double now, std::vector<Action> & out)
{
  // 已經派過的（中信心先派 1 隻、證據變強要升級）不併：它的角色已經在做了
  if (config_.storm_max_per_zone <= 0 || rec.zone.empty() || rec.storm || !rec.parent.empty() || !rec.rule.empty() ||
    rules_.rule(config_.storm_rule) == nullptr)
  {
    return false;
  }
  EventRecord * storm = nullptr;
  int active = 0;
  for (auto & [id, r] : events_) {
    if (id == rec.event_id || r.zone != rec.zone || isTerminal(r.state)) {
      continue;
    }
    if (r.storm) {
      storm = &r;
    } else if (r.parent.empty() && isDispatched(r.state)) {
      ++active;
    }
  }
  if (storm == nullptr && active < config_.storm_max_per_zone) {
    return false;
  }
  if (storm == nullptr) {
    EventRecord fresh;
    fresh.event_id = newId();
    fresh.event_type = rec.event_type;
    fresh.storm = true;
    fresh.first_t = now;
    fresh.s_t = now;
    fresh.last_t = now;
    fresh.decided_t = now;
    fresh.x = rec.x;
    fresh.y = rec.y;
    fresh.z = rec.z;
    fresh.zone = rec.zone;
    fresh.confidence = rec.confidence;
    storm = &events_.emplace(fresh.event_id, fresh).first->second;
    log_.push_back(fmt(now, 3) + " " + storm->event_id + " 告警風暴：" + rec.zone + " 已有 " + std::to_string(active) +
      " 件派遣中（上限 " + std::to_string(config_.storm_max_per_zone) + "）→ 合併成區域巡查");
    out.push_back(action(Action::Kind::NOTIFY, *storm, "", rec.zone + " 同時 " + std::to_string(active + 1) +
      " 件告警，超過上限 " + std::to_string(config_.storm_max_per_zone) + "：合併成區域巡查 " + storm->event_id +
      "（" + config_.storm_rule + "），請值班注意"));
    dispatch(*storm, config_.storm_rule, now, out);
  }
  storm->merged.push_back(rec.event_id);
  storm->sources.insert(rec.sources.begin(), rec.sources.end());
  rec.merged_into = storm->event_id;
  setState(rec, EventState::MERGED, "告警風暴：併進區域巡查 " + storm->event_id, now);
  out.push_back(action(Action::Kind::NOTIFY, rec, "", rec.event_id + "（" + rec.event_type + "）併進 " + rec.zone +
    " 的區域巡查 " + storm->event_id));
  return true;
}

std::vector<Action> IncidentManager::onDetection(const DetectionIn & det, double now)
{
  std::vector<Action> out;
  const swarm_cbba::DetectorEntry * detector = nullptr;
  for (const auto & d : registry_.detectors) {
    if (d.name == det.detector) {
      detector = &d;
    }
  }
  if (detector == nullptr || det.score < detector->min_score ||
    std::find(detector->labels.begin(), detector->labels.end(), det.label) == detector->labels.end())
  {
    return out;
  }
  for (const auto & et : registry_.event_types) {
    if (et.status != "open" || et.category != "template" ||
      std::find(et.detectors.begin(), et.detectors.end(), det.detector) == et.detectors.end() ||
      !triggered(et, det, now))
    {
      continue;
    }
    // ---- L3 關聯 ----
    EventRecord * rec = nullptr;
    for (auto & [id, r] : events_) {
      if (r.event_type != et.name || isTerminal(r.state) || !r.parent.empty() || r.storm) {
        continue;
      }
      const bool same_track = det.track_id != 0 && det.track_id == r.track_id;
      const bool near = det.position_valid && (!et.has_associate || now - r.last_t <= et.associate_dt_s) &&
        std::hypot(det.x - r.x, det.y - r.y) <= (et.has_associate ? et.associate_r_m : 0.0);
      if (same_track || near) {
        rec = &r;
        break;
      }
    }
    if (rec == nullptr) {
      EventRecord fresh;
      fresh.event_id = newId();
      fresh.event_type = et.name;
      fresh.first_t = now;
      fresh.s_t = now;
      fresh.x = det.x;
      fresh.y = det.y;
      fresh.z = det.z;
      fresh.radius_m = et.has_associate ? et.associate_r_m : 0.0;
      rec = &events_.emplace(fresh.event_id, fresh).first->second;
      log_.push_back(fmt(now, 3) + " " + rec->event_id + " 新候選 " + et.name + "（" + det.detector + "）");
    }
    decay(*rec, et, now);
    // ---- L2 累積 ----
    rec->s = (et.has_accumulate ? et.lambda : 1.0) * rec->s + logit(det.score);
    rec->s_t = now;
    rec->last_t = now;
    rec->detections += 1;
    rec->sources.insert(det.detector + ":" + det.source);
    if (det.track_id != 0) {
      rec->track_id = det.track_id;
    }
    if (!det.source.empty()) {
      rec->evidence.cameras.insert(det.source);
    }
    if ((!det.snapshot_uri.empty() || !det.evidence_uri.empty()) && det.score >= rec->evidence.score) {
      rec->evidence.score = det.score;
      rec->evidence.snapshot_uri = det.snapshot_uri;
      rec->evidence.clip_uri = det.clip_uri;
      rec->evidence.evidence_uri = det.evidence_uri;
    }
    if (det.position_valid) {
      rec->x = det.x;
      rec->y = det.y;
      rec->z = det.z;
      rec->zone = zoneAt(site_, det.x, det.y);
    }
    switch (rec->state) {
      case EventState::CANDIDATE:
        advance(*rec, et, now, out);
        break;
      case EventState::LOGGED:
      case EventState::NEEDS_CHECK:
        // 證據變強：重新決定（LOGGED → 派確認；中信心 → 升級成完整範本）
        if (sigmoid(rec->s) >= (rec->state == EventState::LOGGED ? thetaLow(et) : et.theta_high)) {
          decide(*rec, et, sigmoid(rec->s), now, out);
        }
        break;
      case EventState::DISPATCHED:
      case EventState::ON_SITE:
      case EventState::EXECUTING:
        if (std::hypot(rec->x - rec->dispatched_x, rec->y - rec->dispatched_y) > config_.retarget_m) {
          // 中信心派的確認任務：證據夠了直接升級成完整範本；否則照新位置重新展開
          if (rec->rule != et.default_rule && sigmoid(rec->s) >= et.theta_high) {
            decide(*rec, et, sigmoid(rec->s), now, out);
          } else {
            rec->plan_version = static_cast<std::uint16_t>(rec->plan_version + 1);
            rec->dispatched_x = rec->x;
            rec->dispatched_y = rec->y;
            log_.push_back(fmt(now, 3) + " " + rec->event_id + " 位置移動 → 重新展開（plan " +
              std::to_string(rec->plan_version) + "）");
            out.push_back(action(Action::Kind::REPLAN, *rec, rec->rule));
          }
        } else if (rec->rule != et.default_rule && sigmoid(rec->s) >= et.theta_high) {
          decide(*rec, et, sigmoid(rec->s), now, out);
        }
        break;
      default:
        break;
    }
  }
  return out;
}

std::vector<Action> IncidentManager::onVlmResult(const std::string & event_id, const std::string & verdict,
  double confidence, double now)
{
  std::vector<Action> out;
  auto it = events_.find(event_id);
  if (it == events_.end() || it->second.state != EventState::VERIFYING) {
    return out;
  }
  EventRecord & rec = it->second;
  const auto * et = typeOf(rec);
  if (et == nullptr) {
    return out;
  }
  rec.vlm_pending = false;
  if (verdict == "rejected") {
    finish(rec, EventState::REJECTED, "VLM 否定", now, out);
    return out;
  }
  double p = std::clamp(confidence, 0.0, 1.0);
  if (verdict == "uncertain") {
    p = std::min(p, et->theta_high - 1e-6);   // 不確定：最多派 1 隻確認
  }
  decide(rec, *et, p, now, out);
  return out;
}

std::vector<Action> IncidentManager::onDispatchResult(const std::string & event_id, bool ok,
  const std::vector<std::pair<std::string, bool>> & roles, const std::string & reason, double now)
{
  std::vector<Action> out;
  auto it = events_.find(event_id);
  if (it == events_.end()) {
    return out;
  }
  EventRecord & rec = it->second;
  if (!ok) {
    setState(rec, EventState::ESCALATED, "展開失敗：" + reason, now);
    out.push_back(action(Action::Kind::NOTIFY, rec, "", rec.event_id + " 展開失敗，請值班處理：" + reason));
    return out;
  }
  std::map<std::string, RoleStatus> next;
  for (const auto & [role, optional] : roles) {
    RoleStatus st;
    st.optional = optional;
    const auto old = rec.roles.find(role);
    if (old != rec.roles.end()) {
      st.state = old->second.state;   // 改版：保留角色的進度
    }
    next[role] = st;
  }
  rec.roles = std::move(next);
  aggregate(rec, roles.empty() ? std::string() : roles.front().first, now);
  return out;
}

std::vector<Action> IncidentManager::onIncidentUpdate(const std::string & event_id, std::uint16_t revision,
  bool position_valid, double x, double y, double now)
{
  std::vector<Action> out;
  auto it = events_.find(event_id);
  if (it == events_.end() || isTerminal(it->second.state) || revision <= it->second.incident_revision) {
    return out;
  }
  EventRecord & rec = it->second;
  rec.incident_revision = revision;
  if (position_valid) {
    rec.x = x;
    rec.y = y;
    rec.zone = zoneAt(site_, x, y);
  }
  if (isDispatched(rec.state) && !rec.rule.empty()) {
    rec.plan_version = static_cast<std::uint16_t>(rec.plan_version + 1);
    rec.dispatched_x = rec.x;
    rec.dispatched_y = rec.y;
    log_.push_back(fmt(now, 3) + " " + rec.event_id + " incident revision " + std::to_string(revision) +
      " → 重新展開（plan " + std::to_string(rec.plan_version) + "）");
    out.push_back(action(Action::Kind::REPLAN, rec, rec.rule));
  }
  return out;
}

std::vector<Action> IncidentManager::onRoleState(const std::string & event_id, const std::string & role,
  RoleState state, const std::string & failure_code, double now)
{
  std::vector<Action> out;
  auto it = events_.find(event_id);
  if (it == events_.end() || isTerminal(it->second.state)) {
    return out;
  }
  EventRecord & rec = it->second;
  RoleStatus & rs = rec.roles[role];
  rs.state = state;
  if (state == RoleState::FAILED) {
    rs.failure_code = failure_code;   // 之後重新指派也留著最近一次的失敗碼
  }
  if (state == RoleState::FAILED && failure_code == "E_TARGET_LOST") {
    finish(rec, EventState::FALSE_ALARM, "角色 " + role + " 到場沒有看到目標（E_TARGET_LOST）", now, out);
    return out;
  }
  aggregate(rec, role, now);
  bool any_mandatory = false;
  bool all_done = true;
  for (const auto & [name, st] : rec.roles) {
    if (st.optional) {
      continue;
    }
    any_mandatory = true;
    all_done = all_done && st.state == RoleState::DONE;
  }
  if (any_mandatory && all_done) {
    finish(rec, EventState::RESOLVED, "必要角色都完成", now, out);
  }
  return out;
}

std::vector<Action> IncidentManager::onEscalate(const std::string & event_id, const std::string & reason,
  const std::string & fallback_rule, double now)
{
  std::vector<Action> out;
  auto it = events_.find(event_id);
  if (it == events_.end() || isTerminal(it->second.state)) {
    return out;
  }
  EventRecord & rec = it->second;
  setState(rec, EventState::ESCALATED, reason, now);
  std::string fb = fallback_rule;
  if (fb.empty()) {
    const Rule * r = rules_.rule(rec.rule);
    fb = r ? r->fallback_template : std::string();
  }
  if (fb.empty() || fb == rec.rule) {
    out.push_back(action(Action::Kind::NOTIFY, rec, "", rec.event_id + " 升級，請值班處理：" + reason));
    return out;
  }
  // 降級：派 fallback 範本的子事件（新的 event_id，閘道當成新的事件）
  EventRecord child;
  child.event_id = rec.event_id + "-fb" + std::to_string(rec.children.size() + 1);
  child.event_type = rec.event_type;
  child.first_t = now;
  child.s_t = now;
  child.last_t = now;
  child.decided_t = now;
  child.x = rec.x;
  child.y = rec.y;
  child.z = rec.z;
  child.zone = rec.zone;
  child.track_id = rec.track_id;
  child.radius_m = rec.radius_m;
  child.confidence = rec.confidence;
  child.parent = rec.event_id;
  rec.children.push_back(child.event_id);
  out.push_back(action(Action::Kind::NOTIFY, rec, "", rec.event_id + " 升級（" + reason + "），降級派 " + fb));
  EventRecord & c = events_.emplace(child.event_id, child).first->second;
  log_.push_back(fmt(now, 3) + " " + c.event_id + " 降級子事件（" + rec.event_id + "）");
  dispatch(c, fb, now, out);
  return out;
}

std::vector<Action> IncidentManager::resolve(const std::string & event_id, bool false_alarm, const std::string & who,
  double now)
{
  std::vector<Action> out;
  auto it = events_.find(event_id);
  if (it == events_.end() || isTerminal(it->second.state)) {
    return out;
  }
  finish(it->second, false_alarm ? EventState::FALSE_ALARM : EventState::RESOLVED, "值班 " + who + " 判定", now,
    out);
  return out;
}

std::vector<Action> IncidentManager::step(double now)
{
  std::vector<Action> out;
  for (auto & [id, rec] : events_) {
    if (isTerminal(rec.state)) {
      continue;
    }
    const auto * et = typeOf(rec);
    if (et == nullptr) {
      continue;
    }
    if (rec.state == EventState::CANDIDATE) {
      decay(rec, *et, now);
      advance(rec, *et, now, out);
      continue;
    }
    if (rec.state == EventState::VERIFYING && rec.vlm_pending && now >= rec.vlm_deadline) {
      // VLM 逾時（2-1 L4）：P0、P1 直接用 L2 的信心決定；P2、P3 再等一次，還是逾時才用 L2
      if (et->priority >= 2 && rec.vlm_retries == 0) {
        rec.vlm_retries = 1;
        rec.vlm_deadline = now + et->vlm_timeout_s;
        log_.push_back(fmt(now, 3) + " " + rec.event_id + " VLM 逾時，再送一次");
        out.push_back(action(Action::Kind::VLM_REQUEST, rec, "", et->vlm_prompt_id));
      } else {
        rec.vlm_pending = false;
        log_.push_back(fmt(now, 3) + " " + rec.event_id + " VLM 逾時，用 L2 的信心決定");
        decide(rec, *et, sigmoid(rec.s), now, out);
      }
      continue;
    }
    if (et->expires_s > 0.0 && rec.decided_t > 0.0 && now - rec.decided_t > et->expires_s) {
      finish(rec, EventState::EXPIRED, "超過 expires_s " + fmt(et->expires_s, 0) + " s", now, out);
    }
  }
  return out;
}

std::string IncidentManager::save() const
{
  YAML::Emitter e;
  e << YAML::BeginMap << YAML::Key << "next_id" << YAML::Value << next_id_ << YAML::Key << "events"
    << YAML::Value << YAML::BeginSeq;
  for (const auto & [id, r] : events_) {
    if (isTerminal(r.state)) {
      continue;
    }
    e << YAML::BeginMap;
    e << YAML::Key << "event_id" << YAML::Value << r.event_id;
    e << YAML::Key << "event_type" << YAML::Value << r.event_type;
    e << YAML::Key << "state" << YAML::Value << stateName(r.state);
    e << YAML::Key << "incident_revision" << YAML::Value << r.incident_revision;
    e << YAML::Key << "plan_version" << YAML::Value << r.plan_version;
    e << YAML::Key << "pos" << YAML::Value << YAML::Flow << YAML::BeginSeq << r.x << r.y << r.z << YAML::EndSeq;
    e << YAML::Key << "radius_m" << YAML::Value << r.radius_m;
    e << YAML::Key << "zone" << YAML::Value << r.zone;
    e << YAML::Key << "track_id" << YAML::Value << r.track_id;
    e << YAML::Key << "s" << YAML::Value << r.s;
    e << YAML::Key << "confidence" << YAML::Value << r.confidence;
    e << YAML::Key << "first_t" << YAML::Value << r.first_t;
    e << YAML::Key << "last_t" << YAML::Value << r.last_t;
    e << YAML::Key << "decided_t" << YAML::Value << r.decided_t;
    e << YAML::Key << "rule" << YAML::Value << r.rule;
    e << YAML::Key << "parent" << YAML::Value << r.parent;
    e << YAML::Key << "storm" << YAML::Value << r.storm;
    e << YAML::Key << "merged_into" << YAML::Value << r.merged_into;
    e << YAML::Key << "merged" << YAML::Value << YAML::Flow << r.merged;
    e << YAML::Key << "evidence" << YAML::Value << YAML::Flow << YAML::BeginSeq << r.evidence.snapshot_uri <<
      r.evidence.clip_uri << r.evidence.evidence_uri << r.evidence.score << YAML::EndSeq;
    e << YAML::Key << "roles" << YAML::Value << YAML::BeginMap;
    for (const auto & [name, st] : r.roles) {
      e << YAML::Key << name << YAML::Value << YAML::Flow << YAML::BeginSeq << st.optional
        << roleStateName(st.state) << YAML::EndSeq;
    }
    e << YAML::EndMap << YAML::EndMap;
  }
  e << YAML::EndSeq << YAML::EndMap;
  return e.c_str();
}

void IncidentManager::load(const std::string & yaml)
{
  const YAML::Node doc = YAML::Load(yaml);
  if (doc["next_id"]) {
    next_id_ = std::max(next_id_, doc["next_id"].as<std::uint64_t>());
  }
  if (!doc["events"]) {
    return;
  }
  for (const auto & n : doc["events"]) {
    EventRecord r;
    r.event_id = n["event_id"].as<std::string>();
    r.event_type = n["event_type"].as<std::string>();
    r.state = stateFromName(n["state"].as<std::string>()).value_or(EventState::ESCALATED);
    r.incident_revision = n["incident_revision"].as<std::uint16_t>();
    r.plan_version = n["plan_version"].as<std::uint16_t>();
    r.x = n["pos"][0].as<double>();
    r.y = n["pos"][1].as<double>();
    r.z = n["pos"][2].as<double>();
    r.dispatched_x = r.x;
    r.dispatched_y = r.y;
    r.radius_m = n["radius_m"].as<double>();
    r.zone = n["zone"].as<std::string>();
    r.track_id = n["track_id"].as<std::uint32_t>();
    r.s = n["s"].as<double>();
    r.confidence = n["confidence"].as<double>();
    r.first_t = n["first_t"].as<double>();
    r.last_t = n["last_t"].as<double>();
    r.s_t = r.last_t;
    r.decided_t = n["decided_t"].as<double>();
    r.rule = n["rule"].as<std::string>();
    r.parent = n["parent"].as<std::string>();
    // C 批加的欄位（B 批的存檔沒有）
    r.storm = n["storm"] && n["storm"].as<bool>();
    r.merged_into = n["merged_into"] ? n["merged_into"].as<std::string>() : std::string();
    if (n["merged"]) {
      r.merged = n["merged"].as<std::vector<std::string>>();
    }
    if (const auto ev = n["evidence"]; ev && ev.size() == 4) {
      r.evidence.snapshot_uri = ev[0].as<std::string>();
      r.evidence.clip_uri = ev[1].as<std::string>();
      r.evidence.evidence_uri = ev[2].as<std::string>();
      r.evidence.score = ev[3].as<double>();
    }
    if (const auto roles = n["roles"]) {
      for (const auto & kv : roles) {
        RoleStatus st;
        st.optional = kv.second[0].as<bool>();
        st.state = roleStateFromName(kv.second[1].as<std::string>()).value_or(RoleState::PLANNED);
        r.roles[kv.first.as<std::string>()] = st;
      }
    }
    events_[r.event_id] = r;
  }
  for (auto & [id, r] : events_) {
    if (!r.parent.empty()) {
      auto p = events_.find(r.parent);
      if (p != events_.end() &&
        std::find(p->second.children.begin(), p->second.children.end(), id) == p->second.children.end())
      {
        p->second.children.push_back(id);
      }
    }
  }
}

}  // namespace swarm_events
