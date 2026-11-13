#include "cbba_core/gateway.hpp"

#include <cmath>
#include <iterator>
#include <set>

#include "cbba_core/wire_task_spec.hpp"

namespace cbba_core
{

namespace
{
// 期限、失效等時刻在封包裡是毫秒的低 32 位元（約 49.7 天循環，還原時取 ±24.8 天內最近的），超過就會繞回去
constexpr double kMaxRelativeSec = 20.0 * 86400.0;
// 封包裡 uint16 的欄位（service_time_s、commit_radius_dm）
constexpr double kMaxU16 = 65535.0;
}  // namespace

const char * toString(InjectStatus status)
{
  switch (status) {
    case InjectStatus::CREATED: return "建立";
    case InjectStatus::DUPLICATE: return "重複（沿用）";
    case InjectStatus::REVISED: return "改版";
    case InjectStatus::REJECTED: return "拒絕";
  }
  return "?";
}

std::string Gateway::check(const InjectRequest & req) const
{
  const Task & t = req.task;
  const TaskExt & e = t.ext;
  if (req.incident_id.empty()) {
    return "incident_id 是空的";
  }
  if (t.platform > 2) {
    return "platform 要是 0（不限）、1（uav）、2（quadruped）";
  }
  if (t.priority > 3) {
    return "priority 要是 0～3（P0 最高）";
  }
  if (!std::isfinite(t.position.x) || !std::isfinite(t.position.y) || !std::isfinite(t.position.z)) {
    return "position 不是有限值";
  }
  if (!std::isfinite(t.value) || t.value <= 0.0) {
    return "value 要大於 0";
  }
  if (!std::isfinite(t.duration_sec) || t.duration_sec < 0.0 || t.duration_sec > kMaxU16) {
    return "service_time_s 要在 0～65535";
  }
  for (double s : {req.deadline_sec, req.release_sec, req.expires_sec}) {
    if (!std::isfinite(s) || s < 0.0 || s > kMaxRelativeSec) {
      return "deadline_sec、release_sec、expires_sec 要在 0～20 天（0 = 沒有）";
    }
  }
  if (req.expires_sec > 0.0 && req.release_sec >= req.expires_sec) {
    return "release_sec 要早於 expires_sec";
  }
  if (!std::isfinite(e.commit_radius) || e.commit_radius < 0.0 || e.commit_radius * 10.0 > kMaxU16) {
    return "commit_radius 要在 0～6553 m";
  }
  if (e.min_battery_reserve > 100 || e.preempt_margin_pct > 100) {
    return "min_battery_reserve、preempt_margin_pct 要在 0～100";
  }
  if (e.can_preempt > 0x0F) {
    return "can_preempt 只有 P0～P3 四個位元";
  }
  if (e.role_count > 8) {
    return "role_count 最多 8（precedence 是 8 位元）";
  }
  if (e.role_count == 0) {
    if (e.role_index != 0 || e.precedence != 0 || e.optional || e.all_or_nothing || e.skip_satisfies ||
      e.sync_window_s != 0)
    {
      return "單獨任務（role_count = 0）不能設角色、precedence、optional 等協同欄位";
    }
  } else {
    if (req.role.empty()) {
      return "角色組（role_count > 0）要給 role 名稱";
    }
    if (e.role_index >= e.role_count) {
      return "role_index 要小於 role_count";
    }
    if ((e.precedence >> e.role_count) != 0 || (e.precedence & (1u << e.role_index)) != 0) {
      return "precedence 只能指同組的其他角色";
    }
  }
  if (e.skip_satisfies && !e.optional) {
    return "skip_satisfies 只用在 optional 角色";
  }
  if (req.spec.size() > wire::kMaxSpecChunk * wire::kMaxSpecParts) {
    return "spec 太大（上限 " + std::to_string(wire::kMaxSpecChunk * wire::kMaxSpecParts) + " B）";
  }
  return {};
}

InjectResult Gateway::inject(const InjectRequest & req, double now)
{
  InjectResult r;
  r.revision = req.revision;
  const auto key = std::make_pair(req.incident_id, req.role);
  const auto known = injected_.find(key);
  if (known != injected_.end()) {
    r.task_id = known->second.task_id;
    const auto group = groups_.find(req.incident_id);
    r.group_id = group != groups_.end() ? group->second : 0;
    if (req.revision == known->second.revision) {
      r.status = InjectStatus::DUPLICATE;   // 冪等：同一個鍵重送，不重發
      return r;
    }
    if (req.revision > known->second.revision) {
      return revise(req, known->second, now);
    }
    r.reason = "revision " + std::to_string(req.revision) + " 比已注入的 " +
      std::to_string(known->second.revision) + " 舊";
    r.revision = known->second.revision;
    return r;
  }
  if (const std::string bad = check(req); !bad.empty()) {
    r.reason = bad;
    return r;
  }
  if (comm_.epoch() == 0) {
    r.reason = "沒有世代（epoch = 0），不能產生版本 4 的 task_id";
    return r;
  }

  TaskId group_id = 0;
  if (req.task.ext.role_count > 0) {
    const auto g = groups_.find(req.incident_id);
    if (g != groups_.end()) {
      group_id = g->second;
    } else if (const auto id = comm_.newTaskId()) {
      group_id = *id;
    } else {
      r.reason = "流水號用完（要重新啟動換世代）";
      return r;
    }
  }
  const auto id = comm_.newTaskId();
  if (!id) {
    r.reason = "流水號用完（要重新啟動換世代）";
    return r;
  }

  Task t = build(req, now, now);
  t.id = *id;
  t.ext.group_id = group_id;
  if (!comm_.addLocalTask(t, now, req.spec)) {
    r.reason = "協定層拒收（task_id 重複或 spec 超過上限）";
    return r;
  }
  if (group_id != 0) {
    groups_.emplace(req.incident_id, group_id);
  }
  injected_[key] = Injected{t.id, t.revision};
  r.status = InjectStatus::CREATED;
  r.task_id = t.id;
  r.group_id = group_id;
  return r;
}

const char * toString(OptionalDrop reason)
{
  switch (reason) {
    case OptionalDrop::NO_CAPABLE: return "O1 全隊沒有能做的載具";
    case OptionalDrop::UNASSIGNED: return "O2 有人能做但沒人接";
    case OptionalDrop::GROUP_DONE: return "O3 同組的必要角色都結束了";
  }
  return "?";
}

bool Gateway::membershipReady(double now)
{
  const auto alive = comm_.aliveNeighbors(now);
  if (alive.empty()) {
    connected_since_ = -1.0;   // 斷線：恢復後重新等
    return false;
  }
  if (connected_since_ < 0.0) {
    connected_since_ = now;
  }
  const CommConfig & c = comm_.config();
  if (!comm_.joined() || now - connected_since_ < c.lost_timeout + c.status_period) {
    return false;
  }
  for (const AgentId a : alive) {
    const NeighborInfo & n = comm_.neighbors().at(a);
    if (!n.has_status || n.last_status < connected_since_) {
      return false;
    }
  }
  return true;
}

std::vector<DroppedOptional> Gateway::step(double now)
{
  std::vector<DroppedOptional> out;
  const bool ready = membershipReady(now);
  if (now < next_check_) {
    return out;
  }
  next_check_ = now + 0.2;
  std::vector<std::pair<TaskId, DroppedOptional>> todo;
  std::set<TaskId> watched;
  for (const auto & [id, task] : comm_.agent().tasks()) {
    if (task.status != TaskStatus::OPEN || !task.ext.optional || taskCreator(id) != comm_.id()) {
      continue;
    }
    watched.insert(id);
    if (comm_.groupMandatoryDone(task.ext.group_id, task.ext.role_count)) {
      todo.push_back({id, DroppedOptional{id, OptionalDrop::GROUP_DONE, 0.0}});   // O3：執行中的也關
      continue;
    }
    if (comm_.assignment(id).assignee != kNoAgent) {
      no_capable_since_.erase(id);   // 有正式指派（待接受、保留、執行中）：O1、O2 不關
      unassigned_since_.erase(id);
      continue;
    }
    if (comm_.capableAgents(task, now).empty()) {
      unassigned_since_.erase(id);
      if (!ready) {
        no_capable_since_.erase(id);   // 還不知道全隊有誰（剛啟動、斷線）：不計時
        continue;
      }
      const double since = no_capable_since_.emplace(id, now).first->second;
      const double limit = task.ext.optional_no_capable_s > 0 ? task.ext.optional_no_capable_s : 10.0;
      if (now - since >= limit) {
        todo.push_back({id, DroppedOptional{id, OptionalDrop::NO_CAPABLE, now - since}});
      }
    } else {
      no_capable_since_.erase(id);
      const double since = unassigned_since_.emplace(id, now).first->second;
      const double limit = task.ext.optional_unassigned_s > 0 ? task.ext.optional_unassigned_s : 15.0;
      if (now - since >= limit) {
        todo.push_back({id, DroppedOptional{id, OptionalDrop::UNASSIGNED, now - since}});
      }
    }
  }
  for (const auto & [id, drop] : todo) {
    if (comm_.dropOptional(id, now) == ReportCheck::ACCEPTED) {
      out.push_back(drop);
    }
    watched.erase(id);
  }
  // 結束了、不再看的任務：計時清掉
  for (auto * timers : {&no_capable_since_, &unassigned_since_}) {
    for (auto it = timers->begin(); it != timers->end(); ) {
      it = watched.count(it->first) > 0 ? std::next(it) : timers->erase(it);
    }
  }
  return out;
}

Task Gateway::build(const InjectRequest & req, double created, double now)
{
  Task t = req.task;
  t.type = TaskType::NONE;
  t.status = TaskStatus::OPEN;
  t.created = created;
  t.status_stamp = now;
  t.explicit_deadline = req.deadline_sec > 0.0;
  // 期限是「從現在起」：改版時 created 是原本的建立時刻，換算成從 created 起算
  t.deadline_sec = t.explicit_deadline ? now + req.deadline_sec - created : 300.0;
  t.revision = req.revision;
  t.success_prob = 1.0;
  t.ext.release = req.release_sec > 0.0 ? now + req.release_sec : 0.0;
  t.ext.expires = req.expires_sec > 0.0 ? now + req.expires_sec : 0.0;
  return t;
}

InjectResult Gateway::revise(const InjectRequest & req, Injected & known, double now)
{
  InjectResult r;
  r.task_id = known.task_id;
  r.revision = known.revision;
  const auto group = groups_.find(req.incident_id);
  r.group_id = group != groups_.end() ? group->second : 0;
  if (const std::string bad = check(req); !bad.empty()) {
    r.reason = bad;
    return r;
  }
  const auto it = comm_.agent().tasks().find(known.task_id);
  if (it == comm_.agent().tasks().end()) {
    r.reason = "任務還沒送出（閘道還在和鄰居同步），稍後再改版";
    return r;
  }
  const Task & old = it->second;
  if (old.status != TaskStatus::OPEN) {
    r.reason = "任務已結束，不能改版";
    return r;
  }
  if (req.task.ext.role_count != old.ext.role_count || req.task.ext.role_index != old.ext.role_index) {
    r.reason = "角色組（role_index、role_count）不能改版：要另外建任務";
    return r;
  }
  Task t = build(req, old.created, now);
  t.id = known.task_id;
  t.ext.group_id = old.ext.group_id;
  if (!comm_.reviseLocalTask(t, now, req.spec)) {
    r.reason = "協定層拒收改版";
    return r;
  }
  known.revision = req.revision;
  r.status = InjectStatus::REVISED;
  r.revision = req.revision;
  return r;
}

}  // namespace cbba_core
