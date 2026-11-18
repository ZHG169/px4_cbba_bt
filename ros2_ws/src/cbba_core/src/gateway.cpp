#include "cbba_core/gateway.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <optional>
#include <set>
#include <sstream>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/wire_task_spec.hpp"
#include "cbba_core/zones.hpp"

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
  // 時間的先後（任務欄位補齊 06，doc/protocol.md 第九節）：release < 硬期限 ≤ expires；軟期限只影響排隊，不檢查先後
  if (req.expires_sec > 0.0 && req.release_sec >= req.expires_sec) {
    return "release_sec 要早於 expires_sec";
  }
  if (e.deadline_hard && req.deadline_sec > 0.0) {
    if (req.release_sec >= req.deadline_sec) {
      return "release_sec 要早於硬期限 deadline_sec（release 之後才能開始，永遠來不及）";
    }
    if (req.expires_sec > 0.0 && req.deadline_sec > req.expires_sec) {
      return "硬期限 deadline_sec 不能晚於 expires_sec（expires 時任務已失效）";
    }
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
  if (e.role_count > 0 && e.role_count < 8 && (e.distinct_agent_set >> e.role_count) != 0) {
    return "distinct_agent_set 只能指同組的角色";
  }
  if (e.role_count == 0 && e.distinct_agent_set != 0) {
    return "單獨任務不能設 distinct_agent_set";
  }
  // 區域（任務欄位補齊 05）：政策要是閘道的區域註冊表裡有的；指定了平台時，觀察點不能在那個平台不能進的區域
  //（只有閘道知道的事先擋下：機器人那邊各自再檢查一次，所以這裡不擋樓層、高度上限這些看載具的）
  const ZoneRegistry * zones = comm_.agent().params().zones.get();
  if (e.zone_policy_id != 0 && (zones == nullptr || zones->policy(e.zone_policy_id) == nullptr)) {
    return "zone_policy_id " + std::to_string(e.zone_policy_id) + " 不在閘道的區域註冊表（site_dir）";
  }
  if (zones != nullptr && t.platform != kPlatformAny) {
    AgentState probe;   // 只看平台的區域限制：樓層、高度上限不知道（0）
    probe.type = static_cast<AgentType>(t.platform);
    probe.capabilities = ~Capabilities{0};
    std::string zone;
    const ZoneVerdict v = checkZones(zones, probe, t.platform, t, &zone);
    if (v == ZoneVerdict::ZONE_FORBIDDEN || v == ZoneVerdict::ALTITUDE_LIMIT) {
      return std::string("觀察點（position）在 ") + zone + "：" + platformName(t.platform) + " " +
             (v == ZoneVerdict::ZONE_FORBIDDEN ? "不能進" : "超過天花板") + "（" + toString(v) + "；規劃器要換觀察點）";
    }
  }
  if (e.skip_satisfies && !e.optional) {
    return "skip_satisfies 只用在 optional 角色";
  }
  if (req.spec.size() > wire::kMaxSpecChunk * wire::kMaxSpecParts) {
    return "spec 太大（上限 " + std::to_string(wire::kMaxSpecChunk * wire::kMaxSpecParts) + " B）";
  }
  return checkMeta(req.meta, t.position);
}

namespace
{
// 對照表記住的 meta：incident_id、role 以請求的為準
TaskMeta keptMeta(const InjectRequest & req)
{
  TaskMeta m = req.meta;
  m.incident_id = req.incident_id;
  m.role = req.role;
  return m;
}
}  // namespace

const Gateway::Injected * Gateway::findTask(TaskId task_id) const
{
  const auto key = by_task_.find(task_id);
  if (key == by_task_.end()) {
    return nullptr;
  }
  const auto it = injected_.find(key->second);
  return it != injected_.end() ? &it->second : nullptr;
}

InjectResult Gateway::inject(const InjectRequest & req, double now)
{
  InjectResult r;
  r.revision = req.revision;
  confirmRestored(now);
  const auto key = std::make_pair(req.incident_id, req.role);
  const auto known = injected_.find(key);
  if (known != injected_.end()) {
    r.task_id = known->second.task_id;
    const auto group = groups_.find(req.incident_id);
    r.group_id = group != groups_.end() ? group->second : 0;
    if (known->second.restored) {
      // 重開前注入過，任務還沒從鄰居補回來：現在建新的會變成兩個任務，改版也沒有原本的內容可以比
      char buf[256];
      std::snprintf(buf, sizeof(buf), "閘道剛重開，任務 %s 還在從鄰居補回來（最多再 %.0f s），稍後再送",
        taskName(known->second.task_id).c_str(), std::max(0.0, restored_at_ + config_.restore_confirm - now));
      r.reason = buf;
      r.revision = known->second.revision;
      return r;
    }
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
      const GroupData & data = group_data_[g->second];
      if (data.fallback_template != req.fallback_template) {
        r.reason = "fallback_template 和同組其他角色不同（" + data.fallback_template + "）";
        return r;
      }
      if (req.incident_revision > comm_.incidentRevision(g->second)) {
        r.reason = "incident_revision 比閘道目前的新";
        return r;
      }
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
  t.ext.zone_registry_version = zonesVersion();
  t.id = *id;
  t.ext.group_id = group_id;
  if (!comm_.addLocalTask(t, now, req.spec)) {
    r.reason = "協定層拒收（task_id 重複或 spec 超過上限）";
    return r;
  }
  if (group_id != 0) {
    groups_.emplace(req.incident_id, group_id);
    GroupData & data = group_data_[group_id];
    data.incident_id = req.incident_id;
    data.fallback_template = req.fallback_template;
  }
  Injected entry;
  entry.task_id = t.id;
  entry.revision = t.revision;
  entry.deadline_sec = req.deadline_sec;
  entry.release_sec = req.release_sec;
  entry.expires_sec = req.expires_sec;
  entry.meta = keptMeta(req);
  entry.fallback_template = group_id == 0 ? req.fallback_template : std::string();
  injected_[key] = entry;
  by_task_[t.id] = key;
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
  confirmRestored(now);
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

const char * toString(GroupFailure f)
{
  switch (f) {
    case GroupFailure::CANCELLED: return "必要角色被取消";
    case GroupFailure::EXPIRED: return "必要角色過期";
    case GroupFailure::RECOVERY_TIMEOUT: return "必要角色等不到恢復";
  }
  return "?";
}

const char * toString(RecoveryTrigger t)
{
  switch (t) {
    case RecoveryTrigger::NONE: return "無";
    case RecoveryTrigger::NO_CAPABLE: return "沒有存活的、平台與能力符合的載具";
    case RecoveryTrigger::NO_WINNER: return "沒有得標者也沒有正式指派";
    case RecoveryTrigger::HOLDER_LOST: return "執行者失聯";
  }
  return "?";
}

// 連鎖撤銷（CB-4 第三批，2026-11-16；使用者 11/16 修訂：沒有能做的載具算暫時不完整）
std::vector<GroupEvent> Gateway::stepGroups(double now)
{
  std::vector<GroupEvent> out;
  if (!membershipReady(now) || now < next_group_check_) {
    return out;   // 閘道還不知道全隊有誰（剛啟動、斷線）：不判斷
  }
  next_group_check_ = now + 0.2;
  std::set<TaskId> lost;   // 正式執行者失聯的任務
  for (const auto & [task, holder] : comm_.lockedByLostAgents(now)) {
    lost.insert(task);
  }
  for (auto & [group_id, data] : group_data_) {
    if (data.terminated) {
      continue;
    }
    const std::vector<TaskId> tasks = comm_.groupTasks(group_id);
    std::map<std::uint8_t, TaskId> by_role;
    bool all_or_nothing = false;
    for (const TaskId t : tasks) {
      const Task & task = comm_.agent().tasks().at(t);
      by_role[task.ext.role_index] = t;
      all_or_nothing = all_or_nothing || task.ext.all_or_nothing;
    }
    // 找新的失效（必要角色）
    GroupEvent ev;
    for (const TaskId t : tasks) {
      const Task & task = comm_.agent().tasks().at(t);
      const std::uint8_t role = task.ext.role_index;
      if (task.ext.optional || data.failed_roles.count(role) > 0) {
        continue;
      }
      std::optional<GroupFailure> failure;
      RecoveryTrigger trigger = RecoveryTrigger::NONE;
      if (!comm_.taskOpen(t)) {
        // 確定終止：合法取消、過期 → 立即
        data.recovering.erase(t);
        data.holder_lost.erase(t);
        const auto reason = comm_.closeReason(t);
        if (cascade_cancelled_.count(t) > 0 || reason == wire::kCloseDone) {
          continue;
        }
        failure = reason == wire::kCloseExpired ? GroupFailure::EXPIRED : GroupFailure::CANCELLED;
      } else {
        // 執行者失聯（做法 A，使用者 11/16）：先等原執行者恢復，到失聯接手的時限（takeover_by_priority）由
        // CbbaComm::takeOverLost 撤銷舊指派、重新開放競標，這段期間只標記、不算恢復期限。
        // 撤銷之後才開始算 group_recovery_timeout，觸發原因記 HOLDER_LOST
        if (lost.count(t) > 0) {
          data.holder_lost.emplace(t, now);
          data.recovering.erase(t);
          continue;
        }
        // 暫時不完整：沒有能做的載具 → 沒人接（同時成立時記前面的）
        RecoveryTrigger now_trigger = RecoveryTrigger::NONE;
        const bool assigned = comm_.agent().winner(t) != kNoAgent || isLocking(comm_.assignment(t).state);
        if (comm_.capableAgents(task, now).empty()) {
          now_trigger = RecoveryTrigger::NO_CAPABLE;
        } else if (!assigned) {
          now_trigger = RecoveryTrigger::NO_WINNER;
        }
        const bool after_takeover = data.holder_lost.erase(t) > 0;
        if (now_trigger == RecoveryTrigger::NONE) {
          data.recovering.erase(t);   // 恢復了（原執行者回來，或撤銷後有人接）：繼續執行
          continue;
        }
        if (after_takeover) {
          now_trigger = RecoveryTrigger::HOLDER_LOST;   // 失聯接手撤銷的這一刻開始算
        }
        // 最初的觸發原因保留到恢復為止（中途變成別的原因不改、也不重新計時）
        const auto & rec = data.recovering.emplace(t, GroupData::Recovery{now, now_trigger}).first->second;
        if (now - rec.since >= config_.group_recovery_timeout) {
          failure = GroupFailure::RECOVERY_TIMEOUT;
          trigger = rec.trigger;
        }
      }
      if (failure) {
        if (ev.failed_roles.empty()) {
          ev.reason = *failure;
        }
        ev.failed_roles.push_back(role);
        ev.failed_tasks.push_back(t);
        ev.triggers.push_back(trigger);
        data.recovering.erase(t);
      }
    }
    if (ev.failed_roles.empty()) {
      continue;
    }
    for (const std::uint8_t r : ev.failed_roles) {
      data.failed_roles.insert(r);
    }
    ++data.failure_version;
    // 要取消的角色：all_or_nothing → 整組；否則失效的角色＋依賴它的角色（A 層 precedence，遞移）
    std::set<std::uint8_t> doomed(data.failed_roles.begin(), data.failed_roles.end());
    if (all_or_nothing) {
      for (const auto & [role, t] : by_role) {
        doomed.insert(role);
      }
      data.terminated = true;
    } else {
      bool grew = true;
      while (grew) {
        grew = false;
        for (const auto & [role, t] : by_role) {
          const std::uint8_t pre = comm_.agent().tasks().at(t).ext.precedence;
          if (doomed.count(role) > 0) {
            continue;
          }
          for (const std::uint8_t d : doomed) {
            if ((pre & (1u << d)) != 0) {
              doomed.insert(role);
              grew = true;
              break;
            }
          }
        }
      }
    }
    for (const std::uint8_t role : doomed) {
      const auto it = by_role.find(role);
      if (it == by_role.end() || !comm_.taskOpen(it->second)) {
        continue;
      }
      if (comm_.cancelTask(it->second, now) == ReportCheck::ACCEPTED) {
        cascade_cancelled_.insert(it->second);
        ev.cancelled_roles.push_back(role);
      }
    }
    ev.group_id = group_id;
    ev.incident_id = data.incident_id;
    ev.failure_version = data.failure_version;
    ev.all_or_nothing = all_or_nothing;
    ev.fallback_template = data.fallback_template;
    ev.fallback = !data.fallback_template.empty() && !ev.cancelled_roles.empty();
    ev.incident_revision = comm_.incidentRevision(group_id);
    if (const auto * inc = comm_.incident(group_id)) {
      ev.position_valid = inc->position_valid;
      ev.position = inc->position;
    }
    out.push_back(std::move(ev));
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
  if (old.ext.group_id != 0) {
    // 規劃器依 incident_updates 重新展開後改版（第三批 B）：依據的 incident revision 不能比目前的新；fallback_template 不能改
    if (req.incident_revision > comm_.incidentRevision(old.ext.group_id)) {
      r.reason = "incident_revision 比閘道目前的新";
      return r;
    }
    const auto data = group_data_.find(old.ext.group_id);
    if (data != group_data_.end() && data->second.fallback_template != req.fallback_template) {
      r.reason = "fallback_template 不能改版";
      return r;
    }
  }
  Task t = build(req, old.created, now);
  t.ext.zone_registry_version = zonesVersion();
  t.id = known.task_id;
  t.ext.group_id = old.ext.group_id;
  // 相對時間和上一版一樣：規劃器沒改期限，沿用原本的絕對時刻（不然每次改版期限都往後推，也認不出只動位置的改版）
  if (req.deadline_sec == known.deadline_sec) {
    t.deadline_sec = old.deadline_sec;
    t.explicit_deadline = old.explicit_deadline;
  }
  if (req.release_sec == known.release_sec) {
    t.ext.release = old.ext.release;
  }
  if (req.expires_sec == known.expires_sec) {
    t.ext.expires = old.ext.expires;
  }
  if (!comm_.reviseLocalTask(t, now, req.spec)) {
    r.reason = "協定層拒收改版";
    return r;
  }
  known.revision = req.revision;
  known.deadline_sec = req.deadline_sec;
  known.release_sec = req.release_sec;
  known.expires_sec = req.expires_sec;
  known.meta = keptMeta(req);
  if (old.ext.group_id == 0) {
    known.fallback_template = req.fallback_template;
  }
  r.status = InjectStatus::REVISED;
  r.revision = req.revision;
  return r;
}

// ===========================================================================
// 對照表存檔（任務欄位補齊 01，2026-11-17）
// ===========================================================================
// 格式（文字，一行一筆，欄位以空白分隔；字串用 %XX 編碼，空字串寫成 -）：
//   cbba_gateway_state 1
//   i <incident> <role> <task_id> <revision> <deadline_sec> <release_sec> <expires_sec> <source> <event_type>
//     <event_type_id> <task_type> <task_type_id> <role_id> [<fallback_template>]（最後一欄 2026-11-18 加，單獨任務的）
//   g <incident> <group_id> <failure_version> <terminated 0/1> <fallback_template> <failed_roles：逗號分隔或 ->
//   c <task_id>                                   連鎖撤銷取消過的
namespace
{
constexpr const char * kStateMagic = "cbba_gateway_state";
constexpr int kStateVersion = 1;

std::string enc(const std::string & s)
{
  if (s.empty()) {
    return "-";
  }
  std::string out;
  for (const char ch : s) {
    const auto c = static_cast<unsigned char>(ch);
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' || c == '.' ||
      c == ':' || c == '/')
    {
      out += ch;
    } else {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "%%%02X", static_cast<unsigned>(c));
      out += buf;
    }
  }
  return out;
}

std::optional<std::string> dec(const std::string & s)
{
  if (s == "-") {
    return std::string();
  }
  std::string out;
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] != '%') {
      out += s[i];
      continue;
    }
    if (i + 2 >= s.size()) {
      return std::nullopt;
    }
    const std::string hex = s.substr(i + 1, 2);
    if (hex.size() != 2 || hex.find_first_not_of("0123456789ABCDEFabcdef") != std::string::npos) {
      return std::nullopt;
    }
    out += static_cast<char>(std::stoi(hex, nullptr, 16));
    i += 2;
  }
  return out;
}

std::string num(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  return buf;
}
}  // namespace

std::string Gateway::saveState() const
{
  std::ostringstream out;
  out << kStateMagic << " " << kStateVersion << "\n";
  for (const auto & [key, e] : injected_) {
    const TaskMeta & m = e.meta;
    out << "i " << enc(key.first) << " " << enc(key.second) << " " << e.task_id << " " << e.revision << " " <<
      num(e.deadline_sec) << " " << num(e.release_sec) << " " << num(e.expires_sec) << " " <<
      static_cast<unsigned>(m.source) << " " << enc(m.event_type) << " " << m.event_type_id << " " <<
      enc(m.task_type) << " " << m.task_type_id << " " << m.role_id << " " << enc(e.fallback_template) << "\n";
  }
  for (const auto & [incident, group_id] : groups_) {
    const auto data = group_data_.find(group_id);
    if (data == group_data_.end()) {
      continue;
    }
    std::string roles;
    for (const std::uint8_t r : data->second.failed_roles) {
      roles += (roles.empty() ? "" : ",") + std::to_string(r);
    }
    out << "g " << enc(incident) << " " << group_id << " " << data->second.failure_version << " " <<
      (data->second.terminated ? 1 : 0) << " " << enc(data->second.fallback_template) << " " <<
      (roles.empty() ? "-" : roles) << "\n";
  }
  for (const TaskId t : cascade_cancelled_) {
    out << "c " << t << "\n";
  }
  return out.str();
}

bool Gateway::loadState(const std::string & text, double now, std::string * error)
{
  const auto fail = [error](const std::string & why) {
      if (error != nullptr) {
        *error = why;
      }
      return false;
    };
  std::istringstream in(text);
  std::string line;
  if (!std::getline(in, line)) {
    return fail("空檔案");
  }
  {
    std::istringstream head(line);
    std::string magic;
    int version = 0;
    if (!(head >> magic >> version) || magic != kStateMagic || version != kStateVersion) {
      return fail("第一行不是 " + std::string(kStateMagic) + " " + std::to_string(kStateVersion));
    }
  }
  std::map<std::pair<std::string, std::string>, Injected> injected;
  std::map<TaskId, std::pair<std::string, std::string>> by_task;
  std::map<std::string, TaskId> groups;
  std::map<TaskId, GroupData> group_data;
  std::set<TaskId> cascade;
  int n = 1;
  while (std::getline(in, line)) {
    ++n;
    if (line.empty()) {
      continue;
    }
    std::istringstream ls(line);
    std::string kind;
    ls >> kind;
    const std::string where = "第 " + std::to_string(n) + " 行";
    if (kind == "i") {
      std::string incident, role, event_type, task_type;
      Injected e;
      unsigned source = 0, revision = 0, event_id = 0, task_type_id = 0, role_id = 0;
      if (!(ls >> incident >> role >> e.task_id >> revision >> e.deadline_sec >> e.release_sec >> e.expires_sec >>
        source >> event_type >> event_id >> task_type >> task_type_id >> role_id) || revision > 0xFFFFu ||
        source > kMaxTaskSource || event_id > 0xFFFFu || task_type_id > 0xFFFFu || role_id > 0xFFFFu)
      {
        return fail(where + "（i）欄位不對");
      }
      const auto inc = dec(incident), rl = dec(role), ev = dec(event_type), tt = dec(task_type);
      if (!inc || !rl || !ev || !tt || inc->empty()) {
        return fail(where + "（i）字串編碼不對");
      }
      e.revision = static_cast<std::uint16_t>(revision);
      e.meta.incident_id = *inc;
      e.meta.role = *rl;
      e.meta.source = static_cast<TaskSource>(source);
      e.meta.event_type = *ev;
      e.meta.event_type_id = static_cast<std::uint16_t>(event_id);
      e.meta.task_type = *tt;
      e.meta.task_type_id = static_cast<std::uint16_t>(task_type_id);
      e.meta.role_id = static_cast<std::uint16_t>(role_id);
      std::string fallback;
      if (ls >> fallback) {   // 11/17 的檔案沒有這一欄
        const auto fb = dec(fallback);
        if (!fb) {
          return fail(where + "（i）fallback_template 編碼不對");
        }
        e.fallback_template = *fb;
      }
      e.restored = true;
      const auto key = std::make_pair(*inc, *rl);
      by_task[e.task_id] = key;
      injected[key] = e;
    } else if (kind == "g") {
      std::string incident, fallback, roles;
      TaskId group_id = 0;
      unsigned terminated = 0;
      GroupData d;
      if (!(ls >> incident >> group_id >> d.failure_version >> terminated >> fallback >> roles) || terminated > 1) {
        return fail(where + "（g）欄位不對");
      }
      const auto inc = dec(incident), fb = dec(fallback);
      if (!inc || !fb || inc->empty() || group_id == 0) {
        return fail(where + "（g）字串編碼不對");
      }
      if (roles != "-") {
        std::istringstream rs(roles);
        std::string r;
        while (std::getline(rs, r, ',')) {
          if (r.empty() || r.find_first_not_of("0123456789") != std::string::npos || std::stoul(r) > 255) {
            return fail(where + "（g）failed_roles 不對");
          }
          d.failed_roles.insert(static_cast<std::uint8_t>(std::stoul(r)));
        }
      }
      d.incident_id = *inc;
      d.fallback_template = *fb;
      d.terminated = terminated == 1;
      d.restored = true;
      groups[*inc] = group_id;
      group_data[group_id] = d;
    } else if (kind == "c") {
      TaskId t = 0;
      if (!(ls >> t)) {
        return fail(where + "（c）欄位不對");
      }
      cascade.insert(t);
    } else {
      return fail(where + "：不認得的種類 " + kind);
    }
  }
  injected_ = std::move(injected);
  by_task_ = std::move(by_task);
  groups_ = std::move(groups);
  group_data_ = std::move(group_data);
  cascade_cancelled_ = std::move(cascade);
  restored_at_ = injected_.empty() && group_data_.empty() ? -1.0 : now;
  confirmRestored(now);
  return true;
}

void Gateway::confirmRestored(double now)
{
  if (restored_at_ < 0.0) {
    return;
  }
  const auto & tasks = comm_.agent().tasks();
  const bool expired = now - restored_at_ >= config_.restore_confirm;
  bool pending = false;
  for (auto it = injected_.begin(); it != injected_.end(); ) {
    Injected & e = it->second;
    if (!e.restored) {
      ++it;
      continue;
    }
    if (tasks.count(e.task_id) > 0) {
      e.restored = false;   // 鄰居補回來了：之後照一般的冪等、改版
      ++it;
    } else if (expired) {
      purged_.push_back(it->first);   // 全隊都不記得這個任務了（整隊重開）
      by_task_.erase(e.task_id);
      it = injected_.erase(it);
    } else {
      pending = true;
      ++it;
    }
  }
  for (auto it = group_data_.begin(); it != group_data_.end(); ) {
    GroupData & d = it->second;
    if (!d.restored) {
      ++it;
      continue;
    }
    if (!comm_.groupTasks(it->first).empty()) {
      d.restored = false;
      ++it;
    } else if (expired) {
      groups_.erase(d.incident_id);
      it = group_data_.erase(it);
    } else {
      pending = true;
      ++it;
    }
  }
  if (!pending) {
    restored_at_ = -1.0;
  }
}

std::vector<std::pair<std::string, std::string>> Gateway::takePurged()
{
  std::vector<std::pair<std::string, std::string>> out;
  out.swap(purged_);
  return out;
}

std::uint16_t Gateway::zonesVersion() const
{
  const ZoneRegistry * zones = comm_.agent().params().zones.get();
  return zones != nullptr ? zones->version : 0;
}

// ===========================================================================
// 單獨 P0 任務的看門狗（任務欄位補齊 07，2026-11-18）
// ===========================================================================
std::vector<UnassignedP0> Gateway::stepWatchdog(double now)
{
  std::vector<UnassignedP0> out;
  if (config_.p0_unassigned_timeout <= 0.0) {
    return out;
  }
  std::set<TaskId> watched;
  const auto & agent = comm_.agent();
  for (const auto & [id, task] : agent.tasks()) {
    if (task.status != TaskStatus::OPEN || task.ext.group_id != 0 || priorityLevel(task) != 0 ||
      taskCreator(id) != comm_.id())
    {
      continue;
    }
    if (task.ext.release > now) {
      continue;   // 還不能開始：沒人接是正常的
    }
    if (agent.winner(id) != kNoAgent || comm_.assignment(id).assignee != kNoAgent) {
      p0_reported_.erase(id);   // 有人了：下次再沒人時重新計時、可以再報
      continue;
    }
    watched.insert(id);
    const double since = p0_unassigned_since_.emplace(id, now).first->second;
    if (now - since < config_.p0_unassigned_timeout || p0_reported_.count(id) > 0) {
      continue;
    }
    p0_reported_.insert(id);
    UnassignedP0 ev;
    ev.task = id;
    ev.waited = now - since;
    if (const Injected * known = findTask(id)) {
      ev.incident_id = known->meta.incident_id;
      ev.fallback_template = known->fallback_template;
    }
    out.push_back(ev);
  }
  for (auto it = p0_unassigned_since_.begin(); it != p0_unassigned_since_.end(); ) {
    it = watched.count(it->first) > 0 ? std::next(it) : p0_unassigned_since_.erase(it);
  }
  return out;
}

}  // namespace cbba_core
