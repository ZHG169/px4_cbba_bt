#include "cbba_core/approval.hpp"

#include <algorithm>

namespace cbba_core
{

const char * toString(ApprovalState s)
{
  switch (s) {
    case ApprovalState::PROPOSED: return "PROPOSED";
    case ApprovalState::WAITING_APPROVAL: return "WAITING_APPROVAL";
    case ApprovalState::APPROVED: return "APPROVED";
    case ApprovalState::INJECTED: return "INJECTED";
    case ApprovalState::REJECTED: return "REJECTED";
    case ApprovalState::EXPIRED: return "EXPIRED";
  }
  return "?";
}

void ApprovalQueue::set(ApprovalRecord & r, ApprovalState s)
{
  r.state = s;
  changes_.push_back({r.id, s});
}

ApprovalQueue::Proposed ApprovalQueue::propose(const InjectRequest & request, const std::vector<std::string> & reasons,
  double now)
{
  const auto key = std::make_tuple(request.incident_id, request.role, request.revision);
  if (const auto it = by_key_.find(key); it != by_key_.end()) {
    return Proposed{it->second, true};
  }
  ApprovalRecord r;
  r.id = next_id_++;
  r.request = request;
  r.reasons = reasons;
  r.proposed_at = now;
  auto & stored = records_[r.id] = r;
  by_key_[key] = r.id;
  set(stored, ApprovalState::PROPOSED);
  set(stored, ApprovalState::WAITING_APPROVAL);
  return Proposed{r.id, false};
}

std::vector<std::uint32_t> ApprovalQueue::decide(std::uint32_t id, const std::string & incident_id, bool approve,
  const std::string & operator_name, const std::string & note, double now, std::string * error)
{
  std::vector<std::uint32_t> out;
  const auto fail = [&](const std::string & why) {
      if (error != nullptr) {
        *error = why;
      }
      return out;
    };
  if (operator_name.empty()) {
    return fail("要給 operator_name（核准紀錄要知道是誰）");
  }
  for (auto & [rid, r] : records_) {
    if ((id != 0 && rid != id) || (id == 0 && r.request.incident_id != incident_id)) {
      continue;
    }
    if (r.state != ApprovalState::WAITING_APPROVAL) {
      if (id != 0) {
        return fail("核准單 " + std::to_string(id) + " 已經是 " + toString(r.state));
      }
      continue;
    }
    r.operator_name = operator_name;
    r.note = note;
    r.decided_at = now;
    set(r, approve ? ApprovalState::APPROVED : ApprovalState::REJECTED);
    out.push_back(rid);
  }
  if (out.empty()) {
    return fail(id != 0 ? "沒有核准單 " + std::to_string(id) : "事件 " + incident_id + " 沒有在等核准的");
  }
  return out;
}

std::optional<InjectRequest> ApprovalQueue::injectable(std::uint32_t id, double now, std::string * why) const
{
  const ApprovalRecord * r = find(id);
  if (r == nullptr || r->state != ApprovalState::APPROVED) {
    if (why != nullptr) {
      *why = "不是已核准的";
    }
    return std::nullopt;
  }
  InjectRequest req = r->request;
  const double waited = now - r->proposed_at;
  const auto expired = [&](double & rel, const char * name) {
      if (rel <= 0.0) {
        return false;   // 0 = 沒有
      }
      rel -= waited;
      if (rel > 0.0) {
        return false;
      }
      if (why != nullptr) {
        *why = std::string("等核准時已過 ") + name;
      }
      return true;
    };
  if (expired(req.expires_sec, "expires") || (req.task.ext.deadline_hard && expired(req.deadline_sec, "硬期限"))) {
    return std::nullopt;
  }
  if (!req.task.ext.deadline_hard && req.deadline_sec > 0.0) {
    req.deadline_sec = std::max(req.deadline_sec - waited, 1.0);   // 軟期限過了也照做（不影響可行性）
  }
  if (req.release_sec > 0.0) {
    req.release_sec = std::max(req.release_sec - waited, 0.0);
  }
  return req;
}

void ApprovalQueue::markInjected(std::uint32_t id, const InjectResult & result, double now)
{
  auto it = records_.find(id);
  if (it == records_.end()) {
    return;
  }
  it->second.task_id = result.task_id;
  it->second.group_id = result.group_id;
  it->second.decided_at = now;
  set(it->second, ApprovalState::INJECTED);
}

void ApprovalQueue::markFailed(std::uint32_t id, ApprovalState state, const std::string & why, double now)
{
  auto it = records_.find(id);
  if (it == records_.end()) {
    return;
  }
  it->second.note = it->second.note.empty() ? why : it->second.note + "；" + why;
  it->second.decided_at = now;
  set(it->second, state);
}

void ApprovalQueue::expire(double now)
{
  for (auto & [id, r] : records_) {
    if (r.state == ApprovalState::WAITING_APPROVAL && r.request.expires_sec > 0.0 &&
      now - r.proposed_at >= r.request.expires_sec)
    {
      r.note = "等核准時已過 expires";
      r.decided_at = now;
      set(r, ApprovalState::EXPIRED);
    }
  }
}

const ApprovalRecord * ApprovalQueue::find(std::uint32_t id) const
{
  const auto it = records_.find(id);
  return it != records_.end() ? &it->second : nullptr;
}

std::vector<std::uint32_t> ApprovalQueue::waiting() const
{
  std::vector<std::uint32_t> out;
  for (const auto & [id, r] : records_) {
    if (r.state == ApprovalState::WAITING_APPROVAL) {
      out.push_back(id);
    }
  }
  return out;
}

std::vector<std::pair<std::uint32_t, ApprovalState>> ApprovalQueue::takeChanges()
{
  std::vector<std::pair<std::uint32_t, ApprovalState>> out;
  out.swap(changes_);
  return out;
}

}  // namespace cbba_core
