#include "cbba_core/assignment_manager.hpp"

#include <algorithm>
#include <cstdio>

namespace cbba_core
{

AssignmentManager::AssignmentManager(CbbaComm & comm, const AssignmentConfig & config, Logger log)
: comm_(comm), config_(config), log_(std::move(log))
{
}

std::optional<HeldAssignment> AssignmentManager::pending() const
{
  if (!pending_) {
    return std::nullopt;
  }
  return HeldAssignment{pending_->task, pending_->version};
}

std::string AssignmentManager::name(TaskId task, std::uint64_t version) const
{
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%s v%llu", taskName(task).c_str(),
    static_cast<unsigned long long>(version));
  return buf;
}

bool AssignmentManager::inPath(TaskId task) const
{
  const auto & path = comm_.agent().path();
  return std::find(path.begin(), path.end(), task) != path.end();
}

std::vector<AssignmentRequest> AssignmentManager::takeRequests()
{
  std::vector<AssignmentRequest> out;
  out.swap(requests_);
  return out;
}

bool AssignmentManager::takeAssignedChanged()
{
  const bool changed = assigned_changed_;
  assigned_changed_ = false;
  return changed;
}

std::vector<AssignmentTiming> AssignmentManager::takeTimings()
{
  std::vector<AssignmentTiming> out;
  out.swap(finished_timings_);
  return out;
}

// ===========================================================================
// BT 的執行狀態
// ===========================================================================
void AssignmentManager::setExec(const ExecInput & exec, double now)
{
  exec_ = exec;
  // 暫時拒絕過的任務：BT 的狀態改變（例如走到可以中斷的地方）時重新評估。
  // 先套用新的限制（例如故障時不接新任務），再恢復出價
  const auto signature = std::make_tuple(exec.fresh, exec.execution_state, exec.preemptible,
      exec.has_active, exec.active_task);
  if (signature != exec_signature_) {
    exec_signature_ = signature;
    updateConstraints(now);
    comm_.resumeSuspended(now);
  }
  // 量測：BT 回報開始執行這個任務、這個版本
  if (exec.fresh && exec.has_active) {
    const auto it = timings_.find({exec.active_task, exec.active_version});
    if (it != timings_.end() && it->second.running < 0.0) {
      it->second.running = now;
      finished_timings_.push_back(it->second);
      timings_.erase(it);
    }
  }
}

void AssignmentManager::updateConstraints(double now)
{
  ExecConstraints c;
  const bool no_bt = !config_.require_accept && !exec_.fresh;   // 無人機：還沒有 BT 的執行狀態
  c.accept_new = no_bt || (exec_.fresh && exec_.execution_state != kExecFault);
  if (active_) {
    c.pinned.push_back(active_->task);
  }
  if (reserved_) {
    c.pinned.push_back(reserved_->task);
  }
  c.active_pinned = active_.has_value();
  c.preemptible = no_bt || (exec_.fresh && exec_.preemptible);
  c.active_remaining = exec_.fresh ? exec_.remaining_time : -1.0;
  c.max_queued_fires = config_.max_queued_fires;
  c.max_queue_wait = config_.max_queue_wait;
  comm_.setExecConstraints(c, now);
  comm_.setExecReport(ExecReport{exec_.fresh, exec_.execution_state, exec_.preemptible,
      exec_.remaining_time});
}

// ===========================================================================
// 每個 tick
// ===========================================================================
void AssignmentManager::step(double now)
{
  checkHeld(now);

  // 待接受逾時：當成暫時拒絕；送 RELEASE，之後才到的接受回覆不會啟動任務
  if (pending_ && now - pending_->sent > config_.accept_timeout) {
    const Pending p = *pending_;
    warn("指派請求 " + name(p.task, p.version) + " 逾時（" +
      std::to_string(static_cast<int>(config_.accept_timeout * 1000)) + " ms 沒有回覆）：當成暫時拒絕");
    comm_.endAssignment(p.task);
    comm_.suspendTask(p.task, now);
    release(p.task, p.version);
    pending_.reset();
    timings_.erase({p.task, p.version});
  }

  // CBBA 已經不要的：執行中的任務不在路徑上（電量不足、停止參與）→ 撤銷；保留的同樣處理
  if (active_ && !inPath(active_->task)) {
    warn("執行中的 " + name(active_->task, active_->version) + " 已不在路徑上（電量、停止參與）：撤銷，任務保留");
    comm_.endAssignment(active_->task);
    active_.reset();
    assigned_changed_ = true;
  }
  if (reserved_ && !inPath(reserved_->task)) {
    warn("保留的 " + name(reserved_->task, reserved_->version) + " 已不在路徑上（期限、電量）：撤銷，任務保留");
    comm_.endAssignment(reserved_->task);
    release(reserved_->task, reserved_->version);
    reserved_.reset();
  }

  const auto current = comm_.currentTask();
  const TaskId head = current ? current->id : 0;
  if (head != candidate_) {
    candidate_ = head;
    candidate_since_ = now;
  }

  // 待接受的請求，CBBA 已經不要了（不再排第一、不在路徑上）→ 作廢
  if (pending_) {
    const bool wanted = pending_->kind == RequestKind::ACTIVATE ? pending_->task == head :
      inPath(pending_->task);
    if (!wanted) {
      abandonPending("CBBA 的分配已經改變");
    }
  }

  const bool stable = head != 0 && now - candidate_since_ >= config_.assign_hold;
  const TaskId active_task = active_ ? active_->task : 0;
  if (!pending_ && stable && head != active_task) {
    if (reserved_ && head == reserved_->task && !active_) {
      // 保留的排到第一：核心已經重新檢查過可行性（期限、電量），轉成執行中，不再問 BT
      const HeldAssignment r = *reserved_;
      if (comm_.confirmAssignment(r.task, r.version, AssignState::ACTIVE)) {
        active_ = r;
        active_since_ = now;
        reserved_.reset();
        assigned_changed_ = true;
        info("保留的 " + name(r.task, r.version) + " 開始執行");
      } else {
        warn("保留的 " + name(r.task, r.version) + " 已被取代，不執行");
        reserved_.reset();
      }
    } else {
      // 新的、或搶占（核心只在可以安全中斷、優先級較高時才把它排到執行中任務的前面）
      startRequest(RequestKind::ACTIVATE, head, active_.has_value(), candidate_since_, now);
    }
  }

  // 排隊：執行中任務之後的第一個火警（核心已檢查數量與期限），一樣經過接受，變成保留
  TaskId queued = 0;
  if (active_) {
    const auto & path = comm_.agent().path();
    for (std::size_t i = 1; i < path.size(); ++i) {
      const auto it = comm_.agent().tasks().find(path[i]);
      if (it != comm_.agent().tasks().end() && taskPriority(it->second.type) >= kPriorityFire) {
        queued = path[i];
        break;
      }
    }
  }
  if (queued != queue_candidate_) {
    queue_candidate_ = queued;
    queue_since_ = now;
  }
  if (!pending_ && active_ && !reserved_ && queued != 0 &&
    now - queue_since_ >= config_.assign_hold)
  {
    startRequest(RequestKind::RESERVE, queued, false, queue_since_, now);
  }

  // BT 回報的和自己的不一致（超過 1 s）：只警告，以 CBBA 的正式指派為準
  if (exec_.fresh && active_ && now - active_since_ > 1.0 &&
    (!exec_.has_active || exec_.active_task != active_->task ||
    exec_.active_version != active_->version) && now - last_mismatch_warn_ > 5.0)
  {
    last_mismatch_warn_ = now;
    warn("BT 回報的執行中任務和正式指派 " + name(active_->task, active_->version) + " 不一致");
  }

  updateConstraints(now);
}

// 持有的指派還有效嗎：任務結束、回報完成、被更新的版本取代 → 不再持有
void AssignmentManager::checkHeld(double /*now*/)
{
  const AgentId me = comm_.id();
  const auto valid = [&](const HeldAssignment & h, AssignState state) {
      const Assignment a = comm_.assignment(h.task);
      return comm_.taskOpen(h.task) && a.assignee == me && a.version == h.version && a.state == state;
    };
  if (active_) {
    if (!comm_.taskOpen(active_->task) || comm_.completionPending(active_->task)) {
      active_.reset();   // 結束了（完成、取消），或已回報完成、在等全隊確認
      assigned_changed_ = true;
    } else if (!valid(*active_, AssignState::ACTIVE)) {
      warn("執行中的 " + name(active_->task, active_->version) + " 被更新的正式指派取代");
      active_.reset();
      assigned_changed_ = true;
    }
  }
  if (reserved_ && !valid(*reserved_, AssignState::RESERVED)) {
    if (comm_.taskOpen(reserved_->task)) {
      warn("保留的 " + name(reserved_->task, reserved_->version) + " 被更新的正式指派取代");
    }
    release(reserved_->task, reserved_->version);
    reserved_.reset();
  }
  if (pending_ && !valid(HeldAssignment{pending_->task, pending_->version}, AssignState::PENDING)) {
    abandonPending("請求已被取代或任務已結束");
  }
}

void AssignmentManager::startRequest(
  RequestKind kind, TaskId task, bool preempt, double head_since, double now)
{
  const auto it = comm_.agent().tasks().find(task);
  if (it == comm_.agent().tasks().end()) {
    return;
  }
  if (reserved_ && reserved_->task == task) {
    reserved_.reset();   // 保留的火警要搶占（BT 走到可以中斷的地方）：保留換成新的執行請求
  }
  const std::uint64_t version = comm_.beginAssignment(task);
  Pending p{kind, preempt, task, version, now};
  AssignmentTiming t;
  t.task = task;
  t.version = version;
  t.preempt = preempt;
  t.created = it->second.created;
  t.head = head_since;
  t.request = now;
  timings_[{task, version}] = t;

  if (!config_.require_accept) {
    accept(p, now);   // 不用 BT 接受：當下就算接受
    return;
  }
  pending_ = p;
  requests_.push_back(AssignmentRequest{kind, preempt, it->second, version});
  info(std::string(kind == RequestKind::ACTIVATE ? (preempt ? "請 BT 接受（中斷目前的任務）：" :
    "請 BT 接受：") : "請 BT 保留（排隊）：") + name(task, version));
}

void AssignmentManager::accept(const Pending & p, double now)
{
  const auto timing = timings_.find({p.task, p.version});
  if (timing != timings_.end()) {
    timing->second.accept = now;
  }
  if (p.kind == RequestKind::RESERVE) {
    if (comm_.confirmAssignment(p.task, p.version, AssignState::RESERVED)) {
      reserved_ = HeldAssignment{p.task, p.version};
      info("保留 " + name(p.task, p.version));
    }
    timings_.erase({p.task, p.version});   // 只量測開始執行的
    return;
  }
  if (active_ && active_->task != p.task) {
    // 搶占：被中斷的任務結束這次指派、任務保留（重新開放競標），不送 TASK_CLOSE
    info("中斷 " + name(active_->task, active_->version) + "（任務保留，重新開放競標）");
    comm_.endAssignment(active_->task);
    active_.reset();
  }
  if (!comm_.confirmAssignment(p.task, p.version, AssignState::ACTIVE)) {
    warn("接受的 " + name(p.task, p.version) + " 已被取代，不執行");
    timings_.erase({p.task, p.version});
    assigned_changed_ = true;
    return;
  }
  active_ = HeldAssignment{p.task, p.version};
  active_since_ = now;
  if (reserved_ && reserved_->task == p.task) {
    reserved_.reset();
  }
  assigned_changed_ = true;
  if (timing != timings_.end()) {
    timing->second.assigned = now;
    if (!config_.require_accept && !exec_.fresh) {
      finished_timings_.push_back(timing->second);   // 沒有 BT 回報：量到發布為止
      timings_.erase(timing);
    }
  }
}

void AssignmentManager::release(TaskId task, std::uint64_t version)
{
  if (!config_.require_accept) {
    return;
  }
  AssignmentRequest r;
  r.kind = RequestKind::RELEASE;
  r.version = version;
  const auto it = comm_.agent().tasks().find(task);
  if (it != comm_.agent().tasks().end()) {
    r.task = it->second;
  } else {
    r.task.id = task;
  }
  requests_.push_back(r);
}

void AssignmentManager::abandonPending(const std::string & why)
{
  const Pending p = *pending_;
  info("指派請求 " + name(p.task, p.version) + " 作廢：" + why);
  comm_.endAssignment(p.task);
  release(p.task, p.version);
  pending_.reset();
  timings_.erase({p.task, p.version});
}

// ===========================================================================
// BT 的回覆與結果
// ===========================================================================
void AssignmentManager::onResponse(
  TaskId task, std::uint64_t version, bool accepted, RejectReason reason, const std::string & detail,
  double now)
{
  if (!pending_ || pending_->task != task || pending_->version != version) {
    warn("BT 對 " + name(task, version) + " 的回覆不是目前在等的請求（已逾時、作廢或不存在）：忽略");
    return;
  }
  const Pending p = *pending_;
  pending_.reset();
  if (accepted) {
    accept(p, now);
    return;
  }
  comm_.endAssignment(task);
  timings_.erase({task, version});
  const std::string why = detail.empty() ? "" : "（" + detail + "）";
  if (reason == RejectReason::PERMANENT) {
    warn("BT 拒絕 " + name(task, version) + why + "：永久，不再出價");
    comm_.excludeTask(task, now);
  } else {
    warn("BT 拒絕 " + name(task, version) + why + "：暫時，撤回出價，BT 狀態改變時重新評估");
    comm_.suspendTask(task, now);
  }
}

ReportCheck AssignmentManager::onResult(TaskId task, std::uint64_t version, bool success, double now)
{
  if (!active_ || active_->task != task || active_->version != version) {
    return ReportCheck::STALE;
  }
  const ReportCheck check = comm_.reportResult(task, success, version, now);
  if (check == ReportCheck::ACCEPTED) {
    active_.reset();
    assigned_changed_ = true;
  }
  return check;
}

}  // namespace cbba_core
