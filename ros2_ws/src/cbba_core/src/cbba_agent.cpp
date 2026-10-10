#include "cbba_core/cbba_agent.hpp"

#include <algorithm>
#include <cmath>

namespace cbba_core
{

namespace
{
double stampOf(const std::map<AgentId, double> * stamps, AgentId agent)
{
  if (stamps == nullptr) {
    return -kInf;
  }
  const auto it = stamps->find(agent);
  return it == stamps->end() ? -kInf : it->second;
}
}  // namespace

// ---------------------------------------------------------------------------
// 消解規則表：Choi, Brunet, How (2009) Table 1
//   i = 接收者（me）、k = 傳送者、m / n = 其他載具
// ---------------------------------------------------------------------------
RuleResult resolveConflict(const RuleInput & in)
{
  const AgentId i = in.me;
  const AgentId k = in.sender;
  const AgentId zk = in.sender_winner;
  const AgentId zi = in.my_winner;

  // 「k 對 m 的資訊較新」
  const auto sender_newer = [&in](AgentId m) {
      return stampOf(in.sender_stamps, m) > stampOf(in.my_stamps, m);
    };
  const auto mine_newer = [&in](AgentId m) {
      return stampOf(in.my_stamps, m) > stampOf(in.sender_stamps, m);
    };
  // 「k 的分數較高」：平手時得標者編號小的優先
  const double diff = in.sender_score - in.my_score;
  const bool sender_higher =
    diff > in.epsilon || (std::abs(diff) <= in.epsilon && zk < zi);

  const auto result = [](RuleAction action, int rule) {return RuleResult{action, rule};};
  const auto update_if = [&result](bool cond, int rule, RuleAction otherwise = RuleAction::Leave) {
      return result(cond ? RuleAction::Update : otherwise, rule);
    };

  if (zk == k) {                      // 傳送者認為自己得標
    if (zi == i) {return update_if(sender_higher, 1);}
    if (zi == k) {return result(RuleAction::Update, 2);}
    if (zi == kNoAgent) {return result(RuleAction::Update, 4);}
    return update_if(sender_newer(zi) || sender_higher, 3);
  }

  if (zk == i) {                      // 傳送者認為我得標
    if (zi == i) {return result(RuleAction::Leave, 5);}
    if (zi == k) {return result(RuleAction::Reset, 6);}
    if (zi == kNoAgent) {return result(RuleAction::Leave, 8);}
    return result(sender_newer(zi) ? RuleAction::Reset : RuleAction::Leave, 7);
  }

  if (zk == kNoAgent) {               // 傳送者認為無人得標
    if (zi == i) {return result(RuleAction::Leave, 14);}
    if (zi == k) {return result(RuleAction::Update, 15);}
    if (zi == kNoAgent) {return result(RuleAction::Leave, 17);}
    return update_if(sender_newer(zi), 16);
  }

  // 傳送者認為第三方 m 得標
  const AgentId m = zk;
  if (zi == i) {return update_if(sender_newer(m) && sender_higher, 9);}
  if (zi == k) {return update_if(sender_newer(m), 10, RuleAction::Reset);}
  if (zi == m) {return update_if(sender_newer(m), 11);}
  if (zi == kNoAgent) {return update_if(sender_newer(m), 13);}

  // 規則 12：我認為另一個第三方 n 得標（時間戳相等時依論文為「不變」）
  const AgentId n = zi;
  if (sender_newer(m) && sender_newer(n)) {return result(RuleAction::Update, 12);}
  if (sender_newer(m) && sender_higher) {return result(RuleAction::Update, 12);}
  if (sender_newer(n) && mine_newer(m)) {return result(RuleAction::Reset, 12);}
  return result(RuleAction::Leave, 12);
}

// ---------------------------------------------------------------------------
// CbbaAgent
// ---------------------------------------------------------------------------
CbbaAgent::CbbaAgent(const AgentState & state, const ScoringParams & params)
: state_(state), params_(params)
{
}

AgentId CbbaAgent::winner(TaskId id) const
{
  const auto it = z_.find(id);
  return it == z_.end() ? kNoAgent : it->second;
}

double CbbaAgent::score(TaskId id) const
{
  const auto it = y_.find(id);
  return it == y_.end() ? 0.0 : it->second;
}

bool CbbaAgent::isOpen(TaskId id) const
{
  const auto it = tasks_.find(id);
  return it != tasks_.end() && it->second.status == TaskStatus::OPEN;
}

std::vector<Task> CbbaAgent::pathTasks(const std::vector<TaskId> & ids) const
{
  std::vector<Task> out;
  out.reserve(ids.size());
  for (TaskId id : ids) {
    out.push_back(tasks_.at(id));
  }
  return out;
}

// 連鎖退標：釋放 bundle[bundle_index] 和它之後加入的所有任務
bool CbbaAgent::releaseFrom(std::size_t bundle_index)
{
  if (bundle_index >= bundle_.size()) {
    return false;
  }
  for (std::size_t b = bundle_index; b < bundle_.size(); ++b) {
    const TaskId id = bundle_[b];
    if (winner(id) == state_.id) {   // 被連帶釋放的任務：得標者設為無人
      z_[id] = kNoAgent;
      y_[id] = 0.0;
    }
    path_.erase(std::remove(path_.begin(), path_.end(), id), path_.end());
  }
  bundle_.resize(bundle_index);
  return true;
}

// 找出 bundle 中第一個已經不屬於自己（或已終止）的任務，從它開始釋放
bool CbbaAgent::releaseLost()
{
  for (std::size_t b = 0; b < bundle_.size(); ++b) {
    const TaskId id = bundle_[b];
    if (winner(id) != state_.id || !isOpen(id)) {
      return releaseFrom(b);
    }
  }
  return false;
}

// 持續加入任務，直到沒有能贏的為止。
//
// 出價上限（bid warping，Johnson et al. 2012）：
//   插入式的邊際分數不保證「接的任務越多、分數越低」，例如新任務剛好在既有路徑附近時，
//   分數反而會變高，這會讓 CBBA 無法收斂。做法是把新任務的出價壓在 bundle 中既有出價
//   的最小值以下，出價就一定隨 bundle 變長而不增，收斂性得以保證。
bool CbbaAgent::buildBundle(double now)
{
  if (!active_ || !exec_.accept_new) {
    return false;
  }
  bool changed = false;
  while (bundle_.size() < params_.max_bundle) {
    const std::vector<Task> current_path = pathTasks(path_);

    double bid_cap = kInf;
    for (TaskId held : bundle_) {
      bid_cap = std::min(bid_cap, score(held));
    }

    TaskId best_id = 0;
    Insertion best;
    double best_bid = 0.0;

    for (const auto & [id, task] : tasks_) {
      if (task.status != TaskStatus::OPEN) {continue;}
      if (excluded_.count(id) > 0) {continue;}   // 自己做失敗的任務不再出價
      if (suspended_.count(id) > 0) {continue;}  // BT 暫時拒絕
      if (locked_.count(id) > 0) {continue;}     // 別台已接受
      if (std::find(bundle_.begin(), bundle_.end(), id) != bundle_.end()) {continue;}

      Insertion ins = bestInsertion(state_, current_path, task, now, params_, limitsFor(task, path_, now));
      if (!ins.feasible || !(ins.score > 0.0)) {continue;}
      const double bid = std::min(ins.score, bid_cap);

      // 自己的出價要贏過已知的得標價才出價
      const AgentId cur_z = winner(id);
      const double cur_y = score(id);
      bool can_bid = false;
      if (cur_z == kNoAgent || cur_z == state_.id) {
        can_bid = true;
      } else if (bid > cur_y + params_.score_epsilon) {
        can_bid = true;
      } else if (std::abs(bid - cur_y) <= params_.score_epsilon && state_.id < cur_z) {
        can_bid = true;  // 平手：編號小的優先
      }
      if (!can_bid) {continue;}

      // 以實際分數挑選（出價被上限壓平時，仍先接真正最划算的）
      if (!best.feasible || ins.score > best.score) {
        best = std::move(ins);
        best_id = id;
        best_bid = bid;
      }
    }

    if (!best.feasible) {
      break;
    }
    path_.insert(path_.begin() + static_cast<std::ptrdiff_t>(best.position), best_id);
    bundle_.push_back(best_id);
    y_[best_id] = best_bid;
    z_[best_id] = state_.id;
    changed = true;
  }
  return changed;
}

bool CbbaAgent::onTask(const Task & task, double now)
{
  const auto it = tasks_.find(task.id);
  if (it == tasks_.end()) {
    tasks_[task.id] = task;
    if (task.status != TaskStatus::OPEN) {
      return false;
    }
    y_[task.id] = 0.0;
    z_[task.id] = kNoAgent;
    return buildBundle(now);
  }

  Task & known = it->second;
  if (known.status != TaskStatus::OPEN || task.status == TaskStatus::OPEN) {
    return false;  // 終止狀態不會回到 OPEN；重複的 OPEN 不需處理
  }

  // 任務終止：從 bundle 移除它和它之後的任務，再重新出價
  known.status = task.status;
  known.status_stamp = task.status_stamp;
  bool changed = releaseLost();
  changed = buildBundle(now) || changed;
  return changed;
}

bool CbbaAgent::onMessage(const CbbaMessage & msg, double now)
{
  // 1. 過濾
  if (msg.sender == state_.id || msg.sender == kNoAgent) {
    return false;
  }
  const auto seq_it = last_seq_.find(msg.sender);
  if (seq_it != last_seq_.end() && msg.seq <= seq_it->second) {
    return false;
  }
  last_seq_[msg.sender] = msg.seq;
  last_heard_[msg.sender] = now;

  bool changed = false;

  for (const Bid & bid : msg.bids) {
    // 2. 略過還沒收到內容、或已終止的任務
    if (!isOpen(bid.task_id)) {
      continue;
    }
    // 自己已接受（執行中、保留）的任務：別台的得標資訊不改變它。那台還不知道已鎖定（例如剛加入），
    // 照共識規則會把任務從持有者手上拿走；它收到鎖定後會自己撤回出價
    if (std::find(exec_.pinned.begin(), exec_.pinned.end(), bid.task_id) != exec_.pinned.end()) {
      continue;
    }

    // 3. 套用消解規則（比較的是更新前的時間戳）
    RuleInput in;
    in.me = state_.id;
    in.sender = msg.sender;
    in.sender_winner = bid.winner;
    in.sender_score = bid.score;
    in.my_winner = winner(bid.task_id);
    in.my_score = score(bid.task_id);
    in.sender_stamps = &msg.stamps;
    in.my_stamps = &stamps_;
    in.epsilon = params_.score_epsilon;

    const RuleResult rule = resolveConflict(in);
    if (rule.action == RuleAction::Update) {
      if (in.my_winner != bid.winner || in.my_score != bid.score) {
        z_[bid.task_id] = bid.winner;
        y_[bid.task_id] = bid.score;
        changed = true;
      }
    } else if (rule.action == RuleAction::Reset) {
      if (in.my_winner != kNoAgent || in.my_score != 0.0) {
        z_[bid.task_id] = kNoAgent;
        y_[bid.task_id] = 0.0;
        changed = true;
      }
    }
  }

  // 4. 更新時間戳
  for (const auto & [agent, stamp] : msg.stamps) {
    if (agent == state_.id || agent == msg.sender) {
      continue;
    }
    const auto it = stamps_.find(agent);
    if (it == stamps_.end() || it->second < stamp) {
      stamps_[agent] = stamp;
    }
  }
  stamps_[msg.sender] = now;

  // 5. 連鎖退標  6. 重新出價
  changed = releaseLost() || changed;
  changed = buildBundle(now) || changed;
  return changed;
}

bool CbbaAgent::reevaluate(double now)
{
  bool changed = releaseLost();

  // 用目前的位置、時間、電量，依原本的加入順序重新建立一次路徑
  std::vector<TaskId> new_path;
  double bid_cap = kInf;   // 出價上限，和 buildBundle 相同
  // 固定順序的前綴：已接受的任務；正在搶占（已經排在最前面、可以搶占）的任務放在它們前面，
  // 否則重排時又被擠回後面，指派會來回切換
  std::vector<TaskId> fixed = exec_.pinned;
  if (exec_.active_pinned && fixed.size() >= 2 && tasks_.count(fixed[1]) > 0 &&
    canPreempt(tasks_.at(fixed[1])))
  {
    std::swap(fixed[0], fixed[1]);   // 保留的優先級較高、現在可以中斷：排到執行中的前面（等 BT 接受）
  }
  if (!path_.empty() && std::find(fixed.begin(), fixed.end(), path_.front()) == fixed.end() &&
    !exec_.pinned.empty() && canPreempt(tasks_.at(path_.front())))
  {
    fixed.insert(fixed.begin(), path_.front());
  }
  for (std::size_t b = 0; b < bundle_.size(); ++b) {
    const TaskId id = bundle_[b];
    InsertionLimits limits = limitsFor(tasks_.at(id), new_path, now);
    const auto pin = std::find(fixed.begin(), fixed.end(), id);
    if (pin != fixed.end()) {
      // 固定順序的任務：位置在它前面那些固定的任務之後
      std::size_t before = 0;
      for (auto it = fixed.begin(); it != pin; ++it) {
        before += std::count(new_path.begin(), new_path.end(), *it);
      }
      limits.min_position = before;
      const auto user_allow = limits.allow;
      limits.allow = [before, user_allow](const PathEval & e, std::size_t pos, const std::vector<Task> & c) {
          return pos == before && (!user_allow || user_allow(e, pos, c));
        };
    }
    const Insertion ins = bestInsertion(state_, pathTasks(new_path), tasks_.at(id), now, params_, limits);
    if (!ins.feasible) {
      // 電量已不足以完成：釋放這個任務和之後的任務，讓別人接手
      path_ = new_path;
      for (std::size_t r = b; r < bundle_.size(); ++r) {
        z_[bundle_[r]] = kNoAgent;
        y_[bundle_[r]] = 0.0;
      }
      bundle_.resize(b);
      changed = true;
      break;
    }
    new_path.insert(new_path.begin() + static_cast<std::ptrdiff_t>(ins.position), id);

    // 分數變動超過門檻才更新，避免小幅波動造成任務被搶來搶去
    const double new_bid = std::min(ins.score, bid_cap);
    const double old_bid = score(id);
    const double base = std::max(old_bid, params_.score_epsilon);
    const bool significant = std::abs(new_bid - old_bid) > params_.rebid_threshold * base;
    if (significant || old_bid > bid_cap) {   // 超過上限時一定要壓下來
      y_[id] = new_bid;
      changed = true;
    }
    bid_cap = std::min(bid_cap, score(id));
  }
  if (new_path.size() == bundle_.size()) {
    path_ = new_path;
  }

  changed = buildBundle(now) || changed;
  return changed;
}

std::optional<Task> CbbaAgent::completeTask(TaskId id, double now)
{
  const auto it = tasks_.find(id);
  if (it == tasks_.end() || it->second.status != TaskStatus::OPEN) {
    return std::nullopt;
  }
  it->second.status = TaskStatus::DONE;
  it->second.status_stamp = now;
  bundle_.erase(std::remove(bundle_.begin(), bundle_.end(), id), bundle_.end());
  path_.erase(std::remove(path_.begin(), path_.end(), id), path_.end());
  return it->second;
}

CbbaMessage CbbaAgent::makeMessage(double now)
{
  CbbaMessage msg;
  msg.stamp = now;
  msg.sender = state_.id;
  msg.sender_type = state_.type;
  msg.seq = ++seq_;
  for (const auto & [id, task] : tasks_) {
    if (task.status != TaskStatus::OPEN) {
      continue;
    }
    msg.bids.push_back(Bid{id, score(id), winner(id)});
  }
  for (const auto & [agent, stamp] : stamps_) {
    if (agent != state_.id) {
      msg.stamps[agent] = stamp;
    }
  }
  return msg;
}

bool CbbaAgent::setActive(bool active, double now)
{
  if (active == active_) {
    return false;
  }
  active_ = active;
  return active ? buildBundle(now) : releaseFrom(0);
}

bool CbbaAgent::releaseAgent(AgentId agent, double now)
{
  if (agent == state_.id || agent == kNoAgent) {
    return false;
  }
  bool changed = false;
  for (auto & [id, winner] : z_) {
    if (winner == agent) {
      winner = kNoAgent;
      y_[id] = 0.0;
      changed = true;
    }
  }
  if (changed) {
    buildBundle(now);
  }
  return changed;
}

bool CbbaAgent::releaseStale(double now, double timeout)
{
  bool changed = false;
  for (const auto & [agent, stamp] : stamps_) {
    if (agent != state_.id && now - stamp > timeout) {
      changed = releaseAgent(agent, now) || changed;
    }
  }
  return changed;
}

// ===========================================================================
// 執行狀態的限制（2026-10-09）
// ===========================================================================
double CbbaAgent::queueDeadline(const Task & task) const
{
  return task.explicit_deadline ? task.created + task.deadline_sec : task.created + exec_.max_queue_wait;
}

// 能不能排到已接受的任務前面（搶占）：
//   沒有已接受的任務 → 可以；
//   有執行中的任務 → 要可以安全中斷，而且新任務的優先級比它高（同級不搶占）；
//   只有保留、還沒開始 → 新任務的優先級要比它高（保留是容量承諾，同級不插隊）
bool CbbaAgent::canPreempt(const Task & task) const
{
  if (exec_.pinned.empty()) {
    return true;
  }
  const auto head = tasks_.find(exec_.pinned.front());
  if (head == tasks_.end()) {
    return true;
  }
  const bool higher = taskPriority(task.type) > taskPriority(head->second.type);
  return exec_.active_pinned ? exec_.preemptible && higher : higher;
}

// path：插入前的路徑（釘住的任務在最前面）
InsertionLimits CbbaAgent::limitsFor(
  const Task & task, const std::vector<TaskId> & path, double now) const
{
  InsertionLimits limits;
  std::size_t pinned_in_path = 0;
  for (TaskId p : exec_.pinned) {
    pinned_in_path += std::count(path.begin(), path.end(), p);
  }
  const bool active_head = exec_.active_pinned && !path.empty() && !exec_.pinned.empty() &&
    path.front() == exec_.pinned.front();
  limits.head_remaining = active_head ? exec_.active_remaining : -1.0;
  limits.min_position = canPreempt(task) ? 0 : pinned_in_path;

  const ExecConstraints exec = exec_;
  const bool fire = taskPriority(task.type) >= kPriorityFire;
  const double deadline = queueDeadline(task) - now;
  limits.allow = [exec, fire, deadline, active_head](
    const PathEval & e, std::size_t pos, const std::vector<Task> & candidate) {
      if (!fire || pos == 0) {
        return true;
      }
      // 排隊：執行中任務的剩餘時間要知道；最多 max_queued_fires 個；等待＋前往＋處置要在期限內
      if (active_head && exec.active_remaining < 0.0) {
        return false;
      }
      std::size_t fires = 0;
      for (std::size_t i = 1; i < candidate.size(); ++i) {
        fires += taskPriority(candidate[i].type) >= kPriorityFire ? 1 : 0;
      }
      if (fires > exec.max_queued_fires) {
        return false;
      }
      return e.legs[pos].completion <= deadline;
    };
  return limits;
}

bool CbbaAgent::releaseIf(const std::set<TaskId> & ids)
{
  for (std::size_t b = 0; b < bundle_.size(); ++b) {
    if (ids.count(bundle_[b]) > 0) {
      return releaseFrom(b);
    }
  }
  return false;
}

bool CbbaAgent::setExecConstraints(const ExecConstraints & c, double now)
{
  // 已接受的任務、能不能中斷、能不能接新任務改了：馬上照新的限制重排（不等每秒的重新評估），
  // 例如 BT 走到可以中斷的地方時，保留的火警要馬上排到前面
  const bool changed = c.pinned != exec_.pinned || c.active_pinned != exec_.active_pinned ||
    c.preemptible != exec_.preemptible || c.accept_new != exec_.accept_new;
  exec_ = c;
  if (!changed) {
    return false;
  }
  bool released = false;
  if (!exec_.accept_new) {
    // 不能接新任務（故障、BT 執行狀態逾時）：還沒被 BT 接受的出價全部撤回，讓別台接；已接受的由 BT 處理
    std::set<TaskId> unaccepted;
    for (TaskId id : bundle_) {
      if (std::find(exec_.pinned.begin(), exec_.pinned.end(), id) == exec_.pinned.end()) {
        unaccepted.insert(id);
      }
    }
    released = releaseIf(unaccepted);
  }
  return reevaluate(now) || released;
}

bool CbbaAgent::setLocked(const std::set<TaskId> & locked, double now)
{
  if (locked == locked_) {
    return false;
  }
  locked_ = locked;
  bool changed = releaseIf(locked_);
  if (changed) {
    buildBundle(now);
  }
  return changed;
}

bool CbbaAgent::suspendTask(TaskId id, double now)
{
  suspended_.insert(id);
  const bool changed = releaseIf({id});
  if (changed) {
    buildBundle(now);
  }
  return changed;
}

bool CbbaAgent::resumeSuspended(double now)
{
  if (suspended_.empty()) {
    return false;
  }
  suspended_.clear();
  return buildBundle(now);
}

bool CbbaAgent::excludeTask(TaskId id, double now)
{
  excluded_.insert(id);
  const auto it = std::find(bundle_.begin(), bundle_.end(), id);
  if (it == bundle_.end()) {
    return false;
  }
  releaseFrom(static_cast<std::size_t>(it - bundle_.begin()));
  buildBundle(now);
  return true;
}

std::vector<AgentId> CbbaAgent::lostNeighbors(double now, double timeout) const
{
  std::vector<AgentId> lost;
  for (const auto & [agent, heard] : last_heard_) {
    if (now - heard > timeout) {
      lost.push_back(agent);
    }
  }
  return lost;
}

}  // namespace cbba_core
