#include "cbba_core/cbba_comm.hpp"

#include "cbba_core/capabilities.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <utility>

namespace cbba_core
{

using wire::PacketType;

namespace
{
std::uint16_t clampU16(double v)
{
  return static_cast<std::uint16_t>(std::lround(std::min(std::max(v, 0.0), 65535.0)));
}

// CRC-32（IEEE 802.3）
std::uint32_t crc32Update(std::uint32_t crc, std::uint8_t byte)
{
  crc ^= byte;
  for (int k = 0; k < 8; ++k) {
    crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return crc;
}

int kindOf(PacketType type) {return static_cast<int>(type);}

// 每次啟動不同：亂數裝置加上時鐘（沒有亂數裝置時至少時鐘不同）
std::uint64_t randomSession()
{
  std::random_device rd;
  const std::uint64_t r = (static_cast<std::uint64_t>(rd()) << 32) ^ rd();
  const auto t = static_cast<std::uint64_t>(
    std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const std::uint64_t s = r ^ (t * 0x9E3779B97F4A7C15ull);
  return s == 0 ? 1 : s;
}

std::vector<std::uint16_t> toIds(const std::set<AgentId> & ids)
{
  return std::vector<std::uint16_t>(ids.begin(), ids.end());
}
}  // namespace

// ===========================================================================
// 工具
// ===========================================================================
std::string taskName(TaskId id)
{
  // 舊型照版本 3（8 位十六進位，itest、e2e 的 grep 照舊）；版本 4 是 建立者.世代.流水號（十六進位）
  char buf[32];
  if (isLegacyTaskId(id)) {
    std::snprintf(buf, sizeof(buf), "%08X", static_cast<unsigned>(id));
  } else {
    std::snprintf(buf, sizeof(buf), "%04X.%04X.%08X", static_cast<unsigned>(taskCreator(id)),
      static_cast<unsigned>(taskEpoch(id)), static_cast<unsigned>(taskSeq(id)));
  }
  return buf;
}

std::uint32_t toStampMs(double t)
{
  return static_cast<std::uint32_t>(static_cast<std::uint64_t>(std::llround(t * 1e3)));
}

double fromStampMs(std::uint32_t ms, double now)
{
  const auto now_ms = static_cast<std::uint64_t>(std::llround(now * 1e3));
  const std::uint32_t age = static_cast<std::uint32_t>(now_ms) - ms;   // mod 2^32
  // 對方時鐘稍快時 age 會變成很大的數：視為現在
  const std::uint64_t back = age > 0xF0000000u ? 0 : age;
  return static_cast<double>(now_ms - back) / 1e3;
}

// ===========================================================================
// 本機輸入
// ===========================================================================
CbbaComm::CbbaComm(
  const AgentState & state, const ScoringParams & params, const CommConfig & config,
  std::uint64_t session_id)
: agent_(state, params), config_(config), session_(session_id != 0 ? session_id : randomSession())
{
  if (!wire::validAgentId(state.id)) {
    throw std::invalid_argument("agent id must be 1~65534");
  }
}

void CbbaComm::setParticipating(bool participating, double now)
{
  if (participating == agent_.active()) {
    return;
  }
  markChanged(agent_.setActive(participating, now));
  dirty_ = true;   // 鄰居要馬上知道
}

bool CbbaComm::addLocalTask(const Task & task, double now)
{
  if (!validTaskId(task.id) || taskCreator(task.id) != id() || origins_.count(task.id) > 0 ||
    std::any_of(queued_.begin(), queued_.end(),
    [&task](const PendingTask & p) {return p.task.id == task.id;}))
  {
    return false;
  }
  if (start_ < 0.0) {
    start_ = now;
  }
  updateJoined(now);
  if (!joined_) {
    queued_.push_back(PendingTask{task, now});   // 先和鄰居同步，補齊既有任務
    return true;
  }
  return assign(task, now);
}

const char * toString(ReportCheck check)
{
  switch (check) {
    case ReportCheck::ACCEPTED: return "接受";
    case ReportCheck::NOT_OPEN: return "不認得或已結束";
    case ReportCheck::STALE: return "不是目前的正式指派";
    case ReportCheck::PENDING: return "正在確認完成";
    case ReportCheck::NOT_AUTHORIZED: return "沒有權限";
  }
  return "?";
}

// ===========================================================================
// 正式指派
// ===========================================================================
Assignment CbbaComm::assignment(TaskId task_id) const
{
  const auto it = assignments_.find(task_id);
  return it != assignments_.end() ? it->second : Assignment{};
}

bool CbbaComm::mergeAssignment(TaskId task_id, const Assignment & a)
{
  Assignment & cur = assignments_[task_id];
  if (!newerThan(a, cur)) {
    return false;
  }
  cur = a;
  dirty_ = true;   // 盡快傳給鄰居
  return true;
}

std::uint64_t CbbaComm::beginAssignment(TaskId task_id)
{
  // 已知的最新版本、自己發過的版本，取大的再加一：作廢的版本（拒絕、逾時）不會重複使用
  std::uint64_t base = assignment(task_id).version;
  const auto used = used_versions_.find(task_id);
  if (used != used_versions_.end()) {
    base = std::max(base, used->second);
  }
  const Assignment a{base + 1, id(), AssignState::PENDING};
  assignments_[task_id] = a;
  used_versions_[task_id] = a.version;
  dirty_ = true;
  return a.version;
}

bool CbbaComm::confirmAssignment(TaskId task_id, std::uint64_t version, AssignState state)
{
  const Assignment cur = assignment(task_id);
  if (!isOpen(task_id) || cur.assignee != id() || cur.version != version ||
    static_cast<int>(state) < static_cast<int>(cur.state))
  {
    return false;
  }
  assignments_[task_id].state = state;
  dirty_ = true;
  return true;
}

bool CbbaComm::endAssignment(TaskId task_id)
{
  const Assignment cur = assignment(task_id);
  if (cur.assignee != id() || cur.state == AssignState::NONE || !isOpen(task_id) ||
    completions_.count(task_id) > 0)
  {
    return false;
  }
  const std::uint64_t version = std::max(cur.version, used_versions_[task_id]) + 1;
  assignments_[task_id] = Assignment{version, kNoAgent, AssignState::NONE};
  used_versions_[task_id] = version;
  dirty_ = true;
  return true;
}

std::vector<std::pair<TaskId, AgentId>> CbbaComm::lockedByLostAgents(double now) const
{
  std::vector<std::pair<TaskId, AgentId>> out;
  for (const auto & [task_id, a] : assignments_) {
    if (!isLocking(a.state) || a.assignee == id() || !isOpen(task_id)) {
      continue;
    }
    const auto stamp = agent_.stamps().find(a.assignee);
    if (stamp == agent_.stamps().end() || now - stamp->second > config_.lost_timeout) {
      out.emplace_back(task_id, a.assignee);
    }
  }
  return out;
}

double CbbaComm::takeoverLimit(TaskId task_id) const
{
  const auto task = agent_.tasks().find(task_id);
  // 火警（P0，2026-11-11 起看優先級；舊型的 GROUND_INTERVENTION 就是 P0）。依優先級分級的時限是之後的 takeover_by_priority
  const bool fire = task != agent_.tasks().end() && taskPriority(task->second) >= kPriorityFire;
  return fire ? config_.takeover_fire : config_.takeover_patrol;
}

std::vector<std::pair<TaskId, AgentId>> CbbaComm::takeTakeovers()
{
  std::vector<std::pair<TaskId, AgentId>> out;
  out.swap(takeovers_);
  return out;
}

// 失聯接手：持有者失聯超過時限，撤銷它的鎖定（新版本，執行者 0），任務重新競標。
// 失聯的時刻用持有者最後的 s + lost_timeout（各機看到的差不多，同時撤銷）；沒有它的 s 時（剛開機，
// 還沒收過它的資訊）用第一次看到「鎖著、持有者失聯」的時刻，不會比別台早撤銷
void CbbaComm::takeOverLost(double now)
{
  std::map<TaskId, double> since;
  for (const auto & [task_id, holder] : lockedByLostAgents(now)) {
    const auto first = lost_since_.find(task_id);
    double t = first != lost_since_.end() ? first->second : now;
    const auto stamp = agent_.stamps().find(holder);
    if (stamp != agent_.stamps().end()) {
      t = std::min(t, stamp->second + config_.lost_timeout);
    }
    if (now - t < takeoverLimit(task_id)) {
      since[task_id] = t;
      continue;
    }
    const std::uint64_t version = std::max(assignment(task_id).version, used_versions_[task_id]) + 1;
    assignments_[task_id] = Assignment{version, kNoAgent, AssignState::NONE};
    used_versions_[task_id] = version;
    dirty_ = true;
    takeovers_.emplace_back(task_id, holder);
    ++stats_.takeovers;
  }
  lost_since_ = std::move(since);
}

// 別台「保留」或「執行中」的任務交給核心：不出價（持有者失聯也先鎖著，到時限才由 takeOverLost 撤銷）
void CbbaComm::updateLocks(double now)
{
  std::set<TaskId> locked;
  for (const auto & [task_id, a] : assignments_) {
    if (isLocking(a.state) && a.assignee != id() && isOpen(task_id)) {
      locked.insert(task_id);
    }
  }
  markChanged(agent_.setLocked(locked, now));
}

void CbbaComm::setExecConstraints(const ExecConstraints & c, double now)
{
  markChanged(agent_.setExecConstraints(c, now));
}

bool CbbaComm::suspendTask(TaskId task_id, double now)
{
  const bool changed = agent_.suspendTask(task_id, now);
  markChanged(changed);
  return changed;
}

void CbbaComm::resumeSuspended(double now)
{
  markChanged(agent_.resumeSuspended(now));
}

void CbbaComm::excludeTask(TaskId task_id, double now, double duration)
{
  markChanged(agent_.excludeTask(task_id, now, duration));
}

void CbbaComm::countFailure(TaskId task_id, double now)
{
  markChanged(agent_.countFailure(task_id, now));
}

ReportCheck CbbaComm::reportResult(TaskId task_id, bool success, std::uint64_t version, double now,
  double exclude_sec)
{
  if (!isOpen(task_id)) {
    return ReportCheck::NOT_OPEN;
  }
  const Assignment cur = assignment(task_id);
  if (cur.assignee != id() || cur.version != version || cur.state != AssignState::ACTIVE) {
    return ReportCheck::STALE;
  }
  if (completions_.count(task_id) > 0) {
    return ReportCheck::PENDING;
  }
  if (!success) {
    // 執行失敗：結束指派（新版本）、交回競標池，自己 exclude_sec 秒內不再出價；任務保留，不送 TASK_CLOSE
    endAssignment(task_id);
    markChanged(agent_.excludeTask(task_id, now, exclude_sec));
    return ReportCheck::ACCEPTED;
  }
  PendingCompletion pc;
  pc.round_key = roundKey();
  pc.version = version;
  const auto alive = aliveSet(now);
  pc.view.assign(alive.begin(), alive.end());
  completions_[task_id] = pc;
  if (allAcked(pc, now)) {
    finishCompletion(task_id, now);   // 沒有其他存活的載具
  } else {
    sendAnnounce(task_id, now);
  }
  return ReportCheck::ACCEPTED;
}

bool CbbaComm::canCancel(AgentId actor, TaskId task_id) const
{
  return actor == taskCreator(task_id) ||
         std::find(config_.cancel_authorities.begin(), config_.cancel_authorities.end(), actor) !=
         config_.cancel_authorities.end();
}

ReportCheck CbbaComm::cancelTask(TaskId task_id, double now)
{
  if (!isOpen(task_id)) {
    return ReportCheck::NOT_OPEN;
  }
  if (!canCancel(id(), task_id)) {
    return ReportCheck::NOT_AUTHORIZED;
  }
  const wire::Header h = newHeader(PacketType::TASK_CLOSE);
  wire::TaskCloseBody close;
  close.relay = wire::originOf(h);
  close.task_id = task_id;
  close.reason = wire::kCloseCancelled;
  close.actor_id = id();
  close.round_key = roundKey();
  close.assign_version = assignment(task_id).version;
  dedup_.firstSeen(PacketType::TASK_CLOSE, close.relay, now);
  closes_[task_id] = close;
  completions_.erase(task_id);
  send(wire::encodeTaskClose(h, close), PacketType::TASK_CLOSE);
  markClosed(task_id, wire::kCloseCancelled, now);
  return ReportCheck::ACCEPTED;
}

// ===========================================================================
// 收發
// ===========================================================================
wire::Header CbbaComm::newHeader(PacketType type)
{
  wire::Header h;
  h.type = type;
  h.sender_id = id();
  h.session_id = session_;
  h.sequence = seq_.next();
  return h;
}

void CbbaComm::send(Bytes bytes, PacketType type, TxKind kind)
{
  ++stats_.sent[type];
  stats_.sent_bytes[type] += bytes.size();
  stats_.tx_bytes += bytes.size();
  if (kind == TxKind::RETRY) {
    stats_.tx_retry_bytes += bytes.size();
  } else if (kind == TxKind::RELAY) {
    stats_.tx_relay_bytes += bytes.size();
  }
  outbox_.push_back(std::move(bytes));
}

void CbbaComm::relay(const Bytes & bytes)
{
  if (auto fwd = wire::makeRelay(bytes, id(), session_, seq_)) {
    const auto type = wire::parseHeader(*fwd)->type;
    ++stats_.relayed;
    send(std::move(*fwd), type, TxKind::RELAY);
  }
}

void CbbaComm::receive(const std::uint8_t * data, std::size_t size, double now)
{
  const auto h = wire::parseHeader(data, size);
  if (!h) {
    ++stats_.malformed;
    return;
  }
  if (h->sender_id == id()) {
    return;   // multicast loopback 收到自己的
  }
  if (!filter_.accept(*h)) {
    ++stats_.stale;
    return;
  }
  ++stats_.received[h->type];
  neighborOf(*h).last_heard = now;   // 發送者（含轉送者）一定是直接鄰居
  const Bytes bytes(data, data + size);
  switch (h->type) {
    case PacketType::AGENT_STATE: onAgentState(*h, bytes); break;
    case PacketType::CBBA_STATE: onCbbaState(*h, bytes, now); break;
    case PacketType::TASK_ANNOUNCE: onTaskAnnounce(bytes, now); break;
    case PacketType::COMPLETION: onCompletion(bytes, now); break;
    case PacketType::COMPLETION_ACK: onCompletionAck(bytes, now); break;
    case PacketType::TASK_CLOSE: onTaskClose(bytes, now); break;
    case PacketType::GAP_REQUEST: onGapRequest(bytes); break;
    case PacketType::SYNC_BATCH: onSyncBatch(*h, bytes, now); break;
  }
}

std::vector<Bytes> CbbaComm::poll(double now)
{
  if (start_ < 0.0) {
    start_ = now;
  }

  // 依目前位置、電量重新評估
  if (now >= next_reevaluate_) {
    next_reevaluate_ = now + config_.reevaluate_period;
    markChanged(agent_.reevaluate(now));
  }

  // 失聯：太久沒有某台的新資訊，釋放它得標的任務（它已接受的任務先鎖著，到時限才撤銷，見 takeOverLost）
  markChanged(agent_.releaseStale(now, config_.lost_timeout));
  takeOverLost(now);
  updateLocks(now);

  // 和鄰居同步之後：排隊中的任務廣播出去
  updateJoined(now);
  if (!queued_.empty() && joined_) {
    std::vector<PendingTask> queued;
    queued.swap(queued_);
    for (const PendingTask & p : queued) {
      assign(p.task, now);
    }
  }

  // 保底：輪流重播一個已知任務（已結束的連同 TASK_CLOSE）
  if (now >= next_replay_) {
    next_replay_ = now + config_.replay_period;
    if (!origins_.empty() && !aliveNeighbors(now).empty()) {
      auto it = replay_cursor_ ? origins_.upper_bound(*replay_cursor_) : origins_.begin();
      if (it == origins_.end()) {
        it = origins_.begin();
      }
      replay_cursor_ = it->first;
      resendTask(it->first, now);
      if (!isOpen(it->first)) {
        resendClose(it->first, now);
      }
    }
  }

  // 完成宣告：收齊就送 TASK_CLOSE；沒收齊每 completion_retry 重送，超過次數就以收到的確認送出
  std::vector<TaskId> finished;
  for (auto & [task_id, pc] : completions_) {
    if (allAcked(pc, now)) {
      finished.push_back(task_id);
    } else if (now - pc.last_sent >= config_.completion_retry) {
      if (pc.tries > config_.completion_max) {
        finished.push_back(task_id);
      } else {
        sendAnnounce(task_id, now);
      }
    }
  }
  for (TaskId task_id : finished) {
    finishCompletion(task_id, now);
  }

  // 收斂偵測：得標者一變就歸零，之後每個週期加一
  auto winners = winnerMap();
  if (winners != last_winners_) {
    last_winners_ = std::move(winners);
    stable_steps_ = 0;
  }

  // CBBA_STATE：週期、變動時立即、和鄰居不一致時快速重送
  const bool periodic = now >= next_periodic_;
  const bool immediate = dirty_ && now - last_state_sent_ >= config_.min_gap;
  const bool fast = !dirty_ && fast_count_ < config_.fast_max &&
    now - last_state_sent_ >= config_.fast_period && disagrees(now);
  if (periodic || immediate || fast) {
    if (periodic) {
      next_periodic_ = std::max(next_periodic_ + config_.state_period, now);
      stable_steps_ = std::min(stable_steps_ + 1, 255);
    }
    if (fast) {
      ++fast_count_;
    }
    // 只因為和鄰居不一致才送的（不是週期、也不是自己變動）算重傳
    sendCbbaState(now, fast && !periodic && !immediate);
    dirty_ = false;
  }

  // AGENT_STATE：定期
  if (now >= next_status_) {
    next_status_ = std::max(next_status_ + config_.status_period, now);
    sendAgentState(now);
  }

  // 補發佇列：狀態送完之後，用剩下的額度
  drainRepair(now);

  std::vector<Bytes> out;
  out.swap(outbox_);
  return out;
}

void CbbaComm::markChanged(bool changed)
{
  if (changed) {
    dirty_ = true;
    ++exchange_step_;
    fast_count_ = 0;
  }
}

// ===========================================================================
// AGENT_STATE
// ===========================================================================
wire::AgentStateBody CbbaComm::agentStateBody(double now) const
{
  const AgentState & s = agent_.state();
  wire::AgentStateBody b;
  b.flags.participating = agent_.active();
  b.flags.telemetry_ok = status_.telemetry_ok;
  b.flags.flight_state_valid = status_.flight_state_valid;
  if (status_.flight_state_valid) {
    b.flags.armed = status_.armed;
    b.flags.offboard = status_.offboard;
    b.flags.landed = status_.landed;
  }
  b.vehicle_type = static_cast<std::uint8_t>(s.type);
  b.capabilities = availableCaps(s);
  b.x = static_cast<float>(s.position.x);
  b.y = static_cast<float>(s.position.y);
  b.z = static_cast<float>(s.position.z);
  b.battery = static_cast<float>(std::min(std::max(s.battery, 0.0), 100.0));
  b.exec_known = exec_report_.known;
  b.execution_state = exec_report_.execution_state;
  b.preemptible = exec_report_.preemptible;
  b.remaining_time = static_cast<float>(exec_report_.remaining_time);
  for (const auto & [task_id, a] : assignments_) {
    if (a.assignee != id() || !isOpen(task_id)) {
      continue;
    }
    if (a.state == AssignState::ACTIVE) {
      b.has_active = true;
      b.active_task = task_id;
      b.active_version = a.version;
    } else if (a.state == AssignState::RESERVED) {
      b.has_queued = true;
      b.queued_task = task_id;
      b.queued_version = a.version;
    }
  }
  b.path = agent_.path();
  for (AgentId a : aliveNeighbors(now)) {
    b.neighbors.push_back(a);
  }
  return b;
}

void CbbaComm::sendAgentState(double now)
{
  send(wire::encodeAgentState(newHeader(PacketType::AGENT_STATE), agentStateBody(now)),
    PacketType::AGENT_STATE);
}

// 發送者的紀錄；session 換了（重開機）就把「新舊判斷」重來
NeighborInfo & CbbaComm::neighborOf(const wire::Header & h)
{
  NeighborInfo & n = neighbors_[h.sender_id];
  if (n.session != h.session_id) {
    n.session = h.session_id;
    n.has_snapshot = false;
    n.pending_parts.clear();
    n.pending_view.clear();
    n.pending_count = 0;
    n.has_status = false;
    n.has_digest = false;
    n.digest.clear();
  }
  return n;
}

void CbbaComm::onAgentState(const wire::Header & h, const Bytes & bytes)
{
  const auto b = wire::decodeAgentState(bytes);
  if (!b) {
    ++stats_.malformed;
    return;
  }
  NeighborInfo & n = neighborOf(h);
  if (n.has_status && static_cast<std::int32_t>(h.sequence - n.status_seq) < 0) {
    ++stats_.stale;   // 亂序：比上一則舊
    return;
  }
  n.status_seq = h.sequence;
  n.has_status = true;
  n.flags = b->flags;
  n.vehicle_type = static_cast<AgentType>(b->vehicle_type);
  n.capabilities = b->capabilities;
  n.position = {b->x, b->y, b->z};
  n.battery = b->battery;
  n.exec_known = b->exec_known;
  n.execution_state = b->execution_state;
  n.preemptible = b->preemptible;
  n.remaining_time = b->remaining_time;
  n.active.reset();
  n.queued.reset();
  if (b->has_active) {
    n.active = std::make_pair(b->active_task, b->active_version);
  }
  if (b->has_queued) {
    n.queued = std::make_pair(b->queued_task, b->queued_version);
  }
  n.neighbors = std::set<AgentId>(b->neighbors.begin(), b->neighbors.end());
}

// ===========================================================================
// CBBA_STATE
// ===========================================================================
bool CbbaComm::isOpen(TaskId task_id) const
{
  const auto t = agent_.tasks().find(task_id);
  return origins_.count(task_id) > 0 && t != agent_.tasks().end() &&
         t->second.status == TaskStatus::OPEN;
}

std::map<TaskId, AgentId> CbbaComm::winnerMap() const
{
  std::map<TaskId, AgentId> out;
  for (const auto & [task_id, origin] : origins_) {
    if (isOpen(task_id)) {
      out[task_id] = agent_.winner(task_id);
    }
  }
  return out;
}

wire::CbbaStateBody CbbaComm::cbbaStateBody(double now) const
{
  wire::CbbaStateBody b;
  b.round_key = roundKey();
  b.exchange_step = exchange_step_;
  b.participating = agent_.active();
  b.stable_steps = static_cast<std::uint8_t>(stable_steps_);
  b.snapshot_id = snapshot_id_;
  // 不參與出價時不送 s（收的那端不會把它的紀錄交給 CBBA 規則），但紀錄照送：
  // 停止參與（PX4 斷線、降落）時自己撤銷的正式指派只有自己知道，不送的話別台一直以為它還在執行、鎖著不出價
  // （2026-10-11 SITL 測試發現：飛行中關掉 PX4，任務卡住沒人接手）。收的那端只從紀錄合併正式指派
  if (agent_.active()) {
    for (const auto & [agent, stamp] : agent_.stamps()) {
      if (agent != id() && wire::validAgentId(agent)) {
        b.stamps.push_back(wire::Stamp{agent, toStampMs(stamp)});
      }
    }
    b.stamps.push_back(wire::Stamp{id(), toStampMs(now)});
  }
  for (const auto & [task_id, winner] : winnerMap()) {
    const Assignment a = assignment(task_id);
    b.records.push_back(wire::Record{task_id, agent_.tasks().at(task_id).revision, winner,
        static_cast<float>(std::max(agent_.score(task_id), 0.0)), a.version, a.assignee,
        static_cast<std::uint8_t>(a.state)});
  }
  return b;
}

void CbbaComm::sendCbbaState(double now, bool retry)
{
  ++snapshot_id_;
  wire::CbbaStateBody full = cbbaStateBody(now);
  if (wantDigest(now)) {
    full.has_digest = true;
    for (const auto & [creator, d] : digestMap()) {
      if (full.digest.size() == 255) {
        break;   // 封包格式最多 255 個建立者；之後的這次不比對（實際上不會有這麼多台）
      }
      full.digest.push_back(d);
    }
    last_digest_sent_ = now;
    last_digest_key_ = full.round_key;
  }
  for (const wire::CbbaStateBody & part : wire::splitCbbaState(full, config_.max_packet)) {
    send(wire::encodeCbbaState(newHeader(PacketType::CBBA_STATE), part), PacketType::CBBA_STATE,
      retry ? TxKind::RETRY : TxKind::BASE);
    ++stats_.cbba_parts;
  }
  last_state_sent_ = now;
}

void CbbaComm::onCbbaState(const wire::Header & h, const Bytes & bytes, double now)
{
  const auto b = wire::decodeCbbaState(bytes);
  if (!b) {
    ++stats_.malformed;
    return;
  }
  NeighborInfo & n = neighborOf(h);
  if (n.has_snapshot && static_cast<std::int16_t>(b->snapshot_id - n.snapshot) < 0) {
    ++stats_.stale;   // 亂序：比已經收到的那次舊
    return;
  }
  n.has_snapshot = true;
  n.snapshot = b->snapshot_id;
  const bool was_participating = n.participating;
  n.participating = b->participating;
  n.round_key = b->round_key;
  n.stable_steps = b->stable_steps;
  if (b->has_digest) {
    n.has_digest = true;
    n.digest_key = b->round_key;
    n.digest.clear();
    for (const wire::DigestEntry & d : b->digest) {
      n.digest[DigestKey{d.creator, d.epoch}] = d;
    }
  }

  // 收集同一次的各批；換了 snapshot 就丟掉還沒收齊的
  if (b->snapshot_id != n.pending_snapshot || b->part_count != n.pending_count) {
    if (!n.pending_parts.empty() && n.pending_parts.size() < n.pending_count) {
      ++stats_.incomplete_snapshots;
    }
    n.pending_snapshot = b->snapshot_id;
    n.pending_count = b->part_count;
    n.pending_parts.clear();
    n.pending_view.clear();
  }
  if (n.pending_parts.insert(b->part_index).second) {
    for (const wire::Record & r : b->records) {
      n.pending_view[r.task_id] = r.winner_id;
      if (origins_.count(r.task_id) > 0) {
        mergeAssignment(r.task_id,
          Assignment{r.assign_version, r.assignee_id, static_cast<AssignState>(r.assign_state)});
      }
    }
    if (n.pending_parts.size() == n.pending_count) {
      n.view.swap(n.pending_view);
      n.pending_view.clear();
      n.has_view = true;
    }
  }

  // 補發要在判斷參與之前：還在地面、剛開機的鄰居最需要補。摘要每一批都帶，不用等收齊
  compareDigest(h.sender_id, n, now);

  if (!n.participating) {
    // 停止出價（降落、重開機）：它得標的任務立即重新分配
    if (was_participating) {
      markChanged(agent_.releaseAgent(h.sender_id, now));
    }
    return;
  }

  // 這一批直接交給 CBBA 規則；還不認得的任務由核心略過。
  // 得標者是「已知停止參與」的鄰居時略過那筆：我已經釋放它的任務，但別台在知道它停止之前送出、
  // 晚到的訊息還寫它得標，而且那台對它的時間戳比我新，規則會把它當得標者再寫回來，
  // 要等 lost_timeout 才又釋放（2026-10-09 測試發現，版本 1 也有同樣的競賽）。
  CbbaMessage msg;
  msg.stamp = now;
  msg.sender = h.sender_id;
  msg.sender_type = n.vehicle_type;
  msg.seq = ++core_seq_[h.sender_id];   // 封包層已過濾舊封包，核心只要遞增的序號
  for (const wire::Record & r : b->records) {
    const auto w = neighbors_.find(r.winner_id);
    if (w != neighbors_.end() && w->second.has_snapshot && !w->second.participating) {
      continue;
    }
    // 任務定義的版本不同（版本 4）：出價是照不同內容算的，不交給規則。對方較新時等新的 TASK_ANNOUNCE（補發會帶來）；
    // 較舊時等它更新。正式指派和任務定義的版本無關，上面照常合併
    const auto known = agent_.tasks().find(r.task_id);
    if (known != agent_.tasks().end() && known->second.revision != r.revision) {
      continue;
    }
    msg.bids.push_back(Bid{r.task_id, r.winning_bid, r.winner_id});
  }
  for (const wire::Stamp & s : b->stamps) {
    if (s.stamp_ms != 0 && s.agent_id != h.sender_id) {
      msg.stamps[s.agent_id] = fromStampMs(s.stamp_ms, now);
    }
  }
  markChanged(agent_.onMessage(msg, now));
}

// ===========================================================================
// 補發（版本 3，2026-10-28，doc/protocol_v3_sync.md）：壓縮摘要 → 缺口請求 → 受限分批回覆
// ===========================================================================
std::map<DigestKey, wire::DigestEntry> CbbaComm::digestMap() const
{
  // origins_ 依 task_id 排序：同一個（建立者, 世代）的任務連在一起、流水號由小到大
  //（舊型 ID 是 建立者 << 16 | 流水號，版本 4 是 建立者 << 48 | 世代 << 32 | 流水號，兩種各自連續）
  std::map<DigestKey, wire::DigestEntry> out;
  for (const auto & [task_id, origin] : origins_) {
    const DigestKey key{taskCreator(task_id), taskEpoch(task_id)};
    const std::uint32_t seq = taskSeq(task_id);
    const bool closed = !isOpen(task_id);
    wire::DigestEntry & d = out[key];
    if (d.known == 0) {
      d.creator = key.first;
      d.epoch = key.second;
      d.min_seq = seq;
      d.crc = 0xFFFFFFFFu;
    }
    d.max_seq = seq;
    ++d.known;
    d.closed += closed ? 1 : 0;
    for (int shift = 24; shift >= 0; shift -= 8) {
      d.crc = crc32Update(d.crc, static_cast<std::uint8_t>(seq >> shift));
    }
    d.crc = crc32Update(d.crc, closed ? 1 : 0);
  }
  for (auto & [key, d] : out) {
    d.crc = ~d.crc;
  }
  return out;
}

wire::SeqState CbbaComm::seqState(TaskId task_id) const
{
  if (origins_.count(task_id) == 0) {
    return wire::SeqState::UNKNOWN;
  }
  return isOpen(task_id) ? wire::SeqState::OPEN : wire::SeqState::CLOSED;
}

// 這次 CBBA_STATE 要不要帶摘要：收斂後每 digest_period 一次；自己的狀態變了、或有鄰居和我不一致時每次都帶
bool CbbaComm::wantDigest(double now) const
{
  const std::uint32_t key = roundKey();
  if (now - last_digest_sent_ >= config_.digest_period || key != last_digest_key_) {
    return true;
  }
  for (AgentId a : aliveNeighbors(now)) {
    const NeighborInfo & n = neighbors_.at(a);
    if (n.has_snapshot && n.round_key != key) {
      return true;
    }
  }
  return false;
}

// 和鄰居的摘要逐個建立者比對：
//   它有我可能缺的（它那筆存在、和我的不同）→ 送缺口請求（那段流水號我的狀態），它只回我缺的；
//   我有它可能缺的（我那筆存在、和它的不同）→ 把那個建立者最近新增、結束的任務直接放進補發佇列（不等請求）。
// 兩邊都會比，各自問自己缺的。摘要不是最新的（它的 round_key 已經變了）就不比
void CbbaComm::compareDigest(AgentId neighbor, const NeighborInfo & n, double now)
{
  if (!n.has_digest || n.digest_key != n.round_key || n.round_key == roundKey()) {
    return;
  }
  const auto mine = digestMap();
  std::set<DigestKey> streams;
  for (const auto & [c, d] : mine) {
    streams.insert(c);
  }
  for (const auto & [c, d] : n.digest) {
    streams.insert(c);
  }
  for (const DigestKey & c : streams) {
    const auto m = mine.find(c);
    const auto t = n.digest.find(c);
    const bool have_m = m != mine.end();
    const bool have_t = t != n.digest.end();
    if (have_m && have_t && m->second == t->second) {
      continue;
    }
    if (have_t) {
      const std::uint32_t lo = have_m ? std::min(m->second.min_seq, t->second.min_seq) : t->second.min_seq;
      const std::uint32_t hi = have_m ? std::max(m->second.max_seq, t->second.max_seq) : t->second.max_seq;
      sendGapRequest(neighbor, c, lo, hi, now);
    }
    if (have_m) {
      const TaskId first = makeTaskId(c.first, c.second, 0);
      for (auto it = origins_.lower_bound(first);
        it != origins_.end() && taskCreator(it->first) == c.first && taskEpoch(it->first) == c.second; ++it)
      {
        const TaskId id = it->first;
        const auto added = added_at_.find(id);
        const auto closed = closed_at_.find(id);
        const bool new_task = added != added_at_.end() && now - added->second <= config_.push_recent_sec;
        const bool new_close = closed != closed_at_.end() && now - closed->second <= config_.push_recent_sec &&
          closes_.count(id) > 0;
        if (!new_task && !new_close) {
          continue;
        }
        double & last = last_resend_.try_emplace({0, id}, -1e9).first->second;   // 0：直接推送
        if (now - last < config_.resend_gap) {
          continue;
        }
        last = now;
        queueRepair(id, new_task, new_close);
      }
    }
  }
}

// 缺口請求：建立者 c、流水號 lo～hi 我的狀態。超過 kMaxGapSeqs 時分段，下次從上次問到的地方接著問
void CbbaComm::sendGapRequest(AgentId target, DigestKey stream, std::uint32_t lo, std::uint32_t hi,
  double now)
{
  const auto key = std::make_pair(target, stream);
  double & last = last_gap_request_.try_emplace(key, -1e9).first->second;
  if (now - last < config_.gap_request_gap) {
    return;
  }
  last = now;
  // 64 位元算：版本 4 的流水號到 0xFFFFFFFF，first + count 不能在 32 位元繞回
  std::uint64_t first = lo;
  const auto cursor = gap_cursor_.find(key);
  if (cursor != gap_cursor_.end() && cursor->second > lo && cursor->second <= hi) {
    first = cursor->second;
  }
  const std::uint64_t count = std::min<std::uint64_t>(std::uint64_t{hi} - first + 1, wire::kMaxGapSeqs);
  gap_cursor_[key] = first + count > hi ? lo : first + count;

  wire::GapRequestBody b;
  b.target_id = target;
  b.creator = stream.first;
  b.epoch = stream.second;
  b.first_seq = static_cast<std::uint32_t>(first);
  b.states.reserve(count);
  for (std::uint64_t k = 0; k < count; ++k) {
    const auto seq = static_cast<std::uint32_t>(first + k);
    b.states.push_back(seqState(makeTaskId(stream.first, stream.second, seq)));
  }
  send(wire::encodeGapRequest(newHeader(PacketType::GAP_REQUEST), b), PacketType::GAP_REQUEST,
    TxKind::RETRY);
  ++stats_.gap_requests;
}

// 回答指名給我的缺口請求：只補對方缺的（我知道、它不認得 → 任務定義；我已結束、它不知道結束 → 結束紀錄）
void CbbaComm::onGapRequest(const Bytes & bytes)
{
  const auto b = wire::decodeGapRequest(bytes);
  if (!b) {
    ++stats_.malformed;
    return;
  }
  if (b->target_id != id()) {
    return;
  }
  bool any = false;
  for (std::size_t k = 0; k < b->states.size(); ++k) {
    const TaskId task_id = makeTaskId(b->creator, b->epoch, static_cast<std::uint32_t>(b->first_seq + k));
    const wire::SeqState mine = seqState(task_id);
    const wire::SeqState theirs = b->states[k];
    if (mine == wire::SeqState::UNKNOWN) {
      continue;
    }
    const bool announce = theirs == wire::SeqState::UNKNOWN;
    const bool close = mine == wire::SeqState::CLOSED && theirs != wire::SeqState::CLOSED &&
      closes_.count(task_id) > 0;
    if (announce || close) {
      queueRepair(task_id, announce, close);
      any = true;
    }
  }
  if (any) {
    ++stats_.gap_answers;
  }
}

void CbbaComm::queueRepair(TaskId task_id, bool announce, bool close)
{
  RepairNeed & need = isOpen(task_id) ? repair_open_[task_id] : repair_closed_[task_id];
  need.announce = need.announce || announce;
  need.close = need.close || close;
}

// 補發佇列 → SYNC_BATCH：進行中的任務先、已結束的後；每批 ≤ max_packet、≤ 255 筆；token bucket 限速
void CbbaComm::drainRepair(double now)
{
  const double cap = std::max(config_.repair_burst, static_cast<double>(config_.max_packet));
  if (repair_last_ < 0.0) {
    repair_tokens_ = cap;
  } else {
    repair_tokens_ = std::min(cap, repair_tokens_ + (now - repair_last_) * config_.repair_rate);
  }
  repair_last_ = now;

  // 每一筆只要表頭之後的內容：用不佔序號的表頭編碼（額度不夠、下次重組時不浪費序號）
  wire::Header item_header;
  item_header.sender_id = id();
  item_header.session_id = session_;
  while (!repair_open_.empty() || !repair_closed_.empty()) {
    std::vector<wire::SyncItem> items;
    std::vector<std::pair<std::map<TaskId, RepairNeed> *, TaskId>> taken;
    std::size_t size = wire::kSyncBatchBaseSize;
    bool full = false;
    for (auto * queue : {&repair_open_, &repair_closed_}) {
      for (const auto & [task_id, need] : *queue) {
        std::vector<wire::SyncItem> mine;
        if (need.announce && origins_.count(task_id) > 0) {
          if (auto item = wire::toSyncItem(wire::encodeTaskAnnounce(item_header, announceBody(task_id))))
          {
            mine.push_back(std::move(*item));
          }
        }
        const auto close = closes_.find(task_id);
        if (need.close && close != closes_.end()) {
          if (auto item = wire::toSyncItem(wire::encodeTaskClose(item_header, close->second)))
          {
            mine.push_back(std::move(*item));
          }
        }
        std::size_t add = 0;
        for (const wire::SyncItem & item : mine) {
          add += wire::syncItemSize(item);
        }
        // 第一筆一定放得進去（單筆超過上限時照樣送，不會卡住佇列）
        if (!items.empty() && (size + add > config_.max_packet || items.size() + mine.size() > 255)) {
          full = true;
          break;
        }
        size += add;
        for (wire::SyncItem & item : mine) {
          items.push_back(std::move(item));
        }
        taken.emplace_back(queue, task_id);
      }
      if (full) {
        break;
      }
    }
    if (items.empty()) {   // 佇列裡的都已經沒有東西可送（例如結束紀錄不見了）
      for (const auto & [queue, task_id] : taken) {
        queue->erase(task_id);
      }
      if (taken.empty()) {
        repair_open_.clear();
        repair_closed_.clear();
      }
      break;
    }
    if (repair_tokens_ < static_cast<double>(size)) {
      ++stats_.repair_deferred;   // 額度不夠：留到下次
      break;
    }
    repair_tokens_ -= static_cast<double>(size);
    stats_.repair_items_sent += static_cast<int>(items.size());
    send(wire::encodeSyncBatch(newHeader(PacketType::SYNC_BATCH), items), PacketType::SYNC_BATCH,
      TxKind::RETRY);
    for (const auto & [queue, task_id] : taken) {
      queue->erase(task_id);
    }
    if (!full) {
      break;
    }
  }
}

// 收到補發：每一筆照 TASK_ANNOUNCE／TASK_CLOSE 處理（驗證、權限一樣），但不轉送（只在直接鄰居之間補），
// 並登記去重複（之後同一則的轉送不再處理、不再轉送）
void CbbaComm::onSyncBatch(const wire::Header & h, const Bytes & bytes, double now)
{
  const auto items = wire::decodeSyncBatch(bytes);
  if (!items) {
    ++stats_.malformed;
    return;
  }
  for (const wire::SyncItem & item : *items) {
    const Bytes packet = wire::syncItemPacket(item, h);
    if (item.type == PacketType::TASK_ANNOUNCE) {
      const auto b = wire::decodeTaskAnnounce(packet);
      if (!b) {
        ++stats_.malformed;
        continue;
      }
      dedup_.firstSeen(PacketType::TASK_ANNOUNCE, b->relay, now);
      applyAnnounce(*b, now);
    } else {
      const auto b = wire::decodeTaskClose(packet);
      if (!b) {
        ++stats_.malformed;
        continue;
      }
      if (!closeAllowed(*b)) {
        ++stats_.rejected_closes;
        continue;
      }
      dedup_.firstSeen(PacketType::TASK_CLOSE, b->relay, now);
      applyClose(*b, now);
    }
    ++stats_.repair_items_applied;
  }
}

// ===========================================================================
// 鄰居、存活、收斂
// ===========================================================================
std::vector<AgentId> CbbaComm::aliveNeighbors(double now) const
{
  std::vector<AgentId> out;
  for (const auto & [agent, n] : neighbors_) {
    if (now - n.last_heard <= config_.neighbor_timeout) {
      out.push_back(agent);
    }
  }
  return out;
}

std::set<AgentId> CbbaComm::aliveSet(double now) const
{
  std::set<AgentId> out{id()};
  for (AgentId a : aliveNeighbors(now)) {
    out.insert(a);
    const auto & theirs = neighbors_.at(a).neighbors;
    out.insert(theirs.begin(), theirs.end());
  }
  return out;
}

bool CbbaComm::disagrees(double now) const
{
  if (!agent_.active()) {
    return false;
  }
  const auto mine = winnerMap();
  for (AgentId a : aliveNeighbors(now)) {
    const NeighborInfo & n = neighbors_.at(a);
    if (n.participating && n.view != mine) {
      return true;
    }
  }
  return false;
}

bool CbbaComm::converged(double now) const
{
  if (!agent_.active() || stable_steps_ < config_.stable_steps || disagrees(now)) {
    return false;
  }
  for (AgentId a : aliveNeighbors(now)) {
    const NeighborInfo & n = neighbors_.at(a);
    if (n.participating && n.stable_steps < config_.stable_steps) {
      return false;
    }
  }
  return true;
}

// 先和鄰居同步才廣播自己建立的任務（常見的「先追上再上線」）
void CbbaComm::updateJoined(double now)
{
  if (joined_) {
    return;
  }
  const auto alive = aliveNeighbors(now);
  const std::uint32_t key = roundKey();
  const bool synced = std::any_of(alive.begin(), alive.end(),
      [this, key](AgentId a) {
        const NeighborInfo & n = neighbors_.at(a);
        return n.has_view && n.round_key == key;
      });
  if (synced || (alive.empty() && now - start_ >= config_.join_wait)) {
    joined_ = true;   // 已補齊，或附近沒有別台
    return;
  }
  if (now - start_ >= config_.join_timeout) {
    ++stats_.join_timeouts;   // 鄰居之間本來就還沒一致、或一直掉包：照樣開始
    joined_ = true;
  }
}

std::uint32_t CbbaComm::roundKey() const
{
  // 已知任務與狀態的摘要：兩台知道的任務或結束狀態有任何不同，數字就不同
  std::uint32_t crc = 0xFFFFFFFFu;
  for (const auto & [task_id, origin] : origins_) {
    for (int shift = 56; shift >= 0; shift -= 8) {
      crc = crc32Update(crc, static_cast<std::uint8_t>(task_id >> shift));
    }
    crc = crc32Update(crc, isOpen(task_id) ? 0 : 1);
  }
  return ~crc;
}

// ===========================================================================
// 任務
// ===========================================================================
bool CbbaComm::completionPending(TaskId task_id) const
{
  return completions_.count(task_id) > 0;
}

std::optional<Task> CbbaComm::currentTask() const
{
  if (!agent_.active()) {
    return std::nullopt;
  }
  for (TaskId t : agent_.path()) {
    if (completions_.count(t) == 0) {   // 已回報完成、等確認中的不再交給 BT
      return agent_.tasks().at(t);
    }
  }
  return std::nullopt;
}

bool CbbaComm::assign(const Task & task, double now)
{
  if (origins_.count(task.id) > 0) {
    ++stats_.rejected_local_tasks;   // 同步後才發現這個 task_id 已經有了（BT 重用了 task_id）
    return false;
  }
  const wire::Header h = newHeader(PacketType::TASK_ANNOUNCE);
  const wire::RelayHeader origin = wire::originOf(h);
  Task t = task;
  t.status = TaskStatus::OPEN;
  if (!addTask(origin, t, now)) {
    return false;
  }
  dedup_.firstSeen(PacketType::TASK_ANNOUNCE, origin, now);   // 自己發的先登記
  send(wire::encodeTaskAnnounce(h, announceBody(t.id)), PacketType::TASK_ANNOUNCE);
  return true;
}

// 已知任務的定義（origin 沿用建立時的）：建立、保底重播、補發共用
wire::TaskAnnounceBody CbbaComm::announceBody(TaskId task_id) const
{
  const Task & t = agent_.tasks().at(task_id);
  wire::TaskAnnounceBody b;
  b.relay = origins_.at(task_id);
  b.task_id = task_id;
  b.revision = t.revision;
  b.task_type = static_cast<std::uint8_t>(t.type);
  b.platform = t.platform;
  b.required_caps = t.required_caps;
  b.priority = t.priority;
  b.x = static_cast<float>(t.position.x);
  b.y = static_cast<float>(t.position.y);
  b.z = static_cast<float>(t.position.z);
  b.value = static_cast<float>(t.value);
  b.created_ms = toStampMs(t.created);
  b.deadline_ms = t.explicit_deadline ? wire::toDeadlineMs(t.created + t.deadline_sec) : 0;
  b.duration_sec = clampU16(t.duration_sec);
  const TaskExt & e = t.ext;
  b.deadline_hard = e.deadline_hard;
  b.optional = e.optional;
  b.all_or_nothing = e.all_or_nothing;
  b.skip_satisfies = e.skip_satisfies;
  b.floor = e.floor;
  b.release_ms = e.release > 0.0 ? wire::toDeadlineMs(e.release) : 0;
  b.expires_ms = e.expires > 0.0 ? wire::toDeadlineMs(e.expires) : 0;
  b.half_life_s = e.half_life_s;
  b.can_preempt = e.can_preempt;
  b.preempt_margin_pct = e.preempt_margin_pct;
  b.commit_radius_dm = clampU16(e.commit_radius * 10.0);
  b.min_battery_reserve = e.min_battery_reserve;
  b.group_id = e.group_id;
  b.role_index = e.role_index;
  b.role_count = e.role_count;
  b.precedence = e.precedence;
  b.sync_window_s = e.sync_window_s;
  b.optional_no_capable_s = e.optional_no_capable_s;
  b.optional_unassigned_s = e.optional_unassigned_s;
  b.spec_length = e.spec_length;
  b.spec_crc = e.spec_crc;
  return b;
}

bool CbbaComm::addTask(const wire::RelayHeader & origin, const Task & task, double now)
{
  const auto known = origins_.find(task.id);
  if (known != origins_.end()) {
    if (known->second != origin) {
      ++stats_.task_conflicts;   // 同一個 task_id 由別次宣布：保留先收到的
    }
    return false;
  }
  origins_[task.id] = origin;
  added_at_[task.id] = now;
  markChanged(agent_.onTask(task, now));
  dirty_ = true;   // 任務清單變了，鄰居要知道

  const auto early = early_closes_.find(task.id);
  if (early != early_closes_.end()) {
    const std::uint8_t reason = early->second.reason;
    closes_[task.id] = early->second;
    early_closes_.erase(early);
    markClosed(task.id, reason, now);
  }
  return true;
}

void CbbaComm::resendTask(TaskId task_id, double now)
{
  double & last = last_resend_.try_emplace({kindOf(PacketType::TASK_ANNOUNCE), task_id}, -1e9)
    .first->second;
  if (now - last < config_.resend_gap) {
    return;
  }
  last = now;
  // 沿用原始的 origin：已收過的載具會當成重複丟掉
  send(wire::encodeTaskAnnounce(newHeader(PacketType::TASK_ANNOUNCE), announceBody(task_id)),
    PacketType::TASK_ANNOUNCE, TxKind::RETRY);
}

void CbbaComm::resendClose(TaskId task_id, double now)
{
  const auto close = closes_.find(task_id);
  if (close == closes_.end()) {
    return;
  }
  double & last = last_resend_.try_emplace({kindOf(PacketType::TASK_CLOSE), task_id}, -1e9)
    .first->second;
  if (now - last < config_.resend_gap) {
    return;
  }
  last = now;
  send(wire::encodeTaskClose(newHeader(PacketType::TASK_CLOSE), close->second),
    PacketType::TASK_CLOSE, TxKind::RETRY);
}

void CbbaComm::onTaskAnnounce(const Bytes & bytes, double now)
{
  const auto b = wire::decodeTaskAnnounce(bytes);
  if (!b) {
    ++stats_.malformed;
    return;
  }
  if (!dedup_.firstSeen(PacketType::TASK_ANNOUNCE, b->relay, now)) {
    ++stats_.duplicates;
    return;
  }
  if (b->relay.origin_id != id()) {
    relay(bytes);   // 第一次收到時原封不動轉送一次
  }
  applyAnnounce(*b, now);
}

// 任務定義加進本機（TASK_ANNOUNCE、SYNC_BATCH 共用）
void CbbaComm::applyAnnounce(const wire::TaskAnnounceBody & b, double now)
{
  Task t;
  t.id = b.task_id;
  t.type = static_cast<TaskType>(b.task_type);
  t.position = {b.x, b.y, b.z};
  t.value = b.value;
  t.duration_sec = b.duration_sec;
  // 建立時刻用建立者的（各節點、每輪競標都一樣）；沒有期限時出價用預設 300 s，排隊用 max_queue_wait
  t.created = fromStampMs(b.created_ms, now);
  t.status_stamp = now;
  t.explicit_deadline = b.deadline_ms != 0;
  t.deadline_sec = t.explicit_deadline ? wire::fromDeadlineMs(b.deadline_ms, now) - t.created : 300.0;
  t.revision = b.revision;
  t.platform = b.platform;
  t.required_caps = b.required_caps;
  t.priority = b.priority;
  TaskExt & e = t.ext;
  e.deadline_hard = b.deadline_hard;
  e.optional = b.optional;
  e.all_or_nothing = b.all_or_nothing;
  e.skip_satisfies = b.skip_satisfies;
  e.floor = b.floor;
  e.release = b.release_ms != 0 ? wire::fromDeadlineMs(b.release_ms, now) : 0.0;
  e.expires = b.expires_ms != 0 ? wire::fromDeadlineMs(b.expires_ms, now) : 0.0;
  e.half_life_s = b.half_life_s;
  e.can_preempt = b.can_preempt;
  e.preempt_margin_pct = b.preempt_margin_pct;
  e.commit_radius = b.commit_radius_dm / 10.0;
  e.min_battery_reserve = b.min_battery_reserve;
  e.group_id = b.group_id;
  e.role_index = b.role_index;
  e.role_count = b.role_count;
  e.precedence = b.precedence;
  e.sync_window_s = b.sync_window_s;
  e.optional_no_capable_s = b.optional_no_capable_s;
  e.optional_unassigned_s = b.optional_unassigned_s;
  e.spec_length = b.spec_length;
  e.spec_crc = b.spec_crc;
  addTask(b.relay, t, now);
}

// ===========================================================================
// 任務完成
// ===========================================================================
void CbbaComm::sendAnnounce(TaskId task_id, double now)
{
  PendingCompletion & pc = completions_.at(task_id);
  const wire::Header h = newHeader(PacketType::COMPLETION);   // 每次重送都是新序號
  wire::CompletionBody b;
  b.relay = wire::originOf(h);
  b.task_id = task_id;
  b.executor_id = id();
  b.round_key = pc.round_key;
  b.assign_version = pc.version;
  b.view.assign(pc.view.begin(), pc.view.end());
  dedup_.firstSeen(PacketType::COMPLETION, b.relay, now);
  send(wire::encodeCompletion(h, b), PacketType::COMPLETION,
    pc.tries > 0 ? TxKind::RETRY : TxKind::BASE);
  pc.last_sent = now;
  ++pc.tries;
}

bool CbbaComm::allAcked(const PendingCompletion & pc, double now) const
{
  // 要確認的：宣告當下的存活名單 ∩ 現在還活著的（宣告後才失聯的不再等）
  const auto alive = aliveSet(now);
  for (AgentId a : pc.view) {
    if (a != id() && alive.count(a) > 0 && pc.acks.count(a) == 0) {
      return false;
    }
  }
  return true;
}

void CbbaComm::finishCompletion(TaskId task_id, double now)
{
  const auto it = completions_.find(task_id);
  if (it == completions_.end()) {
    return;
  }
  const PendingCompletion pc = it->second;
  completions_.erase(it);

  const wire::Header h = newHeader(PacketType::TASK_CLOSE);
  wire::TaskCloseBody close;
  close.relay = wire::originOf(h);
  close.task_id = task_id;
  close.reason = wire::kCloseDone;
  close.actor_id = id();
  close.round_key = pc.round_key;
  close.assign_version = pc.version;
  close.view.assign(pc.view.begin(), pc.view.end());
  for (const auto & [agent, seq] : pc.acks) {
    close.acks.push_back(wire::AckRef{agent, seq});
  }
  dedup_.firstSeen(PacketType::TASK_CLOSE, close.relay, now);
  closes_[task_id] = close;
  send(wire::encodeTaskClose(h, close), PacketType::TASK_CLOSE);
  markClosed(task_id, wire::kCloseDone, now);
}

void CbbaComm::markClosed(TaskId task_id, std::uint8_t reason, double now)
{
  if (!isOpen(task_id)) {
    return;
  }
  Task done = agent_.tasks().at(task_id);
  done.status = reason == wire::kCloseCancelled ? TaskStatus::CANCELLED : TaskStatus::DONE;
  done.status_stamp = now;
  markChanged(agent_.onTask(done, now));
  closed_at_[task_id] = now;
  dirty_ = true;   // round_key 變了
}

void CbbaComm::onCompletion(const Bytes & bytes, double now)
{
  const auto b = wire::decodeCompletion(bytes);
  if (!b) {
    ++stats_.malformed;
    return;
  }
  if (!dedup_.firstSeen(PacketType::COMPLETION, b->relay, now)) {
    ++stats_.duplicates;
    return;
  }
  if (b->relay.origin_id != id()) {
    relay(bytes);
  }
  if (b->executor_id == id()) {
    return;
  }

  // 回確認：不認得 → 0；知道比它新的正式指派 → 2（舊的回報）；否則接受，並記下它的版本。
  // 同一個宣告的決定不變；每次回送都是新序號
  const auto key = std::make_tuple(b->executor_id, b->task_id, b->assign_version);
  const auto decided = decisions_.find(key);
  std::uint8_t status;
  if (decided != decisions_.end()) {
    status = decided->second;
  } else if (origins_.count(b->task_id) == 0) {
    status = wire::kAckUnknownTask;
  } else {
    const Assignment claim{b->assign_version, b->executor_id, AssignState::ACTIVE};
    if (newerThan(assignment(b->task_id), claim)) {
      status = wire::kAckStaleAssignment;
    } else {
      status = wire::kAckAccepted;
      mergeAssignment(b->task_id, claim);
    }
  }
  decisions_[key] = status;

  const wire::Header h = newHeader(PacketType::COMPLETION_ACK);
  wire::CompletionAckBody ack;
  ack.relay = wire::originOf(h);
  ack.task_id = b->task_id;
  ack.executor_id = b->executor_id;
  ack.round_key = b->round_key;
  ack.assign_version = b->assign_version;
  ack.status = status;
  ack.view = toIds(aliveSet(now));
  dedup_.firstSeen(PacketType::COMPLETION_ACK, ack.relay, now);
  send(wire::encodeCompletionAck(h, ack), PacketType::COMPLETION_ACK);
}

void CbbaComm::onCompletionAck(const Bytes & bytes, double now)
{
  const auto b = wire::decodeCompletionAck(bytes);
  if (!b) {
    ++stats_.malformed;
    return;
  }
  if (!dedup_.firstSeen(PacketType::COMPLETION_ACK, b->relay, now)) {
    ++stats_.duplicates;
    return;
  }

  if (b->executor_id == id()) {
    const auto it = completions_.find(b->task_id);
    if (it == completions_.end() || it->second.version != b->assign_version) {
      return;
    }
    if (b->status == wire::kAckStaleAssignment) {
      // 有載具知道更新的正式指派：這次完成是舊的，作廢（任務保留，不送 TASK_CLOSE）
      ++stats_.stale_acks;
      ++stats_.aborted_completions;
      completions_.erase(it);
      return;
    }
    if (b->status == wire::kAckUnknownTask) {
      ++stats_.rejected_acks;   // 它還不認得這個任務：補發會補上，照樣算確認過
    }
    it->second.acks[b->relay.origin_id] = b->relay.origin_seq;   // 同一台只算一次
    if (allAcked(it->second, now)) {
      finishCompletion(b->task_id, now);
    }
    return;
  }

  // 只在確認者和執行機互相聽不到（或不確定）時轉送；而且自己要聽得到執行機
  const auto alive = aliveNeighbors(now);
  const bool executor_near =
    std::find(alive.begin(), alive.end(), b->executor_id) != alive.end();
  const auto origin = neighbors_.find(b->relay.origin_id);
  const bool origin_hears_executor = origin != neighbors_.end() &&
    now - origin->second.last_heard <= config_.neighbor_timeout &&
    origin->second.neighbors.count(b->executor_id) > 0;
  if (executor_near && !origin_hears_executor) {
    relay(bytes);
  }
}

void CbbaComm::onTaskClose(const Bytes & bytes, double now)
{
  const auto b = wire::decodeTaskClose(bytes);
  if (!b) {
    ++stats_.malformed;
    return;
  }
  if (!dedup_.firstSeen(PacketType::TASK_CLOSE, b->relay, now)) {
    ++stats_.duplicates;
    return;
  }
  // 權限不合的不結束、不轉送
  if (!closeAllowed(*b)) {
    ++stats_.rejected_closes;
    return;
  }
  if (b->relay.origin_id != id()) {
    relay(bytes);
  }
  applyClose(*b, now);
}

// 任務結束套用到本機（TASK_CLOSE、SYNC_BATCH 共用；權限已經檢查過）。
// 執行機是自己也要套用：重開機後，開機前完成的任務靠鄰居補發的 TASK_CLOSE 才知道已結束
void CbbaComm::applyClose(const wire::TaskCloseBody & b, double now)
{
  if (origins_.count(b.task_id) == 0) {
    early_closes_[b.task_id] = b;   // 還不認得這個任務：先存起來
    return;
  }
  if (b.reason == wire::kCloseDone) {
    mergeAssignment(b.task_id, Assignment{b.assign_version, b.actor_id, AssignState::ACTIVE});
  }
  completions_.erase(b.task_id);
  closes_.try_emplace(b.task_id, b);
  markClosed(b.task_id, b.reason, now);
}

// TASK_CLOSE 的權限：actor 是自己發的（等於 origin）；
// 完成：知道的最新正式指派不比它新；取消：actor 是建立者或授權的管理端
bool CbbaComm::closeAllowed(const wire::TaskCloseBody & close) const
{
  if (close.actor_id != close.relay.origin_id) {
    return false;
  }
  if (close.reason == wire::kCloseCancelled) {
    return canCancel(close.actor_id, close.task_id);
  }
  return !newerThan(assignment(close.task_id),
           Assignment{close.assign_version, close.actor_id, AssignState::ACTIVE});
}

}  // namespace cbba_core
