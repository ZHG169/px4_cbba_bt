#include "uav_cbba/cbba_comm.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace uav_cbba
{

using wire::PacketType;

namespace
{
constexpr float kUnknownScore = -1.0f;   // AGENT_STATE 的 y：不認得或已結束

std::uint8_t bitOf(AgentId id)
{
  return wire::neighborBit(id);
}

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
}  // namespace

// ===========================================================================
// 工具
// ===========================================================================
std::string taskName(TaskId id)
{
  char buf[9];
  std::snprintf(buf, sizeof(buf), "%08X", static_cast<unsigned>(id));
  return buf;
}

std::optional<TaskId> parseTaskName(const std::string & name)
{
  if (name.size() != 8) {
    return std::nullopt;
  }
  TaskId id = 0;
  for (char c : name) {
    int v;
    if (c >= '0' && c <= '9') {v = c - '0';} else if (c >= 'A' && c <= 'F') {v = c - 'A' + 10;} else {
      return std::nullopt;
    }
    id = (id << 4) | static_cast<TaskId>(v);
  }
  return id;
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
CbbaComm::CbbaComm(const AgentState & state, const ScoringParams & params, const CommConfig & config)
: agent_(state, params), config_(config)
{
}

void CbbaComm::setStatus(bool armed, bool offboard, bool telemetry_ok)
{
  armed_ = armed;
  offboard_ = offboard;
  telemetry_ok_ = telemetry_ok;
}

void CbbaComm::setSeqStore(
  std::optional<std::uint32_t> stored, std::function<bool(std::uint32_t)> persist)
{
  seq_stored_ = stored;
  seq_persist_ = std::move(persist);
}

void CbbaComm::setParticipating(bool participating, double now)
{
  if (participating == agent_.active()) {
    return;
  }
  markChanged(agent_.setActive(participating, now));
  dirty_ = true;   // 鄰居要馬上知道（M、N 變成 0 或恢復）
}

bool CbbaComm::addLocalTask(const Task & task, double now)
{
  if ((task.id >> 16) != id() || index_of_.count(task.id) > 0 ||
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

bool CbbaComm::reportResult(TaskId task_id, bool success, double now)
{
  const auto index = indexOf(task_id);
  if (!index || !isOpen(*index)) {
    return false;
  }
  if (!success) {
    markChanged(agent_.excludeTask(task_id, now));   // 交回競標池，自己不再出價
    return true;
  }
  if (completions_.count(task_id) > 0) {
    return false;
  }
  PendingCompletion pc;
  pc.index = *index;
  pc.round_key = roundKey();
  pc.revision = exchange_step_;
  pc.view_bits = aliveBits(now);
  completions_[task_id] = pc;
  if (allAcked(pc, now)) {
    finishCompletion(task_id, now);   // 沒有其他存活的飛機
  } else {
    sendAnnounce(task_id, now);
  }
  return true;
}

// ===========================================================================
// 收發
// ===========================================================================
// seq 起點：上次預約的上限之後（一定比開機前用過的大）；沒有紀錄時用開機時的時鐘
void CbbaComm::seedSeq(double now)
{
  if (!seq_seeded_) {
    const std::uint32_t start = seq_stored_ ? *seq_stored_ : toStampMs(now);
    seq_ = wire::SeqCounter(start);
    seq_limit_ = start;   // 還沒預約：第一個封包就會先寫入新上限
    seq_seeded_ = true;
  }
}

wire::Header CbbaComm::newHeader(PacketType type, double now)
{
  seedSeq(now);
  wire::Header h;
  h.type = type;
  h.agent_id = id();
  h.seq = seq_.next(type);
  h.stamp_us = wire::toStampUs(now);
  return h;
}

void CbbaComm::send(Bytes bytes, PacketType type)
{
  // 所有封包都經過這裡：seq 超過預約的上限就先寫入新上限，封包才放進 outbox
  const std::uint32_t seq = wire::parseHeader(bytes)->seq;
  if (static_cast<std::int32_t>(seq - seq_limit_) > 0) {
    seq_limit_ = seq + config_.seq_block;
    ++stats_.seq_reserves;
    if (seq_persist_ && !seq_persist_(seq_limit_)) {
      ++stats_.seq_reserve_failures;
    }
  }
  ++stats_.sent[type];
  stats_.sent_bytes[type] += bytes.size();
  outbox_.push_back(std::move(bytes));
}

void CbbaComm::relay(const Bytes & bytes, double now)
{
  seedSeq(now);
  if (auto fwd = wire::makeRelay(bytes, id(), seq_, now)) {
    const auto type = wire::parseHeader(*fwd)->type;
    ++stats_.relayed;
    send(std::move(*fwd), type);
  }
}

void CbbaComm::receive(const std::uint8_t * data, std::size_t size, double now)
{
  const auto h = wire::parseHeader(data, size);
  if (!h) {
    ++stats_.malformed;
    return;
  }
  if (h->agent_id == id()) {
    return;   // multicast loopback 收到自己的
  }
  if (!seq_filter_.accept(*h, now)) {
    ++stats_.stale;
    return;
  }
  ++stats_.received[h->type];
  const Bytes bytes(data, data + size);
  switch (h->type) {
    case PacketType::AGENT_STATE: onAgentState(*h, bytes, now); break;
    case PacketType::TASK_EVENT: onTaskEvent(bytes, now); break;
    case PacketType::COMPLETION: onCompletion(bytes, now); break;
    case PacketType::COMPLETION_ACK: onCompletionAck(bytes, now); break;
    default: break;   // 其他封包不屬於 CBBA
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

  // 失聯：太久沒有某台的新資訊，釋放它得標的任務
  markChanged(agent_.releaseStale(now, config_.lost_timeout));

  // 和鄰居同步之後：排隊中的任務自編並廣播
  updateJoined(now);
  if (!queued_.empty() && joined_) {
    std::vector<PendingTask> queued;
    queued.swap(queued_);
    for (const PendingTask & p : queued) {
      if (index_of_.count(p.task.id) == 0) {
        assign(p.task, now);
      }
    }
  }

  // 保底：輪流重播一個已知任務（已結束的連同證明）
  if (now >= next_replay_) {
    next_replay_ = now + config_.replay_period;
    if (!meta_.empty() && !aliveNeighbors(now).empty()) {
      auto it = replay_cursor_ < 0 ? meta_.begin() :
        meta_.upper_bound(static_cast<std::uint8_t>(replay_cursor_));
      if (it == meta_.end()) {
        it = meta_.begin();
      }
      replay_cursor_ = it->first;
      resendTask(it->first, now);
      if (!isOpen(it->first) && proofs_.count(it->first) > 0) {
        resendProof(it->first, now);
      }
    }
  }

  // 完成宣告：收齊就送證明；沒收齊每 completion_retry 重送，超過次數就以收到的確認送證明
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
  std::vector<std::uint8_t> z = zVector();
  if (z != last_z_) {
    last_z_ = std::move(z);
    stable_steps_ = 0;
  }

  // AGENT_STATE：週期、變動時立即、和鄰居不一致時快速重送
  const bool periodic = now >= next_periodic_;
  const bool immediate = dirty_ && now - last_state_sent_ >= config_.min_gap;
  const bool fast = !dirty_ && fast_count_ < config_.fast_max &&
    now - last_state_sent_ >= config_.fast_period && disagrees(now);
  if (periodic || immediate || fast) {
    if (periodic) {
      next_periodic_ = std::max(next_periodic_ + config_.state_period, now);
      stable_steps_ = std::min(stable_steps_ + 1, 15);
    }
    if (fast) {
      ++fast_count_;
    }
    sendAgentState(now);
    dirty_ = false;
  }

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
bool CbbaComm::isOpen(std::uint8_t index) const
{
  const auto m = meta_.find(index);
  if (m == meta_.end()) {
    return false;
  }
  const auto t = agent_.tasks().find(m->second.id);
  return t != agent_.tasks().end() && t->second.status == TaskStatus::OPEN;
}

std::size_t CbbaComm::indexCount() const
{
  return meta_.empty() ? 0 : static_cast<std::size_t>(meta_.rbegin()->first) + 1;
}

std::vector<std::uint8_t> CbbaComm::zVector() const
{
  std::vector<std::uint8_t> z(indexCount(), kNoAgent);
  for (const auto & [index, meta] : meta_) {
    if (isOpen(index)) {
      z[index] = agent_.winner(meta.id);
    }
  }
  return z;
}

wire::AgentStateBody CbbaComm::agentStateBody(double now) const
{
  wire::AgentStateBody b;
  b.round_key = roundKey();
  b.exchange_step = exchange_step_;
  b.soc = static_cast<float>(std::min(std::max(agent_.state().battery / 100.0, 0.0), 1.0));
  wire::StateFlags f;
  f.neighbors = neighborBits(now);
  f.armed = armed_;
  f.offboard = offboard_;
  f.telemetry_ok = telemetry_ok_;
  f.stable_steps = static_cast<std::uint8_t>(stable_steps_);
  b.flags = wire::packFlags(f);
  if (const auto current = currentTask()) {
    b.progress_task = index_of_.at(current->id);
  }
  if (!agent_.active()) {
    return b;   // 不參與出價：M、N、Lt 都是 0
  }

  b.z = zVector();
  b.y.assign(b.z.size(), kUnknownScore);
  for (const auto & [index, meta] : meta_) {
    if (isOpen(index)) {
      b.y[index] = static_cast<float>(agent_.score(meta.id));
    }
  }

  AgentId max_id = id();
  for (const auto & [agent, stamp] : agent_.stamps()) {
    max_id = std::max(max_id, agent);
  }
  b.s.assign(max_id, 0);
  for (const auto & [agent, stamp] : agent_.stamps()) {
    if (agent >= 1) {
      b.s[agent - 1] = toStampMs(stamp);
    }
  }
  if (id() >= 1) {
    b.s[id() - 1] = toStampMs(now);
  }

  for (TaskId t : agent_.path()) {
    b.path.push_back(index_of_.at(t));
  }
  return b;
}

void CbbaComm::sendAgentState(double now)
{
  send(wire::encodeAgentState(newHeader(PacketType::AGENT_STATE, now), agentStateBody(now)),
    PacketType::AGENT_STATE);
  last_state_sent_ = now;
}

void CbbaComm::onAgentState(const wire::Header & h, const Bytes & bytes, double now)
{
  const auto b = wire::decodeAgentState(bytes);
  if (!b) {
    ++stats_.malformed;
    return;
  }
  NeighborInfo & n = neighbors_[h.agent_id];
  const bool was_participating = n.participating;
  n.last_heard = now;
  n.participating = !b->s.empty();
  n.neighbor_bits = static_cast<std::uint8_t>(b->flags & 0xFF);
  n.round_key = b->round_key;
  n.z = b->z;
  n.y = b->y;
  n.stable_steps = wire::unpackFlags(b->flags).stable_steps;

  // 補發要在判斷參與之前：還在地面、剛開機的鄰居最需要補
  resendMissing(*b, now);

  if (!n.participating) {
    // 停止出價（降落、重開機）：它得標的任務立即重新分配
    if (was_participating) {
      markChanged(agent_.releaseAgent(h.agent_id, now));
    }
    return;
  }

  // 轉成核心的出價訊息；y < 0 是它不認得或已結束的任務，略過
  CbbaMessage msg;
  msg.stamp = static_cast<double>(h.stamp_us) / 1e6;
  msg.sender = h.agent_id;
  msg.sender_type = AgentType::UAV;
  msg.seq = ++core_seq_[h.agent_id];   // 封包層已過濾舊封包，核心只要遞增的序號
  for (std::size_t i = 0; i < b->z.size(); ++i) {
    const auto m = meta_.find(static_cast<std::uint8_t>(i));
    if (b->y[i] >= 0.0f && m != meta_.end()) {
      msg.bids.push_back(Bid{m->second.id, b->y[i], b->z[i]});
    }
  }
  for (std::size_t k = 0; k < b->s.size(); ++k) {
    const auto agent = static_cast<AgentId>(k + 1);
    if (b->s[k] != 0 && agent != h.agent_id) {
      msg.stamps[agent] = fromStampMs(b->s[k], now);
    }
  }
  markChanged(agent_.onMessage(msg, now));
}

// 補發（anti-entropy）：摘要不同才逐格比對，補對方缺的。
// 不參與出價的鄰居沒有附 y（M = 0），下面每一格都當成「不認得」，等於把所有任務和證明補一次。
void CbbaComm::resendMissing(const wire::AgentStateBody & b, double now)
{
  if (b.round_key == roundKey()) {
    return;
  }
  for (const auto & [index, meta] : meta_) {
    const bool theirs_open = index < b.y.size() && b.y[index] >= 0.0f;
    if (isOpen(index)) {
      if (!theirs_open) {
        resendTask(index, now);    // 它不認得（中間編號漏收也抓得到）
      }
    } else if (theirs_open) {
      resendProof(index, now);     // 它以為還在進行，其實已完成
    } else {
      resendTask(index, now);      // 已結束、它填 −1：分不出不認得或已知結束，一起補
      resendProof(index, now);
    }
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

std::uint8_t CbbaComm::neighborBits(double now) const
{
  std::uint8_t bits = 0;
  for (AgentId a : aliveNeighbors(now)) {
    bits |= bitOf(a);
  }
  return bits;
}

std::uint8_t CbbaComm::aliveBits(double now) const
{
  std::uint8_t bits = bitOf(id());
  for (AgentId a : aliveNeighbors(now)) {
    bits |= bitOf(a) | neighbors_.at(a).neighbor_bits;
  }
  return bits;
}

bool CbbaComm::disagrees(double now) const
{
  if (!agent_.active()) {
    return false;
  }
  const std::vector<std::uint8_t> mine = zVector();
  for (AgentId a : aliveNeighbors(now)) {
    const NeighborInfo & n = neighbors_.at(a);
    if (n.participating && n.z != mine) {
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

// 先和鄰居同步才自編任務編號（常見的「先追上再上線」）
void CbbaComm::updateJoined(double now)
{
  if (joined_) {
    return;
  }
  const auto alive = aliveNeighbors(now);
  const std::uint32_t key = roundKey();
  const bool synced = std::any_of(alive.begin(), alive.end(),
      [this, key](AgentId a) {return neighbors_.at(a).round_key == key;});
  if (synced || (alive.empty() && now - start_ >= config_.join_wait)) {
    joined_ = true;   // 已補齊，或附近沒有別台
    return;
  }
  if (now - start_ >= config_.join_timeout) {
    // 鄰居之間本來就還沒一致、或一直掉包：照樣開始，但不用鄰居已用過的編號
    for (AgentId a : alive) {
      index_floor_ = std::max(index_floor_, neighbors_.at(a).z.size());
    }
    ++stats_.join_timeouts;
    joined_ = true;
  }
}

std::uint32_t CbbaComm::roundKey() const
{
  // 已知任務與狀態的摘要：兩台知道的任務或完成狀態有任何不同，數字就不同
  std::uint32_t crc = 0xFFFFFFFFu;
  for (const auto & [index, meta] : meta_) {
    crc = crc32Update(crc, index);
    crc = crc32Update(crc, isOpen(index) ? 0 : 1);
  }
  return ~crc;
}

// ===========================================================================
// 任務
// ===========================================================================
std::optional<std::uint8_t> CbbaComm::indexOf(TaskId task_id) const
{
  const auto it = index_of_.find(task_id);
  if (it == index_of_.end()) {
    return std::nullopt;
  }
  return it->second;
}

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

std::optional<std::uint8_t> CbbaComm::nextIndex() const
{
  // 自己那一份裡第一個沒用過的編號：slot、slot + index_slots、…
  const int stride = std::max(config_.index_slots, 1);
  for (int i = static_cast<int>(id()) - 1; i >= 0 && i <= wire::kMaxTaskIndex; i += stride) {
    if (static_cast<std::size_t>(i) >= index_floor_ && meta_.count(static_cast<std::uint8_t>(i)) == 0) {
      return static_cast<std::uint8_t>(i);
    }
  }
  return std::nullopt;
}

bool CbbaComm::assign(const Task & task, double now)
{
  const auto index = nextIndex();
  if (!index) {
    return false;
  }
  const wire::Header h = newHeader(PacketType::TASK_EVENT, now);
  const TaskMeta meta{task.id, wire::originOf(h)};
  Task t = task;
  t.status = TaskStatus::OPEN;
  if (!addTask(*index, meta, t, now)) {
    return false;
  }
  dedup_.firstSeen(PacketType::TASK_EVENT, meta.origin, now);   // 自己發的先登記
  wire::TaskEventBody b;
  b.relay = meta.origin;
  b.task_index = *index;
  b.x = static_cast<float>(t.position.x);
  b.y = static_cast<float>(t.position.y);
  b.z = static_cast<float>(t.position.z);
  b.value = static_cast<float>(t.value);
  b.name = taskName(t.id);
  b.ext = wire::TaskExtension{static_cast<std::uint8_t>(t.type),
    wire::toDeadlineMs(t.created + t.deadline_sec), clampU16(t.duration_sec)};
  send(wire::encodeTaskEvent(h, b), PacketType::TASK_EVENT);
  return true;
}

bool CbbaComm::addTask(std::uint8_t index, const TaskMeta & meta, const Task & task, double now)
{
  const auto known = meta_.find(index);
  if (known != meta_.end()) {
    if (known->second.id != meta.id) {
      ++stats_.index_conflicts;   // 保留先收到的
    }
    return false;
  }
  if (index_of_.count(meta.id) > 0) {
    ++stats_.index_conflicts;     // 同一個 task_id 出現在兩個編號
    return false;
  }
  meta_[index] = meta;
  index_of_[meta.id] = index;
  markChanged(agent_.onTask(task, now));
  dirty_ = true;   // 任務清單變了，鄰居要知道

  const auto early = early_proofs_.find(index);
  if (early != early_proofs_.end()) {
    proofs_[index] = early->second;
    early_proofs_.erase(early);
    markDone(index, now);
  }
  return true;
}

Bytes CbbaComm::taskEventPacket(std::uint8_t index, double now)
{
  const TaskMeta & meta = meta_.at(index);
  const Task & t = agent_.tasks().at(meta.id);
  wire::TaskEventBody b;
  b.relay = meta.origin;   // 補發沿用原始的 origin：已收過的飛機會當成重複丟掉
  b.task_index = index;
  b.x = static_cast<float>(t.position.x);
  b.y = static_cast<float>(t.position.y);
  b.z = static_cast<float>(t.position.z);
  b.value = static_cast<float>(t.value);
  b.name = taskName(t.id);
  b.ext = wire::TaskExtension{static_cast<std::uint8_t>(t.type),
    wire::toDeadlineMs(t.created + t.deadline_sec), clampU16(t.duration_sec)};
  return wire::encodeTaskEvent(newHeader(PacketType::TASK_EVENT, now), b);
}

void CbbaComm::resendTask(std::uint8_t index, double now)
{
  double & last = last_resend_.try_emplace({kindOf(PacketType::TASK_EVENT), index}, -1e9)
    .first->second;
  if (now - last < config_.resend_gap) {
    return;
  }
  last = now;
  send(taskEventPacket(index, now), PacketType::TASK_EVENT);
}

void CbbaComm::resendProof(std::uint8_t index, double now)
{
  const auto proof = proofs_.find(index);
  if (proof == proofs_.end()) {
    return;
  }
  double & last = last_resend_.try_emplace({kindOf(PacketType::COMPLETION), index}, -1e9)
    .first->second;
  if (now - last < config_.resend_gap) {
    return;
  }
  last = now;
  send(wire::encodeCompletion(newHeader(PacketType::COMPLETION, now), proof->second),
    PacketType::COMPLETION);
}

void CbbaComm::onTaskEvent(const Bytes & bytes, double now)
{
  const auto b = wire::decodeTaskEvent(bytes);
  if (!b) {
    ++stats_.malformed;
    return;
  }
  if (!dedup_.firstSeen(PacketType::TASK_EVENT, b->relay, now)) {
    ++stats_.duplicates;
    return;
  }
  if (b->relay.origin_id != id()) {
    relay(bytes, now);   // 第一次收到時原封不動轉送一次
  }

  const auto task_id = parseTaskName(b->name);
  if (!task_id) {
    ++stats_.foreign_tasks;
    return;
  }
  // 出價只用到截止時刻：created 記本機收到的時間，deadline_sec = 截止時刻 − created
  Task t;
  t.id = *task_id;
  t.position = {b->x, b->y, b->z};
  t.value = b->value;
  t.created = now;
  t.status_stamp = now;
  if (b->ext) {
    t.type = static_cast<TaskType>(b->ext->task_type);
    t.duration_sec = b->ext->duration_sec;
    t.deadline_sec = b->ext->deadline_ms != 0 ?
      wire::fromDeadlineMs(b->ext->deadline_ms, now) - now : config_.default_deadline;
  } else {
    t.deadline_sec = config_.default_deadline;
  }
  addTask(b->task_index, TaskMeta{*task_id, b->relay}, t, now);
}

// ===========================================================================
// 任務完成
// ===========================================================================
void CbbaComm::sendAnnounce(TaskId task_id, double now)
{
  PendingCompletion & pc = completions_.at(task_id);
  const wire::Header h = newHeader(PacketType::COMPLETION, now);   // 每次重送都是新序號
  wire::CompletionBody b;
  b.relay = wire::originOf(h);
  b.kind = wire::kCompletionAnnounce;
  b.task_index = pc.index;
  b.executor_id = id();
  b.round_key = pc.round_key;
  b.control_revision = pc.revision;
  b.view_bits = pc.view_bits;
  dedup_.firstSeen(PacketType::COMPLETION, b.relay, now);
  send(wire::encodeCompletion(h, b), PacketType::COMPLETION);
  pc.last_sent = now;
  ++pc.tries;
}

bool CbbaComm::allAcked(const PendingCompletion & pc, double now) const
{
  // 要確認的：宣告當下的存活名單 ∩ 現在還活著的（宣告後才失聯的不再等）
  std::uint8_t acked = bitOf(id());
  for (const auto & [agent, seq] : pc.acks) {
    acked |= bitOf(agent);
  }
  const std::uint8_t required = pc.view_bits & aliveBits(now);
  return (required & static_cast<std::uint8_t>(~acked)) == 0;
}

void CbbaComm::finishCompletion(TaskId task_id, double now)
{
  const auto it = completions_.find(task_id);
  if (it == completions_.end()) {
    return;
  }
  const PendingCompletion pc = it->second;
  completions_.erase(it);

  const wire::Header h = newHeader(PacketType::COMPLETION, now);
  wire::CompletionBody proof;
  proof.relay = wire::originOf(h);
  proof.kind = wire::kCompletionProof;
  proof.task_index = pc.index;
  proof.executor_id = id();
  proof.round_key = pc.round_key;
  proof.control_revision = pc.revision;
  proof.view_bits = pc.view_bits;
  for (const auto & [agent, seq] : pc.acks) {
    proof.acks.push_back(wire::AckRef{agent, seq});
  }
  dedup_.firstSeen(PacketType::COMPLETION, proof.relay, now);
  proofs_[pc.index] = proof;
  send(wire::encodeCompletion(h, proof), PacketType::COMPLETION);
  markDone(pc.index, now);
}

void CbbaComm::markDone(std::uint8_t index, double now)
{
  if (!isOpen(index)) {
    return;
  }
  Task done = agent_.tasks().at(meta_.at(index).id);
  done.status = TaskStatus::DONE;
  done.status_stamp = now;
  markChanged(agent_.onTask(done, now));
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
    relay(bytes, now);
  }
  if (b->executor_id == id()) {
    return;
  }

  if (b->kind == wire::kCompletionProof) {
    if (meta_.count(b->task_index) > 0) {
      proofs_[b->task_index] = *b;
      markDone(b->task_index, now);
    } else {
      early_proofs_[b->task_index] = *b;   // 還不認得這個任務：先存起來
    }
    return;
  }

  // 宣告：回確認。同一個宣告的決定不變；每次回送都是新序號
  const auto key = std::make_pair(b->executor_id, b->task_index);
  const auto decided = decisions_.find(key);
  const bool accepted = decided != decisions_.end() ? decided->second : meta_.count(b->task_index) > 0;
  decisions_[key] = accepted;

  const wire::Header h = newHeader(PacketType::COMPLETION_ACK, now);
  wire::CompletionAckBody ack;
  ack.relay = wire::originOf(h);
  ack.task_index = b->task_index;
  ack.executor_id = b->executor_id;
  ack.round_key = b->round_key;
  ack.control_revision = b->control_revision;
  ack.view_bits = aliveBits(now);
  ack.accepted = accepted;
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
    const auto m = meta_.find(b->task_index);
    if (m == meta_.end()) {
      return;
    }
    const auto it = completions_.find(m->second.id);
    if (it == completions_.end()) {
      return;
    }
    if (!b->accepted) {
      ++stats_.rejected_acks;
    }
    it->second.acks[b->relay.origin_id] = b->relay.origin_seq;   // 同一台只算一次
    if (allAcked(it->second, now)) {
      finishCompletion(m->second.id, now);
    }
    return;
  }

  // 規格：只在確認者和執行機互相聽不到（或不確定）時轉送；而且自己要聽得到執行機
  const auto alive = aliveNeighbors(now);
  const bool executor_near =
    std::find(alive.begin(), alive.end(), b->executor_id) != alive.end();
  const auto origin = neighbors_.find(b->relay.origin_id);
  const bool origin_hears_executor = origin != neighbors_.end() &&
    now - origin->second.last_heard <= config_.neighbor_timeout &&
    (origin->second.neighbor_bits & bitOf(b->executor_id)) != 0;
  if (executor_near && !origin_hears_executor) {
    relay(bytes, now);
  }
}

}  // namespace uav_cbba
