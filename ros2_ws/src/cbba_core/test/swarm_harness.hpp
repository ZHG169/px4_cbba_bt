// 測試共用：模擬的廣播網路（誰聽得到誰、掉包、延遲）上跑多個 CbbaComm
#pragma once

#include <gtest/gtest.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "cbba_core/cbba_comm.hpp"

namespace cbba_test
{
using namespace cbba_core;
using wire::PacketType;

struct InFlight
{
  double at;
  std::size_t to;
  Bytes bytes;
};

class Swarm
{
public:
  explicit Swarm(double loss = 0.0, unsigned seed = 1, CommConfig config = CommConfig{})
  : loss_(loss), rng_(seed), config_(config) {}

  CbbaComm & add(AgentId id, Vec3 pos, double battery = 100.0, AgentType type = AgentType::UAV)
  {
    AgentState s = makeState(id, pos, battery);
    s.type = type;
    nodes_.push_back(std::make_unique<CbbaComm>(s, ScoringParams{}, config_));
    online_.push_back(true);
    return *nodes_.back();
  }

  // 模擬重開機：換一個全新的節點（新的 session、什麼都不知道）
  CbbaComm & restart(AgentId id)
  {
    const std::size_t i = indexOf(id);
    const AgentState s = nodes_[i]->agent().state();
    nodes_[i] = std::make_unique<CbbaComm>(s, ScoringParams{}, config_);
    return *nodes_[i];
  }

  CbbaComm & node(AgentId id) {return *nodes_.at(indexOf(id));}
  void setOnline(AgentId id, bool on) {online_[indexOf(id)] = on;}
  void cut(AgentId a, AgentId b) {cut_.insert({a, b}); cut_.insert({b, a});}
  // from 送給 to 的下一則 type 封包會掉
  void dropNext(AgentId from, AgentId to, PacketType type) {drop_.insert({from, to, type});}
  // 錄下 from 送出的封包（之後可以拿來重放）
  void record(AgentId from) {record_from_ = from;}
  // 每個模擬步驟最後呼叫（節點層的事：指派流程、假的 BT）
  void onTick(std::function<void(double)> f) {tick_ = std::move(f);}
  const std::vector<Bytes> & recorded() const {return recorded_;}

  void run(double seconds)
  {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> delay(0.005, 0.02);
    const double end = now_ + seconds;
    for (; now_ < end; now_ += 0.005) {
      std::stable_sort(flight_.begin(), flight_.end(),
        [](const InFlight & a, const InFlight & b) {return a.at < b.at;});
      std::size_t k = 0;
      for (; k < flight_.size() && flight_[k].at <= now_; ++k) {
        if (online_[flight_[k].to]) {
          nodes_[flight_[k].to]->receive(flight_[k].bytes, now_);
        }
      }
      flight_.erase(flight_.begin(), flight_.begin() + static_cast<std::ptrdiff_t>(k));

      for (std::size_t a = 0; a < nodes_.size(); ++a) {
        if (!online_[a]) {
          continue;
        }
        for (Bytes & bytes : nodes_[a]->poll(now_)) {
          bytes_ += bytes.size();
          const PacketType type = wire::parseHeader(bytes)->type;
          const AgentId from = nodes_[a]->id();
          if (from == record_from_) {
            recorded_.push_back(bytes);
          }
          for (std::size_t b = 0; b < nodes_.size(); ++b) {
            const AgentId to = nodes_[b]->id();
            if (b == a || cut_.count({from, to}) > 0 || unit(rng_) < loss_) {
              continue;
            }
            if (drop_.erase({from, to, type}) > 0) {
              continue;
            }
            flight_.push_back(InFlight{now_ + delay(rng_), b, bytes});
          }
        }
      }
      if (tick_) {
        tick_(now_);
      }
    }
  }

  double now() const {return now_;}
  std::size_t bytes() const {return bytes_;}

  // 在線節點的任務清單（task_id、狀態）一致；進行中任務的得標者一致，而且恰好在一台的路徑上
  ::testing::AssertionResult consistent() const
  {
    const CbbaComm * ref = nullptr;
    for (std::size_t a = 0; a < nodes_.size(); ++a) {
      if (!online_[a]) {continue;}
      const CbbaComm & n = *nodes_[a];
      if (ref == nullptr) {ref = &n; continue;}
      if (n.taskCount() != ref->taskCount()) {
        return ::testing::AssertionFailure() << "agent " << n.id() << " knows " <<
               n.taskCount() << " tasks, agent " << ref->id() << " knows " << ref->taskCount();
      }
      for (const auto & [id, task] : ref->agent().tasks()) {
        if (!n.knows(id)) {
          return ::testing::AssertionFailure() << "task " << taskName(id) << " unknown to " << n.id();
        }
        if (n.agent().tasks().at(id).status != task.status) {
          return ::testing::AssertionFailure() << "task " << taskName(id) << " status differs";
        }
        if (task.status == TaskStatus::OPEN && n.agent().winner(id) != ref->agent().winner(id)) {
          return ::testing::AssertionFailure() << "task " << taskName(id) << " winner " <<
                 n.agent().winner(id) << " vs " << ref->agent().winner(id);
        }
      }
    }
    if (ref == nullptr) {
      return ::testing::AssertionSuccess();
    }
    for (const auto & [id, task] : ref->agent().tasks()) {
      if (task.status != TaskStatus::OPEN) {continue;}
      int holders = 0;
      for (std::size_t a = 0; a < nodes_.size(); ++a) {
        const auto & p = nodes_[a]->agent().path();
        if (online_[a] && std::find(p.begin(), p.end(), id) != p.end()) {
          ++holders;
        }
      }
      if (holders > 1) {
        return ::testing::AssertionFailure() << "task " << taskName(id) << " held by " << holders;
      }
    }
    return ::testing::AssertionSuccess();
  }

private:
  static AgentState makeState(AgentId id, Vec3 pos, double battery)
  {
    AgentState s;
    s.id = id;
    s.position = pos;
    s.home = pos;
    s.battery = battery;
    s.energy_per_meter = 0.05;
    s.hover_energy_per_sec = 0.05;
    return s;
  }

  std::size_t indexOf(AgentId id) const
  {
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      if (nodes_[i]->id() == id) {return i;}
    }
    throw std::out_of_range("no such agent");
  }

  double loss_;
  std::mt19937 rng_;
  CommConfig config_;
  double now_{1.7e9};   // 系統時鐘的量級
  std::vector<std::unique_ptr<CbbaComm>> nodes_;
  std::vector<bool> online_;
  std::set<std::pair<AgentId, AgentId>> cut_;
  std::set<std::tuple<AgentId, AgentId, PacketType>> drop_;
  std::vector<InFlight> flight_;
  std::size_t bytes_{0};
  AgentId record_from_{0};
  std::function<void(double)> tick_;
  std::vector<Bytes> recorded_;
};

// BT 送來的新任務：task_id = 機號 × 65536 + 流水號
inline Task newTask(AgentId creator, std::uint16_t seq, double x, double y, double now,
  double value = 80.0, double deadline = 60.0)
{
  Task t;
  t.id = makeTaskId(creator, seq);
  t.position = {x, y, 5.0};
  t.created = now;
  t.deadline_sec = deadline;
  t.value = value;
  t.duration_sec = 5.0;
  return t;
}

// 節點層做的事：正式指派（BT 接受、執行中）
inline std::uint64_t activate(CbbaComm & node, TaskId t)
{
  const std::uint64_t version = node.beginAssignment(t);
  EXPECT_TRUE(node.confirmAssignment(t, version, AssignState::ACTIVE));
  return version;
}

// 正式指派之後，BT 帶著那個版本回報
inline ReportCheck assignAndReport(CbbaComm & node, TaskId t, bool success, double now)
{
  const std::uint64_t version = activate(node, t);
  return node.reportResult(t, success, version, now);
}

inline Task groundTask(AgentId creator, std::uint16_t seq, double x, double y, double now)
{
  Task t = newTask(creator, seq, x, y, now, 100.0, 120.0);
  t.type = TaskType::GROUND_INTERVENTION;
  t.position.z = 0.0;
  return t;
}

}  // namespace cbba_test
