// CbbaComm：經過模擬的廣播網路（誰聽得到誰、掉包、延遲）跑完整的機間協定
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <random>
#include <set>
#include <tuple>

#include "uav_cbba/cbba_comm.hpp"

using namespace uav_cbba;
using wire::PacketType;

namespace
{
struct InFlight
{
  double at;
  std::size_t to;
  Bytes bytes;
};

class Swarm
{
public:
  explicit Swarm(double loss = 0.0, unsigned seed = 1)
  : loss_(loss), rng_(seed) {}

  CbbaComm & add(AgentId id, Vec3 pos, double battery = 100.0)
  {
    nodes_.push_back(std::make_unique<CbbaComm>(makeState(id, pos, battery), ScoringParams{},
      CommConfig{}));
    online_.push_back(true);
    return *nodes_.back();
  }

  // 模擬重開機：換一個全新的節點（seq 從 1 重算、什麼都不知道）
  CbbaComm & restart(AgentId id)
  {
    const std::size_t i = indexOf(id);
    const AgentState s = nodes_[i]->agent().state();
    nodes_[i] = std::make_unique<CbbaComm>(s, ScoringParams{}, CommConfig{});
    return *nodes_[i];
  }

  CbbaComm & node(AgentId id) {return *nodes_.at(indexOf(id));}
  void setOnline(AgentId id, bool on) {online_[indexOf(id)] = on;}
  void cut(AgentId a, AgentId b) {cut_.insert({a, b}); cut_.insert({b, a});}
  // from 送給 to 的下一則 type 封包會掉
  void dropNext(AgentId from, AgentId to, PacketType type) {drop_.insert({from, to, type});}

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
          for (std::size_t b = 0; b < nodes_.size(); ++b) {
            const AgentId from = nodes_[a]->id();
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
    }
  }

  double now() const {return now_;}
  std::size_t bytes() const {return bytes_;}

  // 在線節點的任務清單（編號、task_id、狀態）一致；進行中任務的得標者一致，而且恰好在一台的路徑上
  ::testing::AssertionResult consistent() const
  {
    const CbbaComm * ref = nullptr;
    for (std::size_t a = 0; a < nodes_.size(); ++a) {
      if (!online_[a]) {continue;}
      const CbbaComm & n = *nodes_[a];
      if (ref == nullptr) {ref = &n; continue;}
      if (n.taskCount() != ref->taskCount()) {
        return ::testing::AssertionFailure() << "uav" << int(n.id()) << " knows " <<
               n.taskCount() << " tasks, uav" << int(ref->id()) << " knows " << ref->taskCount();
      }
      for (const auto & [id, task] : ref->agent().tasks()) {
        if (n.indexOf(id) != ref->indexOf(id)) {
          return ::testing::AssertionFailure() << "task " << std::hex << id << " index differs";
        }
        if (n.agent().tasks().at(id).status != task.status) {
          return ::testing::AssertionFailure() << "task " << std::hex << id << " status differs";
        }
        if (task.status == TaskStatus::OPEN && n.agent().winner(id) != ref->agent().winner(id)) {
          return ::testing::AssertionFailure() << "task " << std::hex << id << " winner " <<
                 std::dec << int(n.agent().winner(id)) << " vs " << int(ref->agent().winner(id));
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
        return ::testing::AssertionFailure() << "task " << std::hex << id << " held by " <<
               std::dec << holders;
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
  double now_{1.7e9};   // 系統時鐘的量級
  std::vector<std::unique_ptr<CbbaComm>> nodes_;
  std::vector<bool> online_;
  std::set<std::pair<AgentId, AgentId>> cut_;
  std::set<std::tuple<AgentId, AgentId, PacketType>> drop_;
  std::vector<InFlight> flight_;
  std::size_t bytes_{0};
};

// BT 送來的新任務：task_id = 機號 × 65536 + 流水號
Task newTask(AgentId creator, std::uint16_t seq, double x, double y, double now,
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
}  // namespace

TEST(TaskName, HexRoundTrip)
{
  EXPECT_EQ(taskName(makeTaskId(3, 5)), "00030005");
  EXPECT_EQ(parseTaskName("00030005"), makeTaskId(3, 5));
  EXPECT_EQ(parseTaskName("FFFFFFFF"), 0xFFFFFFFFu);
  EXPECT_FALSE(parseTaskName("0003005"));    // 7 字元
  EXPECT_FALSE(parseTaskName("0003000g"));
  EXPECT_FALSE(parseTaskName("F3-01"));
}

TEST(StampMs, NearNow)
{
  const double now = 1.7e9 + 123.456;
  EXPECT_NEAR(fromStampMs(toStampMs(now - 2.5), now), now - 2.5, 1e-3);
  EXPECT_NEAR(fromStampMs(toStampMs(now + 0.01), now), now, 1e-3);   // 對方時鐘稍快
}

TEST(CbbaComm, TasksSpreadWithInterleavedIndices)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.run(2.0);   // 過了 join_wait

  const double now = swarm.now();
  ASSERT_TRUE(swarm.node(1).addLocalTask(newTask(1, 1, 1, 1, now), now));
  ASSERT_TRUE(swarm.node(1).addLocalTask(newTask(1, 2, 2, 2, now), now));
  ASSERT_TRUE(swarm.node(3).addLocalTask(newTask(3, 1, 19, 1, now), now));
  EXPECT_FALSE(swarm.node(3).addLocalTask(newTask(3, 1, 19, 1, now), now));   // 重複
  EXPECT_FALSE(swarm.node(2).addLocalTask(newTask(1, 9, 0, 0, now), now));    // 建立者不是自己
  swarm.run(2.0);

  const CbbaComm & uav2 = swarm.node(2);
  EXPECT_EQ(uav2.indexOf(makeTaskId(1, 1)), 0);   // uav1：0、8、16…
  EXPECT_EQ(uav2.indexOf(makeTaskId(1, 2)), 8);
  EXPECT_EQ(uav2.indexOf(makeTaskId(3, 1)), 2);   // uav3：2、10、18…
  EXPECT_TRUE(swarm.consistent());
  for (AgentId a : {1, 2, 3}) {
    EXPECT_TRUE(swarm.node(a).converged(swarm.now())) << int(a);
    EXPECT_EQ(swarm.node(a).roundKey(), swarm.node(1).roundKey());
    EXPECT_EQ(swarm.node(a).stats().index_conflicts, 0);
  }
  // 截止時刻在各台相同（created 是各自收到的時間）
  const auto deadline = [&](AgentId a) {
      const Task & t = swarm.node(a).agent().tasks().at(makeTaskId(3, 1));
      return t.created + t.deadline_sec;
    };
  EXPECT_NEAR(deadline(1), deadline(3), 1e-3);
  EXPECT_NEAR(deadline(2), deadline(3), 1e-3);
  EXPECT_EQ(swarm.node(3).currentTask()->id, makeTaskId(3, 1));   // 離 uav3 最近
}

TEST(CbbaComm, AgentStateFields)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {30, 0, 5});
  swarm.run(2.0);
  swarm.node(2).addLocalTask(newTask(2, 1, 28, 0, swarm.now()), swarm.now());
  swarm.run(1.0);

  const auto b = swarm.node(1).agentStateBody(swarm.now());
  ASSERT_EQ(b.z.size(), 2u);           // uav2 的第一個編號是 1，所以 M = 2
  EXPECT_EQ(b.z[0], kNoAgent);
  EXPECT_FLOAT_EQ(b.y[0], -1.0f);      // 編號 0 不存在：不認得
  EXPECT_EQ(b.z[1], 2);
  EXPECT_GT(b.y[1], 0.0f);
  ASSERT_EQ(b.s.size(), 2u);
  EXPECT_NE(b.s[0], 0u);
  EXPECT_NE(b.s[1], 0u);
  EXPECT_EQ(wire::unpackFlags(b.flags).neighbors, 0x02);   // 聽得到 uav2
  EXPECT_TRUE(b.path.empty());
  EXPECT_EQ(b.progress_task, wire::kNoTaskIndex);
  const auto b2 = swarm.node(2).agentStateBody(swarm.now());
  EXPECT_EQ(b2.path, std::vector<std::uint8_t>{1});
  EXPECT_EQ(b2.progress_task, 1);
}

TEST(CbbaComm, UnknownTaskDoesNotEraseWinner)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  swarm.node(2).addLocalTask(newTask(2, 1, 11, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  const TaskId t = makeTaskId(2, 1);
  ASSERT_EQ(swarm.node(1).agent().winner(t), 2);
  swarm.setOnline(2, false);   // 0.3 s 沒聽到 uav2（還不到失聯），讓 uav3 對 uav2 的資訊比較新
  swarm.run(0.3);

  const auto inject = [&](float y, std::uint32_t seq) {
      wire::AgentStateBody b;
      b.z = {0, 0};
      b.y = {-1.0f, y};
      b.s = {0, toStampMs(swarm.now()), toStampMs(swarm.now())};
      wire::Header h;
      h.agent_id = 3;
      h.seq = seq;
      h.stamp_us = wire::toStampUs(swarm.now());
      swarm.node(1).receive(wire::encodeAgentState(h, b), swarm.now());
    };
  inject(-1.0f, 1);   // 不認得：略過
  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);
  swarm.run(0.05);
  inject(0.0f, 2);    // 對照：「認得但沒人得標」依規則 16 會清掉 uav2
  EXPECT_NE(swarm.node(1).agent().winner(t), 2);
}

TEST(CbbaComm, MultiHopRelayAndCompletion)
{
  // 1 — 2 — 3：1 和 3 互相聽不到
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {30, 0, 5});
  swarm.add(3, {60, 0, 5});
  swarm.cut(1, 3);
  swarm.run(2.0);
  swarm.node(3).addLocalTask(newTask(3, 1, 0, 2, swarm.now()), swarm.now());    // 在 uav1 旁邊
  swarm.node(1).addLocalTask(newTask(1, 1, 61, 0, swarm.now()), swarm.now());   // 在 uav3 旁邊
  swarm.run(3.0);
  EXPECT_TRUE(swarm.consistent());
  EXPECT_EQ(swarm.node(3).agent().winner(makeTaskId(3, 1)), 1);
  EXPECT_EQ(swarm.node(1).agent().winner(makeTaskId(1, 1)), 3);
  EXPECT_GT(swarm.node(2).stats().relayed, 0);

  // uav3 完成：uav1 的確認要經 uav2 轉送；第一次轉送故意掉，靠重送（新序號）補回
  swarm.dropNext(2, 3, PacketType::COMPLETION_ACK);
  ASSERT_TRUE(swarm.node(3).reportResult(makeTaskId(1, 1), true, swarm.now()));
  EXPECT_FALSE(swarm.node(3).currentTask());   // 已回報完成：不再交給 BT
  swarm.run(0.5);
  EXPECT_TRUE(swarm.node(3).completionPending(makeTaskId(1, 1)));   // 還缺 uav1 的確認
  swarm.run(1.5);
  EXPECT_FALSE(swarm.node(3).completionPending(makeTaskId(1, 1)));
  for (AgentId a : {1, 2, 3}) {
    EXPECT_EQ(swarm.node(a).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE) << int(a);
  }
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, CompletionSkipsAgentLostAfterAnnounce)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.run(1.0);

  swarm.setOnline(3, false);   // 剛斷線，還在存活名單裡
  ASSERT_TRUE(swarm.node(1).reportResult(makeTaskId(1, 1), true, swarm.now()));
  swarm.run(1.0);
  EXPECT_TRUE(swarm.node(1).completionPending(makeTaskId(1, 1)));
  swarm.run(1.0);              // 判定 uav3 失聯後就不再等它，不用等 10 次重送
  EXPECT_FALSE(swarm.node(1).completionPending(makeTaskId(1, 1)));
  EXPECT_EQ(swarm.node(2).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);
}

TEST(CbbaComm, FailedTaskGoesBackToPool)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(2).agent().winner(t), 1);

  ASSERT_TRUE(swarm.node(1).reportResult(t, false, swarm.now()));   // uav1 做失敗
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);    // 交給 uav2
  EXPECT_EQ(swarm.node(2).currentTask()->id, t);
  EXPECT_FALSE(swarm.node(1).currentTask());
  EXPECT_EQ(swarm.node(1).agent().tasks().at(t).status, TaskStatus::OPEN);
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, NonParticipantDoesNotBid)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.node(3).setParticipating(false, swarm.now());   // 還沒起飛
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 21, 0, swarm.now()), swarm.now());   // 最靠近 uav3
  swarm.run(1.0);

  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);     // uav3 不出價，由次近的 uav2 接
  EXPECT_EQ(swarm.node(3).taskCount(), 1u);          // 但它有收到任務
  EXPECT_FALSE(swarm.node(3).currentTask());
  const auto b = swarm.node(3).agentStateBody(swarm.now());
  EXPECT_EQ(wire::encodeAgentState(wire::Header{wire::PacketType::AGENT_STATE, 1, 3, 1, 0}, b).size(),
    34u);                                            // 規格的「尚未分配」
  EXPECT_TRUE(swarm.node(1).converged(swarm.now()));   // 不參與的鄰居不影響收斂判定

  swarm.node(3).setParticipating(true, swarm.now());  // 起飛
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 3);

  swarm.node(3).setParticipating(false, swarm.now()); // 降落：任務立即交回
  swarm.run(0.5);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);
  EXPECT_EQ(swarm.node(2).currentTask()->id, t);
}

TEST(CbbaComm, LostAgentTasksAreReassigned)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 21, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(1).agent().winner(t), 3);

  swarm.setOnline(3, false);   // 墜毀
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 3);   // 1.5 s 內還不算失聯
  swarm.run(1.5);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);
  EXPECT_EQ(swarm.node(2).currentTask()->id, t);
}

TEST(CbbaComm, MultiHopAgentIsNotLost)
{
  // 1 — 2 — 3：uav1 聽不到 uav3，但 uav3 的時間戳經 uav2 傳來
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {30, 0, 5});
  swarm.add(3, {60, 0, 5});
  swarm.cut(1, 3);
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 61, 0, swarm.now()), swarm.now());
  swarm.run(5.0);
  EXPECT_EQ(swarm.node(1).agent().winner(makeTaskId(1, 1)), 3);
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, MiddleIndexGapIsResent)
{
  // uav3 漏收 uav2 的編號 1，之後收到編號 9：M 一樣是 10，仍要補到編號 1
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.run(2.0);
  swarm.setOnline(3, false);
  swarm.node(2).addLocalTask(newTask(2, 1, 11, 0, swarm.now()), swarm.now());
  swarm.run(0.5);
  swarm.setOnline(3, true);
  swarm.node(2).addLocalTask(newTask(2, 2, 12, 0, swarm.now()), swarm.now());
  swarm.run(2.0);
  EXPECT_EQ(swarm.node(3).indexOf(makeTaskId(2, 1)), 1);
  EXPECT_EQ(swarm.node(3).indexOf(makeTaskId(2, 2)), 9);
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, LateJoinerGetsTasksAndProofs)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.setOnline(3, false);
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.node(2).addLocalTask(newTask(2, 1, 11, 0, swarm.now()), swarm.now());
  swarm.node(2).addLocalTask(newTask(2, 2, 12, 3, swarm.now()), swarm.now());
  swarm.run(1.0);
  swarm.node(1).reportResult(makeTaskId(1, 1), true, swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(2).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);

  swarm.setOnline(3, true);   // 晚起飛：靠補發拿到任務與完成證明
  swarm.run(3.0);
  EXPECT_EQ(swarm.node(3).taskCount(), 3u);
  EXPECT_EQ(swarm.node(3).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);
  EXPECT_EQ(swarm.node(3).roundKey(), swarm.node(1).roundKey());
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, NonParticipantCatchesUpQuickly)
{
  // 還在地面（不出價）的晚加入者沒有附出價表：鄰居看到摘要不同就全部補，不用等每 5 s 的保底重播
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.setOnline(3, false);
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.node(1).addLocalTask(newTask(1, 2, 3, 0, swarm.now()), swarm.now());
  swarm.node(2).addLocalTask(newTask(2, 1, 11, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  swarm.node(1).reportResult(makeTaskId(1, 1), true, swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(2).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);

  swarm.node(3).setParticipating(false, swarm.now());
  swarm.setOnline(3, true);
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(3).taskCount(), 3u);
  EXPECT_EQ(swarm.node(3).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);
  EXPECT_EQ(swarm.node(3).roundKey(), swarm.node(1).roundKey());
}

TEST(CbbaComm, RestartedGroundNodeDoesNotReuseIndex)
{
  // 重開機後還在地面就發現新任務：先和鄰居同步、補回自己以前的任務，才不會再用到編號 1
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  swarm.node(2).addLocalTask(newTask(2, 1, 9, 0, swarm.now()), swarm.now());
  swarm.run(2.0);
  swarm.setOnline(2, false);
  swarm.run(4.0);
  swarm.restart(2).setParticipating(false, swarm.now());
  swarm.setOnline(2, true);
  ASSERT_TRUE(swarm.node(2).addLocalTask(newTask(2, 2, 12, 0, swarm.now()), swarm.now()));
  swarm.run(3.0);

  EXPECT_EQ(swarm.node(2).indexOf(makeTaskId(2, 1)), 1);
  EXPECT_EQ(swarm.node(1).indexOf(makeTaskId(2, 2)), 9);
  EXPECT_EQ(swarm.node(1).stats().index_conflicts, 0);
  EXPECT_EQ(swarm.node(2).stats().index_conflicts, 0);
  EXPECT_EQ(swarm.node(2).stats().join_timeouts, 0);
  EXPECT_EQ(swarm.node(2).roundKey(), swarm.node(1).roundKey());
}

TEST(CbbaComm, JoinTimeoutSkipsNeighborIndices)
{
  // 鄰居的摘要一直對不上（它知道的任務一直補不過來）：等滿 join_timeout 才自編，
  // 而且跳過鄰居 M 以下的編號，不會撞到還沒補到的任務
  Swarm swarm;
  CbbaComm & node = swarm.add(3, {0, 0, 5});
  const double start = swarm.now();
  wire::AgentStateBody b;
  b.round_key = 0x12345678u;   // 和 node 的摘要永遠不同
  b.z.assign(10, 0);           // M = 10：鄰居知道的最大編號是 9
  b.y.assign(10, -1.0f);
  std::uint32_t seq = 0;
  int tick = 0;
  auto step = [&](double t) {
    if (tick++ % 20 == 0) {    // 每 0.2 s 聽到一次
      wire::Header h;
      h.agent_id = 1;
      h.seq = ++seq;
      h.stamp_us = wire::toStampUs(t);
      b.s = {toStampMs(t)};    // N = 1：鄰居參與出價
      node.receive(wire::encodeAgentState(h, b), t);
    }
    node.poll(t);
  };

  double t = start;
  step(t);
  ASSERT_TRUE(node.addLocalTask(newTask(3, 1, 5, 0, t), t));
  for (; t < start + 4.9; t += 0.01) {
    step(t);
  }
  EXPECT_FALSE(node.joined());
  EXPECT_FALSE(node.indexOf(makeTaskId(3, 1)).has_value());
  for (; t < start + 5.2; t += 0.01) {
    step(t);
  }
  EXPECT_TRUE(node.joined());
  EXPECT_EQ(node.stats().join_timeouts, 1);
  EXPECT_EQ(node.indexOf(makeTaskId(3, 1)), 10);   // 第 3 台的編號 2、10、18…，跳過 2
}

TEST(CbbaComm, AloneNodeStartsAfterJoinWait)
{
  // 附近沒有別台：join_wait 後就自編，不用等 join_timeout
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  ASSERT_TRUE(swarm.node(1).addLocalTask(newTask(1, 1, 5, 0, swarm.now()), swarm.now()));
  swarm.run(1.0);
  EXPECT_FALSE(swarm.node(1).indexOf(makeTaskId(1, 1)).has_value());
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(1).indexOf(makeTaskId(1, 1)), 0);
  EXPECT_EQ(swarm.node(1).stats().join_timeouts, 0);
}

// 單獨一台送封包：回傳每一則的 seq
std::vector<std::uint32_t> sentSeqs(CbbaComm & node, double from, double seconds)
{
  std::vector<std::uint32_t> out;
  for (double t = from; t < from + seconds; t += 0.01) {
    for (const Bytes & bytes : node.poll(t)) {
      out.push_back(wire::parseHeader(bytes)->seq);
    }
  }
  return out;
}

AgentState soloState(AgentId id)
{
  AgentState s;
  s.id = id;
  s.position = {0, 0, 5};
  s.home = s.position;
  return s;
}

TEST(CbbaComm, SeqReservedBeforeUse)
{
  // 每個離開 outbox 的封包，seq 都不超過「當時已寫入」的上限；用完就續約
  CommConfig cc;
  cc.seq_block = 10;
  CbbaComm node(soloState(1), ScoringParams{}, cc);
  std::vector<std::uint32_t> written;
  node.setSeqStore(std::nullopt, [&written](std::uint32_t limit) {
      written.push_back(limit);
      return true;
    });
  const double t0 = 1.7e9;
  for (double t = t0; t < t0 + 5.0; t += 0.01) {
    for (const Bytes & bytes : node.poll(t)) {
      ASSERT_FALSE(written.empty());
      EXPECT_LE(wire::parseHeader(bytes)->seq, written.back());
    }
  }
  EXPECT_GE(written.size(), 2u);   // 5 s 約 25 則 AGENT_STATE，10 號一個區塊
  EXPECT_EQ(node.stats().seq_reserves, static_cast<int>(written.size()));
}

TEST(CbbaComm, SeqContinuesAfterRestartEvenIfClockGoesBack)
{
  // 重開機時時鐘往回跳 10 s（樹莓派沒有 RTC、還沒對時）：seq 照樣接著上次預約的上限
  std::uint32_t stored = 0;
  auto persist = [&stored](std::uint32_t limit) {stored = limit; return true;};
  const double t0 = 1.7e9;

  CbbaComm before(soloState(2), ScoringParams{}, CommConfig{});
  before.setSeqStore(std::nullopt, persist);
  const auto old_seqs = sentSeqs(before, t0, 3.0);
  ASSERT_FALSE(old_seqs.empty());

  CbbaComm after(soloState(2), ScoringParams{}, CommConfig{});
  after.setSeqStore(stored, persist);
  const auto new_seqs = sentSeqs(after, t0 - 10.0, 1.0);
  ASSERT_FALSE(new_seqs.empty());
  const auto old_max = *std::max_element(old_seqs.begin(), old_seqs.end());
  for (std::uint32_t seq : new_seqs) {
    EXPECT_GT(static_cast<std::int32_t>(seq - old_max), 0);
  }
}

TEST(CbbaComm, ClockStartWithoutStore)
{
  // 沒有紀錄：用時鐘起點（第一次開機）
  CbbaComm node(soloState(1), ScoringParams{}, CommConfig{});
  const double t0 = 1.7e9;
  const auto seqs = sentSeqs(node, t0, 0.5);
  ASSERT_FALSE(seqs.empty());
  EXPECT_EQ(seqs.front(), toStampMs(t0) + 1);
}

TEST(CbbaComm, SeqStoreFailureKeepsSending)
{
  // 寫入失敗只記數，不擋封包（節點不能因為儲存壞掉就停擺）
  CbbaComm node(soloState(1), ScoringParams{}, CommConfig{});
  node.setSeqStore(std::nullopt, [](std::uint32_t) {return false;});
  EXPECT_FALSE(sentSeqs(node, 1.7e9, 1.0).empty());
  EXPECT_GE(node.stats().seq_reserve_failures, 1);
}

TEST(CbbaComm, RestartedNodeRejoins)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.node(2).addLocalTask(newTask(2, 1, 9, 0, swarm.now()), swarm.now());
  swarm.run(2.0);

  swarm.setOnline(2, false);
  swarm.run(4.0);
  swarm.restart(2);            // 重開機：seq 從 1 重算、任務清單是空的
  swarm.setOnline(2, true);
  // 開機馬上發現新任務：先和鄰居同步、補齊既有任務才自編，不會撞到編號 1
  ASSERT_TRUE(swarm.node(2).addLocalTask(newTask(2, 2, 12, 0, swarm.now()), swarm.now()));
  swarm.run(4.0);

  EXPECT_EQ(swarm.node(1).taskCount(), 3u);
  EXPECT_EQ(swarm.node(1).indexOf(makeTaskId(2, 1)), 1);
  EXPECT_EQ(swarm.node(1).indexOf(makeTaskId(2, 2)), 9);
  EXPECT_EQ(swarm.node(1).agent().winner(makeTaskId(2, 1)), 2);
  EXPECT_EQ(swarm.node(1).stats().index_conflicts, 0);
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, ConvergesUnderLoss)
{
  for (double loss : {0.1, 0.3}) {
    int ok = 0;
    const int trials = 10;
    for (int trial = 0; trial < trials; ++trial) {
      Swarm swarm(loss, 100 + trial);
      for (AgentId a = 1; a <= 5; ++a) {
        swarm.add(a, {10.0 * a, (a % 2) * 15.0, 5.0}, 40.0 + 12.0 * a);
      }
      swarm.run(2.0);
      for (AgentId a = 1; a <= 5; ++a) {
        swarm.node(a).addLocalTask(newTask(a, 1, 12.0 * a, 8.0, swarm.now()), swarm.now());
      }
      swarm.node(3).addLocalTask(newTask(3, 2, 25, 25, swarm.now(), 160.0), swarm.now());
      swarm.run(5.0);
      if (swarm.consistent() && swarm.node(1).taskCount() == 6) {
        ++ok;
      }
    }
    EXPECT_EQ(ok, trials) << "loss " << loss;
  }
}

TEST(CbbaComm, QuietWhenConverged)
{
  Swarm swarm;
  for (AgentId a = 1; a <= 5; ++a) {
    swarm.add(a, {10.0 * a, 0, 5});
  }
  swarm.run(2.0);
  for (AgentId a = 1; a <= 5; ++a) {
    swarm.node(a).addLocalTask(newTask(a, 1, 10.0 * a + 1, 0, swarm.now()), swarm.now());
  }
  swarm.run(3.0);
  ASSERT_TRUE(swarm.consistent());

  // 收斂後：每架每秒 5 則 AGENT_STATE（5 架 5 任務、Lt = 1：80 B），加上每 5 s 一則保底重播
  const std::size_t before = swarm.bytes();
  swarm.run(10.0);
  const double per_agent_per_sec = (swarm.bytes() - before) / 10.0 / 5.0;
  EXPECT_NEAR(per_agent_per_sec, 400.0 + 56.0 / 5.0, 20.0);
  for (AgentId a = 1; a <= 5; ++a) {
    EXPECT_TRUE(swarm.node(a).converged(swarm.now()));
  }
}
