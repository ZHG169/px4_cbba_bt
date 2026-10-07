// 單一載具的共識行為：連鎖退標、任務終止、訊息過濾、重新評估
#include <gtest/gtest.h>

#include <algorithm>

#include "uav_cbba/cbba_agent.hpp"

using namespace uav_cbba;

namespace
{
Task makeTask(TaskId id, double x, double y, TaskType type = TaskType::AIR_RECON)
{
  Task t;
  t.id = id;
  t.type = type;
  t.position = {x, y, 0.0};
  t.deadline_sec = 120.0;
  t.value = 80.0;
  return t;
}

AgentState makeUav(AgentId id, double x, double y, double battery = 200.0)
{
  AgentState a;
  a.id = id;
  a.type = AgentType::UAV;
  a.position = {x, y, 0.0};
  a.home = a.position;
  a.battery = battery;
  a.safety_reserve = 20.0;
  a.energy_per_meter = 0.5;
  a.hover_energy_per_sec = 0.2;
  return a;
}

bool contains(const std::vector<TaskId> & v, TaskId id)
{
  return std::find(v.begin(), v.end(), id) != v.end();
}

// 兩台互相傳訊息，直到雙方都沒有改變
void exchangeUntilQuiet(CbbaAgent & a, CbbaAgent & b, double now)
{
  for (int round = 0; round < 50; ++round) {
    const bool cb = b.onMessage(a.makeMessage(now), now);
    const bool ca = a.onMessage(b.makeMessage(now), now);
    now += 0.01;
    if (!ca && !cb) {return;}
  }
  FAIL() << "did not settle within 50 rounds";
}
}  // namespace

TEST(Agent, TakesAllCompatibleTasksWhenAlone)
{
  CbbaAgent uav(makeUav(1, 0, 0), ScoringParams{});
  EXPECT_TRUE(uav.onTask(makeTask(101, 10, 0), 0.0));
  EXPECT_TRUE(uav.onTask(makeTask(102, 20, 0), 0.0));
  EXPECT_FALSE(uav.onTask(makeTask(103, 5, 5, TaskType::GROUND_INTERVENTION), 0.0));
  EXPECT_EQ(uav.bundle().size(), 2u);
  EXPECT_EQ(uav.winner(101), 1);
  EXPECT_EQ(uav.winner(103), kNoAgent);
  EXPECT_DOUBLE_EQ(uav.score(103), 0.0);
}

TEST(Agent, BidsNeverIncreaseAlongTheBundle)
{
  // 介面規格的條件 3：已接的任務越多，出價只能持平或降低。
  // 任務刻意排成一直線，後面的任務都「順路」，原始分數會變高；出價上限要把它壓住。
  CbbaAgent uav(makeUav(1, 0, 0), ScoringParams{});
  for (int k = 0; k < 5; ++k) {
    uav.onTask(makeTask(static_cast<TaskId>(101 + k), 40.0 + 2.0 * k, 0), 0.0);
  }
  ASSERT_EQ(uav.bundle().size(), 5u);
  for (std::size_t b = 1; b < uav.bundle().size(); ++b) {
    EXPECT_LE(uav.score(uav.bundle()[b]), uav.score(uav.bundle()[b - 1]) + 1e-12) << "index " << b;
  }
}

TEST(Agent, TwoAgentsAgreeWithoutDuplicates)
{
  CbbaAgent a(makeUav(1, 0, 0), ScoringParams{});
  CbbaAgent b(makeUav(2, 100, 0), ScoringParams{});
  const std::vector<Task> tasks{
    makeTask(101, 10, 0), makeTask(102, 90, 0), makeTask(103, 30, 10), makeTask(104, 70, -10)};
  for (const Task & t : tasks) {
    a.onTask(t, 0.0);
    b.onTask(t, 0.0);
  }
  exchangeUntilQuiet(a, b, 0.0);

  for (const Task & t : tasks) {
    EXPECT_EQ(a.winner(t.id), b.winner(t.id)) << "task " << t.id;
    EXPECT_NE(a.winner(t.id), kNoAgent) << "task " << t.id;
    EXPECT_FALSE(contains(a.bundle(), t.id) && contains(b.bundle(), t.id)) << "task " << t.id;
    EXPECT_NEAR(a.score(t.id), b.score(t.id), 1e-9) << "task " << t.id;
  }
  // 靠近各自起點的任務應歸各自所有
  EXPECT_EQ(a.winner(101), 1);
  EXPECT_EQ(a.winner(102), 2);
}

TEST(Agent, CascadeReleaseWhenOutbidInTheMiddle)
{
  ScoringParams params;
  CbbaAgent uav(makeUav(1, 0, 0), params);
  uav.onTask(makeTask(101, 10, 0), 0.0);
  uav.onTask(makeTask(102, 20, 0), 0.0);
  uav.onTask(makeTask(103, 30, 0), 0.0);
  ASSERT_EQ(uav.bundle().size(), 3u);
  const TaskId first = uav.bundle()[0];
  const TaskId middle = uav.bundle()[1];
  const TaskId last = uav.bundle()[2];
  const double last_score_before = uav.score(last);

  // 另一台以更高的分數拿走中間的任務
  CbbaMessage msg;
  msg.sender = 2;
  msg.seq = 1;
  msg.bids.push_back(Bid{middle, 1000.0, 2});
  EXPECT_TRUE(uav.onMessage(msg, 1.0));

  EXPECT_EQ(uav.winner(middle), 2);
  EXPECT_FALSE(contains(uav.bundle(), middle));
  EXPECT_FALSE(contains(uav.path(), middle));
  EXPECT_EQ(uav.bundle()[0], first);            // 被搶之前的任務保留
  // 之後的任務被釋放並重新出價：仍然是自己的，但分數是重新算的
  EXPECT_TRUE(contains(uav.bundle(), last));
  EXPECT_EQ(uav.winner(last), 1);
  EXPECT_NE(uav.score(last), last_score_before);
}

TEST(Agent, BidForUnknownTaskIsSkipped)
{
  CbbaAgent uav(makeUav(1, 0, 0), ScoringParams{});
  CbbaMessage msg;
  msg.sender = 2;
  msg.seq = 1;
  msg.bids.push_back(Bid{999, 50.0, 2});
  EXPECT_FALSE(uav.onMessage(msg, 0.0));
  EXPECT_EQ(uav.winner(999), kNoAgent);
}

TEST(Agent, ClosedTaskLeavesTheBundle)
{
  CbbaAgent uav(makeUav(1, 0, 0), ScoringParams{});
  uav.onTask(makeTask(101, 10, 0), 0.0);
  uav.onTask(makeTask(102, 20, 0), 0.0);
  ASSERT_TRUE(contains(uav.bundle(), 101));

  Task closed = makeTask(101, 10, 0);
  closed.status = TaskStatus::CANCELLED;
  closed.status_stamp = 5.0;
  EXPECT_TRUE(uav.onTask(closed, 5.0));
  EXPECT_FALSE(contains(uav.bundle(), 101));
  EXPECT_TRUE(contains(uav.bundle(), 102));

  // 終止後不會被舊的 OPEN 訊息重新打開，也不會出現在廣播的出價裡
  EXPECT_FALSE(uav.onTask(makeTask(101, 10, 0), 6.0));
  for (const Bid & bid : uav.makeMessage(6.0).bids) {
    EXPECT_NE(bid.task_id, 101u);
  }
}

TEST(Agent, StaleSequenceIsIgnored)
{
  CbbaAgent uav(makeUav(1, 0, 0), ScoringParams{});
  uav.onTask(makeTask(101, 10, 0), 0.0);

  CbbaMessage newer;
  newer.sender = 2;
  newer.seq = 5;
  newer.bids.push_back(Bid{101, 1000.0, 2});
  EXPECT_TRUE(uav.onMessage(newer, 1.0));
  EXPECT_EQ(uav.winner(101), 2);

  // 延遲到達的舊訊息（seq 較小）說 2 號沒有得標：應被丟棄
  CbbaMessage older;
  older.sender = 2;
  older.seq = 4;
  older.bids.push_back(Bid{101, 0.0, kNoAgent});
  EXPECT_FALSE(uav.onMessage(older, 1.1));
  EXPECT_EQ(uav.winner(101), 2);
}

TEST(Agent, WinnerLoweringItsScoreLetsOthersTakeOver)
{
  CbbaAgent uav(makeUav(1, 0, 0), ScoringParams{});
  uav.onTask(makeTask(101, 10, 0), 0.0);
  const double my_score = uav.score(101);

  CbbaMessage high;
  high.sender = 2;
  high.seq = 1;
  high.bids.push_back(Bid{101, my_score + 10.0, 2});
  uav.onMessage(high, 1.0);
  ASSERT_EQ(uav.winner(101), 2);

  // 2 號落後了，調低自己的分數（規則 2：直接接受）-> 1 號的分數較高，接手
  CbbaMessage low;
  low.sender = 2;
  low.seq = 2;
  low.bids.push_back(Bid{101, my_score - 10.0, 2});
  EXPECT_TRUE(uav.onMessage(low, 2.0));
  EXPECT_EQ(uav.winner(101), 1);
  EXPECT_TRUE(contains(uav.bundle(), 101));
}

TEST(Agent, CompleteTaskMarksDoneAndKeepsTheRest)
{
  CbbaAgent uav(makeUav(1, 0, 0), ScoringParams{});
  uav.onTask(makeTask(101, 10, 0), 0.0);
  uav.onTask(makeTask(102, 20, 0), 0.0);
  const TaskId first = uav.path().front();
  const auto done = uav.completeTask(first, 10.0);
  ASSERT_TRUE(done.has_value());
  EXPECT_EQ(done->status, TaskStatus::DONE);
  EXPECT_DOUBLE_EQ(done->status_stamp, 10.0);
  EXPECT_EQ(uav.bundle().size(), 1u);
  EXPECT_FALSE(uav.completeTask(first, 11.0).has_value());
}

TEST(Agent, ReevaluateReleasesTasksWhenBatteryDrops)
{
  CbbaAgent uav(makeUav(1, 0, 0, 100.0), ScoringParams{});
  uav.onTask(makeTask(101, 30, 0), 0.0);
  ASSERT_TRUE(contains(uav.bundle(), 101));

  // 實際耗電比預估多：電量掉到做不完也回不來
  AgentState low = uav.state();
  low.battery = 30.0;
  uav.setState(low);
  EXPECT_TRUE(uav.reevaluate(5.0));
  EXPECT_TRUE(uav.bundle().empty());
  EXPECT_EQ(uav.winner(101), kNoAgent);
  EXPECT_DOUBLE_EQ(uav.score(101), 0.0);
}

TEST(Agent, ReevaluateIgnoresSmallScoreChanges)
{
  CbbaAgent uav(makeUav(1, 0, 0), ScoringParams{});
  uav.onTask(makeTask(101, 30, 0), 0.0);
  const double before = uav.score(101);

  AgentState moved = uav.state();
  moved.position = {0.2, 0.0, 0.0};   // 幾乎沒動
  uav.setState(moved);
  EXPECT_FALSE(uav.reevaluate(0.1));
  EXPECT_DOUBLE_EQ(uav.score(101), before);
}

TEST(Agent, LostNeighborsAfterTimeout)
{
  CbbaAgent uav(makeUav(1, 0, 0), ScoringParams{});
  CbbaMessage msg;
  msg.sender = 2;
  msg.seq = 1;
  uav.onMessage(msg, 10.0);
  EXPECT_TRUE(uav.lostNeighbors(14.0, 5.0).empty());
  ASSERT_EQ(uav.lostNeighbors(15.5, 5.0).size(), 1u);
  EXPECT_EQ(uav.lostNeighbors(15.5, 5.0)[0], 2);
}

TEST(Agent, InactiveAgentDoesNotBidAndReleasesOnStop)
{
  AgentState s;
  s.id = 1;
  CbbaAgent uav(s, ScoringParams{});
  Task t;
  t.id = 7;
  t.position = {5, 0, 0};
  EXPECT_TRUE(uav.onTask(t, 0.0));
  ASSERT_EQ(uav.bundle().size(), 1u);

  EXPECT_TRUE(uav.setActive(false, 1.0));   // 停止：全部釋放
  EXPECT_TRUE(uav.bundle().empty());
  EXPECT_EQ(uav.winner(7), kNoAgent);
  EXPECT_FALSE(uav.reevaluate(2.0));        // 不參與時不出價
  EXPECT_TRUE(uav.bundle().empty());

  EXPECT_TRUE(uav.setActive(true, 3.0));    // 恢復：重新出價
  EXPECT_EQ(uav.winner(7), 1);
}

TEST(Agent, ReleaseAgentAndStale)
{
  AgentState s;
  s.id = 1;
  s.position = {100, 0, 0};
  CbbaAgent uav(s, ScoringParams{});
  Task t;
  t.id = 7;
  t.position = {0, 0, 0};
  uav.onTask(t, 0.0);

  CbbaMessage msg;
  msg.sender = 2;
  msg.seq = 1;
  msg.bids.push_back(Bid{7, 79.0, 2});
  uav.onMessage(msg, 10.0);
  ASSERT_EQ(uav.winner(7), 2);

  EXPECT_FALSE(uav.releaseStale(11.0, 1.5));  // 1 s：還不算失聯
  EXPECT_EQ(uav.winner(7), 2);
  EXPECT_TRUE(uav.releaseStale(11.6, 1.5));   // 1.6 s 沒有 uav2 的新資訊
  EXPECT_EQ(uav.winner(7), 1);                // 自己接手

  EXPECT_FALSE(uav.releaseAgent(1, 12.0));    // 不能釋放自己
}

TEST(Agent, ExcludedTaskIsNotBidAgain)
{
  AgentState s;
  s.id = 1;
  CbbaAgent uav(s, ScoringParams{});
  Task a;
  a.id = 7;
  a.position = {5, 0, 0};
  Task b;
  b.id = 8;
  b.position = {6, 0, 0};
  uav.onTask(a, 0.0);
  uav.onTask(b, 0.0);
  ASSERT_EQ(uav.bundle().size(), 2u);

  EXPECT_TRUE(uav.excludeTask(7, 1.0));       // 做任務 7 失敗
  EXPECT_TRUE(uav.excluded(7));
  EXPECT_EQ(uav.winner(7), kNoAgent);         // 交回競標池
  EXPECT_EQ(uav.bundle(), std::vector<TaskId>{8});   // 其他任務重新接回
  uav.reevaluate(2.0);
  EXPECT_EQ(uav.winner(7), kNoAgent);         // 之後也不再出價
}
