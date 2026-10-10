// 核心的執行狀態限制（2026-10-09）：已接受的任務釘住、搶占、火警排隊（數量、期限）、鎖定、暫時撤回
#include <gtest/gtest.h>

#include <vector>

#include "cbba_core/cbba_agent.hpp"

using namespace cbba_core;

namespace
{
constexpr double kNow = 1000.0;

// 狗：1 m/s、耗電很小（電量不是限制）
AgentState makeDog()
{
  AgentState a;
  a.id = 50;
  a.type = AgentType::UGV;
  a.position = {0, 0, 0};
  a.home = a.position;
  a.battery = 100.0;
  a.safety_reserve = 10.0;
  a.energy_per_meter = 0.01;
  a.hover_energy_per_sec = 0.01;
  a.cruise_speed = 1.0;
  return a;
}

Task fire(TaskId id, double x, double y, double created = kNow, double deadline = 300.0,
  bool explicit_deadline = true)
{
  Task t;
  t.id = id;
  t.type = TaskType::GROUND_INTERVENTION;
  t.position = {x, y, 0};
  t.created = created;
  t.deadline_sec = deadline;
  t.explicit_deadline = explicit_deadline;
  t.value = 100.0;
  t.duration_sec = 20.0;
  return t;
}

Task patrol(TaskId id, double x, double y)
{
  Task t = fire(id, x, y);
  t.type = TaskType::PATROL;
  t.value = 80.0;
  return t;
}

// 先讓狗接下 first，再把它設成執行中（已接受）
CbbaAgent dogWithActive(const Task & first, bool preemptible, double remaining)
{
  CbbaAgent dog(makeDog(), ScoringParams{});
  dog.onTask(first, kNow);
  EXPECT_EQ(dog.path(), std::vector<TaskId>{first.id});
  ExecConstraints c;
  c.pinned = {first.id};
  c.active_pinned = true;
  c.preemptible = preemptible;
  c.active_remaining = remaining;
  dog.setExecConstraints(c, kNow);
  return dog;
}
}  // namespace

TEST(ExecConstraints, SamePriorityDoesNotPreempt)
{
  // 正在處理火警 1（遠）：近的火警 2 排在後面，不中斷火警 1
  CbbaAgent dog = dogWithActive(fire(1, 30, 0), true, 40.0);
  dog.onTask(fire(2, 3, 0), kNow);
  EXPECT_EQ(dog.path(), (std::vector<TaskId>{1, 2}));
}

TEST(ExecConstraints, HigherPriorityPreemptsWhenPreemptible)
{
  // 巡檢中、可以安全中斷：火警排到最前面（等 BT 接受才真的切換）
  CbbaAgent dog = dogWithActive(patrol(1, 30, 0), true, 40.0);
  dog.onTask(fire(2, 3, 0), kNow);
  EXPECT_EQ(dog.path(), (std::vector<TaskId>{2, 1}));
  // 重新評估後還是在最前面，不會來回切換
  dog.reevaluate(kNow + 1.0);
  EXPECT_EQ(dog.path(), (std::vector<TaskId>{2, 1}));
}

TEST(ExecConstraints, NotPreemptibleQueuesBehindActive)
{
  // 不可中斷階段：火警排在後面（排隊）；走到可以中斷的地方時，馬上排到前面
  CbbaAgent dog = dogWithActive(patrol(1, 30, 0), false, 40.0);
  dog.onTask(fire(2, 3, 0), kNow);
  EXPECT_EQ(dog.path(), (std::vector<TaskId>{1, 2}));

  ExecConstraints c = dog.execConstraints();
  c.pinned = {1, 2};   // 火警已保留
  dog.setExecConstraints(c, kNow);
  EXPECT_EQ(dog.path(), (std::vector<TaskId>{1, 2}));
  c.preemptible = true;
  dog.setExecConstraints(c, kNow + 1.0);
  EXPECT_EQ(dog.path(), (std::vector<TaskId>{2, 1}));
}

TEST(ExecConstraints, QueueNeedsKnownRemainingTime)
{
  // 執行中任務的剩餘時間不知道：不接受排隊
  CbbaAgent dog = dogWithActive(fire(1, 30, 0), false, -1.0);
  dog.onTask(fire(2, 3, 0), kNow);
  EXPECT_EQ(dog.path(), std::vector<TaskId>{1});
}

TEST(ExecConstraints, AtMostOneQueuedFire)
{
  // 執行中的火警之外最多排 1 個火警；巡檢不受這個限制
  CbbaAgent dog = dogWithActive(fire(1, 10, 0), false, 20.0);
  dog.onTask(fire(2, 12, 0), kNow);
  dog.onTask(fire(3, 14, 0), kNow);
  dog.onTask(patrol(4, 16, 0), kNow);
  const auto & path = dog.path();
  ASSERT_EQ(path.front(), 1u);
  int fires = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    fires += (path[i] == 2 || path[i] == 3) ? 1 : 0;
  }
  EXPECT_EQ(fires, 1);
  EXPECT_NE(std::find(path.begin(), path.end(), 4u), path.end());
}

TEST(ExecConstraints, QueueMustFinishBeforeDeadline)
{
  // 等待（剩 100 s）＋前往 3 m（約 4 s）＋處置 20 s = 124 s：期限 120 s 不行、130 s 可以
  CbbaAgent tight = dogWithActive(fire(1, 0, 0), false, 100.0);
  tight.onTask(fire(2, 3, 0, kNow, 120.0), kNow);
  EXPECT_EQ(tight.path(), std::vector<TaskId>{1});

  CbbaAgent ok = dogWithActive(fire(1, 0, 0), false, 100.0);
  ok.onTask(fire(2, 3, 0, kNow, 130.0), kNow);
  EXPECT_EQ(ok.path(), (std::vector<TaskId>{1, 2}));
}

TEST(ExecConstraints, NoDeadlineUsesMaxWaitFromCreation)
{
  // 沒有期限：最大等待 120 s 從建立時算。剛建立的可以排；100 s 前建立的（只剩 20 s）不行
  CbbaAgent fresh = dogWithActive(fire(1, 0, 0), false, 60.0);
  fresh.onTask(fire(2, 3, 0, kNow, 300.0, false), kNow);
  EXPECT_EQ(fresh.path(), (std::vector<TaskId>{1, 2}));

  CbbaAgent old = dogWithActive(fire(1, 0, 0), false, 60.0);
  old.onTask(fire(2, 3, 0, kNow - 100.0, 300.0, false), kNow);
  EXPECT_EQ(old.path(), std::vector<TaskId>{1});
  EXPECT_DOUBLE_EQ(old.queueDeadline(fire(2, 3, 0, kNow - 100.0, 300.0, false)), kNow + 20.0);
}

TEST(ExecConstraints, QueuedFireReleasedWhenWaitGrows)
{
  // 排隊後，執行中任務的剩餘時間變長、做不完了：重新評估時釋放
  CbbaAgent dog = dogWithActive(fire(1, 0, 0), false, 30.0);
  dog.onTask(fire(2, 3, 0, kNow, 120.0), kNow);
  ASSERT_EQ(dog.path(), (std::vector<TaskId>{1, 2}));
  ExecConstraints c = dog.execConstraints();
  c.active_remaining = 110.0;
  dog.setExecConstraints(c, kNow);
  dog.reevaluate(kNow);
  EXPECT_EQ(dog.path(), std::vector<TaskId>{1});
  EXPECT_EQ(dog.winner(2), kNoAgent);
}

TEST(ExecConstraints, ReservedIsNotJumpedBySamePriority)
{
  // 只有保留、還沒開始：同級的火警不能插到它前面（容量承諾）
  CbbaAgent dog(makeDog(), ScoringParams{});
  dog.onTask(fire(1, 30, 0), kNow);
  ExecConstraints c;
  c.pinned = {1};
  c.active_pinned = false;
  dog.setExecConstraints(c, kNow);
  dog.onTask(fire(2, 3, 0), kNow);
  ASSERT_FALSE(dog.path().empty());
  EXPECT_EQ(dog.path().front(), 1u);
}

TEST(ExecConstraints, LockedTasksAreNotBid)
{
  // 別台已接受的任務：不出價，手上有的釋放
  CbbaAgent dog(makeDog(), ScoringParams{});
  dog.onTask(fire(1, 3, 0), kNow);
  ASSERT_EQ(dog.path(), std::vector<TaskId>{1});
  EXPECT_TRUE(dog.setLocked({1}, kNow));
  EXPECT_TRUE(dog.path().empty());
  dog.onTask(fire(2, 5, 0), kNow);
  EXPECT_EQ(dog.path(), std::vector<TaskId>{2});
  dog.setLocked({}, kNow);   // 持有者撤銷：重新開放
  dog.reevaluate(kNow);
  EXPECT_EQ(dog.path().size(), 2u);
}

TEST(ExecConstraints, SuspendUntilResumed)
{
  // BT 暫時拒絕：撤回出價；BT 狀態改變（resume）後重新出價
  CbbaAgent dog(makeDog(), ScoringParams{});
  dog.onTask(fire(1, 3, 0), kNow);
  EXPECT_TRUE(dog.suspendTask(1, kNow));
  EXPECT_TRUE(dog.path().empty());
  dog.reevaluate(kNow + 1.0);
  EXPECT_TRUE(dog.path().empty());
  EXPECT_TRUE(dog.resumeSuspended(kNow + 2.0));
  EXPECT_EQ(dog.path(), std::vector<TaskId>{1});
}

TEST(ExecConstraints, NoNewTasksWhenNotAccepting)
{
  // BT 的執行狀態逾時或故障：不加入新任務
  CbbaAgent dog(makeDog(), ScoringParams{});
  ExecConstraints c;
  c.accept_new = false;
  dog.setExecConstraints(c, kNow);
  dog.onTask(fire(1, 3, 0), kNow);
  EXPECT_TRUE(dog.path().empty());
}

TEST(ExecConstraints, FaultReleasesUnacceptedBids)
{
  // 變成不能接新任務（故障）：還沒被 BT 接受的出價撤回；已接受的留著（由 BT 處理）
  CbbaAgent dog = dogWithActive(fire(1, 10, 0), true, 20.0);
  dog.onTask(patrol(2, 12, 0), kNow);
  ASSERT_EQ(dog.path().size(), 2u);
  ExecConstraints c = dog.execConstraints();
  c.accept_new = false;
  EXPECT_TRUE(dog.setExecConstraints(c, kNow));
  EXPECT_EQ(dog.path(), std::vector<TaskId>{1});
  EXPECT_EQ(dog.winner(2), kNoAgent);
}
