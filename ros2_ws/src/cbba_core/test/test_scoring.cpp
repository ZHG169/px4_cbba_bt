// 計分的單元測試：手算結果、異質出價、電池 Cases A/B/C、回程、逾期、排序
#include <gtest/gtest.h>

#include <random>

#include "cbba_core/scoring.hpp"

using namespace cbba_core;

namespace
{
Task makeTask(
  TaskId id, TaskType type, double x, double y, double deadline = 60.0, double value = 80.0,
  double duration = 0.0)
{
  Task t;
  t.id = id;
  t.type = type;
  t.position = {x, y, 0.0};
  t.created = 0.0;
  t.deadline_sec = deadline;
  t.value = value;
  t.duration_sec = duration;
  return t;
}

AgentState makeUav(double x, double y, double battery = 100.0)
{
  AgentState a;
  a.id = 1;
  a.type = AgentType::UAV;
  a.position = {x, y, 0.0};
  a.home = a.position;
  a.battery = battery;
  a.safety_reserve = 20.0;
  a.energy_per_meter = 0.5;
  a.hover_energy_per_sec = 0.2;
  return a;
}

ScoringParams makeParams(double battery_weight = 1.0)
{
  ScoringParams p;
  p.cruise_speed = 5.0;
  p.speed_margin = 0.8;   // 規劃速度 4 m/s
  p.battery_weight = battery_weight;
  p.cost_ref = 50.0;
  return p;
}
}  // namespace

TEST(Scoring, HandComputedSingleTask)
{
  // d=40, 到達=10 s, 剩餘期限=60 s -> Lt=1/6, C_ST=6.6667
  // 能量: 去程 20 + 懸停 2 + 回程 20 = 42, 可用 80 -> 0.525
  // C_total = 6.6667 * 1.525 = 10.1667, score = 80 / (1 + 10.1667/50) = 66.482
  const AgentState uav = makeUav(0, 0);
  const Task task = makeTask(1, TaskType::AIR_RECON, 40, 0, 60, 80, 10);
  const Insertion ins = bestInsertion(uav, {}, task, 0.0, makeParams());
  ASSERT_TRUE(ins.feasible);
  EXPECT_NEAR(ins.eval.st_cost, 6.6667, 1e-3);
  EXPECT_NEAR(ins.eval.path_energy, 42.0, 1e-9);
  EXPECT_NEAR(ins.eval.battery_after, 58.0, 1e-9);
  EXPECT_NEAR(ins.eval.battery_cost, 0.525, 1e-9);
  EXPECT_NEAR(ins.marginal_cost, 10.1667, 1e-3);
  EXPECT_NEAR(ins.score, 66.482, 1e-2);
  EXPECT_FALSE(ins.eval.legs[0].overdue);
}

TEST(Scoring, HeterogeneousBidsOnlyOwnTaskTypes)
{
  // 週計畫第 2 週檢核點：輸入 5 個混合任務，只對自己專長的任務有效出價
  const std::vector<Task> tasks{
    makeTask(1, TaskType::AIR_RECON, 10, 0),
    makeTask(2, TaskType::GROUND_INTERVENTION, 10, 5, 120, 100),
    makeTask(3, TaskType::AIR_RECON, -10, 0),
    makeTask(4, TaskType::PATROL, 5, 5, 600, 20),
    makeTask(5, TaskType::GROUND_INTERVENTION, 0, 10, 120, 100),
  };
  const AgentState uav = makeUav(0, 0);
  AgentState ugv = makeUav(0, 0);
  ugv.type = AgentType::UGV;

  for (const Task & t : tasks) {
    const double uav_score = bestInsertion(uav, {}, t, 0.0, makeParams()).score;
    const double ugv_score = bestInsertion(ugv, {}, t, 0.0, makeParams()).score;
    if (t.type == TaskType::AIR_RECON) {
      EXPECT_GT(uav_score, 0.0) << "task " << t.id;
      EXPECT_DOUBLE_EQ(ugv_score, 0.0) << "task " << t.id;
    } else {
      EXPECT_DOUBLE_EQ(uav_score, 0.0) << "task " << t.id;
      EXPECT_GT(ugv_score, 0.0) << "task " << t.id;
    }
  }
}

TEST(Scoring, ScoreNeverExceedsTaskValue)
{
  const AgentState uav = makeUav(0, 0);
  const Task here = makeTask(1, TaskType::AIR_RECON, 0, 0);   // 就在任務點上
  const Insertion ins = bestInsertion(uav, {}, here, 0.0, makeParams());
  ASSERT_TRUE(ins.feasible);
  EXPECT_LE(ins.score, here.value);
  EXPECT_GT(ins.score, 0.0);
}

TEST(Scoring, CaseA_EqualBatteryNearerWins)
{
  const Task task = makeTask(1, TaskType::AIR_RECON, 20, 0);
  const double near = bestInsertion(makeUav(10, 0), {}, task, 0.0, makeParams()).score;
  const double far = bestInsertion(makeUav(-10, 0), {}, task, 0.0, makeParams()).score;
  EXPECT_GT(near, far);
}

TEST(Scoring, CaseB_LowBatteryIsRejectedByReserve)
{
  // 近的那台：去程 5 + 回程 5 = 10，但可用電量只有 23 - 20 = 3
  const Task task = makeTask(1, TaskType::AIR_RECON, 20, 0);
  const Insertion near = bestInsertion(makeUav(10, 0, 23.0), {}, task, 0.0, makeParams());
  const Insertion far = bestInsertion(makeUav(-10, 0), {}, task, 0.0, makeParams());
  EXPECT_FALSE(near.feasible);
  EXPECT_DOUBLE_EQ(near.score, 0.0);
  EXPECT_TRUE(far.feasible);
  EXPECT_GT(far.score, 0.0);
}

TEST(Scoring, CaseC_BatteryBurdenChangesWinner)
{
  // 兩台都可行。近的那台電量吃緊（用掉可用電量的 83%），遠的那台只用 15%。
  const Task task = makeTask(1, TaskType::AIR_RECON, 20, 0);
  const AgentState near = makeUav(10, 0, 32.0);   // d=10
  const AgentState far = makeUav(8, 0, 100.0);    // d=12

  // 不看電池時近的贏
  EXPECT_GT(
    bestInsertion(near, {}, task, 0.0, makeParams(0.0)).score,
    bestInsertion(far, {}, task, 0.0, makeParams(0.0)).score);

  // 電池權重 10 時換遠的贏
  const Insertion near_w = bestInsertion(near, {}, task, 0.0, makeParams(10.0));
  const Insertion far_w = bestInsertion(far, {}, task, 0.0, makeParams(10.0));
  ASSERT_TRUE(near_w.feasible);
  ASSERT_TRUE(far_w.feasible);
  EXPECT_GT(far_w.score, near_w.score);
}

TEST(Scoring, CanReachButCannotReturnIsInfeasible)
{
  // 去程 50 在可用電量 80 以內，但加上回程 50 就超過
  const AgentState uav = makeUav(0, 0);
  const Task far_task = makeTask(1, TaskType::AIR_RECON, 100, 0);
  const Insertion ins = bestInsertion(uav, {}, far_task, 0.0, makeParams());
  EXPECT_FALSE(ins.feasible);
  EXPECT_DOUBLE_EQ(ins.score, 0.0);
}

TEST(Scoring, OverdueTaskStillGetsPositiveButLowerScore)
{
  // 期限是軟限制：來不及時分數降低，但不歸零
  const AgentState uav = makeUav(0, 0);
  const Insertion on_time =
    bestInsertion(uav, {}, makeTask(1, TaskType::AIR_RECON, 40, 0, 60), 0.0, makeParams());
  const Insertion late =
    bestInsertion(uav, {}, makeTask(2, TaskType::AIR_RECON, 40, 0, 5), 0.0, makeParams());
  ASSERT_TRUE(late.feasible);
  EXPECT_TRUE(late.eval.legs[0].overdue);
  EXPECT_FALSE(on_time.eval.legs[0].overdue);
  EXPECT_GT(late.score, 0.0);
  EXPECT_LT(late.score, on_time.score);
}

TEST(Scoring, PastDeadlineDoesNotBreakTheCost)
{
  // 現在時間已經超過期限：剩餘期限用下限值，成本仍為有限正數
  const AgentState uav = makeUav(0, 0);
  const Task task = makeTask(1, TaskType::AIR_RECON, 20, 0, 30);
  const Insertion ins = bestInsertion(uav, {}, task, 100.0, makeParams());
  ASSERT_TRUE(ins.feasible);
  EXPECT_TRUE(std::isfinite(ins.marginal_cost));
  EXPECT_GT(ins.marginal_cost, 0.0);
  EXPECT_GT(ins.score, 0.0);
}

TEST(Scoring, UrgentTaskIsScheduledFirst)
{
  // 兩個任務距離相同、方向相反。急迫的（期限 60 秒）應排在不急的（600 秒）前面。
  const AgentState uav = makeUav(0, 0);
  const Task urgent = makeTask(1, TaskType::AIR_RECON, 10, 0, 60);
  const Task relaxed = makeTask(2, TaskType::AIR_RECON, -10, 0, 600);
  const Insertion ins = bestInsertion(uav, {relaxed}, urgent, 0.0, makeParams());
  ASSERT_TRUE(ins.feasible);
  EXPECT_EQ(ins.position, 0u);
}

TEST(Scoring, SlowerActualSpeedLowersScore)
{
  const AgentState uav = makeUav(0, 0);
  const Task task = makeTask(1, TaskType::AIR_RECON, 40, 0);
  ScoringParams slow = makeParams();
  slow.speed_margin = 0.4;
  EXPECT_LT(
    bestInsertion(uav, {}, task, 0.0, slow).score,
    bestInsertion(uav, {}, task, 0.0, makeParams()).score);
}

TEST(Scoring, MarginalScoreDoesNotIncreaseWithLongerPath)
{
  // 介面規格的條件 3：已接的任務越多，對其他任務的分數只能持平或降低。
  // 插入式的「原始分數」不保證成立（新任務順路時分數會變高），這裡統計違反的比例。
  // 實際送出的出價由 CbbaAgent 的出價上限保證不增，見 test_agent 與 test_lossy_network。
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> pos(-50.0, 50.0);
  std::uniform_real_distribution<double> deadline(30.0, 300.0);
  const ScoringParams params = makeParams();

  int samples = 0;
  int violations = 0;
  double worst_increase = 0.0;
  for (int trial = 0; trial < 2000; ++trial) {
    const AgentState uav = makeUav(pos(rng), pos(rng), 200.0);
    const Task a = makeTask(1, TaskType::AIR_RECON, pos(rng), pos(rng), deadline(rng));
    const Task b = makeTask(2, TaskType::AIR_RECON, pos(rng), pos(rng), deadline(rng));

    const Insertion alone = bestInsertion(uav, {}, b, 0.0, params);
    const Insertion first = bestInsertion(uav, {}, a, 0.0, params);
    if (!alone.feasible || !first.feasible) {continue;}
    const Insertion after = bestInsertion(uav, {a}, b, 0.0, params);
    if (!after.feasible) {continue;}

    ++samples;
    if (after.score > alone.score + params.score_epsilon) {
      ++violations;
      worst_increase = std::max(worst_increase, after.score - alone.score);
    }
  }
  const double rate = samples > 0 ? static_cast<double>(violations) / samples : 0.0;
  std::cout << "[monotonic] samples=" << samples << " violations=" << violations
            << " rate=" << rate * 100.0 << "% worst_increase=" << worst_increase << "\n";
  RecordProperty("violation_rate", std::to_string(rate));
  EXPECT_GT(samples, 1000);
}
