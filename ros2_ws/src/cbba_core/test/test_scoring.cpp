// 出價（時間折扣報酬 TDR，M1，2026-11-15）的單元測試：手算、異質出價、電池 Cases A/B/C、回程、期限、排序、
// 邊際遞減的統計、失敗打折、硬期限、release、任務的電量下限、half_life（設計見 doc/formula/tdr_design.md）
#include <gtest/gtest.h>

#include <cmath>
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

ScoringParams makeParams(double energy_weight = 50.0)
{
  ScoringParams p;
  p.cruise_speed = 5.0;
  p.speed_margin = 0.8;   // 規劃速度 4 m/s
  p.energy_weight = energy_weight;
  return p;
}

// 不打折（各優先級的預設 half_life 都是 0）：只看耗電
ScoringParams noDecay(double energy_weight = 50.0)
{
  ScoringParams p = makeParams(energy_weight);
  p.half_life_by_priority = {{0.0, 0.0, 0.0, 0.0}};
  return p;
}
}  // namespace

TEST(Scoring, HandComputedSingleTask)
{
  // AIR_RECON（P3，預設 half_life 600 s）、d = 40 m、規劃速度 4 m/s → 到達 10 s
  // 報酬 80 × 2^(−10/600) = 79.081；耗電 去程 20＋懸停 2＋回程 20 = 42，可用 80 → 0.525，耗電成本 50 × 0.525 = 26.25
  // 出價 = 79.081 − 26.25 = 52.831（空路徑的報酬是 0）
  const AgentState uav = makeUav(0, 0);
  const Task task = makeTask(1, TaskType::AIR_RECON, 40, 0, 60, 80, 10);
  const Insertion ins = bestInsertion(uav, {}, task, 0.0, makeParams());
  ASSERT_TRUE(ins.feasible);
  EXPECT_NEAR(ins.eval.legs[0].arrival, 10.0, 1e-9);
  EXPECT_DOUBLE_EQ(ins.eval.legs[0].half_life, 600.0);
  EXPECT_NEAR(ins.eval.reward, 79.0811, 1e-3);
  EXPECT_NEAR(ins.eval.path_energy, 42.0, 1e-9);
  EXPECT_NEAR(ins.eval.battery_after, 58.0, 1e-9);
  EXPECT_NEAR(ins.eval.battery_fraction, 0.525, 1e-9);
  EXPECT_NEAR(ins.eval.energy_cost, 26.25, 1e-9);
  EXPECT_NEAR(ins.gain, 52.8311, 1e-3);
  EXPECT_NEAR(ins.score, 52.8311, 1e-3);
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
  // 就在任務點上：不用飛、不打折、不耗電 → 出價 = 價值
  const AgentState uav = makeUav(0, 0);
  const Task here = makeTask(1, TaskType::AIR_RECON, 0, 0);
  const Insertion ins = bestInsertion(uav, {}, here, 0.0, makeParams());
  ASSERT_TRUE(ins.feasible);
  EXPECT_LE(ins.score, here.value);
  EXPECT_NEAR(ins.score, here.value, 1e-9);
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

  // 不看耗電（w_E = 0）時近的贏（先到）
  EXPECT_GT(
    bestInsertion(near, {}, task, 0.0, makeParams(0.0)).score,
    bestInsertion(far, {}, task, 0.0, makeParams(0.0)).score);

  // w_E = 50：近的那台用掉大部分可用電量，換遠的贏
  const Insertion near_w = bestInsertion(near, {}, task, 0.0, makeParams(50.0));
  const Insertion far_w = bestInsertion(far, {}, task, 0.0, makeParams(50.0));
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

TEST(Scoring, SoftDeadlineDoesNotChangeTheScore)
{
  // 軟期限不影響分數（「急」由 half_life 表達）；來不及時只標 overdue
  const AgentState uav = makeUav(0, 0);
  const Insertion on_time =
    bestInsertion(uav, {}, makeTask(1, TaskType::AIR_RECON, 40, 0, 60), 0.0, makeParams());
  const Insertion late =
    bestInsertion(uav, {}, makeTask(2, TaskType::AIR_RECON, 40, 0, 5), 0.0, makeParams());
  ASSERT_TRUE(late.feasible);
  EXPECT_TRUE(late.eval.legs[0].overdue);
  EXPECT_FALSE(on_time.eval.legs[0].overdue);
  EXPECT_NEAR(late.score, on_time.score, 1e-9);
}

TEST(Scoring, ScoreDoesNotDependOnTaskAge)
{
  // 沒有期限、或期限已經過了：分數只看「從現在起」多久到，和任務放了多久無關、不會逾期
  const AgentState uav = makeUav(0, 0);
  Task t = makeTask(1, TaskType::AIR_RECON, 30, 0, 300.0, 100.0);
  t.explicit_deadline = false;
  const double fresh = bestInsertion(uav, {}, t, 0.0, makeParams()).score;
  for (double now : {250.0, 400.0, 3600.0}) {
    const Insertion later = bestInsertion(uav, {}, t, now, makeParams());
    EXPECT_NEAR(later.score, fresh, 1e-9) << now;
    EXPECT_FALSE(later.eval.legs[0].overdue) << now;
  }
}

TEST(Scoring, UrgentTaskIsScheduledFirst)
{
  // 兩個任務距離相同、方向相反。急的（half_life 30 s）排在不急的（600 s）前面：晚到 5 s，急的少得多
  const AgentState uav = makeUav(0, 0);
  Task urgent = makeTask(1, TaskType::AIR_RECON, 10, 0, 600);
  urgent.ext.half_life_s = 30;
  Task relaxed = makeTask(2, TaskType::AIR_RECON, -10, 0, 600);
  relaxed.ext.half_life_s = 600;
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
  // 邊際遞減（DMG）：已接的任務越多，對其他任務的出價只能持平或降低。只有時間折扣時成立（Choi 2009），
  // 加上耗電成本後，新任務剛好順路時可能變高（tdr_design.md「收斂」）。這裡統計違反的比例；
  // 實際送出的出價由 CbbaAgent 的出價上限保證不增，見 test_agent 與 test_lossy_network。
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> pos(-50.0, 50.0);
  const ScoringParams params = makeParams();

  int samples = 0;
  int violations = 0;
  double worst_increase = 0.0;
  for (int trial = 0; trial < 2000; ++trial) {
    const AgentState uav = makeUav(pos(rng), pos(rng), 200.0);
    const Task a = makeTask(1, TaskType::AIR_RECON, pos(rng), pos(rng));
    const Task b = makeTask(2, TaskType::AIR_RECON, pos(rng), pos(rng));

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

  // 不算耗電時（純時間折扣）不會違反
  const ScoringParams time_only = makeParams(0.0);
  std::mt19937 rng2(54321);
  int pure_violations = 0;
  for (int trial = 0; trial < 500; ++trial) {
    const AgentState uav = makeUav(pos(rng2), pos(rng2), 200.0);
    const Task a = makeTask(1, TaskType::AIR_RECON, pos(rng2), pos(rng2));
    const Task b = makeTask(2, TaskType::AIR_RECON, pos(rng2), pos(rng2));
    const Insertion alone = bestInsertion(uav, {}, b, 0.0, time_only);
    const Insertion after = bestInsertion(uav, {a}, b, 0.0, time_only);
    if (alone.feasible && after.feasible && after.score > alone.score + time_only.score_epsilon) {
      ++pure_violations;
    }
  }
  EXPECT_EQ(pure_violations, 0);
}

// ===========================================================================
// 失敗過的任務：期望價值、期望到達時間（2026-10-17）
// ===========================================================================
TEST(Scoring, FullSuccessProbKeepsOriginalScore)
{
  // success_prob = 1（沒失敗過）：數字同 HandComputedSingleTask
  const AgentState uav = makeUav(0, 0);
  Task task = makeTask(1, TaskType::AIR_RECON, 40, 0, 60, 80, 10);
  task.success_prob = 1.0;
  const Insertion ins = bestInsertion(uav, {}, task, 0.0, makeParams());
  ASSERT_TRUE(ins.feasible);
  EXPECT_NEAR(ins.score, 52.8311, 1e-3);
}

TEST(Scoring, UrgentTaskPenalizedMore)
{
  // 失敗一次（p = 0.5）、距離 20 m、規劃速度 4 m/s、執行 10 s、不算耗電：期望重試 (1 − p)/p ×（5＋10）= 15 s
  //   急的（half_life 60 s、價值 80）：80 × 2^(−5/60) = 75.51 → 0.5 × 80 × 2^(−20/60) = 31.75（42 %）
  //   不急的（600 s、價值 40）：39.77 → 19.54（49 %）：只剩價值減半
  const AgentState uav = makeUav(0, 0);
  const ScoringParams params = makeParams(0.0);
  auto score = [&](std::uint16_t half_life, double value, double p) {
      Task t = makeTask(1, TaskType::AIR_RECON, 20, 0, 600, value, 10);
      t.ext.half_life_s = half_life;
      t.success_prob = p;
      return bestInsertion(uav, {}, t, 0.0, params).score;
    };
  EXPECT_NEAR(score(60, 80, 1.0), 75.510, 0.01);
  EXPECT_NEAR(score(60, 80, 0.5), 31.748, 0.01);
  EXPECT_NEAR(score(600, 40, 1.0), 39.770, 0.01);
  EXPECT_NEAR(score(600, 40, 0.5), 19.543, 0.01);
  EXPECT_LT(score(60, 80, 0.5) / score(60, 80, 1.0), score(600, 40, 0.5) / score(600, 40, 1.0));
}

TEST(Scoring, FailureDelaysLaterTasks)
{
  // 路徑上第一個任務 p = 0.5：它多 (1 − p)/p × (前往 5 s＋執行 10 s) = 15 s，後面的任務一起延後、報酬變少
  const AgentState uav = makeUav(0, 0);
  const ScoringParams params = makeParams(0.0);
  Task a = makeTask(1, TaskType::AIR_RECON, 20, 0, 600, 80, 10);
  const Task b = makeTask(2, TaskType::AIR_RECON, 40, 0, 600, 80, 0);
  const PathEval base = evaluatePath(uav, {a, b}, 0.0, params);
  EXPECT_NEAR(base.legs[1].arrival, 20.0, 1e-9);    // 5 + 10 + 5
  a.success_prob = 0.5;
  const PathEval slow = evaluatePath(uav, {a, b}, 0.0, params);
  EXPECT_NEAR(slow.legs[0].arrival, 20.0, 1e-9);    // 5 + 15
  EXPECT_NEAR(slow.legs[1].arrival, 35.0, 1e-9);    // 30 + 5
  EXPECT_LT(slow.legs[1].reward, base.legs[1].reward);
  EXPECT_LT(slow.score, base.score);
}

TEST(Scoring, ActiveHeadWithRemainingTimeIsNotDelayed)
{
  // 執行中、有 BT 的剩餘時間：第一個任務的完成時間就用它，不再加期望重試時間
  const AgentState uav = makeUav(0, 0);
  const ScoringParams params = makeParams(0.0);
  Task a = makeTask(1, TaskType::AIR_RECON, 20, 0, 600, 80, 10);
  a.success_prob = 0.5;
  const Task b = makeTask(2, TaskType::AIR_RECON, 40, 0, 600, 80, 0);
  const PathEval eval = evaluatePath(uav, {a, b}, 0.0, params, 3.0);
  EXPECT_NEAR(eval.legs[0].completion, 3.0, 1e-9);
  EXPECT_NEAR(eval.legs[1].arrival, 8.0, 1e-9);     // 3 + 20 m / 4 m/s
}

// ===========================================================================
// 任務定義的時間與電量限制（4f 第一批，2026-11-13）
// ===========================================================================
TEST(Scoring, HardDeadlineIsInfeasibleSoftIsNot)
{
  // 到達 10 s、期限 5 s：軟期限照樣出價，硬期限不可行（0）；執行中的（exempt）不檢查
  const AgentState uav = makeUav(0, 0);
  Task soft = makeTask(1, TaskType::NONE, 40, 0, 5.0);
  soft.platform = 1;
  Task hard = soft;
  hard.ext.deadline_hard = true;
  EXPECT_GT(bestInsertion(uav, {}, soft, 0.0, makeParams()).score, 0.0);
  const Insertion no = bestInsertion(uav, {}, hard, 0.0, makeParams());
  EXPECT_FALSE(no.feasible);
  EXPECT_EQ(no.score, 0.0);
  EXPECT_FALSE(evaluatePath(uav, {hard}, 0.0, makeParams()).feasible);
  EXPECT_TRUE(evaluatePath(uav, {hard}, 0.0, makeParams(), -1.0, hard.id).feasible);
}

TEST(Scoring, NoDeadlineHardFlagIsIgnored)
{
  // deadline_hard 但沒有期限：沒有東西可以來不及，永遠可行
  const AgentState uav = makeUav(0, 0);
  Task t = makeTask(1, TaskType::AIR_RECON, 30, 0);
  t.explicit_deadline = false;
  t.ext.deadline_hard = true;
  EXPECT_TRUE(evaluatePath(uav, {t}, 10000.0, makeParams()).feasible);
}

TEST(Scoring, InsertionMustNotPushHardDeadlineTaskLate)
{
  // 路徑上已有 A（到達 10 s、硬期限 12 s）；B 插在 A 前面會讓 A 遲到 → 只能排在 A 後面
  const AgentState uav = makeUav(0, 0);
  Task a = makeTask(1, TaskType::NONE, 40, 0, 12.0);
  a.platform = 1;
  a.ext.deadline_hard = true;
  Task b = makeTask(2, TaskType::NONE, 0, 30, 600.0);
  b.platform = 1;
  const Insertion ins = bestInsertion(uav, {a}, b, 0.0, makeParams());
  ASSERT_TRUE(ins.feasible);
  EXPECT_EQ(ins.position, 1u);
}

TEST(Scoring, ReleaseMakesItWait)
{
  // 到達 10 s、release 30 s：等到 30 s 才算到達；晚到分數較低，後面的任務跟著延後
  const AgentState uav = makeUav(0, 0);
  Task now_task = makeTask(1, TaskType::NONE, 40, 0, 120.0);
  now_task.platform = 1;
  Task later = now_task;
  later.ext.release = 30.0;
  const PathEval e = evaluatePath(uav, {later}, 0.0, makeParams());
  ASSERT_EQ(e.legs.size(), 1u);
  EXPECT_DOUBLE_EQ(e.legs[0].arrival, 30.0);
  EXPECT_LT(bestInsertion(uav, {}, later, 0.0, makeParams()).score,
    bestInsertion(uav, {}, now_task, 0.0, makeParams()).score);
  Task next = makeTask(2, TaskType::NONE, 40, 40, 600.0);
  next.platform = 1;
  EXPECT_DOUBLE_EQ(evaluatePath(uav, {later, next}, 0.0, makeParams()).legs[1].arrival, 30.0 + 40.0 / 4.0);
  // release 已經過了：不用等
  later.ext.release = 5.0;
  EXPECT_DOUBLE_EQ(evaluatePath(uav, {later}, 0.0, makeParams()).legs[0].arrival, 10.0);
}

TEST(Scoring, TaskBatteryReserveRaisesTheFloor)
{
  // 電量 70、載具存量 20：路徑耗能 42 → 可行；任務要求做完剩 40 → 只能用 30 → 不可行
  const AgentState uav = makeUav(0, 0, 70.0);
  Task t = makeTask(1, TaskType::NONE, 40, 0, 60.0, 80.0, 10.0);
  t.platform = 1;
  EXPECT_TRUE(evaluatePath(uav, {t}, 0.0, makeParams()).feasible);
  t.ext.min_battery_reserve = 40;
  EXPECT_FALSE(evaluatePath(uav, {t}, 0.0, makeParams()).feasible);
  t.ext.min_battery_reserve = 10;   // 比載具的低：照載具的
  EXPECT_TRUE(evaluatePath(uav, {t}, 0.0, makeParams()).feasible);
}

// ===========================================================================
// half_life（時間折扣）
// ===========================================================================
TEST(Scoring, HalfLifeFromTaskOrPriority)
{
  // 任務有帶就用任務的；沒帶照優先級的預設（P0 60、P1 120、P2 300、P3 600）；舊型火警 P0、空中偵察 P3
  const ScoringParams params = makeParams();
  Task fire = makeTask(1, TaskType::GROUND_INTERVENTION, 0, 0, 600, 100);
  Task recon = makeTask(2, TaskType::AIR_RECON, 0, 0, 600, 100);
  Task p1 = makeTask(3, TaskType::NONE, 0, 0, 600, 100);
  p1.priority = 1;
  EXPECT_DOUBLE_EQ(halfLifeOf(fire, params), 60.0);
  EXPECT_DOUBLE_EQ(halfLifeOf(recon, params), 600.0);
  EXPECT_DOUBLE_EQ(halfLifeOf(p1, params), 120.0);
  recon.ext.half_life_s = 45;
  EXPECT_DOUBLE_EQ(halfLifeOf(recon, params), 45.0);
  // 每 half_life 秒價值剩一半；不會超過原本的價值；預設是 0 時不打折
  EXPECT_NEAR(decayedValue(fire, 60.0, params), 50.0, 1e-9);
  EXPECT_NEAR(decayedValue(fire, 120.0, params), 25.0, 1e-9);
  EXPECT_NEAR(decayedValue(fire, -5.0, params), 100.0, 1e-9);
  EXPECT_DOUBLE_EQ(decayedValue(fire, 120.0, noDecay()), 100.0);
}

TEST(Scoring, HalfLifeWidensGapBetweenNearAndFar)
{
  // 急的任務（half_life 短）：近的載具和遠的差距比不打折時大
  const AgentState near = makeUav(20, 0);
  const AgentState far = makeUav(-30, 0);   // 60 m：來回 60 % 電量，還在可行範圍
  Task t = makeTask(1, TaskType::AIR_RECON, 30, 0, 600.0, 100.0);
  const double near0 = bestInsertion(near, {}, t, 0.0, noDecay()).score;
  const double far0 = bestInsertion(far, {}, t, 0.0, noDecay()).score;
  ASSERT_GT(near0, far0);
  ASSERT_GT(far0, 0.0);
  t.ext.half_life_s = 20;
  const double near1 = bestInsertion(near, {}, t, 0.0, noDecay()).score;
  const double far1 = bestInsertion(far, {}, t, 0.0, noDecay()).score;
  EXPECT_LT(near1, near0);
  EXPECT_LT(far1, far0);
  EXPECT_GT(near1 / std::max(far1, 1e-9), near0 / far0);   // 差距變大
}

TEST(Scoring, HalfLifePrefersEarlierInsertion)
{
  // 起降點在 (−40, 0)。路徑上已有 (10, 0)，新任務在 (−30, 0)：
  //   不打折：先 (10, 0) 再 (−30, 0) 順路回家，耗電少 → 排在後面
  //   新任務 half_life 20 s：先去它（7.5 s 到，比 12.5 s 多 12 分）勝過多繞的耗電（約 8 分）→ 插在前面
  AgentState uav = makeUav(0, 0, 200.0);
  uav.home = {-40.0, 0.0, 0.0};
  const Task first = makeTask(1, TaskType::AIR_RECON, 10, 0, 600.0, 100.0);
  Task other = makeTask(2, TaskType::AIR_RECON, -30, 0, 600.0, 100.0);
  const Insertion slow = bestInsertion(uav, {first}, other, 0.0, noDecay());
  ASSERT_TRUE(slow.feasible);
  EXPECT_EQ(slow.position, 1u);
  other.ext.half_life_s = 20;
  const Insertion fast = bestInsertion(uav, {first}, other, 0.0, noDecay());
  ASSERT_TRUE(fast.feasible);
  EXPECT_EQ(fast.position, 0u);
}

TEST(Scoring, DelayingOtherTasksCountsInTheBid)
{
  // TDR：插入讓路徑上其他任務晚到，它們少掉的報酬算在出價裡。手上的任務很急（30 s）時，
  // 同一個新任務的出價比手上的任務不急（600 s）時低
  const AgentState uav = makeUav(0, 0, 200.0);
  //   新任務 half_life 60 s：插在前面 5 s 到（94.4）、排在後面 12.1 s 到（87.0）
  //   手上的任務 600 s：插在前面它只少 0.8 → 出價 93.6；30 s：它少 13.4 → 插前面只剩 81.0，改排後面 87.0
  Task held = makeTask(1, TaskType::AIR_RECON, 20, 0, 600.0, 100.0);
  Task incoming = makeTask(2, TaskType::AIR_RECON, 0, 20, 600.0, 100.0);
  incoming.ext.half_life_s = 60;
  held.ext.half_life_s = 600;
  const double relaxed = bestInsertion(uav, {held}, incoming, 0.0, noDecay(0.0)).score;
  held.ext.half_life_s = 30;
  const double urgent = bestInsertion(uav, {held}, incoming, 0.0, noDecay(0.0)).score;
  EXPECT_NEAR(relaxed, 93.58, 0.05);
  EXPECT_NEAR(urgent, 86.97, 0.05);
  EXPECT_LT(urgent, relaxed);
}

TEST(Scoring, NotWorthItGivesNoBid)
{
  // 報酬增加 ≤ 0（價值小、耗電成本大）：可行但不出價
  const AgentState uav = makeUav(0, 0);
  const Task cheap = makeTask(1, TaskType::AIR_RECON, 60, 0, 600.0, 5.0);   // 來回 60 % 電量，價值 5
  const Insertion ins = bestInsertion(uav, {}, cheap, 0.0, makeParams());
  EXPECT_TRUE(ins.feasible);
  EXPECT_LT(ins.gain, 0.0);
  EXPECT_DOUBLE_EQ(ins.score, 0.0);
}

// ===========================================================================
// TDR 參數實驗的公式檢查（docs/tdr_experiment_plan.md 階段 B）：調 w_E、half_life 時出價的方向要對
// ===========================================================================
namespace
{
// 隨機的平台不限任務（新型，優先級 P0～P3）
Task randomTask(TaskId id, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> xy(-60.0, 60.0);
  std::uniform_int_distribution<int> priority(0, 3);
  std::uniform_real_distribution<double> duration(0.0, 20.0);
  Task t = makeTask(id, TaskType::NONE, xy(rng), xy(rng), 60.0, 100.0, duration(rng));
  t.explicit_deadline = false;
  t.priority = static_cast<std::uint8_t>(priority(rng));
  return t;
}

ScoringParams scaledHalfLife(double energy_weight, double gamma)
{
  ScoringParams p = makeParams(energy_weight);
  const double base[4] = {60.0, 120.0, 300.0, 600.0};
  for (std::size_t k = 0; k < 4; ++k) {
    p.half_life_by_priority[k] = base[k] * gamma;
  }
  return p;
}
}  // namespace

TEST(Scoring, HigherEnergyWeightNeverRaisesBid)
{
  // 其他條件不變、w_E 變大：插入多花的電 ≥ 0，所以出價不會變高（取最好的插入位置也一樣）
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> battery(40.0, 100.0);
  std::uniform_int_distribution<int> length(0, 3);
  const double weights[] = {0.0, 25.0, 50.0, 100.0, 200.0};
  for (int k = 0; k < 400; ++k) {
    AgentState uav = makeUav(0, 0, battery(rng));
    uav.energy_per_meter = 0.1;   // 0.5 %/m 時大部分的路徑都不可行，測不到東西
    std::vector<Task> path;
    const int n = length(rng);
    for (int i = 0; i < n; ++i) {
      path.push_back(randomTask(static_cast<TaskId>(i + 1), rng));
    }
    const Task candidate = randomTask(99, rng);
    double previous = kInf;
    for (double w : weights) {
      const double score = bestInsertion(uav, path, candidate, 0.0, scaledHalfLife(w, 1.0)).score;
      EXPECT_LE(score, previous + 1e-9) << "case " << k << " w_E " << w;
      previous = score;
    }
  }
}

TEST(Scoring, ShorterHalfLifeNeverRaisesLateReward)
{
  // half_life 變短：同樣晚到的報酬不會變高；單一任務的出價（能量項和 half_life 無關）也不會變高
  std::mt19937 rng(12);
  std::uniform_real_distribution<double> arrival(0.1, 600.0);
  const double gammas[] = {2.0, 1.0, 0.5};
  for (int k = 0; k < 400; ++k) {
    const Task t = randomTask(1, rng);
    const double tau = arrival(rng);
    const AgentState uav = makeUav(0, 0);
    double previous_value = kInf;
    double previous_bid = kInf;
    for (double g : gammas) {
      const ScoringParams p = scaledHalfLife(50.0, g);
      const double value = decayedValue(t, tau, p);
      const double bid = bestInsertion(uav, {}, t, 0.0, p).score;
      EXPECT_LE(value, previous_value + 1e-9) << "case " << k << " gamma " << g;
      EXPECT_LE(bid, previous_bid + 1e-9) << "case " << k << " gamma " << g;
      previous_value = value;
      previous_bid = bid;
    }
  }
  // 兩台一近一遠：half_life 變短時，遠的那台（晚到）報酬下降得比較多
  Task fire = makeTask(1, TaskType::NONE, 0, 0, 600, 100);
  fire.priority = 0;
  const double near_long = decayedValue(fire, 20.0, scaledHalfLife(50.0, 2.0));
  const double far_long = decayedValue(fire, 80.0, scaledHalfLife(50.0, 2.0));
  const double near_short = decayedValue(fire, 20.0, scaledHalfLife(50.0, 0.5));
  const double far_short = decayedValue(fire, 80.0, scaledHalfLife(50.0, 0.5));
  EXPECT_GT(far_long - far_short, near_long - near_short);
}

TEST(Scoring, InfeasibleBidIsZeroForEveryParamSet)
{
  // 平台不符、電量不夠返航、硬期限來不及：不管 w_E、half_life 怎麼調，出價都是 0（不可行是條件，不是分數）
  const AgentState uav = makeUav(0, 0);
  const AgentState low = makeUav(0, 0, 21.0);
  Task ground = makeTask(1, TaskType::NONE, 10, 0);
  ground.platform = 2;
  Task far = makeTask(2, TaskType::NONE, 60, 0);
  far.explicit_deadline = false;
  Task hard = makeTask(3, TaskType::NONE, 40, 0, 5.0);
  hard.ext.deadline_hard = true;
  for (double w : {0.0, 25.0, 50.0, 100.0, 200.0}) {
    for (double g : {0.5, 1.0, 2.0}) {
      const ScoringParams p = scaledHalfLife(w, g);
      for (const auto & [agent, task] : {std::make_pair(uav, ground), std::make_pair(low, far),
          std::make_pair(uav, hard)})
      {
        const Insertion ins = bestInsertion(agent, {}, task, 0.0, p);
        EXPECT_FALSE(ins.feasible) << "task " << task.id << " w_E " << w << " gamma " << g;
        EXPECT_EQ(ins.score, 0.0) << "task " << task.id << " w_E " << w << " gamma " << g;
      }
    }
  }
}
