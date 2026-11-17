// TDR 參數實驗（tdr_experiment、mission_sim 的量測）：情境的判讀方向、收斂與出價的紀錄、可重現、異質耗電、
// 搶占不算接錯、角色組不同載具（docs/tdr_experiment_plan.md 階段 A、B）
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "cbba_core/tdr_experiment.hpp"

using namespace cbba_core;

namespace
{
// 和 cbba_sim tdr 一樣的設定
MissionResult runCase(
  const std::string & name, std::uint32_t seed, const ParamSet & set, double loss = 0.0, bool bids = false)
{
  const ScenarioCase c = makeScenario(name, seed);
  MissionConfig config;
  config.params = applyParamSet(ScoringParams{}, set);
  config.loss = loss;
  config.max_time = c.max_time;
  config.assign.assign_hold = 0.4;
  config.record_bids = bids;
  return runMission(c.agents, c.tasks, config, seed);
}

const ParamSet kBase{50.0, 1.0};
}  // namespace

TEST(TdrExperiment, ParamSetScalesAllPriorities)
{
  // γ 只乘共同倍率，P0～P3 的相對比例不變
  const ScoringParams p = applyParamSet(ScoringParams{}, ParamSet{100.0, 0.5});
  EXPECT_DOUBLE_EQ(p.energy_weight, 100.0);
  EXPECT_DOUBLE_EQ(p.half_life_by_priority[0], 30.0);
  EXPECT_DOUBLE_EQ(p.half_life_by_priority[3], 300.0);
  EXPECT_EQ(halfLifeLabel(2.0), "120/240/600/1200");
  EXPECT_THROW(makeScenario("S9", 1), std::invalid_argument);
}

TEST(TdrExperiment, ConvergenceAndBidsAreRecorded)
{
  // S1、不掉包：任務收斂（< 2 s）、兩隻狗都出價、記到分項、沒有重複認領；實際折扣價值照固定的 half_life 算
  const MissionResult r = runCase("S1", 2, kBase, 0.0, true);
  ASSERT_EQ(r.tasks.size(), 1u);
  const TaskRecord & t = r.tasks[0];
  ASSERT_GE(t.converged_at, 0.0);
  EXPECT_LT(t.converged_at - t.inserted, 2.0);
  EXPECT_EQ(t.bidders, 2);
  EXPECT_GT(t.score_gap, 0.0);
  EXPECT_LE(t.score_gap, 1.0);
  EXPECT_EQ(t.duplicate_time, 0.0);
  ASSERT_EQ(r.bids.size(), 2u);
  for (const BidRecord & b : r.bids) {
    EXPECT_TRUE(b.feasible);
    EXPECT_GT(b.eta, 0.0);
    EXPECT_GT(b.time_reward, 0.0);
    EXPECT_GE(b.energy_penalty, 0.0);
    EXPECT_EQ(b.winner, b.agent == t.converged_winner);
  }
  ASSERT_GE(t.arrived_at, 0.0);
  EXPECT_NEAR(t.discounted_value, 100.0 * std::exp2(-t.eta() / 60.0), 1e-6);   // seed 2 → P0
  const RunSummary s = summarize(r);
  EXPECT_TRUE(s.conv_5s);
  EXPECT_EQ(s.completed, 1);
}

TEST(TdrExperiment, S1UrgentTaskGoesToFasterAgent)
{
  // 能量項相同、只有到達時間不同：不管參數組，緊急任務都給快的（51）
  for (std::uint32_t seed : {2u, 3u, 4u, 5u}) {
    for (const ParamSet & set : {ParamSet{0.0, 0.5}, ParamSet{50.0, 1.0}, ParamSet{200.0, 2.0}}) {
      const MissionResult r = runCase("S1", seed, set);
      EXPECT_EQ(r.tasks[0].first_executor, 51) << "seed " << seed << " w_E " << set.energy_weight;
    }
  }
}

TEST(TdrExperiment, S2EnergyWeightAvoidsLowBattery)
{
  // 速度、距離相同：w_E = 0 時平手比機號，給了電量少的 50；w_E > 0 時給電量多的 51
  for (std::uint32_t seed : {1u, 2u, 3u}) {
    EXPECT_EQ(runCase("S2", seed, ParamSet{0.0, 1.0}).tasks[0].first_executor, 50) << "seed " << seed;
    EXPECT_EQ(runCase("S2", seed, kBase).tasks[0].first_executor, 51) << "seed " << seed;
  }
}

TEST(TdrExperiment, S3PriorityChangesTheTradeOff)
{
  // 基準參數：P0（偶數 seed）給快但耗電的 50；P3（奇數 seed）給慢但省電的 51；w_E = 0 時兩者都給快的
  for (std::uint32_t seed : {2u, 4u}) {
    const MissionResult r = runCase("S3", seed, kBase);
    EXPECT_EQ(r.tasks[0].priority, 0);
    EXPECT_EQ(r.tasks[0].first_executor, 50) << "seed " << seed;
  }
  for (std::uint32_t seed : {3u, 5u}) {
    const MissionResult r = runCase("S3", seed, kBase);
    EXPECT_EQ(r.tasks[0].priority, 3);
    EXPECT_EQ(r.tasks[0].first_executor, 51) << "seed " << seed;
    EXPECT_EQ(runCase("S3", seed, ParamSet{0.0, 1.0}).tasks[0].first_executor, 50) << "seed " << seed;
  }
}

TEST(TdrExperiment, S4PreemptionIsNotAWrongAccept)
{
  // 舊任務執行中插入 P0：兩個都做完；被搶占的不算接錯，舊任務的延後有記錄
  for (std::uint32_t seed : {1u, 2u, 3u}) {
    const MissionResult r = runCase("S4", seed, kBase);
    EXPECT_EQ(r.count(TaskOutcome::DONE), 2) << "seed " << seed;
    EXPECT_EQ(r.wrong_accepts, 0) << "seed " << seed;
    EXPECT_GE(r.tasks[0].delay, 0.0) << "seed " << seed;
    EXPECT_EQ(r.preempted, r.tasks[0].preemptions) << "seed " << seed;
  }
}

TEST(TdrExperiment, SameSeedSameRunWithLoss)
{
  // 同一個 seed（含 30 % 掉包）結果相同：參數組之間的配對比較要靠這個
  const RunSummary a = summarize(runCase("S5", 7, kBase, 0.3));
  const RunSummary b = summarize(runCase("S5", 7, kBase, 0.3));
  EXPECT_EQ(a.winner_map, b.winner_map);
  EXPECT_DOUBLE_EQ(a.converge_s, b.converge_s);
  EXPECT_DOUBLE_EQ(a.energy_used, b.energy_used);
  // 情境本身只由 seed 決定，和參數無關
  const ScenarioCase c1 = makeScenario("S6", 9);
  const ScenarioCase c2 = makeScenario("S6", 9);
  ASSERT_EQ(c1.tasks.size(), c2.tasks.size());
  for (std::size_t k = 0; k < c1.tasks.size(); ++k) {
    EXPECT_DOUBLE_EQ(c1.tasks[k].insert_at, c2.tasks[k].insert_at);
    EXPECT_DOUBLE_EQ(c1.tasks[k].task.position.x, c2.tasks[k].task.position.x);
  }
}

TEST(TdrExperiment, DogDrainsPerMeter)
{
  // 狗的實際耗電有每公尺的部分：走 30 m（不算待機）約用掉 30 × 0.1 %
  MissionAgent dog;
  dog.state.id = 50;
  dog.state.type = AgentType::UGV;
  dog.state.battery = 80.0;
  dog.state.cruise_speed = 1.0;
  dog.state.energy_per_meter = 0.1;
  dog.state.hover_energy_per_sec = 0.0;
  dog.drain_rate = 0.0;
  dog.drain_per_meter = 0.1;
  MissionTask m;
  m.task.id = makeTaskId(50, 1);
  m.task.type = TaskType::NONE;
  m.task.platform = 2;
  m.task.position = {30.0, 0.0, 0.0};
  m.task.explicit_deadline = false;
  m.label = "T1";
  const MissionResult r = runMission({dog}, {m}, MissionConfig{}, 1);
  ASSERT_EQ(r.count(TaskOutcome::DONE), 1);
  EXPECT_NEAR(r.agents[0].energy_used, 3.0, 0.1);
  EXPECT_NEAR(r.agents[0].min_battery, 77.0, 0.1);
}

TEST(TdrExperiment, S6FleetRespectsPlatformAndDistinctRoles)
{
  // 多任務機隊、不掉包：沒有重複認領；平台限定的任務只給對的平台；角色組的前、後由不同的狗做
  for (std::uint32_t seed : {1u, 2u}) {
    const MissionResult r = runCase("S6", seed, kBase);
    AgentId front = kNoAgent;
    AgentId rear = kNoAgent;
    for (const TaskRecord & t : r.tasks) {
      EXPECT_EQ(t.duplicate_time, 0.0) << t.label;
      if (t.done_by == kNoAgent) {
        continue;
      }
      if (t.platform == 1) {
        EXPECT_LT(t.done_by, 50) << t.label;
      } else if (t.platform == 2) {
        EXPECT_GE(t.done_by, 50) << t.label;
      }
      front = t.label == "G_front" ? t.done_by : front;
      rear = t.label == "G_rear" ? t.done_by : rear;
    }
    if (front != kNoAgent && rear != kNoAgent) {
      EXPECT_NE(front, rear) << "seed " << seed;
    }
    // 底線：15 個任務大部分做得完（無人機 0.5 m/s、模擬電池依時間下降，電量 60 % 的那台約 360 s 就返航）
    EXPECT_GE(summarize(r).completed, 10) << "seed " << seed;
  }
}
