// 能量模型的單元測試（對應 energy_model.py 與 cbba_battery.py 的 path energy 測試）
#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "cbba_core/energy_model.hpp"

using namespace cbba_core;

namespace
{
Task makeTask(TaskId id, double x, double y, double duration)
{
  Task t;
  t.id = id;
  t.position = {x, y, 0.0};
  t.duration_sec = duration;
  return t;
}

AgentState makeAgent()
{
  AgentState a;
  a.position = {0.0, 0.0, 0.0};
  a.home = {0.0, 0.0, 0.0};
  a.battery = 100.0;
  a.safety_reserve = 20.0;
  a.energy_per_meter = 1.0;
  a.hover_energy_per_sec = 1.0;
  return a;
}
}  // namespace

TEST(EnergyModel, TravelAndTaskEnergy)
{
  EXPECT_DOUBLE_EQ(travelEnergy(10.0, 0.5), 5.0);
  EXPECT_DOUBLE_EQ(predictedTaskEnergy(10.0, 0.5, 2.0), 7.0);
  EXPECT_DOUBLE_EQ(remainingEnergy(100.0, 30.0), 70.0);
}

TEST(EnergyModel, RejectsInvalidInput)
{
  EXPECT_THROW(travelEnergy(-1.0, 0.5), std::invalid_argument);
  EXPECT_THROW(travelEnergy(1.0, std::nan("")), std::invalid_argument);
  EXPECT_THROW(batteryCost(100.0, -1.0, 20.0), std::invalid_argument);
}

TEST(EnergyModel, BatteryFeasibleKeepsReserve)
{
  EXPECT_TRUE(batteryFeasible(100.0, 80.0, 20.0));    // 剛好等於安全存量
  EXPECT_FALSE(batteryFeasible(100.0, 80.1, 20.0));
}

TEST(EnergyModel, BatteryCostIsFractionOfUsable)
{
  EXPECT_DOUBLE_EQ(batteryCost(100.0, 40.0, 20.0), 0.5);
  EXPECT_TRUE(std::isinf(batteryCost(100.0, 81.0, 20.0)));  // 超過可用電量
  EXPECT_TRUE(std::isinf(batteryCost(20.0, 0.0, 20.0)));    // 沒有可用電量
}

TEST(EnergyModel, PathEnergyIncludesReturnLeg)
{
  // 和 Python 測試相同的兩個任務：去程加執行 = 2+2+3+4 = 11，回程 5，共 16
  const AgentState agent = makeAgent();
  const std::vector<Task> tasks{makeTask(0, 2.0, 0.0, 2.0), makeTask(1, 5.0, 0.0, 4.0)};
  const PathEnergy e = pathEnergy(agent, tasks);
  ASSERT_EQ(e.legs.size(), 2u);
  EXPECT_DOUBLE_EQ(e.legs[0].cumulative_energy, 4.0);
  EXPECT_DOUBLE_EQ(e.legs[1].cumulative_energy, 11.0);
  EXPECT_DOUBLE_EQ(e.return_energy, 5.0);
  EXPECT_DOUBLE_EQ(e.total, 16.0);
}

TEST(EnergyModel, EmptyPathCostsNothing)
{
  AgentState agent = makeAgent();
  agent.position = {30.0, 0.0, 0.0};  // 不在 home 也不計回程：沒有任務就原地待命
  const PathEnergy e = pathEnergy(agent, {});
  EXPECT_DOUBLE_EQ(e.total, 0.0);
}
