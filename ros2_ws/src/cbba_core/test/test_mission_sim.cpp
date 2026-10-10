// 離線任務模擬（mission_sim）：插入時刻、完成、沒人接、接了沒飛到（返航）、交給別台、可重現
#include <gtest/gtest.h>

#include <vector>

#include "cbba_core/mission_sim.hpp"

using namespace cbba_core;

namespace
{
// 和 cbba_uav.sh 一樣的能量模型：續航 endurance 秒、5 m/s
MissionAgent uav(AgentId id, double x, double y, double battery, double endurance = 300.0)
{
  MissionAgent a;
  a.state.id = id;
  a.state.position = {x, y, 5.0};
  a.state.battery = battery;
  a.state.safety_reserve = 20.0;
  a.state.cruise_speed = 5.0;
  a.state.hover_energy_per_sec = 100.0 / endurance;
  a.state.energy_per_meter = a.state.hover_energy_per_sec / 5.0;
  a.drain_rate = a.state.hover_energy_per_sec;
  return a;
}

MissionTask task(std::uint16_t seq, double x, double y, double insert_at = 0.0, double duration = 5.0)
{
  MissionTask m;
  m.task.id = makeTaskId(1, seq);
  m.task.position = {x, y, 5.0};
  m.task.value = 80.0;
  m.task.duration_sec = duration;
  m.task.deadline_sec = 300.0;
  m.insert_at = insert_at;
  m.label = "T" + std::to_string(seq);
  return m;
}
}  // namespace

TEST(MissionSim, SingleAgentCompletesAllTasks)
{
  // 一台電量充足：兩個任務都在期限內完成，沒有接錯
  const MissionResult r = runMission({uav(1, 0, 0, 100)}, {task(1, 20, 0), task(2, 20, 20)}, MissionConfig{}, 1);
  EXPECT_EQ(r.count(TaskOutcome::DONE), 2);
  EXPECT_EQ(r.wrong_accepts, 0);
  for (const TaskRecord & t : r.tasks) {
    EXPECT_EQ(t.done_by, 1);
  }
  EXPECT_EQ(r.agents[0].completed, 2);
  EXPECT_LT(r.end_time, 60.0);
  EXPECT_NEAR(r.agents[0].distance, 40.0, 1.5);
}

TEST(MissionSim, TaskInsertedAtItsTime)
{
  // 插入的任務在插入時刻才出現，之後才被指派
  const MissionResult r = runMission({uav(1, 0, 0, 100)}, {task(1, 10, 0), task(2, -10, 0, 30.0)},
      MissionConfig{}, 1);
  ASSERT_EQ(r.count(TaskOutcome::DONE), 2);
  EXPECT_NEAR(r.tasks[1].inserted, 30.0, 0.02);
  EXPECT_GE(r.tasks[1].first_assigned, 30.0);
  EXPECT_GT(r.tasks[1].done_at, r.tasks[0].done_at);
}

TEST(MissionSim, TwoAgentsSplitByDistance)
{
  // 兩台各在一個任務旁邊：各做近的那一個
  const MissionResult r = runMission({uav(1, 0, 0, 100), uav(2, 0, 40, 100)},
      {task(1, 5, 0), task(2, 5, 40)}, MissionConfig{}, 1);
  ASSERT_EQ(r.count(TaskOutcome::DONE), 2);
  EXPECT_EQ(r.tasks[0].done_by, 1);
  EXPECT_EQ(r.tasks[1].done_by, 2);
}

TEST(MissionSim, NobodyBidsWithoutBattery)
{
  // 電量只比安全存量多一點：不出價，任務記為沒人接（不是接錯）；待命到電量用完後返航
  const MissionResult r = runMission({uav(1, 0, 0, 21)}, {task(1, 40, 0)}, MissionConfig{}, 1);
  EXPECT_EQ(r.count(TaskOutcome::UNASSIGNED), 1);
  EXPECT_EQ(r.wrong_accepts, 0);
  EXPECT_GE(r.agents[0].rtl_at, 0.0);
  EXPECT_EQ(r.agents[0].rtl_task, 0u);
}

TEST(MissionSim, UnderestimatedDrainCausesWrongAccept)
{
  // 實際耗電是模型的 3 倍：出價時可行，執行中重新評估發現做不完 → 撤銷，記一次接錯
  MissionAgent a = uav(1, 0, 0, 40);
  a.drain_rate *= 3.0;
  const MissionResult r = runMission({a}, {task(1, 60, 0, 0.0, 10.0)}, MissionConfig{}, 1);
  EXPECT_EQ(r.count(TaskOutcome::FAILED), 1);
  EXPECT_EQ(r.wrong_accepts, 1);
  EXPECT_EQ(r.released, 1);
  EXPECT_EQ(r.tasks[0].failures, 1);
}

TEST(MissionSim, BatteryRtlWithTaskIsWrongAccept)
{
  // 核心來不及重新評估（重新評估的間隔拉長）：電量先降到 20%，返航時手上有任務 → 回報失敗
  MissionAgent a = uav(1, 0, 0, 40);
  a.drain_rate *= 3.0;
  MissionConfig c;
  c.comm.reevaluate_period = 1000.0;
  const MissionResult r = runMission({a}, {task(1, 60, 0, 0.0, 10.0)}, c, 1);
  EXPECT_EQ(r.count(TaskOutcome::FAILED), 1);
  EXPECT_EQ(r.wrong_accepts, 1);
  EXPECT_EQ(r.released, 0);
  EXPECT_EQ(r.agents[0].rtl_task, r.tasks[0].id);
}

TEST(MissionSim, FailedTaskTakenOverByAnotherAgent)
{
  // 近的那台耗電被低估、做不完返航；任務交回競標池，由另一台完成（記一次接錯，結果是完成）
  MissionAgent near = uav(1, 0, 0, 40);
  near.drain_rate *= 3.0;
  const MissionResult r = runMission({near, uav(2, 0, -80, 100)}, {task(1, 60, 0, 0.0, 10.0)},
      MissionConfig{}, 1);
  EXPECT_LT(r.tasks[0].first_assigned, 1.0);   // 一開始是近的那台得標
  EXPECT_EQ(r.wrong_accepts, 1);
  EXPECT_EQ(r.tasks[0].failures, 1);
  EXPECT_EQ(r.tasks[0].done_by, 2);
  EXPECT_EQ(r.count(TaskOutcome::DONE), 1);
}

TEST(MissionSim, SameSeedSameResult)
{
  // 同一個 seed（含掉包）結果相同：各權重的配對比較要靠這個
  MissionConfig c;
  c.loss = 0.3;
  const std::vector<MissionAgent> fleet{uav(1, 0, 0, 100), uav(2, 0, 2, 70), uav(3, 0, 4, 45)};
  const std::vector<MissionTask> tasks{task(1, 20, 25), task(2, -25, 20), task(3, 35, 5, 12.0),
    task(4, -35, -10, 40.0)};
  const MissionResult a = runMission(fleet, tasks, c, 7);
  const MissionResult b = runMission(fleet, tasks, c, 7);
  ASSERT_EQ(a.tasks.size(), b.tasks.size());
  for (std::size_t k = 0; k < a.tasks.size(); ++k) {
    EXPECT_EQ(a.tasks[k].done_by, b.tasks[k].done_by);
    EXPECT_DOUBLE_EQ(a.tasks[k].done_at, b.tasks[k].done_at);
  }
  EXPECT_DOUBLE_EQ(a.end_time, b.end_time);
}
