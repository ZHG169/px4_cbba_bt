// CPF 避碰：用簡化的動態（速度一階追隨命令、時間常數 0.3 s）模擬多台同高度飛行。
// 鄰機資料和實際一樣有延遲：位置廣播 10 Hz，用的是最後收到的一筆（age 0～0.1 s）
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "uav_px4_bt/cpf.hpp"

using namespace uav_px4_bt;

namespace
{
struct Agent
{
  Vec3 p;
  Vec3 v;
  Vec3 goal;
  bool idle{false};   // 待命：執行中的鄰機在影響範圍內時讓路（同 LandOrLoiter）
};

struct SimResult
{
  double min_distance{1e9};
  bool all_reached{false};
  double time{0.0};
};

SimResult simulate(std::vector<Agent> agents, const CpfParams & params, double duration = 60.0)
{
  const double dt = 0.05;
  const double tau = 0.3;
  SimResult r;
  std::vector<Agent> beacon = agents;   // 最後一次廣播的內容
  double beacon_time = 0.0;
  for (double t = 0.0; t < duration; t += dt) {
    if (t - beacon_time >= 0.1 - 1e-9) {
      beacon = agents;
      beacon_time = t;
    }
    std::vector<Vec3> cmd(agents.size());
    for (std::size_t i = 0; i < agents.size(); ++i) {
      std::vector<Neighbor> n;
      for (std::size_t j = 0; j < agents.size(); ++j) {
        if (j != i) {
          n.push_back(Neighbor{static_cast<std::uint16_t>(j + 1), beacon[j].p, beacon[j].v, t - beacon_time});
        }
      }
      if (agents[i].idle) {
        for (std::size_t j = 0; j < agents.size(); ++j) {
          if (j != i && !agents[j].idle && norm(beacon[j].p - agents[i].p) < params.influence_radius) {
            agents[i].goal = agents[i].p;
          }
        }
      }
      cmd[i] = cpfVelocity(agents[i].p, agents[i].v, agents[i].goal, n, params).velocity;
    }
    for (std::size_t i = 0; i < agents.size(); ++i) {
      agents[i].v = agents[i].v + (cmd[i] - agents[i].v) * (dt / tau);
      agents[i].p = agents[i].p + agents[i].v * dt;
    }
    for (std::size_t i = 0; i < agents.size(); ++i) {
      for (std::size_t j = i + 1; j < agents.size(); ++j) {
        r.min_distance = std::min(r.min_distance, norm(agents[i].p - agents[j].p));
      }
    }
    bool reached = true;
    for (const Agent & a : agents) {
      reached = reached && (a.idle || norm(a.goal - a.p) < 0.5);
    }
    if (reached) {
      r.all_reached = true;
      r.time = t;
      return r;
    }
  }
  return r;
}
}  // namespace

TEST(Cpf, NoNeighborsFliesStraightAtMaxSpeed)
{
  // 沒有鄰機：直線往目標、速度上限 max_speed；接近時減速
  CpfParams p;
  const CpfOutput far = cpfVelocity({0, 0, 5}, {}, {30, 0, 5}, {}, p);
  EXPECT_NEAR(far.velocity.x, p.max_speed, 1e-9);
  EXPECT_NEAR(far.velocity.y, 0.0, 1e-9);
  EXPECT_EQ(far.active_neighbors, 0);
  const CpfOutput near = cpfVelocity({0, 0, 5}, {}, {1, 0, 5}, {}, p);
  EXPECT_NEAR(near.velocity.x, 1.0, 1e-9);
}

TEST(Cpf, VerticalSpeedLimited)
{
  CpfParams p;
  const CpfOutput out = cpfVelocity({0, 0, 0}, {}, {0, 0, 20}, {}, p);
  EXPECT_NEAR(out.velocity.z, p.max_vertical_speed, 1e-9);
}

TEST(Cpf, StaleNeighborIgnored)
{
  // 超過 neighbor_timeout 沒更新的鄰機不算
  CpfParams p;
  const std::vector<Neighbor> n{{2, {1, 0, 5}, {}, p.neighbor_timeout + 0.1}};
  EXPECT_EQ(cpfVelocity({0, 0, 5}, {}, {0, 0, 5}, n, p).active_neighbors, 0);
}

TEST(Cpf, HeadOnSwapKeepsSafeDistance)
{
  // 兩台同高度正對面互換位置：純位勢場會卡住或擦撞；環流讓兩台各往右閃
  CpfParams p;
  const SimResult r = simulate({{{0, 0, 5}, {}, {30, 0, 5}}, {{30, 0, 5}, {}, {0, 0, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, 2.0);
  EXPECT_LT(r.time, 20.0);
}

TEST(Cpf, CrossingPathsKeepSafeDistance)
{
  // 兩條航線在 (15, 0) 垂直交叉，同時到達交叉點
  CpfParams p;
  const SimResult r = simulate({{{0, 0, 5}, {}, {30, 0, 5}}, {{15, -15, 5}, {}, {15, 15, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, 2.0);
}

TEST(Cpf, PassesHoveringVehicle)
{
  // 航線正好穿過一台在任務點懸停的鄰機：要繞過去；懸停的那台被推開後回到原位
  CpfParams p;
  const SimResult r = simulate({{{0, 0, 5}, {}, {30, 0, 5}}, {{15, 0, 5}, {}, {15, 0, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, 2.0);
}

TEST(Cpf, SpawnLineDeparture)
{
  // 出生點只隔 2 m（UAV_SPAWN_SPACING 預設）：uav1 往 +y 飛會經過 uav2、uav3 頭上（2026-10-07 SITL 撞機的情況）
  CpfParams p;
  const SimResult r = simulate({
      {{0, 0, 5}, {}, {-8, 6, 5}},
      {{0, 2, 5}, {}, {0, 2, 5}},
      {{0, 4, 5}, {}, {0, 12, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  // 一開始就只隔 2 m（< safe_radius）：不要求 > 2 m，只要求沒有更靠近
  EXPECT_GT(r.min_distance, 1.5);
}

TEST(Cpf, FourWaySwap)
{
  // 4 台從正方形四角飛到對角，同時在中心交會
  CpfParams p;
  const SimResult r = simulate({
      {{0, 0, 5}, {}, {20, 20, 5}},
      {{20, 20, 5}, {}, {0, 0, 5}},
      {{20, 0, 5}, {}, {0, 20, 5}},
      {{0, 20, 5}, {}, {20, 0, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, 2.0);
}

TEST(Cpf, IdleVehicleYieldsItsSpot)
{
  // 任務點上有一台待命（剛做完或失敗的）：不讓路的話兩台都被吸向同一點、繞圈到不了；
  // 待命的讓路（不拉回原點）時，執行中的那台到得了，而且一路保持距離（SITL 2026-10-09 發現）
  CpfParams p;
  Agent idle{{6, -10, 5}, {}, {6, -10, 5}, true};
  const SimResult r = simulate({{{0, 10, 5}, {}, {6, -10, 5}}, idle}, p, 30.0);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, 2.0);
}
