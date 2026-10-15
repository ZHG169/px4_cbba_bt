// CBF 避碰：單步的約束是否成立，以及用簡化的動態（速度一階追隨命令、時間常數 0.3 s）模擬多台同高度飛行。
// 鄰機資料和實際一樣有延遲：位置廣播 10 Hz，用的是最後收到的一筆（age 0～0.1 s）
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "uav_px4_bt/cbf.hpp"

using namespace uav_px4_bt;

namespace
{
struct Agent
{
  Vec3 p;
  Vec3 v;
  Vec3 goal;
  bool idle{false};   // 待命：執行中的鄰機在影響範圍內時讓路（同 LandOrLoiter：待命點改成目前位置）
};

struct SimResult
{
  double min_distance{1e9};
  bool all_reached{false};
  double time{0.0};
};

SimResult simulate(std::vector<Agent> agents, const CbfParams & params, double duration = 60.0)
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
          n.push_back(Neighbor{static_cast<std::uint16_t>(j + 1), beacon[j].p, beacon[j].v, t - beacon_time,
              !beacon[j].idle});
        }
      }
      if (agents[i].idle) {
        for (std::size_t j = 0; j < agents.size(); ++j) {
          if (j != i && !agents[j].idle && norm(beacon[j].p - agents[i].p) < params.influence_radius) {
            agents[i].goal = agents[i].p;
          }
        }
      }
      cmd[i] = cbfVelocity(agents[i].p, agents[i].goal, n, params, agents[i].idle).velocity;
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

// 約束 a·u ≥ b 的左右兩邊（同 cbf.cpp），驗證用
double margin(const Vec3 & self, const Vec3 & u, const Neighbor & n, const CbfParams & p, bool yield)
{
  const double rx = self.x - n.position.x, ry = self.y - n.position.y;
  const bool y = yield && n.busy;
  const double r = y ? p.yield_radius : p.safe_radius;
  const double h = rx * rx + ry * ry - r * r;
  const double b = (y ? 2.0 * (rx * n.velocity.x + ry * n.velocity.y) : 0.0) - (y ? 1.0 : 0.5) * p.gamma * h;
  return 2.0 * (rx * u.x + ry * u.y) - b;
}
}  // namespace

TEST(Cbf, NoNeighborsFliesStraightAtMaxSpeed)
{
  // 沒有鄰機：直線往目標、速度上限 max_speed；接近時減速
  CbfParams p;
  const CbfOutput far = cbfVelocity({0, 0, 5}, {30, 0, 5}, {}, p);
  EXPECT_NEAR(far.velocity.x, p.max_speed, 1e-9);
  EXPECT_NEAR(far.velocity.y, 0.0, 1e-9);
  EXPECT_EQ(far.active_neighbors, 0);
  const CbfOutput near = cbfVelocity({0, 0, 5}, {1, 0, 5}, {}, p);
  EXPECT_NEAR(near.velocity.x, 1.0, 1e-9);
}

TEST(Cbf, VerticalSpeedLimited)
{
  CbfParams p;
  const CbfOutput out = cbfVelocity({0, 0, 0}, {0, 0, 20}, {}, p);
  EXPECT_NEAR(out.velocity.z, p.max_vertical_speed, 1e-9);
}

TEST(Cbf, StaleNeighborIgnored)
{
  // 超過 neighbor_timeout 沒更新的鄰機不算
  CbfParams p;
  const std::vector<Neighbor> n{{2, {1, 0, 5}, {}, p.neighbor_timeout + 0.1}};
  EXPECT_EQ(cbfVelocity({0, 0, 5}, {0, 0, 5}, n, p).active_neighbors, 0);
}

TEST(Cbf, OutsideInfluenceRadiusIgnored)
{
  // influence_radius 外的鄰機不加約束：直直飛過去
  CbfParams p;
  p.influence_radius = 6.0;
  const std::vector<Neighbor> n{{2, {7, 0, 5}, {}, 0.0}};
  const CbfOutput out = cbfVelocity({0, 0, 5}, {30, 0, 5}, n, p);
  EXPECT_EQ(out.active_neighbors, 0);
  EXPECT_NEAR(out.velocity.x, p.max_speed, 1e-9);
}

TEST(Cbf, ClosingSpeedLimitedByBarrier)
{
  // 正對一台懸停的鄰機飛：靠近速度 ≤ ρ γ h / (2d)（ρ = 0.5），而且往右繞（ENU：往 +x 飛、鄰機在前方 → 右手邊是 −y）
  CbfParams p;
  const std::vector<Neighbor> n{{2, {3, 0, 5}, {}, 0.0, true}};
  const CbfOutput out = cbfVelocity({0, 0, 5}, {30, 0, 5}, n, p);
  EXPECT_EQ(out.active_neighbors, 1);
  const double h = 9.0 - p.safe_radius * p.safe_radius;
  EXPECT_LE(out.velocity.x, 0.5 * p.gamma * h / (2.0 * 3.0) + 1e-6);
  EXPECT_LT(out.velocity.y, -0.1);
  EXPECT_GE(margin({0, 0, 5}, out.velocity, n[0], p, false), -1e-6);
}

TEST(Cbf, AllConstraintsSatisfiedWithSeveralNeighbors)
{
  // 三台從前、左前、右前擠過來：投影後每一個約束都成立
  CbfParams p;
  const std::vector<Neighbor> n{
    {2, {2.5, 0, 5}, {}, 0.0, true},
    {3, {2.0, 1.8, 5}, {}, 0.0, true},
    {4, {2.0, -1.8, 5}, {}, 0.0, true}};
  const CbfOutput out = cbfVelocity({0, 0, 5}, {30, 0, 5}, n, p);
  EXPECT_EQ(out.active_neighbors, 3);
  for (const Neighbor & nb : n) {
    EXPECT_GE(margin({0, 0, 5}, out.velocity, nb, p, false), -1e-6);
  }
}

TEST(Cbf, HoveringNeighborOutsideSafeRadiusDoesNotPush)
{
  // 懸停、旁邊 2 m 有鄰機（漂移、甚至慢慢靠近）：u = 0 在 h ≥ 0 時滿足約束，不會被推（CPF 2026-10-13 的互推問題）
  CbfParams p;
  const std::vector<Neighbor> drift{{2, {0, 2.0, 5}, {0, -0.3, 0}, 0.05}};
  const CbfOutput out = cbfVelocity({0, 0, 5}, {0, 0, 5}, drift, p);
  EXPECT_EQ(out.active_neighbors, 0);
  EXPECT_LT(norm(out.velocity), 1e-9);
}

TEST(Cbf, InsideSafeRadiusPushedOut)
{
  // 已經在 safe_radius 內（h < 0）：往外退
  CbfParams p;
  const std::vector<Neighbor> n{{2, {0, 0.8, 5}, {}, 0.0}};
  const CbfOutput out = cbfVelocity({0, 0, 5}, {0, 0, 5}, n, p);
  EXPECT_EQ(out.active_neighbors, 1);
  EXPECT_LT(out.velocity.y, -0.05);
}

TEST(Cbf, IdleYieldsToApproachingBusyNeighbor)
{
  // 待命（yield_to_busy）、執行中的鄰機 3 m 外以 3 m/s 飛過來：自己全額閃開；同樣的鄰機是待命的：不動
  CbfParams p;
  std::vector<Neighbor> n{{2, {0, 3, 5}, {0, -3, 0}, 0.0, true}};
  const CbfOutput yield = cbfVelocity({0, 0, 5}, {0, 0, 5}, n, p, true);
  EXPECT_EQ(yield.active_neighbors, 1);
  EXPECT_LT(yield.velocity.y, -0.5);
  EXPECT_GE(margin({0, 0, 5}, yield.velocity, n[0], p, true), -1e-6);
  n[0].busy = false;
  EXPECT_EQ(cbfVelocity({0, 0, 5}, {0, 0, 5}, n, p, true).active_neighbors, 0);
}

TEST(Cbf, HeadOnSwapKeepsSafeDistance)
{
  // 兩台同高度正對面互換位置：單純的 CBF 會停在中間（死結）；往右繞讓兩台錯開
  CbfParams p;
  const SimResult r = simulate({{{0, 0, 5}, {}, {30, 0, 5}}, {{30, 0, 5}, {}, {0, 0, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, p.safe_radius * 0.9);
  EXPECT_LT(r.time, 30.0);
}

TEST(Cbf, CrossingPathsKeepSafeDistance)
{
  // 兩條航線在 (15, 0) 垂直交叉，同時到達交叉點
  CbfParams p;
  const SimResult r = simulate({{{0, 0, 5}, {}, {30, 0, 5}}, {{15, -15, 5}, {}, {15, 15, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, p.safe_radius * 0.9);
}

TEST(Cbf, PassesHoveringVehicle)
{
  // 航線正好穿過一台在任務點懸停的鄰機（執行中）：繞過去
  CbfParams p;
  const SimResult r = simulate({{{0, 0, 5}, {}, {30, 0, 5}}, {{15, 0, 5}, {}, {15, 0, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, p.safe_radius * 0.9);
}

TEST(Cbf, SpawnLineDeparture)
{
  // 出生點只隔 2 m（UAV_SPAWN_SPACING 預設）：uav1 往 +y 飛會經過 uav2、uav3 旁邊（2026-10-07 SITL 撞機的情況）
  CbfParams p;
  const SimResult r = simulate({
      {{0, 0, 5}, {}, {-8, 6, 5}},
      {{0, 2, 5}, {}, {0, 2, 5}},
      {{0, 4, 5}, {}, {0, 12, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, p.safe_radius * 0.9);
}

TEST(Cbf, FourWaySwap)
{
  // 4 台從正方形四角飛到對角，同時在中心交會
  CbfParams p;
  const SimResult r = simulate({
      {{0, 0, 5}, {}, {20, 20, 5}},
      {{20, 20, 5}, {}, {0, 0, 5}},
      {{20, 0, 5}, {}, {0, 20, 5}},
      {{0, 20, 5}, {}, {20, 0, 5}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, p.safe_radius * 0.9);
}

TEST(Cbf, IdleVehicleYieldsItsSpot)
{
  // 任務點上有一台待命（剛做完或失敗的）：執行中的那台飛過來時，待命的讓開，執行中的到得了（SITL 2026-10-09 的情況）
  CbfParams p;
  Agent idle{{6, -10, 5}, {}, {6, -10, 5}, true};
  const SimResult r = simulate({{{0, 10, 5}, {}, {6, -10, 5}}, idle}, p, 30.0);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GT(r.min_distance, p.safe_radius * 0.9);
}

TEST(Cbf, HoveringLineStaysStillWithJitter)
{
  // 出生點一排（間隔 2 m）懸停、每台有位置控制的小抖動：不會互相推開、越晃越大
  CbfParams cp;
  std::vector<Agent> a(3);
  for (std::size_t i = 0; i < a.size(); ++i) {
    a[i].p = {0, 2.0 * i, 5};
    a[i].goal = a[i].p;
  }
  const double dt = 0.05, tau = 0.3;
  double worst = 0.0;
  for (int k = 0; k < 400; ++k) {
    std::vector<Vec3> cmd(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
      std::vector<Neighbor> n;
      for (std::size_t j = 0; j < a.size(); ++j) {
        if (j != i) {
          n.push_back(Neighbor{static_cast<std::uint16_t>(j + 1), a[j].p, a[j].v, 0.05});
        }
      }
      // 抖動：±0.15 m/s，各台相位不同
      const double s = 0.15 * std::sin(0.9 * k * dt * (i + 1) + i);
      cmd[i] = cbfVelocity(a[i].p, a[i].goal, n, cp, true).velocity + Vec3{0, s, 0.5 * s};
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
      a[i].v = a[i].v + (cmd[i] - a[i].v) * (dt / tau);
      a[i].p = a[i].p + a[i].v * dt;
      worst = std::max(worst, norm(a[i].p - a[i].goal));
    }
  }
  EXPECT_LT(worst, 0.4);
}
