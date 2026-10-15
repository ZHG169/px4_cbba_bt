// 運動平滑層（MotionShaper）：加速度上限、到達保持、leash、重新開始；
// 和 CBF 一起飛一段（簡化的 PX4 位置控制：速度 = 前饋 + kp × 位置誤差，速度一階追隨 0.3 s）：不衝過頭、加速度平順
#include <gtest/gtest.h>

#include <cmath>

#include "uav_px4_bt/cbf.hpp"
#include "uav_px4_bt/motion.hpp"

using namespace uav_px4_bt;

TEST(Motion, AccelerationLimited)
{
  // 靜止、想要 3 m/s：參考速度每個週期最多增加 max_accel × dt，約 2 s 才到 3 m/s
  MotionShaper m;
  const double dt = 0.05;
  m.step(0.0, {0, 0, 5}, {}, {3, 0, 0}, {50, 0, 5}, false);
  Vec3 last;
  for (int i = 1; i <= 60; ++i) {
    const Setpoint sp = m.step(i * dt, {0, 0, 5}, {}, {3, 0, 0}, {50, 0, 5}, false);
    EXPECT_LE(norm(sp.velocity - last), m.params().max_accel * dt + 1e-9);
    last = sp.velocity;
    if (i == 20) {
      EXPECT_NEAR(sp.velocity.x, 1.5, 1e-6);   // 1 s × 1.5 m/s²
    }
  }
  EXPECT_NEAR(last.x, 3.0, 1e-6);
}

TEST(Motion, VerticalAccelerationLimitedSeparately)
{
  MotionShaper m;
  m.step(0.0, {0, 0, 0}, {}, {0, 0, 2}, {0, 0, 5}, false);
  const Setpoint sp = m.step(0.1, {0, 0, 0}, {}, {0, 0, 2}, {0, 0, 5}, false);
  EXPECT_NEAR(sp.velocity.z, m.params().max_vertical_accel * 0.1, 1e-9);
}

TEST(Motion, HoldsExactlyAtGoal)
{
  // 離目標 < hold_radius、沒有鄰機要閃：直接位置保持（參考位置 = 目標、速度 0）
  MotionShaper m;
  m.step(0.0, {9.9, 0, 5}, {0.2, 0, 0}, {0.1, 0, 0}, {10, 0, 5}, false);
  const Setpoint sp = m.step(0.05, {9.9, 0, 5}, {0.2, 0, 0}, {0.1, 0, 0}, {10, 0, 5}, false);
  EXPECT_DOUBLE_EQ(sp.position.x, 10.0);
  EXPECT_DOUBLE_EQ(norm(sp.velocity), 0.0);
  // 鄰機在作用（avoiding）：不直接保持，讓 CBF 把它推開
  MotionShaper m2;
  m2.step(0.0, {9.9, 0, 5}, {}, {0, 1, 0}, {10, 0, 5}, true);
  const Setpoint sp2 = m2.step(0.05, {9.9, 0, 5}, {}, {0, 1, 0}, {10, 0, 5}, true);
  EXPECT_GT(sp2.velocity.y, 0.0);
}

TEST(Motion, LeashKeepsReferenceNearVehicle)
{
  // 載具卡住不動（追不上）：參考位置不會越拉越遠
  MotionShaper m;
  m.step(0.0, {0, 0, 5}, {}, {3, 0, 0}, {50, 0, 5}, false);
  Setpoint sp;
  for (int i = 1; i <= 200; ++i) {
    sp = m.step(i * 0.05, {0, 0, 5}, {}, {3, 0, 0}, {50, 0, 5}, false);
  }
  EXPECT_LE(norm(sp.position - Vec3{0, 0, 5}), m.params().leash + 1e-9);
}

TEST(Motion, RestartsFromCurrentStateAfterGap)
{
  // 很久沒呼叫（換了動作節點）：從目前的位置、速度開始，不會跳回舊的參考位置
  MotionShaper m;
  m.step(0.0, {0, 0, 5}, {}, {3, 0, 0}, {50, 0, 5}, false);
  m.step(0.05, {0, 0, 5}, {}, {3, 0, 0}, {50, 0, 5}, false);
  const Setpoint sp = m.step(5.0, {20, 3, 5}, {1, 0, 0}, {3, 0, 0}, {50, 0, 5}, false);
  EXPECT_DOUBLE_EQ(sp.position.x, 20.0);
  EXPECT_DOUBLE_EQ(sp.position.y, 3.0);
  EXPECT_DOUBLE_EQ(sp.velocity.x, 1.0);
}

TEST(Motion, SmoothFlightWithCbfNoOvershoot)
{
  // CBF（剎車曲線）＋平滑層＋簡化的 PX4：從靜止飛 20 m、停在目標。不衝過頭、實際加速度不超過約 2 m/s²、
  // 到了之後穩定（不來回修正）
  CbfParams cp;
  cp.max_speed = 3.0;
  MotionShaper m;
  const double dt = 0.05, tau = 0.3, kp = 1.0;
  Vec3 p{0, 0, 5}, v;
  const Vec3 goal{20, 0, 5};
  double max_x = 0.0, max_acc = 0.0, arrived = -1.0;
  for (double t = 0.0; t < 30.0; t += dt) {
    const CbfOutput c = cbfVelocity(p, goal, {}, cp);
    const Setpoint sp = m.step(t, p, v, c.velocity, goal, false);
    const Vec3 cmd = sp.velocity + (sp.position - p) * kp;
    const Vec3 nv = v + (cmd - v) * (dt / tau);
    max_acc = std::max(max_acc, norm(nv - v) / dt);
    v = nv;
    p = p + v * dt;
    max_x = std::max(max_x, p.x);
    if (arrived < 0.0 && norm(goal - p) < 0.2) {
      arrived = t;
    }
  }
  EXPECT_GT(arrived, 0.0);
  EXPECT_LT(arrived, 15.0);                 // 20 m、3 m/s，加減速各約 2～3 s
  EXPECT_LT(max_x - goal.x, 0.15);          // 不衝過頭
  EXPECT_LT(max_acc, 2.5);                  // 沒有大角度傾斜
  EXPECT_LT(norm(goal - p), 0.05);          // 最後停在目標
  EXPECT_LT(norm(v), 0.05);
}

TEST(Motion, HoveringNeighborsAtSpawnSpacingDoNotPush)
{
  // 出生點只隔 2 m、兩台都懸停（沒有在靠近）：CBF 只限制靠近的速度，懸停（u = 0）不會被推（2026-10-12 前 CPF 一直互推、很晃）
  CbfParams cp;
  const std::vector<Neighbor> n{{2, {0, 2, 5}, {}, 0.05}};
  const CbfOutput out = cbfVelocity({0, 0, 5}, {0, 0, 5}, n, cp);
  EXPECT_EQ(out.active_neighbors, 0);
  EXPECT_LT(norm(out.velocity), 1e-9);
}

namespace
{
// CBF＋平滑層＋簡化的 PX4 飛到 goal，回傳沿飛行方向衝過目標的最大距離
double overshoot(const Vec3 & start, const Vec3 & goal, double tau, double seconds)
{
  CbfParams cp;
  MotionShaper m;
  const double dt = 0.05, kp = 1.0;
  Vec3 p = start, v;
  const Vec3 dir = (goal - start) * (1.0 / norm(goal - start));
  double worst = 0.0;
  for (double t = 0.0; t < seconds; t += dt) {
    const CbfOutput c = cbfVelocity(p, goal, {}, cp);
    const Setpoint sp = m.step(t, p, v, c.velocity, goal, false);
    const Vec3 cmd = sp.velocity + (sp.position - p) * kp;
    v = v + (cmd - v) * (dt / tau);
    p = p + v * dt;
    worst = std::max(worst, dot(p - goal, dir));
  }
  return worst;
}
}  // namespace

TEST(Motion, ClimbDoesNotOvershoot)
{
  // 起飛爬升 5 m（2026-10-13 SITL：衝到 6.8 m 再掉回來）：PX4 落後時參考位置不能越過目標
  EXPECT_LT(overshoot({0, 0, 0}, {0, 0, 5}, 0.3, 15.0), 0.2);
  EXPECT_LT(overshoot({0, 0, 0}, {0, 0, 5}, 0.6, 15.0), 0.3);
  EXPECT_LT(overshoot({0, 0, 5}, {20, 0, 5}, 0.6, 20.0), 0.3);
}
