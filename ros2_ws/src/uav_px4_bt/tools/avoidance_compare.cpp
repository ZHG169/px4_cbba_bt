// 避碰的離線對照（2026-10-27，報告用）：同一個場景、同一套參數，分別用
//   legacy  cbfVelocity（10/14～10/23 的做法：一階 CBF 約束一直開著，違反後往右繞）
//   new     CollisionAvoider（10/24 起：預測 → 路權 → VO 繞 → 二階 CBF 保底）
// 跑一次，輸出每台的軌跡（CSV）與指標，給 docs/cbf_compare/plot_compare.py 畫圖。
//
// 動態和正式 BT 一樣經過運動平滑層（MotionShaper，加速度上限 1.5 m/s²）；PX4 簡化成位置＋速度前饋的追隨：
//   v̇ = (v_sp + k_p (p_sp − p) − v) / τ，τ = 0.3 s、k_p = 1 /s；位置廣播 10 Hz（鄰機資料最多舊 0.1 s，用速度外推）。
// 參數：itest --speed 3 時的值（uav_bt.yaml＋cbf_speed_args：miss_clearance 2.1、yield_exit 2.6）；舊做法的
// influence_radius 照當時的公式 ≥ (V + √(V² + (0.5 γ d_s)²)) / (0.5 γ) = 12 m（3 m/s）。
// 場景點：yard 世界（G1、G2、起飛點）和 2026-10-24 SITL（20261024_235929_demo）的 X1 交會。
//
// 用法：avoidance_compare <輸出資料夾>   → <場景>_<方法>.csv、summary.csv
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "uav_px4_bt/avoidance.hpp"
#include "uav_px4_bt/motion.hpp"

using namespace uav_px4_bt;

namespace
{
constexpr double kAlt = 8.0;
constexpr double kDt = 0.05;
constexpr double kTau = 0.3;
constexpr double kKp = 1.0;

struct Agent
{
  std::string name;
  Vec3 p;
  Vec3 goal;
  bool busy{true};
};

struct Scenario
{
  std::string key;
  std::string title;
  std::vector<Agent> agents;
  double duration{60.0};
};

CbfParams params3()
{
  CbfParams p;
  p.max_speed = 3.0;
  p.k_att = 1.0;
  p.brake_accel = 0.5;
  p.brake_vertical_accel = 0.1;
  p.safe_radius = 1.0;
  p.gamma = 1.0;
  p.ttc_horizon = 5.0;
  p.miss_clearance = 2.1;
  p.guard_radius = 1.3;
  p.yield_exit = 2.6;
  p.emergency_speed = 0.3;
  p.second_order = true;
  p.accel_max = 1.5;
  p.response_delay = 0.3;
  p.max_extrapolation = 0.5;
  p.neighbor_timeout = 1.0;
  p.influence_radius = 12.0;
  p.yield_radius = 2.0;
  p.k_tan = 0.8;
  return p;
}

struct Metrics
{
  double min_distance{1e9};
  double time{0.0};
  bool reached{false};
  std::vector<double> path, stall, min_speed, arrive;
};

Metrics run(const Scenario & sc, bool legacy, const std::string & csv)
{
  const CbfParams params = params3();
  MotionParams mp;
  mp.max_accel = params.accel_max;
  std::vector<Agent> a = sc.agents;
  const std::size_t n = a.size();
  std::vector<Vec3> v(n);
  std::vector<MotionShaper> shaper(n, MotionShaper(mp));
  std::vector<CollisionAvoider> av(n);
  std::vector<Agent> beacon = a;
  std::vector<Vec3> beacon_v = v;
  double beacon_time = 0.0;
  Metrics m;
  m.path.assign(n, 0.0);
  m.stall.assign(n, 0.0);
  m.min_speed.assign(n, 1e9);
  m.arrive.assign(n, -1.0);
  std::vector<double> still(n, 0.0);
  auto hover = [](const Agent & x, const Vec3 & p) {
      return x.busy && std::hypot(x.goal.x - p.x, x.goal.y - p.y) < 0.5;
    };
  FILE * f = std::fopen(csv.c_str(), "w");
  std::fprintf(f, "t,agent,x,y,vx,vy,speed,cmd_speed\n");
  double t = 0.0;
  for (int k = 0; t < sc.duration; ++k, t = k * kDt) {
    if (t - beacon_time >= 0.1 - 1e-9 || k == 0) {
      beacon = a;
      beacon_v = v;
      beacon_time = t;
    }
    std::vector<Setpoint> sp(n);
    std::vector<double> cmd(n);
    for (std::size_t i = 0; i < n; ++i) {
      std::vector<Neighbor> nb;
      for (std::size_t j = 0; j < n; ++j) {
        if (j != i) {
          nb.push_back(Neighbor{static_cast<std::uint16_t>(j + 1), beacon[j].p, beacon_v[j], t - beacon_time,
              beacon[j].busy, hover(beacon[j], beacon[j].p)});
        }
      }
      Vec3 u;
      bool avoiding = false;
      if (legacy) {
        const CbfOutput out = cbfVelocity(a[i].p, a[i].goal, nb, params, !a[i].busy);
        u = out.velocity;
        avoiding = out.active_neighbors > 0;
      } else {
        const AvoidSelf self{a[i].p, a[i].goal, {hover(a[i], a[i].p), a[i].busy, static_cast<std::uint16_t>(i + 1)},
          v[i], shaper[i].referenceVelocity(t, v[i]), shaper[i].sinceLast(t, kDt)};
        const AvoidOutput out = av[i].step(self, nb, params);
        u = out.velocity;
        avoiding = out.active_neighbors > 0;
      }
      cmd[i] = std::hypot(u.x, u.y);
      sp[i] = shaper[i].step(t, a[i].p, v[i], u, a[i].goal, avoiding);
    }
    for (std::size_t i = 0; i < n; ++i) {
      const Vec3 before = a[i].p;
      const Vec3 want = sp[i].velocity + (sp[i].position - a[i].p) * kKp;
      v[i] = v[i] + (want - v[i]) * (kDt / kTau);
      a[i].p = a[i].p + v[i] * kDt;
      a[i].p.z = kAlt;
      v[i].z = 0.0;
      m.path[i] += std::hypot(a[i].p.x - before.x, a[i].p.y - before.y);
      const double to_goal = std::hypot(a[i].goal.x - a[i].p.x, a[i].goal.y - a[i].p.y);
      const double speed = std::hypot(v[i].x, v[i].y);
      still[i] = (to_goal > 1.0 && t > 1.0 && speed < 0.1) ? still[i] + kDt : 0.0;
      m.stall[i] = std::max(m.stall[i], still[i]);
      if (a[i].busy && to_goal > 1.5 && t > 3.0) {
        m.min_speed[i] = std::min(m.min_speed[i], speed);
      }
      if (m.arrive[i] < 0.0 && to_goal < 0.3) {
        m.arrive[i] = t;
      }
      if (k % 2 == 0) {
        std::fprintf(f, "%.2f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", t, a[i].name.c_str(), a[i].p.x, a[i].p.y,
          v[i].x, v[i].y, speed, cmd[i]);
      }
    }
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {
        m.min_distance = std::min(m.min_distance, std::hypot(a[i].p.x - a[j].p.x, a[i].p.y - a[j].p.y));
      }
    }
    bool all = true;
    for (std::size_t i = 0; i < n; ++i) {
      all = all && m.arrive[i] >= 0.0;
    }
    if (all && !m.reached) {
      m.reached = true;
      m.time = t;
    }
    if (m.reached && t > m.time + 2.0) {
      break;
    }
  }
  if (!m.reached) {
    m.time = t;
  }
  std::fclose(f);
  return m;
}

std::vector<Scenario> scenarios()
{
  const Vec3 g1{-25.0, -7.5, kAlt}, g2{5.0, -22.0, kAlt};
  std::vector<Scenario> s;
  // 1. SITL X1：uav1 做完 G2 → T1，航線經過在 M1 旁盯火、懸停中的 uav2（路權高、不讓）
  s.push_back({"x1_pass_hover", "X1：經過懸停中的無人機（SITL 10/24 的位置）",
      {{"uav1", {4.91, -22.01, kAlt}, {5.4, 10.0, kAlt}, true},
        {"uav2", {5.35, 3.99, kAlt}, {5.35, 3.99, kAlt}, true}}, 40.0});
  // 2. 對飛：G1 ↔ G2（兩台都在執行任務）
  s.push_back({"head_on", "對飛：G1 ↔ G2", {{"uav1", g1, g2, true}, {"uav2", g2, g1, true}}, 50.0});
  // 3. 垂直交叉：兩條航線在 (−10, −15) 交叉，同時到
  s.push_back({"crossing", "垂直交叉",
      {{"uav1", {-25.0, -15.0, kAlt}, {5.0, -15.0, kAlt}, true},
        {"uav2", {-10.0, -30.0, kAlt}, {-10.0, 0.0, kAlt}, true}}, 40.0});
  // 4. 起降區回程：起飛點間隔 5 m，uav2、uav3 回自己的起飛點時交錯，uav1 停在自己的起飛點待命
  s.push_back({"homing", "起降區回程（起飛點間隔 5 m）",
      {{"uav1", {-40.0, -30.0, kAlt}, {-40.0, -30.0, kAlt}, false},
        {"uav2", {-40.2, -14.0, kAlt}, {-40.0, -25.0, kAlt}, false},
        {"uav3", {-40.2, -26.5, kAlt}, {-40.0, -20.0, kAlt}, false}}, 40.0});
  return s;
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string out = argc > 1 ? argv[1] : ".";
  FILE * sum = std::fopen((out + "/summary.csv").c_str(), "w");
  std::fprintf(sum, "scenario,title,method,agent,min_distance,all_reached,time,path,stall,min_speed,arrive\n");
  for (const Scenario & sc : scenarios()) {
    for (const bool legacy : {true, false}) {
      const std::string method = legacy ? "legacy" : "new";
      const Metrics m = run(sc, legacy, out + "/" + sc.key + "_" + method + ".csv");
      for (std::size_t i = 0; i < sc.agents.size(); ++i) {
        std::fprintf(sum, "%s,%s,%s,%s,%.3f,%d,%.2f,%.2f,%.2f,%.3f,%.2f\n", sc.key.c_str(), sc.title.c_str(),
          method.c_str(), sc.agents[i].name.c_str(), m.min_distance, m.reached ? 1 : 0, m.time, m.path[i],
          m.stall[i], m.min_speed[i] > 1e8 ? -1.0 : m.min_speed[i], m.arrive[i]);
      }
      std::printf("%-14s %-6s 最近 %.2f m  全部到達 %s %.1f s\n", sc.key.c_str(), method.c_str(), m.min_distance,
        m.reached ? "是" : "否", m.time);
    }
  }
  std::fclose(sum);
  return 0;
}
