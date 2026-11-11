// 避碰（avoidance.hpp，2026-10-24）：路權、最近點預測，以及用簡化的動態（速度一階追隨命令、時間常數 0.3 s、
// 位置廣播 10 Hz）模擬多台同高度飛行。參數用 uav_bt.yaml 的值（0.5 m/s），另外幾項用 itest --speed 3 的值。
// 每個情境的要求：不低於 safe_radius（或 0.9 ×，有延遲時）、都到目標、交會時不停下來等（離目標 > 1 m 卻 < 0.1 m/s 不超過 1 s）
#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "uav_px4_bt/avoidance.hpp"

using namespace uav_px4_bt;

namespace
{
CbfParams yamlParams()
{
  CbfParams p;
  p.max_speed = 0.5;
  p.brake_accel = 0.5;
  p.safe_radius = 1.0;
  p.gamma = 1.0;
  p.influence_radius = 3.0;
  p.yield_radius = 2.0;
  p.ttc_horizon = 5.0;
  p.miss_clearance = 1.5;
  p.guard_radius = 1.3;
  p.yield_exit = 2.0;
  p.emergency_speed = 0.3;
  return p;
}

// itest --speed 3 給的（itest.py 的 cbf_speed_args；二階 CBF 時剎車距離由 CBF 自己處理，guard_radius 不用加大）
CbfParams fastParams()
{
  CbfParams p = yamlParams();
  p.max_speed = 3.0;
  p.miss_clearance = 2.1;
  p.yield_exit = 2.6;
  return p;
}

struct Agent
{
  Vec3 p;
  Vec3 v;
  Vec3 goal;
  bool busy{true};          // 執行中（false = 待命）
};

struct Result
{
  double min_distance{1e9};
  bool all_reached{false};
  std::vector<double> path;         // 每台飛的水平距離
  std::vector<double> stall;        // 每台離目標 > 1 m 卻 < 0.1 m/s 最長連續多久（s；開始 1 s 不算）
  std::vector<double> min_speed;    // 每台離目標 > 1.5 m、開始 2 s 後的最低水平速度
  std::vector<std::vector<AvoidEvent>> events;
  bool emergency{false};
  std::pair<double, double> first_emergency{0.0, 0.0};   // （機號, 秒）
  std::vector<Agent> final_state;   // 結束時每台的位置、速度、目標（診斷用）
  std::vector<std::string> trace;   // 每台角色開始／解除的時刻（診斷用）
};

// agents[i] 的機號 = i + 1；task_hover = 執行中而且離目標 < 0.5 m（同 UavContext::moveTo）。
// 平滑層的參考速度 = 上一步的命令（二階 CBF 的加速度上限＝平滑層的，平滑層不會改掉它）；實際速度一階追隨命令。
// script：每一步開始時呼叫，可以改目標（例如懸停的那台接到下一個任務）
Result simulate(std::vector<Agent> agents, const CbfParams & params, double duration = 120.0,
  const std::function<void(double, std::vector<Agent> &)> & script = nullptr)
{
  const double dt = 0.05, tau = 0.3;
  const std::size_t n = agents.size();
  Result r;
  r.path.assign(n, 0.0);
  r.stall.assign(n, 0.0);
  r.min_speed.assign(n, 1e9);
  r.events.resize(n);
  std::vector<double> still(n, 0.0);
  std::vector<CollisionAvoider> av(n);
  std::vector<Vec3> vref(n);
  for (std::size_t i = 0; i < n; ++i) {
    vref[i] = agents[i].v;
  }
  auto hover = [](const Agent & a) {   // 停住＝已到目標（同 UavContext::moveTo、位置廣播 bit 3；2026-11-07 不看 busy）
      return std::hypot(a.goal.x - a.p.x, a.goal.y - a.p.y) < 0.5;
    };
  std::vector<Agent> beacon = agents;
  double beacon_time = 0.0;
  for (double t = 0.0; t < duration; t += dt) {
    if (script) {
      script(t, agents);
    }
    if (t - beacon_time >= 0.1 - 1e-9) {
      beacon = agents;
      beacon_time = t;
    }
    std::vector<Vec3> cmd(n);
    for (std::size_t i = 0; i < n; ++i) {
      std::vector<Neighbor> nb;
      for (std::size_t j = 0; j < n; ++j) {
        if (j != i) {
          nb.push_back(Neighbor{static_cast<std::uint16_t>(j + 1), beacon[j].p, beacon[j].v, t - beacon_time,
              beacon[j].busy, hover(beacon[j])});
        }
      }
      const AvoidSelf self{agents[i].p, agents[i].goal,
        {hover(agents[i]), static_cast<std::uint16_t>(i + 1)}, agents[i].v, vref[i], dt};
      const AvoidOutput out = av[i].step(self, nb, params);
      vref[i] = out.velocity;
      r.events[i].insert(r.events[i].end(), out.events.begin(), out.events.end());
      for (const AvoidEvent & e : out.events) {
        if (e.kind == AvoidEvent::Kind::START || e.kind == AvoidEvent::Kind::RELEASE) {
          r.trace.push_back("t=" + std::to_string(t).substr(0, 6) + " uav" + std::to_string(i + 1) +
            (e.kind == AvoidEvent::Kind::START ? " START nb=" : " RELEASE nb=") + std::to_string(e.neighbor) +
            " yields=" + std::to_string(e.self_yields) + " dist=" + std::to_string(e.distance).substr(0, 5) +
            " speed=" + std::to_string(std::hypot(agents[i].v.x, agents[i].v.y)).substr(0, 5));
        }
      }
      if (out.emergency && !r.emergency) {
        r.first_emergency = {static_cast<double>(i + 1), t};   // 第一次緊急退讓：哪一台（機號）、第幾秒（診斷用）
      }
      r.emergency = r.emergency || out.emergency;
      cmd[i] = out.velocity;
    }
    for (std::size_t i = 0; i < n; ++i) {
      const Vec3 before = agents[i].p;
      agents[i].v = agents[i].v + (cmd[i] - agents[i].v) * (dt / tau);
      agents[i].p = agents[i].p + agents[i].v * dt;
      r.path[i] += std::hypot(agents[i].p.x - before.x, agents[i].p.y - before.y);
      const double to_goal = std::hypot(agents[i].goal.x - agents[i].p.x, agents[i].goal.y - agents[i].p.y);
      const double speed = std::hypot(agents[i].v.x, agents[i].v.y);
      still[i] = (to_goal > 1.0 && t > 1.0 && speed < 0.1) ? still[i] + dt : 0.0;
      r.stall[i] = std::max(r.stall[i], still[i]);
      if (to_goal > 1.5 && t > 2.0) {
        r.min_speed[i] = std::min(r.min_speed[i], speed);
      }
    }
    for (std::size_t i = 0; i < n; ++i) {
      for (std::size_t j = i + 1; j < n; ++j) {
        r.min_distance = std::min(r.min_distance,
            std::hypot(agents[i].p.x - agents[j].p.x, agents[i].p.y - agents[j].p.y));
      }
    }
    bool reached = true;
    for (const Agent & a : agents) {
      reached = reached && norm(a.goal - a.p) < 0.3;
    }
    if (reached) {
      r.all_reached = true;
      r.final_state = agents;
      return r;
    }
  }
  r.final_state = agents;
  return r;
}

// 診斷：沒到的時候印每台最後的位置、速度、目標，和角色開始／解除的時序
std::string dump(const Result & r)
{
  std::string m;
  for (std::size_t i = 0; i < r.final_state.size(); ++i) {
    const Agent & a = r.final_state[i];
    m += "uav" + std::to_string(i + 1) + " p=(" + std::to_string(a.p.x).substr(0, 6) + "," +
      std::to_string(a.p.y).substr(0, 6) + ") goal=(" + std::to_string(a.goal.x).substr(0, 6) + "," +
      std::to_string(a.goal.y).substr(0, 6) + ") speed=" + std::to_string(std::hypot(a.v.x, a.v.y)).substr(0, 5) + "\n";
  }
  for (const std::string & line : r.trace) {
    m += "  " + line + "\n";
  }
  return m;
}

const AvoidEvent * firstStart(const std::vector<AvoidEvent> & ev)
{
  for (const AvoidEvent & e : ev) {
    if (e.kind == AvoidEvent::Kind::START) {
      return &e;
    }
  }
  return nullptr;
}
}  // namespace

TEST(RightOfWay, KeyOrder)
{
  // 2026-11-07：停住的優先（移動中的讓）；兩台一樣時機號小的優先（機號大的讓）。不看 busy、任務類型
  EXPECT_TRUE(hasRightOfWay({true, 9}, {false, 1}));
  EXPECT_FALSE(hasRightOfWay({false, 1}, {true, 9}));
  EXPECT_TRUE(hasRightOfWay({false, 2}, {false, 3}));
  EXPECT_FALSE(hasRightOfWay({false, 3}, {false, 2}));
  EXPECT_TRUE(hasRightOfWay({true, 2}, {true, 3}));
  // 兩台用同一份資料算：一定一台有、一台沒有
  for (bool h1 : {false, true}) {
    for (bool h2 : {false, true}) {
      const RightOfWayKey a{h1, 2}, b{h2, 3};
      EXPECT_NE(hasRightOfWay(a, b), hasRightOfWay(b, a));
    }
  }
}

TEST(ConflictPredictor, FiniteSegmentCpa)
{
  // 正對面 10 m、相對速度 1 m/s：10 s 後重疊；窗口 5 s 時最近點在 5 s（還有 5 m）
  const Cpa a = predictCpa(10.0, 0.0, -1.0, 0.0, 5.0);
  EXPECT_TRUE(a.approaching);
  EXPECT_NEAR(a.t, 5.0, 1e-9);
  EXPECT_NEAR(a.d, 5.0, 1e-9);
  // 遠離中：不算靠近，d = 現在的距離
  const Cpa b = predictCpa(2.0, 0.0, 1.0, 0.0, 5.0);
  EXPECT_FALSE(b.approaching);
  EXPECT_NEAR(b.d, 2.0, 1e-9);
  // 沒有相對運動
  EXPECT_FALSE(predictCpa(1.0, 1.0, 0.0, 0.0, 5.0).approaching);
}

TEST(ConflictPredictor, NeighborArrivesAfterSelfStops)
{
  // 自己 0.1 m/s、1 s 後停在目標；鄰機在 10 m 外以 2.5 m/s 正對飛來（4 s 後經過）。只看到自己到目標（1 s）時最近點還有
  // 7.4 m，看不到；兩段一起算：停住之後鄰機飛過來，最近點在 4 s 時、幾乎重疊（2026-11-06，D7 的原因）
  const Cpa old = predictCpa(-10.0, 0.0, 0.1 + 2.5, 0.0, 1.0);
  EXPECT_GT(old.d, 7.0);
  const Cpa c = predictCpaStop(-10.0, 0.0, 0.1, 0.0, -2.5, 0.0, 1.0, 5.0);
  EXPECT_TRUE(c.approaching);
  EXPECT_LT(c.d, 0.1);
  EXPECT_NEAR(c.t, 4.0, 0.05);
  // 停住之前就最近（還沒到目標時對方從旁邊過去）：和單段一樣
  const Cpa a = predictCpaStop(-10.0, 1.0, 0.0, 0.0, -2.0, 0.0, 10.0, 5.0);
  const Cpa b = predictCpa(-10.0, 1.0, 2.0, 0.0, 5.0);
  EXPECT_NEAR(a.d, b.d, 1e-9);
  EXPECT_NEAR(a.t, b.t, 1e-9);
}

TEST(Avoidance, OffsetPassNoConstraintNoSlowdown)
{
  // 航線從懸停的鄰機旁 2 m 經過（> miss_clearance 1.5 m）：不算衝突、不加約束、不減速
  // （原本的 CBF 一直開著：路線錯開 2.36 m 也被砍速度，2026-10-21 影片）
  const CbfParams p = yamlParams();
  const Result r = simulate({{{0, 0, 8}, {}, {30, 0, 8}}, {{15, 2.0, 8}, {}, {15, 2.0, 8}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_EQ(firstStart(r.events[0]), nullptr);
  EXPECT_EQ(firstStart(r.events[1]), nullptr);
  EXPECT_GT(r.min_speed[0], 0.45);
  EXPECT_LT(r.path[0], 30.3);
}

TEST(Avoidance, DepartureTowardPassingNeighbor)
{
  // 2026-10-24 yard SITL 的 0.88 m，照當時的順序：uav3（機號 3）在 G1 (−25, −7.5) 執行任務、懸停；uav2（機號 2）往東北
  // 飛（直線幾乎正對 uav3）。照路權，任務懸停的 uav3 優先，uav2 提早用 VO 繞開；uav2 接近到 2 m 時 uav3 接到往東南的
  // G2、起步——角色已經鎖定（uav2 讓），uav3 不會朝 uav2 起步撞上。兩台都到、不低於 safe_radius、沒有緊急退讓、uav2 不停
  const CbfParams p = yamlParams();
  const Vec3 dir{0.54, 0.84, 0.0};
  const Vec3 g1{-25.0, -7.5, 8};
  bool departed = false;
  const Result r = simulate({{{-60, -60, 8}, {}, {-60, -60, 8}, false},
      {Vec3{-25.6, -8.8, 8} - dir * 6.0, dir * 0.5, {-5, 25, 8}},
      {g1, {}, g1}}, p, 150.0,
      [&](double, std::vector<Agent> & a) {
        if (!departed && std::hypot(a[1].p.x - a[2].p.x, a[1].p.y - a[2].p.y) < 2.0) {
          a[2].goal = {5, -22, 8};
          departed = true;
        }
      });
  EXPECT_TRUE(departed);
  EXPECT_TRUE(r.all_reached) << dump(r);
  const AvoidEvent * s2 = firstStart(r.events[1]);
  ASSERT_NE(s2, nullptr);
  EXPECT_EQ(s2->neighbor, 3u);
  EXPECT_TRUE(s2->self_yields);               // 任務懸停的 uav3 優先：uav2 讓
  EXPECT_GE(r.min_distance, p.safe_radius);
  EXPECT_LT(r.stall[1], 1.0);
  EXPECT_FALSE(r.emergency);
}

TEST(Avoidance, HeadOnNoStall)
{
  // 正對面互換：機號大的用 VO 往右偏，兩台都不停住
  const CbfParams p = yamlParams();
  const Result r = simulate({{{0, 0, 8}, {}, {20, 0, 8}}, {{20, 0, 8}, {}, {0, 0, 8}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GE(r.min_distance, p.safe_radius);
  EXPECT_LT(r.stall[0], 1.0);
  EXPECT_LT(r.stall[1], 1.0);
  ASSERT_NE(firstStart(r.events[1]), nullptr);
  EXPECT_TRUE(firstStart(r.events[1])->self_yields);
  EXPECT_GT(r.min_speed[0], 0.45);            // 直行的那台不減速
}

TEST(Avoidance, CrossingNoStall)
{
  const CbfParams p = yamlParams();
  const Result r = simulate({{{0, 0, 8}, {}, {20, 0, 8}}, {{10, -10, 8}, {}, {10, 10, 8}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GE(r.min_distance, p.safe_radius);
  EXPECT_LT(r.stall[0], 1.0);
  EXPECT_LT(r.stall[1], 1.0);
}

TEST(Avoidance, TaskHoverHasRightOfWay)
{
  // 航線正好穿過一台在任務點懸停的鄰機（機號比較大）：停著的優先 → 懸停的不動，飛的那台繞過去、不停
  const CbfParams p = yamlParams();
  const Result r = simulate({{{0, 0, 8}, {}, {20, 0, 8}}, {{10, 0.2, 8}, {}, {10, 0.2, 8}}}, p);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GE(r.min_distance, p.safe_radius);
  EXPECT_LT(r.stall[0], 1.0);
  EXPECT_LT(r.path[1], 0.1);                  // 懸停的沒被推開
  ASSERT_NE(firstStart(r.events[0]), nullptr);
  EXPECT_TRUE(firstStart(r.events[0])->self_yields);
}

TEST(Avoidance, BusyGoesAroundParkedIdle)
{
  // 待命的停在執行中那台的航線上（機號比較小）：停著的優先（2026-11-06 使用者：不管停著的原因，都由移動中的繞開；
  // 原本是待命的往旁邊移開）→ 執行中的繞過去、不停，待命的不動
  const CbfParams p = yamlParams();
  const Result r = simulate({{{10, 0, 8}, {}, {10, 0, 8}, false}, {{0, 0.1, 8}, {}, {20, 0.1, 8}}}, p, 120.0);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GE(r.min_distance, p.safe_radius);
  EXPECT_LT(r.stall[1], 1.0);
  EXPECT_LT(r.path[0], 0.1);
  ASSERT_NE(firstStart(r.events[1]), nullptr);
  EXPECT_TRUE(firstStart(r.events[1])->self_yields);
}

TEST(Avoidance, HomingPassesParkedNeighborAtSpeed)
{
  // 2026-11-05 yard SITL 的 0.41 m（D7）：uav3 已經停在起飛點 (−40, −20) 上空待命，uav2 以 3 m/s 從北邊回 (−40, −25)，
  // 直線航線偏差約 0.4 m 穿過 uav3。停著的優先 → uav2 提早用 VO 繞開；uav3 不動、沒有緊急退讓、不低於 safe_radius
  const CbfParams p = fastParams();
  const Vec3 from{-42.4, 11.7, 8}, home2{-40, -25, 8};
  const Vec3 dir = (home2 - from) * (1.0 / norm(home2 - from));
  const Result r = simulate({{{-40, -30, 8}, {}, {-40, -30, 8}, false},
      {from, dir * 3.0, home2, false},
      {{-39.9, -20, 8}, {}, {-39.9, -20, 8}, false}}, p, 60.0);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GE(r.min_distance, p.safe_radius);
  EXPECT_FALSE(r.emergency);
  EXPECT_LT(r.path[2], 0.3);                  // 停著的 uav3 沒被推開
  const AvoidEvent * s2 = firstStart(r.events[1]);
  ASSERT_NE(s2, nullptr);
  EXPECT_EQ(s2->neighbor, 3u);
  EXPECT_TRUE(s2->self_yields);
}

TEST(Avoidance, SpawnPadIdleStaysStill)
{
  // 起降板上三台待命、間隔 2 m：沒有衝突、不互繞、不動
  const CbfParams p = yamlParams();
  std::vector<Agent> a;
  for (int i = 0; i < 3; ++i) {
    a.push_back({{-40, -30.0 + 2.0 * i, 8}, {}, {-40, -30.0 + 2.0 * i, 8}, false});
  }
  const Result r = simulate(a, p, 10.0);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(firstStart(r.events[i]), nullptr);
    EXPECT_LT(r.path[i], 0.05);
  }
}

TEST(Avoidance, FastHomingSwapNextToNeighborsHome)
{
  // 3 m/s 回起飛點時兩台位置對調（2026-10-24 yard SITL 相撞的情況）。停機點間隔 5 m（yard 的 spawn_spacing，2026-10-24
  // 從 2 m 改：3 m/s 剎停要約 3.9 m，間隔 2 m 時讓的那台被夾住、沒有解）。不低於 0.9 × safe_radius、都回到起飛點、不繞大圈
  const CbfParams p = fastParams();
  const Result r = simulate({{{-40, -30, 8}, {}, {-40, -30, 8}, false},
      {{-40.2, -14.0, 8}, {}, {-40, -25, 8}, false},
      {{-40.2, -26.5, 8}, {}, {-40, -20, 8}, false}}, p, 60.0);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GE(r.min_distance, 0.9 * p.safe_radius);
  // 同一條線上互換：讓的那台一定要側移讓對方過去，出去再回來
  EXPECT_LT(r.path[1], 11.0 + 2.0 * p.miss_clearance);
  EXPECT_LT(r.path[2], 6.5 + 2.0 * p.miss_clearance);
}

TEST(Avoidance, HomingPassesNeighborParkedAtHome)
{
  // 2026-11-05 yard SITL 的 0.41 m（D7）：uav3 已經回到起飛點 (−40, −20) 上空待命（還在修正幾公分，約 0.1 m/s），
  // uav2 以 3 m/s 從北邊回 (−40, −25)，直線航線穿過 uav3。2026-11-07 使用者的規則：有一台停著，就由移動中的讓開 →
  // uav2 提早（6 m 外）用 VO 繞開，uav3 不讓、沒有緊急退讓、不低於 safe_radius、都回到起飛點。
  // （11/06 的版本是「都待命 → 機號大的 uav3 讓」，用來驗證兩段預測：原本預測窗口被截成約 1 s，剩 3.8 m 才看到）
  const CbfParams p = fastParams();
  const Vec3 from{-42.4, 11.7, 8}, home2{-40, -25, 8};
  const Vec3 dir = (home2 - from) * (1.0 / norm(home2 - from));
  // uav3 的目標每秒在起飛點左右 0.12 m 切換：模擬懸停時一直在修正幾公分（名目速度約 0.1 m/s，不算「沒在動」）；
  // 不這樣做的話模擬沒有雜訊，uav3 幾秒內就完全停住，走的是「沒在動」那條路，重現不了 SITL
  const Result r = simulate({{{-40, -30, 8}, {}, {-40, -30, 8}, false},
      {from, dir * 3.0, home2, false},
      {{-39.9, -20, 8}, {}, {-40, -20, 8}, false}}, p, 60.0,
      [](double t, std::vector<Agent> & a) {
        a[2].goal = {-40.0 + (std::fmod(t, 2.0) < 1.0 ? 0.12 : -0.12), -20.0, 8.0};
      });
  EXPECT_TRUE(r.all_reached);
  EXPECT_GE(r.min_distance, p.safe_radius);
  if (r.emergency) {   // 診斷：哪一台、什麼時候、當時的距離與違反量
    std::string msg = "first emergency: uav" + std::to_string(static_cast<int>(r.first_emergency.first)) +
      " t=" + std::to_string(r.first_emergency.second) + "\n";
    for (std::size_t i = 0; i < r.events.size(); ++i) {
      for (const AvoidEvent & e : r.events[i]) {
        if (e.kind == AvoidEvent::Kind::EMERGENCY_ON || e.kind == AvoidEvent::Kind::START ||
          e.kind == AvoidEvent::Kind::RELEASE)
        {
          msg += "  uav" + std::to_string(i + 1) + " kind=" + std::to_string(static_cast<int>(e.kind)) +
            " nb=" + std::to_string(e.neighbor) + " yields=" + std::to_string(e.self_yields) +
            " dist=" + std::to_string(e.distance) + " d_cpa=" + std::to_string(e.d_cpa) +
            " t_cpa=" + std::to_string(e.t_cpa) + " worst=" + std::to_string(e.worst) + "\n";
        }
      }
    }
    ADD_FAILURE() << msg;
  }
  const AvoidEvent * s2 = firstStart(r.events[1]);
  ASSERT_NE(s2, nullptr);
  EXPECT_EQ(s2->neighbor, 3u);
  EXPECT_TRUE(s2->self_yields);               // 停著的 uav3 優先：移動中的 uav2 讓
  EXPECT_GT(s2->distance, 6.0);               // 提早看到、提早繞
  const AvoidEvent * s3 = firstStart(r.events[2]);
  EXPECT_TRUE(s3 == nullptr || !s3->self_yields);   // uav3 不讓
}

TEST(Avoidance, FastCrossingNoStall)
{
  // 3 m/s 垂直交叉：不低於 safe_radius、都不停
  const CbfParams p = fastParams();
  const Result r = simulate({{{0, 0, 8}, {}, {60, 0, 8}}, {{30, -30, 8}, {}, {30, 30, 8}}}, p, 60.0);
  EXPECT_TRUE(r.all_reached);
  EXPECT_GE(r.min_distance, p.safe_radius);
  EXPECT_LT(r.stall[0], 1.0);
  EXPECT_LT(r.stall[1], 1.0);
}

TEST(Avoidance, RolesLockedDuringEncounter)
{
  // 交會開始後，即使雙方的停住／移動改變，角色也不換
  const CbfParams p = yamlParams();
  CollisionAvoider av;
  AvoidSelf self{{0, 0, 8}, {20, 0, 8}, {false, 3}, {}, {}, 0.05};
  std::vector<Neighbor> nb{Neighbor{2, {6, 0, 8}, {-0.5, 0, 0}, 0.0, true, false}};
  av.step(self, nb, p);
  ASSERT_EQ(av.encounters().count(2), 1u);
  EXPECT_TRUE(av.encounters().at(2).self_yields);     // 兩台都在動、機號 3 > 2：讓
  self.key.stopped = true;                             // 自己變成停住（照路權應該換成對方讓）
  av.step(self, nb, p);
  ASSERT_EQ(av.encounters().count(2), 1u);
  EXPECT_TRUE(av.encounters().at(2).self_yields);     // 不換
}

TEST(Avoidance, EmergencyWhenSqueezed)
{
  // 兩台鄰機從兩側擠進 safe_radius：緊急退讓（不用投影的結果），速度往外
  const CbfParams p = yamlParams();
  CollisionAvoider av;
  const AvoidSelf self{{0, 0, 8}, {0, 0, 8}, {false, 3}, {}, {}, 0.05};
  const std::vector<Neighbor> nb{Neighbor{1, {0.7, 0.1, 8}, {}, 0.0, true, false},
    Neighbor{2, {-0.9, 0.1, 8}, {}, 0.0, true, false}};
  const AvoidOutput out = av.step(self, nb, p);
  EXPECT_TRUE(out.emergency);
  ASSERT_FALSE(out.events.empty());
  EXPECT_EQ(out.events.back().kind, AvoidEvent::Kind::EMERGENCY_ON);
  // 二階：最大加速度、對兩台的約束違反最少的方向——兩台都在 y = +0.1，往 −y 逃（離兩台都變遠），
  // 這一步的速度 = 參考速度（0）＋ accel_max × dt
  EXPECT_LT(out.velocity.y, 0.0);
  EXPECT_LT(std::fabs(out.velocity.x), std::fabs(out.velocity.y));
  EXPECT_NEAR(std::hypot(out.velocity.x, out.velocity.y), p.accel_max * self.dt, 1e-6);
  // 一階：以 emergency_speed 往外退（離比較近的那台遠一點）
  CbfParams p1 = p;
  p1.second_order = false;
  CollisionAvoider av1;
  const AvoidOutput out1 = av1.step(self, nb, p1);
  EXPECT_TRUE(out1.emergency);
  EXPECT_LT(out1.velocity.x, 0.0);
  EXPECT_NEAR(std::hypot(out1.velocity.x, out1.velocity.y), p.emergency_speed, 1e-6);
}

TEST(Cbf2, BrakesInTimeAtSpeed)
{
  // 二階 CBF：3 m/s 正對著一台不動的鄰機（6 m 外）飛、名目加速度 0（想維持速度），全額負責（ρ = 1）：
  // 照剎車距離提早剎，不低於 safe_radius，停下來時速度不會反向衝（一階只看距離，3 m/s 時剎不住，2026-10-24 SITL 0.88 m）
  CbfParams p = yamlParams();
  p.max_speed = 3.0;
  const Neighbor nb{2, {6.0, 0.0, 8.0}, {}, 0.0, true, false};
  Vec3 pos{0.0, 0.0, 8.0}, vel{3.0, 0.0, 0.0};
  const double dt = 0.05;
  double min_d = 1e9, first_brake = -1.0;
  for (int k = 0; k < 200; ++k) {
    const Cbf2Result f = cbf2Filter(pos, vel, {}, {{nb, 1.0}}, p);
    EXPECT_LE(std::hypot(f.accel.x, f.accel.y), p.accel_max + 1e-6);
    if (first_brake < 0.0 && f.accel.x < -1e-6) {
      first_brake = 6.0 - pos.x;
    }
    vel = vel + f.accel * dt;
    pos = pos + vel * dt;
    min_d = std::min(min_d, 6.0 - pos.x);
  }
  EXPECT_GE(min_d, p.safe_radius);
  EXPECT_GT(first_brake, p.safe_radius + 9.0 / (2.0 * p.accel_max));   // 剎車距離（3 m）之前就開始剎
  EXPECT_GT(vel.x, -0.05);
}

TEST(Cbf2, WidePassNotBraked)
{
  // 3 m/s 從不動的鄰機旁 3 m 經過：剎車距離夠，不用剎（加速度 = 名目的 0）
  CbfParams p = yamlParams();
  p.max_speed = 3.0;
  const Neighbor nb{2, {6.0, 3.0, 8.0}, {}, 0.0, true, false};
  const Cbf2Result f = cbf2Filter({0.0, 0.0, 8.0}, {3.0, 0.0, 0.0}, {}, {{nb, 1.0}}, p);
  EXPECT_TRUE(f.feasible);
  EXPECT_NEAR(f.accel.x, 0.0, 1e-6);
  EXPECT_NEAR(f.accel.y, 0.0, 1e-6);
}
