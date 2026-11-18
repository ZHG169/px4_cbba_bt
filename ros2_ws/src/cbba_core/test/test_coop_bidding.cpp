// 任務欄位補齊 09（2026-11-18，doc/cb4_design.md「協同出價」）：協同出價第二階段——有效到達 = max(自身到達、
// 前置角色達成里程碑、會合隊友到齊〔最多等 sync_window_s〕)。CbbaComm 估別台的時刻寫進 Task::coop，出價時像 release 一樣等。
// PDF 的四個驗收項目（第一階段的執行端，已經有測試）：
//   1 UAV 定位後狗的 gate 才開       Network.UavLocalizesThenDogGateOpens（test_group_progress）、itest group G2
//   2 兩隻狗都到位才同步開始         test_spec_gates 的 Sync*、cb4b_iface.sh S1、itest group G4
//   3 同一 Agent 不承接同組互斥角色   Distinct.*（test_incident）、itest group G12
//   4 必要角色失敗 → 依賴角色取消、fallback   Cascade.*（test_incident）、cb4c_iface.sh D1、itest group G9
// 這裡測第二階段：出價看協同等待
#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/gateway.hpp"
#include "cbba_core/scoring.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
constexpr AgentId kGateway = 200;

// ===========================================================================
// 出價（不經網路）
// ===========================================================================
Task groundAt(TaskId id, double x, double y, double value, double duration)
{
  Task t;
  t.id = id;
  t.type = TaskType::NONE;
  t.platform = 2;
  t.required_caps = capBit(Capability::MOBILITY_GROUND);
  t.priority = 1;   // half_life 120 s
  t.position = {x, y, 0.0};
  t.value = value;
  t.duration_sec = duration;
  t.explicit_deadline = false;
  t.deadline_sec = 300.0;
  return t;
}

AgentState dog()
{
  AgentState a;
  a.id = 50;
  a.type = AgentType::UGV;
  a.home = a.position;
  a.energy_per_meter = 0.05;
  a.hover_energy_per_sec = 0.05;
  return a;
}
}  // namespace

TEST(CoopScoring, WaitingRoleLetsOtherTaskGoFirst)
{
  // 依賴角色 G（價值 100）要等前置角色到 60 s；巡檢 P（80）在旁邊。規劃速度 4 m/s，兩個都 2 s 到。
  // 不看協同：G 先（G 98.9 + P 73.4 > P 79.1 + G 86.6）；看協同：G 反正要等到 60 s，先做 P 再去 G 等（149.8 > 123.2）
  const double now = 1000.0;
  ScoringParams p;
  Task g = groundAt(1, 8.0, 0.0, 100.0, 10.0);
  const Task pt = groundAt(2, 0.0, 8.0, 80.0, 20.0);
  const auto better_g_first = [&]() {
      return evaluatePath(dog(), {g, pt}, now, p).score > evaluatePath(dog(), {pt, g}, now, p).score;
    };
  EXPECT_TRUE(better_g_first());
  g.coop.ready = now + 60.0;
  EXPECT_FALSE(better_g_first());
  const PathEval e = evaluatePath(dog(), {pt, g}, now, p);
  EXPECT_NEAR(e.legs[1].arrival, 60.0, 1e-6);   // P 做完 24.8 s 到 G，等到 60 s
  EXPECT_NEAR(e.legs[1].wait, 60.0 - (22.0 + std::hypot(8.0, 8.0) / 4.0), 1e-6);
  // 出價也跟著低（等待的時間價值衰減）
  Task g0 = g;
  g0.coop = CoopEta{};
  EXPECT_LT(bestInsertion(dog(), {}, g, now, p).score, bestInsertion(dog(), {}, g0, now, p).score - 20.0);
}

TEST(CoopScoring, SyncWaitsForTeammatesUpToWindow)
{
  // 會合：自己 2 s 到。隊友最晚 100 s → 從第一個到的（自己 2 s）起最多等 sync_window 10 s → 12 s 開始；
  // 隊友最晚 8 s → 8 s 開始；隊友都比自己早 → 不用等；sync_window 0 → 不看
  const double now = 1000.0;
  ScoringParams p;
  Task g = groundAt(1, 8.0, 0.0, 100.0, 10.0);
  g.ext.sync_window_s = 10;
  const auto start = [&](double first, double last) {
      g.coop.sync_first = now + first;
      g.coop.sync_last = now + last;
      return evaluatePath(dog(), {g}, now, p).legs[0].arrival;
    };
  EXPECT_NEAR(start(5.0, 100.0), 12.0, 1e-6);
  EXPECT_NEAR(start(5.0, 8.0), 8.0, 1e-6);
  EXPECT_NEAR(start(1.0, 1.5), 2.0, 1e-6);
  EXPECT_NEAR(start(-3.0, 30.0), 7.0, 1e-6);   // 隊友 3 s 前就到了：從那時起算 10 s
  g.ext.sync_window_s = 0;
  EXPECT_NEAR(start(5.0, 100.0), 2.0, 1e-6);
}

// ===========================================================================
// 模擬網路：閘道＋UAV（1）＋狗；role 0 = 空中（UAV），role 1 = 地面（等 role 0）
// ===========================================================================
namespace
{
CommConfig coopConfig(bool coop = true)
{
  CommConfig cc;
  cc.epoch = 3;
  cc.coop_bidding = coop;
  cc.platform_speed = {{0.0, 5.0, 5.0}};   // 和 harness 的載具一樣（沒設 cruise_speed → ScoringParams 的 5 m/s）
  return cc;
}

InjectRequest roleRequest(const std::string & role, std::uint8_t index, std::uint8_t count, std::uint8_t platform,
  Vec3 position, double duration)
{
  InjectRequest req;
  req.incident_id = "coop-1";
  req.role = role;
  req.revision = 1;
  req.task.platform = platform;
  req.task.required_caps = platform == 1 ?
    capBit(Capability::MOBILITY_AIR) | capBit(Capability::RGB_CAMERA) : capBit(Capability::MOBILITY_GROUND);
  req.task.priority = 0;   // half_life 60 s
  req.task.position = position;
  req.task.value = 100.0;
  req.task.duration_sec = duration;
  req.task.ext.role_count = count;
  req.task.ext.role_index = index;
  return req;
}

struct Pair
{
  Swarm swarm;
  TaskId air{0};
  TaskId ground{0};
};

// UAV 在 (0,0,5)，空中角色在 (100,0,5)、執行 30 s → 25 s 到、55 s 完成；狗在 (90,0,0)，地面角色在 (100,0,0)
Pair makePair(bool coop = true, bool with_uav = true, const std::string & spec = std::string())
{
  Pair g{Swarm(0.0, 1, coopConfig(coop)), 0, 0};
  g.swarm.addGateway(kGateway);
  if (with_uav) {
    g.swarm.add(1, {0, 0, 5});
  }
  g.swarm.add(50, {90, 0, 0}, 100.0, AgentType::UGV);
  g.swarm.run(2.0);
  Gateway gw(g.swarm.node(kGateway));
  InjectRequest air = roleRequest("aerial_overwatch", 0, 2, 1, {100.0, 0.0, 5.0}, 30.0);
  air.task.ext.optional = !with_uav;
  air.task.ext.skip_satisfies = !with_uav;
  InjectRequest ground = roleRequest("front", 1, 2, 2, {100.0, 0.0, 0.0}, 10.0);
  ground.task.ext.precedence = 0b01;
  ground.spec = spec;
  const InjectResult a = gw.inject(air, g.swarm.now());
  const InjectResult b = gw.inject(ground, g.swarm.now());
  EXPECT_EQ(a.status, InjectStatus::CREATED) << a.reason;
  EXPECT_EQ(b.status, InjectStatus::CREATED) << b.reason;
  g.air = a.task_id;
  g.ground = b.task_id;
  g.swarm.run(3.0);
  return g;
}
}  // namespace

TEST(CoopBidding, DependentRoleWaitsForPredecessorEta)
{
  // 狗 2.5 s 就到地面角色，但要等 UAV 完成空中角色（25 s 到 + 30 s 執行）：預估在現在 + 55 s 左右（UAV 在模擬裡不動，
  // 預估每秒往後 1 s，變動超過 coop_eta_tolerance 2 s 才更新），出價照「55 s 才開始」打折
  Pair g = makePair();
  CbbaComm & d = g.swarm.node(50);
  ASSERT_EQ(d.agent().winner(g.air), 1);
  ASSERT_EQ(d.agent().winner(g.ground), 50);
  const Task & t = d.agent().tasks().at(g.ground);
  EXPECT_NEAR(t.coop.ready - g.swarm.now(), 55.0, 3.5);
  EXPECT_EQ(t.coop.sync_last, 0.0);
  EXPECT_GT(d.stats().coop_updates, 0);
  // 100 × 2^(−55/60) ≈ 53；不看協同時 100 × 2^(−2.5/60) ≈ 97
  EXPECT_LT(d.agent().score(g.ground), 60.0);
  Pair off = makePair(false);
  EXPECT_EQ(off.swarm.node(50).agent().tasks().at(off.ground).coop.ready, 0.0);
  EXPECT_GT(off.swarm.node(50).agent().score(off.ground), 90.0);
  // UAV、閘道不估（閘道不出價；UAV 的空中角色沒有 precedence）
  EXPECT_EQ(g.swarm.node(1).agent().tasks().at(g.air).coop.ready, 0.0);
  EXPECT_EQ(g.swarm.node(kGateway).agent().tasks().at(g.ground).coop.ready, 0.0);
}

TEST(CoopBidding, SpecWaitForPicksTheMilestone)
{
  // spec 的 wait_for 寫 target_localized（節點層的 resolver 解析）：只等 UAV 到達（25 s），不等它做完
  Pair g = makePair(true, true, "{\"steps\":[{\"action\":\"go_to\"}]}");
  CbbaComm & d = g.swarm.node(50);
  ASSERT_TRUE(d.taskSpec(g.ground).has_value());
  d.setWaitResolver([](const Task &, const std::string & spec) {
      EXPECT_NE(spec.find("go_to"), std::string::npos);
      return std::vector<WaitFor>{WaitFor{0, Milestone::TARGET_LOCALIZED}};
    });
  g.swarm.run(2.0);
  EXPECT_NEAR(d.agent().tasks().at(g.ground).coop.ready - g.swarm.now(), 25.0, 3.5);
}

TEST(CoopBidding, NoPredecessorWinnerAddsNoWait)
{
  // 沒有 UAV（空中角色 optional、沒人得標）：不知道什麼時候 → 不加限制，狗照到達出價
  Pair g = makePair(true, false);
  CbbaComm & d = g.swarm.node(50);
  EXPECT_EQ(d.agent().winner(g.air), kNoAgent);
  EXPECT_EQ(d.agent().tasks().at(g.ground).coop.ready, 0.0);
  EXPECT_GT(d.agent().score(g.ground), 90.0);
}

TEST(CoopBidding, PredecessorDoneClearsTheWait)
{
  // 空中角色完成（TASK_CLOSE 完成 → gate OPEN）：預估清掉，出價回到照到達算
  Pair g = makePair();
  CbbaComm & d = g.swarm.node(50);
  ASSERT_GT(d.agent().tasks().at(g.ground).coop.ready, 0.0);
  ASSERT_EQ(assignAndReport(g.swarm.node(1), g.air, true, g.swarm.now()), ReportCheck::ACCEPTED);
  g.swarm.run(4.0);
  ASSERT_FALSE(d.taskOpen(g.air));
  EXPECT_EQ(d.agent().tasks().at(g.ground).coop.ready, 0.0);
  EXPECT_GT(d.agent().score(g.ground), 90.0);
}

TEST(CoopBidding, SyncRoleSeesTeammateArrival)
{
  // 兩個地面角色會合（sync_window 30 s、要不同的狗）：狗 50 近（2 m）、狗 51 遠（70 m → 17.5 s）。
  // 各自的任務看到隊友的預估到達；近的那隻出價照「等到隊友到」打折
  Swarm swarm(0.0, 1, coopConfig());
  swarm.addGateway(kGateway);
  swarm.add(50, {10, 0, 0}, 100.0, AgentType::UGV);
  swarm.add(51, {84, 0, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  Gateway gw(swarm.node(kGateway));
  std::vector<TaskId> roles;
  for (std::uint8_t k = 0; k < 2; ++k) {
    InjectRequest req = roleRequest(k == 0 ? "front" : "rear", k, 2, 2, {12.0 + 2.0 * k, 0.0, 0.0}, 10.0);
    req.task.ext.sync_window_s = 30;
    req.task.ext.distinct_agent_set = 0b11;
    const InjectResult r = gw.inject(req, swarm.now());
    ASSERT_EQ(r.status, InjectStatus::CREATED) << r.reason;
    roles.push_back(r.task_id);
  }
  swarm.run(4.0);
  CbbaComm & near = swarm.node(50);
  const AgentId w0 = near.agent().winner(roles[0]);
  const AgentId w1 = near.agent().winner(roles[1]);
  ASSERT_NE(w0, kNoAgent);
  ASSERT_NE(w1, kNoAgent);
  ASSERT_NE(w0, w1);
  const TaskId mine = w0 == 50 ? roles[0] : roles[1];
  const Task & t = near.agent().tasks().at(mine);
  EXPECT_NEAR(t.coop.sync_last - swarm.now(), 70.0 / 4.0, 3.0);
  EXPECT_EQ(t.coop.ready, 0.0);
  EXPECT_LT(near.agent().score(mine), 100.0 * std::exp2(-15.0 / 60.0));
}
