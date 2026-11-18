// 任務欄位補齊 06（2026-11-18，doc/protocol.md 第九節）：時間的合約——絕對時刻（系統時鐘、chrony 對時）、
// ms 低 32 位元的循環、release < 硬期限 ≤ expires 的先後、expires 一律是硬限制、時鐘偏差的偵測與影響、亂序到達。
// PDF 的四個驗收項目已經有測試：
//   1 release 前不開始執行      Release.DispatchedJustInTime（test_gateway）、Scoring.ReleaseMakesItWait
//   2 趕不上硬期限不出價        Scoring.HardDeadlineIsInfeasibleSoftIsNot、Scoring.InsertionMustNotPushHardDeadlineTaskLate
//   3 expires 到期自動關閉      Expiry.CreatorClosesExpiredTask、Expiry.WithoutCreatorStopsLocallyThenClosesOnReturn
//   4 half-life 越短遠處越低    Scoring.HalfLifeWidensGapBetweenNearAndFar、Scoring.ShorterHalfLifeNeverRaisesLateReward
// 這裡補合約新加的部分
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/gateway.hpp"
#include "cbba_core/scoring.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
constexpr AgentId kGateway = 200;

Task groundTaskAt(double x, double duration)
{
  Task t;
  t.id = makeTaskId(9, 1);
  t.type = TaskType::NONE;
  t.platform = 2;
  t.required_caps = capBit(Capability::MOBILITY_GROUND);
  t.priority = 1;
  t.position = {x, 0.0, 0.0};
  t.value = 100.0;
  t.duration_sec = duration;
  t.explicit_deadline = false;
  t.deadline_sec = 300.0;
  return t;
}

AgentState dogAt(double x)
{
  AgentState a;
  a.id = 50;
  a.type = AgentType::UGV;
  a.position = {x, 0.0, 0.0};
  a.home = a.position;
  a.energy_per_meter = 0.05;
  a.hover_energy_per_sec = 0.05;
  return a;
}

InjectRequest request(const std::string & incident)
{
  InjectRequest req;
  req.incident_id = incident;
  req.revision = 1;
  req.task.platform = 2;
  req.task.required_caps = capBit(Capability::MOBILITY_GROUND);
  req.task.priority = 1;
  req.task.position = {12.0, 0.0, 0.0};
  req.task.value = 100.0;
  req.task.duration_sec = 5.0;
  return req;
}

Swarm makeSwarm()
{
  CommConfig cc;
  cc.epoch = 3;
  Swarm swarm(0.0, 1, cc);
  swarm.addGateway(kGateway);
  swarm.add(1, {0, 0, 5});
  swarm.add(50, {5, 0, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  return swarm;
}
}  // namespace

// ===========================================================================
// 先後關係
// ===========================================================================
TEST(TimeContract, ExpiresIsAHardLimitOnCompletion)
{
  // expires 到了任務失效（執行中的也撤銷）：完成時刻晚於 expires 的不可行，不看 deadline_hard。規劃速度 5 × 0.8 = 4 m/s
  const double now = 1000.0;
  ScoringParams p;
  Task t = groundTaskAt(20.0, 10.0);   // 到達 5 s、完成 15 s
  t.ext.expires = now + 14.0;
  EXPECT_FALSE(bestInsertion(dogAt(0.0), {}, t, now, p).feasible);
  t.ext.expires = now + 16.0;
  EXPECT_TRUE(bestInsertion(dogAt(0.0), {}, t, now, p).feasible);
  // release 的等待也算：release 在 10 s → 完成 20 s，16 s 失效來不及
  t.ext.release = now + 10.0;
  EXPECT_FALSE(bestInsertion(dogAt(0.0), {}, t, now, p).feasible);
  // 執行中的（exempt）不檢查：失效時由 expireTasks 撤銷，不是在出價時提早放掉
  t.ext.release = 0.0;
  t.ext.expires = now + 1.0;
  EXPECT_FALSE(evaluatePath(dogAt(0.0), {t}, now, p).feasible);
  EXPECT_TRUE(evaluatePath(dogAt(0.0), {t}, now, p, -1.0, t.id).feasible);
}

TEST(TimeContract, GatewayChecksReleaseDeadlineExpiresOrder)
{
  // release < 硬期限 ≤ expires；軟期限不檢查先後（只影響排隊）
  Swarm swarm = makeSwarm();
  Gateway gw(swarm.node(kGateway));
  const auto status = [&](InjectRequest req) {return gw.inject(req, swarm.now()).status;};

  InjectRequest hard = request("t-hard-1");
  hard.task.ext.deadline_hard = true;
  hard.deadline_sec = 30.0;
  hard.release_sec = 30.0;   // release 之後才能開始：永遠來不及
  EXPECT_EQ(status(hard), InjectStatus::REJECTED);
  hard.incident_id = "t-hard-2";
  hard.release_sec = 5.0;
  hard.expires_sec = 20.0;   // 硬期限晚於 expires
  EXPECT_EQ(status(hard), InjectStatus::REJECTED);
  hard.incident_id = "t-hard-3";
  hard.expires_sec = 30.0;   // 硬期限 = expires：可以
  EXPECT_EQ(status(hard), InjectStatus::CREATED);

  InjectRequest soft = request("t-soft");
  soft.deadline_sec = 10.0;
  soft.release_sec = 20.0;   // 軟期限早於 release：照收（軟期限過了也照做）
  soft.expires_sec = 60.0;
  EXPECT_EQ(status(soft), InjectStatus::CREATED);
}

// ===========================================================================
// 絕對時刻與循環
// ===========================================================================
TEST(TimeContract, MillisecondFieldsWrapAround)
{
  // ms 的低 32 位元約 49.7 天循環：跨過 2^32 ms 的時刻照樣還原成離 now 最近的那一個（±24.8 天）
  const double wrap = 4294967.296;            // 2^32 ms（秒）
  const double now = 1.7e9 - std::fmod(1.7e9, wrap) + wrap - 10.0;   // 再 10 s 就循環
  const double later = now + 60.0;            // 循環之後
  const std::uint32_t ms = wire::toDeadlineMs(later);
  EXPECT_LT(ms, 60000u);                      // 已經回到小的數值
  EXPECT_NEAR(wire::fromDeadlineMs(ms, now), later, 1e-3);
  EXPECT_NEAR(wire::fromDeadlineMs(wire::toDeadlineMs(now - 30.0), later), now - 30.0, 1e-3);
  // 時間戳 s（不晚於 now）
  EXPECT_NEAR(fromStampMs(toStampMs(now - 1.0), later), now - 1.0, 1e-3);
  // 對方時鐘稍快（比 now 晚 0.2 s 的時間戳）：當成現在，不會變成 49.7 天前
  EXPECT_NEAR(fromStampMs(toStampMs(later + 0.2), later), later, 1e-3);
}

TEST(TimeContract, AbsoluteTimesSurviveLateDelivery)
{
  // 收到的 release、expires、建立時刻是建立者的絕對時刻：晚 3 s 收到（補發）也一樣，不會往後推
  Swarm swarm = makeSwarm();
  swarm.setOnline(50, false);
  Gateway gw(swarm.node(kGateway));
  InjectRequest req = request("t-abs");
  req.release_sec = 20.0;
  req.expires_sec = 60.0;
  const double t0 = swarm.now();
  const InjectResult r = gw.inject(req, t0);
  ASSERT_EQ(r.status, InjectStatus::CREATED);
  swarm.run(3.0);
  swarm.setOnline(50, true);
  swarm.run(8.0);   // 摘要比對 → 補發
  for (AgentId a : {kGateway, AgentId{1}, AgentId{50}}) {
    ASSERT_TRUE(swarm.node(a).knows(r.task_id)) << a;
    const Task & t = swarm.node(a).agent().tasks().at(r.task_id);
    EXPECT_NEAR(t.ext.release, t0 + 20.0, 0.01) << a;
    EXPECT_NEAR(t.ext.expires, t0 + 60.0, 0.01) << a;
    EXPECT_NEAR(t.created, t0, 0.01) << a;
  }
}

// ===========================================================================
// 時鐘偏差
// ===========================================================================
TEST(TimeContract, ClockSkewIsDetectedIncludingGateway)
{
  // 閘道（不出價、不送別台的 s）也帶自己的時刻：它的時鐘快 0.8 s → 機器人告警一次；0.1 s 的不告警。
  // 降到告警值的一半以下才解除，之後再超過再告警
  Swarm swarm = makeSwarm();
  swarm.setClockOffset(kGateway, 0.8);
  swarm.setClockOffset(1, 0.1);
  swarm.run(2.0);
  CbbaComm & dog = swarm.node(50);
  ASSERT_TRUE(dog.clockSkew(kGateway).has_value());
  EXPECT_NEAR(*dog.clockSkew(kGateway), 0.8, 0.05);   // 含網路延遲（5～20 ms）
  EXPECT_NEAR(*dog.clockSkew(1), 0.1, 0.05);
  auto warnings = dog.takeSkewWarnings();
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_EQ(warnings[0].first, kGateway);
  EXPECT_TRUE(dog.takeSkewWarnings().empty());   // 取走了；還超過的不重複
  swarm.run(2.0);
  EXPECT_TRUE(dog.takeSkewWarnings().empty());
  EXPECT_EQ(dog.stats().clock_skew_warnings, 1);

  swarm.setClockOffset(kGateway, 0.4);   // 0.25 ～ 0.5 之間：不告警、也不解除
  swarm.run(2.0);
  swarm.setClockOffset(kGateway, 0.9);
  swarm.run(2.0);
  EXPECT_TRUE(dog.takeSkewWarnings().empty());
  swarm.setClockOffset(kGateway, 0.0);   // 解除
  swarm.run(2.0);
  swarm.setClockOffset(kGateway, -0.7);  // 慢也算
  swarm.run(2.0);
  warnings = dog.takeSkewWarnings();
  ASSERT_EQ(warnings.size(), 1u);
  EXPECT_NEAR(warnings[0].second, -0.7, 0.05);
}

TEST(TimeContract, SkewShiftsAbsoluteTimesByTheOffset)
{
  // 閘道的時鐘快 0.3 s：絕對時刻照閘道的時鐘定，機器人在真實時間看到的 expires 晚 0.3 s（偏差直接平移，不會累積；
  // 閘道自己在真實時間早 0.3 s 判定過期、送 TASK_CLOSE）。chrony 對時（毫秒級）時可以忽略；告警值 0.5 s。
  // 閘道注入後離線：只看機器人自己的判定（不被閘道的 TASK_CLOSE 先關掉）
  Swarm swarm = makeSwarm();
  swarm.setClockOffset(kGateway, 0.3);
  swarm.run(1.0);
  Gateway gw(swarm.node(kGateway));
  InjectRequest req = request("t-skew");
  req.expires_sec = 10.0;
  req.task.duration_sec = 0.0;
  const double t0 = swarm.now();   // 模擬時間（真實時間）
  const InjectResult r = gw.inject(req, swarm.clockOf(kGateway));
  ASSERT_EQ(r.status, InjectStatus::CREATED);
  swarm.run(1.0);
  swarm.setOnline(kGateway, false);
  const Task & seen = swarm.node(50).agent().tasks().at(r.task_id);
  EXPECT_NEAR(seen.ext.expires, t0 + 10.3, 0.01);
  // 真實時間 t0 + 10.3 之前，機器人自己的時鐘還沒到
  swarm.run(t0 + 10.2 - swarm.now());
  EXPECT_FALSE(swarm.node(50).expired(r.task_id, swarm.clockOf(50)));
  swarm.run(0.2);
  EXPECT_TRUE(swarm.node(50).expired(r.task_id, swarm.clockOf(50)));
  EXPECT_TRUE(swarm.node(50).taskOpen(r.task_id));   // 建立者離線：本機停止出價，任務等閘道回來再關
}

TEST(TimeContract, SmallSkewKeepsConsensusWithoutFalseLoss)
{
  // 各台時鐘差 ±0.3 s（< lost_timeout 1.5 s）：轉送的時間戳 s 被平移，但不會誤判失聯、分配照樣收斂
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {20, 0, 5});
  swarm.add(3, {40, 0, 5});
  swarm.cut(1, 3);   // 1、3 互相聽不到：它們之間的 s 只能經過 2 轉送
  swarm.setClockOffset(1, 0.3);
  swarm.setClockOffset(3, -0.3);
  swarm.run(2.0);
  for (std::uint16_t k = 1; k <= 3; ++k) {
    swarm.node(2).addLocalTask(newTask(2, k, 15.0 * k, 5.0, swarm.clockOf(2)), swarm.clockOf(2));
  }
  swarm.run(10.0);
  EXPECT_TRUE(swarm.consistent());
  for (AgentId a : {AgentId{1}, AgentId{2}, AgentId{3}}) {
    EXPECT_EQ(swarm.node(a).stats().takeovers, 0) << a;
    EXPECT_TRUE(swarm.node(a).lockedByLostAgents(swarm.clockOf(a)).empty()) << a;
  }
  // 1 和 3 的分配互相看得到（沒有被當成失聯而釋放）
  int held = 0;
  for (std::uint16_t k = 1; k <= 3; ++k) {
    held += swarm.node(2).agent().winner(makeTaskId(2, k)) != kNoAgent ? 1 : 0;
  }
  EXPECT_EQ(held, 3);
}

// ===========================================================================
// 亂序、重放
// ===========================================================================
TEST(TimeContract, OldRevisionArrivingLateIsIgnored)
{
  // 改版之後，舊版的任務定義（錄下來的、晚到的）再送來：照樣是新版、位置不變回去
  Swarm swarm = makeSwarm();
  swarm.record(kGateway);
  Gateway gw(swarm.node(kGateway));
  InjectRequest req = request("t-order");
  const InjectResult r = gw.inject(req, swarm.now());
  ASSERT_EQ(r.status, InjectStatus::CREATED);
  swarm.run(2.0);
  std::vector<Bytes> old_announces;
  for (const Bytes & b : swarm.recorded()) {
    if (wire::parseHeader(b)->type == PacketType::TASK_ANNOUNCE) {
      old_announces.push_back(b);
    }
  }
  ASSERT_FALSE(old_announces.empty());
  req.revision = 2;
  req.task.position = {14.0, 0.0, 0.0};
  ASSERT_EQ(gw.inject(req, swarm.now()).status, InjectStatus::REVISED);
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(50).agent().tasks().at(r.task_id).revision, 2);
  for (const Bytes & b : old_announces) {
    swarm.node(50).receive(b, swarm.now());
  }
  swarm.run(1.0);
  const Task & t = swarm.node(50).agent().tasks().at(r.task_id);
  EXPECT_EQ(t.revision, 2);
  EXPECT_NEAR(t.position.x, 14.0, 1e-3);
}

TEST(TimeContract, OldCbbaStateArrivingLateIsDropped)
{
  // 舊的 CBBA_STATE（snapshot 比收過的舊、或滑動視窗外）晚到：丟掉，不改分配
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {30, 0, 5});
  swarm.run(2.0);
  swarm.record(1);
  swarm.node(1).addLocalTask(newTask(1, 1, 2, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(2).agent().winner(makeTaskId(1, 1)), 1);
  std::vector<Bytes> states;
  for (const Bytes & b : swarm.recorded()) {
    if (wire::parseHeader(b)->type == PacketType::CBBA_STATE) {
      states.push_back(b);
    }
  }
  ASSERT_GE(states.size(), 2u);
  swarm.run(2.0);
  const int stale = swarm.node(2).stats().stale;
  swarm.node(2).receive(states.front(), swarm.now());   // 最舊的那一則（任務還沒出現時的）
  EXPECT_GT(swarm.node(2).stats().stale, stale);
  EXPECT_EQ(swarm.node(2).agent().winner(makeTaskId(1, 1)), 1);
}
