// CB-4 第一批（2026-11-16，doc/cb4_design.md）：角色進度的合併（progress_seq、指派版本、事件結果型跟著 revision）、
// gate 的判斷，以及在模擬網路上：UAV 回報 target_localized → 狗的 gate 解除；非執行者的回報被拒；新鄰居從快照補到；
// completed、略過（skip_satisfies）、取消；BLOCKED 不計入執行逾時
#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <utility>

#include "cbba_core/assignment_manager.hpp"
#include "cbba_core/capabilities.hpp"
#include "cbba_core/gateway.hpp"
#include "cbba_core/group_progress.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
wire::TaskProgressBody report(std::uint64_t version, std::uint16_t seq, MilestoneMask m, std::uint16_t revision = 1,
  AgentId executor = 1)
{
  wire::TaskProgressBody b;
  b.relay = wire::RelayHeader{executor, 0xAB, seq};
  b.task_id = 7;
  b.revision = revision;
  b.assign_version = version;
  b.executor = executor;
  b.progress_seq = seq;
  b.phase = static_cast<std::uint8_t>(ProgressPhase::EXECUTING);
  b.milestones = m;
  return b;
}

constexpr MilestoneMask kArrived = milestoneBit(Milestone::ARRIVED);
constexpr MilestoneMask kLocalized = milestoneBit(Milestone::TARGET_LOCALIZED);
}  // namespace

// ===========================================================================
// 合併（不經網路）
// ===========================================================================
TEST(ProgressMerge, SeqAndVersion)
{
  // 同一個指派版本：progress_seq 較大的更新、里程碑 OR；較小的不影響
  ProgressRecord rec;
  EXPECT_TRUE(mergeProgress(rec, false, report(2, 1, kArrived), 0.0));
  EXPECT_FALSE(mergeProgress(rec, true, report(2, 1, kArrived), 0.1));   // 重複
  EXPECT_TRUE(mergeProgress(rec, true, report(2, 3, milestoneBit(Milestone::STARTED)), 0.2));
  EXPECT_EQ(rec.executor_bits, kArrived | milestoneBit(Milestone::STARTED));
  EXPECT_FALSE(mergeProgress(rec, true, report(2, 2, milestoneBit(Milestone::TARGET_LOCKED)), 0.3));   // 舊的 seq
  EXPECT_EQ(rec.progress_seq, 3);
  // 新的指派版本（換人）：執行者相關的重來
  EXPECT_TRUE(mergeProgress(rec, true, report(3, 1, milestoneBit(Milestone::EN_ROUTE), 1, 2), 0.4));
  EXPECT_EQ(rec.executor, 2);
  EXPECT_EQ(rec.executor_bits, milestoneBit(Milestone::EN_ROUTE));
  // 舊版本晚到的回報：不影響執行者相關的部分
  EXPECT_FALSE(mergeProgress(rec, true, report(2, 9, kArrived), 0.5));
  EXPECT_EQ(rec.assign_version, 3u);
}

TEST(ProgressMerge, EventMilestonesFollowRevision)
{
  // 事件結果型：換人後保留（同一個 revision）；revision 變新就清掉；舊的執行者晚到的定位也算（同 revision）
  ProgressRecord rec;
  mergeProgress(rec, false, report(2, 1, kArrived | kLocalized), 0.0);
  mergeProgress(rec, true, report(3, 1, 0, 1, 2), 0.1);   // 換人
  EXPECT_EQ(rec.executor_bits, 0);
  EXPECT_EQ(rec.event_bits, kLocalized);
  // gate 用：新的指派版本 3 → arrived 不算、target_localized 算
  EXPECT_EQ(effectiveMilestones(&rec, 3, 1, true, false, false) & (kArrived | kLocalized), kLocalized);
  // 目前的 revision 已經是 2（incident 改版）→ 舊的定位不算；過期 → 不算
  EXPECT_EQ(effectiveMilestones(&rec, 3, 2, true, false, false) & kLocalized, 0);
  EXPECT_EQ(effectiveMilestones(&rec, 3, 1, true, true, false) & kLocalized, 0);
  // revision 2 的回報（沒有定位）→ 事件結果清掉
  mergeProgress(rec, true, report(3, 2, 0, 2, 2), 0.2);
  EXPECT_EQ(rec.event_bits, 0);
  EXPECT_EQ(rec.revision, 2);
  // 舊 revision 晚到的定位不算
  EXPECT_FALSE(mergeProgress(rec, true, report(2, 5, kLocalized, 1), 0.3));
  EXPECT_EQ(rec.event_bits, 0);
  // completed：全部
  EXPECT_EQ(effectiveMilestones(nullptr, 0, 0, false, false, true), kAllMilestones);
  // 指派紀錄比進度新（新的執行者還沒回報）：執行者相關的不算，accepted 照指派
  ProgressRecord old;
  mergeProgress(old, false, report(2, 1, kArrived), 0.0);
  EXPECT_EQ(effectiveMilestones(&old, 4, 1, true, false, false), milestoneBit(Milestone::ACCEPTED));
}

// ===========================================================================
// 模擬網路：閘道＋UAV（1）＋狗（50）；role 0 = 空中（optional），role 1 = 地面（等 role 0）
// ===========================================================================
namespace
{
constexpr AgentId kGateway = 200;

struct Group
{
  Swarm swarm;
  TaskId air{0};
  TaskId ground{0};
};

Group makeGroup(bool optional_air, bool skip_satisfies, bool with_uav = true)
{
  CommConfig cc;
  cc.epoch = 3;
  Group g{Swarm(0.0, 1, cc), 0, 0};
  g.swarm.addGateway(kGateway);
  if (with_uav) {
    g.swarm.add(1, {0, 0, 5});
  }
  g.swarm.add(50, {5, 0, 0}, 100.0, AgentType::UGV);
  g.swarm.run(2.0);
  Gateway gw(g.swarm.node(kGateway));
  InjectRequest air;
  air.incident_id = "intrusion-1";
  air.role = "aerial_overwatch";
  air.revision = 1;
  air.task.platform = 1;
  air.task.required_caps = capBit(Capability::MOBILITY_AIR) | capBit(Capability::RGB_CAMERA);
  air.task.priority = 0;
  air.task.position = {12.0, 0.0, 5.0};
  air.task.value = 100.0;
  air.task.duration_sec = 30.0;
  air.task.ext.role_count = 2;
  air.task.ext.role_index = 0;
  air.task.ext.optional = optional_air;
  air.task.ext.skip_satisfies = skip_satisfies;
  InjectRequest ground = air;
  ground.role = "front";
  ground.task.platform = 2;
  ground.task.required_caps = capBit(Capability::MOBILITY_GROUND);
  ground.task.position = {12.0, 0.0, 0.0};
  ground.task.ext.role_index = 1;
  ground.task.ext.optional = false;
  ground.task.ext.skip_satisfies = false;
  ground.task.ext.precedence = 0b01;   // 等 role 0
  const InjectResult a = gw.inject(air, g.swarm.now());
  const InjectResult b = gw.inject(ground, g.swarm.now());
  EXPECT_EQ(a.status, InjectStatus::CREATED) << a.reason;
  EXPECT_EQ(b.status, InjectStatus::CREATED) << b.reason;
  g.air = a.task_id;
  g.ground = b.task_id;
  g.swarm.run(2.0);
  return g;
}

// 正式指派（像 AssignmentManager 做的：開始＋接受成執行中）
std::uint64_t takeOn(CbbaComm & n, TaskId task)
{
  const std::uint64_t v = n.beginAssignment(task);
  EXPECT_TRUE(n.confirmAssignment(task, v, AssignState::ACTIVE));
  return v;
}

const WaitFor kWaitLocalized{0, Milestone::TARGET_LOCALIZED};
}  // namespace

TEST(GroupProgress, LocalizedOpensGroundGate)
{
  // 主要驗收：UAV 回報 target_localized → 狗的 gate（等 role 0 的 target_localized）從 WAITING 變 OPEN
  Group g = makeGroup(true, true);
  CbbaComm & uav = g.swarm.node(1);
  CbbaComm & dog = g.swarm.node(50);
  ASSERT_EQ(uav.agent().winner(g.air), 1);
  ASSERT_EQ(dog.roleTask(uav.agent().tasks().at(g.air).ext.group_id, 0), std::optional<TaskId>(g.air));
  takeOn(uav, g.air);
  g.swarm.run(1.0);
  EXPECT_EQ(dog.gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::WAITING);
  // accepted 不用回報：正式指派就有
  EXPECT_NE(dog.milestones(g.air, g.swarm.now()) & milestoneBit(Milestone::ACCEPTED), 0);

  ASSERT_EQ(uav.reportProgress(g.air, ProgressPhase::EN_ROUTE, 0, milestoneBit(Milestone::EN_ROUTE),
    g.swarm.now()), ReportCheck::ACCEPTED);
  g.swarm.run(0.5);
  EXPECT_EQ(dog.gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::WAITING);
  ASSERT_NE(dog.progress(g.air), nullptr);
  EXPECT_EQ(dog.progress(g.air)->phase, ProgressPhase::EN_ROUTE);

  ASSERT_EQ(uav.reportProgress(g.air, ProgressPhase::EXECUTING, 1, kLocalized, g.swarm.now()), ReportCheck::ACCEPTED);
  g.swarm.run(0.2);
  EXPECT_EQ(dog.gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::OPEN);
  EXPECT_EQ(g.swarm.node(kGateway).gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::OPEN);
  // 有變化時送 3 份（馬上、0.2 s、0.6 s）；之後一樣的回報不再送，到 2 s 才送定期快照
  g.swarm.run(0.6);   // 回報後 0.8 s：錯開的 3 份都送完了，還沒到 2 s
  // EN_ROUTE 送了 2 份（0、0.2 s；0.6 s 那份被 0.5 s 時新的回報取代）＋ EXECUTING 3 份
  EXPECT_EQ(uav.stats().progress_sent, 5);
  const int sent = uav.stats().progress_sent;
  uav.reportProgress(g.air, ProgressPhase::EXECUTING, 1, kLocalized, g.swarm.now());
  g.swarm.run(1.0);   // 回報後 1.8 s
  EXPECT_EQ(uav.stats().progress_sent, sent);
  EXPECT_EQ(uav.stats().progress_reports, 2);
  // 執行中每 2 s 重送最新的快照
  g.swarm.run(2.5);
  EXPECT_GE(uav.stats().progress_sent, sent + 1);
}

TEST(GroupProgress, OnlyExecutorMayReport)
{
  // 不是自己的正式指派：STALE；別台假冒（同一個指派版本、執行者不同）的封包被拒
  Group g = makeGroup(true, true);
  CbbaComm & uav = g.swarm.node(1);
  CbbaComm & dog = g.swarm.node(50);
  EXPECT_EQ(uav.reportProgress(g.air, ProgressPhase::EXECUTING, 1, kLocalized, g.swarm.now()), ReportCheck::STALE);
  const std::uint64_t v = takeOn(uav, g.air);
  g.swarm.run(1.0);
  wire::TaskProgressBody fake = report(v, 1, kLocalized, uav.agent().tasks().at(g.air).revision, 50);
  fake.task_id = g.air;
  fake.relay = wire::RelayHeader{50, dog.sessionId(), 999};
  wire::Header h;
  h.sender_id = 50;
  h.session_id = dog.sessionId();
  h.sequence = 999;
  const int before = g.swarm.node(kGateway).stats().progress_rejected;
  g.swarm.node(kGateway).receive(wire::encodeTaskProgress(h, fake), g.swarm.now());
  EXPECT_EQ(g.swarm.node(kGateway).stats().progress_rejected, before + 1);
  EXPECT_EQ(g.swarm.node(kGateway).gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::WAITING);
}

TEST(GroupProgress, NewNeighborGetsSnapshot)
{
  // 狗在 UAV 回報之後才上線（重開機）：定期重送之外，發現新鄰居時用 SYNC_BATCH 補進度
  Group g = makeGroup(true, true);
  CbbaComm & uav = g.swarm.node(1);
  takeOn(uav, g.air);
  g.swarm.run(0.5);
  uav.reportProgress(g.air, ProgressPhase::EXECUTING, 1, kLocalized, g.swarm.now());
  g.swarm.run(0.2);
  // UAV 不再送進度（只看閘道的快照）：把 UAV → 狗的 TASK_PROGRESS 擋住
  g.swarm.block(1, 50, PacketType::TASK_PROGRESS);
  g.swarm.block(kGateway, 50, PacketType::TASK_PROGRESS);
  CbbaComm & dog = g.swarm.restart(50);
  g.swarm.run(4.0);
  ASSERT_TRUE(dog.knows(g.ground));
  EXPECT_GT(g.swarm.node(kGateway).stats().progress_snapshots + uav.stats().progress_snapshots, 0);
  EXPECT_EQ(dog.gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::OPEN);
}

TEST(GroupProgress, LostFirstCopyStillOpens)
{
  // 掉包：第一份給狗的（UAV 直送、閘道轉送）都掉了，錯開的重送補上，5 s 內狗的 gate 打開
  Group g = makeGroup(true, true);
  CbbaComm & uav = g.swarm.node(1);
  takeOn(uav, g.air);
  g.swarm.run(1.0);
  uav.reportProgress(g.air, ProgressPhase::EXECUTING, 1, kLocalized, g.swarm.now());
  g.swarm.dropNext(1, 50, PacketType::TASK_PROGRESS);
  g.swarm.dropNext(kGateway, 50, PacketType::TASK_PROGRESS);
  g.swarm.run(5.0);
  EXPECT_EQ(g.swarm.node(50).gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::OPEN);
}

TEST(GroupProgress, CompletedSkippedCancelled)
{
  // 前面的角色：完成 → OPEN（completed = 全部里程碑）；略過＋skip_satisfies → OPEN；略過但 false、取消 → UNSATISFIABLE
  {
    Group g = makeGroup(true, true);
    CbbaComm & uav = g.swarm.node(1);
    const std::uint64_t v = takeOn(uav, g.air);
    g.swarm.run(1.0);
    ASSERT_EQ(uav.reportResult(g.air, true, v, g.swarm.now()), ReportCheck::ACCEPTED);
    g.swarm.run(3.0);
    ASSERT_FALSE(g.swarm.node(50).taskOpen(g.air));
    EXPECT_EQ(g.swarm.node(50).gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::OPEN);
    EXPECT_EQ(g.swarm.node(50).milestones(g.air, g.swarm.now()), kAllMilestones);
  }
  for (const bool skip : {true, false}) {
    Group g = makeGroup(true, skip);
    ASSERT_EQ(g.swarm.node(kGateway).dropOptional(g.air, g.swarm.now()), ReportCheck::ACCEPTED);
    g.swarm.run(1.0);
    EXPECT_EQ(g.swarm.node(50).gateState(g.ground, kWaitLocalized, g.swarm.now()),
      skip ? GateState::OPEN : GateState::UNSATISFIABLE) << skip;
  }
  {
    Group g = makeGroup(false, false);
    ASSERT_EQ(g.swarm.node(kGateway).cancelTask(g.air, g.swarm.now()), ReportCheck::ACCEPTED);
    g.swarm.run(1.0);
    EXPECT_EQ(g.swarm.node(50).gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::UNSATISFIABLE);
  }
}

TEST(GroupProgress, HandoverClearsExecutorMilestones)
{
  // UAV 到位（arrived）、定位後撤銷指派（換人）：arrived 不再算，target_localized 保留（同一個 revision）
  Group g = makeGroup(true, true);
  CbbaComm & uav = g.swarm.node(1);
  takeOn(uav, g.air);
  g.swarm.run(0.5);
  uav.reportProgress(g.air, ProgressPhase::EXECUTING, 1, kArrived | kLocalized, g.swarm.now());
  g.swarm.run(0.5);
  const WaitFor wait_arrived{0, Milestone::ARRIVED};
  EXPECT_EQ(g.swarm.node(50).gateState(g.ground, wait_arrived, g.swarm.now()), GateState::OPEN);
  ASSERT_TRUE(uav.endAssignment(g.air));
  g.swarm.run(1.0);
  EXPECT_EQ(g.swarm.node(50).gateState(g.ground, wait_arrived, g.swarm.now()), GateState::WAITING);
  EXPECT_EQ(g.swarm.node(50).gateState(g.ground, kWaitLocalized, g.swarm.now()), GateState::OPEN);
}

TEST(GroupProgress, BlockedDoesNotCountAsOverrun)
{
  // 狗執行中、停在 gate（BLOCKED）的時間不計入執行逾時：停 60 s 之後還不算超時；解除後預估完成時刻往後延
  Group g = makeGroup(true, true);
  AssignmentConfig ac;
  ac.require_accept = false;
  ac.assign_hold = 0.0;
  AssignmentManager dog(g.swarm.node(50), ac, [](int, const std::string &) {});
  g.swarm.onTick([&dog](double now) {
      dog.setExec(ExecInput{}, now);
      dog.step(now);
      dog.takeRequests();
      dog.takeAssignedChanged();
    });
  g.swarm.run(2.0);
  ASSERT_TRUE(dog.active());
  dog.setBlocked(g.ground, true, g.swarm.now());
  EXPECT_TRUE(dog.blocked());
  g.swarm.run(60.0);
  EXPECT_TRUE(g.swarm.node(50).agent().execConstraints().queue_behind_active);
  dog.setBlocked(g.ground, false, g.swarm.now());
  g.swarm.run(1.0);
  EXPECT_FALSE(dog.blocked());
  EXPECT_TRUE(g.swarm.node(50).agent().execConstraints().queue_behind_active);
}
