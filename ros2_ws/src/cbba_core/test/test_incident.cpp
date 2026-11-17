// CB-4 第三批（2026-11-16，doc/cb4_design.md）：組層級的 incident state（Proposal → 閘道驗證 → Accepted／Rejected）、
// 事件結果型里程碑跟著 incident revision、連鎖撤銷與 FallbackRequest（group_failure_version）、
// distinct_agent_set（出價的硬性限制＋Group Commit 再驗證）。都在模擬網路上，閘道是 200
#include <gtest/gtest.h>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

#include "cbba_core/assignment_manager.hpp"
#include "cbba_core/capabilities.hpp"
#include "cbba_core/gateway.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
constexpr AgentId kGateway = 200;
constexpr MilestoneMask kLocalized = milestoneBit(Milestone::TARGET_LOCALIZED);
const WaitFor kWaitLocalized{0, Milestone::TARGET_LOCALIZED};

// 角色：平台（1 uav、2 quadruped）、位置、optional、precedence
struct RoleSpec
{
  std::uint8_t platform{2};
  Vec3 position{10.0, 0.0, 0.0};
  bool optional{false};
  std::uint8_t precedence{0};
  std::uint8_t priority{0};
};

InjectRequest roleRequest(const std::string & incident, std::uint8_t index, std::uint8_t count, const RoleSpec & r)
{
  InjectRequest req;
  req.incident_id = incident;
  req.role = "role" + std::to_string(index);
  req.revision = 1;
  req.task.platform = r.platform;
  req.task.required_caps = r.platform == 1 ?
    capBit(Capability::MOBILITY_AIR) | capBit(Capability::RGB_CAMERA) : capBit(Capability::MOBILITY_GROUND);
  req.task.priority = r.priority;
  req.task.position = r.position;
  req.task.value = 100.0;
  req.task.duration_sec = 30.0;
  req.task.ext.role_count = count;
  req.task.ext.role_index = index;
  req.task.ext.optional = r.optional;
  req.task.ext.precedence = r.precedence;
  return req;
}

// 閘道注入一組角色，回傳每個角色的 task_id；all_or_nothing、distinct、fallback_template 整組一樣
std::vector<TaskId> injectGroup(Gateway & gw, Swarm & swarm, const std::string & incident,
  const std::vector<RoleSpec> & roles, bool all_or_nothing, std::uint8_t distinct, const std::string & fallback)
{
  std::vector<TaskId> ids;
  for (std::size_t i = 0; i < roles.size(); ++i) {
    InjectRequest req = roleRequest(incident, static_cast<std::uint8_t>(i), static_cast<std::uint8_t>(roles.size()), roles[i]);
    req.task.ext.all_or_nothing = all_or_nothing;
    req.task.ext.distinct_agent_set = distinct;
    req.fallback_template = fallback;
    const InjectResult r = gw.inject(req, swarm.now());
    EXPECT_EQ(r.status, InjectStatus::CREATED) << r.reason;
    ids.push_back(r.task_id);
  }
  return ids;
}

Swarm makeSwarm(const std::vector<std::pair<AgentId, Vec3>> & uavs, const std::vector<std::pair<AgentId, Vec3>> & dogs,
  double takeover_p0 = 10.0, double takeover_p1 = 15.0)
{
  CommConfig cc;
  cc.epoch = 3;
  cc.takeover_by_priority[0] = takeover_p0;
  cc.takeover_by_priority[1] = takeover_p1;
  Swarm swarm(0.0, 1, cc);
  swarm.addGateway(kGateway);
  for (const auto & [id, pos] : uavs) {
    swarm.add(id, pos);
  }
  for (const auto & [id, pos] : dogs) {
    swarm.add(id, pos, 100.0, AgentType::UGV);
  }
  swarm.run(3.0);   // 開機同步、membership_ready
  return swarm;
}

std::uint64_t takeOn(CbbaComm & n, TaskId task)
{
  const std::uint64_t v = n.beginAssignment(task);
  EXPECT_TRUE(n.confirmAssignment(task, v, AssignState::ACTIVE));
  return v;
}

CbbaComm::IncidentProposal proposal(TaskId task, bool localized)
{
  CbbaComm::IncidentProposal p;
  p.task = task;
  p.position = {14.0, 3.0, 0.0};
  p.radius = 1.5;
  p.track_id = 88;
  p.declares_localized = localized;
  return p;
}

// 一台載具的指派流程（不握手）
struct Runner
{
  AssignmentManager am;
  std::vector<GroupTimeout> timeouts;
  Runner(CbbaComm & comm, double commit_timeout)
  : am(comm, config(commit_timeout), [](int, const std::string &) {}) {}
  static AssignmentConfig config(double commit_timeout)
  {
    AssignmentConfig ac;
    ac.require_accept = false;
    ac.assign_hold = 0.0;
    ac.group_commit_timeout = commit_timeout;
    return ac;
  }
  void tick(double now)
  {
    am.setExec(ExecInput{}, now);
    am.step(now);
    am.takeRequests();
    am.takeAssignedChanged();
    for (const GroupTimeout & t : am.takeGroupTimeouts()) {
      timeouts.push_back(t);
    }
  }
};
}  // namespace

// ===========================================================================
// 組層級的 incident state
// ===========================================================================
TEST(Incident, AcceptedRaisesRevisionAndLocalizesReporter)
{
  // UAV（role 0 的正式執行者）提案新位置＋宣告定位 → 閘道驗證通過、incident revision 1 → 全隊知道；
  // UAV 的 target_localized 記在 revision 1，狗（等 role 0 定位）的 gate OPEN；閘道交給節點發 incident_updates
  Swarm swarm = makeSwarm({{1, {0, 0, 5}}}, {{50, {5, 0, 0}}});
  Gateway gw(swarm.node(kGateway));
  const auto ids = injectGroup(gw, swarm, "inc-a", {{1, {10, 0, 5}}, {2, {10, 0, 0}, false, 0b01}}, false, 0, "");
  swarm.run(2.0);
  CbbaComm & uav = swarm.node(1);
  takeOn(uav, ids[0]);
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(50).gateState(ids[1], kWaitLocalized, swarm.now()), GateState::WAITING);
  ASSERT_EQ(uav.proposeIncident(proposal(ids[0], true), swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(1.0);
  const TaskId group = uav.agent().tasks().at(ids[0]).ext.group_id;
  for (const AgentId a : {AgentId{1}, AgentId{50}, kGateway}) {
    EXPECT_EQ(swarm.node(a).incidentRevision(group), 1) << a;
  }
  ASSERT_NE(swarm.node(50).incident(group), nullptr);
  EXPECT_NEAR(swarm.node(50).incident(group)->position.x, 14.0, 1e-4);
  EXPECT_EQ(swarm.node(50).incident(group)->track_id, 88u);
  EXPECT_EQ(swarm.node(50).incident(group)->localized_by, ids[0]);
  EXPECT_EQ(swarm.node(50).gateState(ids[1], kWaitLocalized, swarm.now()), GateState::OPEN);
  const auto results = uav.takeIncidentResults();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_TRUE(results[0].accepted);
  EXPECT_EQ(results[0].incident_revision, 1);
  EXPECT_EQ(swarm.node(kGateway).takeAcceptedIncidents().size(), 1u);
  EXPECT_EQ(swarm.node(kGateway).stats().incident_accepted, 1);
}

TEST(Incident, UndeclaredUpdateInvalidatesOldLocalization)
{
  // UAV 先回報 target_localized（incident revision 0）→ 狗的 gate OPEN；之後提案新位置但沒宣告定位 →
  // revision 1，舊的定位不算，狗的 gate 回到 WAITING；UAV 在 revision 1 再回報定位 → OPEN
  Swarm swarm = makeSwarm({{1, {0, 0, 5}}}, {{50, {5, 0, 0}}});
  Gateway gw(swarm.node(kGateway));
  const auto ids = injectGroup(gw, swarm, "inc-b", {{1, {10, 0, 5}}, {2, {10, 0, 0}, false, 0b01}}, false, 0, "");
  swarm.run(2.0);
  CbbaComm & uav = swarm.node(1);
  takeOn(uav, ids[0]);
  swarm.run(0.5);
  uav.reportProgress(ids[0], ProgressPhase::EXECUTING, 1, kLocalized, swarm.now());
  swarm.run(0.5);
  EXPECT_EQ(swarm.node(50).gateState(ids[1], kWaitLocalized, swarm.now()), GateState::OPEN);
  uav.proposeIncident(proposal(ids[0], false), swarm.now());
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(50).gateState(ids[1], kWaitLocalized, swarm.now()), GateState::WAITING);
  // reportProgress 帶目前的 incident revision（1）；同樣的 phase、step 但里程碑在新 revision 是新的 → 會送
  uav.reportProgress(ids[0], ProgressPhase::EXECUTING, 1, kLocalized, swarm.now());
  swarm.run(0.5);
  EXPECT_EQ(swarm.node(50).gateState(ids[1], kWaitLocalized, swarm.now()), GateState::OPEN);
}

TEST(Incident, GatewayRejectsInvalidProposals)
{
  // 不是自己的正式指派：本機就拒（STALE）；舊的 base（STALE）、不是那個角色的執行者（NOT_EXECUTOR）：閘道回 Rejected，
  // revision 不變；同一個提案重送：回同一個結果（冪等）
  Swarm swarm = makeSwarm({{1, {0, 0, 5}}}, {{50, {5, 0, 0}}});
  Gateway gw(swarm.node(kGateway));
  const auto ids = injectGroup(gw, swarm, "inc-c", {{1, {10, 0, 5}}, {2, {10, 0, 0}}}, false, 0, "");
  swarm.run(2.0);
  CbbaComm & uav = swarm.node(1);
  EXPECT_EQ(uav.proposeIncident(proposal(ids[0], true), swarm.now()), ReportCheck::STALE);
  const std::uint64_t v = takeOn(uav, ids[0]);
  swarm.run(1.0);
  uav.proposeIncident(proposal(ids[0], true), swarm.now());
  swarm.run(1.0);
  const TaskId group = uav.agent().tasks().at(ids[0]).ext.group_id;
  ASSERT_EQ(swarm.node(kGateway).incidentRevision(group), 1);

  // 手做提案封包送給閘道
  std::uint32_t header_seq = 50000;   // 每則封包的序號都不同（重送同一個提案也是新封包，只有 proposal_seq 一樣）
  const auto send = [&](std::uint16_t base, TaskId reporter, std::uint64_t version, std::uint32_t seq) {
      wire::IncidentUpdateBody b;
      b.kind = wire::IncidentKind::PROPOSAL;
      b.group_id = group;
      b.incident_revision = base;
      b.proposer = 1;
      b.proposal_seq = seq;
      b.reporter_task = reporter;
      b.assign_version = version;
      b.flags = wire::kIncidentPositionValid;
      ++header_seq;
      b.relay = wire::RelayHeader{1, uav.sessionId(), header_seq};
      wire::Header h;
      h.sender_id = 1;
      h.session_id = uav.sessionId();
      h.sequence = header_seq;
      swarm.node(kGateway).receive(wire::encodeIncidentUpdate(h, b), swarm.now());
    };
  send(0, ids[0], v, 900);   // base 0 已經舊了
  send(1, ids[1], v, 901);   // role 1 不是 UAV 的
  EXPECT_EQ(swarm.node(kGateway).stats().incident_rejected, 2);
  EXPECT_EQ(swarm.node(kGateway).incidentRevision(group), 1);
  send(1, ids[1], v, 901);   // 同一個提案重送：回同一個結果，不重算
  EXPECT_EQ(swarm.node(kGateway).stats().incident_rejected, 2);
}

TEST(Incident, NewNeighborGetsIncidentState)
{
  // 狗在 Accepted 之後才重開：閘道的定期重送／快照補到 incident revision
  Swarm swarm = makeSwarm({{1, {0, 0, 5}}}, {{50, {5, 0, 0}}});
  Gateway gw(swarm.node(kGateway));
  const auto ids = injectGroup(gw, swarm, "inc-d", {{1, {10, 0, 5}}, {2, {10, 0, 0}, false, 0b01}}, false, 0, "");
  swarm.run(2.0);
  takeOn(swarm.node(1), ids[0]);
  swarm.run(0.5);
  swarm.node(1).proposeIncident(proposal(ids[0], true), swarm.now());
  swarm.run(1.0);
  CbbaComm & dog = swarm.restart(50);
  swarm.run(4.0);
  const TaskId group = swarm.node(1).agent().tasks().at(ids[0]).ext.group_id;
  EXPECT_EQ(dog.incidentRevision(group), 1);
  EXPECT_EQ(dog.gateState(ids[1], kWaitLocalized, swarm.now()), GateState::OPEN);
}

// ===========================================================================
// distinct_agent_set（使用者 11/16 的三個案例）
// ===========================================================================
TEST(Distinct, NearDogTakesOnlyOneRole)
{
  // 案例 1：dog50 靠近 front、rear，dog51 很遠 → dog50 只拿其中一個，dog51 拿另一個；dog50 對另一個的出價原因是 distinct
  Swarm swarm = makeSwarm({}, {{50, {0, 0, 0}}, {51, {60, 0, 0}}});
  Gateway gw(swarm.node(kGateway));
  const auto ids = injectGroup(gw, swarm, "dist-1", {{2, {10, 0, 0}}, {2, {12, 0, 0}}}, false, 0b11, "");
  swarm.run(3.0);
  const AgentId w0 = swarm.node(50).agent().winner(ids[0]);
  const AgentId w1 = swarm.node(50).agent().winner(ids[1]);
  EXPECT_NE(w0, kNoAgent);
  EXPECT_NE(w1, kNoAgent);
  EXPECT_NE(w0, w1);
  EXPECT_TRUE((w0 == 50 && w1 == 51) || (w0 == 51 && w1 == 50));
  const TaskId other = w0 == 50 ? ids[1] : ids[0];
  EXPECT_EQ(swarm.node(50).agent().bidFor(other, swarm.now()).why, std::string("distinct"));
  EXPECT_TRUE(swarm.consistent());
  // 都有人接：沒有失效事件
  swarm.onTick([&gw](double now) {EXPECT_TRUE(gw.stepGroups(now).empty());});
  swarm.run(12.0);
}

TEST(Distinct, OnlyOneDogGroupIncompleteThenFallback)
{
  // 案例 2：只有 dog50、組是 all-or-nothing（front、rear 都必要、distinct）→ dog50 只拿一個角色；組維持 WAITING_GROUP，
  // 機器人端 GROUP_INCOMPLETE（這裡 group_commit_timeout 5 s）；閘道等 group_recovery_timeout 10 s 後整組取消、FallbackRequest
  Swarm swarm = makeSwarm({}, {{50, {0, 0, 0}}});
  Gateway gw(swarm.node(kGateway));
  Runner dog(swarm.node(50), 5.0);
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      dog.tick(now);
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  const auto ids = injectGroup(gw, swarm, "dist-2", {{2, {10, 0, 0}}, {2, {12, 0, 0}}}, true, 0b11, "single_dog_verify");
  swarm.run(7.0);
  const int held = (swarm.node(50).agent().winner(ids[0]) == 50) + (swarm.node(50).agent().winner(ids[1]) == 50);
  EXPECT_EQ(held, 1);
  EXPECT_FALSE(dog.am.active());
  ASSERT_EQ(dog.timeouts.size(), 1u);   // GROUP_INCOMPLETE（機器人端）
  EXPECT_TRUE(events.empty());          // 閘道還在等恢復
  swarm.run(6.0);
  ASSERT_EQ(events.size(), 1u);
  const GroupEvent & e = events[0];
  EXPECT_EQ(e.reason, GroupFailure::RECOVERY_TIMEOUT);
  EXPECT_EQ(e.triggers, std::vector<RecoveryTrigger>{RecoveryTrigger::NO_WINNER});   // 有能做的狗，但被 distinct 擋住、沒人得標
  EXPECT_EQ(e.failure_version, 1u);
  EXPECT_TRUE(e.all_or_nothing);
  EXPECT_TRUE(e.fallback);
  EXPECT_EQ(e.fallback_template, "single_dog_verify");
  EXPECT_EQ(e.incident_id, "dist-2");
  EXPECT_EQ(e.cancelled_roles.size(), 2u);   // 整組取消
  swarm.run(1.0);
  EXPECT_FALSE(swarm.node(50).taskOpen(ids[0]));
  EXPECT_FALSE(swarm.node(50).taskOpen(ids[1]));
  EXPECT_FALSE(dog.am.active());
  swarm.run(5.0);
  EXPECT_EQ(events.size(), 1u);   // 冪等：同一個失效不再發
}

TEST(Distinct, SameAgentSequenceIsOneRole)
{
  // 案例 3：同一台依序做兩個動作 → 寫成同一個角色的兩個 steps（role_count 1），不受互斥限制，正常提交執行
  Swarm swarm = makeSwarm({}, {{50, {0, 0, 0}}});
  Gateway gw(swarm.node(kGateway));
  Runner dog(swarm.node(50), 30.0);
  swarm.onTick([&dog](double now) {dog.tick(now);});
  InjectRequest req = roleRequest("dist-3", 0, 1, {2, {10, 0, 0}});
  req.task.ext.all_or_nothing = true;
  req.task.ext.distinct_agent_set = 0b1;
  req.spec = R"({"steps": [{"action": "go_to", "location": {"x": 10, "y": 0, "z": 0}},
    {"action": "inspect", "object": "person"}, {"action": "go_to", "location": {"x": 12, "y": 0, "z": 0}},
    {"action": "inspect", "object": "person"}]})";
  const InjectResult r = gw.inject(req, swarm.now());
  ASSERT_EQ(r.status, InjectStatus::CREATED) << r.reason;
  swarm.run(4.0);
  ASSERT_TRUE(dog.am.active());
  EXPECT_EQ(dog.am.active()->task, r.task_id);
  EXPECT_TRUE(dog.timeouts.empty());
}

// ===========================================================================
// 連鎖撤銷（非 all-or-nothing 的依賴失效）
// ===========================================================================
TEST(Cascade, DependentRolesCancelledIndependentKept)
{
  // role 0 空中（必要，但沒有 UAV）、role 1 front 等 role 0（precedence）、role 2 rear 不依賴。
  // 沒有能做 role 0 的載具不是永久（使用者 11/16）：暫時不完整，等 group_recovery_timeout 10 s 才以 RECOVERY_TIMEOUT 收尾
  //（最初觸發 NO_CAPABLE）；role 0、role 1 取消，role 2 照做；有 fallback_template → FallbackRequest
  Swarm swarm = makeSwarm({}, {{50, {0, 0, 0}}, {51, {0, 5, 0}}});
  Gateway gw(swarm.node(kGateway));
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  const auto ids = injectGroup(gw, swarm, "casc-1",
      {{1, {10, 0, 5}}, {2, {10, 0, 0}, false, 0b001}, {2, {12, 0, 0}}}, false, 0b110, "single_dog_verify");
  swarm.run(8.0);
  EXPECT_TRUE(events.empty());   // 3 s 不再是永久（membership 2 s ＋ 10 s 才到）
  EXPECT_TRUE(swarm.node(50).taskOpen(ids[0]));
  swarm.run(6.0);
  ASSERT_EQ(events.size(), 1u);
  const GroupEvent & e = events[0];
  EXPECT_EQ(e.reason, GroupFailure::RECOVERY_TIMEOUT);
  EXPECT_EQ(e.triggers, std::vector<RecoveryTrigger>{RecoveryTrigger::NO_CAPABLE});
  EXPECT_EQ(e.failed_roles, std::vector<std::uint8_t>{0});
  std::vector<std::uint8_t> cancelled = e.cancelled_roles;
  std::sort(cancelled.begin(), cancelled.end());
  EXPECT_EQ(cancelled, (std::vector<std::uint8_t>{0, 1}));
  EXPECT_FALSE(e.all_or_nothing);
  EXPECT_TRUE(e.fallback);
  swarm.run(1.0);
  EXPECT_FALSE(swarm.node(50).taskOpen(ids[1]));
  EXPECT_TRUE(swarm.node(50).taskOpen(ids[2]));   // 不依賴的照做
  swarm.run(5.0);
  EXPECT_EQ(events.size(), 1u);   // 被連鎖取消的不算成新的失效
}

TEST(Cascade, NoDependentsOnlyNotifies)
{
  // 非 all-or-nothing、沒有角色依賴失效的角色：只取消失效的那個；沒有 fallback_template → 不發 FallbackRequest
  Swarm swarm = makeSwarm({}, {{50, {0, 0, 0}}});
  Gateway gw(swarm.node(kGateway));
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  const auto ids = injectGroup(gw, swarm, "casc-2", {{1, {10, 0, 5}}, {2, {10, 0, 0}}}, false, 0, "");
  swarm.run(14.0);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].cancelled_roles, std::vector<std::uint8_t>{0});
  EXPECT_FALSE(events[0].fallback);
  EXPECT_TRUE(swarm.node(50).taskOpen(ids[1]));
}

TEST(Cascade, CancelledMandatoryRoleBumpsFailureVersion)
{
  // 必要角色被（外部）取消 → 永久失效、立即處理；第二個角色之後也被取消 → 新的失效，group_failure_version 2
  Swarm swarm = makeSwarm({}, {{50, {0, 0, 0}}, {51, {0, 5, 0}}, {52, {0, 10, 0}}});
  Gateway gw(swarm.node(kGateway));
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  const auto ids = injectGroup(gw, swarm, "casc-3", {{2, {10, 0, 0}}, {2, {12, 0, 0}}, {2, {14, 0, 0}}}, false, 0b111, "fb");
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(kGateway).cancelTask(ids[0], swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(1.0);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, GroupFailure::CANCELLED);   // 確定終止：立即
  EXPECT_EQ(events[0].triggers, std::vector<RecoveryTrigger>{RecoveryTrigger::NONE});
  EXPECT_EQ(events[0].failure_version, 1u);
  EXPECT_TRUE(events[0].cancelled_roles.empty());   // 已經被取消了、也沒有依賴它的 → 沒有新取消的
  EXPECT_FALSE(events[0].fallback);
  ASSERT_EQ(swarm.node(kGateway).cancelTask(ids[1], swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(1.0);
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[1].failure_version, 2u);
  EXPECT_EQ(events[1].failed_roles, std::vector<std::uint8_t>{1});
  EXPECT_TRUE(swarm.node(50).taskOpen(ids[2]));
}

TEST(Cascade, PlannerRevisionCannotBeAheadOfIncident)
{
  // 規劃器依 incident_updates 改版：依據的 incident revision 不能比閘道目前的新；fallback_template 要和組一樣
  Swarm swarm = makeSwarm({}, {{50, {0, 0, 0}}});
  Gateway gw(swarm.node(kGateway));
  injectGroup(gw, swarm, "plan-1", {{2, {10, 0, 0}}, {2, {12, 0, 0}}}, false, 0, "fb");
  swarm.run(2.0);
  InjectRequest rev = roleRequest("plan-1", 1, 2, {2, {13, 0, 0}});
  rev.revision = 2;
  rev.fallback_template = "fb";
  rev.incident_revision = 1;   // 閘道目前是 0
  EXPECT_EQ(gw.inject(rev, swarm.now()).status, InjectStatus::REJECTED);
  rev.incident_revision = 0;
  rev.fallback_template = "other";
  EXPECT_EQ(gw.inject(rev, swarm.now()).status, InjectStatus::REJECTED);
  rev.fallback_template = "fb";
  EXPECT_EQ(gw.inject(rev, swarm.now()).status, InjectStatus::REVISED);
  InjectRequest other_role = roleRequest("plan-1", 0, 2, {2, {10, 0, 0}});
  other_role.revision = 1;
  other_role.role = "roleX";   // 新角色名稱 → 新任務；fallback_template 和組不同 → 拒絕
  other_role.fallback_template = "zzz";
  EXPECT_EQ(gw.inject(other_role, swarm.now()).status, InjectStatus::REJECTED);
}

TEST(Cascade, NoCapableRecoversWhenDogJoins)
{
  // 地面角色一開始沒有能做的載具（狗還沒上線）→ 暫時不完整；5 s 後狗上線、得標 → 恢復，繼續執行，沒有失效事件
  Swarm swarm = makeSwarm({{1, {0, 0, 5}}}, {});
  Gateway gw(swarm.node(kGateway));
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  const auto ids = injectGroup(gw, swarm, "rec-1", {{2, {10, 0, 0}}, {1, {10, 0, 5}}}, false, 0, "fb");
  swarm.run(5.0);
  swarm.add(50, {5, 0, 0}, 100.0, AgentType::UGV);
  swarm.run(12.0);
  EXPECT_TRUE(events.empty());
  EXPECT_TRUE(swarm.node(kGateway).taskOpen(ids[0]));
  EXPECT_EQ(swarm.node(kGateway).agent().winner(ids[0]), 50);
  EXPECT_TRUE(gw.groups().begin()->second.recovering.empty());
}

TEST(Cascade, HolderLostRecoversByTakeover)
{
  // 執行者失聯 → 先標記、等失聯接手（這裡 P0 3 s），這段期間不算恢復期限（做法 A）；撤銷後另一隻狗得標 → 恢復，沒有失效事件
  Swarm swarm = makeSwarm({}, {{50, {0, 0, 0}}, {51, {30, 0, 0}}}, 3.0);
  Gateway gw(swarm.node(kGateway));
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  const auto ids = injectGroup(gw, swarm, "rec-2", {{2, {10, 0, 0}}}, false, 0, "fb");
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(50).agent().winner(ids[0]), 50);
  takeOn(swarm.node(50), ids[0]);
  swarm.run(1.0);
  swarm.setOnline(50, false);
  swarm.run(4.0);   // 失聯（約 1.5 s）之後、接手（約 4.5 s）之前
  EXPECT_EQ(gw.groups().begin()->second.holder_lost.count(ids[0]), 1u);
  EXPECT_TRUE(gw.groups().begin()->second.recovering.empty());
  swarm.run(10.0);
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(swarm.node(kGateway).agent().winner(ids[0]), 51);
  EXPECT_TRUE(swarm.node(kGateway).taskOpen(ids[0]));
}

TEST(Cascade, HolderLostKeepsInitialTrigger)
{
  // dog50 執行 role 0、dog51 執行 role 1（distinct）；dog50 失聯 → 等失聯接手（P0 3 s）；撤銷後 dog51 因為 distinct 不能接、
  // 沒人得標（NO_WINNER）→ 觸發原因記 HOLDER_LOST；撤銷後 10 s 才 RECOVERY_TIMEOUT，只取消 role 0（role 1 不依賴）
  Swarm swarm = makeSwarm({}, {{50, {8, 0, 0}}, {51, {40, 0, 0}}}, 3.0);
  Gateway gw(swarm.node(kGateway));
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  const auto ids = injectGroup(gw, swarm, "rec-3", {{2, {10, 0, 0}}, {2, {42, 0, 0}}}, false, 0b11, "fb");
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(50).agent().winner(ids[0]), 50);
  ASSERT_EQ(swarm.node(51).agent().winner(ids[1]), 51);
  takeOn(swarm.node(50), ids[0]);
  takeOn(swarm.node(51), ids[1]);
  swarm.run(1.0);
  swarm.setOnline(50, false);
  swarm.run(13.5);   // 失聯 1.5 s ＋ 接手 3 s ＋ 恢復 10 s ≈ 14.5 s 才到
  EXPECT_TRUE(events.empty());
  swarm.run(1.7);
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].reason, GroupFailure::RECOVERY_TIMEOUT);
  EXPECT_EQ(events[0].failed_roles, std::vector<std::uint8_t>{0});
  EXPECT_EQ(events[0].triggers, std::vector<RecoveryTrigger>{RecoveryTrigger::HOLDER_LOST});
  EXPECT_EQ(events[0].cancelled_roles, std::vector<std::uint8_t>{0});
  EXPECT_TRUE(swarm.node(51).taskOpen(ids[1]));
}

// ===========================================================================
// 執行者失聯 vs 失聯接手（做法 A，使用者 11/16）：接手時限內不算恢復期限，撤銷、重新開放競標後才開始算 10 s
// ===========================================================================
TEST(Takeover, SuccessfulTakeoverDoesNotCancel)
{
  // 接手時限（P0 12 s）比恢復期限（10 s）長：舊規則在失聯 1.5 + 10 s 就會 RECOVERY_TIMEOUT，接手根本輪不到。
  // 做法 A：接手前只標記；撤銷後 dog51 得標並建立新的正式指派 → 組繼續，之後也沒有失效事件
  Swarm swarm = makeSwarm({}, {{50, {0, 0, 0}}, {51, {30, 0, 0}}}, 12.0);
  Gateway gw(swarm.node(kGateway));
  Runner dog51(swarm.node(51), 30.0);
  bool dog51_runs = false;   // dog50 接下之後才跑 dog51 的指派流程：否則一開始 dog51 短暫得標就被正式指派、鎖住
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      if (dog51_runs) {
        dog51.tick(now);
      }
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  // 只有一個角色：dog51 閒著才接得到（執行中的 P0 之後不排隊第二個 P0）
  const auto ids = injectGroup(gw, swarm, "tk-1", {{2, {10, 0, 0}}}, false, 0, "fb");
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(50).agent().winner(ids[0]), 50);
  takeOn(swarm.node(50), ids[0]);
  dog51_runs = true;
  swarm.run(1.0);
  swarm.setOnline(50, false);
  swarm.run(12.5);   // 舊規則的 RECOVERY_TIMEOUT（≈ 11.5 s）已過，接手（≈ 13.5 s）還沒到
  EXPECT_TRUE(events.empty());
  EXPECT_EQ(gw.groups().begin()->second.holder_lost.count(ids[0]), 1u);
  EXPECT_EQ(gw.groups().begin()->second.recovering.count(ids[0]), 0u);
  EXPECT_TRUE(swarm.node(kGateway).taskOpen(ids[0]));
  swarm.run(6.0);   // 接手撤銷 → dog51 得標、正式指派
  EXPECT_EQ(swarm.node(kGateway).agent().winner(ids[0]), 51);
  const Assignment a = swarm.node(kGateway).assignment(ids[0]);
  EXPECT_EQ(a.assignee, 51);
  EXPECT_TRUE(isLocking(a.state));
  swarm.run(15.0);
  EXPECT_TRUE(events.empty());
  EXPECT_TRUE(gw.groups().begin()->second.recovering.empty());
  EXPECT_TRUE(gw.groups().begin()->second.holder_lost.empty());
  EXPECT_TRUE(swarm.node(kGateway).taskOpen(ids[0]));
}

TEST(Takeover, TakeoverTimeoutTriggersFallback)
{
  // 能做地面角色的只有 dog50（UAV 讓閘道一直有鄰居）：失聯 → 等接手 3 s → 撤銷、重新開放競標，沒有人能接 →
  // 從撤銷起 10 s 才 RECOVERY_TIMEOUT（HOLDER_LOST）；非 all-or-nothing：取消失效的 role 0 和依賴它的 role 1，發 fallback
  Swarm swarm = makeSwarm({{1, {0, 0, 5}}}, {{50, {8, 0, 0}}}, 3.0);
  Gateway gw(swarm.node(kGateway));
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  const auto ids = injectGroup(gw, swarm, "tk-2", {{2, {10, 0, 0}}, {1, {10, 0, 5}, false, 0b001}}, false, 0,
      "single_dog_verify");
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(50).agent().winner(ids[0]), 50);
  takeOn(swarm.node(50), ids[0]);
  swarm.run(1.0);
  swarm.setOnline(50, false);
  swarm.run(4.0);   // 等接手中
  EXPECT_EQ(gw.groups().begin()->second.holder_lost.count(ids[0]), 1u);
  swarm.run(9.5);   // 接手 ≈ 4.5 s、撤銷後 10 s ≈ 14.5 s
  EXPECT_TRUE(events.empty());
  ASSERT_EQ(gw.groups().begin()->second.recovering.count(ids[0]), 1u);
  EXPECT_EQ(gw.groups().begin()->second.recovering.at(ids[0]).trigger, RecoveryTrigger::HOLDER_LOST);
  swarm.run(1.7);
  ASSERT_EQ(events.size(), 1u);
  const GroupEvent & e = events[0];
  EXPECT_EQ(e.reason, GroupFailure::RECOVERY_TIMEOUT);
  EXPECT_EQ(e.triggers, std::vector<RecoveryTrigger>{RecoveryTrigger::HOLDER_LOST});
  EXPECT_EQ(e.failed_roles, std::vector<std::uint8_t>{0});
  std::vector<std::uint8_t> cancelled = e.cancelled_roles;
  std::sort(cancelled.begin(), cancelled.end());
  EXPECT_EQ(cancelled, (std::vector<std::uint8_t>{0, 1}));
  EXPECT_TRUE(e.fallback);
  EXPECT_EQ(e.failure_version, 1u);
}

TEST(Takeover, WaitFollowsTaskPriority)
{
  // 同一隻狗執行 P0、P1 兩個角色後失聯：接手時限 P0 3 s、P1 8 s，各自從撤銷起算 10 s。
  // P0 ≈ 1.5 + 3 + 10 = 14.5 s 失效，P1 ≈ 1.5 + 8 + 10 = 19.5 s 失效（兩次事件，failure_version 1、2）
  Swarm swarm = makeSwarm({{1, {0, 0, 5}}}, {{50, {8, 0, 0}}}, 3.0, 8.0);
  Gateway gw(swarm.node(kGateway));
  std::vector<GroupEvent> events;
  swarm.onTick([&](double now) {
      for (GroupEvent & e : gw.stepGroups(now)) {
        events.push_back(std::move(e));
      }
    });
  RoleSpec p0{2, {10, 0, 0}};
  RoleSpec p1{2, {12, 0, 0}};
  p1.priority = 1;
  const auto ids = injectGroup(gw, swarm, "tk-3", {p0, p1}, false, 0, "fb");
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(50).agent().winner(ids[0]), 50);
  ASSERT_EQ(swarm.node(50).agent().winner(ids[1]), 50);
  takeOn(swarm.node(50), ids[0]);
  takeOn(swarm.node(50), ids[1]);
  swarm.run(1.0);
  swarm.setOnline(50, false);
  swarm.run(8.0);   // P0 已撤銷、在算恢復期限；P1 還在等接手
  const Gateway::GroupData & g = gw.groups().begin()->second;
  EXPECT_EQ(g.recovering.count(ids[0]), 1u);
  EXPECT_EQ(g.holder_lost.count(ids[0]), 0u);
  EXPECT_EQ(g.holder_lost.count(ids[1]), 1u);
  EXPECT_EQ(g.recovering.count(ids[1]), 0u);
  swarm.run(6.0);   // 14 s
  EXPECT_TRUE(events.empty());
  swarm.run(1.2);   // 15.2 s：P0 失效
  ASSERT_EQ(events.size(), 1u);
  EXPECT_EQ(events[0].failed_roles, std::vector<std::uint8_t>{0});
  EXPECT_EQ(events[0].failure_version, 1u);
  EXPECT_TRUE(swarm.node(kGateway).taskOpen(ids[1]));   // P1 不依賴 P0，還在等
  swarm.run(3.6);   // 18.8 s
  EXPECT_EQ(events.size(), 1u);
  swarm.run(1.6);   // 20.4 s：P1 失效
  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[1].failed_roles, std::vector<std::uint8_t>{1});
  EXPECT_EQ(events[1].triggers, std::vector<RecoveryTrigger>{RecoveryTrigger::HOLDER_LOST});
  EXPECT_EQ(events[1].failure_version, 2u);
}
