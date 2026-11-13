// 閘道的任務注入（2-1b M0 第 4a 步）：冪等、欄位檢查、task_id／group_id、經過模擬網路傳到全隊
#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
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

// 地面任務（四足、mobility_ground），P1
InjectRequest groundRequest(const std::string & incident, const std::string & role = std::string())
{
  InjectRequest req;
  req.incident_id = incident;
  req.role = role;
  req.revision = 1;
  req.task.platform = 2;
  req.task.required_caps = capBit(Capability::MOBILITY_GROUND);
  req.task.priority = 1;
  req.task.position = {12.0, 0.0, 0.0};
  req.task.value = 100.0;
  req.task.duration_sec = 5.0;
  return req;
}

// 閘道（不出價）＋一台無人機＋一隻狗；閘道有世代 3
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

TEST(Gateway, InjectReachesTeamAndOnlyCapableAgentWins)
{
  // 注入的任務用版本 4 的 task_id（建立者 = 閘道、世代 3），全隊收到定義與 TaskSpec；只有狗能做、狗得標，閘道不出價
  Swarm swarm = makeSwarm();
  Gateway gw(swarm.node(kGateway));
  InjectRequest req = groundRequest("intrusion-7");
  req.spec = "{\"steps\":[{\"action\":\"inspect\"}]}";
  req.expires_sec = 600.0;
  const InjectResult r = gw.inject(req, swarm.now());
  ASSERT_EQ(r.status, InjectStatus::CREATED) << r.reason;
  EXPECT_EQ(r.task_id, makeTaskId(kGateway, 3, 1));
  EXPECT_EQ(r.group_id, 0u);   // 單獨任務
  swarm.run(2.0);
  for (AgentId a : {AgentId{1}, AgentId{50}}) {
    const auto & tasks = swarm.node(a).agent().tasks();
    ASSERT_EQ(tasks.count(r.task_id), 1u) << a;
    const Task & t = tasks.at(r.task_id);
    EXPECT_EQ(t.type, TaskType::NONE) << a;
    EXPECT_EQ(t.priority, 1) << a;
    EXPECT_EQ(t.revision, 1) << a;
    EXPECT_NEAR(t.ext.expires, t.created + 600.0, 0.01) << a;   // 絕對時刻：轉送不會往後推
    EXPECT_TRUE(swarm.node(a).specReady(r.task_id)) << a;
  }
  EXPECT_EQ(swarm.node(1).agent().winner(r.task_id), 50);
  EXPECT_EQ(swarm.node(kGateway).agent().winner(r.task_id), 50);
  EXPECT_FALSE(swarm.node(kGateway).participating());
}

TEST(Gateway, SameKeyIsIdempotent)
{
  // 同一個 (incident, role, revision) 重送：回同一個 task_id、不再送 TASK_ANNOUNCE；revision 較舊拒絕；較新（改版）第 4b 步才做
  Swarm swarm = makeSwarm();
  Gateway gw(swarm.node(kGateway));
  InjectRequest req = groundRequest("smoke-1");
  req.revision = 2;
  const InjectResult first = gw.inject(req, swarm.now());
  ASSERT_EQ(first.status, InjectStatus::CREATED) << first.reason;
  swarm.run(0.5);
  const int announces = swarm.node(kGateway).stats().sent.at(PacketType::TASK_ANNOUNCE);

  const InjectResult again = gw.inject(req, swarm.now());
  EXPECT_EQ(again.status, InjectStatus::DUPLICATE);
  EXPECT_EQ(again.task_id, first.task_id);
  EXPECT_EQ(swarm.node(kGateway).stats().sent.at(PacketType::TASK_ANNOUNCE), announces);
  EXPECT_EQ(swarm.node(kGateway).taskCount(), 1u);

  req.revision = 1;
  const InjectResult older = gw.inject(req, swarm.now());
  EXPECT_EQ(older.status, InjectStatus::REJECTED);
  EXPECT_EQ(older.task_id, first.task_id);
  EXPECT_EQ(older.revision, 2);

  req.revision = 3;   // 較新：改版（4b），同一個 task_id
  const InjectResult newer = gw.inject(req, swarm.now());
  EXPECT_EQ(newer.status, InjectStatus::REVISED) << newer.reason;
  EXPECT_EQ(newer.task_id, first.task_id);
  EXPECT_EQ(newer.revision, 3);
  EXPECT_EQ(swarm.node(kGateway).taskCount(), 1u);
}

TEST(Gateway, RolesOfOneIncidentShareGroup)
{
  // 同一個事件的角色共用 group_id（格式同 task_id），各自一個任務；別的事件另一個 group
  Swarm swarm = makeSwarm();
  Gateway gw(swarm.node(kGateway));
  InjectRequest verify = groundRequest("intrusion-9", "ground_verify");
  verify.task.ext.role_count = 2;
  verify.task.ext.role_index = 0;
  InjectRequest watch = groundRequest("intrusion-9", "aerial_overwatch");
  watch.task.platform = 1;
  watch.task.required_caps = capBit(Capability::MOBILITY_AIR) | capBit(Capability::RGB_CAMERA);
  watch.task.ext.role_count = 2;
  watch.task.ext.role_index = 1;
  watch.task.ext.optional = true;
  watch.task.ext.skip_satisfies = true;
  verify.task.ext.precedence = 0b10;   // 等 aerial_overwatch（optional，被略過也算滿足）

  const InjectResult a = gw.inject(verify, swarm.now());
  const InjectResult b = gw.inject(watch, swarm.now());
  ASSERT_EQ(a.status, InjectStatus::CREATED) << a.reason;
  ASSERT_EQ(b.status, InjectStatus::CREATED) << b.reason;
  EXPECT_NE(a.group_id, 0u);
  EXPECT_EQ(a.group_id, b.group_id);
  EXPECT_NE(a.task_id, b.task_id);
  EXPECT_EQ(taskCreator(a.group_id), kGateway);

  InjectRequest other = groundRequest("intrusion-10", "ground_verify");
  other.task.ext.role_count = 1;
  const InjectResult c = gw.inject(other, swarm.now());
  ASSERT_EQ(c.status, InjectStatus::CREATED) << c.reason;
  EXPECT_NE(c.group_id, a.group_id);

  swarm.run(2.0);
  const Task & got = swarm.node(1).agent().tasks().at(b.task_id);
  EXPECT_EQ(got.ext.group_id, a.group_id);
  EXPECT_EQ(got.ext.role_index, 1);
  EXPECT_TRUE(got.ext.optional);
  EXPECT_TRUE(got.ext.skip_satisfies);
  EXPECT_EQ(swarm.node(1).agent().winner(b.task_id), 1);   // 空中角色給無人機
}

TEST(Gateway, RejectsBadRequests)
{
  // 欄位不合法時拒絕、不配號；沒有世代時不能注入
  Swarm swarm = makeSwarm();
  Gateway gw(swarm.node(kGateway));
  const auto rejected = [&](InjectRequest req) {
      const InjectResult r = gw.inject(req, swarm.now());
      return r.status == InjectStatus::REJECTED && !r.reason.empty();
    };
  EXPECT_TRUE(rejected(groundRequest("")));
  InjectRequest r1 = groundRequest("x1");
  r1.task.priority = 4;
  EXPECT_TRUE(rejected(r1));
  InjectRequest r2 = groundRequest("x2", "a");
  r2.task.ext.role_count = 2;
  r2.task.ext.role_index = 2;
  EXPECT_TRUE(rejected(r2));
  InjectRequest r3 = groundRequest("x3");
  r3.task.ext.precedence = 1;   // 單獨任務不能有 precedence
  EXPECT_TRUE(rejected(r3));
  InjectRequest r4 = groundRequest("x4");
  r4.release_sec = 30.0;
  r4.expires_sec = 10.0;
  EXPECT_TRUE(rejected(r4));
  InjectRequest r5 = groundRequest("x5");
  r5.task.value = 0.0;
  EXPECT_TRUE(rejected(r5));
  EXPECT_EQ(swarm.node(kGateway).taskCount(), 0u);
  EXPECT_TRUE(gw.injected().empty());

  Swarm legacy;   // 世代 0
  legacy.addGateway(kGateway);
  Gateway no_epoch(legacy.node(kGateway));
  EXPECT_EQ(no_epoch.inject(groundRequest("y"), legacy.now()).status, InjectStatus::REJECTED);
}

TEST(Gateway, IsAProtocolNodeOutsideAnyPlatform)
{
  // 閘道的定義（2026-11-13）：不出價（叫它參與也不理）、AGENT_STATE 平台 0、能力 0；
  // 別台知道它是閘道，能力統計（capableAgents）不算它；它照樣知道全隊的任務與得標者
  Swarm swarm = makeSwarm();
  CbbaComm & gw = swarm.node(kGateway);
  EXPECT_TRUE(gw.isGateway());
  gw.setParticipating(true, swarm.now());
  EXPECT_FALSE(gw.participating());
  const wire::AgentStateBody body = gw.agentStateBody(swarm.now());
  EXPECT_EQ(body.vehicle_type, wire::kVehicleGateway);
  EXPECT_EQ(body.capabilities, 0u);
  EXPECT_FALSE(body.flags.participating);

  swarm.run(1.0);
  const auto & seen = swarm.node(50).neighbors().at(kGateway);
  EXPECT_TRUE(seen.has_status);
  EXPECT_TRUE(seen.gateway);
  EXPECT_EQ(seen.capabilities, 0u);
  EXPECT_FALSE(swarm.node(50).neighbors().at(1).gateway);

  // 不限平台、不需要能力的任務：任何載具都能做，閘道照樣不算
  Task any;
  any.type = TaskType::NONE;
  any.platform = kPlatformAny;
  for (AgentId viewer : {kGateway, AgentId{1}, AgentId{50}}) {
    std::vector<AgentId> got = swarm.node(viewer).capableAgents(any, swarm.now());
    std::sort(got.begin(), got.end());
    EXPECT_EQ(got, (std::vector<AgentId>{1, 50})) << viewer;
  }
  Task ground;
  ground.type = TaskType::NONE;
  ground.platform = 2;
  ground.required_caps = capBit(Capability::MOBILITY_GROUND);
  EXPECT_EQ(swarm.node(kGateway).capableAgents(ground, swarm.now()), (std::vector<AgentId>{50}));
  Task thermal = ground;
  thermal.required_caps |= capBit(Capability::THERMAL_CAMERA);
  EXPECT_TRUE(swarm.node(kGateway).capableAgents(thermal, swarm.now()).empty());

  // 閘道注入後也知道得標者（監控用）
  Gateway inject(gw);
  const InjectResult r = inject.inject(groundRequest("watch-1"), swarm.now());
  ASSERT_EQ(r.status, InjectStatus::CREATED) << r.reason;
  swarm.run(2.0);
  EXPECT_EQ(gw.agent().winner(r.task_id), 50);
}

// ===========================================================================
// 改版（4b，2026-11-13）
// ===========================================================================
namespace
{
// 閘道＋無人機 1＋狗 50（狗有指派流程，不握手，同 cbba_node）。狗得標、執行中
struct Revisable
{
  Swarm swarm = makeSwarm();
  Gateway gw{swarm.node(kGateway)};
  std::unique_ptr<AssignmentManager> dog;
  int dog_publishes{0};
  InjectRequest req = groundRequest("move-1");
  TaskId id{0};

  explicit Revisable(double expires_sec = 0.0, double commit_radius = 0.0, bool optional = false)
  {
    AssignmentConfig c;
    c.require_accept = false;
    c.assign_hold = 0.4;
    dog = std::make_unique<AssignmentManager>(swarm.node(50), c, [](int, const std::string &) {});
    swarm.onTick([this](double now) {
        dog->setExec(ExecInput{}, now);
        dog->step(now);
        dog->takeRequests();
        dog_publishes += dog->takeAssignedChanged() ? 1 : 0;
      });
    req.spec = "{\"steps\":[{\"action\":\"inspect\"}]}";
    req.expires_sec = expires_sec;
    req.task.ext.commit_radius = commit_radius;
    if (optional) {   // 兩個角色的組裡的 optional 角色（另一個角色沒建：這一組不會被判成「必要角色都結束」）
      req.role = "extra";
      req.task.ext.role_count = 2;
      req.task.ext.role_index = 1;
      req.task.ext.optional = true;
    }
    const InjectResult r = gw.inject(req, swarm.now());
    EXPECT_EQ(r.status, InjectStatus::CREATED) << r.reason;
    id = r.task_id;
    swarm.run(2.0);
  }
};
}  // namespace

TEST(Revision, ContentChangeKeepsAssignmentAndRepublishes)
{
  // 只改位置與 TaskSpec：全隊換成 revision 2，正式指派（版本、執行者、執行中）不變；狗重發一次 current_task（新 revision）
  Revisable t;
  ASSERT_TRUE(t.dog->active());
  EXPECT_EQ(t.dog->active()->task, t.id);
  const Assignment before = t.swarm.node(kGateway).assignment(t.id);
  EXPECT_EQ(before.assignee, 50);
  EXPECT_EQ(before.state, AssignState::ACTIVE);
  const int publishes = t.dog_publishes;

  t.req.revision = 2;
  t.req.task.position = {14.0, 1.0, 0.0};
  t.req.spec = "{\"steps\":[{\"action\":\"inspect\"},{\"action\":\"inspect\"}]}";
  const InjectResult r = t.gw.inject(t.req, t.swarm.now());
  ASSERT_EQ(r.status, InjectStatus::REVISED) << r.reason;
  EXPECT_EQ(r.task_id, t.id);
  t.swarm.run(2.0);

  for (AgentId a : {kGateway, AgentId{1}, AgentId{50}}) {
    const Task & got = t.swarm.node(a).agent().tasks().at(t.id);
    EXPECT_EQ(got.revision, 2) << a;
    EXPECT_NEAR(got.position.x, 14.0, 1e-3) << a;
    const Assignment now = t.swarm.node(a).assignment(t.id);
    EXPECT_EQ(now.version, before.version) << a;
    EXPECT_EQ(now.assignee, 50) << a;
    EXPECT_EQ(now.state, AssignState::ACTIVE) << a;
  }
  ASSERT_TRUE(t.swarm.node(50).specReady(t.id));
  EXPECT_EQ(*t.swarm.node(50).taskSpec(t.id), t.req.spec);
  ASSERT_TRUE(t.dog->active());
  EXPECT_EQ(t.dog->active()->version, before.version);
  EXPECT_EQ(t.dog_publishes, publishes + 1);
  EXPECT_EQ(t.swarm.node(50).stats().reauctions, 0);
  EXPECT_EQ(t.swarm.node(50).agent().winner(t.id), 50);
}

TEST(Revision, PlatformChangeReauctions)
{
  // 平台、能力改成空中：狗撤銷自己的指派（新版本、執行者 0），無人機得標
  Revisable t;
  ASSERT_TRUE(t.dog->active());
  const std::uint64_t version = t.swarm.node(kGateway).assignment(t.id).version;

  t.req.revision = 2;
  t.req.task.platform = 1;
  t.req.task.required_caps = capBit(Capability::MOBILITY_AIR) | capBit(Capability::RGB_CAMERA);
  ASSERT_EQ(t.gw.inject(t.req, t.swarm.now()).status, InjectStatus::REVISED);
  t.swarm.run(2.0);

  EXPECT_FALSE(t.dog->active());
  EXPECT_EQ(t.swarm.node(50).stats().reauctions, 1);
  for (AgentId a : {kGateway, AgentId{1}, AgentId{50}}) {
    const Assignment now = t.swarm.node(a).assignment(t.id);
    EXPECT_GT(now.version, version) << a;
    EXPECT_EQ(now.assignee, kNoAgent) << a;   // 無人機沒有指派流程：得標但沒有正式指派
    EXPECT_EQ(t.swarm.node(a).agent().winner(t.id), 1) << a;
  }
}

TEST(Revision, LaggingNodeCatchesUp)
{
  // 無人機漏掉改版的 TASK_ANNOUNCE（直接的和轉送的都掉）：看到別台的紀錄是較新的 revision，向鄰居要，補到新版與新的 TaskSpec
  Revisable t;
  t.swarm.block(kGateway, 1, PacketType::TASK_ANNOUNCE);
  t.swarm.block(50, 1, PacketType::TASK_ANNOUNCE);
  t.req.revision = 2;
  t.req.task.position = {15.0, 0.0, 0.0};
  t.req.spec = "{\"steps\":[]}";
  ASSERT_EQ(t.gw.inject(t.req, t.swarm.now()).status, InjectStatus::REVISED);
  t.swarm.run(3.0);
  const Task & got = t.swarm.node(1).agent().tasks().at(t.id);
  EXPECT_EQ(got.revision, 2);
  EXPECT_NEAR(got.position.x, 15.0, 1e-3);
  ASSERT_TRUE(t.swarm.node(1).specReady(t.id));
  EXPECT_EQ(*t.swarm.node(1).taskSpec(t.id), t.req.spec);
}

TEST(Revision, OnlyCreatorRevisesAndRolesStay)
{
  // 不是建立者不能改版；revision 沒變大不行；閘道不讓角色組改版
  Revisable t;
  Task mine = t.swarm.node(50).agent().tasks().at(t.id);
  mine.revision = 9;
  EXPECT_FALSE(t.swarm.node(50).reviseLocalTask(mine, t.swarm.now()));
  Task same = t.swarm.node(kGateway).agent().tasks().at(t.id);
  EXPECT_FALSE(t.swarm.node(kGateway).reviseLocalTask(same, t.swarm.now()));

  InjectRequest grouped = groundRequest("grp", "front");
  grouped.task.ext.role_count = 2;
  ASSERT_EQ(t.gw.inject(grouped, t.swarm.now()).status, InjectStatus::CREATED);
  grouped.revision = 2;
  grouped.task.ext.role_count = 3;
  EXPECT_EQ(t.gw.inject(grouped, t.swarm.now()).status, InjectStatus::REJECTED);
}

// ===========================================================================
// 過期（4c，2026-11-13）
// ===========================================================================
TEST(Expiry, CreatorClosesExpiredTask)
{
  // expires 到了：狗停止出價、撤銷自己的指派；建立者（閘道）送 TASK_CLOSE reason 3，全隊關掉（狀態 CANCELLED）
  Revisable t(3.0);
  ASSERT_TRUE(t.dog->active());
  t.swarm.run(2.0);
  EXPECT_FALSE(t.dog->active());
  for (AgentId a : {kGateway, AgentId{1}, AgentId{50}}) {
    EXPECT_EQ(t.swarm.node(a).agent().tasks().at(t.id).status, TaskStatus::CANCELLED) << a;
    EXPECT_EQ(t.swarm.node(a).closeReason(t.id), std::optional<std::uint8_t>(wire::kCloseExpired)) << a;
  }
  EXPECT_EQ(t.swarm.node(kGateway).stats().expired_closes, 1);
  EXPECT_EQ(t.swarm.node(kGateway).stats().rejected_closes, 0);
  EXPECT_EQ(t.swarm.node(50).stats().rejected_closes, 0);
}

TEST(Expiry, WithoutCreatorStopsLocallyThenClosesOnReturn)
{
  // 建立者離線時：狗照樣在 expires 停止、撤銷指派，任務先開著（沒人出價）；建立者回來後送 TASK_CLOSE reason 3
  Revisable t(3.0);
  ASSERT_TRUE(t.dog->active());
  t.swarm.setOnline(kGateway, false);
  t.swarm.run(2.0);
  EXPECT_FALSE(t.dog->active());
  EXPECT_TRUE(t.swarm.node(50).taskOpen(t.id));
  EXPECT_TRUE(t.swarm.node(50).expired(t.id, t.swarm.now()));
  EXPECT_EQ(t.swarm.node(50).assignment(t.id).assignee, kNoAgent);
  EXPECT_EQ(t.swarm.node(50).agent().winner(t.id), kNoAgent);
  EXPECT_EQ(t.swarm.node(1).agent().winner(t.id), kNoAgent);
  EXPECT_EQ(t.swarm.node(50).stats().expired_local, 1);

  t.swarm.setOnline(kGateway, true);
  t.swarm.run(3.0);
  for (AgentId a : {AgentId{1}, AgentId{50}}) {
    EXPECT_FALSE(t.swarm.node(a).taskOpen(t.id)) << a;
    EXPECT_EQ(t.swarm.node(a).closeReason(t.id), std::optional<std::uint8_t>(wire::kCloseExpired)) << a;
  }
}

TEST(Expiry, NoExpiresNeverExpires)
{
  // 沒設 expires 的任務（舊型任務都是）照常
  Revisable t;
  t.swarm.run(5.0);
  EXPECT_TRUE(t.swarm.node(50).taskOpen(t.id));
  EXPECT_FALSE(t.swarm.node(50).expired(t.id, t.swarm.now()));
  EXPECT_TRUE(t.dog->active());
}

// ===========================================================================
// 搶占與接手（4d，2026-11-13）
// ===========================================================================
TEST(Preempt, MaskAndLegacyRule)
{
  // 舊型：火警（P0）可以搶巡檢（P3），反過來、同級不行。新型：預設搶比自己低的級；遮罩可以指定（含同級）
  Task fire;
  fire.type = TaskType::GROUND_INTERVENTION;
  Task patrol;
  patrol.type = TaskType::PATROL;
  EXPECT_TRUE(mayPreempt(fire, patrol));
  EXPECT_FALSE(mayPreempt(patrol, fire));
  EXPECT_FALSE(mayPreempt(fire, fire));

  Task p1;
  p1.type = TaskType::NONE;
  p1.priority = 1;
  Task p0 = p1;
  p0.priority = 0;
  Task p3 = p1;
  p3.priority = 3;
  EXPECT_TRUE(mayPreempt(p1, p3));
  EXPECT_FALSE(mayPreempt(p1, p1));
  EXPECT_FALSE(mayPreempt(p1, p0));
  p1.ext.can_preempt = 0b0010;   // 只搶 P1（同級）
  EXPECT_TRUE(mayPreempt(p1, p1));
  EXPECT_FALSE(mayPreempt(p1, p3));
}

TEST(Takeover, LimitFollowsPriority)
{
  // 持有者失聯後依優先級撤銷：P1 15 s（P0 10 s、P2／P3 30 s）。狗離線 → 閘道約 1.5 + 15 s 後撤銷它的指派
  Revisable t;
  ASSERT_TRUE(t.dog->active());
  EXPECT_DOUBLE_EQ(t.swarm.node(kGateway).takeoverLimit(t.id), 15.0);
  t.swarm.setOnline(50, false);
  t.swarm.run(12.0);
  EXPECT_EQ(t.swarm.node(kGateway).assignment(t.id).assignee, 50);   // 還鎖著（寧可等）
  t.swarm.run(7.0);
  EXPECT_EQ(t.swarm.node(kGateway).assignment(t.id).assignee, kNoAgent);
  EXPECT_GE(t.swarm.node(kGateway).stats().takeovers, 1);
}

TEST(Commit, NearTargetCannotBePreempted)
{
  // 執行中任務的 commit_radius 內（狗 (5,0)、目標 (12,0)、半徑 10）：P0 的新任務搶不到，沒人接；
  // 狗離開 1.5 倍半徑後解除，P0 搶占成功。AGENT_STATE 帶「已接近」
  Revisable t(0.0, 10.0);
  ASSERT_TRUE(t.dog->active());
  t.swarm.run(1.0);
  EXPECT_TRUE(t.swarm.node(kGateway).neighbors().at(50).committed);

  InjectRequest urgent = groundRequest("fire-9");
  urgent.task.priority = 0;
  urgent.task.position = {6.0, 1.0, 0.0};
  const InjectResult u = t.gw.inject(urgent, t.swarm.now());
  ASSERT_EQ(u.status, InjectStatus::CREATED) << u.reason;
  t.swarm.run(2.0);
  EXPECT_EQ(t.dog->active()->task, t.id);
  EXPECT_EQ(t.swarm.node(kGateway).agent().winner(u.task_id), kNoAgent);

  AgentState far = t.swarm.node(50).agent().state();
  far.position = {30.0, 0.0, 0.0};   // 離目標 18 m > 15 m
  t.swarm.node(50).setState(far);
  t.swarm.run(3.0);
  EXPECT_FALSE(t.swarm.node(kGateway).neighbors().at(50).committed);
  ASSERT_TRUE(t.dog->active());
  EXPECT_EQ(t.dog->active()->task, u.task_id);
}

TEST(Commit, WithoutRadiusHigherPriorityPreempts)
{
  // 對照：沒有 commit_radius 時，P0 搶占執行中的 P1
  Revisable t;
  ASSERT_TRUE(t.dog->active());
  InjectRequest urgent = groundRequest("fire-10");
  urgent.task.priority = 0;
  urgent.task.position = {6.0, 1.0, 0.0};
  const InjectResult u = t.gw.inject(urgent, t.swarm.now());
  ASSERT_EQ(u.status, InjectStatus::CREATED) << u.reason;
  t.swarm.run(3.0);
  ASSERT_TRUE(t.dog->active());
  EXPECT_EQ(t.dog->active()->task, u.task_id);
  EXPECT_FALSE(t.swarm.node(kGateway).neighbors().at(50).committed);
}

// ===========================================================================
// optional 角色的自動關閉（4e，2026-11-13）
// ===========================================================================
namespace
{
// 一組：必要的地面角色（role 0）＋ optional 的空中角色（role 1）
std::pair<InjectRequest, InjectRequest> groundPlusOptionalAir(const std::string & incident)
{
  InjectRequest ground = groundRequest(incident, "front");
  ground.task.ext.role_count = 2;
  ground.task.ext.role_index = 0;
  InjectRequest air = groundRequest(incident, "overwatch");
  air.task.platform = 1;
  air.task.required_caps = capBit(Capability::MOBILITY_AIR) | capBit(Capability::RGB_CAMERA);
  air.task.position = {12.0, 0.0, 5.0};
  air.task.ext.role_count = 2;
  air.task.ext.role_index = 1;
  air.task.ext.optional = true;
  return {ground, air};
}

struct Dropper
{
  Gateway & gw;
  std::vector<DroppedOptional> drops;
  void tick(double now)
  {
    for (const DroppedOptional & d : gw.step(now)) {
      drops.push_back(d);
    }
  }
};
}  // namespace

TEST(Optional, O1NoCapableVehicleDropsAfterWait)
{
  // 全隊只有狗：空中的 optional 角色沒人能做 → membership_ready 後 10 s 閘道送 reason 4；之前不關
  CommConfig cc;
  cc.epoch = 3;
  Swarm swarm(0.0, 1, cc);
  swarm.addGateway(kGateway);
  swarm.add(50, {5, 0, 0}, 100.0, AgentType::UGV);
  Gateway gw(swarm.node(kGateway));
  Dropper d{gw, {}};
  swarm.onTick([&d](double now) {d.tick(now);});
  swarm.run(2.0);
  EXPECT_FALSE(gw.membershipReady(swarm.now()));   // 剛連上：還沒等滿 lost_timeout ＋ status_period
  swarm.run(1.0);
  EXPECT_TRUE(gw.membershipReady(swarm.now()));
  auto [ground, air] = groundPlusOptionalAir("o1");
  ASSERT_EQ(gw.inject(ground, swarm.now()).status, InjectStatus::CREATED);
  const InjectResult a = gw.inject(air, swarm.now());
  ASSERT_EQ(a.status, InjectStatus::CREATED) << a.reason;
  swarm.run(8.0);
  EXPECT_TRUE(swarm.node(50).taskOpen(a.task_id));
  swarm.run(4.0);
  EXPECT_FALSE(swarm.node(50).taskOpen(a.task_id));
  EXPECT_EQ(swarm.node(50).closeReason(a.task_id), std::optional<std::uint8_t>(wire::kCloseOptionalDropped));
  ASSERT_EQ(d.drops.size(), 1u);
  EXPECT_EQ(d.drops[0].reason, OptionalDrop::NO_CAPABLE);
  EXPECT_EQ(swarm.node(50).stats().rejected_closes, 0);
}

TEST(Optional, O2CapableButUnassignedDropsAfterWait)
{
  // 無人機能做、也得標，但沒有正式指派（這台沒有指派流程）→ 15 s 後略過；必要的地面角色不受影響
  Swarm swarm = makeSwarm();
  Gateway gw(swarm.node(kGateway));
  Dropper d{gw, {}};
  swarm.onTick([&d](double now) {d.tick(now);});
  swarm.run(1.0);
  auto [ground, air] = groundPlusOptionalAir("o2");
  const InjectResult g = gw.inject(ground, swarm.now());
  const InjectResult a = gw.inject(air, swarm.now());
  ASSERT_EQ(a.status, InjectStatus::CREATED) << a.reason;
  swarm.run(10.0);
  EXPECT_TRUE(swarm.node(1).taskOpen(a.task_id));
  EXPECT_EQ(swarm.node(1).agent().winner(a.task_id), 1);
  swarm.run(7.0);
  EXPECT_FALSE(swarm.node(1).taskOpen(a.task_id));
  ASSERT_EQ(d.drops.size(), 1u);
  EXPECT_EQ(d.drops[0].reason, OptionalDrop::UNASSIGNED);
  EXPECT_TRUE(swarm.node(50).taskOpen(g.task_id));   // 必要角色不會被關
}

TEST(Optional, AssignedOptionalIsKept)
{
  // 有正式指派（狗執行中）的 optional 角色：O1、O2 都不關
  Revisable t(0.0, 0.0, true);
  ASSERT_TRUE(t.dog->active());
  Dropper d{t.gw, {}};
  t.swarm.onTick([&](double now) {
      t.dog->setExec(ExecInput{}, now);
      t.dog->step(now);
      t.dog->takeRequests();
      t.dog->takeAssignedChanged();
      d.tick(now);
    });
  t.swarm.run(20.0);
  EXPECT_TRUE(t.swarm.node(50).taskOpen(t.id));
  EXPECT_TRUE(d.drops.empty());
  EXPECT_TRUE(t.dog->active());
}

TEST(Optional, O3GroupDoneDropsAtOnce)
{
  // 必要角色結束（閘道取消）→ 同組的 optional 角色馬上略過（不等 O2 的 15 s）
  Swarm swarm = makeSwarm();
  Gateway gw(swarm.node(kGateway));
  Dropper d{gw, {}};
  swarm.onTick([&d](double now) {d.tick(now);});
  swarm.run(1.0);
  auto [ground, air] = groundPlusOptionalAir("o3");
  const InjectResult g = gw.inject(ground, swarm.now());
  const InjectResult a = gw.inject(air, swarm.now());
  swarm.run(3.0);
  ASSERT_EQ(swarm.node(kGateway).cancelTask(g.task_id, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(1.0);
  EXPECT_FALSE(swarm.node(1).taskOpen(a.task_id));
  EXPECT_EQ(swarm.node(1).closeReason(a.task_id), std::optional<std::uint8_t>(wire::kCloseOptionalDropped));
  ASSERT_EQ(d.drops.size(), 1u);
  EXPECT_EQ(d.drops[0].reason, OptionalDrop::GROUP_DONE);
}

TEST(Optional, O3WithoutGatewayOnlyStopsLocalBidding)
{
  // 閘道離線時必要角色做完：無人機本機不再對 optional 角色出價，但任務不關（沒有 TASK_CLOSE）；閘道回來後才正式略過
  Swarm swarm = makeSwarm();
  Gateway gw(swarm.node(kGateway));
  Dropper d{gw, {}};
  swarm.onTick([&d](double now) {d.tick(now);});
  swarm.run(1.0);
  auto [ground, air] = groundPlusOptionalAir("o3l");
  const InjectResult g = gw.inject(ground, swarm.now());
  const InjectResult a = gw.inject(air, swarm.now());
  swarm.run(3.0);
  swarm.setOnline(kGateway, false);
  ASSERT_EQ(assignAndReport(swarm.node(50), g.task_id, true, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(3.0);
  EXPECT_FALSE(swarm.node(1).taskOpen(g.task_id));
  EXPECT_TRUE(swarm.node(1).taskOpen(a.task_id));
  EXPECT_STREQ(swarm.node(1).agent().bidFor(a.task_id, swarm.now()).why, "excluded");
  EXPECT_EQ(swarm.node(1).stats().optional_local_drops, 1);

  swarm.setOnline(kGateway, true);
  swarm.run(5.0);
  EXPECT_FALSE(swarm.node(1).taskOpen(a.task_id));
  EXPECT_EQ(swarm.node(1).closeReason(a.task_id), std::optional<std::uint8_t>(wire::kCloseOptionalDropped));
}
