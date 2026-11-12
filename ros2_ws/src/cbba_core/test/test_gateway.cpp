// 閘道的任務注入（2-1b M0 第 4a 步）：冪等、欄位檢查、task_id／group_id、經過模擬網路傳到全隊
#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

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

  req.revision = 3;
  EXPECT_EQ(gw.inject(req, swarm.now()).status, InjectStatus::REJECTED);
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
