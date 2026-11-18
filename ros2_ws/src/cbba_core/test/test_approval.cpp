// 閘道的核准（任務欄位補齊 12，2026-11-18）：核准前不建 CBBA 任務、冪等、核准／拒絕、等核准時過期、時間扣掉等待
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "cbba_core/approval.hpp"
#include "cbba_core/capabilities.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
constexpr AgentId kGateway = 200;

InjectRequest request(const std::string & incident, std::uint16_t revision = 1)
{
  InjectRequest req;
  req.incident_id = incident;
  req.revision = revision;
  req.task.platform = 2;
  req.task.required_caps = capBit(Capability::MOBILITY_GROUND);
  req.task.priority = 0;
  req.task.position = {10.0, 0.0, 0.0};
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
  swarm.add(50, {0, 0, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  return swarm;
}

std::vector<ApprovalState> states(std::vector<std::pair<std::uint32_t, ApprovalState>> changes)
{
  std::vector<ApprovalState> out;
  for (const auto & c : changes) {
    out.push_back(c.second);
  }
  return out;
}
}  // namespace

TEST(Approval, NothingInjectedUntilApproved)
{
  // 12 驗收：要核准的請求先在佇列裡（WAITING_APPROVAL），CBBA 沒有任務；核准 → 注入 → INJECTED，狗得標
  Swarm swarm = makeSwarm();
  Gateway gw(swarm.node(kGateway));
  ApprovalQueue q;
  const auto p = q.propose(request("smoke-1"), {"規劃器要求核准"}, swarm.now());
  EXPECT_FALSE(p.duplicate);
  EXPECT_EQ(states(q.takeChanges()),
    (std::vector<ApprovalState>{ApprovalState::PROPOSED, ApprovalState::WAITING_APPROVAL}));
  swarm.run(3.0);
  EXPECT_EQ(swarm.node(kGateway).taskCount(), 0u);
  EXPECT_EQ(swarm.node(50).taskCount(), 0u);

  std::string error;
  const auto decided = q.decide(p.id, "", true, "operator_a", "確認是煙", swarm.now(), &error);
  ASSERT_EQ(decided.size(), 1u) << error;
  const auto req = q.injectable(p.id, swarm.now());
  ASSERT_TRUE(req);
  const InjectResult r = gw.inject(*req, swarm.now());
  ASSERT_EQ(r.status, InjectStatus::CREATED) << r.reason;
  q.markInjected(p.id, r, swarm.now());
  EXPECT_EQ(states(q.takeChanges()), (std::vector<ApprovalState>{ApprovalState::APPROVED, ApprovalState::INJECTED}));
  EXPECT_EQ(q.find(p.id)->task_id, r.task_id);
  EXPECT_EQ(q.find(p.id)->operator_name, "operator_a");
  swarm.run(2.0);
  EXPECT_EQ(swarm.node(50).agent().winner(r.task_id), 50);
}

TEST(Approval, RejectDuplicateAndOperatorRequired)
{
  // 拒絕的不注入；同一個 (incident, role, revision) 再送回同一筆（被拒絕的要換 revision）；沒有操作者不能決定
  ApprovalQueue q;
  const auto p = q.propose(request("bag-1"), {"高風險"}, 0.0);
  EXPECT_TRUE(q.decide(p.id, "", false, "", "", 1.0).empty());   // 沒有 operator_name
  ASSERT_EQ(q.decide(p.id, "", false, "operator_b", "誤報", 1.0).size(), 1u);
  EXPECT_EQ(q.find(p.id)->state, ApprovalState::REJECTED);
  EXPECT_FALSE(q.injectable(p.id, 1.0));
  const auto again = q.propose(request("bag-1"), {"高風險"}, 2.0);
  EXPECT_TRUE(again.duplicate);
  EXPECT_EQ(again.id, p.id);
  const auto newer = q.propose(request("bag-1", 2), {"高風險"}, 2.0);
  EXPECT_FALSE(newer.duplicate);
  std::string error;
  EXPECT_TRUE(q.decide(p.id, "", true, "operator_b", "", 3.0, &error).empty());   // 已經決定過
  EXPECT_NE(error.find("REJECTED"), std::string::npos) << error;
}

TEST(Approval, DecideWholeIncident)
{
  // id = 0：同一個事件的角色一起核准（規劃器一個事件注入多個角色）
  ApprovalQueue q;
  InjectRequest front = request("intrusion-1");
  front.role = "front";
  InjectRequest rear = request("intrusion-1");
  rear.role = "rear";
  q.propose(front, {"x"}, 0.0);
  q.propose(rear, {"x"}, 0.0);
  q.propose(request("other"), {"x"}, 0.0);
  EXPECT_EQ(q.decide(0, "intrusion-1", true, "op", "", 1.0).size(), 2u);
  EXPECT_EQ(q.waiting().size(), 1u);
}

TEST(Approval, WaitingTimeIsSubtracted)
{
  // 等核准的時間從期限、release 扣掉；等的時候 expires 到了 → EXPIRED；硬期限過了不注入
  ApprovalQueue q;
  InjectRequest req = request("t1");
  req.deadline_sec = 100.0;
  req.release_sec = 30.0;
  req.expires_sec = 300.0;
  const auto p = q.propose(req, {"x"}, 0.0);
  q.decide(p.id, "", true, "op", "", 40.0);
  const auto adjusted = q.injectable(p.id, 40.0);
  ASSERT_TRUE(adjusted);
  EXPECT_DOUBLE_EQ(adjusted->deadline_sec, 60.0);
  EXPECT_DOUBLE_EQ(adjusted->release_sec, 0.0);
  EXPECT_DOUBLE_EQ(adjusted->expires_sec, 260.0);

  InjectRequest hard = request("t2");
  hard.deadline_sec = 20.0;
  hard.task.ext.deadline_hard = true;
  const auto h = q.propose(hard, {"x"}, 0.0);
  q.decide(h.id, "", true, "op", "", 25.0);
  std::string why;
  EXPECT_FALSE(q.injectable(h.id, 25.0, &why));
  EXPECT_NE(why.find("硬期限"), std::string::npos);

  InjectRequest short_lived = request("t3");
  short_lived.expires_sec = 10.0;
  const auto s = q.propose(short_lived, {"x"}, 0.0);
  q.expire(5.0);
  EXPECT_EQ(q.find(s.id)->state, ApprovalState::WAITING_APPROVAL);
  q.expire(10.0);
  EXPECT_EQ(q.find(s.id)->state, ApprovalState::EXPIRED);
}
