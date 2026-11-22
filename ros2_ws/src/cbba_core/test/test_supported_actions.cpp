// 執行端支援的動作（2026-11-19，docs/execution_backend_plan.md「not_started：不全域放行」）：
// 這台能做的動作（cbba_node 由註冊表推導後交給協定層）；TaskSpec 有不在清單上的動作 → 不出價（unsupported_action），交給支援的那台；
// 沒宣告（nullopt）不限制；鄰居的能力統計不受本機旗標影響
#include <gtest/gtest.h>

#include <regex>
#include <set>
#include <string>
#include <vector>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/gateway.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
constexpr AgentId kGateway = 200;

// 測試用的 spec → 動作（cbba_node 用 yaml-cpp 解析；這裡用 regex 就夠）
std::vector<std::string> listActions(const std::string & spec)
{
  std::vector<std::string> out;
  const std::regex re("\"action\"\\s*:\\s*\"([a-z_]+)\"");
  for (auto it = std::sregex_iterator(spec.begin(), spec.end(), re); it != std::sregex_iterator(); ++it) {
    out.push_back((*it)[1]);
  }
  return out;
}

struct Fixture
{
  Swarm swarm;
  TaskId id{0};

  explicit Fixture(const std::string & spec)
  : swarm(0.0, 1, [] {CommConfig c; c.epoch = 3; return c;}())
  {
    swarm.addGateway(kGateway);
    swarm.add(50, {5, 0, 0}, 100.0, AgentType::UGV);    // 近
    swarm.add(51, {30, 0, 0}, 100.0, AgentType::UGV);   // 遠
    for (AgentId a : {AgentId{50}, AgentId{51}}) {
      swarm.node(a).setActionLister(listActions);
    }
    swarm.run(2.0);
    Gateway gw(swarm.node(kGateway));
    InjectRequest req;
    req.incident_id = "sup-1";
    req.revision = 1;
    req.task.platform = 2;
    req.task.required_caps = capBit(Capability::MOBILITY_GROUND);
    req.task.priority = 1;
    req.task.position = {10.0, 0.0, 0.0};
    req.task.value = 100.0;
    req.task.duration_sec = 5.0;
    req.spec = spec;
    const InjectResult r = gw.inject(req, swarm.now());
    EXPECT_EQ(r.status, InjectStatus::CREATED) << r.reason;
    id = r.task_id;
  }
};

const char * kSpeakSpec = R"({"steps": [{"action": "go_to"}, {"action": "speak", "params": {"template": "x"}}]})";
}  // namespace

TEST(SupportedActions, UnsupportedAgentDoesNotBid)
{
  // 近的 dog 50 只宣告 go_to、inspect（沒有 speak）→ 不出價（unsupported_action）；遠的 dog 51 支援 → 它接
  Fixture f(kSpeakSpec);
  f.swarm.node(50).setSupportedActions(std::set<std::string>{"go_to", "inspect"}, f.swarm.now());
  f.swarm.node(51).setSupportedActions(std::set<std::string>{"go_to", "inspect", "speak"}, f.swarm.now());
  f.swarm.run(4.0);
  EXPECT_EQ(f.swarm.node(50).agent().winner(f.id), 51);
  EXPECT_EQ(f.swarm.node(50).agent().bidFor(f.id, f.swarm.now()).why, std::string("unsupported_action"));
  // 閘道看「能做的載具」時不受本機旗標影響（不知道鄰居支不支援）
  EXPECT_EQ(f.swarm.node(kGateway).capableAgents(f.swarm.node(kGateway).agent().tasks().at(f.id),
    f.swarm.now()).size(), 2u);
}

TEST(SupportedActions, HolderReleasesWhenSupportIsWithdrawn)
{
  // 一開始沒宣告（不限制）：近的 dog 50 得標；之後它宣告不支援 speak → 釋放、dog 51 接手。宣告回來 → 又可以出價
  Fixture f(kSpeakSpec);
  f.swarm.run(4.0);
  ASSERT_EQ(f.swarm.node(51).agent().winner(f.id), 50);
  f.swarm.node(50).setSupportedActions(std::set<std::string>{"go_to"}, f.swarm.now());
  f.swarm.run(3.0);
  EXPECT_EQ(f.swarm.node(51).agent().winner(f.id), 51);
  EXPECT_TRUE(f.swarm.consistent());
  f.swarm.node(50).setSupportedActions(std::nullopt, f.swarm.now());
  f.swarm.run(3.0);
  EXPECT_NE(std::string(f.swarm.node(50).agent().bidFor(f.id, f.swarm.now()).why), "unsupported_action");
}

TEST(SupportedActions, TaskWithoutSpecIsNotBlocked)
{
  // 沒有 TaskSpec 的任務（舊型、規劃器沒給 steps）：不擋，照常出價
  Fixture f("");
  f.swarm.node(50).setSupportedActions(std::set<std::string>{}, f.swarm.now());
  f.swarm.run(4.0);
  EXPECT_EQ(f.swarm.node(51).agent().winner(f.id), 50);
}
