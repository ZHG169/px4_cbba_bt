// L6 範本展開器（2026-11-23，docs/l5_l6_plan.md A 批）的單元測試：用 repo 的 registry/ 與 site-config/yard。
// 回歸：perimeter_intrusion、fire_response、single_dog_verify 展開的結果要和 itest group 原本寫死的假 Planner 範本
// （docker/itest/itest.py 的 TEMPLATES、G7 的 verify）一樣：角色、平台、能力、optional、skip_satisfies、位置偏移、
// precedence、distinct、all-or-nothing、sync_window、steps（wait_for、sync、參數）。fallback_template、expires 是新加的
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "swarm_cbba/registry.hpp"
#include "swarm_events/event_rules.hpp"
#include "swarm_events/expander.hpp"

using swarm_events::EventRules;
using swarm_events::ExpandOptions;
using swarm_events::Incident;
using swarm_events::Plan;
using swarm_events::PlannedRole;

namespace
{

struct Fixture
{
  swarm_cbba::Registry registry = swarm_cbba::loadRegistry(REGISTRY_DIR);
  swarm_cbba::Site site = swarm_cbba::loadSite(SITE_DIR);
  EventRules rules = swarm_events::loadEventRules(std::string(SITE_DIR) + "/event_rules.yaml");
};

Fixture & fx()
{
  static Fixture f;
  return f;
}

Incident incident(const std::string & type, double x, double y, const std::string & id = "ev_1")
{
  Incident inc;
  inc.event_id = id;
  inc.event_type = type;
  inc.x = x;
  inc.y = y;
  return inc;
}

// 名稱收 const char *：收 std::string 時呼叫端會產生暫時物件，gcc 14 誤報 -Wdangling-reference
const PlannedRole & role(const Plan & plan, const char * name)
{
  for (const auto & r : plan.roles) {
    if (r.role == name) {
      return r;
    }
  }
  throw std::runtime_error(std::string("沒有角色 ") + name);
}

YAML::Node steps(const PlannedRole & r)
{
  return YAML::Load(r.spec)["steps"];
}

std::string writeRules(const std::string & body)
{
  const std::string path = ::testing::TempDir() + "/event_rules_test.yaml";
  std::ofstream(path) << "schema_version: \"1.0\"\nrules:\n" << body;
  return path;
}

}  // namespace

TEST(EventRules, YardRulesLoadClean)
{
  // yard 的規則：和註冊表、場域一致；event_types.yaml 的每個 default_rule 都有規則
  const auto errors = swarm_events::checkEventRules(fx().rules, fx().registry, &fx().site);
  EXPECT_TRUE(errors.empty()) << errors.front();
  for (const char * name : {"perimeter_intrusion", "fire_response", "single_dog_verify", "smoke_check",
      "thermal_check", "left_object_check"})
  {
    EXPECT_NE(fx().rules.rule(name), nullptr) << name;
  }
}

TEST(EventRules, BadRulesReported)
{
  // 不認得的角色、wait_for 指到不存在的角色、fallback 不存在、不認得的動作 → 都列出來
  const auto path = writeRules(
    "  broken:\n"
    "    fallback_template: nope\n"
    "    roles:\n"
    "      - role: not_a_role\n"
    "        platform: quadruped\n"
    "        steps:\n"
    "          - {action: fly_away}\n"
    "          - {action: inspect, wait_for: [{role: ghost, milestone: arrived}]}\n");
  const auto rules = swarm_events::loadEventRules(path);
  const auto errors = swarm_events::checkEventRules(rules, fx().registry, &fx().site);
  const auto has = [&errors](const std::string & needle) {
      for (const auto & e : errors) {
        if (e.find(needle) != std::string::npos) {
          return true;
        }
      }
      return false;
    };
  EXPECT_TRUE(has("not_a_role"));
  EXPECT_TRUE(has("fallback_template nope"));
  EXPECT_TRUE(has("fly_away"));
  EXPECT_TRUE(has("ghost"));
}

TEST(Expander, IntrusionMatchesItestTemplate)
{
  // 回歸（itest 的 perimeter_intrusion）：空地的事件（Z_YARD 沒有觀察點）→ 位置 = 事件＋偏移
  const Plan plan = swarm_events::expand(incident("perimeter_intrusion", -20.0, 8.0), fx().rules, fx().registry,
      fx().site);
  ASSERT_TRUE(plan.ok) << plan.code << " " << plan.reason;
  ASSERT_EQ(plan.roles.size(), 4u);
  const auto & air = role(plan, "aerial_overwatch");
  const auto & front = role(plan, "front");
  const auto & rear = role(plan, "rear");
  const auto & obs = role(plan, "observer");
  EXPECT_EQ(air.platform_code, 1);
  EXPECT_EQ(front.platform_code, 2);
  EXPECT_EQ(obs.platform_code, 0);
  EXPECT_EQ(air.required_capabilities, (std::vector<std::string>{"mobility_air", "rgb_camera"}));
  EXPECT_EQ(front.required_capabilities, (std::vector<std::string>{"mobility_ground", "rgb_camera"}));
  EXPECT_EQ(obs.required_capabilities, (std::vector<std::string>{"rgb_camera"}));
  EXPECT_TRUE(air.optional && air.skip_satisfies);
  EXPECT_FALSE(front.optional);
  EXPECT_DOUBLE_EQ(front.x, -17.0);
  EXPECT_DOUBLE_EQ(rear.x, -23.0);
  EXPECT_DOUBLE_EQ(obs.y, 2.0);
  EXPECT_EQ(air.precedence, 0);
  EXPECT_EQ(front.precedence, 0b1);
  EXPECT_EQ(rear.precedence, 0b1);
  EXPECT_EQ(obs.precedence, 0);
  for (const auto & r : plan.roles) {
    EXPECT_TRUE(r.all_or_nothing);
    EXPECT_EQ(r.distinct_agent_set, 0b0110);
    EXPECT_EQ(r.sync_window_s, 15);
    EXPECT_EQ(r.priority, 0);
    EXPECT_EQ(r.role_count, 4);
  }
  // steps：front 的 inspect 等 role 0 的 target_localized、會合 front_rear；speak 的範本；go_to 填了觀察點
  const auto fs = steps(front);
  ASSERT_EQ(fs.size(), 3u);
  EXPECT_EQ(fs[0]["action"].as<std::string>(), "go_to");
  EXPECT_DOUBLE_EQ(fs[0]["location"]["x"].as<double>(), -17.0);
  EXPECT_EQ(fs[1]["action"].as<std::string>(), "inspect");
  EXPECT_EQ(fs[1]["object"].as<std::string>(), "person");
  EXPECT_EQ(fs[1]["params"]["duration_s"].as<int>(), 3);
  EXPECT_EQ(fs[1]["wait_for"][0]["role_index"].as<int>(), 0);
  EXPECT_EQ(fs[1]["wait_for"][0]["milestone"].as<std::string>(), "target_localized");
  EXPECT_EQ(fs[1]["sync"].as<std::string>(), "front_rear");
  EXPECT_EQ(fs[2]["params"]["template"].as<std::string>(), "intrusion_warning");
  EXPECT_EQ(YAML::Load(front.spec)["template"].as<std::string>(), "perimeter_intrusion");
  EXPECT_EQ(steps(air)[1]["action"].as<std::string>(), "track");
  EXPECT_EQ(steps(air)[1]["params"]["duration_s"].as<int>(), 5);
}

TEST(Expander, FireMatchesItestTemplate)
{
  // 回歸（itest 的 fire_response）：空中 optional 在事件 +2 m、地面在事件上；report_policy 寫進每個角色
  const Plan plan = swarm_events::expand(incident("fire", 20.0, -25.0), fx().rules, fx().registry, fx().site);
  ASSERT_TRUE(plan.ok) << plan.code << " " << plan.reason;
  ASSERT_EQ(plan.roles.size(), 2u);
  const auto & air = role(plan, "aerial_overwatch");
  const auto & gnd = role(plan, "ground_fire_verify");
  EXPECT_TRUE(air.optional && air.skip_satisfies);
  EXPECT_DOUBLE_EQ(air.y, -23.0);
  EXPECT_DOUBLE_EQ(gnd.x, 20.0);
  EXPECT_EQ(gnd.platform_code, 2);
  EXPECT_FALSE(gnd.all_or_nothing);
  EXPECT_EQ(gnd.distinct_agent_set, 0);
  const auto policy = YAML::Load(gnd.spec)["report_policy"];
  EXPECT_EQ(policy["channels"][0].as<std::string>(), "gcs");
  EXPECT_EQ(policy["channels"][1].as<std::string>(), "operator");
  EXPECT_EQ(policy["required_findings"][0].as<std::string>(), "image");
  EXPECT_EQ(steps(gnd)[1]["object"].as<std::string>(), "fire");
  EXPECT_TRUE(steps(air)[1]["params"]["stream"].as<bool>());
  EXPECT_EQ(gnd.priority, 0);
}

TEST(Expander, SingleDogVerifyIsTheFallback)
{
  // 回歸（itest 的 fallback：G5 → G7 單狗確認）：一個限狗的角色；檢查的對象是事件類型的主體
  ExpandOptions opt;
  opt.rule = "single_dog_verify";
  const Plan fire = swarm_events::expand(incident("fire", 0.0, -25.0), fx().rules, fx().registry, fx().site, opt);
  ASSERT_TRUE(fire.ok) << fire.reason;
  ASSERT_EQ(fire.roles.size(), 1u);
  EXPECT_EQ(fire.roles[0].role, "verify");
  EXPECT_EQ(fire.roles[0].platform_code, 2);
  EXPECT_EQ(fire.roles[0].role_count, 1);
  EXPECT_EQ(steps(fire.roles[0])[1]["object"].as<std::string>(), "fire");
  const Plan intr = swarm_events::expand(incident("perimeter_intrusion", 0.0, -25.0), fx().rules, fx().registry,
      fx().site, opt);
  ASSERT_TRUE(intr.ok) << intr.reason;
  EXPECT_EQ(steps(intr.roles[0])[1]["object"].as<std::string>(), "person");
  // 入侵與火警的規則都把它當降級範本
  EXPECT_EQ(role(swarm_events::expand(incident("fire", 0.0, -25.0), fx().rules, fx().registry, fx().site),
    "ground_fire_verify").fallback_template, "single_dog_verify");
}

TEST(Expander, PoiInIncidentZone)
{
  // 事件在 Z_HV_A 裡：front、rear 用那個區域、狗的觀察點；空中照事件位置（UAV 可以進）
  const Plan plan = swarm_events::expand(incident("perimeter_intrusion", -10.0, 15.0), fx().rules, fx().registry,
      fx().site);
  ASSERT_TRUE(plan.ok) << plan.reason;
  EXPECT_EQ(plan.zone, "Z_HV_A");
  EXPECT_EQ(role(plan, "front").location_source, "poi:OBS_HV_A_FRONT");
  EXPECT_EQ(role(plan, "rear").location_source, "poi:OBS_HV_A_REAR");
  EXPECT_DOUBLE_EQ(role(plan, "front").y, 7.5);
  EXPECT_DOUBLE_EQ(role(plan, "aerial_overwatch").y, 15.0);
}

TEST(Expander, PreflightMovesRestrictedRoleOutOfZone)
{
  // 限狗的角色的觀察點落在禁狗區（Z_HV_B）→ 移到最近的可達點；不限平台的照原位置
  const Plan plan = swarm_events::expand(incident("left_object", -10.0, 0.0), fx().rules, fx().registry, fx().site);
  ASSERT_TRUE(plan.ok) << plan.reason;
  const auto & r = plan.roles.front();
  EXPECT_NE(r.location_source.find("anchor"), std::string::npos) << r.location_source;
  EXPECT_NE(r.location_source.find("Z_HV_B"), std::string::npos);
  EXPECT_TRUE(swarm_events::forbiddenZone(fx().site, "quadruped", r.x, r.y).empty());
  EXPECT_LT(std::hypot(r.x - -7.0, r.y - 0.0), 4.0);   // 最近的可達點：Z_HV_B 只有 6 m 寬
  EXPECT_DOUBLE_EQ(steps(r)[0]["location"]["x"].as<double>(), r.x);   // go_to 填的是移過的點
  // 入侵的 observer（any）在 Z_HV_B 裡照原位置
  const Plan intr = swarm_events::expand(incident("perimeter_intrusion", -20.0, 6.0), fx().rules, fx().registry,
      fx().site);
  ASSERT_TRUE(intr.ok);
  EXPECT_FALSE(swarm_events::forbiddenZone(fx().site, "quadruped", role(intr, "observer").x,
    role(intr, "observer").y).empty());
  EXPECT_EQ(role(intr, "observer").location_source, "incident");
}

TEST(Expander, AssetUsesObservationPose)
{
  // thermal_check 用設備的觀察姿態；yard 的機隊沒有熱像儀 → 機隊可行性不通過
  Incident inc = incident("thermal_anomaly", -20.0, 15.0);
  inc.asset = "machine:TA2";
  const Plan plan = swarm_events::expand(inc, fx().rules, fx().registry, fx().site);
  EXPECT_FALSE(plan.ok);
  EXPECT_EQ(plan.code, "FLEET_INFEASIBLE");
  ASSERT_FALSE(plan.roles.empty());
  EXPECT_EQ(plan.roles.front().location_source, "asset:machine:TA2");
  EXPECT_DOUBLE_EQ(plan.roles.front().y, 7.5);   // 狗的觀察姿態，不是設備本身
  inc.asset = "machine:NOPE";
  EXPECT_EQ(swarm_events::expand(inc, fx().rules, fx().registry, fx().site).code, "BAD_LOCATION");
}

TEST(Expander, FleetFeasibilityUsesAvailableAgents)
{
  // 只有無人機可用：入侵的 front 是必要角色 → FLEET_INFEASIBLE；只有狗可用：空中 optional 只警告
  ExpandOptions only_uav;
  only_uav.available = {1, 2, 3};
  const Plan a = swarm_events::expand(incident("perimeter_intrusion", -20.0, 8.0), fx().rules, fx().registry,
      fx().site, only_uav);
  EXPECT_FALSE(a.ok);
  EXPECT_EQ(a.code, "FLEET_INFEASIBLE");
  ExpandOptions only_dogs;
  only_dogs.available = {50, 51, 52};
  const Plan b = swarm_events::expand(incident("perimeter_intrusion", -20.0, 8.0), fx().rules, fx().registry,
      fx().site, only_dogs);
  ASSERT_TRUE(b.ok) << b.reason;
  bool warned = false;
  for (const auto & w : b.warnings) {
    warned = warned || (w.find("aerial_overwatch") != std::string::npos && w.find("NO_CAPABLE") != std::string::npos);
  }
  EXPECT_TRUE(warned);
}

TEST(Expander, EnduranceWarning)
{
  // 作業時間超過 UAV 的續航（900 s）→ 只警告，不擋
  const auto path = writeRules(
    "  long_watch:\n"
    "    roles:\n"
    "      - role: aerial_overwatch\n"
    "        platform: uav\n"
    "        required_capabilities: [mobility_air, rgb_camera]\n"
    "        service_time_s: 2000\n"
    "        steps:\n"
    "          - {action: go_to}\n");
  const auto rules = swarm_events::loadEventRules(path);
  ExpandOptions opt;
  opt.rule = "long_watch";
  const Plan plan = swarm_events::expand(incident("fire", 0.0, 0.0), rules, fx().registry, fx().site, opt);
  ASSERT_TRUE(plan.ok) << plan.reason;
  bool warned = false;
  for (const auto & w : plan.warnings) {
    warned = warned || w.find("ENDURANCE") != std::string::npos;
  }
  EXPECT_TRUE(warned);
}

TEST(Expander, RevisionChangesOnlyMovedRoles)
{
  // 回歸（itest G6）：入侵者移動 8 m → front、rear 改版；空中（追蹤，retarget false）、observer（retarget false）不改。
  // 小位移（1 m）不重新拍賣
  Incident inc = incident("perimeter_intrusion", -20.0, 8.0);
  const Plan before = swarm_events::expand(inc, fx().rules, fx().registry, fx().site);
  inc.x += 8.0;
  inc.revision = 1;
  const Plan moved = swarm_events::expand(inc, fx().rules, fx().registry, fx().site);
  ASSERT_TRUE(before.ok && moved.ok);
  EXPECT_EQ(swarm_events::changedRoles(before, moved), (std::vector<std::string>{"front", "rear"}));
  inc.x = -19.0;
  const Plan small = swarm_events::expand(inc, fx().rules, fx().registry, fx().site);
  EXPECT_TRUE(swarm_events::changedRoles(before, small).empty());
}

TEST(Expander, UnknownTypeAndRule)
{
  EXPECT_EQ(swarm_events::expand(incident("no_such_event", 0, 0), fx().rules, fx().registry, fx().site).code,
    "UNKNOWN_EVENT_TYPE");
  ExpandOptions opt;
  opt.rule = "no_such_rule";
  EXPECT_EQ(swarm_events::expand(incident("fire", 0, 0), fx().rules, fx().registry, fx().site, opt).code,
    "UNKNOWN_RULE");
}

TEST(Expander, PlanJsonParses)
{
  // 整組的 JSON（給日誌、控制中心）可以解析，角色數一樣
  const Plan plan = swarm_events::expand(incident("perimeter_intrusion", -20.0, 8.0), fx().rules, fx().registry,
      fx().site);
  ASSERT_TRUE(plan.ok);
  const auto doc = YAML::Load(plan.json);
  EXPECT_EQ(doc["roles"].size(), 4u);
  EXPECT_EQ(doc["rule"].as<std::string>(), "perimeter_intrusion");
  EXPECT_EQ(doc["coupling"]["distinct_agent_set"].as<int>(), 6);
}
