// L2 累積、L3 關聯、L5 事件狀態機（2026-11-23，docs/l5_l6_plan.md B 批）的單元測試：用 repo 的 registry/、site-config/yard。
// 對照 2-1 第五節階段 1 的案例：2（同一事件連續告警只有 1 個事件）、3（中信心只派 1 隻確認）、10（無人可得標 → 升級、fallback）
#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "swarm_cbba/registry.hpp"
#include "swarm_events/event_rules.hpp"
#include "swarm_events/incident_manager.hpp"

using swarm_events::Action;
using swarm_events::DetectionIn;
using swarm_events::EventState;
using swarm_events::IncidentManager;
using swarm_events::ManagerConfig;
using swarm_events::RoleState;

namespace
{

struct Fixture
{
  swarm_cbba::Registry registry = swarm_cbba::loadRegistry(REGISTRY_DIR);
  swarm_cbba::Site site = swarm_cbba::loadSite(SITE_DIR);
  swarm_events::EventRules rules = swarm_events::loadEventRules(std::string(SITE_DIR) + "/event_rules.yaml");
};

Fixture & fx()
{
  static Fixture f;
  return f;
}

ManagerConfig config(int hour = 22, bool vlm = false)
{
  ManagerConfig c;
  c.hour_of = [hour](double) {return hour;};
  c.vlm_enabled = vlm;
  return c;
}

DetectionIn fire(double x, double y, double score = 0.9, std::uint32_t track = 0)
{
  DetectionIn d;
  d.agent_id = 3;
  d.detector = "apriltag_fire";
  d.label = "fire";
  d.source = "robot_camera:uav3";
  d.score = score;
  d.track_id = track;
  d.position_valid = true;
  d.x = x;
  d.y = y;
  return d;
}

DetectionIn person(double x, double y, double score = 0.9)
{
  DetectionIn d;
  d.agent_id = 50;
  d.detector = "person_yolo_v1";
  d.label = "person";
  d.source = "robot_camera:dog1";
  d.score = score;
  d.position_valid = true;
  d.x = x;
  d.y = y;
  return d;
}

bool has(const std::vector<Action> & acts, Action::Kind kind, const std::string & rule = "")
{
  return std::any_of(acts.begin(), acts.end(), [&](const Action & a) {
             return a.kind == kind && (rule.empty() || a.rule == rule);
           });
}

std::vector<Action> join(std::vector<Action> a, const std::vector<Action> & b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

// 火警：兩筆偵測 → s ≥ theta_on（2.0）持續 1 s → CONFIRMED → 派 fire_response；回傳事件 id
std::string confirmedFire(IncidentManager & m, double x = 0.0, double y = -25.0)
{
  m.onDetection(fire(x, y), 0.0);
  m.onDetection(fire(x, y), 0.8);
  m.step(1.1);
  return m.events().begin()->first;
}

}  // namespace

TEST(Incident, FireConfirmsAfterMinDuration)
{
  // 第一筆就 ≥ theta_on，但要持續 min_duration_s（1 s）才決定；信心高 → CONFIRMED → 派預設範本
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  EXPECT_TRUE(m.onDetection(fire(0.0, -25.0), 0.0).empty());
  ASSERT_EQ(m.events().size(), 1u);
  EXPECT_EQ(m.events().begin()->second.state, EventState::CANDIDATE);
  EXPECT_TRUE(m.step(0.5).empty());
  m.onDetection(fire(0.0, -25.0), 0.8);
  const auto acts = m.step(1.1);
  EXPECT_TRUE(has(acts, Action::Kind::DISPATCH, "fire_response"));
  EXPECT_TRUE(has(acts, Action::Kind::NOTIFY));
  const auto & rec = m.events().begin()->second;
  EXPECT_EQ(rec.state, EventState::DISPATCHED);
  EXPECT_GT(rec.confidence, 0.8);
  EXPECT_EQ(rec.plan_version, 1);
}

TEST(Incident, RepeatedAlarmsMergeIntoOneEvent)
{
  // 2-1 案例 2：同一處 5 則告警只有 1 個事件；10 m 外是另一件事；同一個 track 就算遠也是同一件
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  for (int i = 0; i < 5; ++i) {
    m.onDetection(fire(0.0 + 0.2 * i, -25.0, 0.9, 101), 0.5 * i);
  }
  ASSERT_EQ(m.events().size(), 1u);
  EXPECT_EQ(m.events().begin()->second.detections, 5u);
  m.onDetection(fire(10.0, -25.0), 2.5);
  EXPECT_EQ(m.events().size(), 2u);
  m.onDetection(fire(20.0, -25.0, 0.9, 101), 3.0);
  EXPECT_EQ(m.events().size(), 2u);
  EXPECT_DOUBLE_EQ(m.events().begin()->second.x, 20.0);   // 位置跟著最新的偵測
}

TEST(Incident, TriggerFiltersZoneAndTime)
{
  // 入侵：要在 restricted 區域、要在下班時段（trigger zone_attr restricted、time_rule off_hours）
  IncidentManager night(fx().registry, fx().site, fx().rules, config(22));
  night.onDetection(person(0.0, -25.0), 0.0);    // Z_YARD：不是 restricted
  EXPECT_TRUE(night.events().empty());
  night.onDetection(person(-10.0, 15.0), 0.0);   // Z_HV_A：restricted
  EXPECT_EQ(night.events().size(), 1u);
  IncidentManager day(fx().registry, fx().site, fx().rules, config(10));
  day.onDetection(person(-10.0, 15.0), 0.0);
  EXPECT_TRUE(day.events().empty());
  // 偵測器不認得、分數低於 min_score、label 不對 → 不算
  DetectionIn low = fire(0.0, -25.0, 0.5);
  night.onDetection(low, 0.0);
  DetectionIn wrong = fire(0.0, -25.0);
  wrong.label = "smoke";
  night.onDetection(wrong, 0.0);
  EXPECT_EQ(night.events().size(), 1u);
}

TEST(Incident, WeakEvidenceFadesAway)
{
  // 遲滯：一筆 0.86（logit 1.8 < theta_on 2.0）之後沒有新的 → 每秒乘 λ 遺忘 → < theta_off 0.5 → 取消
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  m.onDetection(fire(0.0, -25.0, 0.86), 0.0);
  m.step(3.0);
  EXPECT_EQ(m.events().begin()->second.state, EventState::CANDIDATE);
  m.step(10.0);
  EXPECT_EQ(m.events().begin()->second.state, EventState::REJECTED);
  const auto log = m.takeLog();
  EXPECT_NE(log.back().find("證據消退"), std::string::npos) << log.back();
}

TEST(Incident, VlmMediumConfidenceSendsOneDog)
{
  // 2-1 案例 3：VLM 判讀中信心 → NEEDS_CHECK → 只派 1 隻確認（入侵範本的 fallback_template single_dog_verify）
  IncidentManager m(fx().registry, fx().site, fx().rules, config(22, true));
  m.onDetection(person(-10.0, 15.0), 0.0);
  m.onDetection(person(-10.0, 15.0), 0.5);
  const auto req = m.step(2.1);
  ASSERT_TRUE(has(req, Action::Kind::VLM_REQUEST));
  const std::string id = m.events().begin()->first;
  EXPECT_EQ(m.find(id)->state, EventState::VERIFYING);
  const auto acts = m.onVlmResult(id, "confirmed", 0.5, 2.5);
  EXPECT_TRUE(has(acts, Action::Kind::DISPATCH, "single_dog_verify"));
  EXPECT_EQ(m.find(id)->rule, "single_dog_verify");
  // 之後證據變強（L2 的信心 ≥ theta_high）→ 升級成完整範本
  const auto up = join(m.onDetection(person(-10.0, 15.0), 3.0), m.onDetection(person(-10.0, 15.0), 3.5));
  EXPECT_TRUE(has(up, Action::Kind::DISPATCH, "perimeter_intrusion"));
  EXPECT_EQ(m.find(id)->rule, "perimeter_intrusion");
}

TEST(Incident, VlmRejectsOrConfirms)
{
  IncidentManager m(fx().registry, fx().site, fx().rules, config(22, true));
  m.onDetection(person(-10.0, 15.0), 0.0);
  m.onDetection(person(-10.0, 15.0), 0.5);
  m.step(2.1);
  const std::string a = m.events().begin()->first;
  m.onVlmResult(a, "rejected", 0.9, 2.5);
  EXPECT_EQ(m.find(a)->state, EventState::REJECTED);
  // 另一件（20 m 外、還在 Z_HV_A）：VLM 高信心 → 完整範本
  m.onDetection(person(10.0, 15.0), 3.0);
  m.onDetection(person(10.0, 15.0), 3.5);
  m.step(5.1);
  std::string b;
  for (const auto & [id, r] : m.events()) {
    if (id != a) {
      b = id;
    }
  }
  ASSERT_FALSE(b.empty());
  EXPECT_TRUE(has(m.onVlmResult(b, "confirmed", 0.95, 5.5), Action::Kind::DISPATCH, "perimeter_intrusion"));
}

TEST(Incident, VlmTimeoutDegradesByPriority)
{
  // P0（入侵）：VLM 逾時直接用 L2 的信心決定
  IncidentManager m(fx().registry, fx().site, fx().rules, config(22, true));
  m.onDetection(person(-10.0, 15.0), 0.0);
  m.onDetection(person(-10.0, 15.0), 0.5);
  m.step(2.1);
  EXPECT_TRUE(has(m.step(7.2), Action::Kind::DISPATCH, "perimeter_intrusion"));
  // P2（遺留物）：逾時先再送一次，還是逾時才用 L2
  IncidentManager l(fx().registry, fx().site, fx().rules, config(22, true));
  DetectionIn bag;
  bag.detector = "left_object_yolo";
  bag.label = "bag";
  bag.source = "fixed_camera:CAM-01";
  bag.score = 0.95;
  bag.position_valid = true;
  bag.x = 0.0;
  bag.y = -25.0;
  std::vector<Action> seen;
  for (int t = 0; t <= 32; ++t) {
    seen = join(seen, l.onDetection(bag, t));   // s ≥ theta_on 5 從 t = 1 起、持續 30 s → t = 31 送 VLM
  }
  ASSERT_TRUE(has(seen, Action::Kind::VLM_REQUEST));
  EXPECT_TRUE(has(l.step(38.0), Action::Kind::VLM_REQUEST));   // 第一次逾時（31 + 5 s）：再送
  EXPECT_TRUE(has(l.step(44.0), Action::Kind::DISPATCH, "left_object_check"));
}

TEST(Incident, LifecycleResolvedAndFalseAlarm)
{
  // 派遣 → 角色到場 ON_SITE → 必要角色完成 RESOLVED（optional 的空中角色沒做完也算）
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  const std::string id = confirmedFire(m);
  m.onDispatchResult(id, true, {{"aerial_overwatch", true}, {"ground_fire_verify", false}}, "", 1.2);
  m.onRoleState(id, "ground_fire_verify", RoleState::ARRIVED, "", 5.0);
  EXPECT_EQ(m.find(id)->state, EventState::ON_SITE);
  m.onRoleState(id, "ground_fire_verify", RoleState::DONE, "", 9.0);
  EXPECT_EQ(m.find(id)->state, EventState::RESOLVED);
  // 確認角色到場沒看到目標（E_TARGET_LOST）→ FALSE_ALARM
  IncidentManager f(fx().registry, fx().site, fx().rules, config());
  const std::string id2 = confirmedFire(f);
  f.onDispatchResult(id2, true, {{"ground_fire_verify", false}}, "", 1.2);
  f.onRoleState(id2, "ground_fire_verify", RoleState::FAILED, "E_TARGET_LOST", 6.0);
  EXPECT_EQ(f.find(id2)->state, EventState::FALSE_ALARM);
  // 值班也能結案
  IncidentManager o(fx().registry, fx().site, fx().rules, config());
  const std::string id3 = confirmedFire(o);
  o.resolve(id3, true, "op_test", 3.0);
  EXPECT_EQ(o.find(id3)->state, EventState::FALSE_ALARM);
}

TEST(Incident, IncidentUpdateReplans)
{
  // 閘道接受 UAV 的更新（revision 1）→ REPLAN（帶 revision、plan_version + 1）；同一個 revision 再來不重複
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  const std::string id = confirmedFire(m);
  const auto acts = m.onIncidentUpdate(id, 1, true, 6.0, -25.0, 3.0);
  ASSERT_EQ(acts.size(), 1u);
  EXPECT_EQ(acts[0].kind, Action::Kind::REPLAN);
  EXPECT_EQ(acts[0].incident_revision, 1);
  EXPECT_EQ(acts[0].plan_version, 2);
  EXPECT_DOUBLE_EQ(m.find(id)->x, 6.0);
  EXPECT_TRUE(m.onIncidentUpdate(id, 1, true, 7.0, -25.0, 4.0).empty());
}

TEST(Incident, DetectionsMovingTheTargetReplan)
{
  // 已派遣的事件，同一個 track 的偵測移動超過 retarget_m（3 m）→ REPLAN；小位移不重新展開
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  m.onDetection(fire(0.0, -25.0, 0.9, 7), 0.0);
  m.onDetection(fire(0.0, -25.0, 0.9, 7), 0.8);
  m.step(1.1);
  EXPECT_FALSE(has(m.onDetection(fire(1.0, -25.0, 0.9, 7), 2.0), Action::Kind::REPLAN));
  EXPECT_TRUE(has(m.onDetection(fire(5.0, -25.0, 0.9, 7), 3.0), Action::Kind::REPLAN, "fire_response"));
}

TEST(Incident, EscalationDispatchesFallbackChild)
{
  // 2-1 案例 10：P0 沒人接 → ESCALATED，照範本的 fallback_template 派子事件；子事件結案 → 原本的事件跟著結案
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  const std::string id = confirmedFire(m);
  const auto acts = m.onEscalate(id, "P0 10 s 沒人得標", "", 12.0);
  EXPECT_EQ(m.find(id)->state, EventState::ESCALATED);
  ASSERT_TRUE(has(acts, Action::Kind::DISPATCH, "single_dog_verify"));
  const std::string child = id + "-fb1";
  ASSERT_NE(m.find(child), nullptr);
  EXPECT_EQ(m.find(child)->parent, id);
  m.onDispatchResult(child, true, {{"verify", false}}, "", 12.5);
  m.onRoleState(child, "verify", RoleState::DONE, "", 20.0);
  EXPECT_EQ(m.find(child)->state, EventState::RESOLVED);
  EXPECT_EQ(m.find(id)->state, EventState::RESOLVED);
}

TEST(Incident, DispatchFailureEscalates)
{
  // 展開失敗（例如 FLEET_INFEASIBLE）→ ESCALATED、通知值班
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  const std::string id = confirmedFire(m);
  const auto acts = m.onDispatchResult(id, false, {}, "FLEET_INFEASIBLE", 1.2);
  EXPECT_EQ(m.find(id)->state, EventState::ESCALATED);
  EXPECT_TRUE(has(acts, Action::Kind::NOTIFY));
}

TEST(Incident, ExpiresAfterExpiresS)
{
  // fire 的 expires_s 900：決定之後 900 s 還沒結案 → EXPIRED
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  const std::string id = confirmedFire(m);
  m.step(800.0);
  EXPECT_EQ(m.find(id)->state, EventState::DISPATCHED);
  m.step(902.0);
  EXPECT_EQ(m.find(id)->state, EventState::EXPIRED);
}

TEST(Incident, SaveAndLoadKeepOpenEvents)
{
  // 存檔讀回：開著的事件、角色、編號計數都還在；結案的不存
  IncidentManager m(fx().registry, fx().site, fx().rules, config());
  const std::string id = confirmedFire(m);
  m.onDispatchResult(id, true, {{"aerial_overwatch", true}, {"ground_fire_verify", false}}, "", 1.2);
  m.onRoleState(id, "ground_fire_verify", RoleState::ARRIVED, "", 5.0);
  const std::string text = m.save();
  IncidentManager back(fx().registry, fx().site, fx().rules, config());
  back.load(text);
  ASSERT_NE(back.find(id), nullptr);
  EXPECT_EQ(back.find(id)->state, EventState::ON_SITE);
  EXPECT_EQ(back.find(id)->rule, "fire_response");
  EXPECT_EQ(back.find(id)->roles.at("ground_fire_verify").state, RoleState::ARRIVED);
  EXPECT_TRUE(back.find(id)->roles.at("aerial_overwatch").optional);
  back.onDetection(fire(30.0, -25.0), 10.0);
  EXPECT_EQ(back.events().size(), 2u);   // 新的事件編號不和讀回的重複
  back.onRoleState(id, "ground_fire_verify", RoleState::DONE, "", 11.0);
  EXPECT_EQ(back.find(id)->state, EventState::RESOLVED);
}
