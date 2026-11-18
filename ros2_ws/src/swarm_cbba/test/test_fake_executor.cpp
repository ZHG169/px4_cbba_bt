// 假 adapter 的模擬（ExecuteStep.action 草案，2026-11-19，fake_executor.hpp）：檢查和閘道同一套、移動與停留、UAV 的平台狀態機
//（自動起飛、降落）、取消、逾時、故障注入、場域的 location。讀 repo 的 registry/、site-config/yard
#include <gtest/gtest.h>

#include <string>

#include "swarm_cbba/fake_executor.hpp"
#include "swarm_cbba/registry.hpp"

using namespace swarm_cbba;

namespace
{
const Registry & registry()
{
  static const Registry r = loadRegistry(REGISTRY_DIR);
  return r;
}

const Site & site()
{
  static const Site s = loadSite(SITE_DIR);
  return s;
}

FakeExecutorConfig config(const std::string & platform)
{
  FakeExecutorConfig c;
  c.platform = platform;
  c.speed = 1.0;
  c.cruise_altitude = 3.0;
  c.home = {{0.0, 0.0, 0.0}};
  return c;
}

StepGoal goal(const std::string & action, const std::string & location = "", const std::string & params = "",
  const std::string & object = "")
{
  StepGoal g;
  g.task_id = 7;
  g.step_id = 1;
  g.action = action;
  g.location_json = location;
  g.params_json = params;
  g.object = object;
  return g;
}

// 每 0.1 s 推一次，直到結束或超過 limit 秒；回傳最後的快照，t 停在結束的時刻
StepSnapshot runUntilDone(FakeExecutor & ex, double & t, double limit)
{
  StepSnapshot s = ex.step(t);
  const double end = t + limit;
  while (s.status == StepStatus::RUNNING && t < end) {
    t += 0.1;
    s = ex.step(t);
  }
  return s;
}
}  // namespace

TEST(FakeExecutor, GoToMovesWithProgressAndSucceeds)
{
  // 狗 1 m/s，到 (5, 0)：中途進度約一半、phase moving，約 5 s 成功，停在目標
  FakeExecutor ex(registry(), nullptr, config("quadruped"));
  double t = 100.0;
  ASSERT_EQ(ex.start(goal("go_to", R"({"x": 5.0, "y": 0.0})"), t).status, StepStatus::RUNNING);
  t += 2.5;
  const StepSnapshot mid = ex.step(t);
  EXPECT_EQ(mid.status, StepStatus::RUNNING);
  EXPECT_EQ(mid.phase, "moving");
  EXPECT_NEAR(mid.progress, 0.5, 0.05);
  const StepSnapshot end = runUntilDone(ex, t, 10.0);
  EXPECT_EQ(end.status, StepStatus::SUCCEEDED);
  EXPECT_NEAR(t, 105.0, 0.15);
  EXPECT_NEAR(end.position[0], 5.0, 1e-6);
  EXPECT_DOUBLE_EQ(end.progress, 1.0);
  EXPECT_EQ(end.platform, PlatformState::READY);
}

TEST(FakeExecutor, StandoffStopsShortOfTheLocation)
{
  FakeExecutor ex(registry(), nullptr, config("quadruped"));
  double t = 0.0;
  ex.start(goal("go_to", R"({"x": 10.0, "y": 0.0})", R"({"standoff_m": 3.0})"), t);
  const StepSnapshot end = runUntilDone(ex, t, 20.0);
  EXPECT_EQ(end.status, StepStatus::SUCCEEDED);
  EXPECT_NEAR(end.position[0], 7.0, 1e-6);
}

TEST(FakeExecutor, UavTakesOffAutomaticallyAndLandsAtDock)
{
  // UAV 在地上（LANDED）：go_to 先 takeoff（1.5 m）→ climbing（巡航 3 m）→ moving，之後 AIRBORNE；
  // return_to_dock：回起點上空 → landing → LANDED
  FakeExecutor ex(registry(), nullptr, config("uav"));
  EXPECT_EQ(ex.platformState(), PlatformState::LANDED);
  double t = 0.0;
  ASSERT_EQ(ex.start(goal("go_to", R"({"x": 4.0, "y": 0.0})"), t).status, StepStatus::RUNNING);
  t += 1.0;
  EXPECT_EQ(ex.step(t).phase, "takeoff");
  t += 1.0;
  EXPECT_EQ(ex.step(t).phase, "climbing");
  EXPECT_EQ(ex.platformState(), PlatformState::AIRBORNE);
  StepSnapshot s = runUntilDone(ex, t, 20.0);
  EXPECT_EQ(s.status, StepStatus::SUCCEEDED);
  EXPECT_NEAR(s.position[0], 4.0, 1e-6);
  EXPECT_NEAR(s.position[2], 3.0, 1e-6);
  EXPECT_EQ(s.platform, PlatformState::AIRBORNE);

  ASSERT_EQ(ex.start(goal("return_to_dock"), t).status, StepStatus::RUNNING);
  s = runUntilDone(ex, t, 20.0);
  EXPECT_EQ(s.status, StepStatus::SUCCEEDED);
  EXPECT_NEAR(s.position[0], 0.0, 1e-6);
  EXPECT_NEAR(s.position[2], 0.0, 1e-6);
  EXPECT_EQ(s.platform, PlatformState::LANDED);
}

TEST(FakeExecutor, InvalidStepsFailWithPrecondition)
{
  // 和閘道同一套檢查：不認得的動作、參數超出範圍、缺 required 參數、這個平台沒有實作 → FAILED、E_PRECONDITION
  FakeExecutor dog(registry(), nullptr, config("quadruped"));
  FakeExecutor uav(registry(), nullptr, config("uav"));
  const auto failed = [](const StepSnapshot & s, const std::string & code) {
      return s.status == StepStatus::FAILED && s.failure_code == "E_PRECONDITION" &&
             s.detail.find(code) != std::string::npos;
    };
  EXPECT_TRUE(failed(dog.start(goal("fly_away"), 0.0), "UNKNOWN_ACTION"));
  EXPECT_TRUE(failed(dog.start(goal("inspect", "", R"({"duration_s": 1000})"), 0.0), "PARAM_OUT_OF_RANGE"));
  EXPECT_TRUE(failed(dog.start(goal("wait"), 0.0), "PARAM_MISSING"));
  EXPECT_TRUE(failed(dog.start(goal("go_to"), 0.0), "LOCATION_MISSING"));
  EXPECT_TRUE(failed(uav.start(goal("speak", "", R"({"template": "hello"})"), 0.0), "PLATFORM_NOT_SUPPORTED"));
  EXPECT_TRUE(failed(uav.start(goal("emergency_stop"), 0.0), "PLATFORM_NOT_SUPPORTED"));
  EXPECT_FALSE(dog.busy());
  // 不認得的參數只警告（狗 BT 的舊名稱 snapshots）：照樣執行，警告寫在 detail
  const StepSnapshot s = dog.start(goal("inspect", "", R"({"snapshots": 2})"), 0.0);
  EXPECT_EQ(s.status, StepStatus::RUNNING);
  EXPECT_NE(s.detail.find("PARAM_UNKNOWN"), std::string::npos);
}

TEST(FakeExecutor, NotStartedImplementationIsSimulatedWithWarning)
{
  // patrol 在 actions.yaml 兩個平台都是 not_started：閘道會擋，假 adapter 照樣模擬（附警告）；關掉 allow_not_started 就擋
  FakeExecutor ex(registry(), nullptr, config("quadruped"));
  double t = 0.0;
  StepSnapshot s = ex.start(goal("patrol", "", R"({"waypoints": [[2, 0], [2, 2]], "loops": 2})"), t);
  ASSERT_EQ(s.status, StepStatus::RUNNING);
  EXPECT_NE(s.detail.find("not_started"), std::string::npos);
  s = runUntilDone(ex, t, 30.0);
  EXPECT_EQ(s.status, StepStatus::SUCCEEDED);
  EXPECT_NEAR(t, 8.0, 0.3);   // (0,0)→(2,0)→(2,2)→(2,0)→(2,2)：4 段各 2 m
  FakeExecutorConfig strict = config("quadruped");
  strict.allow_not_started = false;
  FakeExecutor ex2(registry(), nullptr, strict);
  s = ex2.start(goal("patrol", "", R"({"waypoints": [[2, 0]]})"), 0.0);
  EXPECT_EQ(s.status, StepStatus::FAILED);
  EXPECT_NE(s.detail.find("PLATFORM_NOT_SUPPORTED"), std::string::npos);
}

TEST(FakeExecutor, CancelStopsInPlace)
{
  FakeExecutor ex(registry(), nullptr, config("quadruped"));
  double t = 0.0;
  ex.start(goal("go_to", R"({"x": 10.0, "y": 0.0})"), t);
  t += 2.0;
  ex.step(t);
  const StepSnapshot s = ex.cancel("BT 取消");
  EXPECT_EQ(s.status, StepStatus::CANCELED);
  EXPECT_TRUE(s.failure_code.empty());
  EXPECT_NEAR(s.position[0], 2.0, 1e-6);
  EXPECT_FALSE(ex.busy());
  EXPECT_EQ(ex.step(t + 5.0).status, StepStatus::CANCELED);   // 結束後一直回傳結束的那一份
}

TEST(FakeExecutor, SlowdownHitsTheTimeout)
{
  // go_to 10 m：估計 10 s，逾時 = 10 × 1.5 + 10 = 25 s；實際速度只有 0.1 倍 → 25 s 時 TIMEOUT、E_TIMEOUT
  FakeExecutorConfig c = config("quadruped");
  c.slowdown = 0.1;
  FakeExecutor ex(registry(), nullptr, c);
  double t = 0.0;
  ex.start(goal("go_to", R"({"x": 10.0, "y": 0.0})"), t);
  const StepSnapshot s = runUntilDone(ex, t, 60.0);
  EXPECT_EQ(s.status, StepStatus::TIMEOUT);
  EXPECT_EQ(s.failure_code, "E_TIMEOUT");
  EXPECT_NEAR(t, 25.0, 0.15);
  // goal 帶 deadline 時用它
  FakeExecutor ex2(registry(), nullptr, c);
  t = 0.0;
  StepGoal g = goal("go_to", R"({"x": 10.0, "y": 0.0})");
  g.deadline = 3.0;
  ex2.start(g, t);
  EXPECT_EQ(runUntilDone(ex2, t, 60.0).status, StepStatus::TIMEOUT);
  EXPECT_NEAR(t, 3.0, 0.15);
}

TEST(FakeExecutor, FaultInjection)
{
  const auto f = FakeExecutor::parseFault("inspect=E_HW:thermal_camera@0.3");
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->action, "inspect");
  EXPECT_EQ(f->failure_code, "E_HW");
  EXPECT_EQ(f->failed_capability, "thermal_camera");
  EXPECT_DOUBLE_EQ(f->at, 0.3);
  EXPECT_FALSE(FakeExecutor::parseFault("inspect").has_value());
  EXPECT_FALSE(FakeExecutor::parseFault("inspect=HW").has_value());
  EXPECT_FALSE(FakeExecutor::parseFault("inspect=E_HW@2").has_value());

  FakeExecutorConfig c = config("quadruped");
  c.faults = {*f};
  FakeExecutor ex(registry(), nullptr, c);
  double t = 0.0;
  ex.start(goal("go_to", R"({"x": 1.0, "y": 0.0})"), t);   // 別的動作不受影響
  EXPECT_EQ(runUntilDone(ex, t, 5.0).status, StepStatus::SUCCEEDED);
  ex.start(goal("inspect", "", R"({"mode": "thermal", "duration_s": 9})"), t);   // 瞄準 1 s＋拍 9 s
  const StepSnapshot s = runUntilDone(ex, t, 20.0);
  EXPECT_EQ(s.status, StepStatus::FAILED);
  EXPECT_EQ(s.failure_code, "E_HW");
  EXPECT_EQ(s.failed_capability, "thermal_camera");
  EXPECT_EQ(s.findings_json, "[]");
}

TEST(FakeExecutor, InspectReturnsFindings)
{
  FakeExecutor ex(registry(), nullptr, config("quadruped"));
  double t = 0.0;
  ex.start(goal("inspect", "", R"({"mode": "thermal", "duration_s": 2})", "machine"), t);
  t += 0.5;
  EXPECT_EQ(ex.step(t).phase, "aiming");
  t += 1.0;
  EXPECT_EQ(ex.step(t).phase, "capturing");
  const StepSnapshot s = runUntilDone(ex, t, 10.0);
  EXPECT_EQ(s.status, StepStatus::SUCCEEDED);
  EXPECT_NEAR(t, 3.0, 0.15);
  EXPECT_NE(s.findings_json.find("\"thermal\""), std::string::npos);
  EXPECT_NE(s.findings_json.find("evidence://fake/7/1"), std::string::npos);
  EXPECT_NE(s.findings_json.find("\"machine\""), std::string::npos);
}

TEST(FakeExecutor, SiteLocationsResolve)
{
  // 有場域設定：poi、dock 換成座標；沒有場域時只收座標
  FakeExecutor ex(registry(), &site(), config("quadruped"));
  double t = 0.0;
  ASSERT_EQ(ex.start(goal("go_to", R"({"poi": "OBS_HV_A_FRONT"})"), t).status, StepStatus::RUNNING);
  StepSnapshot s = runUntilDone(ex, t, 60.0);
  EXPECT_EQ(s.status, StepStatus::SUCCEEDED);
  EXPECT_NEAR(s.position[0], -10.0, 1e-6);
  EXPECT_NEAR(s.position[1], 7.5, 1e-6);
  FakeExecutor bare(registry(), nullptr, config("quadruped"));
  s = bare.start(goal("go_to", R"({"poi": "OBS_HV_A_FRONT"})"), 0.0);
  EXPECT_EQ(s.status, StepStatus::FAILED);
  EXPECT_EQ(s.failure_code, "E_PRECONDITION");
}

TEST(FakeExecutor, DogEmergencyStopThenRecovers)
{
  FakeExecutor ex(registry(), nullptr, config("quadruped"));
  double t = 0.0;
  ex.start(goal("emergency_stop"), t);
  EXPECT_EQ(runUntilDone(ex, t, 1.0).status, StepStatus::SUCCEEDED);
  EXPECT_EQ(ex.platformState(), PlatformState::EMERGENCY);
  ex.start(goal("go_to", R"({"x": 1.0, "y": 0.0})"), t);   // 下一個 goal 恢復
  EXPECT_EQ(ex.platformState(), PlatformState::READY);
}
