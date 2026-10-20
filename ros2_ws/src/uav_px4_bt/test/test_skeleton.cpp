// 機器狗端骨架＋無人機葉節點（trees/uav_mission.xml、skeleton/*.hpp）用假平台跑，不開 ROS。
// node 層（uav_task_manager_node.cpp）的 onCurrentTask、releaseExternal 用同樣的純函式在這裡重做一份：
// insertExternal／cancelExternal 寫黑板 → 立刻 tick（kick）；每拍後用 waypointInHand 記「手上的點」。
// 20 Hz tick；假平台見 fake_platform.hpp。
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <rcutils/logging.h>

#include "fake_platform.hpp"
#include "uav_px4_bt/bt_nodes.hpp"
#include "uav_px4_bt/skeleton/bt_nodes.hpp"
#include "uav_px4_bt/skeleton/external_task.hpp"
#include "uav_px4_bt/skeleton/uav_flight.hpp"
#include "uav_px4_bt/skeleton/uav_leaves.hpp"

using namespace uav_px4_bt;
using task_manager::g_ctx;
using task_manager::Waypoint;

namespace
{
using uav_px4_bt::test::FakePlatform;

struct Reported
{
  std::uint32_t task_id;
  bool success;
  std::string detail;
  std::uint8_t reason;
};

class SkeletonTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_ERROR);   // say() 靜音
    g_ctx = task_manager::Ctx{};
    g_ctx.task_result_pub = [this](std::uint32_t id, bool ok, const std::string & detail) {
        reported.push_back({id, ok, detail, task_manager::failureReason(ok, detail)});
      };
    // 已經在 5 m 懸停（起飛另外測）
    platform.v.valid = true;
    platform.v.ready = true;
    platform.v.armed = true;
    platform.v.offboard = true;
    platform.v.landed = false;
    platform.v.battery = 90.0;
    platform.v.position = {0, 0, 5};
    ctx = std::make_unique<UavContext>(platform, UavContextConfig{});
    task_manager::registerAll(factory);
    registerUavLeaves(factory);
    waypoints = std::make_shared<std::deque<Waypoint>>();
    bb = BT::Blackboard::create();
    bb->set("waypoints", waypoints);
    bb->set("current_task_id", std::uint32_t{0});
    bb->set("active_task_id", std::uint32_t{0});
    bb->set<UavContext *>("uav_ctx", ctx.get());
    bb->set<double>("altitude", 5.0);
    bb->set<double>("fire_monitor_timeout", 30.0);
    tree = std::make_unique<BT::Tree>(factory.createTreeFromFile(UAV_MISSION_FILE, bb));
  }

  void TearDown() override {g_ctx = task_manager::Ctx{};}

  // node 層的 onTick（任務部分）：佇列有點才開跑，跑完（不是哨兵那一拍）就結束
  void tick()
  {
    for (auto & f : platform.fires) {
      f.received = platform.t;   // 偵測持續在送
    }
    platform.velocity_cmd = {};
    if (!running) {
      if (waypoints->empty()) {
        platform.advance(0.05);
        return;
      }
      g_ctx.reset();
      running = true;
    }
    ++g_ctx.tick;
    last = task_manager::tickOrAbort(*tree);
    const bool sentinel = last == BT::NodeStatus::FAILURE && !g_ctx.emergency;
    if (last != BT::NodeStatus::RUNNING && !sentinel) {
      running = false;
      ++finished;
      in_hand.reset();
    } else {
      in_hand = task_manager::waypointInHand(*tree);
    }
    platform.advance(0.05);
  }

  void run(double seconds)
  {
    for (double s = 0.0; s < seconds; s += 0.05) {
      tick();
    }
  }

  template<typename F>
  bool runUntil(F done, double seconds)
  {
    for (double s = 0.0; s < seconds; s += 0.05) {
      if (done()) {
        return true;
      }
      tick();
    }
    return done();
  }

  // node 層的 onCurrentTask
  void dispatch(std::uint32_t id, double x, double y, double duration)
  {
    std::uint32_t cur = bb->get<std::uint32_t>("current_task_id");
    if (id == 0) {
      Waypoint buf;
      const Waypoint * active = in_hand ? &(buf = *in_hand) : nullptr;
      if (task_manager::cancelExternal(*waypoints, active, "cbba_", cur)) {
        bb->set("current_task_id", cur);
        g_ctx.preempt_t0 = std::chrono::steady_clock::now();
        tick();
      }
      return;
    }
    if (id == cur) {
      return;
    }
    Waypoint buf;
    const Waypoint * active = in_hand ? &(buf = *in_hand) : nullptr;
    const Waypoint wp = task_manager::makeExternalWaypoint("cbba_", id, x, y, 5.0, 5.0, duration);
    task_manager::insertExternal(*waypoints, active, wp, id, cur);
    bb->set("current_task_id", cur);
    g_ctx.preempt_t0 = std::chrono::steady_clock::now();
    tick();
  }

  FireObservation fire(Vec3 p)
  {
    FireObservation o;
    o.tag_id = 101;
    o.confidence = 0.9;
    o.frames = 10;
    o.position = p;
    o.received = platform.t;
    return o;
  }

  FakePlatform platform;
  std::unique_ptr<UavContext> ctx;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb;
  std::shared_ptr<std::deque<Waypoint>> waypoints;
  std::unique_ptr<BT::Tree> tree;
  std::vector<Reported> reported;
  std::optional<Waypoint> in_hand;
  bool running{false};
  int finished{0};
  BT::NodeStatus last{BT::NodeStatus::IDLE};
};
}  // namespace

TEST_F(SkeletonTest, CbbaTaskFliesHoversAndReportsOnce)
{
  // 指派 → NavigateTo 飛到上空 → AirRecon 懸停 duration → ReportTaskResult 回報成功一次、兩個 id 歸零 → 樹結束
  dispatch(0x00010001, 10, 0, 3.0);
  EXPECT_TRUE(running);
  run(0.5);
  EXPECT_EQ(platform.exec.mode, ExecMode::NAVIGATING);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 20.0));
  EXPECT_EQ(reported[0].task_id, 0x00010001u);
  EXPECT_TRUE(reported[0].success);
  EXPECT_EQ(reported[0].reason, task_manager::reason::NONE);
  EXPECT_NEAR(platform.v.position.x, 10.0, 0.5);
  EXPECT_NEAR(platform.v.position.z, 5.0, 0.3);
  run(1.0);
  EXPECT_FALSE(running);
  EXPECT_EQ(finished, 1);
  EXPECT_EQ(reported.size(), 1u);
  EXPECT_EQ(bb->get<std::uint32_t>("current_task_id"), 0u);
  EXPECT_EQ(bb->get<std::uint32_t>("active_task_id"), 0u);
  EXPECT_TRUE(platform.created.empty());
}

TEST_F(SkeletonTest, HoverLastsDurationAfterArrival)
{
  // 懸停時間從 AirRecon 開始（到達後）算
  dispatch(0x00010001, 10, 0, 4.0);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  const double arrived = platform.t;
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 10.0));
  EXPECT_NEAR(platform.t - arrived, 4.0, 0.2);
}

TEST_F(SkeletonTest, NewTaskPreemptsWithinOneTickAndOldIsNotReported)
{
  // 飛往 A 途中換成 B：哨兵在 kick 的那一拍 halt NavigateTo，接著做 B；A 不回報（CBBA 自己換掉的）
  dispatch(0x00010001, 20, 0, 2.0);
  run(1.0);
  const int halts_before = g_ctx.tick;
  dispatch(0x00010002, 0, 10, 2.0);
  EXPECT_EQ(g_ctx.tick, halts_before + 1);
  EXPECT_FALSE(g_ctx.preempt_t0.has_value());   // NavigateTo::onHalted 印了中斷延遲
  EXPECT_EQ(bb->get<std::uint32_t>("active_task_id"), 0x00010002u);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 20.0));
  ASSERT_EQ(reported.size(), 1u);
  EXPECT_EQ(reported[0].task_id, 0x00010002u);
  EXPECT_TRUE(reported[0].success);
  EXPECT_NEAR(platform.v.position.y, 10.0, 0.5);
  EXPECT_TRUE(waypoints->empty());
}

TEST_F(SkeletonTest, PreemptDuringAirReconHaltsHover)
{
  // 懸停中換任務：AirRecon 被 halt，不記結果、不回報舊的
  dispatch(0x00010001, 5, 0, 10.0);
  ASSERT_TRUE(runUntil([&] {return platform.exec.mode == ExecMode::EXECUTING;}, 15.0));
  dispatch(0x00010002, 5, 8, 1.0);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 20.0));
  ASSERT_EQ(reported.size(), 1u);
  EXPECT_EQ(reported[0].task_id, 0x00010002u);
}

TEST_F(SkeletonTest, ResendOfSameTaskIsIgnored)
{
  // transient_local 重播同一個 task_id：什麼都不變
  dispatch(0x00010001, 10, 0, 1.0);
  run(0.5);
  const int tick_before = g_ctx.tick;
  dispatch(0x00010001, 10, 0, 1.0);
  EXPECT_EQ(g_ctx.tick, tick_before);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 20.0));
  EXPECT_EQ(reported.size(), 1u);
}

TEST_F(SkeletonTest, CancelWhileFlyingHaltsWithoutReport)
{
  // task_id 0（CBBA 撤銷）：哨兵 halt，佇列空 → 樹結束；不回報（CBBA 自己撤的）
  dispatch(0x00010001, 20, 0, 1.0);
  run(1.0);
  dispatch(0, 0, 0, 0);
  run(0.5);
  EXPECT_FALSE(running);
  EXPECT_TRUE(reported.empty());
  EXPECT_EQ(bb->get<std::uint32_t>("current_task_id"), 0u);
}

TEST_F(SkeletonTest, CancelAfterReportIsIgnored)
{
  // 回報之後 cbba_node 才送 task_id 0：已經不在手上，不當成取消
  dispatch(0x00010001, 5, 0, 1.0);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 15.0));
  std::uint32_t cur = bb->get<std::uint32_t>("current_task_id");
  Waypoint buf;
  const Waypoint * active = in_hand ? &(buf = *in_hand) : nullptr;
  EXPECT_FALSE(task_manager::cancelExternal(*waypoints, active, "cbba_", cur));
}

TEST_F(SkeletonTest, FireSeenEnRouteDivertsHoversThenResumes)
{
  // 路過發現（週計畫，2026-10-14）：目的地 (20, 0)，航線旁 (8, 3) 有火點 → 發布地面處置任務（z = 0）一次 →
  // 轉向火點上空懸停監控（執行中）→ 地面任務結束 → 回到原本的目的地、照樣完成
  platform.fires.push_back(fire({8.0, 3.0, 0.2}));
  dispatch(0x00010001, 20, 0, 1.0);
  ASSERT_TRUE(runUntil([&] {return !platform.created.empty();}, 15.0));
  EXPECT_EQ(platform.created[0].type, kTaskGroundIntervention);
  EXPECT_NEAR(platform.created[0].position.x, 8.0, 1e-9);
  EXPECT_DOUBLE_EQ(platform.created[0].position.z, 0.0);
  const std::uint32_t ground = platform.created[0].task_id;
  ASSERT_TRUE(runUntil([&] {return std::hypot(platform.v.position.x - 8.0, platform.v.position.y - 3.0) < 0.5;},
    15.0));
  run(5.0);   // 懸停監控中：不離開、不回報
  EXPECT_NEAR(platform.v.position.x, 8.0, 0.5);
  EXPECT_NEAR(platform.v.position.y, 3.0, 0.5);
  EXPECT_EQ(platform.exec.mode, ExecMode::EXECUTING);
  EXPECT_TRUE(reported.empty());
  platform.closed.insert(ground);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 25.0));
  EXPECT_TRUE(reported[0].success);
  EXPECT_EQ(platform.created.size(), 1u);
  EXPECT_NEAR(platform.v.position.x, 20.0, 0.5);
}

TEST_F(SkeletonTest, FireSeenEnRouteMonitorTimesOutThenResumes)
{
  // 轉向火點後地面任務一直沒結束（沒有狗）：monitor_timeout（這裡 30 s）後回到航線、完成原本的任務
  platform.fires.push_back(fire({8.0, 3.0, 0.2}));
  dispatch(0x00010001, 20, 0, 1.0);
  ASSERT_TRUE(runUntil([&] {return !platform.created.empty();}, 15.0));
  run(20.0);
  EXPECT_TRUE(reported.empty());
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 40.0));
  EXPECT_TRUE(reported[0].success);
  EXPECT_NEAR(platform.v.position.x, 20.0, 0.5);
}

TEST_F(SkeletonTest, FireAtPatrolPointHoversUntilGroundTaskClosed)
{
  // 週計畫：巡檢點上有火 → 途中不報；到了懸停穩定 settle_sec 後 AirRecon 發布地面任務一次、移到火點上空 →
  // 停留時間到了繼續懸停監控 → 地面任務結束（狗處理完，task_closed）才完成這個點、回報成功
  platform.fires.push_back(fire({10.2, 0.1, 0.2}));
  dispatch(0x00010001, 10, 0, 3.0);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  EXPECT_TRUE(platform.created.empty());
  const double arrived = platform.t;
  ASSERT_TRUE(runUntil([&] {return !platform.created.empty();}, 3.0));
  EXPECT_GE(platform.t - arrived, 1.0 - 1e-9);
  const std::uint32_t ground = platform.created[0].task_id;
  EXPECT_EQ(platform.created[0].type, kTaskGroundIntervention);
  run(10.0);   // 停留 3 s 早就到了：還在監控
  EXPECT_TRUE(reported.empty());
  EXPECT_EQ(platform.exec.mode, ExecMode::EXECUTING);
  EXPECT_NEAR(platform.v.position.x, 10.2, 0.3);
  platform.closed.insert(ground);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 1.0));
  EXPECT_TRUE(reported[0].success);
  EXPECT_EQ(platform.created.size(), 1u);
}

TEST_F(SkeletonTest, FireBesidePatrolPointReturnsToFinishHover)
{
  // 巡檢點 (10, 0) 旁 2 m 有火：停 1 s（settle）看到 → 移到火點上空監控 → 狗處理完 → 飛回 (10, 0) 補完剩下的停留時間
  // 才回報完成（2026-10-20：原本狗處理完就直接回報，巡檢點只停了 1 s）
  platform.fires.push_back(fire({12.0, 0.0, 0.2}));
  dispatch(0x00010001, 10, 0, 4.0);
  ASSERT_TRUE(runUntil([&] {return !platform.created.empty();}, 20.0));
  run(8.0);
  EXPECT_NEAR(platform.v.position.x, 12.0, 0.3);   // 在火點上空監控
  EXPECT_TRUE(reported.empty());
  platform.closed.insert(platform.created[0].task_id);
  const double closed = platform.t;
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 20.0));
  EXPECT_TRUE(reported[0].success);
  EXPECT_NEAR(platform.v.position.x, 10.0, 0.5);   // 回到巡檢點上空
  EXPECT_GT(platform.t - closed, 3.0 - 1e-9);      // 停留 4 s、到達後 1 s 就離開 → 回來補約 3 s（加上飛回來的時間）
  EXPECT_EQ(platform.created.size(), 1u);
}

TEST_F(SkeletonTest, FireMonitorGivesUpAfterTimeout)
{
  // 地面任務一直沒結束（沒有狗）：monitor_timeout（這裡 30 s）後照樣完成、離開
  platform.fires.push_back(fire({10.2, 0.1, 0.2}));
  dispatch(0x00010001, 10, 0, 3.0);
  ASSERT_TRUE(runUntil([&] {return !platform.created.empty();}, 20.0));
  const double seen = platform.t;
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 40.0));
  EXPECT_TRUE(reported[0].success);
  EXPECT_NEAR(platform.t - seen, 2.0 + 30.0, 1.0);   // 停留剩下的約 2 s＋監控 30 s
}

TEST_F(SkeletonTest, FireSeenEnRouteToItsPointIsMonitoredNotRepublished)
{
  // 路上先看到（離目的地 3 m 外，發布一次、轉向上空監控），處理完回到目的地又看到同一個火點：不再發布、
  // 已經處理完不再過去，停留時間到就完成
  platform.fires.push_back(fire({16.5, 0.0, 0.2}));
  dispatch(0x00010001, 20, 0, 1.0);
  ASSERT_TRUE(runUntil([&] {return !platform.created.empty();}, 15.0));
  const std::uint32_t ground = platform.created[0].task_id;
  ASSERT_TRUE(runUntil([&] {return platform.exec.mode == ExecMode::EXECUTING;}, 15.0));
  run(5.0);
  EXPECT_EQ(platform.created.size(), 1u);
  EXPECT_TRUE(reported.empty());
  platform.closed.insert(ground);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 15.0));
  EXPECT_TRUE(reported[0].success);
  EXPECT_EQ(platform.created.size(), 1u);
  EXPECT_NEAR(platform.v.position.x, 20.0, 0.5);
}

TEST_F(SkeletonTest, FireReportedByOtherUavIsNotRepublishedNorDiverted)
{
  // 別台已經報過的火（cbba_node 的 task_announced，2026-10-14）：航線旁看到 → 不再發布、不轉向，照常飛到目的地完成
  platform.ground.push_back({0x00028001u, {8.0, 3.0, 0.0}});
  platform.fires.push_back(fire({8.2, 3.1, 0.2}));
  dispatch(0x00010001, 20, 0, 1.0);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 25.0));
  EXPECT_TRUE(platform.created.empty());
  EXPECT_TRUE(reported[0].success);
  EXPECT_NEAR(platform.v.position.x, 20.0, 0.5);
}

TEST_F(SkeletonTest, FireAtPatrolPointReportedByOtherIsNotMonitored)
{
  // 巡檢點上的火別台已經報過（它在監控）：不再發布、不移過去、不等地面任務，停留時間到就完成
  platform.ground.push_back({0x00028001u, {10.2, 0.1, 0.0}});
  platform.fires.push_back(fire({10.2, 0.1, 0.2}));
  dispatch(0x00010001, 10, 0, 3.0);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 5.0));
  EXPECT_TRUE(platform.created.empty());
  EXPECT_TRUE(reported[0].success);
}

TEST_F(SkeletonTest, NavTimeoutHandsBackTransientAndSkips)
{
  // 飛不到（卡住）：NAV_FAILED(TIMEOUT) → HandleWaypointFailure 交還 CBBA（TRANSIENT）、跳過這點、樹正常結束
  platform.stuck = true;
  dispatch(0x00010001, 10, 0, 1.0);
  ASSERT_TRUE(runUntil([&] {return !reported.empty();}, 60.0));
  EXPECT_EQ(reported[0].detail, "NAV_FAILED(TIMEOUT)");
  EXPECT_FALSE(reported[0].success);
  EXPECT_EQ(reported[0].reason, task_manager::reason::TRANSIENT);
  run(0.5);
  EXPECT_FALSE(running);
  EXPECT_EQ(last, BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(g_ctx.aborted);
  EXPECT_EQ(reported.size(), 1u);
}

TEST_F(SkeletonTest, OffboardLostAbortsWithNoAck)
{
  // 飛行中掉出 offboard：NAV_NO_ACK → 交還 CBBA、整趟中止
  dispatch(0x00010001, 20, 0, 1.0);
  run(1.0);
  platform.v.offboard = false;
  run(0.5);
  ASSERT_EQ(reported.size(), 1u);
  EXPECT_EQ(reported[0].detail, "NAV_NO_ACK");
  EXPECT_FALSE(running);
  EXPECT_TRUE(g_ctx.aborted);
}

TEST_F(SkeletonTest, EmergencyHaltsTree)
{
  // 急停：安全守門 FAILURE、樹 halt；node 層據此交還（這裡只看樹停了、沒有回報成功）
  dispatch(0x00010001, 20, 0, 1.0);
  run(1.0);
  g_ctx.emergency = true;
  tick();
  EXPECT_FALSE(running);
  EXPECT_EQ(last, BT::NodeStatus::FAILURE);
  EXPECT_TRUE(reported.empty());
}

TEST(ExternalWaypoint, AboveTargetWithAirReconAndReport)
{
  // 無人機版的外部點：目標正上方、z = max(高度, 目標 z)、air_recon → report_result
  const Waypoint w = task_manager::makeExternalWaypoint("cbba_", 0x00030007, 4.0, -2.0, 8.0, 5.0, 12.0);
  EXPECT_EQ(w.id, "cbba_" + std::to_string(0x00030007));
  EXPECT_DOUBLE_EQ(w.x, 4.0);
  EXPECT_DOUBLE_EQ(w.y, -2.0);
  ASSERT_EQ(w.tasks.size(), 2u);
  EXPECT_EQ(w.tasks[0].action, "air_recon");
  EXPECT_DOUBLE_EQ(std::stod(w.tasks[0].get("target.z")), 8.0);
  EXPECT_DOUBLE_EQ(std::stod(w.tasks[0].get("duration_sec")), 12.0);
  EXPECT_EQ(w.tasks[1].action, "report_result");
  EXPECT_EQ(w.tasks[1].get("task_id"), std::to_string(0x00030007));
  const Waypoint low = task_manager::makeExternalWaypoint("cbba_", 1, 0, 0, 0.0, 5.0, 1.0);
  EXPECT_DOUBLE_EQ(std::stod(low.tasks[0].get("target.z")), 5.0);
}

TEST(FailureReason, UavStopReasons)
{
  // 無人機加的停止原因：60 s 內不再出價（STOPPED），不是永久
  EXPECT_EQ(task_manager::failureReason(false, "BATTERY_RTL"), task_manager::reason::STOPPED);
  EXPECT_EQ(task_manager::failureReason(false, "NOT_AIRBORNE"), task_manager::reason::STOPPED);
  EXPECT_EQ(task_manager::failureReason(false, "NAV_FAILED(TIMEOUT)"), task_manager::reason::TRANSIENT);
  EXPECT_EQ(task_manager::failureReason(false, "NAV_NO_ACK"), task_manager::reason::TRANSIENT);
}

// ===========================================================================
// 樹外的飛行（UavFlight）
// ===========================================================================
namespace
{
class FlightTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    platform.v.valid = true;
    platform.v.ready = true;
    platform.v.battery = 90.0;
    platform.v.position = {0, 0, 0};
    ctx = std::make_unique<UavContext>(platform, UavContextConfig{});
    flight = std::make_unique<UavFlight>(*ctx, 5.0, 20.0, 2.0);
  }

  FlightEvent step()
  {
    platform.velocity_cmd = {};
    const FlightEvent e = flight->update();
    if (flight->phase() == FlightPhase::AIRBORNE) {
      flight->loiter();
    }
    platform.advance(0.05);
    return e;
  }

  template<typename F>
  bool runUntil(F done, double seconds)
  {
    for (double s = 0.0; s < seconds; s += 0.05) {
      if (done(step())) {
        return true;
      }
    }
    return false;
  }

  long countLogs(const std::string & key) const
  {
    return std::count_if(platform.logs.begin(), platform.logs.end(),
             [&](const std::string & l) {return l.find(key) != std::string::npos;});
  }

  FakePlatform platform;
  std::unique_ptr<UavContext> ctx;
  std::unique_ptr<UavFlight> flight;
};
}  // namespace

TEST_F(FlightTest, TakesOffThenLoiters)
{
  // 先送 1 s 零速度才要求 offboard＋解鎖，爬升到 5 m → TOOK_OFF 一次；之後原地懸停
  for (int i = 0; i < 10; ++i) {
    step();
  }
  EXPECT_EQ(platform.arm_requests, 0);
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  EXPECT_EQ(platform.arm_requests, 1);
  EXPECT_TRUE(ctx->airborne);
  EXPECT_FALSE(runUntil([](FlightEvent e) {return e != FlightEvent::NONE;}, 3.0));
  EXPECT_NEAR(platform.v.position.z, 5.0, 0.3);
  EXPECT_NEAR(platform.v.position.x, 0.0, 0.1);
  EXPECT_EQ(platform.exec.mode, ExecMode::IDLE);
}

TEST_F(FlightTest, HoldsAtPatrolAltitudeAfterTakeoff)
{
  // 起飛完成時離巡檢高度還差 takeoff_tolerance 以內（SITL：4.71 m）：待命點用巡檢高度，最後停在 5.0 m
  flight = std::make_unique<UavFlight>(*ctx, 5.0, 20.0, -1.0);
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  runUntil([](FlightEvent) {return false;}, 10.0);
  EXPECT_NEAR(platform.v.position.z, 5.0, 0.05);
}

TEST_F(FlightTest, NotReadyWaitsWithoutArming)
{
  // 起飛前檢查沒過：只送零速度，不要求解鎖
  platform.v.ready = false;
  EXPECT_FALSE(runUntil([](FlightEvent e) {return e != FlightEvent::NONE;}, 3.0));
  EXPECT_EQ(platform.arm_requests, 0);
}

TEST_F(FlightTest, LowBatteryReturnsToLaunchOnce)
{
  // 在空中、電量 ≤ 20 %：BATTERY_RTL 一次、停止 offboard、RTL 指令最多 3 次；電量回升也不恢復
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  platform.v.battery = 19.0;
  EXPECT_EQ(step(), FlightEvent::BATTERY_RTL);
  EXPECT_EQ(flight->phase(), FlightPhase::RTL);
  EXPECT_FALSE(platform.offboard);
  platform.v.battery = 25.0;
  EXPECT_FALSE(runUntil([](FlightEvent e) {return e != FlightEvent::NONE;}, 10.0));
  EXPECT_EQ(platform.rtl_requests, 3);
  EXPECT_EQ(flight->phase(), FlightPhase::RTL);
  EXPECT_EQ(platform.exec.mode, ExecMode::FAULT);
}

TEST_F(FlightTest, LandedAfterTakeoffReportsAndTakesOffAgain)
{
  // 起飛完成後上鎖：LANDED 一次、回到起飛
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  platform.v.armed = false;
  platform.v.offboard = false;
  EXPECT_EQ(step(), FlightEvent::LANDED);
  EXPECT_EQ(flight->phase(), FlightPhase::GROUND);
  EXPECT_FALSE(ctx->airborne);
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
}

TEST_F(FlightTest, LoiterYieldsToBusyNeighbor)
{
  // 待命中，執行任務中的鄰機飛到身邊（1 m/s 飛過來）：讓開、不拉回原來的待命點（不飛回返航點的設定）。
  // CBF 讓路 = 把它當不會讓的障礙物：b = a·v_j − γ h，退到 d² = d_s² + 2 d·v_j/γ ≈ 1.85 m（2026-10-14）
  flight = std::make_unique<UavFlight>(*ctx, 5.0, 20.0, -1.0);
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  Neighbor n;
  n.id = 2;
  n.position = {1.0, 0.0, 5.0};
  n.velocity = {-1.0, 0.0, 0.0};
  n.busy = true;
  platform.nbrs = {n};
  runUntil([](FlightEvent) {return false;}, 3.0);
  const double pushed = platform.v.position.x;
  EXPECT_LT(pushed, -0.5);
  platform.nbrs.clear();
  runUntil([](FlightEvent) {return false;}, 3.0);
  EXPECT_NEAR(platform.v.position.x, pushed, 0.5);
}

TEST_F(FlightTest, FarBusyNeighborDoesNotStopHoming)
{
  // 飛回起飛點途中，執行中的鄰機在 10 m 外（CBF 的 influence_radius 設成 12 m 時也一樣）：照常飛回去，不停下讓路
  // （2026-10-19 SITL：uav2 回起飛點途中離懸停監控的 uav1 10 m 就停了 12 s）
  UavContextConfig cfg;
  cfg.cbf.influence_radius = 12.0;
  ctx = std::make_unique<UavContext>(platform, cfg);
  flight = std::make_unique<UavFlight>(*ctx, 5.0, 20.0, 0.0);
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  platform.v.position = {12.0, 0.0, 5.0};
  platform.v.velocity = {};
  flight->leaveHold();
  Neighbor n;
  n.id = 2;
  n.position = {12.0, -10.0, 5.0};
  n.busy = true;
  platform.nbrs = {n};
  runUntil([](FlightEvent) {return false;}, 20.0);
  EXPECT_EQ(countLogs("讓路"), 0);
  EXPECT_NEAR(platform.v.position.x, 0.0, 0.5);
  EXPECT_NEAR(platform.v.position.y, 0.0, 0.5);
}

TEST_F(FlightTest, IdleSeesFireReportsAndMonitorsThenReturnsHome)
{
  // 待命（飛回起飛點途中）看到新火點：發布地面處置任務一次 → 飛到火點上空監控 → 地面任務結束 → 回到待命、飛回起飛點
  // （2026-10-19 SITL：回程清楚看到 R1 的火，原本只有執行任務的 NavigateTo 會處理）
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  FireObservation f;
  f.tag_id = 101;
  f.confidence = 0.95;
  f.frames = 10;
  f.position = {6.0, 3.0, 0.2};
  f.received = platform.t + 1e6;   // 一直是新的
  platform.fires = {f};
  step();
  EXPECT_EQ(countLogs("待命中發現火警"), 1);
  ASSERT_EQ(platform.created.size(), 1u);
  EXPECT_TRUE(flight->monitoringFire());
  runUntil([](FlightEvent) {return false;}, 10.0);
  EXPECT_NEAR(platform.v.position.x, 6.0, 0.3);
  EXPECT_NEAR(platform.v.position.y, 3.0, 0.3);
  EXPECT_EQ(countLogs("在火點上空懸停監控"), 1);
  EXPECT_EQ(platform.created.size(), 1u);   // 同一個火點只報一次
  EXPECT_EQ(platform.exec.mode, ExecMode::EXECUTING);

  platform.closed.insert(platform.created[0].task_id);
  step();
  EXPECT_EQ(countLogs("回到待命"), 1);
  EXPECT_FALSE(flight->monitoringFire());
  runUntil([](FlightEvent) {return false;}, 20.0);
  EXPECT_NEAR(platform.v.position.x, 0.0, 0.3);
  EXPECT_NEAR(platform.v.position.y, 0.0, 0.3);
}

TEST_F(FlightTest, NewTaskStopsIdleFireMonitoring)
{
  // 待命監控火點時 CBBA 派了任務（node 層開始跑樹前呼叫 leaveHold）：結束監控去做任務，地面任務照常給狗
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  FireObservation f;
  f.tag_id = 101;
  f.confidence = 0.95;
  f.frames = 10;
  f.position = {4.0, 0.0, 0.2};
  f.received = platform.t + 1e6;
  platform.fires = {f};
  step();
  ASSERT_TRUE(flight->monitoringFire());
  flight->leaveHold();
  EXPECT_FALSE(flight->monitoringFire());
  EXPECT_EQ(countLogs("停止監控火點"), 1);
  EXPECT_EQ(platform.created.size(), 1u);
}

TEST_F(FlightTest, IdleFireWatchCanBeDisabled)
{
  // fire_task.idle_watch_tag = 0：待命時不看火
  flight->setIdleFireWatch(0, 120.0);
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  FireObservation f;
  f.tag_id = 101;
  f.confidence = 0.95;
  f.frames = 10;
  f.position = {4.0, 0.0, 0.2};
  f.received = platform.t + 1e6;
  platform.fires = {f};
  runUntil([](FlightEvent) {return false;}, 3.0);
  EXPECT_TRUE(platform.created.empty());
  EXPECT_FALSE(flight->monitoringFire());
}

TEST_F(FlightTest, ReturnsHomeAfterIdleDelay)
{
  // 任務做完停在 (12, 0)：先原地懸停，2 s 後飛回起飛點上空 (0, 0, 5) 待命
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  EXPECT_NEAR(flight->home().z, 5.0, 1e-9);
  platform.v.position = {12.0, 0.0, 5.0};
  platform.v.velocity = {};
  flight->leaveHold();
  runUntil([](FlightEvent) {return false;}, 1.5);
  EXPECT_NEAR(platform.v.position.x, 12.0, 0.2);
  runUntil([](FlightEvent) {return false;}, 20.0);
  EXPECT_NEAR(platform.v.position.x, 0.0, 0.3);
  EXPECT_NEAR(platform.v.position.y, 0.0, 0.3);
  EXPECT_NEAR(platform.v.position.z, 5.0, 0.3);
  EXPECT_EQ(platform.exec.mode, ExecMode::IDLE);
}

TEST_F(FlightTest, NegativeDelayStaysWhereItFinished)
{
  // return_home_delay < 0：一直原地懸停
  flight = std::make_unique<UavFlight>(*ctx, 5.0, 20.0, -1.0);
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  platform.v.position = {12.0, 0.0, 5.0};
  platform.v.velocity = {};
  flight->leaveHold();
  runUntil([](FlightEvent) {return false;}, 10.0);
  EXPECT_NEAR(platform.v.position.x, 12.0, 0.3);
}

TEST_F(FlightTest, SlowClimbWarnsOnce)
{
  // 解鎖、offboard 之後爬不上去：20 s 後警告一次，一直不算起飛完成
  platform.stuck = true;
  EXPECT_FALSE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 30.0));
  EXPECT_EQ(flight->phase(), FlightPhase::GROUND);
  EXPECT_EQ(countLogs("還沒到"), 1);
}

TEST_F(FlightTest, UserTakeoverEntersOverrideAndResumesHere)
{
  // 起飛完成後被人切出 offboard（failsafe 0）：TAKEOVER 一次、OVERRIDE（node 層：PREEMPTED_BY_MANUAL、mode MANUAL）；
  // 期間被飛到 (6, 0, 2) 再切回 → RESUMED、警告高度差、待命點改成目前的水平位置＋巡檢高度（不飛回接管前的點）
  flight = std::make_unique<UavFlight>(*ctx, 5.0, 20.0, -1.0);
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  platform.v.offboard = false;
  EXPECT_EQ(step(), FlightEvent::TAKEOVER);
  EXPECT_EQ(flight->phase(), FlightPhase::OVERRIDE);
  EXPECT_TRUE(flight->overrideByUser());
  EXPECT_STREQ(flight->overrideReason(), "PREEMPTED_BY_MANUAL");
  EXPECT_EQ(step(), FlightEvent::NONE);
  EXPECT_EQ(countLogs("被接管"), 1);

  platform.v.position = {6.0, 0.0, 2.0};
  platform.v.velocity = {};
  platform.v.offboard = true;
  EXPECT_EQ(step(), FlightEvent::RESUMED);
  EXPECT_EQ(flight->phase(), FlightPhase::AIRBORNE);
  EXPECT_EQ(countLogs("切回 offboard，高度"), 1);
  runUntil([](FlightEvent) {return false;}, 10.0);
  EXPECT_NEAR(platform.v.position.x, 6.0, 0.3);
  EXPECT_NEAR(platform.v.position.z, 5.0, 0.1);
}

TEST_F(FlightTest, RejoinAtAltitudeIsNotWarned)
{
  // 接管期間高度沒變：切回 offboard 只印一般訊息
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  platform.v.offboard = false;
  step();
  platform.v.offboard = true;
  EXPECT_EQ(step(), FlightEvent::RESUMED);
  EXPECT_EQ(countLogs("切回 offboard，高度"), 0);
  EXPECT_EQ(countLogs("切回 offboard（高度"), 1);
}

TEST_F(FlightTest, Px4FailsafeIsNotTakeover)
{
  // PX4 自己的失效保護（failsafe 1，例如 setpoint 斷了 → AUTO_RTL）：FAILSAFE（PX4_FAILSAFE → STOPPED）；
  // 之後有人接手（user_took_over）→ 原因改成人接管；降落 → LANDED
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  platform.v.offboard = false;
  platform.v.failsafe = true;
  EXPECT_EQ(step(), FlightEvent::FAILSAFE);
  EXPECT_EQ(flight->phase(), FlightPhase::OVERRIDE);
  EXPECT_STREQ(flight->overrideReason(), "PX4_FAILSAFE");
  EXPECT_EQ(countLogs("PX4 失效保護"), 1);

  platform.v.user_took_over = true;
  EXPECT_EQ(step(), FlightEvent::NONE);
  EXPECT_STREQ(flight->overrideReason(), "PREEMPTED_BY_MANUAL");

  platform.v.armed = false;
  platform.v.landed = true;
  EXPECT_EQ(step(), FlightEvent::LANDED);
  EXPECT_EQ(flight->phase(), FlightPhase::GROUND);
}

TEST_F(FlightTest, OwnReturnToLaunchIsNotOverride)
{
  // 自己的低電量返航（BT 停 offboard、要 RTL）：一直是 RTL，不當成被接管
  ASSERT_TRUE(runUntil([](FlightEvent e) {return e == FlightEvent::TOOK_OFF;}, 15.0));
  platform.v.battery = 19.0;
  EXPECT_EQ(step(), FlightEvent::BATTERY_RTL);
  EXPECT_FALSE(platform.v.offboard);
  EXPECT_FALSE(runUntil([](FlightEvent e) {return e != FlightEvent::NONE;}, 3.0));
  EXPECT_EQ(flight->phase(), FlightPhase::RTL);
  EXPECT_EQ(countLogs("被接管"), 0);
}

TEST(FailureReason, TakeoverIsPreemptedAndFailsafeIsStopped)
{
  // 被人接管照狗的 PREEMPTED_BY_MANUAL（不算失敗次數）；PX4 失效保護是載具停了
  EXPECT_EQ(task_manager::failureReason(false, "PREEMPTED_BY_MANUAL"), task_manager::reason::PREEMPTED);
  EXPECT_EQ(task_manager::failureReason(false, "PX4_FAILSAFE"), task_manager::reason::STOPPED);
}
