// 整棵備用 BT（trees/main_uav_tree.xml）用假的平台跑：起飛、執行指派、火警判定與廣播、換任務、返航、逾時。
// 假平台：速度一階追隨命令（時間常數 0.3 s），要求 offboard＋解鎖後下一個 tick 就成立；20 Hz tick。
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "uav_px4_bt/bt_nodes.hpp"
#include "fake_platform.hpp"

using namespace uav_px4_bt;

namespace
{
using uav_px4_bt::test::FakePlatform;

class TreeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    platform.v.valid = true;
    platform.v.ready = true;
    platform.v.battery = 90.0;
    platform.v.position = {0, 0, 0};
    ctx = std::make_unique<UavContext>(platform, UavContextConfig{});
    registerUavNodes(factory);
    bb = BT::Blackboard::create();
    bb->set<UavContext *>("ctx", ctx.get());
    bb->set<AssignedTask>("current_uav_task", AssignedTask{});
    bb->set<AssignedTask>("active_uav_task", AssignedTask{});
    bb->set<double>("altitude", 5.0);
    tree = factory.createTreeFromFile(TREE_FILE, bb);
  }

  void run(double seconds)
  {
    for (double s = 0.0; s < seconds; s += 0.05) {
      platform.velocity_cmd = {};
      bb->set<AssignedTask>("current_uav_task", assigned);
      tree.tickOnce();
      platform.advance(0.05);
    }
  }

  // 跑到條件成立（最多 seconds 秒），回傳是否成立
  template<typename F>
  bool runUntil(F done, double seconds)
  {
    for (double s = 0.0; s < seconds; s += 0.05) {
      if (done()) {
        return true;
      }
      run(0.05);
    }
    return done();
  }

  static AssignedTask task(std::uint32_t id, double x, double y, double duration, std::uint64_t version)
  {
    AssignedTask t;
    t.task_id = id;
    t.type = 1;
    t.position = {x, y, 0};
    t.duration = duration;
    t.version = version;
    return t;
  }

  FireObservation fire(std::uint32_t tag, double confidence, int frames, Vec3 position = {10.3, -0.2, 0.3})
  {
    FireObservation o;
    o.tag_id = tag;
    o.confidence = confidence;
    o.frames = frames;
    o.position = position;
    o.received = platform.t;
    return o;
  }

  FakePlatform platform;
  std::unique_ptr<UavContext> ctx;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb;
  BT::Tree tree;
  AssignedTask assigned;
};
}  // namespace

TEST_F(TreeTest, TakesOffAndLoiters)
{
  // 先送 1 s 零速度，再要求 offboard＋解鎖，爬升到 5 m；沒有任務時原地懸停、執行狀態閒置
  run(0.5);
  EXPECT_EQ(platform.arm_requests, 0);
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  EXPECT_EQ(platform.arm_requests, 1);
  run(3.0);
  EXPECT_NEAR(platform.v.position.z, 5.0, 0.3);
  EXPECT_NEAR(platform.v.position.x, 0.0, 0.1);
  EXPECT_EQ(platform.exec.mode, ExecMode::IDLE);
  EXPECT_EQ(platform.exec.active_task, 0u);
}

TEST_F(TreeTest, ExecutesAssignmentAndReportsWithVersion)
{
  // 指派 → 前往（NAVIGATING）→ 到達後觀察、停留（EXECUTING）→ 回報完成→ 閒置
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 10, 0, 5.0, 7);
  run(1.0);
  EXPECT_EQ(platform.exec.mode, ExecMode::NAVIGATING);
  EXPECT_EQ(platform.exec.active_task, 0x00010001u);
  EXPECT_EQ(platform.exec.active_version, 7u);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  run(1.0);
  EXPECT_EQ(platform.exec.mode, ExecMode::EXECUTING);
  EXPECT_NEAR(platform.exec.remaining, 4.0, 0.2);
  ASSERT_TRUE(runUntil([&] {return !platform.results.empty();}, 10.0));
  ASSERT_EQ(platform.results.size(), 1u);
  EXPECT_EQ(platform.results[0].task_id, 0x00010001u);
  EXPECT_EQ(platform.results[0].version, 7u);
  EXPECT_TRUE(platform.results[0].success);
  EXPECT_NEAR(platform.v.position.x, 10.0, 0.5);
  // cbba_node 確認之前還是同一個指派：不重做、不重複回報
  run(3.0);
  EXPECT_EQ(platform.results.size(), 1u);
  EXPECT_EQ(platform.exec.mode, ExecMode::IDLE);
  EXPECT_TRUE(platform.created.empty());
}

TEST_F(TreeTest, FireConfirmedCreatesGroundTaskOnce)
{
  // 到達後看到 tag 101（confidence 0.9、10 張）→ 懸停穩定 1 s 後才判定 → 建立 GROUND_INTERVENTION（地面 z = 0）；
  // 同一個火點只報一次
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 10, 0, 2.0, 1);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  platform.fires = {fire(101, 0.9, 10)};
  run(0.8);
  EXPECT_TRUE(platform.created.empty());
  platform.fires = {fire(101, 0.9, 10)};
  run(0.4);
  EXPECT_EQ(platform.created.size(), 1u);
  ASSERT_TRUE(runUntil([&] {return !platform.results.empty();}, 10.0));
  ASSERT_EQ(platform.created.size(), 1u);
  EXPECT_EQ(platform.created[0].type, kTaskGroundIntervention);
  EXPECT_NEAR(platform.created[0].position.x, 10.3, 1e-9);
  EXPECT_DOUBLE_EQ(platform.created[0].position.z, 0.0);
  EXPECT_TRUE(platform.results[0].success);

  // 別的偵巡任務又看到同一個火點：不再建立
  assigned = task(0x00010002, 11, 1, 2.0, 1);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  platform.fires = {fire(101, 0.95, 10)};
  ASSERT_TRUE(runUntil([&] {return platform.results.size() == 2;}, 10.0));
  EXPECT_EQ(platform.created.size(), 1u);
}

TEST_F(TreeTest, FireAtAnotherPlaceIsReportedAgain)
{
  // 火點都貼 tag 101：另一個地點（超過 fire_dedup_radius 3 m）的火警要再建立一個地面任務（2026-10-12 前只看 tag 編號，
  // 第二個火點不會報）；回到第一個火點附近（3 m 內）則不再報
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 10, 0, 2.0, 1);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  // 到達後才看得到（飛行途中看到會走「路過發現」，見 FireSeenOnTheWayDivertsToMonitor）；
  // 偵測結果一直在更新（max_age 0.5 s，懸停穩定 1 s 後才判斷）
  auto seen = [&](Vec3 p, std::size_t n) {
      return runUntil([&] {
               platform.fires.clear();
               if (ctx->arrival >= 0.0) {
                 platform.fires = {fire(101, 0.9, 10, p)};
               }
               return platform.results.size() == n;
             }, 20.0);
    };
  ASSERT_TRUE(seen({10.3, -0.2, 0.3}, 1));
  ASSERT_EQ(platform.created.size(), 1u);

  assigned = task(0x00010002, 0, 12, 2.0, 1);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  ASSERT_TRUE(seen({0.2, 12.1, 0.3}, 2));
  ASSERT_EQ(platform.created.size(), 2u);
  EXPECT_NEAR(platform.created[1].position.y, 12.1, 1e-9);

  assigned = task(0x00010003, 11, 1, 2.0, 1);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  ASSERT_TRUE(seen({11.5, 1.0, 0.3}, 3));   // 第一個火點 1.6 m 內
  EXPECT_EQ(platform.created.size(), 2u);
}

TEST_F(TreeTest, FireSeenOnTheWayIsReportedAndFlightContinues)
{
  // 路過發現（2026-10-12，預設）：飛往 (30, 0) 的途中看到 tag 101 → 馬上建立地面處置任務（給 V60 競標），
  // 繼續飛完自己的任務；到達後不再報同一個火點
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 30, 0, 1.0, 1);
  ASSERT_TRUE(runUntil([&] {return platform.v.position.x > 8.0;}, 20.0));
  ASSERT_TRUE(runUntil([&] {
      platform.fires = {fire(101, 0.9, 8, {12.0, 0.4, 0.3})};
      return !platform.created.empty();
    }, 5.0));
  EXPECT_LT(ctx->arrival, 0.0);                       // 還在飛
  ASSERT_EQ(platform.created.size(), 1u);
  EXPECT_EQ(platform.created[0].type, kTaskGroundIntervention);
  EXPECT_NEAR(platform.created[0].position.x, 12.0, 1e-9);
  ASSERT_TRUE(runUntil([&] {
      platform.fires = {fire(101, 0.9, 8, {12.0, 0.4, 0.3})};
      return !platform.results.empty();
    }, 30.0));
  EXPECT_TRUE(platform.results[0].success);
  EXPECT_NEAR(platform.v.position.x, 30.0, 1.0);
  EXPECT_EQ(platform.created.size(), 1u);
}

TEST_F(TreeTest, FireSeenOnTheWayDivertsToMonitorWhenEnabled)
{
  // fire_divert_on_sight = true：地面處置任務＋火點監控任務（AIR_RECON，停留 20 s），
  // 手上的任務回報失敗 FIRE_FOUND、原地懸停；CBBA 派下監控任務 → 飛到火點上空懸停到時間、回報完成；不重複報火警
  ctx->config.fire_divert_on_sight = true;
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 30, 0, 1.0, 1);
  ASSERT_TRUE(runUntil([&] {return platform.v.position.x > 8.0;}, 20.0));
  ASSERT_LT(ctx->arrival, 0.0);
  const Vec3 fire_at{12.0, 0.4, 0.3};
  ASSERT_TRUE(runUntil([&] {
      platform.fires = {fire(101, 0.9, 8, fire_at)};
      return !platform.created.empty();
    }, 5.0));
  ASSERT_EQ(platform.created.size(), 2u);
  EXPECT_EQ(platform.created[0].type, kTaskGroundIntervention);
  EXPECT_NEAR(platform.created[0].position.x, 12.0, 1e-9);
  EXPECT_EQ(platform.created[1].type, kTaskAirRecon);
  EXPECT_DOUBLE_EQ(platform.created[1].duration, 20.0);
  ASSERT_EQ(platform.results.size(), 1u);
  EXPECT_FALSE(platform.results[0].success);
  EXPECT_EQ(platform.results[0].detail, "FIRE_FOUND");

  // cbba_node 還沒撤下之前：不再往 (30, 0) 飛
  const double x_stop = platform.v.position.x;
  run(2.0);
  EXPECT_LT(platform.v.position.x, x_stop + 2.0);
  EXPECT_EQ(platform.results.size(), 1u);

  // CBBA 派下監控任務（離火點最近、空著的是自己）
  assigned = task(platform.created[1].task_id, fire_at.x, fire_at.y, 20.0, 1);
  ASSERT_TRUE(runUntil([&] {
      platform.fires = {fire(101, 0.9, 8, fire_at)};
      return platform.results.size() == 2;
    }, 60.0));
  EXPECT_TRUE(platform.results[1].success);
  EXPECT_NEAR(platform.v.position.x, 12.0, 1.0);
  EXPECT_EQ(platform.created.size(), 2u);              // 同一個火點不再報
}

TEST_F(TreeTest, FireAtDestinationIsJudgedOnArrival)
{
  // 被派去火警點（目的地就是火點）：途中就看得到 tag 101，但不走「路過發現」（不建立監控任務、不放下這個任務），
  // 到達、懸停穩定後才判定 → 一個地面處置任務，停留後回報完成
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 20, 0, 3.0, 1);
  ASSERT_TRUE(runUntil([&] {
      platform.fires = {fire(101, 0.9, 8, {20.3, -0.2, 0.3})};
      return !platform.results.empty();
    }, 40.0));
  EXPECT_TRUE(platform.results[0].success);
  ASSERT_EQ(platform.created.size(), 1u);
  EXPECT_EQ(platform.created[0].type, kTaskGroundIntervention);
}

TEST_F(TreeTest, WrongTagOrLowConfidenceIsNotFire)
{
  // 不是 101 的 tag、confidence 不到 0.85、張數太少、太舊：都不算火警，觀察 3 s 後照常完成
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 10, 0, 1.0, 1);
  ASSERT_TRUE(runUntil([&] {return ctx->arrival >= 0.0;}, 15.0));
  FireObservation old = fire(101, 0.99, 20);
  old.received = platform.t - 2.0;
  platform.fires = {fire(7, 0.99, 20), fire(101, 0.80, 20), old};
  const double start = platform.t;
  ASSERT_TRUE(runUntil([&] {
      platform.fires[1].received = platform.t;   // 一直有在更新，但 confidence 不夠
      return !platform.results.empty();
    }, 10.0));
  EXPECT_TRUE(platform.created.empty());
  EXPECT_TRUE(platform.results[0].success);
  EXPECT_GE(platform.t - start, 2.9);   // 觀察時間（3 s）比停留時間（1 s）長
  platform.fires = {fire(101, 0.99, 3)};   // 張數太少
  assigned = task(0x00010002, 0, 10, 0.0, 1);
  ASSERT_TRUE(runUntil([&] {return platform.results.size() == 2;}, 20.0));
  EXPECT_TRUE(platform.created.empty());
}

TEST_F(TreeTest, ReassignmentInterruptsWithoutReporting)
{
  // 飛行中換成別的任務：舊的不回報，改飛新的；同一個任務的新版本也要重新開始，回報帶新版本
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 20, 0, 1.0, 1);
  run(1.5);
  assigned = task(0x00010002, 0, -10, 1.0, 1);
  run(0.5);
  EXPECT_EQ(platform.exec.active_task, 0x00010002u);
  assigned = task(0x00010002, 0, -10, 1.0, 2);
  ASSERT_TRUE(runUntil([&] {return !platform.results.empty();}, 20.0));
  ASSERT_EQ(platform.results.size(), 1u);
  EXPECT_EQ(platform.results[0].task_id, 0x00010002u);
  EXPECT_EQ(platform.results[0].version, 2u);
  EXPECT_NEAR(platform.v.position.y, -10.0, 0.6);
}

TEST_F(TreeTest, AssignmentRemovedStopsAndLoiters)
{
  // 指派變成沒有（task_id = 0）：停在原地，不回報
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 20, 0, 1.0, 1);
  run(2.0);
  assigned = AssignedTask{};
  run(1.0);
  const double x = platform.v.position.x;
  run(3.0);
  EXPECT_NEAR(platform.v.position.x, x, 1.0);
  EXPECT_TRUE(platform.results.empty());
  EXPECT_EQ(platform.exec.mode, ExecMode::IDLE);
}

TEST_F(TreeTest, BatteryCriticalReturnsToLaunch)
{
  // 執行中電量降到 20 %：回報任務失敗（BATTERY_RTL）、執行狀態故障、要求 PX4 返航；之後的指派不再執行
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  assigned = task(0x00010001, 20, 0, 1.0, 3);
  run(1.0);
  platform.v.battery = 19.5;
  run(0.2);
  ASSERT_EQ(platform.results.size(), 1u);
  EXPECT_FALSE(platform.results[0].success);
  EXPECT_EQ(platform.results[0].detail, "BATTERY_RTL");
  EXPECT_EQ(platform.results[0].version, 3u);
  EXPECT_EQ(platform.exec.mode, ExecMode::FAULT);
  EXPECT_EQ(platform.rtl_requests, 1);
  EXPECT_FALSE(platform.offboard);
  // 電量讀數跳回來也不恢復；新指派不執行；返航指令最多重送到 3 次
  platform.v.battery = 25.0;
  assigned = task(0x00010002, 0, 10, 1.0, 1);
  run(10.0);
  EXPECT_EQ(platform.results.size(), 1u);
  EXPECT_EQ(platform.exec.mode, ExecMode::FAULT);
  EXPECT_EQ(platform.rtl_requests, 3);
}

TEST_F(TreeTest, FlyTimeoutReportsFailure)
{
  // 飛不到（卡住）：距離 / 速度 × 2 + 30 s 後回報失敗 FLY_TIMEOUT，任務交回競標池
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  platform.stuck = true;
  assigned = task(0x00010001, 10, 0, 1.0, 1);
  run(30.0);
  EXPECT_TRUE(platform.results.empty());
  ASSERT_TRUE(runUntil([&] {return !platform.results.empty();}, 10.0));
  EXPECT_FALSE(platform.results[0].success);
  EXPECT_EQ(platform.results[0].detail, "FLY_TIMEOUT");
}

TEST_F(TreeTest, LoiterYieldsToBusyNeighbor)
{
  // 待命時，執行任務中的鄰機在 1.5 m 外、1 m/s 飛過來：讓開、在新位置待命（鄰機走了也不回原點）；
  // 待命的鄰機（不是執行中）3 m 外懸停：CBF 不推，不改待命點（2026-10-14 CPF → CBF：只限制靠近的速度）
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  run(1.0);
  const Vec3 start = platform.v.position;
  platform.nbrs = {Neighbor{2, start + Vec3{1.5, 0, 0}, {-1.0, 0, 0}, 0.05, true}};
  run(6.0);
  const double moved = norm(platform.v.position - start);
  EXPECT_GT(moved, 0.4);
  platform.nbrs.clear();
  run(3.0);
  EXPECT_NEAR(norm(platform.v.position - start), moved, 0.5);

  const Vec3 here = platform.v.position;
  platform.nbrs = {Neighbor{3, here + Vec3{0, 3.0, 0}, {}, 0.05, false}};
  run(6.0);
  platform.nbrs.clear();
  run(6.0);
  EXPECT_LT(norm(platform.v.position - here), 0.3);
}
