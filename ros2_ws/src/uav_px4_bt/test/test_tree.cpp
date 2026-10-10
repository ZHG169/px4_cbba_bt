// 整棵備用 BT（trees/main_uav_tree.xml）用假的平台跑：起飛、執行指派、火警判定與廣播、換任務、返航、逾時。
// 假平台：速度一階追隨命令（時間常數 0.3 s），要求 offboard＋解鎖後下一個 tick 就成立；20 Hz tick。
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "uav_px4_bt/bt_nodes.hpp"

using namespace uav_px4_bt;

namespace
{
struct Result
{
  std::uint32_t task_id;
  std::uint64_t version;
  bool success;
  std::string detail;
};

struct Created
{
  std::uint8_t type;
  Vec3 position;
};

class FakePlatform : public UavPlatform
{
public:
  double timeNow() const override {return t;}
  VehicleState vehicle() const override {return v;}
  std::vector<Neighbor> neighbors() const override {return nbrs;}
  std::vector<FireObservation> fireObservations() const override {return fires;}
  void setVelocity(const Vec3 & cmd) override {velocity_cmd = cmd;}
  void requestOffboardAndArm() override {++arm_requests; v.armed = true; v.offboard = true;}
  void requestReturnToLaunch() override {++rtl_requests; v.offboard = false;}
  void stopOffboard() override {offboard = false;}
  void reportResult(const AssignedTask & task, bool success, const std::string & detail) override
  {
    results.push_back({task.task_id, task.version, success, detail});
  }
  std::uint32_t createTask(std::uint8_t type, const Vec3 & p, double, double, double) override
  {
    created.push_back({type, p});
    return 0x00018000u + static_cast<std::uint32_t>(created.size());
  }
  void setExecStatus(const ExecStatus & s) override {exec = s;}
  void log(const std::string &) override {}

  // 一個 tick 之後的動態
  void advance(double dt)
  {
    if (v.armed && v.offboard && !stuck) {
      v.velocity = v.velocity + (velocity_cmd - v.velocity) * (dt / 0.3);
      v.position = v.position + v.velocity * dt;
      v.landed = v.position.z < 0.1;
    }
    t += dt;
  }

  double t{100.0};
  VehicleState v;
  Vec3 velocity_cmd;
  bool stuck{false};
  bool offboard{true};
  int arm_requests{0};
  int rtl_requests{0};
  std::vector<FireObservation> fires;
  std::vector<Neighbor> nbrs;
  std::vector<Result> results;
  std::vector<Created> created;
  ExecStatus exec;
};

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

  FireObservation fire(std::uint32_t tag, double confidence, int frames)
  {
    FireObservation o;
    o.tag_id = tag;
    o.confidence = confidence;
    o.frames = frames;
    o.position = {10.3, -0.2, 0.3};
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
  // 指派 → 前往（NAVIGATING）→ 到達後觀察、停留（EXECUTING）→ 回報完成（帶 assignment_version）→ 閒置
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
  // 同一個 tag 只報一次
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
  // 待命時，執行任務中的鄰機停在 1.5 m 外：被推開、在新位置待命（鄰機走了也不回原點）；
  // 待命的鄰機（不是執行中）只會互相推到平衡點，不改待命點
  ASSERT_TRUE(runUntil([&] {return ctx->airborne;}, 15.0));
  run(1.0);
  const Vec3 start = platform.v.position;
  platform.nbrs = {Neighbor{2, start + Vec3{1.5, 0, 0}, {}, 0.05, true}};
  run(6.0);
  const double moved = norm(platform.v.position - start);
  EXPECT_GT(moved, 3.0);
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
