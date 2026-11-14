// 偵測送閘道（2-1b M0 第 5 步，2026-11-14）：機器人端的佇列（合併、上限、限速、重送）、閘道去重複，
// 經過模擬網路：送達與確認、閘道斷線時暫存、恢復後送出、掉包重送、多跳、閘道本機的偵測
#include <gtest/gtest.h>

#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include "cbba_core/detection.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
constexpr AgentId kGateway = 200;

Detection fire(double x, double y, std::uint32_t track = 0, double t = 100.0)
{
  Detection d;
  d.detector = "apriltag_fire";
  d.source = "uav1/down_cam";
  d.label = "fire";
  d.t = t;
  d.score = 0.9;
  d.track_id = track;
  d.position_valid = true;
  d.position = {x, y, 0.0};
  return d;
}

std::vector<std::uint32_t> seqsOf(const std::vector<DetectionOutbox::Item> & items)
{
  std::vector<std::uint32_t> out;
  for (const auto & i : items) {
    out.push_back(i.seq);
  }
  return out;
}

// 閘道＋一台無人機＋一隻狗（全隊的 gateways 設 200）
Swarm makeSwarm(double loss = 0.0)
{
  CommConfig cc;
  cc.gateways = {kGateway};
  Swarm swarm(loss, 1, cc);
  swarm.addGateway(kGateway);
  swarm.add(1, {0, 0, 5});
  swarm.add(50, {5, 0, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  return swarm;
}
}  // namespace

// ===========================================================================
// 機器人端的佇列
// ===========================================================================
TEST(DetectionOutbox, SameTrackMerges)
{
  // 同一個偵測器、類別、track_id：新的取代舊的（偵測器 10 Hz 送同一個目標，佇列只留最新的）
  DetectionOutbox box;
  EXPECT_EQ(box.add(fire(0, 0, 5), true), DetectionOutbox::AddResult::ADDED);
  EXPECT_EQ(box.add(fire(8, 0, 5), true), DetectionOutbox::AddResult::MERGED);   // 目標移動了，照樣合併
  ASSERT_EQ(box.size(), 1u);
  EXPECT_EQ(box.items().front().seq, 2u);
  EXPECT_DOUBLE_EQ(box.items().front().detection.position.x, 8.0);
  // 不同的 track_id 不合併，就算位置很近
  EXPECT_EQ(box.add(fire(8, 0, 6), true), DetectionOutbox::AddResult::ADDED);
  EXPECT_EQ(box.size(), 2u);
}

TEST(DetectionOutbox, NearbyWithoutTrackMerges)
{
  // 沒有 track_id：同一個偵測器、類別，相距 ≤ merge_radius（2 m）合併；遠的、類別不同、一邊有 track_id 的不合併
  DetectionOutbox box;
  box.add(fire(0, 0), true);
  EXPECT_EQ(box.add(fire(1.5, 0), true), DetectionOutbox::AddResult::MERGED);
  EXPECT_EQ(box.add(fire(5, 0), true), DetectionOutbox::AddResult::ADDED);
  Detection smoke = fire(5, 0);
  smoke.label = "smoke";
  EXPECT_EQ(box.add(smoke, true), DetectionOutbox::AddResult::ADDED);
  EXPECT_EQ(box.add(fire(5, 0, 3), true), DetectionOutbox::AddResult::ADDED);
  Detection no_pos = fire(5, 0);
  no_pos.position_valid = false;
  EXPECT_EQ(box.add(no_pos, true), DetectionOutbox::AddResult::ADDED);   // 沒有位置：不能比
  EXPECT_EQ(box.size(), 5u);
}

TEST(DetectionOutbox, FullDropsOldest)
{
  DetectionOutboxConfig c;
  c.capacity = 3;
  DetectionOutbox box(c);
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(box.add(fire(10.0 * i, 0), false), DetectionOutbox::AddResult::ADDED);
  }
  EXPECT_EQ(box.add(fire(100, 0), false), DetectionOutbox::AddResult::EVICTED);
  ASSERT_EQ(box.size(), 3u);
  EXPECT_EQ(box.items().front().seq, 2u);
  EXPECT_EQ(box.bufferedCount(), 3u);   // 閘道斷線時產生的
}

TEST(DetectionOutbox, RateLimitRetryAndAck)
{
  // 每秒 2 則、突發 2 則；送過 1 s 沒確認就重送（由舊到新）；確認了就拿掉
  DetectionOutboxConfig c;
  c.rate = 2.0;
  c.burst = 2.0;
  c.retry = 1.0;
  DetectionOutbox box(c);
  for (int i = 0; i < 4; ++i) {
    box.add(fire(10.0 * i, 0), true);
  }
  EXPECT_EQ(seqsOf(box.due(0.0)), (std::vector<std::uint32_t>{1, 2}));
  EXPECT_TRUE(box.due(0.1).empty());                                  // 額度用完
  EXPECT_EQ(seqsOf(box.due(0.6)), (std::vector<std::uint32_t>{3}));    // 0.5 s 補 1 則
  EXPECT_TRUE(box.ack(1));
  EXPECT_FALSE(box.ack(1));                                           // 確認過了
  const auto again = box.due(1.55);   // 2 送過 1.55 s 還沒確認 → 重送；3 才 0.95 s；4 第一次
  EXPECT_EQ(seqsOf(again), (std::vector<std::uint32_t>{2, 4}));
  EXPECT_EQ(again.front().tries, 2);
  EXPECT_EQ(again.back().tries, 1);
}

TEST(DetectionOutbox, MergeKeepsBufferedMark)
{
  // 斷線時看到的目標，恢復後又看到：合併後仍標成暫存過（閘道要知道斷線期間就看到了）
  DetectionOutbox box;
  box.add(fire(0, 0, 7), false);
  box.add(fire(1, 0, 7), true);
  ASSERT_EQ(box.size(), 1u);
  EXPECT_TRUE(box.items().front().buffered);
}

TEST(DetectionInbox, Dedup)
{
  DetectionInbox inbox(60.0);
  EXPECT_TRUE(inbox.firstSeen(1, 0xA, 1, 0.0));
  EXPECT_FALSE(inbox.firstSeen(1, 0xA, 1, 1.0));
  EXPECT_TRUE(inbox.firstSeen(1, 0xB, 1, 1.0));   // 重開機後（新 session）流水號從 1 起
  EXPECT_TRUE(inbox.firstSeen(2, 0xA, 1, 1.0));
  EXPECT_TRUE(inbox.firstSeen(1, 0xA, 1, 100.0));   // keep_sec 之前的忘掉
}

// ===========================================================================
// 經過模擬網路
// ===========================================================================
TEST(Detection, ReachesGatewayAndIsAcked)
{
  // 無人機的偵測送到閘道（欄位不變）、閘道確認後從佇列拿掉；狗只轉送、不收
  Swarm swarm = makeSwarm();
  EXPECT_TRUE(swarm.node(1).gatewayOnline(swarm.now()));
  Detection d = fire(12.0, -4.0, 9, 1794597973.5);
  d.bbox = {1.0F, 2.0F, 3.0F, 4.0F};
  d.attrs = "{\"tag\":101}";
  ASSERT_TRUE(swarm.node(1).addDetection(d, swarm.now()));
  swarm.run(0.5);
  const auto got = swarm.node(kGateway).takeDetections();
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].agent, 1);
  EXPECT_EQ(got[0].seq, 1u);
  EXPECT_FALSE(got[0].buffered);
  EXPECT_EQ(got[0].detection.label, "fire");
  EXPECT_EQ(got[0].detection.track_id, 9u);
  EXPECT_NEAR(got[0].detection.t, 1794597973.5, 1e-3);
  EXPECT_NEAR(got[0].detection.position.y, -4.0, 1e-6);
  EXPECT_FLOAT_EQ(got[0].detection.bbox[3], 4.0F);
  EXPECT_EQ(got[0].detection.attrs, "{\"tag\":101}");
  EXPECT_EQ(swarm.node(1).detectionsPending(), 0u);
  EXPECT_EQ(swarm.node(1).stats().detections_acked, 1);
  EXPECT_TRUE(swarm.node(50).takeDetections().empty());
  // 之後也不會再送到（不重複）
  swarm.run(3.0);
  EXPECT_TRUE(swarm.node(kGateway).takeDetections().empty());
}

TEST(Detection, BufferedWhileGatewayOfflineThenFlushedInOrder)
{
  // 閘道斷線（gateway_timeout 3 s 沒聽到）：偵測只暫存、不送；恢復後由舊到新送出，標成暫存過
  Swarm swarm = makeSwarm();
  swarm.setOnline(kGateway, false);
  swarm.run(3.5);
  ASSERT_FALSE(swarm.node(1).gatewayOnline(swarm.now()));
  const int sent_before = swarm.node(1).stats().sent.count(PacketType::DETECTION) ?
    swarm.node(1).stats().sent.at(PacketType::DETECTION) : 0;
  for (int i = 0; i < 3; ++i) {
    swarm.node(1).addDetection(fire(10.0 * i, 0, 0, 100.0 + i), swarm.now());
    swarm.run(0.5);
  }
  EXPECT_EQ(swarm.node(1).detectionsBuffered(), 3u);
  const int sent_after = swarm.node(1).stats().sent.count(PacketType::DETECTION) ?
    swarm.node(1).stats().sent.at(PacketType::DETECTION) : 0;
  EXPECT_EQ(sent_after, sent_before);   // 斷線時不送

  swarm.setOnline(kGateway, true);
  swarm.run(2.0);
  auto got = swarm.node(kGateway).takeDetections();
  // 同一輪送出的到達順序看網路延遲，照流水號排
  std::sort(got.begin(), got.end(), [](const auto & a, const auto & b) {return a.seq < b.seq;});
  ASSERT_EQ(got.size(), 3u);
  for (std::size_t i = 0; i < got.size(); ++i) {
    EXPECT_EQ(got[i].seq, i + 1);
    EXPECT_TRUE(got[i].buffered);
    EXPECT_NEAR(got[i].detection.t, 100.0 + static_cast<double>(i), 1e-3);
  }
  EXPECT_EQ(swarm.node(1).detectionsPending(), 0u);
}

TEST(Detection, LossyNetworkDeliversEachOnce)
{
  // 50% 掉包：沒確認就重送，每一筆最後都送到、閘道每筆只交出一次（重複的只回確認）
  Swarm swarm = makeSwarm(0.5);
  for (int i = 0; i < 8; ++i) {
    swarm.node(1).addDetection(fire(10.0 * i, 0), swarm.now());
    swarm.node(50).addDetection(fire(10.0 * i, 50.0), swarm.now());
  }
  swarm.run(15.0);
  std::set<std::pair<AgentId, std::uint32_t>> seen;
  for (const auto & r : swarm.node(kGateway).takeDetections()) {
    EXPECT_TRUE(seen.insert({r.agent, r.seq}).second) << r.agent << " " << r.seq;
  }
  EXPECT_EQ(seen.size(), 16u);
  EXPECT_EQ(swarm.node(1).detectionsPending(), 0u);
  EXPECT_EQ(swarm.node(50).detectionsPending(), 0u);
}

TEST(Detection, MultiHopThroughNeighbor)
{
  // 無人機聽不到閘道（只聽得到狗）：在線看狗的鄰居清單，偵測經狗轉送到閘道，確認也經狗轉回來
  Swarm swarm = makeSwarm();
  swarm.cut(1, kGateway);
  swarm.run(4.0);
  EXPECT_TRUE(swarm.node(1).gatewayOnline(swarm.now()));
  swarm.node(1).addDetection(fire(3, 3), swarm.now());
  swarm.run(1.0);
  const auto got = swarm.node(kGateway).takeDetections();
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].agent, 1);
  EXPECT_EQ(swarm.node(1).detectionsPending(), 0u);
}

TEST(Detection, GatewayLocalDetection)
{
  // 閘道本機的偵測（例如固定攝影機）：不經過網路，直接交出；太長的字串拒收
  Swarm swarm = makeSwarm();
  ASSERT_TRUE(swarm.node(kGateway).addDetection(fire(1, 1), swarm.now()));
  Detection bad = fire(1, 1);
  bad.label.assign(65, 'x');
  EXPECT_FALSE(swarm.node(kGateway).addDetection(bad, swarm.now()));
  EXPECT_FALSE(swarm.node(1).addDetection(bad, swarm.now()));
  const auto got = swarm.node(kGateway).takeDetections();
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].agent, kGateway);
}
