// 共用表頭與轉送表頭：位元組排列與規格書 §2.1、§2.2 一致、拒絕不合法的表頭、seq 過濾、轉送與去重複
#include <gtest/gtest.h>

#include <algorithm>

#include "uav_cbba/wire_header.hpp"

using namespace uav_cbba::wire;

namespace
{
Header makeHeader(PacketType type = PacketType::AGENT_STATE, std::uint8_t agent = 3,
  std::uint32_t seq = 0x01020304)
{
  Header h;
  h.type = type;
  h.agent_id = agent;
  h.seq = seq;
  h.stamp_us = 0x1122334455667788ULL;
  return h;
}
}  // namespace

TEST(WireHeader, LayoutMatchesSpec)
{
  std::vector<std::uint8_t> out;
  appendHeader(out, makeHeader());
  const std::vector<std::uint8_t> expected = {
    0x22,                                            // 代碼 2 | 版本 1 << 5
    0x03,                                            // agent_id
    0x04, 0x03, 0x02, 0x01,                          // seq（小端序）
    0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,  // stamp_us（小端序）
  };
  EXPECT_EQ(out, expected);
  EXPECT_EQ(out.size(), kHeaderSize);
}

TEST(WireHeader, TypeBytesOfAllPackets)
{
  // 規格書 §1 的「type 位元組」欄
  EXPECT_EQ(typeByte(PacketType::FORMATION), 0x21);
  EXPECT_EQ(typeByte(PacketType::AGENT_STATE), 0x22);
  EXPECT_EQ(typeByte(PacketType::ELECTION), 0x23);
  EXPECT_EQ(typeByte(PacketType::TASK_EVENT), 0x24);
  EXPECT_EQ(typeByte(PacketType::LOST_EVENT), 0x25);
  EXPECT_EQ(typeByte(PacketType::COMPLETION), 0x26);
  EXPECT_EQ(typeByte(PacketType::DOWNLINK), 0x27);
  EXPECT_EQ(typeByte(PacketType::COMPLETION_ACK), 0x28);
  EXPECT_EQ(typeByte(PacketType::FORMATION_RELAY), 0x29);
}

TEST(WireHeader, RoundTripAndIgnoresBody)
{
  std::vector<std::uint8_t> bytes;
  appendHeader(bytes, makeHeader(PacketType::COMPLETION_ACK, 5, 42));
  bytes.insert(bytes.end(), {0xAA, 0xBB, 0xCC});   // 後面接內容不影響表頭
  const auto h = parseHeader(bytes);
  ASSERT_TRUE(h);
  EXPECT_EQ(h->type, PacketType::COMPLETION_ACK);
  EXPECT_EQ(h->version, kVersion);
  EXPECT_EQ(h->agent_id, 5);
  EXPECT_EQ(h->seq, 42u);
  EXPECT_EQ(h->stamp_us, 0x1122334455667788ULL);
}

TEST(WireHeader, RejectsInvalid)
{
  std::vector<std::uint8_t> good;
  appendHeader(good, makeHeader());

  EXPECT_FALSE(parseHeader(good.data(), kHeaderSize - 1));   // 長度不足
  EXPECT_FALSE(parseHeader(nullptr, 0));

  auto bad = good;
  bad[0] = typeByte(PacketType::AGENT_STATE, 2);   // 版本 2 只有 TASK_EVENT 能用
  EXPECT_FALSE(parseHeader(bad));
  bad[0] = typeByte(PacketType::TASK_EVENT, 3);    // 沒有版本 3
  EXPECT_FALSE(parseHeader(bad));
  bad[0] = typeByte(PacketType::TASK_EVENT, 2);
  ASSERT_TRUE(parseHeader(bad));
  EXPECT_EQ(parseHeader(bad)->version, 2);
  bad[0] = 0x20;                                   // 代碼 0
  EXPECT_FALSE(parseHeader(bad));
  bad[0] = 0x20 | 10;                              // 代碼 10（規格只到 9）
  EXPECT_FALSE(parseHeader(bad));

  bad = good;
  bad[1] = 0;                                      // agent_id 0 保留為「無人」
  EXPECT_FALSE(parseHeader(bad));
}

TEST(WireHeader, StampUs)
{
  EXPECT_EQ(toStampUs(1.5), 1500000u);
  EXPECT_EQ(toStampUs(1791276182.813642), 1791276182813642u);
}

TEST(SeqCounter, PerPacketType)
{
  SeqCounter c;
  EXPECT_EQ(c.next(PacketType::AGENT_STATE), 1u);
  EXPECT_EQ(c.next(PacketType::AGENT_STATE), 2u);
  EXPECT_EQ(c.next(PacketType::TASK_EVENT), 1u);   // 每種封包各自遞增
  EXPECT_EQ(c.next(PacketType::AGENT_STATE), 3u);
}

TEST(SeqFilter, DropsOldAndDuplicate)
{
  SeqFilter f;
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 10), 0.0));
  EXPECT_FALSE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 10), 0.1));   // 重複
  EXPECT_FALSE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 9), 0.1));    // 亂序的舊封包
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 12), 0.2));    // 跳號沒關係
}

TEST(SeqFilter, SeparatesSenderAndType)
{
  SeqFilter f;
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 5), 0.0));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::TASK_EVENT, 2, 5), 0.0));   // 同一台、不同種類
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 3, 5), 0.0));  // 同種類、不同台
}

TEST(SeqFilter, AcceptsRestartedSenderAfterTimeout)
{
  SeqFilter f(3.0);
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 500), 10.0));
  // 重開機後 seq 從 1 重算：3 s 內仍當作舊封包
  EXPECT_FALSE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 1), 12.0));
  // 超過 3 s 沒收到可接受的封包：視為重開機，接受較小的 seq
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 2), 13.5));
  EXPECT_FALSE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 2), 13.6));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 3), 13.7));
}

TEST(SeqCounter, StartsAfterGivenValue)
{
  // 開機時以時鐘當起點：每種封包都從 start + 1 開始
  SeqCounter c(1000);
  EXPECT_EQ(c.next(PacketType::AGENT_STATE), 1001u);
  EXPECT_EQ(c.next(PacketType::AGENT_STATE), 1002u);
  EXPECT_EQ(c.next(PacketType::TASK_EVENT), 1001u);
}

TEST(SeqFilter, HandlesWrapAround)
{
  // seq 繞回 0：循環比較下 0、1 比 0xFFFFFFFE 新，不會誤丟
  SeqFilter f(3.0);
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 0xFFFFFFFEu), 10.0));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 0xFFFFFFFFu), 10.1));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 0u), 10.2));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 1u), 10.3));
  EXPECT_FALSE(f.accept(makeHeader(PacketType::AGENT_STATE, 2, 0xFFFFFFFFu), 10.4));   // 舊的
}

// ---------------------------------------------------------------------------
// 轉送表頭（§2.2）
// ---------------------------------------------------------------------------
namespace
{
// 一個有轉送表頭的封包：共用表頭 + 轉送表頭 + 3 B 假內容
std::vector<std::uint8_t> relayPacket(PacketType type, std::uint8_t sender, std::uint32_t seq,
  RelayHeader relay)
{
  std::vector<std::uint8_t> out;
  appendHeader(out, makeHeader(type, sender, seq));
  appendRelayHeader(out, relay);
  out.insert(out.end(), {0xAA, 0xBB, 0xCC});
  return out;
}
}  // namespace

TEST(RelayHeader, LayoutMatchesSpec)
{
  std::vector<std::uint8_t> out;
  appendRelayHeader(out, RelayHeader{7, 0x0102030405060708ULL});
  const std::vector<std::uint8_t> expected = {
    0x07,                                            // origin_id
    0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,  // origin_seq（小端序，8 B）
  };
  EXPECT_EQ(out, expected);
  EXPECT_EQ(out.size(), kRelayHeaderSize);
}

TEST(RelayHeader, OnlyEventPacketsHaveIt)
{
  // 規格 §2.2：TASK_EVENT、LOST_EVENT、COMPLETION、COMPLETION_ACK、FORMATION_RELAY
  EXPECT_FALSE(hasRelayHeader(PacketType::FORMATION));
  EXPECT_FALSE(hasRelayHeader(PacketType::AGENT_STATE));
  EXPECT_FALSE(hasRelayHeader(PacketType::ELECTION));
  EXPECT_TRUE(hasRelayHeader(PacketType::TASK_EVENT));
  EXPECT_TRUE(hasRelayHeader(PacketType::LOST_EVENT));
  EXPECT_TRUE(hasRelayHeader(PacketType::COMPLETION));
  EXPECT_FALSE(hasRelayHeader(PacketType::DOWNLINK));
  EXPECT_TRUE(hasRelayHeader(PacketType::COMPLETION_ACK));
  EXPECT_TRUE(hasRelayHeader(PacketType::FORMATION_RELAY));
}

TEST(RelayHeader, Parse)
{
  const auto packet = relayPacket(PacketType::TASK_EVENT, 2, 9, RelayHeader{4, 77});
  const auto r = parseRelayHeader(packet);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->origin_id, 4);
  EXPECT_EQ(r->origin_seq, 77u);

  auto bad = packet;
  bad[kHeaderSize] = 0;                                         // origin_id 0
  EXPECT_FALSE(parseRelayHeader(bad));
  EXPECT_FALSE(parseRelayHeader(packet.data(), kHeaderSize + kRelayHeaderSize - 1));   // 太短

  std::vector<std::uint8_t> state;
  appendHeader(state, makeHeader(PacketType::AGENT_STATE));
  state.resize(40, 0x01);
  EXPECT_FALSE(parseRelayHeader(state));                        // AGENT_STATE 沒有轉送表頭
}

TEST(RelayHeader, OriginOfOwnEvent)
{
  const RelayHeader r = originOf(makeHeader(PacketType::COMPLETION, 3, 15));
  EXPECT_EQ(r.origin_id, 3);
  EXPECT_EQ(r.origin_seq, 15u);
}

TEST(RelayHeader, MakeRelayKeepsOriginAndBody)
{
  const auto packet = relayPacket(PacketType::COMPLETION, 4, 30, RelayHeader{4, 30});
  SeqCounter mine;
  mine.next(PacketType::COMPLETION);   // 自己先送過一則 COMPLETION
  const auto fwd = makeRelay(packet, 2, mine, 1.5);
  ASSERT_TRUE(fwd);
  ASSERT_EQ(fwd->size(), packet.size());

  const auto h = parseHeader(*fwd);
  ASSERT_TRUE(h);
  EXPECT_EQ(h->type, PacketType::COMPLETION);
  EXPECT_EQ(h->agent_id, 2);           // 換成轉送者
  EXPECT_EQ(h->seq, 2u);               // 轉送者自己的 COMPLETION 流水號
  EXPECT_EQ(h->stamp_us, 1500000u);
  // 轉送表頭與內容原封不動
  EXPECT_TRUE(std::equal(packet.begin() + kHeaderSize, packet.end(), fwd->begin() + kHeaderSize));

  std::vector<std::uint8_t> state;
  appendHeader(state, makeHeader(PacketType::AGENT_STATE));
  state.resize(40, 0x01);
  EXPECT_FALSE(makeRelay(state, 2, mine, 1.5));   // AGENT_STATE 不轉送
}

TEST(RelayDeduper, FirstSeenOnly)
{
  RelayDeduper d;
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_EVENT, {4, 5}, 0.0));
  EXPECT_FALSE(d.firstSeen(PacketType::TASK_EVENT, {4, 5}, 0.1));   // 經別條路徑轉送來的同一則
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_EVENT, {4, 6}, 0.1));
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_EVENT, {3, 5}, 0.1));
}

TEST(RelayDeduper, KeyIncludesPacketType)
{
  // 規格只用 (origin_id, origin_seq)：同一台的第 5 則 TASK_EVENT 和第 5 則 COMPLETION 會撞在一起
  RelayDeduper d;
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_EVENT, {4, 5}, 0.0));
  EXPECT_TRUE(d.firstSeen(PacketType::COMPLETION, {4, 5}, 0.0));
}

TEST(RelayDeduper, ForgetsAfterMemory)
{
  RelayDeduper d(60.0);
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_EVENT, {4, 5}, 0.0));
  EXPECT_FALSE(d.firstSeen(PacketType::TASK_EVENT, {4, 5}, 59.0));
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_EVENT, {4, 5}, 61.0));   // 從第一次算起超過 60 s
  EXPECT_FALSE(d.firstSeen(PacketType::TASK_EVENT, {4, 5}, 62.0));

  // 清理：過期的紀錄會被移除，記憶體不會一直增加
  for (std::uint64_t k = 0; k < 100; ++k) {
    d.firstSeen(PacketType::TASK_EVENT, {7, k}, 100.0);
  }
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_EVENT, {8, 1}, 200.0));
  EXPECT_EQ(d.size(), 1u);
}

TEST(RelayDeduper, FloodRelaysOncePerNode)
{
  // 1 — 2 — 3 — 4 排成一列：uav1 發出的事件，每台只處理、轉送一次，最後每台都收到
  const std::uint8_t ids[4] = {1, 2, 3, 4};
  RelayDeduper dedup[4];
  SeqCounter counters[4];
  int handled[4] = {0, 0, 0, 0};
  int relayed[4] = {0, 0, 0, 0};

  Header h = makeHeader(PacketType::TASK_EVENT, 1, counters[0].next(PacketType::TASK_EVENT));
  std::vector<std::uint8_t> first;
  appendHeader(first, h);
  appendRelayHeader(first, originOf(h));
  dedup[0].firstSeen(PacketType::TASK_EVENT, originOf(h), 0.0);   // 自己發的先登記

  // (目標, 封包)；每則廣播送給左右鄰居
  std::vector<std::pair<int, std::vector<std::uint8_t>>> queue = {{1, first}};
  while (!queue.empty()) {
    auto [to, packet] = queue.front();
    queue.erase(queue.begin());
    const auto r = parseRelayHeader(packet);
    ASSERT_TRUE(r);
    if (!dedup[to].firstSeen(PacketType::TASK_EVENT, *r, 0.0)) {
      continue;
    }
    ++handled[to];
    const auto fwd = makeRelay(packet, ids[to], counters[to], 0.0);
    ASSERT_TRUE(fwd);
    ++relayed[to];
    for (int n : {to - 1, to + 1}) {
      if (n >= 0 && n < 4) {
        queue.emplace_back(n, *fwd);
      }
    }
  }
  EXPECT_EQ(handled[0], 0);   // 自己發的不再處理
  for (int i = 1; i < 4; ++i) {
    EXPECT_EQ(handled[i], 1) << i;
    EXPECT_EQ(relayed[i], 1) << i;
  }
}
