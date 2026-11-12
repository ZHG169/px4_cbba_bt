// 補發的兩種封包（協定版本 3 起；版本 4 的 GAP_REQUEST 帶世代、流水號 4 B）：大小、位元組排列、來回編解碼、拒絕不合法的封包
#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "cbba_core/wire_completion.hpp"
#include "cbba_core/wire_sync.hpp"
#include "cbba_core/wire_task_announce.hpp"

using namespace cbba_core::wire;

namespace
{
Header makeHeader()
{
  Header h;
  h.sender_id = 2;
  h.session_id = 0x55;
  h.sequence = 7;
  return h;
}

TaskAnnounceBody announce()
{
  TaskAnnounceBody b;
  b.relay = RelayHeader{1, 0x11, 4};
  b.task_id = 0x00010005;
  b.task_type = 1;
  b.x = 1.5f;
  b.value = 80.0f;
  b.created_ms = 1000;
  b.duration_sec = 5;
  return b;
}

TaskCloseBody closeBody()
{
  TaskCloseBody b;
  b.relay = RelayHeader{3, 0x33, 9};
  b.task_id = 0x00010005;
  b.reason = kCloseDone;
  b.actor_id = 3;
  b.assign_version = 4;
  b.view = {1, 2, 3};
  b.acks = {AckRef{1, 11}, AckRef{2, 12}};
  return b;
}
}  // namespace

TEST(WireGapRequest, SizeAndLayout)
{
  // 20 + 12 + ⌈count / 4⌉；點陣圖每個流水號 2 bit、高位在前，沒用到的位元填 0
  EXPECT_EQ(gapRequestSize(1), 33u);
  EXPECT_EQ(gapRequestSize(4), 33u);
  EXPECT_EQ(gapRequestSize(5), 34u);
  EXPECT_EQ(gapRequestSize(kMaxGapSeqs), 32u + 1000u);

  GapRequestBody b;
  b.target_id = 0x0003;
  b.creator = 0x0032;
  b.epoch = 0x0004;
  b.first_seq = 0x00010100;
  b.states = {SeqState::OPEN, SeqState::CLOSED, SeqState::UNKNOWN, SeqState::OPEN, SeqState::CLOSED};
  const auto bytes = encodeGapRequest(makeHeader(), b);
  ASSERT_EQ(bytes.size(), gapRequestSize(5));
  EXPECT_EQ(parseHeader(bytes)->type, PacketType::GAP_REQUEST);
  const std::vector<std::uint8_t> body(bytes.begin() + kHeaderSize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x00, 0x03,   // target_id
    0x00, 0x32,   // creator
    0x00, 0x04,   // epoch
    0x00, 0x01, 0x01, 0x00,   // first_seq（uint32）
    0x00, 0x05,   // count
    0x61,         // 01 10 00 01：進行中、已結束、不認得、進行中
    0x80,         // 10 00 00 00：已結束，其餘填 0
  };
  EXPECT_EQ(body, expected);
}

TEST(WireGapRequest, RoundTrip)
{
  GapRequestBody b;
  b.target_id = 50;
  b.creator = 1;
  b.epoch = 2;
  b.first_seq = 0x12345678;
  for (int i = 0; i < 37; ++i) {
    b.states.push_back(static_cast<SeqState>(i % 3));
  }
  const auto d = decodeGapRequest(encodeGapRequest(makeHeader(), b));
  ASSERT_TRUE(d);
  EXPECT_EQ(d->target_id, 50);
  EXPECT_EQ(d->creator, 1);
  EXPECT_EQ(d->epoch, 2);
  EXPECT_EQ(d->first_seq, 0x12345678u);
  EXPECT_EQ(d->states, b.states);
}

TEST(WireGapRequest, RejectsInvalid)
{
  GapRequestBody b;
  b.target_id = 3;
  b.creator = 1;
  b.first_seq = 0;
  b.states = {SeqState::OPEN};
  const auto good = encodeGapRequest(makeHeader(), b);
  ASSERT_TRUE(decodeGapRequest(good));

  // 編碼：筆數 0、超過上限、世代 0 的流水號超過 65535、版本 4 的超過 0xFFFFFFFF、機號不合法
  GapRequestBody bad_body = b;
  bad_body.states.clear();
  EXPECT_THROW(encodeGapRequest(makeHeader(), bad_body), std::invalid_argument);
  bad_body.states.assign(kMaxGapSeqs + 1, SeqState::OPEN);
  EXPECT_THROW(encodeGapRequest(makeHeader(), bad_body), std::invalid_argument);
  bad_body = b;
  bad_body.first_seq = 0xFFFF;
  bad_body.states = {SeqState::OPEN, SeqState::OPEN};
  EXPECT_THROW(encodeGapRequest(makeHeader(), bad_body), std::invalid_argument);
  bad_body.epoch = 1;   // 世代 ≥ 1 時同樣的範圍可以
  EXPECT_NO_THROW(encodeGapRequest(makeHeader(), bad_body));
  bad_body.first_seq = 0xFFFFFFFFu;
  EXPECT_THROW(encodeGapRequest(makeHeader(), bad_body), std::invalid_argument);
  bad_body = b;
  bad_body.creator = 0;
  EXPECT_THROW(encodeGapRequest(makeHeader(), bad_body), std::invalid_argument);

  // 解碼：值 3 是「缺 TaskSpec」（版本 4）；沒用到的位元不是 0、長度不符
  auto bad = good;
  bad[kHeaderSize + 12] = 0xC0;   // 第一格 = 3
  const auto no_spec = decodeGapRequest(bad);
  ASSERT_TRUE(no_spec);
  EXPECT_EQ(no_spec->states[0], SeqState::OPEN_NO_SPEC);
  bad = good;
  bad[kHeaderSize + 12] = 0x41;   // 第一格 1，第四格（沒用到）= 1
  EXPECT_FALSE(decodeGapRequest(bad));
  bad = good;
  bad.push_back(0);
  finishPacket(bad);
  EXPECT_FALSE(decodeGapRequest(bad));
  bad = good;
  bad[kHeaderSize + 10] = 0;     // count = 0
  bad[kHeaderSize + 11] = 0;
  EXPECT_FALSE(decodeGapRequest(bad));
}

TEST(WireSyncBatch, ItemsKeepOriginalPayload)
{
  // 一筆 = 原本封包去掉表頭；補一個表頭後原本的解碼函式解得開、內容（含 origin）不變
  const auto a_packet = encodeTaskAnnounce(makeHeader(), announce());
  const auto c_packet = encodeTaskClose(makeHeader(), closeBody());
  const auto a_item = toSyncItem(a_packet);
  const auto c_item = toSyncItem(c_packet);
  ASSERT_TRUE(a_item);
  ASSERT_TRUE(c_item);
  EXPECT_EQ(a_item->payload.size(), a_packet.size() - kHeaderSize);

  const auto batch = encodeSyncBatch(makeHeader(), {*a_item, *c_item});
  EXPECT_EQ(batch.size(), kSyncBatchBaseSize + syncItemSize(*a_item) + syncItemSize(*c_item));
  EXPECT_EQ(parseHeader(batch)->type, PacketType::SYNC_BATCH);
  const auto items = decodeSyncBatch(batch);
  ASSERT_TRUE(items);
  ASSERT_EQ(items->size(), 2u);

  Header relayer = makeHeader();
  relayer.sender_id = 9;
  const auto a = decodeTaskAnnounce(syncItemPacket((*items)[0], relayer));
  ASSERT_TRUE(a);
  EXPECT_EQ(a->task_id, 0x00010005u);
  EXPECT_EQ(a->relay, announce().relay);
  EXPECT_FLOAT_EQ(a->x, 1.5f);
  const auto c = decodeTaskClose(syncItemPacket((*items)[1], relayer));
  ASSERT_TRUE(c);
  EXPECT_EQ(c->relay, closeBody().relay);
  EXPECT_EQ(c->actor_id, 3);
  EXPECT_EQ(c->acks.size(), 2u);
}

TEST(WireSyncBatch, RejectsInvalid)
{
  const auto item = *toSyncItem(encodeTaskAnnounce(makeHeader(), announce()));
  const auto good = encodeSyncBatch(makeHeader(), {item});
  ASSERT_TRUE(decodeSyncBatch(good));

  // 只收任務定義、結束紀錄；沒有筆數不行
  EXPECT_FALSE(toSyncItem(encodeGapRequest(makeHeader(), GapRequestBody{3, 1, 0, 0, {SeqState::OPEN}})));
  EXPECT_THROW(encodeSyncBatch(makeHeader(), {}), std::invalid_argument);
  SyncItem wrong = item;
  wrong.type = PacketType::COMPLETION;
  EXPECT_THROW(encodeSyncBatch(makeHeader(), {wrong}), std::invalid_argument);

  auto bad = good;
  bad[kHeaderSize] = 0;                 // item_count 0
  EXPECT_FALSE(decodeSyncBatch(bad));
  bad = good;
  bad[kHeaderSize + 1] = 4;             // item 種類 COMPLETION
  EXPECT_FALSE(decodeSyncBatch(bad));
  bad = good;
  bad.pop_back();                       // 內容比 length 短
  finishPacket(bad);
  EXPECT_FALSE(decodeSyncBatch(bad));
  bad = good;
  bad.push_back(0);                     // 多出來的位元組
  finishPacket(bad);
  EXPECT_FALSE(decodeSyncBatch(bad));
}
