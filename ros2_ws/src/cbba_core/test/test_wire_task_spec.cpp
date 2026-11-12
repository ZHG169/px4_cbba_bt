// TASK_SPEC（協定版本 4，2026-11-12）：大小、位元組排列、來回編解碼、轉送、切段、CRC、拒絕不合法的封包
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "cbba_core/wire_task_spec.hpp"

using namespace cbba_core::wire;

namespace
{
Header makeHeader()
{
  Header h;
  h.sender_id = 200;
  h.session_id = 0xAB;
  h.sequence = 4;
  return h;
}

TaskSpecBody sample()
{
  TaskSpecBody b;
  b.relay = RelayHeader{200, 0xAB, 4};
  b.task_id = 0x00C8000300000011ULL;
  b.revision = 2;
  b.encoding = kSpecEncodingJson;
  b.part_index = 1;
  b.part_count = 3;
  b.data = {'{', '}', '\n'};
  return b;
}
}  // namespace

TEST(WireTaskSpec, SizeAndLayout)
{
  // 表頭 20 + 轉送 14 + task_id 8 + revision 2 + encoding 1 + 第幾段 1 + 共幾段 1 + 長度 2 + 資料
  const auto bytes = encodeTaskSpec(makeHeader(), sample());
  ASSERT_EQ(bytes.size(), taskSpecSize(3));
  EXPECT_EQ(taskSpecSize(0), 49u);
  EXPECT_EQ(parseHeader(bytes)->type, PacketType::TASK_SPEC);
  const std::vector<std::uint8_t> body(bytes.begin() + kHeaderSize + kRelaySize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x00, 0xC8, 0x00, 0x03, 0x00, 0x00, 0x00, 0x11,  // task_id
    0x00, 0x02,              // revision
    0x00,                    // encoding：原文 JSON
    0x01, 0x03,              // part_index、part_count
    0x00, 0x03,              // data_length
    '{', '}', '\n',
  };
  EXPECT_EQ(body, expected);
}

TEST(WireTaskSpec, RoundTripAndRelay)
{
  const auto original = encodeTaskSpec(makeHeader(), sample());
  const auto d = decodeTaskSpec(original);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->task_id, 0x00C8000300000011ULL);
  EXPECT_EQ(d->revision, 2);
  EXPECT_EQ(d->part_index, 1);
  EXPECT_EQ(d->part_count, 3);
  EXPECT_EQ(d->data, sample().data);
  // 轉送：換表頭，內容（含 origin）不變
  SeqCounter counter;
  const auto fwd = makeRelay(original, 7, 0x77, counter);
  ASSERT_TRUE(fwd);
  const auto r = decodeTaskSpec(*fwd);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->relay, sample().relay);
  EXPECT_EQ(parseHeader(*fwd)->sender_id, 7);
}

TEST(WireTaskSpec, UnknownEncodingStillDecodes)
{
  // zlib（1）保留、暫不用：解得開，收的一方當成缺 TaskSpec（之後加新編碼不用升協定）
  TaskSpecBody b = sample();
  b.encoding = kSpecEncodingZlib;
  const auto d = decodeTaskSpec(encodeTaskSpec(makeHeader(), b));
  ASSERT_TRUE(d);
  EXPECT_EQ(d->encoding, kSpecEncodingZlib);
}

TEST(WireTaskSpec, SplitAndCrc)
{
  // 每段 ≤ 1100 B，接回去等於原文；CRC-32 是標準的 IEEE 802.3（"123456789" → CBF43926）
  const std::string spec(2500, 'x');
  const auto parts = splitSpec(spec);
  ASSERT_EQ(parts.size(), 3u);
  EXPECT_EQ(parts[0].size(), kMaxSpecChunk);
  EXPECT_EQ(parts[2].size(), 2500u - 2 * kMaxSpecChunk);
  std::string joined;
  for (const auto & p : parts) {
    joined.append(p.begin(), p.end());
  }
  EXPECT_EQ(joined, spec);
  EXPECT_TRUE(splitSpec("").empty());
  EXPECT_EQ(specCrc("123456789"), 0xCBF43926u);
  EXPECT_THROW(splitSpec(std::string(kMaxSpecChunk * kMaxSpecParts + 1, 'x')), std::invalid_argument);
}

TEST(WireTaskSpec, RejectsInvalid)
{
  TaskSpecBody b = sample();
  b.part_index = 3;   // ≥ part_count
  EXPECT_THROW(encodeTaskSpec(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.part_count = 0;
  b.part_index = 0;
  EXPECT_THROW(encodeTaskSpec(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.data.clear();
  EXPECT_THROW(encodeTaskSpec(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.data.assign(kMaxSpecChunk + 1, 'x');
  EXPECT_THROW(encodeTaskSpec(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.task_id = 0x00C8000000000001ULL;   // 版本 4、世代 0
  EXPECT_THROW(encodeTaskSpec(makeHeader(), b), std::invalid_argument);

  const auto good = encodeTaskSpec(makeHeader(), sample());
  auto bad = good;
  bad.pop_back();
  finishPacket(bad);   // 長度和 data_length 對不上
  EXPECT_FALSE(decodeTaskSpec(bad));
  bad = good;
  bad[kHeaderSize + kRelaySize + 11] = 5;   // part_index ≥ part_count
  EXPECT_FALSE(decodeTaskSpec(bad));
}
