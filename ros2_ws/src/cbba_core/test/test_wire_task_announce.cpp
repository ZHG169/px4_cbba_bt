// TASK_ANNOUNCE（協定版本 2）：大小、位元組排列、來回編解碼、轉送後不變、截止時刻、拒絕不合法的封包
#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "cbba_core/wire_task_announce.hpp"

using namespace cbba_core::wire;

namespace
{
Header makeHeader(std::uint16_t sender = 3, std::uint32_t seq = 5)
{
  Header h;
  h.sender_id = sender;
  h.session_id = 0x1234;
  h.sequence = seq;
  return h;
}

TaskAnnounceBody sample()
{
  TaskAnnounceBody b;
  b.relay = RelayHeader{3, 0x1234, 5};
  b.task_id = 0x00320007;
  b.task_type = 2;
  b.x = 6.0f;
  b.y = 15.0f;
  b.z = 0.0f;
  b.value = 100.0f;
  b.created_ms = 0x01020304;
  b.deadline_ms = 123456789;
  b.duration_sec = 20;
  return b;
}
}  // namespace

TEST(WireTaskAnnounce, SizeAndLayout)
{
  const auto bytes = encodeTaskAnnounce(makeHeader(), sample());
  ASSERT_EQ(bytes.size(), 65u);
  EXPECT_EQ(kTaskAnnounceSize, 65u);
  const std::vector<std::uint8_t> body(bytes.begin() + kHeaderSize + kRelaySize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x00, 0x32, 0x00, 0x07,  // task_id
    0x02,                    // 地面處置
    0x40, 0xC0, 0x00, 0x00,  // x = 6
    0x41, 0x70, 0x00, 0x00,  // y = 15
    0x00, 0x00, 0x00, 0x00,  // z
    0x42, 0xC8, 0x00, 0x00,  // value = 100
    0x01, 0x02, 0x03, 0x04,  // created_ms
    0x07, 0x5B, 0xCD, 0x15,  // deadline_ms
    0x00, 0x14,              // duration_sec
  };
  EXPECT_EQ(body, expected);
}

TEST(WireTaskAnnounce, RoundTrip)
{
  const auto d = decodeTaskAnnounce(encodeTaskAnnounce(makeHeader(), sample()));
  ASSERT_TRUE(d);
  EXPECT_EQ(d->relay, (RelayHeader{3, 0x1234, 5}));
  EXPECT_EQ(d->task_id, 0x00320007u);
  EXPECT_EQ(d->task_type, 2);
  EXPECT_FLOAT_EQ(d->y, 15.0f);
  EXPECT_FLOAT_EQ(d->value, 100.0f);
  EXPECT_EQ(d->created_ms, 0x01020304u);
  EXPECT_EQ(d->deadline_ms, 123456789u);
  EXPECT_EQ(d->duration_sec, 20);
}

TEST(WireTaskAnnounce, RelayedContentUnchanged)
{
  // 轉送後內容（含 origin）不變，task_id 是機號 50 建立的也一樣
  const auto original = encodeTaskAnnounce(makeHeader(), sample());
  SeqCounter counter;
  const auto fwd = makeRelay(original, 2, 0x9999, counter);
  ASSERT_TRUE(fwd);
  const auto d = decodeTaskAnnounce(*fwd);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->relay, sample().relay);
  EXPECT_EQ(d->task_id, 0x00320007u);
  EXPECT_EQ(parseHeader(*fwd)->sender_id, 2);
}

TEST(WireTaskAnnounce, DeadlineAbsolute)
{
  const double now = 1.7e9 + 0.25;
  EXPECT_NEAR(fromDeadlineMs(toDeadlineMs(now + 60.0), now), now + 60.0, 1e-3);
  EXPECT_NEAR(fromDeadlineMs(toDeadlineMs(now - 5.0), now), now - 5.0, 1e-3);   // 已過期
  EXPECT_NE(toDeadlineMs(0.0), 0u);   // 0 保留為「沒有期限資訊」
}

TEST(WireTaskAnnounce, RejectsInvalid)
{
  const auto good = encodeTaskAnnounce(makeHeader(), sample());
  ASSERT_TRUE(decodeTaskAnnounce(good));

  TaskAnnounceBody b = sample();
  b.task_id = 0x00000001;   // 建立者 0
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.task_type = 4;
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.value = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.relay.origin_id = 0;
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);

  auto bad = good;
  bad[kHeaderSize + kRelaySize + 4] = 0;   // task_type 0
  EXPECT_FALSE(decodeTaskAnnounce(bad));
  bad = good;
  bad.push_back(0);
  finishPacket(bad);
  EXPECT_FALSE(decodeTaskAnnounce(bad));   // 長度不符
}
