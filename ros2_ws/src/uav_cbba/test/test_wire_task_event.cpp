// TASK_EVENT：大小與位元組排列和規格書 §2.6 一致、版本 2 擴充、轉送、拒絕不合法的封包、截止時刻
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>

#include "uav_cbba/wire_task_event.hpp"

using namespace uav_cbba::wire;

namespace
{
Header header(std::uint8_t agent = 3)
{
  Header h;
  h.agent_id = agent;
  h.seq = 5;
  h.stamp_us = 1000;
  return h;
}

TaskEventBody body(const std::string & name = "F3-01", bool ext = false)
{
  TaskEventBody b;
  b.relay = {3, 5};
  b.task_index = 2;
  b.x = 1.5f;
  b.y = -2.0f;
  b.z = 5.0f;
  b.value = 80.0f;
  b.name = name;
  if (ext) {
    b.ext = TaskExtension{1, 123456789u, 10};
  }
  return b;
}
}  // namespace

TEST(TaskEvent, SizesMatchSpec)
{
  // 規格 §2.6：14 + 27 + 名稱長度 = 42~49 B
  EXPECT_EQ(encodeTaskEvent(header(), body("A")).size(), 42u);
  EXPECT_EQ(encodeTaskEvent(header(), body("ABCDEFGH")).size(), 49u);
  // 版本 2 再加 7 B
  EXPECT_EQ(encodeTaskEvent(header(), body("A", true)).size(), 49u);
  EXPECT_EQ(encodeTaskEvent(header(), body("ABCDEFGH", true)).size(), 56u);
}

TEST(TaskEvent, ByteLayoutVersion1)
{
  TaskEventBody b = body("AB");
  b.relay = {7, 0x0102};
  b.task_index = 9;
  b.x = 1.0f;     // 0x3F800000
  b.y = -1.0f;    // 0xBF800000
  b.z = 0.0f;
  b.value = 2.0f; // 0x40000000
  const auto bytes = encodeTaskEvent(header(), b);
  EXPECT_EQ(bytes[0], 0x24);   // 代碼 4 | 版本 1

  const std::vector<std::uint8_t> expected = {
    0x07, 0x02, 0x01, 0, 0, 0, 0, 0, 0,   // 轉送表頭：origin_id、origin_seq
    0x09,                                 // task_index
    0x00, 0x00, 0x80, 0x3F,               // x
    0x00, 0x00, 0x80, 0xBF,               // y
    0x00, 0x00, 0x00, 0x00,               // z
    0x00, 0x00, 0x00, 0x40,               // value
    0x02, 'A', 'B',                       // id_len、task_id
  };
  EXPECT_EQ(std::vector<std::uint8_t>(bytes.begin() + kHeaderSize, bytes.end()), expected);
}

TEST(TaskEvent, ByteLayoutVersion2Extension)
{
  TaskEventBody b = body("AB");
  b.ext = TaskExtension{2, 0x0A0B0C0D, 0x0102};
  const auto bytes = encodeTaskEvent(header(), b);
  EXPECT_EQ(bytes[0], 0x44);   // 代碼 4 | 版本 2
  const std::vector<std::uint8_t> tail = {
    0x02,                     // task_type
    0x0D, 0x0C, 0x0B, 0x0A,   // deadline_ms
    0x02, 0x01,               // duration_sec
  };
  EXPECT_EQ(std::vector<std::uint8_t>(bytes.end() - 7, bytes.end()), tail);
}

TEST(TaskEvent, RoundTripBothVersions)
{
  for (bool ext : {false, true}) {
    const TaskEventBody in = body("F3-01", ext);
    const auto out = decodeTaskEvent(encodeTaskEvent(header(), in));
    ASSERT_TRUE(out) << ext;
    EXPECT_EQ(out->relay.origin_id, 3);
    EXPECT_EQ(out->relay.origin_seq, 5u);
    EXPECT_EQ(out->task_index, 2);
    EXPECT_FLOAT_EQ(out->x, 1.5f);
    EXPECT_FLOAT_EQ(out->y, -2.0f);
    EXPECT_FLOAT_EQ(out->z, 5.0f);
    EXPECT_FLOAT_EQ(out->value, 80.0f);
    EXPECT_EQ(out->name, "F3-01");
    ASSERT_EQ(out->ext.has_value(), ext);
    if (ext) {
      EXPECT_EQ(out->ext->task_type, 1);
      EXPECT_EQ(out->ext->deadline_ms, 123456789u);
      EXPECT_EQ(out->ext->duration_sec, 10);
    }
  }
}

TEST(TaskEvent, RelayKeepsVersionAndBody)
{
  const auto original = encodeTaskEvent(header(3), body("F3-01", true));
  SeqCounter counter;
  const auto fwd = makeRelay(original, 2, counter, 9.0);
  ASSERT_TRUE(fwd);
  EXPECT_EQ((*fwd)[0], 0x44);    // 版本 2 保留
  EXPECT_EQ((*fwd)[1], 2);       // 轉送者
  const auto out = decodeTaskEvent(*fwd);
  ASSERT_TRUE(out);
  EXPECT_EQ(out->relay.origin_id, 3);   // 原始發送者不變
  ASSERT_TRUE(out->ext);
  EXPECT_EQ(out->ext->deadline_ms, 123456789u);   // 截止時刻不變
}

TEST(TaskEvent, RejectsInvalidBody)
{
  EXPECT_THROW(encodeTaskEvent(header(), body("")), std::invalid_argument);
  EXPECT_THROW(encodeTaskEvent(header(), body("ABCDEFGHI")), std::invalid_argument);   // 9 字元
  EXPECT_THROW(encodeTaskEvent(header(), body("F 1")), std::invalid_argument);         // 空白
  EXPECT_THROW(encodeTaskEvent(header(), body("火點")), std::invalid_argument);        // 非 ASCII
  TaskEventBody b = body();
  b.task_index = 255;
  EXPECT_THROW(encodeTaskEvent(header(), b), std::invalid_argument);
  b = body();
  b.relay.origin_id = 0;
  EXPECT_THROW(encodeTaskEvent(header(), b), std::invalid_argument);
}

TEST(TaskEvent, RejectsMalformed)
{
  const auto good = encodeTaskEvent(header(), body("F3-01"));
  EXPECT_FALSE(decodeTaskEvent(good.data(), good.size() - 1));   // 少 1 B
  auto longer = good;
  longer.push_back('X');
  EXPECT_FALSE(decodeTaskEvent(longer));                         // 多 1 B

  auto v2_without_ext = good;
  v2_without_ext[0] = typeByte(PacketType::TASK_EVENT, 2);      // 標成版本 2 卻沒有擴充欄位
  EXPECT_FALSE(decodeTaskEvent(v2_without_ext));

  auto wrong_type = good;
  wrong_type[0] = typeByte(PacketType::COMPLETION);
  EXPECT_FALSE(decodeTaskEvent(wrong_type));

  const std::size_t index_at = kHeaderSize + kRelayHeaderSize;
  auto bad = good;
  bad[index_at] = 255;                                          // task_index 255
  EXPECT_FALSE(decodeTaskEvent(bad));
  bad = good;
  bad[kHeaderSize] = 0;                                         // origin_id 0
  EXPECT_FALSE(decodeTaskEvent(bad));
  bad = good;
  bad[bad.size() - 1] = 0x01;                                   // 名稱含控制字元
  EXPECT_FALSE(decodeTaskEvent(bad));

  TaskEventBody nan = body();
  nan.x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(decodeTaskEvent(encodeTaskEvent(header(), nan)));
}

TEST(TaskEvent, DeadlineMs)
{
  const double now = 1791276182.5;
  // 未來 60 s、過去 30 s 都能還原
  EXPECT_NEAR(fromDeadlineMs(toDeadlineMs(now + 60.0), now), now + 60.0, 1e-3);
  EXPECT_NEAR(fromDeadlineMs(toDeadlineMs(now - 30.0), now), now - 30.0, 1e-3);
  // 跨過 2^32 ms 的循環點也正確
  const double wrap = 4294967.296;   // 2^32 ms
  EXPECT_NEAR(fromDeadlineMs(toDeadlineMs(wrap + 5.0), wrap - 5.0), wrap + 5.0, 1e-3);
  // 0 保留為「沒有期限資訊」
  EXPECT_NE(toDeadlineMs(wrap), 0u);
}
