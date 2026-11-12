// TASK_ANNOUNCE（協定版本 4）：大小、位元組排列、來回編解碼（A 層全部欄位）、轉送後不變、截止時刻、拒絕不合法的封包
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

// 舊型任務（地面處置）：新欄位都是預設值
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

// 新型任務（閘道建立、任務組的第 2 個角色）：每個欄位都不是預設值
TaskAnnounceBody fullSample()
{
  TaskAnnounceBody b = sample();
  b.task_id = 0x00C8000300000011ULL;
  b.revision = 0x0203;
  b.task_type = 0;
  b.platform = 2;
  b.required_caps = 0x0000000000000051ULL;
  b.priority = 1;
  b.deadline_hard = true;
  b.optional = true;
  b.all_or_nothing = false;
  b.skip_satisfies = true;
  b.floor = 3;
  b.release_ms = 0x11121314;
  b.expires_ms = 0x21222324;
  b.half_life_s = 60;
  b.can_preempt = 0x0C;
  b.preempt_margin_pct = 20;
  b.commit_radius_dm = 50;
  b.min_battery_reserve = 25;
  b.group_id = 0x00C8000300000010ULL;
  b.role_index = 1;
  b.role_count = 3;
  b.precedence = 0x01;
  b.sync_window_s = 15;
  b.optional_no_capable_s = 10;
  b.optional_unassigned_s = 15;
  b.spec_length = 0x00000400;
  b.spec_crc = 0xCAFEBABE;
  return b;
}
}  // namespace

TEST(WireTaskAnnounce, SizeAndLayout)
{
  const auto bytes = encodeTaskAnnounce(makeHeader(), fullSample());
  ASSERT_EQ(bytes.size(), 123u);
  EXPECT_EQ(kTaskAnnounceSize, 123u);
  EXPECT_EQ(encodeTaskAnnounce(makeHeader(), sample()).size(), 123u);   // 固定大小
  const std::vector<std::uint8_t> body(bytes.begin() + kHeaderSize + kRelaySize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x00, 0xC8, 0x00, 0x03, 0x00, 0x00, 0x00, 0x11,  // task_id（建立者 200、世代 3、流水號 0x11）
    0x02, 0x03,              // revision
    0x00,                    // task_type：新型
    0x02,                    // platform：quadruped
    0, 0, 0, 0, 0, 0, 0, 0x51,   // required_caps
    0x01,                    // priority P1
    0x0B,                    // flags：硬期限、optional、skip_satisfies
    0x40, 0xC0, 0x00, 0x00,  // x = 6
    0x41, 0x70, 0x00, 0x00,  // y = 15
    0x00, 0x00, 0x00, 0x00,  // z
    0x03,                    // floor
    0x42, 0xC8, 0x00, 0x00,  // value = 100
    0x01, 0x02, 0x03, 0x04,  // created_ms
    0x11, 0x12, 0x13, 0x14,  // release_ms
    0x07, 0x5B, 0xCD, 0x15,  // deadline_ms
    0x21, 0x22, 0x23, 0x24,  // expires_ms
    0x00, 0x14,              // duration_sec
    0x00, 0x3C,              // half_life_s
    0x0C,                    // can_preempt
    0x14,                    // preempt_margin_pct
    0x00, 0x32,              // commit_radius_dm（5 m）
    0x19,                    // min_battery_reserve
    0x00, 0xC8, 0x00, 0x03, 0x00, 0x00, 0x00, 0x10,  // group_id
    0x01, 0x03,              // role_index、role_count
    0x01,                    // precedence
    0x00, 0x0F,              // sync_window_s
    0x00, 0x0A,              // optional_no_capable_s
    0x00, 0x0F,              // optional_unassigned_s
    0x00, 0x00, 0x04, 0x00,  // spec_length
    0xCA, 0xFE, 0xBA, 0xBE,  // spec_crc
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
  EXPECT_EQ(d->spec_length, 0u);   // 舊型任務沒有 TaskSpec
}

TEST(WireTaskAnnounce, RoundTripAllFields)
{
  const TaskAnnounceBody b = fullSample();
  const auto d = decodeTaskAnnounce(encodeTaskAnnounce(makeHeader(), b));
  ASSERT_TRUE(d);
  EXPECT_EQ(d->task_id, b.task_id);
  EXPECT_EQ(d->revision, b.revision);
  EXPECT_EQ(d->task_type, 0);
  EXPECT_EQ(d->platform, 2);
  EXPECT_EQ(d->required_caps, b.required_caps);
  EXPECT_EQ(d->priority, 1);
  EXPECT_TRUE(d->deadline_hard);
  EXPECT_TRUE(d->optional);
  EXPECT_FALSE(d->all_or_nothing);
  EXPECT_TRUE(d->skip_satisfies);
  EXPECT_EQ(d->floor, 3);
  EXPECT_EQ(d->release_ms, b.release_ms);
  EXPECT_EQ(d->expires_ms, b.expires_ms);
  EXPECT_EQ(d->half_life_s, 60);
  EXPECT_EQ(d->can_preempt, 0x0C);
  EXPECT_EQ(d->preempt_margin_pct, 20);
  EXPECT_EQ(d->commit_radius_dm, 50);
  EXPECT_EQ(d->min_battery_reserve, 25);
  EXPECT_EQ(d->group_id, b.group_id);
  EXPECT_EQ(d->role_index, 1);
  EXPECT_EQ(d->role_count, 3);
  EXPECT_EQ(d->precedence, 0x01);
  EXPECT_EQ(d->sync_window_s, 15);
  EXPECT_EQ(d->optional_no_capable_s, 10);
  EXPECT_EQ(d->optional_unassigned_s, 15);
  EXPECT_EQ(d->spec_length, 0x400u);
  EXPECT_EQ(d->spec_crc, 0xCAFEBABEu);
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
  b.task_id = 0x00000001;   // 舊型、建立者 0
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.task_id = 0x00C8000000000001ULL;   // 版本 4、世代 0
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.task_type = 4;
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.platform = 3;
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.priority = 4;
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = fullSample();
  b.role_count = 9;   // 最多 8 個角色
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = fullSample();
  b.role_index = 3;   // ≥ role_count
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = fullSample();
  b.group_id = 0x0000000300000001ULL;   // 任務組 id 不合法
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.value = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.relay.origin_id = 0;
  EXPECT_THROW(encodeTaskAnnounce(makeHeader(), b), std::invalid_argument);

  auto bad = good;
  bad[kHeaderSize + kRelaySize + 10] = 4;   // task_type 4
  EXPECT_FALSE(decodeTaskAnnounce(bad));
  bad = good;
  bad.push_back(0);
  finishPacket(bad);
  EXPECT_FALSE(decodeTaskAnnounce(bad));   // 長度不符
}
