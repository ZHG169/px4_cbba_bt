// CBBA_STATE（協定版本 2）：大小、位元組排列、來回編解碼、分批、拒絕不合法的封包
#include <gtest/gtest.h>

#include <limits>
#include <set>
#include <vector>

#include "cbba_core/wire_cbba_state.hpp"

using namespace cbba_core::wire;

namespace
{
Header makeHeader()
{
  Header h;
  h.sender_id = 1;
  h.session_id = 9;
  h.sequence = 3;
  return h;
}

CbbaStateBody sample(std::size_t records = 2)
{
  CbbaStateBody b;
  b.round_key = 0xDEADBEEF;
  b.exchange_step = 12;
  b.participating = true;
  b.stable_steps = 4;
  b.snapshot_id = 77;
  b.stamps = {{1, 1000}, {2, 2000}, {50, 3000}};
  for (std::size_t i = 0; i < records; ++i) {
    b.records.push_back(Record{0x00010001u + static_cast<std::uint32_t>(i),
        static_cast<std::uint16_t>(i % 2 == 0 ? 50 : 0), i % 2 == 0 ? 12.5f : 0.0f,
        static_cast<std::uint64_t>(i), static_cast<std::uint16_t>(i % 2 == 0 ? 50 : 0),
        static_cast<std::uint8_t>(i % 2 == 0 ? 3 : 0)});
  }
  return b;
}
}  // namespace

TEST(WireCbbaState, Size)
{
  // 36 + 6N + 21R；不參與出價時 36 B
  EXPECT_EQ(cbbaStateSize(0, 0), 36u);
  EXPECT_EQ(encodeCbbaState(makeHeader(), CbbaStateBody{}).size(), 36u);
  EXPECT_EQ(encodeCbbaState(makeHeader(), sample()).size(), 36u + 18u + 42u);
}

TEST(WireCbbaState, Layout)
{
  CbbaStateBody b;
  b.round_key = 0x01020304;
  b.exchange_step = 0x0506;
  b.participating = true;
  b.stable_steps = 7;
  b.snapshot_id = 0x0809;
  b.part_index = 1;
  b.part_count = 2;
  b.stamps = {{0x0032, 0x0A0B0C0D}};
  b.records = {{0x00320001, 0x0003, 1.0f, 0x0102030405060708ULL, 0x0003, 2}};
  const auto bytes = encodeCbbaState(makeHeader(), b);
  const std::vector<std::uint8_t> body(bytes.begin() + kHeaderSize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x01, 0x02, 0x03, 0x04,  // round_key
    0x05, 0x06,              // exchange_step
    0x01,                    // flags：參與
    0x07,                    // stable_steps
    0x08, 0x09,              // snapshot_id
    0x01, 0x02,              // part_index、part_count
    0x00, 0x01,              // stamp_count
    0x00, 0x32, 0x0A, 0x0B, 0x0C, 0x0D,
    0x00, 0x01,              // record_count
    0x00, 0x32, 0x00, 0x01,  // task_id
    0x00, 0x03,              // winner_id
    0x3F, 0x80, 0x00, 0x00,  // winning_bid
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,  // assign_version（uint64）
    0x00, 0x03,              // assignee_id
    0x02,                    // assign_state：保留
  };
  EXPECT_EQ(body, expected);
}

TEST(WireCbbaState, RoundTrip)
{
  const auto d = decodeCbbaState(encodeCbbaState(makeHeader(), sample()));
  ASSERT_TRUE(d);
  EXPECT_EQ(d->round_key, 0xDEADBEEFu);
  EXPECT_EQ(d->exchange_step, 12);
  EXPECT_TRUE(d->participating);
  EXPECT_EQ(d->stable_steps, 4);
  EXPECT_EQ(d->snapshot_id, 77);
  ASSERT_EQ(d->stamps.size(), 3u);
  EXPECT_EQ(d->stamps[2].agent_id, 50);
  EXPECT_EQ(d->stamps[2].stamp_ms, 3000u);
  ASSERT_EQ(d->records.size(), 2u);
  EXPECT_EQ(d->records[0].task_id, 0x00010001u);
  EXPECT_EQ(d->records[0].winner_id, 50);
  EXPECT_FLOAT_EQ(d->records[0].winning_bid, 12.5f);
  EXPECT_EQ(d->records[1].winner_id, 0);   // 認得、沒人得標
  EXPECT_EQ(d->records[0].assign_version, 0u);
  EXPECT_EQ(d->records[0].assignee_id, 50);
  EXPECT_EQ(d->records[0].assign_state, 3);   // 執行中
  EXPECT_EQ(d->records[1].assign_version, 1u);
  EXPECT_EQ(d->records[1].assignee_id, 0);   // 撤銷
  EXPECT_EQ(d->records[1].assign_state, 0);
}

TEST(WireCbbaState, SplitKeepsEveryPartUnderLimitWithFullStamps)
{
  // 300 筆紀錄、上限 1200 B：每批都帶完整的 s，合起來剛好是全部紀錄
  const CbbaStateBody full = sample(300);
  const auto parts = splitCbbaState(full, 1200);
  ASSERT_GT(parts.size(), 1u);
  std::set<std::uint32_t> ids;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    const auto bytes = encodeCbbaState(makeHeader(), parts[i]);
    EXPECT_LE(bytes.size(), 1200u);
    EXPECT_EQ(parts[i].part_index, i);
    EXPECT_EQ(parts[i].part_count, parts.size());
    EXPECT_EQ(parts[i].snapshot_id, 77);
    EXPECT_EQ(parts[i].stamps.size(), 3u);
    for (const Record & r : parts[i].records) {
      ids.insert(r.task_id);
    }
  }
  EXPECT_EQ(ids.size(), 300u);
  // (1200 − 36 − 18) / 21 = 54 筆一批 → 6 批
  EXPECT_EQ(parts.size(), 6u);
  // 沒有紀錄時也有一批
  EXPECT_EQ(splitCbbaState(CbbaStateBody{}, 1200).size(), 1u);
  EXPECT_THROW(splitCbbaState(full, 40), std::invalid_argument);
}

TEST(WireCbbaState, RejectsInvalid)
{
  const auto good = encodeCbbaState(makeHeader(), sample());
  ASSERT_TRUE(decodeCbbaState(good));

  CbbaStateBody b = sample();
  b.part_index = 2;
  b.part_count = 2;
  EXPECT_THROW(encodeCbbaState(makeHeader(), b), std::invalid_argument);

  auto bad = good;
  bad[kHeaderSize + 10] = 5;   // part_index ≥ part_count
  EXPECT_FALSE(decodeCbbaState(bad));
  bad = good;
  bad.pop_back();
  finishPacket(bad);
  EXPECT_FALSE(decodeCbbaState(bad));
  bad = good;
  bad[kHeaderSize + 12] = 0xFF;   // stamp_count 很大：不會照著配置記憶體
  bad[kHeaderSize + 13] = 0xFF;
  EXPECT_FALSE(decodeCbbaState(bad));

  b = sample();
  b.records[0].task_id = 0x0000FFFF;   // 建立者 0
  EXPECT_THROW(encodeCbbaState(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.records[0].winning_bid = -1.0f;
  EXPECT_THROW(encodeCbbaState(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.records[0].winning_bid = std::numeric_limits<float>::infinity();
  EXPECT_THROW(encodeCbbaState(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.records[0].assignee_id = 0xFFFF;
  EXPECT_THROW(encodeCbbaState(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.records[0].assign_state = 4;              // 狀態只有 0～3
  EXPECT_THROW(encodeCbbaState(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.records[0].assignee_id = 0;               // 執行中卻沒有執行者
  EXPECT_THROW(encodeCbbaState(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.stamps[0].agent_id = 0;
  EXPECT_THROW(encodeCbbaState(makeHeader(), b), std::invalid_argument);
}
