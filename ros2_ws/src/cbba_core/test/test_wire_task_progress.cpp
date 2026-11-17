// TASK_PROGRESS（種類 12，CB-4 第一批，2026-11-16）：大小、位元組排列、來回編解碼、轉送、SYNC_BATCH 可以裝、拒絕不合法的封包
#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "cbba_core/milestones.hpp"
#include "cbba_core/wire_sync.hpp"
#include "cbba_core/wire_task_progress.hpp"

using namespace cbba_core;
using namespace cbba_core::wire;

namespace
{
Header makeHeader()
{
  Header h;
  h.sender_id = 1;
  h.session_id = 0xAB;
  h.sequence = 7;
  return h;
}

TaskProgressBody sample()
{
  TaskProgressBody b;
  b.relay = RelayHeader{1, 0xAB, 7};
  b.task_id = 0x00C8000300000005ULL;
  b.revision = 2;
  b.incident_revision = 3;
  b.assign_version = 4;
  b.executor = 1;
  b.progress_seq = 3;
  b.phase = static_cast<std::uint8_t>(ProgressPhase::EXECUTING);
  b.step_index = 1;
  b.milestones = milestoneBit(Milestone::ARRIVED) | milestoneBit(Milestone::TARGET_LOCALIZED);
  return b;
}
}  // namespace

TEST(WireTaskProgress, SizeAndLayout)
{
  // 表頭 20 + 轉送 14 + 29 = 63 B（第三批加 incident_revision）；task_id 緊接在轉送區塊之後、大端序；
  // incident_revision 在 revision 之後；里程碑在倒數第 3、2 個位元組
  const auto bytes = encodeTaskProgress(makeHeader(), sample());
  ASSERT_EQ(bytes.size(), 63u);
  EXPECT_EQ(bytes.size(), kTaskProgressSize);
  EXPECT_EQ(parseHeader(bytes)->type, PacketType::TASK_PROGRESS);
  EXPECT_EQ(bytes[kHeaderSize + kRelaySize], 0x00);
  EXPECT_EQ(bytes[kHeaderSize + kRelaySize + 1], 0xC8);
  EXPECT_EQ(bytes[kHeaderSize + kRelaySize + 7], 0x05);
  EXPECT_EQ(bytes[kHeaderSize + kRelaySize + 9], 0x02);    // revision
  EXPECT_EQ(bytes[kHeaderSize + kRelaySize + 11], 0x03);   // incident_revision
  EXPECT_EQ(bytes[bytes.size() - 3], 0x00);
  EXPECT_EQ(bytes[bytes.size() - 2], 0x14);   // bit 2（arrived）＋ bit 4（target_localized）
  EXPECT_EQ(bytes.back(), 0x00);              // flags
}

TEST(WireTaskProgress, RoundTripAndRelay)
{
  const TaskProgressBody b = sample();
  const auto bytes = encodeTaskProgress(makeHeader(), b);
  const auto got = decodeTaskProgress(bytes);
  ASSERT_TRUE(got);
  EXPECT_EQ(got->relay, b.relay);
  EXPECT_EQ(got->task_id, b.task_id);
  EXPECT_EQ(got->revision, 2);
  EXPECT_EQ(got->incident_revision, 3);
  EXPECT_EQ(got->assign_version, 4u);
  EXPECT_EQ(got->executor, 1);
  EXPECT_EQ(got->progress_seq, 3);
  EXPECT_EQ(got->phase, b.phase);
  EXPECT_EQ(got->step_index, 1);
  EXPECT_EQ(got->milestones, b.milestones);

  // 轉送：origin 不變、轉送者換表頭
  SeqCounter counter;
  const auto fwd = makeRelay(bytes, 2, 0x22, counter);
  ASSERT_TRUE(fwd);
  const auto relayed = decodeTaskProgress(*fwd);
  ASSERT_TRUE(relayed);
  EXPECT_EQ(relayed->relay, b.relay);
  EXPECT_EQ(parseHeader(*fwd)->sender_id, 2);

  // SYNC_BATCH 可以裝（進度快照）
  const auto item = toSyncItem(bytes);
  ASSERT_TRUE(item);
  EXPECT_EQ(item->type, PacketType::TASK_PROGRESS);
  const auto batch = encodeSyncBatch(makeHeader(), {*item});
  const auto items = decodeSyncBatch(batch);
  ASSERT_TRUE(items);
  ASSERT_EQ(items->size(), 1u);
  const auto again = decodeTaskProgress(syncItemPacket((*items)[0], makeHeader()));
  ASSERT_TRUE(again);
  EXPECT_EQ(again->milestones, b.milestones);
}

TEST(WireTaskProgress, RejectsInvalid)
{
  // 編碼：task_id 0、指派版本 0、progress_seq 0、執行者 0、phase 超出範圍 → 例外
  for (int k = 0; k < 5; ++k) {
    TaskProgressBody b = sample();
    if (k == 0) {b.task_id = 0;}
    if (k == 1) {b.assign_version = 0;}
    if (k == 2) {b.progress_seq = 0;}
    if (k == 3) {b.executor = 0;}
    if (k == 4) {b.phase = 8;}
    EXPECT_THROW(encodeTaskProgress(makeHeader(), b), std::invalid_argument) << k;
  }
  // 解碼：長度不對、phase 超出範圍
  auto bytes = encodeTaskProgress(makeHeader(), sample());
  auto longer = bytes;
  longer.push_back(0);
  finishPacket(longer);
  EXPECT_FALSE(decodeTaskProgress(longer));
  auto bad = bytes;
  bad[kHeaderSize + kRelaySize + 24] = 9;   // phase
  EXPECT_FALSE(decodeTaskProgress(bad));
}

TEST(Milestones, NamesAndClasses)
{
  // 名稱 ⇄ 位元（只增不改）；事件結果型只有 target_localized、has_observation
  EXPECT_EQ(static_cast<int>(Milestone::TARGET_LOCALIZED), 4);
  EXPECT_EQ(static_cast<int>(Milestone::COMPLETED), 7);
  std::vector<std::string> unknown;
  const MilestoneMask m = parseMilestones({"arrived", "target_localized", "flying"}, &unknown);
  EXPECT_EQ(m, milestoneBit(Milestone::ARRIVED) | milestoneBit(Milestone::TARGET_LOCALIZED));
  EXPECT_EQ(unknown, std::vector<std::string>{"flying"});
  EXPECT_EQ(milestoneNames(m), (std::vector<std::string>{"arrived", "target_localized"}));
  EXPECT_EQ(kEventMilestones, milestoneBit(Milestone::TARGET_LOCALIZED) | milestoneBit(Milestone::HAS_OBSERVATION));
  EXPECT_EQ(kExecutorMilestones & milestoneBit(Milestone::COMPLETED), 0);
  EXPECT_EQ(kExecutorMilestones | kEventMilestones | milestoneBit(Milestone::COMPLETED), kAllMilestones);
}
