// INCIDENT_UPDATE（種類 13，CB-4 第三批，2026-11-16）：大小、位元組排列、來回編解碼、轉送、SYNC_BATCH、拒絕不合法的封包
#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "cbba_core/wire_incident_update.hpp"
#include "cbba_core/wire_sync.hpp"

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

IncidentUpdateBody sample()
{
  IncidentUpdateBody b;
  b.relay = RelayHeader{1, 0xAB, 7};
  b.kind = IncidentKind::PROPOSAL;
  b.group_id = 0x00C8000300000010ULL;
  b.incident_revision = 2;
  b.proposer = 1;
  b.proposal_seq = 5;
  b.reporter_task = 0x00C8000300000011ULL;
  b.assign_version = 9;
  b.flags = kIncidentPositionValid | kIncidentLocalized;
  b.x = 6.0F;
  b.y = 15.0F;
  b.z = 0.0F;
  b.floor = 1;
  b.radius = 2.0F;
  b.track_id = 88;
  return b;
}
}  // namespace

TEST(WireIncidentUpdate, SizeAndLayout)
{
  // 表頭 20 + 轉送 14 + 56 = 90 B；kind 緊接在轉送區塊之後，group_id 大端序，最後一個位元組是 reason
  const auto bytes = encodeIncidentUpdate(makeHeader(), sample());
  ASSERT_EQ(bytes.size(), 90u);
  EXPECT_EQ(kIncidentUpdateSize, 90u);
  EXPECT_EQ(parseHeader(bytes)->type, PacketType::INCIDENT_UPDATE);
  EXPECT_EQ(bytes[kHeaderSize + kRelaySize], 1);          // PROPOSAL
  EXPECT_EQ(bytes[kHeaderSize + kRelaySize + 2], 0xC8);   // group_id 的建立者 200
  EXPECT_EQ(bytes[kHeaderSize + kRelaySize + 10], 0x02);  // incident_revision
  EXPECT_EQ(bytes.back(), 0);                             // reason NONE
}

TEST(WireIncidentUpdate, RoundTripRelayAndSync)
{
  IncidentUpdateBody b = sample();
  b.kind = IncidentKind::REJECTED;
  b.reason = IncidentReject::STALE;
  const auto bytes = encodeIncidentUpdate(makeHeader(), b);
  const auto got = decodeIncidentUpdate(bytes);
  ASSERT_TRUE(got);
  EXPECT_EQ(got->relay, b.relay);
  EXPECT_EQ(got->kind, IncidentKind::REJECTED);
  EXPECT_EQ(got->group_id, b.group_id);
  EXPECT_EQ(got->incident_revision, 2);
  EXPECT_EQ(got->proposer, 1);
  EXPECT_EQ(got->proposal_seq, 5u);
  EXPECT_EQ(got->reporter_task, b.reporter_task);
  EXPECT_EQ(got->assign_version, 9u);
  EXPECT_EQ(got->flags, b.flags);
  EXPECT_FLOAT_EQ(got->y, 15.0F);
  EXPECT_EQ(got->floor, 1);
  EXPECT_FLOAT_EQ(got->radius, 2.0F);
  EXPECT_EQ(got->track_id, 88u);
  EXPECT_EQ(got->reason, IncidentReject::STALE);

  SeqCounter counter;
  const auto fwd = makeRelay(bytes, 2, 0x22, counter);
  ASSERT_TRUE(fwd);
  EXPECT_EQ(decodeIncidentUpdate(*fwd)->relay, b.relay);

  // SYNC_BATCH 可以裝（Accepted 的快照）
  const auto item = toSyncItem(bytes);
  ASSERT_TRUE(item);
  const auto items = decodeSyncBatch(encodeSyncBatch(makeHeader(), {*item}));
  ASSERT_TRUE(items);
  EXPECT_TRUE(decodeIncidentUpdate(syncItemPacket((*items)[0], makeHeader())));
}

TEST(WireIncidentUpdate, RejectsInvalid)
{
  // 編碼：group_id 0、proposal_seq 0、提案者 0、半徑是負的 → 例外；解碼：kind、reason 超出範圍、長度不對
  for (int k = 0; k < 4; ++k) {
    IncidentUpdateBody b = sample();
    if (k == 0) {b.group_id = 0;}
    if (k == 1) {b.proposal_seq = 0;}
    if (k == 2) {b.proposer = 0;}
    if (k == 3) {b.radius = -1.0F;}
    EXPECT_THROW(encodeIncidentUpdate(makeHeader(), b), std::invalid_argument) << k;
  }
  const auto bytes = encodeIncidentUpdate(makeHeader(), sample());
  auto bad = bytes;
  bad[kHeaderSize + kRelaySize] = 4;   // kind
  EXPECT_FALSE(decodeIncidentUpdate(bad));
  bad = bytes;
  bad.back() = 6;                      // reason
  EXPECT_FALSE(decodeIncidentUpdate(bad));
  bad = bytes;
  bad.push_back(0);
  finishPacket(bad);
  EXPECT_FALSE(decodeIncidentUpdate(bad));
}
