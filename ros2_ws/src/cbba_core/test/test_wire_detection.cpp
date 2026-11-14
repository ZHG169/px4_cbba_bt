// DETECTION、DETECTION_ACK（協定版本 4，2026-11-14）：大小、位元組排列、來回編解碼、轉送、拒絕不合法的封包
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "cbba_core/wire_detection.hpp"

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

DetectionBody sample()
{
  DetectionBody b;
  b.relay = RelayHeader{1, 0xAB, 7};
  b.detection_seq = 3;
  b.t_ms = 1794597973974ULL;
  b.flags = kDetectionPositionValid | kDetectionBuffered;
  b.score = 0.5F;
  b.x = 12.0F;
  b.y = -4.0F;
  b.z = 0.0F;
  b.floor = 2;
  b.bbox = {10.0F, 20.0F, 30.0F, 40.0F};
  b.track_id = 9;
  b.detector = "apriltag_fire";
  b.source = "uav1/down_cam";
  b.label = "fire";
  b.attrs = "{\"tag\":101}";
  return b;
}
}  // namespace

TEST(WireDetection, SizeAndLayout)
{
  // 表頭 20 + 轉送 14 + 固定 55 + 字串
  DetectionBody b = sample();
  b.detector = "d";
  b.source = "";
  b.label = "fi";
  b.attrs = "";
  const auto bytes = encodeDetection(makeHeader(), b);
  EXPECT_EQ(kDetectionBaseSize, 89u);
  ASSERT_EQ(bytes.size(), kDetectionBaseSize + 3);
  EXPECT_EQ(parseHeader(bytes)->type, PacketType::DETECTION);
  const std::vector<std::uint8_t> body(bytes.begin() + kHeaderSize + kRelaySize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x00, 0x00, 0x00, 0x03,                          // detection_seq
    0x00, 0x00, 0x01, 0xA1, 0xD6, 0x5F, 0xEF, 0xD6,  // t_ms
    0x03,                                            // flags：位置有效、暫存過
    0x3F, 0x00, 0x00, 0x00,                          // score 0.5
    0x41, 0x40, 0x00, 0x00,                          // x 12
    0xC0, 0x80, 0x00, 0x00,                          // y -4
    0x00, 0x00, 0x00, 0x00,                          // z 0
    0x02,                                            // floor
    0x41, 0x20, 0x00, 0x00, 0x41, 0xA0, 0x00, 0x00,  // bbox 10, 20
    0x41, 0xF0, 0x00, 0x00, 0x42, 0x20, 0x00, 0x00,  // bbox 30, 40
    0x00, 0x00, 0x00, 0x09,                          // track_id
    0x01, 0x00, 0x02, 0x00, 0x00,                    // 字串長度：detector 1、source 0、label 2、attrs 0
    'd', 'f', 'i',
  };
  EXPECT_EQ(body, expected);
}

TEST(WireDetection, RoundTripAndRelay)
{
  const auto original = encodeDetection(makeHeader(), sample());
  const auto d = decodeDetection(original);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->relay, (RelayHeader{1, 0xAB, 7}));
  EXPECT_EQ(d->detection_seq, 3u);
  EXPECT_EQ(d->t_ms, 1794597973974ULL);
  EXPECT_EQ(d->flags, kDetectionPositionValid | kDetectionBuffered);
  EXPECT_FLOAT_EQ(d->score, 0.5F);
  EXPECT_FLOAT_EQ(d->y, -4.0F);
  EXPECT_EQ(d->floor, 2);
  EXPECT_FLOAT_EQ(d->bbox[3], 40.0F);
  EXPECT_EQ(d->track_id, 9u);
  EXPECT_EQ(d->detector, "apriltag_fire");
  EXPECT_EQ(d->source, "uav1/down_cam");
  EXPECT_EQ(d->label, "fire");
  EXPECT_EQ(d->attrs, "{\"tag\":101}");
  // 轉送：換表頭，內容（含 origin）不變
  SeqCounter counter;
  const auto fwd = makeRelay(original, 50, 0xCD, counter);
  ASSERT_TRUE(fwd);
  EXPECT_EQ(parseHeader(*fwd)->sender_id, 50);
  const auto r = decodeDetection(*fwd);
  ASSERT_TRUE(r);
  EXPECT_EQ(r->relay.origin_id, 1);
  EXPECT_EQ(r->label, "fire");
}

TEST(WireDetection, LargestFitsInOnePacket)
{
  // 字串都到上限：1181 B，在 1200 B 以內
  DetectionBody b = sample();
  b.detector.assign(kMaxDetectionName, 'a');
  b.source.assign(kMaxDetectionName, 'b');
  b.label.assign(kMaxDetectionName, 'c');
  b.attrs.assign(kMaxDetectionAttrs, 'd');
  const auto bytes = encodeDetection(makeHeader(), b);
  EXPECT_EQ(bytes.size(), 1181u);
  EXPECT_LE(bytes.size(), 1200u);
  EXPECT_TRUE(decodeDetection(bytes));
}

TEST(WireDetection, RejectsInvalid)
{
  DetectionBody b = sample();
  b.detection_seq = 0;
  EXPECT_THROW(encodeDetection(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.label.assign(kMaxDetectionName + 1, 'x');
  EXPECT_THROW(encodeDetection(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.attrs.assign(kMaxDetectionAttrs + 1, 'x');
  EXPECT_THROW(encodeDetection(makeHeader(), b), std::invalid_argument);

  auto bytes = encodeDetection(makeHeader(), sample());
  auto shorter = bytes;
  shorter.pop_back();
  EXPECT_FALSE(decodeDetection(shorter));               // 長度不符
  auto wrong_len = bytes;
  wrong_len[kHeaderSize + kRelaySize + 53] = 0x00;       // attrs_len 和實際不符
  wrong_len[kHeaderSize + kRelaySize + 54] = 0x01;
  EXPECT_FALSE(decodeDetection(wrong_len));
  auto zero_seq = bytes;
  for (int i = 0; i < 4; ++i) {
    zero_seq[kHeaderSize + kRelaySize + i] = 0;
  }
  EXPECT_FALSE(decodeDetection(zero_seq));              // detection_seq 0
  EXPECT_FALSE(decodeDetectionAck(bytes));              // 種類不對
}

TEST(WireDetectionAck, LayoutRoundTripAndRejects)
{
  // 表頭 20 + 轉送 14 + target_id 2 + target_session 8 + count 1 + 4 × count
  DetectionAckBody a;
  a.relay = RelayHeader{200, 0x11, 5};
  a.target_id = 1;
  a.target_session = 0xAB;
  a.seqs = {3, 4};
  Header h = makeHeader();
  h.sender_id = 200;
  const auto bytes = encodeDetectionAck(h, a);
  ASSERT_EQ(bytes.size(), kDetectionAckBaseSize + 8);
  EXPECT_EQ(parseHeader(bytes)->type, PacketType::DETECTION_ACK);
  const std::vector<std::uint8_t> body(bytes.begin() + kHeaderSize + kRelaySize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x00, 0x01,                                      // target_id
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB,  // target_session
    0x02,                                            // count
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x04,
  };
  EXPECT_EQ(body, expected);
  const auto d = decodeDetectionAck(bytes);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->target_id, 1);
  EXPECT_EQ(d->target_session, 0xABu);
  EXPECT_EQ(d->seqs, (std::vector<std::uint32_t>{3, 4}));

  a.seqs.clear();
  EXPECT_THROW(encodeDetectionAck(h, a), std::invalid_argument);   // 空的
  a.seqs.assign(kMaxAckSeqs + 1, 1);
  EXPECT_THROW(encodeDetectionAck(h, a), std::invalid_argument);   // 太多
  a.seqs = {1};
  a.target_id = 0;
  EXPECT_THROW(encodeDetectionAck(h, a), std::invalid_argument);   // 0 = 無人
  auto shorter = bytes;
  shorter.pop_back();
  EXPECT_FALSE(decodeDetectionAck(shorter));
}
