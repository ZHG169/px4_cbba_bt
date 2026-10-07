// COMPLETION / COMPLETION_ACK：大小與位元組排列和規格書 §2.8、§2.10 一致、來回編解碼、
// 拒絕不合法的封包、多跳時重送要換新序號
#include <gtest/gtest.h>

#include <stdexcept>

#include "uav_cbba/wire_completion.hpp"

using namespace uav_cbba::wire;

namespace
{
Header header(std::uint8_t agent = 3)
{
  Header h;
  h.agent_id = agent;
  h.seq = 4;
  h.stamp_us = 1000;
  return h;
}

CompletionBody announce()
{
  CompletionBody b;
  b.relay = {3, 4};
  b.kind = kCompletionAnnounce;
  b.task_index = 2;
  b.executor_id = 3;
  b.round_key = 7;
  b.control_revision = 99;
  b.view_bits = 0x1F;
  return b;
}

CompletionBody proof(std::size_t k)
{
  CompletionBody b = announce();
  b.kind = kCompletionProof;
  for (std::size_t i = 0; i < k; ++i) {
    b.acks.push_back(AckRef{static_cast<std::uint8_t>(i + 1), 100 + i});
  }
  return b;
}

CompletionAckBody ack(bool accepted = true)
{
  CompletionAckBody b;
  b.relay = {1, 6};
  b.task_index = 2;
  b.executor_id = 3;
  b.round_key = 7;
  b.control_revision = 99;
  b.view_bits = 0x07;
  b.accepted = accepted;
  return b;
}
}  // namespace

TEST(Completion, SizesMatchSpec)
{
  EXPECT_EQ(encodeCompletion(header(), announce()).size(), 39u);   // 宣告
  EXPECT_EQ(encodeCompletion(header(), proof(5)).size(), 85u);     // 證明，K = 5
  EXPECT_EQ(encodeCompletion(header(), proof(0)).size(), 40u);
  EXPECT_EQ(completionProofSize(5), 85u);
  EXPECT_EQ(encodeCompletionAck(header(1), ack()).size(), 39u);    // 確認
}

TEST(Completion, ByteLayoutAnnounce)
{
  CompletionBody b = announce();
  b.relay = {3, 0x0102};
  b.round_key = 0x0A0B0C0D;
  b.control_revision = 0x0102030405060708ULL;
  const auto bytes = encodeCompletion(header(), b);
  EXPECT_EQ(bytes[0], 0x26);   // 代碼 6 | 版本 1
  const std::vector<std::uint8_t> expected = {
    0x03, 0x02, 0x01, 0, 0, 0, 0, 0, 0,               // 轉送表頭
    0x00,                                             // kind = 宣告
    0x02,                                             // task_index
    0x03,                                             // executor_id
    0x0D, 0x0C, 0x0B, 0x0A,                           // round_key
    0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,   // control_revision
    0x1F,                                             // view_bits
  };
  EXPECT_EQ(std::vector<std::uint8_t>(bytes.begin() + kHeaderSize, bytes.end()), expected);
}

TEST(Completion, ByteLayoutProofAcks)
{
  CompletionBody b = proof(0);
  b.acks = {AckRef{2, 0x0506}};
  const auto bytes = encodeCompletion(header(), b);
  const std::vector<std::uint8_t> tail = {
    0x01,                                       // ack_count
    0x02, 0x06, 0x05, 0, 0, 0, 0, 0, 0,         // 確認者 2、序號 0x0506
  };
  EXPECT_EQ(std::vector<std::uint8_t>(bytes.end() - 10, bytes.end()), tail);
  EXPECT_EQ(bytes[kHeaderSize + kRelayHeaderSize], kCompletionProof);
}

TEST(Completion, ByteLayoutAck)
{
  const auto bytes = encodeCompletionAck(header(1), ack(false));
  EXPECT_EQ(bytes[0], 0x28);   // 代碼 8 | 版本 1
  const std::vector<std::uint8_t> expected = {
    0x01, 0x06, 0, 0, 0, 0, 0, 0, 0,   // 轉送表頭
    0x02, 0x03,                        // task_index、executor_id
    0x07, 0, 0, 0,                     // round_key
    0x63, 0, 0, 0, 0, 0, 0, 0,         // control_revision = 99
    0x07,                              // view_bits
    0x00,                              // accepted = 拒絕
  };
  EXPECT_EQ(std::vector<std::uint8_t>(bytes.begin() + kHeaderSize, bytes.end()), expected);
}

TEST(Completion, RoundTrip)
{
  const auto a = decodeCompletion(encodeCompletion(header(), announce()));
  ASSERT_TRUE(a);
  EXPECT_EQ(a->kind, kCompletionAnnounce);
  EXPECT_EQ(a->task_index, 2);
  EXPECT_EQ(a->executor_id, 3);
  EXPECT_EQ(a->round_key, 7u);
  EXPECT_EQ(a->control_revision, 99u);
  EXPECT_EQ(a->view_bits, 0x1F);
  EXPECT_TRUE(a->acks.empty());

  const auto p = decodeCompletion(encodeCompletion(header(), proof(5)));
  ASSERT_TRUE(p);
  EXPECT_EQ(p->kind, kCompletionProof);
  ASSERT_EQ(p->acks.size(), 5u);
  EXPECT_EQ(p->acks[4].agent_id, 5);
  EXPECT_EQ(p->acks[4].ack_seq, 104u);

  for (bool accepted : {true, false}) {
    const auto k = decodeCompletionAck(encodeCompletionAck(header(1), ack(accepted)));
    ASSERT_TRUE(k);
    EXPECT_EQ(k->relay.origin_id, 1);
    EXPECT_EQ(k->executor_id, 3);
    EXPECT_EQ(k->accepted, accepted);
  }
}

TEST(Completion, RelayKeepsBody)
{
  const auto original = encodeCompletion(header(3), proof(2));
  SeqCounter counter;
  const auto fwd = makeRelay(original, 2, counter, 5.0);
  ASSERT_TRUE(fwd);
  const auto p = decodeCompletion(*fwd);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->relay.origin_id, 3);
  EXPECT_EQ(p->acks.size(), 2u);
}

TEST(Completion, RejectsInvalidBody)
{
  CompletionBody b = announce();
  b.kind = 2;
  EXPECT_THROW(encodeCompletion(header(), b), std::invalid_argument);
  b = announce();
  b.acks = {AckRef{1, 1}};                 // 宣告不能帶確認清單
  EXPECT_THROW(encodeCompletion(header(), b), std::invalid_argument);
  b = announce();
  b.task_index = 255;
  EXPECT_THROW(encodeCompletion(header(), b), std::invalid_argument);
  b = announce();
  b.executor_id = 0;
  EXPECT_THROW(encodeCompletion(header(), b), std::invalid_argument);
  b = proof(1);
  b.acks[0].agent_id = 0;
  EXPECT_THROW(encodeCompletion(header(), b), std::invalid_argument);

  CompletionAckBody k = ack();
  k.executor_id = 0;
  EXPECT_THROW(encodeCompletionAck(header(), k), std::invalid_argument);
  k = ack();
  k.relay.origin_id = 0;
  EXPECT_THROW(encodeCompletionAck(header(), k), std::invalid_argument);
}

TEST(Completion, RejectsMalformed)
{
  const std::size_t body_at = kHeaderSize + kRelayHeaderSize;

  const auto a = encodeCompletion(header(), announce());
  auto longer = a;
  longer.push_back(0);
  EXPECT_FALSE(decodeCompletion(longer));                      // 宣告後面多東西
  EXPECT_FALSE(decodeCompletion(a.data(), a.size() - 1));
  auto bad = a;
  bad[body_at] = 2;                                            // kind 2
  EXPECT_FALSE(decodeCompletion(bad));
  bad = a;
  bad[body_at + 1] = 255;                                      // task_index 255
  EXPECT_FALSE(decodeCompletion(bad));
  bad = a;
  bad[body_at + 2] = 0;                                        // executor_id 0
  EXPECT_FALSE(decodeCompletion(bad));
  bad = a;
  bad[0] = typeByte(PacketType::COMPLETION_ACK);               // 種類不對
  EXPECT_FALSE(decodeCompletion(bad));

  const auto p = encodeCompletion(header(), proof(3));
  EXPECT_FALSE(decodeCompletion(p.data(), p.size() - 9));      // 少一筆確認
  bad = p;
  bad[body_at + 16] = 4;                                       // ack_count 和長度對不上
  EXPECT_FALSE(decodeCompletion(bad));
  bad = p;
  bad[body_at + 17] = 0;                                       // 確認者機號 0
  EXPECT_FALSE(decodeCompletion(bad));

  const auto k = encodeCompletionAck(header(1), ack());
  bad = k;
  bad.back() = 2;                                              // accepted 只能 0 或 1
  EXPECT_FALSE(decodeCompletionAck(bad));
  EXPECT_FALSE(decodeCompletionAck(k.data(), k.size() - 1));
  bad = k;
  bad[kHeaderSize] = 0;                                        // origin_id 0
  EXPECT_FALSE(decodeCompletionAck(bad));
}

TEST(Completion, MultiHopRetryNeedsNewSeq)
{
  // uav1 ── uav2 ── uav3（執行機），uav1 和 uav3 互相聽不到。
  // uav1 的確認經 uav2 轉送時掉了，uav3 重送宣告，uav1 回送同樣的決定（接受）。
  RelayDeduper uav2;            // 轉送的飛機：照一般規則去重複
  RelayDeduper uav3;            // 執行機
  SeqCounter uav1_seq;
  SeqCounter uav2_seq;

  const auto send_ack = [&]() {
      Header h = header(1);
      h.seq = uav1_seq.next(PacketType::COMPLETION_ACK);
      CompletionAckBody b = ack();
      b.relay = originOf(h);    // 每次送出都是新的 origin_seq
      return encodeCompletionAck(h, b);
    };
  // uav2 收到後決定要不要轉送；delivered = false 代表這次轉送在 uav2 → uav3 途中掉了
  const auto via_uav2 = [&](const std::vector<std::uint8_t> & packet, double now, bool delivered) {
      const auto relay = parseRelayHeader(packet);
      if (!relay || !uav2.firstSeen(PacketType::COMPLETION_ACK, *relay, now)) {
        return false;           // 被當成重複，不轉送
      }
      const auto fwd = makeRelay(packet, 2, uav2_seq, now);
      if (!fwd || !delivered) {
        return false;
      }
      const auto got = decodeCompletionAck(*fwd);
      return got && uav3.firstSeen(PacketType::COMPLETION_ACK, got->relay, now);
    };

  const auto first = send_ack();
  EXPECT_FALSE(via_uav2(first, 0.0, false));   // 第一次：途中掉了

  // 規格字面的「原樣回送」（序號不變）：uav2 判定重複，永遠送不到
  EXPECT_FALSE(via_uav2(first, 1.0, true));
  EXPECT_FALSE(via_uav2(first, 2.0, true));

  // 換新序號：uav2 照一般規則轉送，uav3 收到
  const auto retry = send_ack();
  EXPECT_TRUE(via_uav2(retry, 3.0, true));
  const auto got = decodeCompletionAck(retry);
  ASSERT_TRUE(got);
  EXPECT_TRUE(got->accepted);                  // 決定和第一次相同
  EXPECT_NE(got->relay.origin_seq, decodeCompletionAck(first)->relay.origin_seq);
}
