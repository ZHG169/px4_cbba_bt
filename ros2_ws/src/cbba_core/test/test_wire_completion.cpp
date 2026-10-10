// COMPLETION、COMPLETION_ACK、TASK_CLOSE（協定版本 2）：大小、位元組排列、來回編解碼、拒絕不合法的封包
#include <gtest/gtest.h>

#include <vector>

#include "cbba_core/wire_completion.hpp"

using namespace cbba_core::wire;

namespace
{
Header makeHeader(std::uint16_t sender = 3, std::uint32_t seq = 8)
{
  Header h;
  h.sender_id = sender;
  h.session_id = 0xABCD;
  h.sequence = seq;
  return h;
}

CompletionBody announce()
{
  CompletionBody b;
  b.relay = RelayHeader{3, 0xABCD, 8};
  b.task_id = 0x00010002;
  b.executor_id = 3;
  b.round_key = 0x11223344;
  b.assign_version = 9;
  b.view = {1, 2, 3, 50};
  return b;
}

TaskCloseBody close()
{
  TaskCloseBody b;
  b.relay = RelayHeader{3, 0xABCD, 9};
  b.task_id = 0x00010002;
  b.reason = kCloseDone;
  b.actor_id = 3;
  b.round_key = 0x11223344;
  b.assign_version = 9;
  b.view = {1, 2, 3, 50};
  b.acks = {{1, 100}, {50, 200}};
  return b;
}
}  // namespace

TEST(WireCompletion, Sizes)
{
  EXPECT_EQ(completionSize(0), 53u);
  EXPECT_EQ(completionAckSize(0), 54u);
  EXPECT_EQ(taskCloseSize(0, 0), 55u);
  EXPECT_EQ(encodeCompletion(makeHeader(), announce()).size(), completionSize(4));
  EXPECT_EQ(encodeTaskClose(makeHeader(), close()).size(), taskCloseSize(4, 2));
  EXPECT_EQ(taskCloseSize(4, 2), 55u + 8u + 12u);
}

TEST(WireCompletion, AnnounceLayoutAndRoundTrip)
{
  const auto bytes = encodeCompletion(makeHeader(), announce());
  const std::vector<std::uint8_t> body(bytes.begin() + kHeaderSize + kRelaySize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x00, 0x01, 0x00, 0x02,  // task_id
    0x00, 0x03,              // executor_id
    0x11, 0x22, 0x33, 0x44,  // round_key
    0, 0, 0, 0, 0x00, 0x00, 0x00, 0x09,  // assign_version（uint64）
    0x04, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x32,   // view：機號 50 也放得進去
  };
  EXPECT_EQ(body, expected);
  const auto d = decodeCompletion(bytes);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->relay, announce().relay);
  EXPECT_EQ(d->task_id, 0x00010002u);
  EXPECT_EQ(d->view, (std::vector<std::uint16_t>{1, 2, 3, 50}));
}

TEST(WireCompletion, AckRoundTrip)
{
  CompletionAckBody b;
  b.relay = RelayHeader{50, 0x50, 4};
  b.task_id = 0x00010002;
  b.executor_id = 3;
  b.round_key = 1;
  b.assign_version = 2;
  b.status = kAckStaleAssignment;
  b.view = {3, 50};
  const auto bytes = encodeCompletionAck(makeHeader(50, 4), b);
  EXPECT_EQ(bytes.size(), completionAckSize(2));
  const auto d = decodeCompletionAck(bytes);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->relay.origin_id, 50);
  EXPECT_EQ(d->status, kAckStaleAssignment);
  EXPECT_EQ(d->assign_version, 2u);
  EXPECT_EQ(d->view, (std::vector<std::uint16_t>{3, 50}));

  auto bad = bytes;
  bad[kHeaderSize + kRelaySize + 18] = 3;   // status 只能是 0～2
  EXPECT_FALSE(decodeCompletionAck(bad));
  b.status = 3;
  EXPECT_THROW(encodeCompletionAck(makeHeader(50, 4), b), std::invalid_argument);
}

TEST(WireCompletion, CloseRoundTrip)
{
  const auto d = decodeTaskClose(encodeTaskClose(makeHeader(), close()));
  ASSERT_TRUE(d);
  EXPECT_EQ(d->reason, kCloseDone);
  EXPECT_EQ(d->actor_id, 3);
  EXPECT_EQ(d->assign_version, 9u);
  ASSERT_EQ(d->acks.size(), 2u);
  EXPECT_EQ(d->acks[1].agent_id, 50);
  EXPECT_EQ(d->acks[1].ack_seq, 200u);

  TaskCloseBody cancelled = close();
  cancelled.reason = kCloseCancelled;
  cancelled.acks.clear();
  EXPECT_EQ(decodeTaskClose(encodeTaskClose(makeHeader(), cancelled))->reason, kCloseCancelled);
}

TEST(WireCompletion, RejectsInvalid)
{
  // 種類不符、reason、機號 0、長度不符
  const auto announce_bytes = encodeCompletion(makeHeader(), announce());
  EXPECT_FALSE(decodeTaskClose(announce_bytes));
  EXPECT_FALSE(decodeCompletionAck(announce_bytes));

  TaskCloseBody b = close();
  b.reason = 3;
  EXPECT_THROW(encodeTaskClose(makeHeader(), b), std::invalid_argument);
  b = close();
  b.acks[0].agent_id = 0;
  EXPECT_THROW(encodeTaskClose(makeHeader(), b), std::invalid_argument);
  CompletionBody a = announce();
  a.executor_id = 0;
  EXPECT_THROW(encodeCompletion(makeHeader(), a), std::invalid_argument);
  a = announce();
  a.view = {0};
  EXPECT_THROW(encodeCompletion(makeHeader(), a), std::invalid_argument);

  auto bad = encodeTaskClose(makeHeader(), close());
  bad[kHeaderSize + kRelaySize + 4] = 0;   // reason 0
  EXPECT_FALSE(decodeTaskClose(bad));
  bad = encodeTaskClose(makeHeader(), close());
  bad.pop_back();
  finishPacket(bad);
  EXPECT_FALSE(decodeTaskClose(bad));
}
