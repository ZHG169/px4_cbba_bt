// AGENT_STATE（協定版本 2）：大小、位元組排列、來回編解碼、flags、執行狀態、拒絕不合法的封包
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "cbba_core/wire_agent_state.hpp"

using namespace cbba_core::wire;

namespace
{
Header makeHeader()
{
  Header h;
  h.sender_id = 2;
  h.session_id = 7;
  h.sequence = 1;
  return h;
}

AgentStateBody sample()
{
  AgentStateBody b;
  b.flags.participating = true;
  b.flags.telemetry_ok = true;
  b.flags.flight_state_valid = true;
  b.flags.armed = true;
  b.flags.offboard = true;
  b.vehicle_type = 2;
  b.x = 1.5f;
  b.y = -2.0f;
  b.z = 5.0f;
  b.battery = 87.5f;
  b.exec_known = true;
  b.execution_state = 3;   // 不可中斷
  b.preemptible = false;
  b.remaining_time = 12.5f;
  b.has_active = true;
  b.active_task = 0x00320003;
  b.active_version = 0x0000000100000004ULL;
  b.has_queued = true;
  b.queued_task = 0x00010007;
  b.queued_version = 9;
  b.path = {0x00320003, 0x00010007};
  b.neighbors = {1, 3, 50};
  return b;
}
}  // namespace

TEST(WireAgentState, Size)
{
  EXPECT_EQ(agentStateSize(0, 0), 74u);
  EXPECT_EQ(encodeAgentState(makeHeader(), AgentStateBody{}).size(), 74u);
  EXPECT_EQ(encodeAgentState(makeHeader(), sample()).size(), agentStateSize(2, 3));
  EXPECT_EQ(agentStateSize(2, 3), 74u + 8u + 6u);
}

TEST(WireAgentState, Layout)
{
  // 表頭之後依序：flags、vehicle_type、x、y、z、battery、progress、execution_state、exec_flags、
  // remaining_time、active_task、active_version、queued_task、queued_version、path、neighbors
  AgentStateBody b;
  b.flags.participating = true;
  b.flags.armed = true;
  b.vehicle_type = 2;
  b.x = 1.0f;                // 0x3F800000
  b.exec_known = true;
  b.execution_state = 1;
  b.preemptible = true;
  b.remaining_time = 2.0f;   // 0x40000000
  b.has_active = true;
  b.active_task = 0x00320001;
  b.active_version = 7;
  b.path = {0x00320001};
  b.neighbors = {0x0102};
  const auto bytes = encodeAgentState(makeHeader(), b);
  ASSERT_EQ(bytes.size(), agentStateSize(1, 1));
  const std::vector<std::uint8_t> body(bytes.begin() + kHeaderSize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x09,                    // bit 0 參與、bit 3 armed
    0x02,                    // 地面載具
    0x3F, 0x80, 0x00, 0x00,  // x
    0, 0, 0, 0,  0, 0, 0, 0,  // y、z
    0, 0, 0, 0,              // battery
    0, 0, 0, 0,              // progress
    0x01,                    // execution_state：前往
    0x07,                    // exec_flags：有執行狀態、可以中斷、有執行中的任務（沒有保留）
    0x40, 0x00, 0x00, 0x00,  // remaining_time
    0x00, 0x32, 0x00, 0x01,  // active_task
    0, 0, 0, 0, 0, 0, 0, 0x07,   // active_version（uint64）
    0, 0, 0, 0,              // queued_task
    0, 0, 0, 0, 0, 0, 0, 0,  // queued_version
    0x01, 0x00, 0x32, 0x00, 0x01,   // path
    0x01, 0x01, 0x02,        // neighbors
  };
  EXPECT_EQ(body, expected);
  EXPECT_EQ(parseHeader(bytes)->type, PacketType::AGENT_STATE);
}

TEST(WireAgentState, RoundTrip)
{
  const auto d = decodeAgentState(encodeAgentState(makeHeader(), sample()));
  ASSERT_TRUE(d);
  EXPECT_TRUE(d->flags.participating);
  EXPECT_TRUE(d->flags.flight_state_valid);
  EXPECT_TRUE(d->flags.armed);
  EXPECT_TRUE(d->flags.offboard);
  EXPECT_FALSE(d->flags.landed);
  EXPECT_EQ(d->vehicle_type, 2);
  EXPECT_FLOAT_EQ(d->y, -2.0f);
  EXPECT_FLOAT_EQ(d->battery, 87.5f);
  EXPECT_TRUE(d->exec_known);
  EXPECT_EQ(d->execution_state, 3);
  EXPECT_FALSE(d->preemptible);
  EXPECT_FLOAT_EQ(d->remaining_time, 12.5f);
  EXPECT_TRUE(d->has_active);
  EXPECT_EQ(d->active_task, 0x00320003u);
  EXPECT_EQ(d->active_version, 0x0000000100000004ULL);   // 超過 32 位元也不會截斷
  EXPECT_TRUE(d->has_queued);
  EXPECT_EQ(d->queued_task, 0x00010007u);
  EXPECT_EQ(d->queued_version, 9u);
  EXPECT_EQ(d->path, (std::vector<std::uint32_t>{0x00320003, 0x00010007}));
  EXPECT_EQ(d->neighbors, (std::vector<std::uint16_t>{1, 3, 50}));
}

TEST(WireAgentState, NoTaskIsAFlagNotAnId)
{
  // 沒有執行中、保留的任務用旗標表示
  AgentStateBody b;
  b.active_task = 0x00010001;   // 旗標是 0：這個值沒有意義
  const auto d = decodeAgentState(encodeAgentState(makeHeader(), b));
  ASSERT_TRUE(d);
  EXPECT_FALSE(d->has_active);
  EXPECT_FALSE(d->has_queued);
  EXPECT_FALSE(d->exec_known);
}

TEST(WireAgentState, FlagsBits)
{
  // 每個旗標獨立
  for (int bit = 0; bit < 7; ++bit) {
    const auto f = unpackFlags(static_cast<std::uint8_t>(1u << bit));
    EXPECT_EQ(packFlags(f), 1u << bit);
  }
  StateFlags f;
  f.landed = true;
  f.flight_state_valid = true;
  EXPECT_EQ(packFlags(f), 0x24);
}

TEST(WireAgentState, RejectsInvalid)
{
  const auto good = encodeAgentState(makeHeader(), sample());
  ASSERT_TRUE(decodeAgentState(good));

  auto bad = good;
  bad[kHeaderSize + 1] = 3;                          // vehicle_type
  EXPECT_FALSE(decodeAgentState(bad));
  bad = good;
  bad[kHeaderSize + 22] = 6;                         // execution_state 超過 5
  EXPECT_FALSE(decodeAgentState(bad));
  bad = good;
  bad.pop_back();
  finishPacket(bad);                                 // 長度和筆數對不上
  EXPECT_FALSE(decodeAgentState(bad));
  bad = good;
  bad[bad.size() - 1] = 0;
  bad[bad.size() - 2] = 0;                           // 鄰居機號 0
  EXPECT_FALSE(decodeAgentState(bad));

  AgentStateBody nan = sample();
  nan.x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(decodeAgentState(encodeAgentState(makeHeader(), nan)));

  AgentStateBody wrong = sample();
  wrong.neighbors = {0};
  EXPECT_THROW(encodeAgentState(makeHeader(), wrong), std::invalid_argument);
  wrong = sample();
  wrong.execution_state = 6;
  EXPECT_THROW(encodeAgentState(makeHeader(), wrong), std::invalid_argument);
  wrong = sample();
  wrong.path.assign(256, 0x00010001);
  EXPECT_THROW(encodeAgentState(makeHeader(), wrong), std::invalid_argument);
}
