// AGENT_STATE：大小與位元組排列和規格書 §2.4 一致、來回編解碼、拒絕不合法的封包、flags
#include <gtest/gtest.h>

#include <stdexcept>

#include "uav_cbba/wire_agent_state.hpp"

using namespace uav_cbba::wire;

namespace
{
Header header(std::uint8_t agent = 2)
{
  Header h;
  h.agent_id = agent;
  h.seq = 7;
  h.stamp_us = 1000;
  return h;
}

AgentStateBody body(std::size_t m, std::size_t n, std::size_t lt)
{
  AgentStateBody b;
  b.round_key = 3;
  b.exchange_step = 12;
  b.z.assign(m, 2);
  b.y.assign(m, 61.5f);
  b.s.assign(n, 4000u);
  b.path.assign(lt, 1);
  b.soc = 0.75f;
  b.flags = 0x3F06;
  b.progress_task = 1;
  b.progress = 0.5f;
  return b;
}
}  // namespace

TEST(AgentState, SizesMatchSpec)
{
  // 規格 §2.4 與 §4 的數字
  EXPECT_EQ(encodeAgentState(header(), body(0, 0, 0)).size(), 34u);    // 尚未分配
  EXPECT_EQ(encodeAgentState(header(), body(5, 5, 1)).size(), 80u);    // 5 架 5 任務、Lt = 1
  EXPECT_EQ(encodeAgentState(header(), body(6, 5, 1)).size(), 85u);    // 5 架 6 任務
  EXPECT_EQ(encodeAgentState(header(), body(20, 10, 5)).size(), 179u); // 擴展估算
  EXPECT_EQ(agentStateSize(20, 10, 5), 179u);
}

TEST(AgentState, ByteLayout)
{
  AgentStateBody b;
  b.round_key = 0x01020304;
  b.exchange_step = 0x0506;
  b.z = {3};
  b.y = {1.0f};          // 0x3F800000
  b.s = {0x0A0B0C0D};
  b.path = {9};
  b.soc = 0.5f;          // 0x3F000000
  b.flags = 0x1203;
  b.progress_task = 0xFF;
  b.progress = 0.0f;
  const auto bytes = encodeAgentState(header(), b);
  ASSERT_EQ(bytes.size(), agentStateSize(1, 1, 1));
  EXPECT_EQ(bytes[0], 0x22);   // 表頭：代碼 2 | 版本 1

  const std::vector<std::uint8_t> expected_body = {
    0x04, 0x03, 0x02, 0x01,   // round_key
    0x06, 0x05,               // exchange_step
    0x01, 0x01, 0x01,         // M、N、Lt
    0x03,                     // z
    0x00, 0x00, 0x80, 0x3F,   // y = 1.0f
    0x0D, 0x0C, 0x0B, 0x0A,   // s
    0x09,                     // path
    0x00, 0x00, 0x00, 0x3F,   // soc = 0.5f
    0x03, 0x12,               // flags
    0xFF,                     // progress_task
    0x00, 0x00, 0x00, 0x00,   // progress
  };
  EXPECT_EQ(std::vector<std::uint8_t>(bytes.begin() + kHeaderSize, bytes.end()), expected_body);
}

TEST(AgentState, RoundTrip)
{
  const AgentStateBody in = body(4, 3, 2);
  const auto bytes = encodeAgentState(header(5), in);
  const auto h = parseHeader(bytes);
  ASSERT_TRUE(h);
  EXPECT_EQ(h->type, PacketType::AGENT_STATE);
  EXPECT_EQ(h->agent_id, 5);

  const auto out = decodeAgentState(bytes);
  ASSERT_TRUE(out);
  EXPECT_EQ(out->round_key, in.round_key);
  EXPECT_EQ(out->exchange_step, in.exchange_step);
  EXPECT_EQ(out->z, in.z);
  EXPECT_EQ(out->y, in.y);
  EXPECT_EQ(out->s, in.s);
  EXPECT_EQ(out->path, in.path);
  EXPECT_FLOAT_EQ(out->soc, in.soc);
  EXPECT_EQ(out->flags, in.flags);
  EXPECT_EQ(out->progress_task, in.progress_task);
  EXPECT_FLOAT_EQ(out->progress, in.progress);
}

TEST(AgentState, HeaderTypeIsForced)
{
  Header h = header();
  h.type = PacketType::TASK_EVENT;   // 傳錯也會改成 AGENT_STATE
  const auto bytes = encodeAgentState(h, body(0, 0, 0));
  EXPECT_EQ(bytes[0], typeByte(PacketType::AGENT_STATE));
}

TEST(AgentState, RejectsMalformed)
{
  const auto good = encodeAgentState(header(), body(3, 2, 1));
  EXPECT_FALSE(decodeAgentState(good.data(), good.size() - 1));   // 少 1 B
  auto longer = good;
  longer.push_back(0);
  EXPECT_FALSE(decodeAgentState(longer));                         // 多 1 B
  EXPECT_FALSE(decodeAgentState(good.data(), kHeaderSize + 4));   // 讀不到 M、N、Lt

  auto wrong_type = good;
  wrong_type[0] = typeByte(PacketType::COMPLETION);
  EXPECT_FALSE(decodeAgentState(wrong_type));

  auto bad_header = good;
  bad_header[1] = 0;                                              // agent_id 0
  EXPECT_FALSE(decodeAgentState(bad_header));

  auto wrong_m = good;
  wrong_m[kHeaderSize + 6] = 4;                                   // M 和實際長度對不上
  EXPECT_FALSE(decodeAgentState(wrong_m));
}

TEST(AgentState, RejectsInvalidBody)
{
  AgentStateBody b = body(3, 1, 0);
  b.y.pop_back();
  EXPECT_THROW(encodeAgentState(header(), b), std::invalid_argument);   // z、y 長度不同
  EXPECT_THROW(encodeAgentState(header(), body(256, 1, 0)), std::invalid_argument);
  EXPECT_NO_THROW(encodeAgentState(header(), body(255, 1, 0)));
}

TEST(AgentState, Flags)
{
  StateFlags f;
  f.neighbors = neighborBit(1) | neighborBit(3);
  f.armed = true;
  f.telemetry_ok = true;
  f.stable_steps = 4;
  const std::uint16_t packed = packFlags(f);
  EXPECT_EQ(packed, 0x4905);   // 鄰居 0x05、armed bit 8、遙測 bit 11、穩定 4 步

  const StateFlags back = unpackFlags(packed);
  EXPECT_EQ(back.neighbors, 0x05);
  EXPECT_TRUE(back.armed);
  EXPECT_FALSE(back.offboard);
  EXPECT_FALSE(back.following);
  EXPECT_TRUE(back.telemetry_ok);
  EXPECT_EQ(back.stable_steps, 4);

  f.stable_steps = 40;   // 只有 4 bit，存成 15
  EXPECT_EQ(unpackFlags(packFlags(f)).stable_steps, 15);
  EXPECT_EQ(unpackFlags(packFlags(f)).neighbors, 0x05);   // 不會溢位到其他欄位
}

TEST(AgentState, NeighborBit)
{
  EXPECT_EQ(neighborBit(1), 0x01);
  EXPECT_EQ(neighborBit(8), 0x80);
  EXPECT_EQ(neighborBit(0), 0);
  EXPECT_EQ(neighborBit(9), 0);
  EXPECT_EQ(neighborBit(50), 0);   // 機器狗放不進 8 個 bit
}
