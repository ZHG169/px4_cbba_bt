// 協定版本 2 的表頭與轉送區塊：位元組排列（大端序）、拒絕不合法的表頭、session 過濾、轉送與去重複
#include <gtest/gtest.h>

#include <vector>

#include "cbba_core/wire_header.hpp"

using namespace cbba_core::wire;

namespace
{
Header makeHeader(PacketType type = PacketType::AGENT_STATE, std::uint16_t sender = 0x0102,
  std::uint32_t seq = 0x0A0B0C0D, std::uint64_t session = 0x1122334455667788ULL)
{
  Header h;
  h.type = type;
  h.sender_id = sender;
  h.session_id = session;
  h.sequence = seq;
  return h;
}

// 表頭加上 n 個位元組的內容，長度填好
std::vector<std::uint8_t> packet(const Header & h, std::size_t n = 0)
{
  std::vector<std::uint8_t> out;
  appendHeader(out, h);
  out.insert(out.end(), n, 0xAB);
  finishPacket(out);
  return out;
}
}  // namespace

TEST(WireHeader, LayoutIsBigEndian)
{
  // 欄位依序、網路位元組順序，payload_length 由 finishPacket 填
  const auto out = packet(makeHeader(PacketType::CBBA_STATE), 3);
  const std::vector<std::uint8_t> expected = {
    0x43, 0x42,                                      // magic "CB"
    0x02,                                            // version
    0x02,                                            // type = CBBA_STATE
    0x01, 0x02,                                      // sender_id
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,  // session_id
    0x0A, 0x0B, 0x0C, 0x0D,                          // sequence
    0x00, 0x03,                                      // payload_length
    0xAB, 0xAB, 0xAB,
  };
  EXPECT_EQ(out, expected);
}

TEST(WireHeader, RoundTrip)
{
  const auto bytes = packet(makeHeader(PacketType::TASK_CLOSE, 50, 42), 5);
  const auto h = parseHeader(bytes);
  ASSERT_TRUE(h);
  EXPECT_EQ(h->type, PacketType::TASK_CLOSE);
  EXPECT_EQ(h->sender_id, 50);
  EXPECT_EQ(h->sequence, 42u);
  EXPECT_EQ(h->session_id, 0x1122334455667788ULL);
  EXPECT_EQ(h->payload_length, 5);
}

TEST(WireHeader, RejectsInvalid)
{
  // 長度不足、magic、版本、種類、sender_id 0、payload_length 對不上都拒收
  const auto good = packet(makeHeader(), 4);
  ASSERT_TRUE(parseHeader(good));
  EXPECT_FALSE(parseHeader(good.data(), kHeaderSize - 1));
  EXPECT_FALSE(parseHeader(nullptr, 0));

  auto bad = good;
  bad[0] = 0x00;
  EXPECT_FALSE(parseHeader(bad));                      // magic
  bad = good;
  bad[2] = 1;
  EXPECT_FALSE(parseHeader(bad));                      // 版本 1（規格書的格式）
  bad = good;
  bad[3] = 0;
  EXPECT_FALSE(parseHeader(bad));                      // 種類 0
  bad[3] = 7;
  EXPECT_FALSE(parseHeader(bad));                      // 不認得的種類
  EXPECT_FALSE(parseHeader(packet(makeHeader(PacketType::AGENT_STATE, 0))));        // 0 = 無人
  EXPECT_FALSE(parseHeader(packet(makeHeader(PacketType::AGENT_STATE, 0xFFFF))));   // 保留
  bad = good;
  bad.push_back(0);
  EXPECT_FALSE(parseHeader(bad));                      // 多一個位元組
  bad = good;
  bad.pop_back();
  EXPECT_FALSE(parseHeader(bad));                      // 少一個位元組
}

TEST(WireHeader, RelayBlock)
{
  // 只有 TASK_ANNOUNCE、COMPLETION、COMPLETION_ACK、TASK_CLOSE 有轉送區塊
  EXPECT_FALSE(isRelayed(PacketType::AGENT_STATE));
  EXPECT_FALSE(isRelayed(PacketType::CBBA_STATE));
  EXPECT_TRUE(isRelayed(PacketType::TASK_ANNOUNCE));
  EXPECT_TRUE(isRelayed(PacketType::COMPLETION));
  EXPECT_TRUE(isRelayed(PacketType::COMPLETION_ACK));
  EXPECT_TRUE(isRelayed(PacketType::TASK_CLOSE));

  std::vector<std::uint8_t> bytes;
  appendHeader(bytes, makeHeader(PacketType::TASK_ANNOUNCE));
  appendRelay(bytes, RelayHeader{0x0032, 0xAABBCCDDEEFF0011ULL, 0x01020304});
  finishPacket(bytes);
  ASSERT_EQ(bytes.size(), kHeaderSize + kRelaySize);
  const std::vector<std::uint8_t> relay(bytes.begin() + kHeaderSize, bytes.end());
  const std::vector<std::uint8_t> expected = {
    0x00, 0x32,
    0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11,
    0x01, 0x02, 0x03, 0x04,
  };
  EXPECT_EQ(relay, expected);
  const auto r = parseRelay(bytes);
  ASSERT_TRUE(r);
  EXPECT_EQ(*r, (RelayHeader{50, 0xAABBCCDDEEFF0011ULL, 0x01020304}));

  // 種類沒有轉送區塊、origin_id 0 時拒收
  EXPECT_FALSE(parseRelay(packet(makeHeader(PacketType::AGENT_STATE), kRelaySize)));
  bytes[kHeaderSize] = 0;
  bytes[kHeaderSize + 1] = 0;
  EXPECT_FALSE(parseRelay(bytes));
}

TEST(WireHeader, RelayKeepsPayloadAndReplacesHeader)
{
  // 轉送：換上自己的 sender、session、sequence，payload（含轉送區塊）原封不動
  std::vector<std::uint8_t> original;
  appendHeader(original, makeHeader(PacketType::COMPLETION, 3, 7, 0x33));
  appendRelay(original, RelayHeader{3, 0x33, 7});
  original.insert(original.end(), {1, 2, 3});
  finishPacket(original);

  SeqCounter counter;
  counter.next();
  const auto fwd = makeRelay(original, 2, 0x22, counter);
  ASSERT_TRUE(fwd);
  const auto h = parseHeader(*fwd);
  ASSERT_TRUE(h);
  EXPECT_EQ(h->sender_id, 2);
  EXPECT_EQ(h->session_id, 0x22u);
  EXPECT_EQ(h->sequence, 2u);
  EXPECT_EQ(h->type, PacketType::COMPLETION);
  EXPECT_TRUE(std::equal(original.begin() + kHeaderSize, original.end(), fwd->begin() + kHeaderSize));
  EXPECT_EQ(*parseRelay(*fwd), (RelayHeader{3, 0x33, 7}));

  EXPECT_FALSE(makeRelay(packet(makeHeader(PacketType::CBBA_STATE)), 2, 0x22, counter));   // 不轉送的種類
}

TEST(SessionFilter, AcceptsReorderedDropsDuplicates)
{
  // 同一個 session：視窗內亂序到達的照樣接受，收過的丟掉；比視窗還舊的丟掉
  SessionFilter f(100);
  EXPECT_TRUE(f.accept(makeHeader(PacketType::TASK_ANNOUNCE, 1, 10, 0xA)));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::TASK_ANNOUNCE, 1, 12, 0xA)));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::TASK_ANNOUNCE, 1, 11, 0xA)));    // 晚到的
  EXPECT_FALSE(f.accept(makeHeader(PacketType::TASK_ANNOUNCE, 1, 11, 0xA)));   // 重複
  EXPECT_FALSE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 12, 0xA)));     // 同一個號碼
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 200, 0xA)));
  EXPECT_FALSE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 100, 0xA)));    // 掉出視窗
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 150, 0xA)));     // 視窗內、沒收過
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 2, 1, 0xB)));       // 別台
  EXPECT_FALSE(f.restarted(1));
}

TEST(SessionFilter, WrapsAround)
{
  // 循環比較：0xFFFFFFFF 之後的 0、1 算比較新
  SessionFilter f;
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 0xFFFFFFFE, 0xA)));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 0xFFFFFFFF, 0xA)));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 0, 0xA)));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 1, 0xA)));
  EXPECT_FALSE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 0xFFFFFFFF, 0xA)));
}

TEST(SessionFilter, NewSessionAcceptedOldSessionRejected)
{
  // 重開機：新 session 從 1 開始也接受；重開機前、延遲到現在的封包丟掉
  SessionFilter f;
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 500, 0xA)));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 1, 0xB)));
  EXPECT_TRUE(f.restarted(1));
  EXPECT_FALSE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 501, 0xA)));
  EXPECT_TRUE(f.accept(makeHeader(PacketType::CBBA_STATE, 1, 2, 0xB)));
}

TEST(RelayDeduper, KeyIncludesTypeAndSession)
{
  // 同一則事件只處理一次；種類或 session 不同就是不同的事件（重開機後序號重來也不會被擋）
  RelayDeduper d(60.0);
  const RelayHeader r{3, 0xA, 1};
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_ANNOUNCE, r, 0.0));
  EXPECT_FALSE(d.firstSeen(PacketType::TASK_ANNOUNCE, r, 1.0));
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_CLOSE, r, 1.0));
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_ANNOUNCE, RelayHeader{3, 0xB, 1}, 1.0));
  EXPECT_TRUE(d.firstSeen(PacketType::TASK_ANNOUNCE, r, 62.0));   // 記憶過期
}
