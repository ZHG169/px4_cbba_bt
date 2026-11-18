// 封包簽章（任務欄位補齊 13，協定版本 5，2026-11-18）：簽章的格式、竄改、未授權的 issuer、重放、未授權的取消、
// 非 issuer 的補發帶著原本的簽章、SIGN／REQUIRE 模式
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/gateway.hpp"
#include "cbba_core/wire_completion.hpp"
#include "cbba_core/wire_signature.hpp"
#include "cbba_core/wire_task_announce.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
constexpr AgentId kGateway = 200;

std::vector<std::uint8_t> secretOf(AgentId agent, std::uint16_t key_id)
{
  std::vector<std::uint8_t> s(32);
  for (std::size_t i = 0; i < s.size(); ++i) {
    s[i] = static_cast<std::uint8_t>(agent * 7 + key_id * 13 + i);
  }
  return s;
}

// 閘道、無人機 1、狗 50、51 都有金鑰；without 的機號沒有金鑰
CommConfig secureConfig(SecurityConfig::Mode mode, std::set<AgentId> issuers = {}, AgentId without = 0)
{
  CommConfig cc;
  cc.epoch = 3;
  cc.security.mode = mode;
  cc.security.task_issuers = std::move(issuers);
  for (AgentId a : {kGateway, AgentId{1}, AgentId{50}, AgentId{51}}) {
    if (a != without) {
      cc.security.keys.push_back(SecurityConfig::Key{a, 1, secretOf(a, 1)});
    }
  }
  return cc;
}

Swarm makeSwarm(const CommConfig & cc)
{
  Swarm swarm(0.0, 1, cc);
  swarm.addGateway(kGateway);
  swarm.add(1, {0, 0, 5});
  swarm.add(50, {5, 0, 0}, 100.0, AgentType::UGV);
  swarm.add(51, {30, 0, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  return swarm;
}

InjectRequest p0(const std::string & incident)
{
  InjectRequest req;
  req.incident_id = incident;
  req.revision = 1;
  req.task.platform = 2;
  req.task.required_caps = capBit(Capability::MOBILITY_GROUND);
  req.task.priority = 0;
  req.task.position = {12.0, 0.0, 0.0};
  req.task.value = 100.0;
  req.task.duration_sec = 5.0;
  return req;
}

// 錄下的封包換成一個陌生的發送者（新的 session、序號），模擬「從網路上抓下來再送」
std::vector<std::uint8_t> resend(std::vector<std::uint8_t> packet, std::uint16_t sender = 99)
{
  packet[4] = static_cast<std::uint8_t>(sender >> 8);
  packet[5] = static_cast<std::uint8_t>(sender);
  for (int i = 6; i < 14; ++i) {
    packet[static_cast<std::size_t>(i)] = 0x5A;
  }
  packet[14] = 0;
  packet[15] = 0;
  packet[16] = 0;
  packet[17] = 7;
  return packet;
}

std::vector<std::uint8_t> firstOf(const std::vector<std::vector<std::uint8_t>> & packets, PacketType type)
{
  for (const auto & p : packets) {
    if (wire::parseHeader(p)->type == type) {
      return p;
    }
  }
  return {};
}
}  // namespace

TEST(Signature, TrailerRoundTripAndTamper)
{
  // 格式：加上 31 B 的簽章、表頭長度跟著改；拿掉後和原本一樣；錯的金鑰、改一個位元組都驗不過；沒簽章的認不出簽章
  wire::Header h;
  h.sender_id = 200;
  h.session_id = 1;
  h.sequence = 1;
  wire::TaskAnnounceBody b;
  b.relay = wire::RelayHeader{200, 1, 1};
  b.task_id = makeTaskId(200, 3, 1);
  b.task_type = 0;
  const auto plain = wire::encodeTaskAnnounce(h, b);
  const auto key = secretOf(200, 1);
  const auto signed_packet = wire::appendHmacSignature(plain, 1, 1700000000000ULL, key);
  EXPECT_EQ(signed_packet.size(), plain.size() + wire::kSigTrailerSize);
  ASSERT_TRUE(wire::parseHeader(signed_packet));   // payload_length 有改
  const auto split = wire::splitSignature(signed_packet);
  ASSERT_TRUE(split);
  EXPECT_EQ(split->first, plain);
  EXPECT_EQ(split->second.key_id, 1);
  EXPECT_EQ(split->second.issued_ms, 1700000000000ULL);
  EXPECT_TRUE(wire::verifyHmacSignature(split->first, split->second, key));
  EXPECT_FALSE(wire::verifyHmacSignature(split->first, split->second, secretOf(1, 1)));
  auto tampered = split->first;
  tampered[60] ^= 0x01;   // 任務定義裡的一個位元組（位置）
  EXPECT_FALSE(wire::verifyHmacSignature(tampered, split->second, key));
  EXPECT_FALSE(wire::splitSignature(plain));
  EXPECT_TRUE(wire::isSignedType(PacketType::TASK_CLOSE));
  EXPECT_FALSE(wire::isSignedType(PacketType::CBBA_STATE));
}

TEST(Signature, SignedTaskReachesTeamAndTamperIsRejected)
{
  // 13 驗收 1：REQUIRE 模式，閘道注入的任務帶簽章、全隊驗過收下；改了位置（簽章失效）的同一則被丟掉、內容不變
  Swarm swarm = makeSwarm(secureConfig(SecurityConfig::Mode::REQUIRE));
  swarm.record(kGateway);
  Gateway gw(swarm.node(kGateway));
  const InjectResult r = gw.inject(p0("sig-1"), swarm.now());
  ASSERT_EQ(r.status, InjectStatus::CREATED) << r.reason;
  swarm.run(2.0);
  for (AgentId a : {AgentId{1}, AgentId{50}, AgentId{51}}) {
    EXPECT_EQ(swarm.node(a).agent().tasks().count(r.task_id), 1u) << a;
    EXPECT_GT(swarm.node(a).stats().signature_ok, 0) << a;
    EXPECT_EQ(swarm.node(a).stats().bad_signatures, 0) << a;
  }
  EXPECT_GT(swarm.node(kGateway).stats().signed_out, 0);

  auto announce = firstOf(swarm.recorded(), PacketType::TASK_ANNOUNCE);
  ASSERT_FALSE(announce.empty());
  announce = resend(announce);
  announce[wire::kHeaderSize + wire::kRelaySize + 8 + 2 + 1 + 1 + 8 + 1 + 1] ^= 0x40;   // x 的第一個位元組
  swarm.node(50).receive(announce, swarm.now());
  EXPECT_EQ(swarm.node(50).stats().bad_signatures, 1);
  EXPECT_NEAR(swarm.node(50).agent().tasks().at(r.task_id).position.x, 12.0, 1e-3);
}

TEST(Signature, UnauthorizedIssuerIsRejectedAndNotRelayed)
{
  // 13 驗收 2：只有閘道能建任務（task_issuers {200}）。無人機自己建的（簽章對，但不是授權的 issuer）→ 狗 50 丟掉、
  // 不轉送：只聽得到 50 的狗 51 根本沒看到
  Swarm swarm = makeSwarm(secureConfig(SecurityConfig::Mode::REQUIRE, {kGateway}));
  swarm.cut(1, 51);
  swarm.cut(kGateway, 51);
  swarm.run(1.0);
  const Task t = newTask(1, 7, 10.0, 0.0, swarm.now());
  ASSERT_TRUE(swarm.node(1).addLocalTask(t, swarm.now()));
  swarm.run(3.0);
  EXPECT_EQ(swarm.node(50).agent().tasks().count(t.id), 0u);
  EXPECT_GT(swarm.node(50).stats().unauthorized_issuers, 0);
  EXPECT_EQ(swarm.node(51).agent().tasks().count(t.id), 0u);
  EXPECT_EQ(swarm.node(51).stats().unauthorized_issuers, 0);   // 50 沒轉送
}

TEST(Signature, ReplayIsDetected)
{
  // 13 驗收 3：同一個 nonce（origin_session、origin_seq）再送一次 → 判定為重放（signed_repeats），去重複擋下、狀態不變
  Swarm swarm = makeSwarm(secureConfig(SecurityConfig::Mode::REQUIRE));
  Gateway gw(swarm.node(kGateway));
  const InjectResult r = gw.inject(p0("sig-2"), swarm.now());
  ASSERT_EQ(r.status, InjectStatus::CREATED);
  swarm.run(2.0);
  swarm.record(kGateway);
  ASSERT_EQ(swarm.node(kGateway).cancelTask(r.task_id, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(50).agent().tasks().at(r.task_id).status, TaskStatus::CANCELLED);
  const auto close = firstOf(swarm.recorded(), PacketType::TASK_CLOSE);
  ASSERT_FALSE(close.empty());
  const int repeats = swarm.node(50).stats().signed_repeats;
  const int duplicates = swarm.node(50).stats().duplicates;
  swarm.node(50).receive(resend(close), swarm.now());
  EXPECT_EQ(swarm.node(50).stats().signed_repeats, repeats + 1);
  EXPECT_EQ(swarm.node(50).stats().duplicates, duplicates + 1);
  EXPECT_EQ(swarm.node(50).stats().bad_signatures, 0);
}

TEST(Signature, UnauthorizedCancelOfP0IsRejected)
{
  // 13 驗收 4：無人機想取消閘道的 P0 任務。① 冒充閘道（origin 200）但只有自己的金鑰 → 簽章不對；② 用自己的名義簽
  //（簽章對）→ 照舊的取消權限（建立者或 cancel_authorities）拒絕。任務都還開著
  Swarm swarm = makeSwarm(secureConfig(SecurityConfig::Mode::REQUIRE));
  Gateway gw(swarm.node(kGateway));
  const InjectResult r = gw.inject(p0("sig-3"), swarm.now());
  ASSERT_EQ(r.status, InjectStatus::CREATED);
  swarm.run(2.0);

  const auto forged = [&](AgentId origin, std::uint16_t key_owner) {
      wire::Header h;
      h.sender_id = 99;
      h.session_id = 0x77;
      h.sequence = 1;
      wire::TaskCloseBody c;
      c.relay = wire::RelayHeader{origin, swarm.node(origin).sessionId(), 900000};   // 真的 session、不會撞到的序號
      c.task_id = r.task_id;
      c.reason = wire::kCloseCancelled;
      c.actor_id = origin;
      return wire::appendHmacSignature(wire::encodeTaskClose(h, c), 1, 1700000000000ULL, secretOf(key_owner, 1));
    };
  swarm.node(50).receive(forged(kGateway, 1), swarm.now());   // 冒充閘道
  EXPECT_EQ(swarm.node(50).stats().bad_signatures, 1);
  const int rejected = swarm.node(50).stats().rejected_closes;
  auto own = forged(1, 1);
  own[17] = 2;   // 換一個序號
  swarm.node(50).receive(own, swarm.now());                   // 自己的名義
  EXPECT_EQ(swarm.node(50).stats().rejected_closes, rejected + 1);
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(50).agent().tasks().at(r.task_id).status, TaskStatus::OPEN);
}

TEST(Signature, RepairFromNonIssuerKeepsSignature)
{
  // 狗 51 晚加入、閘道已經離線：任務由無人機、狗 50（不是 issuer）用 SYNC_BATCH 補給它，帶著閘道原本的簽章 → 51 驗得過
  Swarm swarm = makeSwarm(secureConfig(SecurityConfig::Mode::REQUIRE));
  swarm.setOnline(51, false);
  Gateway gw(swarm.node(kGateway));
  const InjectResult r = gw.inject(p0("sig-4"), swarm.now());
  ASSERT_EQ(r.status, InjectStatus::CREATED);
  swarm.run(2.0);
  swarm.setOnline(kGateway, false);
  swarm.setOnline(51, true);
  swarm.run(6.0);
  EXPECT_EQ(swarm.node(51).agent().tasks().count(r.task_id), 1u);
  EXPECT_EQ(swarm.node(51).stats().bad_signatures, 0);
  EXPECT_EQ(swarm.node(51).stats().unsigned_rejected, 0);
}

TEST(Signature, SignModeAcceptsUnsignedRequireDoesNot)
{
  // 過渡：沒有金鑰的狗 51 建的任務沒簽章。SIGN 模式照收；REQUIRE 模式丟掉
  {
    Swarm swarm = makeSwarm(secureConfig(SecurityConfig::Mode::SIGN, {}, 51));
    const Task t = newTask(51, 3, 30.0, 0.0, swarm.now());
    ASSERT_TRUE(swarm.node(51).addLocalTask(t, swarm.now()));
    swarm.run(3.0);
    EXPECT_EQ(swarm.node(50).agent().tasks().count(t.id), 1u);
  }
  {
    Swarm swarm = makeSwarm(secureConfig(SecurityConfig::Mode::REQUIRE, {}, 51));
    const Task t = newTask(51, 3, 30.0, 0.0, swarm.now());
    ASSERT_TRUE(swarm.node(51).addLocalTask(t, swarm.now()));
    swarm.run(3.0);
    EXPECT_EQ(swarm.node(50).agent().tasks().count(t.id), 0u);
    EXPECT_GT(swarm.node(50).stats().unsigned_rejected, 0);
  }
}
