// SHA-256（FIPS 180-2 的範例）、HMAC-SHA256（RFC 4231 的測試向量）：簽章的 PoC（任務欄位補齊 13，2026-11-18）
#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "cbba_core/crypto.hpp"

using namespace cbba_core;

namespace
{
std::string hex(const Sha256Digest & d)
{
  std::string out;
  char buf[3];
  for (const std::uint8_t b : d) {
    std::snprintf(buf, sizeof(buf), "%02x", b);
    out += buf;
  }
  return out;
}

std::vector<std::uint8_t> bytes(const std::string & s)
{
  return std::vector<std::uint8_t>(s.begin(), s.end());
}
}  // namespace

TEST(Crypto, Sha256Vectors)
{
  EXPECT_EQ(hex(sha256(bytes(""))), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  EXPECT_EQ(hex(sha256(bytes("abc"))), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  // 56 B：補位後剛好跨到第二個區塊
  EXPECT_EQ(hex(sha256(bytes("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"))),
    "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  // 100 萬個 a（多個區塊）
  EXPECT_EQ(hex(sha256(std::vector<std::uint8_t>(1000000, 'a'))),
    "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Crypto, HmacRfc4231)
{
  EXPECT_EQ(hex(hmacSha256(std::vector<std::uint8_t>(20, 0x0b), bytes("Hi There"))),
    "b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7");
  EXPECT_EQ(hex(hmacSha256(bytes("Jefe"), bytes("what do ya want for nothing?"))),
    "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843");
  // 金鑰比區塊長（131 B）：先雜湊
  EXPECT_EQ(hex(hmacSha256(std::vector<std::uint8_t>(131, 0xaa),
    bytes("Test Using Larger Than Block-Size Key - Hash Key First"))),
    "60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54");
}

TEST(Crypto, ConstantTimeEqual)
{
  const std::uint8_t a[4] = {1, 2, 3, 4};
  const std::uint8_t b[4] = {1, 2, 3, 5};
  EXPECT_TRUE(constantTimeEqual(a, a, 4));
  EXPECT_FALSE(constantTimeEqual(a, b, 4));
  EXPECT_TRUE(constantTimeEqual(a, b, 3));
}
