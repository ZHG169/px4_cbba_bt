// SHA-256 與 HMAC-SHA256（FIPS 180-4、RFC 2104；任務欄位補齊 13「簽章」的 PoC，2026-11-18）。
// 自己實作、不加 OpenSSL 之類的依賴（cbba_core 不依賴其他函式庫）；只給封包簽章用，不追求速度。
// 正式部署建議 Ed25519（wire_signature.hpp 的 alg 欄位留了位置）
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace cbba_core
{

using Sha256Digest = std::array<std::uint8_t, 32>;

Sha256Digest sha256(const std::uint8_t * data, std::size_t size);
inline Sha256Digest sha256(const std::vector<std::uint8_t> & data) {return sha256(data.data(), data.size());}

Sha256Digest hmacSha256(const std::vector<std::uint8_t> & key, const std::uint8_t * data, std::size_t size);
inline Sha256Digest hmacSha256(const std::vector<std::uint8_t> & key, const std::vector<std::uint8_t> & data)
{
  return hmacSha256(key, data.data(), data.size());
}

// 不依內容提早結束的比較（驗簽時不洩漏第幾個位元組不同）
bool constantTimeEqual(const std::uint8_t * a, const std::uint8_t * b, std::size_t size);

}  // namespace cbba_core
