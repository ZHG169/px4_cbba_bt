#include "cbba_core/wire_signature.hpp"

#include <cstring>

#include "cbba_core/crypto.hpp"

namespace cbba_core::wire
{

bool isSignedType(PacketType type)
{
  switch (type) {
    case PacketType::TASK_ANNOUNCE:
    case PacketType::TASK_SPEC:
    case PacketType::TASK_CLOSE:
    case PacketType::INCIDENT_UPDATE:
      return true;
    default:
      return false;
  }
}

namespace
{
// 簽的內容：版本、種類、表頭之後的全部、key_id、issued_at_ms
std::vector<std::uint8_t> signingInput(const std::vector<std::uint8_t> & packet, std::uint16_t key_id,
  std::uint64_t issued_ms)
{
  std::vector<std::uint8_t> in;
  in.reserve(packet.size() + 12);
  in.push_back(kVersion);
  in.push_back(packet[3]);   // 種類
  in.insert(in.end(), packet.begin() + static_cast<std::ptrdiff_t>(kHeaderSize), packet.end());
  in.push_back(static_cast<std::uint8_t>(key_id >> 8));
  in.push_back(static_cast<std::uint8_t>(key_id));
  for (int b = 7; b >= 0; --b) {
    in.push_back(static_cast<std::uint8_t>(issued_ms >> (8 * b)));
  }
  return in;
}

void setLength(std::vector<std::uint8_t> & packet)
{
  const auto length = static_cast<std::uint16_t>(packet.size() - kHeaderSize);
  packet[18] = static_cast<std::uint8_t>(length >> 8);   // 表頭的 payload_length（第 18、19 個位元組）
  packet[19] = static_cast<std::uint8_t>(length);
}
}  // namespace

std::optional<std::pair<std::vector<std::uint8_t>, SignatureTrailer>> splitSignature(
  const std::vector<std::uint8_t> & packet)
{
  if (packet.size() < kHeaderSize + kRelaySize + kSigTrailerSize ||
    std::memcmp(packet.data() + packet.size() - 4, kSigMagic, 4) != 0)
  {
    return std::nullopt;
  }
  const std::uint8_t alg = packet[packet.size() - 5];
  if (alg != kSigHmac128) {
    return std::nullopt;
  }
  const std::size_t start = packet.size() - kSigTrailerSize;
  SignatureTrailer sig;
  sig.alg = alg;
  sig.key_id = static_cast<std::uint16_t>((packet[start] << 8) | packet[start + 1]);
  for (int b = 0; b < 8; ++b) {
    sig.issued_ms = (sig.issued_ms << 8) | packet[start + 2 + static_cast<std::size_t>(b)];
  }
  sig.tag.assign(packet.begin() + static_cast<std::ptrdiff_t>(start + 10),
    packet.begin() + static_cast<std::ptrdiff_t>(start + 10 + kSigHmacTag));
  std::vector<std::uint8_t> stripped(packet.begin(), packet.begin() + static_cast<std::ptrdiff_t>(start));
  setLength(stripped);
  return std::make_pair(std::move(stripped), std::move(sig));
}

std::vector<std::uint8_t> appendHmacSignature(const std::vector<std::uint8_t> & packet, std::uint16_t key_id,
  std::uint64_t issued_ms, const std::vector<std::uint8_t> & key)
{
  const Sha256Digest mac = hmacSha256(key, signingInput(packet, key_id, issued_ms));
  std::vector<std::uint8_t> out = packet;
  out.push_back(static_cast<std::uint8_t>(key_id >> 8));
  out.push_back(static_cast<std::uint8_t>(key_id));
  for (int b = 7; b >= 0; --b) {
    out.push_back(static_cast<std::uint8_t>(issued_ms >> (8 * b)));
  }
  out.insert(out.end(), mac.begin(), mac.begin() + kSigHmacTag);
  out.push_back(kSigHmac128);
  out.insert(out.end(), kSigMagic, kSigMagic + 4);
  setLength(out);
  return out;
}

bool verifyHmacSignature(const std::vector<std::uint8_t> & stripped, const SignatureTrailer & sig,
  const std::vector<std::uint8_t> & key)
{
  if (sig.alg != kSigHmac128 || sig.tag.size() != kSigHmacTag || stripped.size() < kHeaderSize) {
    return false;
  }
  const Sha256Digest mac = hmacSha256(key, signingInput(stripped, sig.key_id, sig.issued_ms));
  return constantTimeEqual(mac.data(), sig.tag.data(), kSigHmacTag);
}

}  // namespace cbba_core::wire
