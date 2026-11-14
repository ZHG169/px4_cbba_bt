// DETECTION（種類 10）、DETECTION_ACK（種類 11）：機器人的偵測送給閘道（協定版本 4，2026-11-14；
// doc/cbba_m0_contract.md「統一由閘道建立任務」、2-1b 第七節的 Detection）。兩種都轉送（多跳）。
//
// DETECTION：機器人 → 閘道
//   表頭 20 | 轉送區塊 14 | detection_seq 4 | t_ms 8 | flags 1 | score 4 | x 4 | y 4 | z 4 | floor 1 |
//   bbox 4×4 | track_id 4 | detector_len 1 | source_len 1 | label_len 1 | attrs_len 2 | detector | source | label | attrs
//   大小 = 89 + 字串長度（最多 89 + 3 × 64 + 900 = 1181 B，在 1200 B 以內）
//
//   detection_seq  origin 這個 session 的偵測流水號（1 起）。閘道去重複的鍵是 (origin_id, origin_session, detection_seq)：
//                  重送一律換新的封包序號與 origin_seq（協定的規則），所以不能用轉送區塊去重複
//   t_ms           偵測的時刻：系統時鐘 ms（64 位元，各機對時）
//   flags          bit 0 位置有效、bit 1 暫存過（閘道斷線時產生，恢復後才送出）；其他保留 0
//   score、x、y、z、bbox  float32（IEEE 754，大端序）；位置是 map ENU；bbox 是影像上的 x、y、w、h（px），全 0 = 沒有
//   track_id       偵測器的追蹤編號；0 = 沒有
//   detector、source、label  ≤ 64 B（UTF-8，不含結尾的 0）；attrs ≤ 900 B（其他屬性，JSON；CBBA 不解析，EVT-1 用）
//
// DETECTION_ACK：閘道 → 機器人（收到就回，重複的也回：上一次的確認可能掉了）
//   表頭 20 | 轉送區塊 14 | target_id 2 | target_session 8 | count 1 | detection_seq 4 × count
//   count 1～kMaxAckSeqs
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kDetectionBaseSize = kHeaderSize + kRelaySize + 55;
constexpr std::size_t kMaxDetectionName = 64;
constexpr std::size_t kMaxDetectionAttrs = 900;
constexpr std::uint8_t kDetectionPositionValid = 0x01;
constexpr std::uint8_t kDetectionBuffered = 0x02;

constexpr std::size_t kDetectionAckBaseSize = kHeaderSize + kRelaySize + 11;
constexpr std::size_t kMaxAckSeqs = 64;

struct DetectionBody
{
  RelayHeader relay;
  std::uint32_t detection_seq{0};
  std::uint64_t t_ms{0};
  std::uint8_t flags{0};
  float score{0.0F};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t floor{0};
  std::array<float, 4> bbox{};
  std::uint32_t track_id{0};
  std::string detector;
  std::string source;
  std::string label;
  std::string attrs;
};

struct DetectionAckBody
{
  RelayHeader relay;
  std::uint16_t target_id{0};
  std::uint64_t target_session{0};
  std::vector<std::uint32_t> seqs;
};

// 編碼成完整封包；header.type 會設成對應的種類。欄位不合法（origin_id、detection_seq 0、字串太長、
// target_id 不合法、seqs 是空的或超過 kMaxAckSeqs）時丟出 std::invalid_argument
std::vector<std::uint8_t> encodeDetection(Header header, const DetectionBody & body);
std::vector<std::uint8_t> encodeDetectionAck(Header header, const DetectionAckBody & body);

// 解碼完整封包。表頭或轉送區塊不合法、種類不對、長度不符、欄位不合法時回傳 nullopt
std::optional<DetectionBody> decodeDetection(const std::uint8_t * data, std::size_t size);
inline std::optional<DetectionBody> decodeDetection(const std::vector<std::uint8_t> & bytes)
{
  return decodeDetection(bytes.data(), bytes.size());
}
std::optional<DetectionAckBody> decodeDetectionAck(const std::uint8_t * data, std::size_t size);
inline std::optional<DetectionAckBody> decodeDetectionAck(const std::vector<std::uint8_t> & bytes)
{
  return decodeDetectionAck(bytes.data(), bytes.size());
}

}  // namespace cbba_core::wire
