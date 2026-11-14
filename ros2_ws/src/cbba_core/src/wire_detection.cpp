#include "cbba_core/wire_detection.hpp"

#include <stdexcept>

#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
bool validBody(const DetectionBody & b)
{
  return validAgentId(b.relay.origin_id) && b.detection_seq != 0 &&
         b.detector.size() <= kMaxDetectionName && b.source.size() <= kMaxDetectionName &&
         b.label.size() <= kMaxDetectionName && b.attrs.size() <= kMaxDetectionAttrs;
}

bool validBody(const DetectionAckBody & b)
{
  return validAgentId(b.relay.origin_id) && validAgentId(b.target_id) && !b.seqs.empty() &&
         b.seqs.size() <= kMaxAckSeqs;
}

void appendString(std::vector<std::uint8_t> & out, const std::string & s)
{
  out.insert(out.end(), s.begin(), s.end());
}
}  // namespace

std::vector<std::uint8_t> encodeDetection(Header header, const DetectionBody & b)
{
  if (!validBody(b)) {
    throw std::invalid_argument("invalid DETECTION body");
  }
  header.type = PacketType::DETECTION;
  std::vector<std::uint8_t> out;
  out.reserve(kDetectionBaseSize + b.detector.size() + b.source.size() + b.label.size() + b.attrs.size());
  appendHeader(out, header);
  appendRelay(out, b.relay);
  ByteWriter w(out);
  w.u32(b.detection_seq);
  w.u64(b.t_ms);
  w.u8(b.flags);
  w.f32(b.score);
  w.f32(b.x);
  w.f32(b.y);
  w.f32(b.z);
  w.u8(b.floor);
  for (float v : b.bbox) {
    w.f32(v);
  }
  w.u32(b.track_id);
  w.u8(static_cast<std::uint8_t>(b.detector.size()));
  w.u8(static_cast<std::uint8_t>(b.source.size()));
  w.u8(static_cast<std::uint8_t>(b.label.size()));
  w.u16(static_cast<std::uint16_t>(b.attrs.size()));
  appendString(out, b.detector);
  appendString(out, b.source);
  appendString(out, b.label);
  appendString(out, b.attrs);
  finishPacket(out);
  return out;
}

std::optional<DetectionBody> decodeDetection(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::DETECTION || size < kDetectionBaseSize) {
    return std::nullopt;
  }
  const auto relay = parseRelay(data, size);
  if (!relay) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelaySize);
  DetectionBody b;
  b.relay = *relay;
  b.detection_seq = r.u32();
  b.t_ms = r.u64();
  b.flags = r.u8();
  b.score = r.f32();
  b.x = r.f32();
  b.y = r.f32();
  b.z = r.f32();
  b.floor = r.u8();
  for (float & v : b.bbox) {
    v = r.f32();
  }
  b.track_id = r.u32();
  const std::size_t detector = r.u8();
  const std::size_t source = r.u8();
  const std::size_t label = r.u8();
  const std::size_t attrs = r.u16();
  if (!r.ok() || r.remaining() != detector + source + label + attrs) {
    return std::nullopt;
  }
  const char * p = reinterpret_cast<const char *>(data + kDetectionBaseSize);
  b.detector.assign(p, detector);
  p += detector;
  b.source.assign(p, source);
  p += source;
  b.label.assign(p, label);
  p += label;
  b.attrs.assign(p, attrs);
  if (!validBody(b)) {
    return std::nullopt;
  }
  return b;
}

std::vector<std::uint8_t> encodeDetectionAck(Header header, const DetectionAckBody & b)
{
  if (!validBody(b)) {
    throw std::invalid_argument("invalid DETECTION_ACK body");
  }
  header.type = PacketType::DETECTION_ACK;
  std::vector<std::uint8_t> out;
  out.reserve(kDetectionAckBaseSize + 4 * b.seqs.size());
  appendHeader(out, header);
  appendRelay(out, b.relay);
  ByteWriter w(out);
  w.u16(b.target_id);
  w.u64(b.target_session);
  w.u8(static_cast<std::uint8_t>(b.seqs.size()));
  for (std::uint32_t seq : b.seqs) {
    w.u32(seq);
  }
  finishPacket(out);
  return out;
}

std::optional<DetectionAckBody> decodeDetectionAck(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::DETECTION_ACK || size < kDetectionAckBaseSize) {
    return std::nullopt;
  }
  const auto relay = parseRelay(data, size);
  if (!relay) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelaySize);
  DetectionAckBody b;
  b.relay = *relay;
  b.target_id = r.u16();
  b.target_session = r.u64();
  const std::size_t count = r.u8();
  if (!r.ok() || r.remaining() != 4 * count) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < count; ++i) {
    b.seqs.push_back(r.u32());
  }
  if (!validBody(b)) {
    return std::nullopt;
  }
  return b;
}

}  // namespace cbba_core::wire
