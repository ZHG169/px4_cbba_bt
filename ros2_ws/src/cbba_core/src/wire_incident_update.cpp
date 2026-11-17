#include "cbba_core/wire_incident_update.hpp"

#include <cmath>
#include <stdexcept>

#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
bool validBody(const IncidentUpdateBody & b)
{
  const auto kind = static_cast<std::uint8_t>(b.kind);
  return validAgentId(b.relay.origin_id) && validAgentId(b.proposer) && cbba_core::validTaskId(b.group_id) &&
         kind >= 1 && kind <= 3 && b.proposal_seq != 0 && static_cast<std::uint8_t>(b.reason) <= 5 &&
         std::isfinite(b.x) && std::isfinite(b.y) && std::isfinite(b.z) && std::isfinite(b.radius) &&
         b.radius >= 0.0F;
}
}  // namespace

const char * toString(IncidentReject r)
{
  switch (r) {
    case IncidentReject::NONE: return "NONE";
    case IncidentReject::STALE: return "STALE";
    case IncidentReject::NOT_EXECUTOR: return "NOT_EXECUTOR";
    case IncidentReject::GROUP_CLOSED: return "GROUP_CLOSED";
    case IncidentReject::BAD_POSITION: return "BAD_POSITION";
    case IncidentReject::UNKNOWN_GROUP: return "UNKNOWN_GROUP";
  }
  return "?";
}

std::vector<std::uint8_t> encodeIncidentUpdate(Header header, const IncidentUpdateBody & b)
{
  if (!validBody(b)) {
    throw std::invalid_argument("invalid INCIDENT_UPDATE body");
  }
  header.type = PacketType::INCIDENT_UPDATE;
  std::vector<std::uint8_t> out;
  out.reserve(kIncidentUpdateSize);
  appendHeader(out, header);
  appendRelay(out, b.relay);
  ByteWriter w(out);
  w.u8(static_cast<std::uint8_t>(b.kind));
  w.u64(b.group_id);
  w.u16(b.incident_revision);
  w.u16(b.proposer);
  w.u32(b.proposal_seq);
  w.u64(b.reporter_task);
  w.u64(b.assign_version);
  w.u8(b.flags);
  w.f32(b.x);
  w.f32(b.y);
  w.f32(b.z);
  w.u8(b.floor);
  w.f32(b.radius);
  w.u32(b.track_id);
  w.u8(static_cast<std::uint8_t>(b.reason));
  finishPacket(out);
  return out;
}

std::optional<IncidentUpdateBody> decodeIncidentUpdate(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::INCIDENT_UPDATE || size != kIncidentUpdateSize) {
    return std::nullopt;
  }
  const auto relay = parseRelay(data, size);
  if (!relay) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelaySize);
  IncidentUpdateBody b;
  b.relay = *relay;
  b.kind = static_cast<IncidentKind>(r.u8());
  b.group_id = r.u64();
  b.incident_revision = r.u16();
  b.proposer = r.u16();
  b.proposal_seq = r.u32();
  b.reporter_task = r.u64();
  b.assign_version = r.u64();
  b.flags = r.u8();
  b.x = r.f32();
  b.y = r.f32();
  b.z = r.f32();
  b.floor = r.u8();
  b.radius = r.f32();
  b.track_id = r.u32();
  b.reason = static_cast<IncidentReject>(r.u8());
  if (!r.ok() || r.remaining() != 0 || !validBody(b)) {
    return std::nullopt;
  }
  return b;
}

}  // namespace cbba_core::wire
