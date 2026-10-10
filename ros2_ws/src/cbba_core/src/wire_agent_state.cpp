#include "cbba_core/wire_agent_state.hpp"

#include <cmath>
#include <stdexcept>

#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
constexpr std::uint8_t kParticipating = 1u << 0;
constexpr std::uint8_t kTelemetryOk = 1u << 1;
constexpr std::uint8_t kFlightStateValid = 1u << 2;
constexpr std::uint8_t kArmed = 1u << 3;
constexpr std::uint8_t kOffboard = 1u << 4;
constexpr std::uint8_t kLanded = 1u << 5;
constexpr std::uint8_t kFollowing = 1u << 6;

constexpr std::uint8_t kExecKnown = 1u << 0;
constexpr std::uint8_t kPreemptible = 1u << 1;
constexpr std::uint8_t kHasActive = 1u << 2;
constexpr std::uint8_t kHasQueued = 1u << 3;

bool validVehicle(std::uint8_t type) {return type == 1 || type == 2;}
}  // namespace

std::uint8_t packFlags(const StateFlags & f)
{
  return static_cast<std::uint8_t>(
    (f.participating ? kParticipating : 0) | (f.telemetry_ok ? kTelemetryOk : 0) |
    (f.flight_state_valid ? kFlightStateValid : 0) | (f.armed ? kArmed : 0) |
    (f.offboard ? kOffboard : 0) | (f.landed ? kLanded : 0) | (f.following ? kFollowing : 0));
}

StateFlags unpackFlags(std::uint8_t flags)
{
  StateFlags f;
  f.participating = (flags & kParticipating) != 0;
  f.telemetry_ok = (flags & kTelemetryOk) != 0;
  f.flight_state_valid = (flags & kFlightStateValid) != 0;
  f.armed = (flags & kArmed) != 0;
  f.offboard = (flags & kOffboard) != 0;
  f.landed = (flags & kLanded) != 0;
  f.following = (flags & kFollowing) != 0;
  return f;
}

std::vector<std::uint8_t> encodeAgentState(Header header, const AgentStateBody & b)
{
  if (b.path.size() > 255 || b.neighbors.size() > 255 || !validVehicle(b.vehicle_type) ||
    b.execution_state > kMaxExecutionState)
  {
    throw std::invalid_argument("invalid AGENT_STATE body");
  }
  for (std::uint16_t n : b.neighbors) {
    if (!validAgentId(n)) {
      throw std::invalid_argument("invalid neighbor id");
    }
  }
  header.type = PacketType::AGENT_STATE;
  std::vector<std::uint8_t> out;
  out.reserve(agentStateSize(b.path.size(), b.neighbors.size()));
  appendHeader(out, header);
  ByteWriter w(out);
  w.u8(packFlags(b.flags));
  w.u8(b.vehicle_type);
  w.f32(b.x);
  w.f32(b.y);
  w.f32(b.z);
  w.f32(b.battery);
  w.f32(b.progress);
  w.u8(b.execution_state);
  w.u8(static_cast<std::uint8_t>((b.exec_known ? kExecKnown : 0) | (b.preemptible ? kPreemptible : 0) |
    (b.has_active ? kHasActive : 0) | (b.has_queued ? kHasQueued : 0)));
  w.f32(b.remaining_time);
  w.u32(b.active_task);
  w.u64(b.active_version);
  w.u32(b.queued_task);
  w.u64(b.queued_version);
  w.u8(static_cast<std::uint8_t>(b.path.size()));
  for (std::uint32_t t : b.path) {
    w.u32(t);
  }
  w.u8(static_cast<std::uint8_t>(b.neighbors.size()));
  for (std::uint16_t n : b.neighbors) {
    w.u16(n);
  }
  finishPacket(out);
  return out;
}

std::optional<AgentStateBody> decodeAgentState(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::AGENT_STATE || size < kAgentStateBaseSize) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize);
  AgentStateBody b;
  b.flags = unpackFlags(r.u8());
  b.vehicle_type = r.u8();
  b.x = r.f32();
  b.y = r.f32();
  b.z = r.f32();
  b.battery = r.f32();
  b.progress = r.f32();
  b.execution_state = r.u8();
  const std::uint8_t exec_flags = r.u8();
  b.exec_known = (exec_flags & kExecKnown) != 0;
  b.preemptible = (exec_flags & kPreemptible) != 0;
  b.has_active = (exec_flags & kHasActive) != 0;
  b.has_queued = (exec_flags & kHasQueued) != 0;
  b.remaining_time = r.f32();
  b.active_task = r.u32();
  b.active_version = r.u64();
  b.queued_task = r.u32();
  b.queued_version = r.u64();
  b.path.resize(r.u8());
  for (auto & t : b.path) {
    t = r.u32();
  }
  b.neighbors.resize(r.u8());
  for (auto & n : b.neighbors) {
    n = r.u16();
  }
  if (!r.ok() || r.remaining() != 0 || !validVehicle(b.vehicle_type) ||
    b.execution_state > kMaxExecutionState || !std::isfinite(b.x) || !std::isfinite(b.y) ||
    !std::isfinite(b.z) || !std::isfinite(b.battery) || std::isnan(b.remaining_time))
  {
    return std::nullopt;
  }
  for (std::uint16_t n : b.neighbors) {
    if (!validAgentId(n)) {
      return std::nullopt;
    }
  }
  return b;
}

}  // namespace cbba_core::wire
