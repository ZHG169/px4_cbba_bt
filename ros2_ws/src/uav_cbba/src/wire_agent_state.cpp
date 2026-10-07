#include "uav_cbba/wire_agent_state.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>

#include "uav_cbba/wire_io.hpp"

namespace uav_cbba::wire
{

namespace
{
std::uint8_t lengthOf(std::size_t n, const char * what)
{
  if (n > 255) {
    throw std::invalid_argument(std::string("AGENT_STATE ") + what + " has more than 255 entries");
  }
  return static_cast<std::uint8_t>(n);
}

constexpr std::uint16_t kArmed = 1u << 8;
constexpr std::uint16_t kOffboard = 1u << 9;
constexpr std::uint16_t kFollowing = 1u << 10;
constexpr std::uint16_t kTelemetryOk = 1u << 11;
constexpr unsigned kStableShift = 12;
}  // namespace

std::vector<std::uint8_t> encodeAgentState(Header header, const AgentStateBody & body)
{
  if (body.z.size() != body.y.size()) {
    throw std::invalid_argument("AGENT_STATE z and y must have the same length");
  }
  const std::uint8_t m = lengthOf(body.z.size(), "z/y");
  const std::uint8_t n = lengthOf(body.s.size(), "s");
  const std::uint8_t lt = lengthOf(body.path.size(), "path");

  header.type = PacketType::AGENT_STATE;
  std::vector<std::uint8_t> out;
  out.reserve(agentStateSize(m, n, lt));
  appendHeader(out, header);

  ByteWriter w(out);
  w.u32(body.round_key);
  w.u16(body.exchange_step);
  w.u8(m);
  w.u8(n);
  w.u8(lt);
  for (std::uint8_t v : body.z) {w.u8(v);}
  for (float v : body.y) {w.f32(v);}
  for (std::uint32_t v : body.s) {w.u32(v);}
  for (std::uint8_t v : body.path) {w.u8(v);}
  w.f32(body.soc);
  w.u16(body.flags);
  w.u8(body.progress_task);
  w.f32(body.progress);
  return out;
}

std::optional<AgentStateBody> decodeAgentState(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::AGENT_STATE) {
    return std::nullopt;
  }

  ByteReader r(data, size, kHeaderSize);
  AgentStateBody b;
  b.round_key = r.u32();
  b.exchange_step = r.u16();
  const std::size_t m = r.u8();
  const std::size_t n = r.u8();
  const std::size_t lt = r.u8();
  if (!r.ok() || size != agentStateSize(m, n, lt)) {
    return std::nullopt;   // 規格不補位：長度必須剛好
  }

  b.z.resize(m);
  b.y.resize(m);
  b.s.resize(n);
  b.path.resize(lt);
  for (auto & v : b.z) {v = r.u8();}
  for (auto & v : b.y) {v = r.f32();}
  for (auto & v : b.s) {v = r.u32();}
  for (auto & v : b.path) {v = r.u8();}
  b.soc = r.f32();
  b.flags = r.u16();
  b.progress_task = r.u8();
  b.progress = r.f32();
  return b;
}

std::uint16_t packFlags(const StateFlags & f)
{
  const unsigned stable = std::min<unsigned>(f.stable_steps, 15);
  return static_cast<std::uint16_t>(
    f.neighbors | (f.armed ? kArmed : 0) | (f.offboard ? kOffboard : 0) |
    (f.following ? kFollowing : 0) | (f.telemetry_ok ? kTelemetryOk : 0) |
    (stable << kStableShift));
}

StateFlags unpackFlags(std::uint16_t flags)
{
  StateFlags f;
  f.neighbors = static_cast<std::uint8_t>(flags & 0xFF);
  f.armed = (flags & kArmed) != 0;
  f.offboard = (flags & kOffboard) != 0;
  f.following = (flags & kFollowing) != 0;
  f.telemetry_ok = (flags & kTelemetryOk) != 0;
  f.stable_steps = static_cast<std::uint8_t>(flags >> kStableShift);
  return f;
}

}  // namespace uav_cbba::wire
