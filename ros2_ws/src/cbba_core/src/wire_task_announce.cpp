#include "cbba_core/wire_task_announce.hpp"

#include <cmath>
#include <stdexcept>

#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
bool validBody(const TaskAnnounceBody & b)
{
  return (b.task_id >> 16) != 0 && b.task_type >= 1 && b.task_type <= 3 &&
         std::isfinite(b.x) && std::isfinite(b.y) && std::isfinite(b.z) && std::isfinite(b.value) &&
         validAgentId(b.relay.origin_id);
}
}  // namespace

std::vector<std::uint8_t> encodeTaskAnnounce(Header header, const TaskAnnounceBody & b)
{
  if (!validBody(b)) {
    throw std::invalid_argument("invalid TASK_ANNOUNCE body");
  }
  header.type = PacketType::TASK_ANNOUNCE;
  std::vector<std::uint8_t> out;
  out.reserve(kTaskAnnounceSize);
  appendHeader(out, header);
  appendRelay(out, b.relay);
  ByteWriter w(out);
  w.u32(b.task_id);
  w.u8(b.task_type);
  w.f32(b.x);
  w.f32(b.y);
  w.f32(b.z);
  w.f32(b.value);
  w.u32(b.created_ms);
  w.u32(b.deadline_ms);
  w.u16(b.duration_sec);
  finishPacket(out);
  return out;
}

std::optional<TaskAnnounceBody> decodeTaskAnnounce(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::TASK_ANNOUNCE || size != kTaskAnnounceSize) {
    return std::nullopt;
  }
  const auto relay = parseRelay(data, size);
  if (!relay) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelaySize);
  TaskAnnounceBody b;
  b.relay = *relay;
  b.task_id = r.u32();
  b.task_type = r.u8();
  b.x = r.f32();
  b.y = r.f32();
  b.z = r.f32();
  b.value = r.f32();
  b.created_ms = r.u32();
  b.deadline_ms = r.u32();
  b.duration_sec = r.u16();
  if (!r.ok() || !validBody(b)) {
    return std::nullopt;
  }
  return b;
}

std::uint32_t toDeadlineMs(double deadline_seconds)
{
  const auto ms = static_cast<std::uint32_t>(
    static_cast<std::uint64_t>(std::llround(deadline_seconds * 1e3)));
  return ms == 0 ? 1 : ms;
}

double fromDeadlineMs(std::uint32_t deadline_ms, double now)
{
  const auto now_ms = static_cast<std::int64_t>(std::llround(now * 1e3));
  const auto diff = static_cast<std::int32_t>(deadline_ms - static_cast<std::uint32_t>(now_ms));
  return static_cast<double>(now_ms + diff) / 1e3;
}

}  // namespace cbba_core::wire
