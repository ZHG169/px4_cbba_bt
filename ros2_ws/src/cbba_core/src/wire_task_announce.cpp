#include "cbba_core/wire_task_announce.hpp"

#include <cmath>
#include <stdexcept>

#include "cbba_core/types.hpp"
#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
constexpr std::uint8_t kDeadlineHard = 1u << 0;
constexpr std::uint8_t kOptional = 1u << 1;
constexpr std::uint8_t kAllOrNothing = 1u << 2;
constexpr std::uint8_t kSkipSatisfies = 1u << 3;

bool validBody(const TaskAnnounceBody & b)
{
  const bool roles_ok = b.role_count <= kMaxRoles && (b.role_count == 0 || b.role_index < b.role_count);
  return cbba_core::validTaskId(b.task_id) && (b.group_id == 0 || cbba_core::validTaskId(b.group_id)) &&
         b.task_type <= 3 && b.platform <= 2 && b.priority <= 3 && roles_ok &&
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
  w.u64(b.task_id);
  w.u16(b.revision);
  w.u8(b.task_type);
  w.u8(b.platform);
  w.u64(b.required_caps);
  w.u8(b.priority);
  w.u8(static_cast<std::uint8_t>((b.deadline_hard ? kDeadlineHard : 0) | (b.optional ? kOptional : 0) |
    (b.all_or_nothing ? kAllOrNothing : 0) | (b.skip_satisfies ? kSkipSatisfies : 0)));
  w.f32(b.x);
  w.f32(b.y);
  w.f32(b.z);
  w.u8(b.floor);
  w.f32(b.value);
  w.u32(b.created_ms);
  w.u32(b.release_ms);
  w.u32(b.deadline_ms);
  w.u32(b.expires_ms);
  w.u16(b.duration_sec);
  w.u16(b.half_life_s);
  w.u8(b.can_preempt);
  w.u8(b.preempt_margin_pct);
  w.u16(b.commit_radius_dm);
  w.u8(b.min_battery_reserve);
  w.u64(b.group_id);
  w.u8(b.role_index);
  w.u8(b.role_count);
  w.u8(b.precedence);
  w.u16(b.sync_window_s);
  w.u16(b.optional_no_capable_s);
  w.u16(b.optional_unassigned_s);
  w.u32(b.spec_length);
  w.u32(b.spec_crc);
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
  b.task_id = r.u64();
  b.revision = r.u16();
  b.task_type = r.u8();
  b.platform = r.u8();
  b.required_caps = r.u64();
  b.priority = r.u8();
  const std::uint8_t flags = r.u8();
  b.deadline_hard = (flags & kDeadlineHard) != 0;
  b.optional = (flags & kOptional) != 0;
  b.all_or_nothing = (flags & kAllOrNothing) != 0;
  b.skip_satisfies = (flags & kSkipSatisfies) != 0;
  b.x = r.f32();
  b.y = r.f32();
  b.z = r.f32();
  b.floor = r.u8();
  b.value = r.f32();
  b.created_ms = r.u32();
  b.release_ms = r.u32();
  b.deadline_ms = r.u32();
  b.expires_ms = r.u32();
  b.duration_sec = r.u16();
  b.half_life_s = r.u16();
  b.can_preempt = r.u8();
  b.preempt_margin_pct = r.u8();
  b.commit_radius_dm = r.u16();
  b.min_battery_reserve = r.u8();
  b.group_id = r.u64();
  b.role_index = r.u8();
  b.role_count = r.u8();
  b.precedence = r.u8();
  b.sync_window_s = r.u16();
  b.optional_no_capable_s = r.u16();
  b.optional_unassigned_s = r.u16();
  b.spec_length = r.u32();
  b.spec_crc = r.u32();
  if (!r.ok() || r.remaining() != 0 || !validBody(b)) {
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
