#include "uav_cbba/wire_task_event.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "uav_cbba/wire_io.hpp"

namespace uav_cbba::wire
{

namespace
{
bool printable(char c)
{
  return c >= 0x21 && c <= 0x7E;
}

bool finite(float v)
{
  return std::isfinite(v);
}
}  // namespace

bool validTaskName(const std::string & name)
{
  return !name.empty() && name.size() <= kMaxTaskName &&
         std::all_of(name.begin(), name.end(), printable);
}

std::vector<std::uint8_t> encodeTaskEvent(Header header, const TaskEventBody & body)
{
  if (!validTaskName(body.name)) {
    throw std::invalid_argument("TASK_EVENT name must be 1~8 printable ASCII characters");
  }
  if (body.task_index > kMaxTaskIndex) {
    throw std::invalid_argument("TASK_EVENT task_index must be 0~254");
  }
  if (body.relay.origin_id == 0) {
    throw std::invalid_argument("TASK_EVENT origin_id must not be 0");
  }

  header.type = PacketType::TASK_EVENT;
  header.version = body.ext ? kTaskEventVersion2 : kVersion;
  std::vector<std::uint8_t> out;
  out.reserve(taskEventSize(body.name.size(), body.ext.has_value()));
  appendHeader(out, header);
  appendRelayHeader(out, body.relay);

  ByteWriter w(out);
  w.u8(body.task_index);
  w.f32(body.x);
  w.f32(body.y);
  w.f32(body.z);
  w.f32(body.value);
  w.u8(static_cast<std::uint8_t>(body.name.size()));
  for (char c : body.name) {
    w.u8(static_cast<std::uint8_t>(c));
  }
  if (body.ext) {
    w.u8(body.ext->task_type);
    w.u32(body.ext->deadline_ms);
    w.u16(body.ext->duration_sec);
  }
  return out;
}

std::optional<TaskEventBody> decodeTaskEvent(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::TASK_EVENT) {
    return std::nullopt;
  }
  const auto relay = parseRelayHeader(data, size);
  if (!relay) {
    return std::nullopt;
  }

  ByteReader r(data, size, kHeaderSize + kRelayHeaderSize);
  TaskEventBody b;
  b.relay = *relay;
  b.task_index = r.u8();
  b.x = r.f32();
  b.y = r.f32();
  b.z = r.f32();
  b.value = r.f32();
  const std::size_t name_length = r.u8();
  const bool with_ext = header->version == kTaskEventVersion2;
  if (!r.ok() || size != taskEventSize(name_length, with_ext)) {
    return std::nullopt;   // 規格不補位：長度必須剛好
  }
  b.name.resize(name_length);
  for (char & c : b.name) {
    c = static_cast<char>(r.u8());
  }
  if (with_ext) {
    TaskExtension ext;
    ext.task_type = r.u8();
    ext.deadline_ms = r.u32();
    ext.duration_sec = r.u16();
    b.ext = ext;
  }

  if (b.task_index > kMaxTaskIndex || !validTaskName(b.name) ||
    !finite(b.x) || !finite(b.y) || !finite(b.z) || !finite(b.value))
  {
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

}  // namespace uav_cbba::wire
