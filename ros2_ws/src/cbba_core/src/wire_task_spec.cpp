#include "cbba_core/wire_task_spec.hpp"

#include <algorithm>
#include <stdexcept>

#include "cbba_core/types.hpp"
#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
bool validBody(const TaskSpecBody & b)
{
  return cbba_core::validTaskId(b.task_id) && validAgentId(b.relay.origin_id) && b.part_count >= 1 &&
         b.part_index < b.part_count && !b.data.empty() && b.data.size() <= kMaxSpecChunk;
}
}  // namespace

std::vector<std::uint8_t> encodeTaskSpec(Header header, const TaskSpecBody & b)
{
  if (!validBody(b)) {
    throw std::invalid_argument("invalid TASK_SPEC body");
  }
  header.type = PacketType::TASK_SPEC;
  std::vector<std::uint8_t> out;
  out.reserve(taskSpecSize(b.data.size()));
  appendHeader(out, header);
  appendRelay(out, b.relay);
  ByteWriter w(out);
  w.u64(b.task_id);
  w.u16(b.revision);
  w.u8(b.encoding);
  w.u8(b.part_index);
  w.u8(b.part_count);
  w.u16(static_cast<std::uint16_t>(b.data.size()));
  out.insert(out.end(), b.data.begin(), b.data.end());
  finishPacket(out);
  return out;
}

std::optional<TaskSpecBody> decodeTaskSpec(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::TASK_SPEC || size < kTaskSpecBaseSize) {
    return std::nullopt;
  }
  const auto relay = parseRelay(data, size);
  if (!relay) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelaySize);
  TaskSpecBody b;
  b.relay = *relay;
  b.task_id = r.u64();
  b.revision = r.u16();
  b.encoding = r.u8();
  b.part_index = r.u8();
  b.part_count = r.u8();
  const std::size_t length = r.u16();
  if (!r.ok() || r.remaining() != length) {
    return std::nullopt;
  }
  b.data.assign(data + kTaskSpecBaseSize, data + size);
  if (!validBody(b)) {
    return std::nullopt;
  }
  return b;
}

std::vector<std::vector<std::uint8_t>> splitSpec(const std::string & spec)
{
  std::vector<std::vector<std::uint8_t>> parts;
  for (std::size_t pos = 0; pos < spec.size(); pos += kMaxSpecChunk) {
    const std::size_t end = std::min(pos + kMaxSpecChunk, spec.size());
    parts.emplace_back(spec.begin() + static_cast<std::ptrdiff_t>(pos),
      spec.begin() + static_cast<std::ptrdiff_t>(end));
  }
  if (parts.size() > kMaxSpecParts) {
    throw std::invalid_argument("TaskSpec too large for 255 parts");
  }
  return parts;
}

std::uint32_t specCrc(const std::string & spec)
{
  std::uint32_t crc = 0xFFFFFFFFu;
  for (const char ch : spec) {
    crc ^= static_cast<std::uint8_t>(ch);
    for (int k = 0; k < 8; ++k) {
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
  }
  return ~crc;
}

}  // namespace cbba_core::wire
