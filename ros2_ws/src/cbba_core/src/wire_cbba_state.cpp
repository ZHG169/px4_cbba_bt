#include "cbba_core/wire_cbba_state.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
constexpr std::uint8_t kParticipating = 1u << 0;

// task_id 的高 16 位元是建立者機號，不能是 0
bool validTaskId(std::uint32_t id) {return (id >> 16) != 0;}

bool validRecord(const Record & r)
{
  return validTaskId(r.task_id) && std::isfinite(r.winning_bid) && r.winning_bid >= 0.0f &&
         r.winner_id != 0xFFFF && r.assignee_id != 0xFFFF && r.assign_state <= 3 &&
         (r.assign_state == 0 || r.assignee_id != 0);
}
}  // namespace

std::vector<CbbaStateBody> splitCbbaState(const CbbaStateBody & full, std::size_t max_packet)
{
  const std::size_t fixed = cbbaStateSize(full.stamps.size(), 0);
  if (fixed + kRecordSize > max_packet) {
    throw std::invalid_argument("max_packet too small for the stamps");
  }
  const std::size_t per_part = (max_packet - fixed) / kRecordSize;
  const std::size_t count = std::max<std::size_t>(1, (full.records.size() + per_part - 1) / per_part);
  if (count > 255) {
    throw std::invalid_argument("too many records for 255 parts");
  }
  std::vector<CbbaStateBody> parts;
  parts.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    CbbaStateBody p = full;
    p.part_index = static_cast<std::uint8_t>(i);
    p.part_count = static_cast<std::uint8_t>(count);
    const std::size_t begin = std::min(i * per_part, full.records.size());
    const std::size_t end = std::min(begin + per_part, full.records.size());
    p.records.assign(full.records.begin() + static_cast<std::ptrdiff_t>(begin),
      full.records.begin() + static_cast<std::ptrdiff_t>(end));
    parts.push_back(std::move(p));
  }
  return parts;
}

std::vector<std::uint8_t> encodeCbbaState(Header header, const CbbaStateBody & b)
{
  if (b.stamps.size() > 0xFFFF || b.records.size() > 0xFFFF || b.part_count == 0 ||
    b.part_index >= b.part_count)
  {
    throw std::invalid_argument("invalid CBBA_STATE body");
  }
  for (const Stamp & s : b.stamps) {
    if (!validAgentId(s.agent_id)) {
      throw std::invalid_argument("invalid stamp agent id");
    }
  }
  for (const Record & r : b.records) {
    if (!validRecord(r)) {
      throw std::invalid_argument("invalid CBBA_STATE record");
    }
  }
  header.type = PacketType::CBBA_STATE;
  std::vector<std::uint8_t> out;
  out.reserve(cbbaStateSize(b.stamps.size(), b.records.size()));
  appendHeader(out, header);
  ByteWriter w(out);
  w.u32(b.round_key);
  w.u16(b.exchange_step);
  w.u8(b.participating ? kParticipating : 0);
  w.u8(b.stable_steps);
  w.u16(b.snapshot_id);
  w.u8(b.part_index);
  w.u8(b.part_count);
  w.u16(static_cast<std::uint16_t>(b.stamps.size()));
  for (const Stamp & s : b.stamps) {
    w.u16(s.agent_id);
    w.u32(s.stamp_ms);
  }
  w.u16(static_cast<std::uint16_t>(b.records.size()));
  for (const Record & r : b.records) {
    w.u32(r.task_id);
    w.u16(r.winner_id);
    w.f32(r.winning_bid);
    w.u64(r.assign_version);
    w.u16(r.assignee_id);
    w.u8(r.assign_state);
  }
  finishPacket(out);
  return out;
}

std::optional<CbbaStateBody> decodeCbbaState(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::CBBA_STATE || size < kCbbaStateBaseSize) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize);
  CbbaStateBody b;
  b.round_key = r.u32();
  b.exchange_step = r.u16();
  b.participating = (r.u8() & kParticipating) != 0;
  b.stable_steps = r.u8();
  b.snapshot_id = r.u16();
  b.part_index = r.u8();
  b.part_count = r.u8();
  const std::size_t stamps = r.u16();
  if (r.remaining() < stamps * kStampSize) {
    return std::nullopt;   // 先擋掉，不要依假的筆數配置記憶體
  }
  b.stamps.resize(stamps);
  for (Stamp & s : b.stamps) {
    s.agent_id = r.u16();
    s.stamp_ms = r.u32();
  }
  const std::size_t records = r.u16();
  if (!r.ok() || r.remaining() != records * kRecordSize) {
    return std::nullopt;
  }
  b.records.resize(records);
  for (Record & rec : b.records) {
    rec.task_id = r.u32();
    rec.winner_id = r.u16();
    rec.winning_bid = r.f32();
    rec.assign_version = r.u64();
    rec.assignee_id = r.u16();
    rec.assign_state = r.u8();
  }
  if (!r.ok() || b.part_count == 0 || b.part_index >= b.part_count) {
    return std::nullopt;
  }
  for (const Stamp & s : b.stamps) {
    if (!validAgentId(s.agent_id)) {
      return std::nullopt;
    }
  }
  for (const Record & rec : b.records) {
    if (!validRecord(rec)) {
      return std::nullopt;
    }
  }
  return b;
}

}  // namespace cbba_core::wire
