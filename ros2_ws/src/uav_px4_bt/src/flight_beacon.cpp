#include "uav_px4_bt/flight_beacon.hpp"

#include "cbba_core/wire_io.hpp"

namespace uav_px4_bt
{

using cbba_core::wire::ByteReader;
using cbba_core::wire::ByteWriter;

std::vector<std::uint8_t> encodeBeacon(const Beacon & b)
{
  std::vector<std::uint8_t> out;
  out.reserve(kBeaconSize);
  ByteWriter w(out);
  w.u16(kBeaconMagic);
  w.u8(kBeaconVersion);
  w.u16(b.sender);
  w.u64(b.session);
  w.u32(b.sequence);
  w.u8(static_cast<std::uint8_t>((b.airborne ? 0x01 : 0x00) | (b.position_valid ? 0x02 : 0x00) |
    (b.busy ? 0x04 : 0x00)));
  for (double v : {b.position.x, b.position.y, b.position.z, b.velocity.x, b.velocity.y, b.velocity.z}) {
    w.f32(static_cast<float>(v));
  }
  return out;
}

std::optional<Beacon> decodeBeacon(const std::uint8_t * data, std::size_t size)
{
  if (size != kBeaconSize) {
    return std::nullopt;
  }
  ByteReader r(data, size);
  if (r.u16() != kBeaconMagic || r.u8() != kBeaconVersion) {
    return std::nullopt;
  }
  Beacon b;
  b.sender = r.u16();
  b.session = r.u64();
  b.sequence = r.u32();
  const std::uint8_t flags = r.u8();
  b.airborne = (flags & 0x01) != 0;
  b.position_valid = (flags & 0x02) != 0;
  b.busy = (flags & 0x04) != 0;
  b.position.x = r.f32();
  b.position.y = r.f32();
  b.position.z = r.f32();
  b.velocity.x = r.f32();
  b.velocity.y = r.f32();
  b.velocity.z = r.f32();
  if (!r.ok() || b.sender == 0) {
    return std::nullopt;
  }
  return b;
}

bool NeighborTable::receive(const std::uint8_t * data, std::size_t size, double now)
{
  const auto b = decodeBeacon(data, size);
  if (!b || b->sender == self_id_) {
    return false;
  }
  const auto it = table_.find(b->sender);
  if (it != table_.end() && it->second.beacon.session == b->session &&
    static_cast<std::int32_t>(b->sequence - it->second.beacon.sequence) <= 0)
  {
    return false;   // 同一個 session 的舊封包（亂序）
  }
  table_[b->sender] = Entry{*b, now};
  return true;
}

std::vector<Neighbor> NeighborTable::neighbors(double now, double timeout) const
{
  std::vector<Neighbor> out;
  for (const auto & [id, e] : table_) {
    const double age = now - e.received;
    if (age > timeout || !e.beacon.airborne || !e.beacon.position_valid) {
      continue;
    }
    out.push_back(Neighbor{id, e.beacon.position, e.beacon.velocity, age, e.beacon.busy});
  }
  return out;
}

}  // namespace uav_px4_bt
