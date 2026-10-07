#include "uav_cbba/wire_header.hpp"

#include <cmath>
#include <iterator>

#include "uav_cbba/wire_io.hpp"

namespace uav_cbba::wire
{

namespace
{
constexpr std::uint8_t kMaxCode = static_cast<std::uint8_t>(PacketType::FORMATION_RELAY);
}  // namespace

void appendHeader(std::vector<std::uint8_t> & out, const Header & header)
{
  ByteWriter w(out);
  w.u8(typeByte(header.type, header.version));
  w.u8(header.agent_id);
  w.u32(header.seq);
  w.u64(header.stamp_us);
}

std::optional<Header> parseHeader(const std::uint8_t * data, std::size_t size)
{
  ByteReader r(data, size);
  const std::uint8_t type_byte = r.u8();
  Header h;
  h.agent_id = r.u8();
  h.seq = r.u32();
  h.stamp_us = r.u64();
  if (!r.ok()) {
    return std::nullopt;   // 長度不足
  }
  const std::uint8_t code = type_byte & 0x1F;
  h.version = static_cast<std::uint8_t>(type_byte >> 5);
  if (code < 1 || code > kMaxCode || h.agent_id == 0) {
    return std::nullopt;
  }
  h.type = static_cast<PacketType>(code);
  const bool version_ok = h.version == kVersion ||
    (h.version == kTaskEventVersion2 && h.type == PacketType::TASK_EVENT);
  if (!version_ok) {
    return std::nullopt;
  }
  return h;
}

bool hasRelayHeader(PacketType type)
{
  switch (type) {
    case PacketType::TASK_EVENT:
    case PacketType::LOST_EVENT:
    case PacketType::COMPLETION:
    case PacketType::COMPLETION_ACK:
    case PacketType::FORMATION_RELAY:
      return true;
    default:
      return false;
  }
}

void appendRelayHeader(std::vector<std::uint8_t> & out, const RelayHeader & relay)
{
  ByteWriter w(out);
  w.u8(relay.origin_id);
  w.u64(relay.origin_seq);
}

std::optional<RelayHeader> parseRelayHeader(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || !hasRelayHeader(header->type) || size < kHeaderSize + kRelayHeaderSize) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize);
  RelayHeader relay;
  relay.origin_id = r.u8();
  relay.origin_seq = r.u64();
  if (relay.origin_id == 0) {
    return std::nullopt;
  }
  return relay;
}

std::uint64_t toStampUs(double seconds)
{
  return static_cast<std::uint64_t>(std::llround(seconds * 1e6));
}

const char * toString(PacketType type)
{
  switch (type) {
    case PacketType::FORMATION: return "FORMATION";
    case PacketType::AGENT_STATE: return "AGENT_STATE";
    case PacketType::ELECTION: return "ELECTION";
    case PacketType::TASK_EVENT: return "TASK_EVENT";
    case PacketType::LOST_EVENT: return "LOST_EVENT";
    case PacketType::COMPLETION: return "COMPLETION";
    case PacketType::DOWNLINK: return "DOWNLINK";
    case PacketType::COMPLETION_ACK: return "COMPLETION_ACK";
    case PacketType::FORMATION_RELAY: return "FORMATION_RELAY";
  }
  return "?";
}

std::uint32_t SeqCounter::next(PacketType type)
{
  return ++seq_[static_cast<std::uint8_t>(type) & 0x1F];
}

std::optional<std::vector<std::uint8_t>> makeRelay(
  const std::vector<std::uint8_t> & packet, std::uint8_t my_id, SeqCounter & counter, double now)
{
  const auto header = parseHeader(packet);
  if (!header || !parseRelayHeader(packet)) {
    return std::nullopt;
  }
  Header mine = *header;
  mine.agent_id = my_id;
  mine.seq = counter.next(header->type);
  mine.stamp_us = toStampUs(now);

  std::vector<std::uint8_t> out;
  out.reserve(packet.size());
  appendHeader(out, mine);
  out.insert(out.end(), packet.begin() + kHeaderSize, packet.end());
  return out;
}

bool SeqFilter::accept(const Header & header, double now)
{
  const auto key = std::make_pair(header.agent_id, static_cast<std::uint8_t>(header.type));
  const auto it = last_.find(key);
  // 循環比較：差值當成有號 32 位元，正的才算比較新（繞回 0 之後照樣判斷正確）
  const bool newer = it == last_.end() ||
    static_cast<std::int32_t>(header.seq - it->second.seq) > 0;
  if (it != last_.end() && !newer &&
    now - it->second.heard <= reset_timeout_)
  {
    return false;   // 舊的或重複的
  }
  last_[key] = Last{header.seq, now};
  return true;
}

bool RelayDeduper::firstSeen(PacketType type, const RelayHeader & relay, double now)
{
  if (now - last_cleanup_ >= memory_ / 4.0) {
    forget(now);
  }
  const Key key{static_cast<std::uint8_t>(type), relay.origin_id, relay.origin_seq};
  const auto it = seen_.find(key);
  if (it != seen_.end() && now - it->second <= memory_) {
    return false;
  }
  seen_[key] = now;   // 第一次，或上次已超過 memory 秒（視為新的）
  return true;
}

void RelayDeduper::forget(double now)
{
  for (auto it = seen_.begin(); it != seen_.end(); ) {
    it = now - it->second > memory_ ? seen_.erase(it) : std::next(it);
  }
  last_cleanup_ = now;
}

}  // namespace uav_cbba::wire
