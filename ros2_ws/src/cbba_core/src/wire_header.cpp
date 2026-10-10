#include "cbba_core/wire_header.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>

#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
constexpr std::uint8_t kMaxType = static_cast<std::uint8_t>(PacketType::TASK_CLOSE);
constexpr std::size_t kLengthOffset = 18;   // payload_length 在表頭裡的位置
}  // namespace

void appendHeader(std::vector<std::uint8_t> & out, const Header & header)
{
  ByteWriter w(out);
  w.u16(kMagic);
  w.u8(kVersion);
  w.u8(static_cast<std::uint8_t>(header.type));
  w.u16(header.sender_id);
  w.u64(header.session_id);
  w.u32(header.sequence);
  w.u16(header.payload_length);
}

void finishPacket(std::vector<std::uint8_t> & packet)
{
  if (packet.size() < kHeaderSize || packet.size() - kHeaderSize > 0xFFFF) {
    throw std::invalid_argument("payload too long");
  }
  const auto length = static_cast<std::uint16_t>(packet.size() - kHeaderSize);
  packet[kLengthOffset] = static_cast<std::uint8_t>(length >> 8);
  packet[kLengthOffset + 1] = static_cast<std::uint8_t>(length);
}

std::optional<Header> parseHeader(const std::uint8_t * data, std::size_t size)
{
  ByteReader r(data, size);
  const std::uint16_t magic = r.u16();
  const std::uint8_t version = r.u8();
  const std::uint8_t type = r.u8();
  Header h;
  h.sender_id = r.u16();
  h.session_id = r.u64();
  h.sequence = r.u32();
  h.payload_length = r.u16();
  if (!r.ok() || magic != kMagic || version != kVersion || type < 1 || type > kMaxType ||
    !validAgentId(h.sender_id) || h.payload_length != size - kHeaderSize)
  {
    return std::nullopt;
  }
  h.type = static_cast<PacketType>(type);
  return h;
}

bool isRelayed(PacketType type)
{
  switch (type) {
    case PacketType::TASK_ANNOUNCE:
    case PacketType::COMPLETION:
    case PacketType::COMPLETION_ACK:
    case PacketType::TASK_CLOSE:
      return true;
    default:
      return false;
  }
}

void appendRelay(std::vector<std::uint8_t> & out, const RelayHeader & relay)
{
  ByteWriter w(out);
  w.u16(relay.origin_id);
  w.u64(relay.origin_session);
  w.u32(relay.origin_seq);
}

std::optional<RelayHeader> parseRelay(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || !isRelayed(header->type)) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize);
  RelayHeader relay;
  relay.origin_id = r.u16();
  relay.origin_session = r.u64();
  relay.origin_seq = r.u32();
  if (!r.ok() || !validAgentId(relay.origin_id)) {
    return std::nullopt;
  }
  return relay;
}

const char * toString(PacketType type)
{
  switch (type) {
    case PacketType::AGENT_STATE: return "AGENT_STATE";
    case PacketType::CBBA_STATE: return "CBBA_STATE";
    case PacketType::TASK_ANNOUNCE: return "TASK_ANNOUNCE";
    case PacketType::COMPLETION: return "COMPLETION";
    case PacketType::COMPLETION_ACK: return "COMPLETION_ACK";
    case PacketType::TASK_CLOSE: return "TASK_CLOSE";
  }
  return "?";
}

std::optional<std::vector<std::uint8_t>> makeRelay(
  const std::vector<std::uint8_t> & packet, std::uint16_t my_id, std::uint64_t my_session,
  SeqCounter & counter)
{
  const auto header = parseHeader(packet);
  if (!header || !parseRelay(packet)) {
    return std::nullopt;
  }
  Header mine = *header;
  mine.sender_id = my_id;
  mine.session_id = my_session;
  mine.sequence = counter.next();

  std::vector<std::uint8_t> out;
  out.reserve(packet.size());
  appendHeader(out, mine);
  out.insert(out.end(), packet.begin() + kHeaderSize, packet.end());
  return out;
}

void SessionFilter::start(Sender & s, const Header & header)
{
  s.session = header.session_id;
  s.highest = header.sequence;
  s.seen.clear();
  s.seen.insert(header.sequence);
}

bool SessionFilter::accept(const Header & header)
{
  const auto found = senders_.find(header.sender_id);
  if (found == senders_.end()) {
    start(senders_[header.sender_id], header);
    return true;
  }
  Sender & s = found->second;
  if (header.session_id != s.session) {
    if (std::find(s.retired.begin(), s.retired.end(), header.session_id) != s.retired.end()) {
      return false;   // 重開機前的封包，在網路上延遲到現在
    }
    s.retired.push_back(s.session);
    if (s.retired.size() > kRetiredMemory) {
      s.retired.pop_front();
    }
    s.restarted = true;
    start(s, header);
    return true;
  }
  // 循環比較：差值當成有號 32 位元
  const auto ahead = static_cast<std::int32_t>(header.sequence - s.highest);
  if (ahead > 0) {
    s.highest = header.sequence;
    s.seen.insert(header.sequence);
    // 忘記掉出視窗的（比較時也用循環差值，繞回 0 時才不會刪錯）
    for (auto it = s.seen.begin(); it != s.seen.end(); ) {
      it = s.highest - *it >= window_ ? s.seen.erase(it) : std::next(it);
    }
    return true;
  }
  if (s.highest - header.sequence >= window_) {
    return false;   // 比視窗還舊
  }
  return s.seen.insert(header.sequence).second;   // 視窗內：沒收過才接受
}

bool SessionFilter::restarted(std::uint16_t sender) const
{
  const auto it = senders_.find(sender);
  return it != senders_.end() && it->second.restarted;
}

bool RelayDeduper::firstSeen(PacketType type, const RelayHeader & relay, double now)
{
  if (now - last_cleanup_ >= memory_ / 4.0) {
    forget(now);
  }
  const Key key{static_cast<std::uint8_t>(type), relay.origin_id, relay.origin_session,
    relay.origin_seq};
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

}  // namespace cbba_core::wire
