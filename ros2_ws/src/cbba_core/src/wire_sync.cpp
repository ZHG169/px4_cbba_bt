#include "cbba_core/wire_sync.hpp"

#include <stdexcept>

#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
// 流水號範圍：世代 0（舊型 ID）16 位元，其他 32 位元
bool seqRangeOk(std::uint16_t epoch, std::uint32_t first, std::size_t count)
{
  const std::uint64_t limit = epoch == 0 ? 0x10000ull : 0x100000000ull;
  return static_cast<std::uint64_t>(first) + count <= limit;
}

bool validItemType(std::uint8_t type)
{
  return type == static_cast<std::uint8_t>(PacketType::TASK_ANNOUNCE) ||
         type == static_cast<std::uint8_t>(PacketType::TASK_CLOSE);
}
}  // namespace

std::vector<std::uint8_t> encodeGapRequest(Header header, const GapRequestBody & b)
{
  const std::size_t count = b.states.size();
  if (!validAgentId(b.target_id) || !validAgentId(b.creator) || count == 0 || count > kMaxGapSeqs ||
    !seqRangeOk(b.epoch, b.first_seq, count))
  {
    throw std::invalid_argument("invalid GAP_REQUEST body");
  }
  header.type = PacketType::GAP_REQUEST;
  std::vector<std::uint8_t> out;
  out.reserve(gapRequestSize(count));
  appendHeader(out, header);
  ByteWriter w(out);
  w.u16(b.target_id);
  w.u16(b.creator);
  w.u16(b.epoch);
  w.u32(b.first_seq);
  w.u16(static_cast<std::uint16_t>(count));
  for (std::size_t i = 0; i < count; i += 4) {
    std::uint8_t byte = 0;
    for (std::size_t k = 0; k < 4; ++k) {
      const auto v = i + k < count ? static_cast<std::uint8_t>(b.states[i + k]) : 0u;
      if (v > static_cast<std::uint8_t>(SeqState::CLOSED)) {
        throw std::invalid_argument("invalid GAP_REQUEST state");
      }
      byte = static_cast<std::uint8_t>(byte | (v << (6 - 2 * k)));
    }
    w.u8(byte);
  }
  finishPacket(out);
  return out;
}

std::optional<GapRequestBody> decodeGapRequest(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::GAP_REQUEST || size < kGapRequestBaseSize) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize);
  GapRequestBody b;
  b.target_id = r.u16();
  b.creator = r.u16();
  b.epoch = r.u16();
  b.first_seq = r.u32();
  const std::size_t count = r.u16();
  if (!r.ok() || count == 0 || count > kMaxGapSeqs || size != gapRequestSize(count) ||
    !validAgentId(b.target_id) || !validAgentId(b.creator) || !seqRangeOk(b.epoch, b.first_seq, count))
  {
    return std::nullopt;
  }
  b.states.reserve(count);
  for (std::size_t i = 0; i < count; i += 4) {
    const std::uint8_t byte = r.u8();
    for (std::size_t k = 0; k < 4; ++k) {
      const auto v = static_cast<std::uint8_t>((byte >> (6 - 2 * k)) & 0x3u);
      if (i + k >= count) {
        if (v != 0) {
          return std::nullopt;   // 沒用到的位元要是 0
        }
        continue;
      }
      if (v > static_cast<std::uint8_t>(SeqState::CLOSED)) {
        return std::nullopt;
      }
      b.states.push_back(static_cast<SeqState>(v));
    }
  }
  if (!r.ok()) {
    return std::nullopt;
  }
  return b;
}

std::optional<SyncItem> toSyncItem(const std::vector<std::uint8_t> & packet)
{
  const auto header = parseHeader(packet);
  if (!header || !validItemType(static_cast<std::uint8_t>(header->type))) {
    return std::nullopt;
  }
  SyncItem item;
  item.type = header->type;
  item.payload.assign(packet.begin() + static_cast<std::ptrdiff_t>(kHeaderSize), packet.end());
  return item;
}

std::vector<std::uint8_t> syncItemPacket(const SyncItem & item, Header header)
{
  header.type = item.type;
  std::vector<std::uint8_t> out;
  out.reserve(kHeaderSize + item.payload.size());
  appendHeader(out, header);
  out.insert(out.end(), item.payload.begin(), item.payload.end());
  finishPacket(out);
  return out;
}

std::vector<std::uint8_t> encodeSyncBatch(Header header, const std::vector<SyncItem> & items)
{
  if (items.empty() || items.size() > 0xFF) {
    throw std::invalid_argument("SYNC_BATCH needs 1~255 items");
  }
  header.type = PacketType::SYNC_BATCH;
  std::vector<std::uint8_t> out;
  appendHeader(out, header);
  ByteWriter w(out);
  w.u8(static_cast<std::uint8_t>(items.size()));
  for (const SyncItem & item : items) {
    if (!validItemType(static_cast<std::uint8_t>(item.type)) || item.payload.size() > 0xFFFF) {
      throw std::invalid_argument("invalid SYNC_BATCH item");
    }
    w.u8(static_cast<std::uint8_t>(item.type));
    w.u16(static_cast<std::uint16_t>(item.payload.size()));
    out.insert(out.end(), item.payload.begin(), item.payload.end());
  }
  finishPacket(out);
  return out;
}

std::optional<std::vector<SyncItem>> decodeSyncBatch(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::SYNC_BATCH || size < kSyncBatchBaseSize) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize);
  const std::size_t count = r.u8();
  if (count == 0) {
    return std::nullopt;
  }
  std::vector<SyncItem> items(count);
  std::size_t pos = kSyncBatchBaseSize;
  for (SyncItem & item : items) {
    const std::uint8_t type = r.u8();
    const std::size_t length = r.u16();
    pos += kSyncItemOverhead;
    if (!r.ok() || !validItemType(type) || r.remaining() < length) {
      return std::nullopt;
    }
    item.type = static_cast<PacketType>(type);
    item.payload.assign(data + pos, data + pos + length);
    for (std::size_t k = 0; k < length; ++k) {
      r.u8();   // 跳過內容
    }
    pos += length;
  }
  if (!r.ok() || r.remaining() != 0) {
    return std::nullopt;
  }
  return items;
}

}  // namespace cbba_core::wire
