#include "cbba_core/wire_completion.hpp"

#include <stdexcept>

#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
bool validIds(const std::vector<std::uint16_t> & ids)
{
  if (ids.size() > 255) {
    return false;
  }
  for (std::uint16_t id : ids) {
    if (!validAgentId(id)) {
      return false;
    }
  }
  return true;
}

bool validCommon(std::uint32_t task_id, std::uint16_t executor, const RelayHeader & relay,
  const std::vector<std::uint16_t> & view)
{
  return (task_id >> 16) != 0 && validAgentId(executor) && validAgentId(relay.origin_id) &&
         validIds(view);
}

bool validAcks(const std::vector<AckRef> & acks)
{
  if (acks.size() > 255) {
    return false;
  }
  for (const AckRef & a : acks) {
    if (!validAgentId(a.agent_id)) {
      return false;
    }
  }
  return true;
}

void writeIds(ByteWriter & w, const std::vector<std::uint16_t> & ids)
{
  w.u8(static_cast<std::uint8_t>(ids.size()));
  for (std::uint16_t id : ids) {
    w.u16(id);
  }
}

std::vector<std::uint16_t> readIds(ByteReader & r)
{
  std::vector<std::uint16_t> ids(r.u8());
  for (auto & id : ids) {
    id = r.u16();
  }
  return ids;
}

std::vector<std::uint8_t> begin(Header header, PacketType type, const RelayHeader & relay)
{
  header.type = type;
  std::vector<std::uint8_t> out;
  appendHeader(out, header);
  appendRelay(out, relay);
  return out;
}

// 解碼的共同部分：表頭種類正確、轉送區塊合法，回傳指向內容開頭的 reader
std::optional<std::pair<RelayHeader, ByteReader>> open(
  const std::uint8_t * data, std::size_t size, PacketType type)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != type) {
    return std::nullopt;
  }
  const auto relay = parseRelay(data, size);
  if (!relay) {
    return std::nullopt;
  }
  return std::make_pair(*relay, ByteReader(data, size, kHeaderSize + kRelaySize));
}
}  // namespace

std::vector<std::uint8_t> encodeCompletion(Header header, const CompletionBody & b)
{
  if (!validCommon(b.task_id, b.executor_id, b.relay, b.view)) {
    throw std::invalid_argument("invalid COMPLETION body");
  }
  auto out = begin(header, PacketType::COMPLETION, b.relay);
  ByteWriter w(out);
  w.u32(b.task_id);
  w.u16(b.executor_id);
  w.u32(b.round_key);
  w.u64(b.assign_version);
  writeIds(w, b.view);
  finishPacket(out);
  return out;
}

std::vector<std::uint8_t> encodeCompletionAck(Header header, const CompletionAckBody & b)
{
  if (!validCommon(b.task_id, b.executor_id, b.relay, b.view) || b.status > kAckStaleAssignment) {
    throw std::invalid_argument("invalid COMPLETION_ACK body");
  }
  auto out = begin(header, PacketType::COMPLETION_ACK, b.relay);
  ByteWriter w(out);
  w.u32(b.task_id);
  w.u16(b.executor_id);
  w.u32(b.round_key);
  w.u64(b.assign_version);
  w.u8(b.status);
  writeIds(w, b.view);
  finishPacket(out);
  return out;
}

std::vector<std::uint8_t> encodeTaskClose(Header header, const TaskCloseBody & b)
{
  if (!validCommon(b.task_id, b.actor_id, b.relay, b.view) || !validAcks(b.acks) ||
    (b.reason != kCloseDone && b.reason != kCloseCancelled))
  {
    throw std::invalid_argument("invalid TASK_CLOSE body");
  }
  auto out = begin(header, PacketType::TASK_CLOSE, b.relay);
  ByteWriter w(out);
  w.u32(b.task_id);
  w.u8(b.reason);
  w.u16(b.actor_id);
  w.u32(b.round_key);
  w.u64(b.assign_version);
  writeIds(w, b.view);
  w.u8(static_cast<std::uint8_t>(b.acks.size()));
  for (const AckRef & a : b.acks) {
    w.u16(a.agent_id);
    w.u32(a.ack_seq);
  }
  finishPacket(out);
  return out;
}

std::optional<CompletionBody> decodeCompletion(const std::uint8_t * data, std::size_t size)
{
  auto opened = open(data, size, PacketType::COMPLETION);
  if (!opened) {
    return std::nullopt;
  }
  ByteReader & r = opened->second;
  CompletionBody b;
  b.relay = opened->first;
  b.task_id = r.u32();
  b.executor_id = r.u16();
  b.round_key = r.u32();
  b.assign_version = r.u64();
  b.view = readIds(r);
  if (!r.ok() || r.remaining() != 0 || !validCommon(b.task_id, b.executor_id, b.relay, b.view)) {
    return std::nullopt;
  }
  return b;
}

std::optional<CompletionAckBody> decodeCompletionAck(const std::uint8_t * data, std::size_t size)
{
  auto opened = open(data, size, PacketType::COMPLETION_ACK);
  if (!opened) {
    return std::nullopt;
  }
  ByteReader & r = opened->second;
  CompletionAckBody b;
  b.relay = opened->first;
  b.task_id = r.u32();
  b.executor_id = r.u16();
  b.round_key = r.u32();
  b.assign_version = r.u64();
  b.status = r.u8();
  b.view = readIds(r);
  if (!r.ok() || r.remaining() != 0 || b.status > kAckStaleAssignment ||
    !validCommon(b.task_id, b.executor_id, b.relay, b.view))
  {
    return std::nullopt;
  }
  return b;
}

std::optional<TaskCloseBody> decodeTaskClose(const std::uint8_t * data, std::size_t size)
{
  auto opened = open(data, size, PacketType::TASK_CLOSE);
  if (!opened) {
    return std::nullopt;
  }
  ByteReader & r = opened->second;
  TaskCloseBody b;
  b.relay = opened->first;
  b.task_id = r.u32();
  b.reason = r.u8();
  b.actor_id = r.u16();
  b.round_key = r.u32();
  b.assign_version = r.u64();
  b.view = readIds(r);
  b.acks.resize(r.u8());
  for (AckRef & a : b.acks) {
    a.agent_id = r.u16();
    a.ack_seq = r.u32();
  }
  if (!r.ok() || r.remaining() != 0 || (b.reason != kCloseDone && b.reason != kCloseCancelled) ||
    !validCommon(b.task_id, b.actor_id, b.relay, b.view) || !validAcks(b.acks))
  {
    return std::nullopt;
  }
  return b;
}

}  // namespace cbba_core::wire
