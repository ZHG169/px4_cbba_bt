#include "uav_cbba/wire_completion.hpp"

#include <stdexcept>

#include "uav_cbba/wire_io.hpp"

namespace uav_cbba::wire
{

namespace
{
constexpr std::uint8_t kNoTaskIndex = 0xFF;

void check(bool ok, const char * what)
{
  if (!ok) {
    throw std::invalid_argument(what);
  }
}

// 兩種封包共同的開頭：檢查表頭種類並取出轉送表頭
bool parsePrefix(
  const std::uint8_t * data, std::size_t size, PacketType type, RelayHeader & relay)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != type) {
    return false;
  }
  const auto r = parseRelayHeader(data, size);
  if (!r) {
    return false;
  }
  relay = *r;
  return true;
}
}  // namespace

std::vector<std::uint8_t> encodeCompletion(Header header, const CompletionBody & body)
{
  check(body.relay.origin_id != 0, "COMPLETION origin_id must not be 0");
  check(body.kind == kCompletionAnnounce || body.kind == kCompletionProof,
    "COMPLETION kind must be 0 (announce) or 1 (proof)");
  check(body.task_index != kNoTaskIndex, "COMPLETION task_index must be 0~254");
  check(body.executor_id != 0, "COMPLETION executor_id must not be 0");
  check(body.kind == kCompletionProof || body.acks.empty(), "COMPLETION announce has no acks");
  check(body.acks.size() <= 255, "COMPLETION has more than 255 acks");
  for (const AckRef & a : body.acks) {
    check(a.agent_id != 0, "COMPLETION ack agent_id must not be 0");
  }

  header.type = PacketType::COMPLETION;
  header.version = kVersion;
  std::vector<std::uint8_t> out;
  out.reserve(body.kind == kCompletionProof ?
    completionProofSize(body.acks.size()) : kCompletionAnnounceSize);
  appendHeader(out, header);
  appendRelayHeader(out, body.relay);

  ByteWriter w(out);
  w.u8(body.kind);
  w.u8(body.task_index);
  w.u8(body.executor_id);
  w.u32(body.round_key);
  w.u64(body.control_revision);
  w.u8(body.view_bits);
  if (body.kind == kCompletionProof) {
    w.u8(static_cast<std::uint8_t>(body.acks.size()));
    for (const AckRef & a : body.acks) {
      w.u8(a.agent_id);
      w.u64(a.ack_seq);
    }
  }
  return out;
}

std::optional<CompletionBody> decodeCompletion(const std::uint8_t * data, std::size_t size)
{
  CompletionBody b;
  if (!parsePrefix(data, size, PacketType::COMPLETION, b.relay)) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelayHeaderSize);
  b.kind = r.u8();
  b.task_index = r.u8();
  b.executor_id = r.u8();
  b.round_key = r.u32();
  b.control_revision = r.u64();
  b.view_bits = r.u8();
  if (!r.ok() || b.task_index == kNoTaskIndex || b.executor_id == 0) {
    return std::nullopt;
  }

  if (b.kind == kCompletionAnnounce) {
    return size == kCompletionAnnounceSize ? std::optional<CompletionBody>(b) : std::nullopt;
  }
  if (b.kind != kCompletionProof) {
    return std::nullopt;
  }
  const std::size_t k = r.u8();
  if (!r.ok() || size != completionProofSize(k)) {
    return std::nullopt;   // 規格不補位：長度必須剛好
  }
  b.acks.resize(k);
  for (AckRef & a : b.acks) {
    a.agent_id = r.u8();
    a.ack_seq = r.u64();
    if (a.agent_id == 0) {
      return std::nullopt;
    }
  }
  return b;
}

std::vector<std::uint8_t> encodeCompletionAck(Header header, const CompletionAckBody & body)
{
  check(body.relay.origin_id != 0, "COMPLETION_ACK origin_id must not be 0");
  check(body.task_index != kNoTaskIndex, "COMPLETION_ACK task_index must be 0~254");
  check(body.executor_id != 0, "COMPLETION_ACK executor_id must not be 0");

  header.type = PacketType::COMPLETION_ACK;
  header.version = kVersion;
  std::vector<std::uint8_t> out;
  out.reserve(kCompletionAckSize);
  appendHeader(out, header);
  appendRelayHeader(out, body.relay);

  ByteWriter w(out);
  w.u8(body.task_index);
  w.u8(body.executor_id);
  w.u32(body.round_key);
  w.u64(body.control_revision);
  w.u8(body.view_bits);
  w.u8(body.accepted ? 1 : 0);
  return out;
}

std::optional<CompletionAckBody> decodeCompletionAck(const std::uint8_t * data, std::size_t size)
{
  CompletionAckBody b;
  if (!parsePrefix(data, size, PacketType::COMPLETION_ACK, b.relay) || size != kCompletionAckSize) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelayHeaderSize);
  b.task_index = r.u8();
  b.executor_id = r.u8();
  b.round_key = r.u32();
  b.control_revision = r.u64();
  b.view_bits = r.u8();
  const std::uint8_t accepted = r.u8();
  if (!r.ok() || b.task_index == kNoTaskIndex || b.executor_id == 0 || accepted > 1) {
    return std::nullopt;
  }
  b.accepted = accepted == 1;
  return b;
}

}  // namespace uav_cbba::wire
