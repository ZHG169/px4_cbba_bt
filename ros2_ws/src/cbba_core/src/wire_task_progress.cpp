#include "cbba_core/wire_task_progress.hpp"

#include <stdexcept>

#include "cbba_core/milestones.hpp"
#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
bool validBody(const TaskProgressBody & b)
{
  return validAgentId(b.relay.origin_id) && validAgentId(b.executor) && b.task_id != 0 &&
         b.assign_version != 0 && b.progress_seq != 0 && b.phase <= kMaxPhase;
}
}  // namespace

std::vector<std::uint8_t> encodeTaskProgress(Header header, const TaskProgressBody & b)
{
  if (!validBody(b)) {
    throw std::invalid_argument("invalid TASK_PROGRESS body");
  }
  header.type = PacketType::TASK_PROGRESS;
  std::vector<std::uint8_t> out;
  out.reserve(kTaskProgressSize);
  appendHeader(out, header);
  appendRelay(out, b.relay);
  ByteWriter w(out);
  w.u64(b.task_id);
  w.u16(b.revision);
  w.u64(b.assign_version);
  w.u16(b.executor);
  w.u16(b.progress_seq);
  w.u8(b.phase);
  w.u8(b.step_index);
  w.u16(b.milestones);
  w.u8(b.flags);
  finishPacket(out);
  return out;
}

std::optional<TaskProgressBody> decodeTaskProgress(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::TASK_PROGRESS || size != kTaskProgressSize) {
    return std::nullopt;
  }
  const auto relay = parseRelay(data, size);
  if (!relay) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelaySize);
  TaskProgressBody b;
  b.relay = *relay;
  b.task_id = r.u64();
  b.revision = r.u16();
  b.assign_version = r.u64();
  b.executor = r.u16();
  b.progress_seq = r.u16();
  b.phase = r.u8();
  b.step_index = r.u8();
  b.milestones = r.u16();
  b.flags = r.u8();
  if (!r.ok() || r.remaining() != 0 || !validBody(b)) {
    return std::nullopt;
  }
  return b;
}

}  // namespace cbba_core::wire
