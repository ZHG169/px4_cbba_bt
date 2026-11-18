#include "cbba_core/wire_task_report.hpp"

#include <stdexcept>

#include "cbba_core/types.hpp"
#include "cbba_core/wire_io.hpp"

namespace cbba_core::wire
{

namespace
{
bool validStrings(const TaskReportBody & b)
{
  if (b.failure_code.size() > kMaxReportCode || b.detail.size() > kMaxReportDetail ||
    b.findings_uri.size() > kMaxReportUri || b.attachments.size() > kMaxReportAttachments)
  {
    return false;
  }
  for (const std::string & a : b.attachments) {
    if (a.size() > kMaxReportUri) {
      return false;
    }
  }
  return true;
}

bool validBody(const TaskReportBody & b)
{
  return validAgentId(b.relay.origin_id) && b.report_seq != 0 && cbba_core::validTaskId(b.task_id) && validStrings(b) &&
         taskReportSize(b) <= kMaxReportPacket;
}

bool validBody(const TaskReportAckBody & b)
{
  return validAgentId(b.relay.origin_id) && validAgentId(b.target_id) && !b.seqs.empty() &&
         b.seqs.size() <= kMaxReportAckSeqs;
}
}  // namespace

std::size_t taskReportSize(const TaskReportBody & b)
{
  std::size_t n = kTaskReportBaseSize + b.failure_code.size() + b.detail.size() + b.findings_uri.size();
  for (const std::string & a : b.attachments) {
    n += 2 + a.size();
  }
  return n;
}

std::vector<std::uint8_t> encodeTaskReport(Header header, const TaskReportBody & b)
{
  if (!validBody(b)) {
    throw std::invalid_argument("invalid TASK_REPORT body");
  }
  header.type = PacketType::TASK_REPORT;
  std::vector<std::uint8_t> out;
  out.reserve(taskReportSize(b));
  appendHeader(out, header);
  appendRelay(out, b.relay);
  ByteWriter w(out);
  w.u32(b.report_seq);
  w.u64(b.task_id);
  w.u16(b.revision);
  w.u64(b.assign_version);
  w.u64(b.t_ms);
  w.u8(b.flags);
  w.u16(b.completed_steps);
  w.f32(b.x);
  w.f32(b.y);
  w.f32(b.z);
  w.u8(static_cast<std::uint8_t>(b.failure_code.size()));
  w.u8(static_cast<std::uint8_t>(b.detail.size()));
  w.u16(static_cast<std::uint16_t>(b.findings_uri.size()));
  w.u8(static_cast<std::uint8_t>(b.attachments.size()));
  out.insert(out.end(), b.failure_code.begin(), b.failure_code.end());
  out.insert(out.end(), b.detail.begin(), b.detail.end());
  out.insert(out.end(), b.findings_uri.begin(), b.findings_uri.end());
  for (const std::string & a : b.attachments) {
    w.u16(static_cast<std::uint16_t>(a.size()));
    out.insert(out.end(), a.begin(), a.end());
  }
  finishPacket(out);
  return out;
}

std::optional<TaskReportBody> decodeTaskReport(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::TASK_REPORT || size < kTaskReportBaseSize || size > kMaxReportPacket) {
    return std::nullopt;
  }
  const auto relay = parseRelay(data, size);
  if (!relay) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelaySize);
  TaskReportBody b;
  b.relay = *relay;
  b.report_seq = r.u32();
  b.task_id = r.u64();
  b.revision = r.u16();
  b.assign_version = r.u64();
  b.t_ms = r.u64();
  b.flags = r.u8();
  b.completed_steps = r.u16();
  b.x = r.f32();
  b.y = r.f32();
  b.z = r.f32();
  const std::size_t code = r.u8();
  const std::size_t detail = r.u8();
  const std::size_t uri = r.u16();
  const std::size_t count = r.u8();
  if (!r.ok() || r.remaining() < code + detail + uri) {
    return std::nullopt;
  }
  std::size_t pos = kTaskReportBaseSize;
  const auto take = [&](std::size_t n) {
      std::string s(reinterpret_cast<const char *>(data + pos), n);
      pos += n;
      return s;
    };
  b.failure_code = take(code);
  b.detail = take(detail);
  b.findings_uri = take(uri);
  for (std::size_t i = 0; i < count; ++i) {
    if (size - pos < 2) {
      return std::nullopt;
    }
    const std::size_t n = (static_cast<std::size_t>(data[pos]) << 8) | data[pos + 1];
    pos += 2;
    if (size - pos < n) {
      return std::nullopt;
    }
    b.attachments.push_back(take(n));
  }
  if (pos != size || !validBody(b)) {
    return std::nullopt;
  }
  return b;
}

std::vector<std::uint8_t> encodeTaskReportAck(Header header, const TaskReportAckBody & b)
{
  if (!validBody(b)) {
    throw std::invalid_argument("invalid TASK_REPORT_ACK body");
  }
  header.type = PacketType::TASK_REPORT_ACK;
  std::vector<std::uint8_t> out;
  out.reserve(kTaskReportAckBaseSize + 4 * b.seqs.size());
  appendHeader(out, header);
  appendRelay(out, b.relay);
  ByteWriter w(out);
  w.u16(b.target_id);
  w.u64(b.target_session);
  w.u8(static_cast<std::uint8_t>(b.seqs.size()));
  for (std::uint32_t seq : b.seqs) {
    w.u32(seq);
  }
  finishPacket(out);
  return out;
}

std::optional<TaskReportAckBody> decodeTaskReportAck(const std::uint8_t * data, std::size_t size)
{
  const auto header = parseHeader(data, size);
  if (!header || header->type != PacketType::TASK_REPORT_ACK || size < kTaskReportAckBaseSize) {
    return std::nullopt;
  }
  const auto relay = parseRelay(data, size);
  if (!relay) {
    return std::nullopt;
  }
  ByteReader r(data, size, kHeaderSize + kRelaySize);
  TaskReportAckBody b;
  b.relay = *relay;
  b.target_id = r.u16();
  b.target_session = r.u64();
  const std::size_t count = r.u8();
  if (!r.ok() || r.remaining() != 4 * count) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < count; ++i) {
    b.seqs.push_back(r.u32());
  }
  if (!validBody(b)) {
    return std::nullopt;
  }
  return b;
}

}  // namespace cbba_core::wire
