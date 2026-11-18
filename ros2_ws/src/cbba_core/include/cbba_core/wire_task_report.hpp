// TASK_REPORT（種類 14）、TASK_REPORT_ACK（種類 15）：任務的執行結果送閘道（協定版本 5，任務欄位補齊 11「回報」，2026-11-18）。
// 兩種都轉送（多跳）。只放結果的摘要與參照（findings、附件是 URI），影像本身不上 mesh。
//
// TASK_REPORT：執行者 → 閘道（BT 回報 task_result 時，成功、失敗都送；閘道斷線時暫存，恢復後送出）
//   表頭 20 | 轉送區塊 14 | report_seq 4 | task_id 8 | revision 2 | assign_version 8 | t_ms 8 | flags 1 |
//   completed_steps 2 | x 4 | y 4 | z 4 | failure_code_len 1 | detail_len 1 | findings_uri_len 2 | attachment_count 1 |
//   failure_code | detail | findings_uri | (len 2 | attachment) × attachment_count
//   大小 = 84 + 字串（全部 ≤ 1200 B）
//
//   report_seq     origin 這個 session 的結果流水號（1 起）；閘道去重複的鍵是 (origin_id, origin_session, report_seq)
//   flags          bit 0 成功、bit 1 最後位置有效、bit 2 暫存過（閘道斷線時產生）
//   completed_steps  做完幾步（steps 的前幾個）
//   x、y、z        最後位置（map ENU）
//   failure_code   E_*（≤ 32 B）；detail ≤ 200 B；findings_uri、每個附件 ≤ 512 B；附件最多 8 個
//
// TASK_REPORT_ACK：閘道 → 執行者（重複的也回）
//   表頭 20 | 轉送區塊 14 | target_id 2 | target_session 8 | count 1 | report_seq 4 × count
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kTaskReportBaseSize = kHeaderSize + kRelaySize + 50;
constexpr std::size_t kMaxReportCode = 32;
constexpr std::size_t kMaxReportDetail = 200;
constexpr std::size_t kMaxReportUri = 512;
constexpr std::size_t kMaxReportAttachments = 8;
constexpr std::size_t kMaxReportPacket = 1200;
constexpr std::uint8_t kReportSuccess = 0x01;
constexpr std::uint8_t kReportPositionValid = 0x02;
constexpr std::uint8_t kReportBuffered = 0x04;

constexpr std::size_t kTaskReportAckBaseSize = kHeaderSize + kRelaySize + 11;
constexpr std::size_t kMaxReportAckSeqs = 64;

struct TaskReportBody
{
  RelayHeader relay;
  std::uint32_t report_seq{0};
  std::uint64_t task_id{0};
  std::uint16_t revision{0};
  std::uint64_t assign_version{0};
  std::uint64_t t_ms{0};
  std::uint8_t flags{0};
  std::uint16_t completed_steps{0};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::string failure_code;
  std::string detail;
  std::string findings_uri;
  std::vector<std::string> attachments;
};

struct TaskReportAckBody
{
  RelayHeader relay;
  std::uint16_t target_id{0};
  std::uint64_t target_session{0};
  std::vector<std::uint32_t> seqs;
};

// 編碼後的大小（不檢查合法）
std::size_t taskReportSize(const TaskReportBody & body);

// 編碼成完整封包。欄位不合法（origin_id、report_seq 0、task_id 不合法、字串太長、附件太多、總長超過 1200 B）時丟出
// std::invalid_argument
std::vector<std::uint8_t> encodeTaskReport(Header header, const TaskReportBody & body);
std::vector<std::uint8_t> encodeTaskReportAck(Header header, const TaskReportAckBody & body);

std::optional<TaskReportBody> decodeTaskReport(const std::uint8_t * data, std::size_t size);
inline std::optional<TaskReportBody> decodeTaskReport(const std::vector<std::uint8_t> & bytes)
{
  return decodeTaskReport(bytes.data(), bytes.size());
}
std::optional<TaskReportAckBody> decodeTaskReportAck(const std::uint8_t * data, std::size_t size);
inline std::optional<TaskReportAckBody> decodeTaskReportAck(const std::vector<std::uint8_t> & bytes)
{
  return decodeTaskReportAck(bytes.data(), bytes.size());
}

}  // namespace cbba_core::wire
