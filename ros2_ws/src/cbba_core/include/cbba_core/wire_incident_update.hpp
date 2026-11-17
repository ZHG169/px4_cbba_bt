// INCIDENT_UPDATE（種類 13）：組層級的 incident state（協定版本 4，CB-4 第三批，2026-11-16；doc/cb4_design.md）。轉送。
//
//   表頭 20 | 轉送區塊 14 | kind 1 | group_id 8 | incident_revision 2 | proposer 2 | proposal_seq 4 |
//   reporter_task 8 | assign_version 8 | flags 1 | x 4 | y 4 | z 4 | floor 1 | radius 4 | track_id 4 | reason 1
//   = 20 + 14 + 56 = 90 B（固定大小）
//
//   kind  1 PROPOSAL（執行中的角色 → 閘道）：incident_revision = 提案時知道的（base）；reporter_task、assign_version 是提案者
//           目前的正式指派；flags bit 1 = 明確宣告「已定位目標」（declares_localized）
//         2 ACCEPTED（閘道 → 全隊）：incident_revision = 接受後的新 revision；proposer、proposal_seq 是回應的提案；
//           flags bit 1 = 宣告了定位而且通過驗證 → reporter_task 在新 revision 記 target_localized。開著的組定期重送、
//           發現新鄰居時用 SYNC_BATCH 補
//         3 REJECTED（閘道 → 提案者）：incident_revision = 目前的；reason 是原因
//   group_id     角色組（閘道建立的，taskCreator(group_id) 是閘道）。ACCEPTED、REJECTED 只收 origin = 那個閘道的
//   proposal_seq 提案者這次啟動的流水號（1 起）；重送同一個提案用同一個 seq（每次的封包序號、origin 照規則換新）
//   flags        bit 0 位置有效、bit 1 定位（見上）
//   x、y、z      map ENU（float32）；floor 樓層（0 = 不分）；radius 不確定半徑（m）；track_id 追蹤編號（0 = 沒有）
//   reason       0 無、1 STALE（base 不是目前的）、2 NOT_EXECUTOR（不是那個角色目前的正式執行者）、3 GROUP_CLOSED、
//                4 BAD_POSITION、5 UNKNOWN_GROUP
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cbba_core/types.hpp"
#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kIncidentUpdateSize = kHeaderSize + kRelaySize + 56;

enum class IncidentKind : std::uint8_t { PROPOSAL = 1, ACCEPTED = 2, REJECTED = 3 };
enum class IncidentReject : std::uint8_t
{
  NONE = 0, STALE = 1, NOT_EXECUTOR = 2, GROUP_CLOSED = 3, BAD_POSITION = 4, UNKNOWN_GROUP = 5,
};
const char * toString(IncidentReject r);

constexpr std::uint8_t kIncidentPositionValid = 0x01;
constexpr std::uint8_t kIncidentLocalized = 0x02;

struct IncidentUpdateBody
{
  RelayHeader relay;
  IncidentKind kind{IncidentKind::PROPOSAL};
  TaskId group_id{0};
  std::uint16_t incident_revision{0};
  std::uint16_t proposer{0};
  std::uint32_t proposal_seq{0};
  TaskId reporter_task{0};
  std::uint64_t assign_version{0};
  std::uint8_t flags{0};
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t floor{0};
  float radius{0.0F};
  std::uint32_t track_id{0};
  IncidentReject reason{IncidentReject::NONE};
};

// 編碼成完整封包；header.type 會設成 INCIDENT_UPDATE。origin、proposer 不合法、group_id 不合法、proposal_seq 0、
// 位置不是有限數時丟出 std::invalid_argument
std::vector<std::uint8_t> encodeIncidentUpdate(Header header, const IncidentUpdateBody & body);

// 解碼完整封包。表頭或轉送區塊不合法、種類不對、長度不符、欄位不合法時回傳 nullopt
std::optional<IncidentUpdateBody> decodeIncidentUpdate(const std::uint8_t * data, std::size_t size);
inline std::optional<IncidentUpdateBody> decodeIncidentUpdate(const std::vector<std::uint8_t> & bytes)
{
  return decodeIncidentUpdate(bytes.data(), bytes.size());
}

}  // namespace cbba_core::wire
