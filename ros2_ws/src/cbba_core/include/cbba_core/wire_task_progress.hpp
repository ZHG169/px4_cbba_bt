// TASK_PROGRESS（種類 12）：角色的執行進度（協定版本 4，CB-4 第一批，2026-11-16；doc/cb4_design.md）。轉送。
//
//   表頭 20 | 轉送區塊 14 | task_id 8 | revision 2 | incident_revision 2 | assign_version 8 | executor 2 | progress_seq 2 |
//   phase 1 | step_index 1 | milestones 2 | flags 1
//   = 63 B（2026-11-16 第三批加 incident_revision，原本 61 B）
//
//   executor        回報的執行者；執行者自己送的 origin = executor。補發（SYNC_BATCH 的快照）沿用原本的轉送區塊
//   revision        回報時任務的 revision（task_revision）
//   incident_revision  回報時這個組的 incident revision（組層級的 incident state，INCIDENT_UPDATE 的 Accepted）；
//                   事件結果型里程碑只在同一個 incident revision 算數。沒有角色組、還沒有 incident state 的是 0
//   assign_version  正式指派版本；換人（新的版本）時執行者相關的里程碑重算
//   progress_seq    同一個 (task_id, assign_version) 內遞增（從 1 起）；收的一方只留最大的
//   phase           cbba_core::ProgressPhase（0～7）
//   step_index      正在做 spec 的第幾步（0 起）；255 = 不知道
//   milestones      已達成的里程碑（cbba_core::Milestone 的位元，只增不減）
//   flags           保留，送 0、收到不看
//
// 送出（cbba_comm）：狀態變化時馬上送一份、之後錯開再送兩份（掉包）；執行中每 progress_period 重送最新的快照。
// 每次都換新序號（協定「重送一律換新序號」）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cbba_core/types.hpp"
#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kTaskProgressSize = kHeaderSize + kRelaySize + 29;
constexpr std::uint8_t kUnknownStep = 255;

struct TaskProgressBody
{
  RelayHeader relay;
  TaskId task_id{0};
  std::uint16_t revision{0};
  std::uint16_t incident_revision{0};
  std::uint64_t assign_version{0};
  std::uint16_t executor{0};
  std::uint16_t progress_seq{0};
  std::uint8_t phase{0};
  std::uint8_t step_index{kUnknownStep};
  std::uint16_t milestones{0};
  std::uint8_t flags{0};
};

// 編碼成完整封包；header.type 會設成 TASK_PROGRESS。origin_id、executor 不合法、task_id 0、
// assign_version 0、progress_seq 0、phase 超出範圍時丟出 std::invalid_argument
std::vector<std::uint8_t> encodeTaskProgress(Header header, const TaskProgressBody & body);

// 解碼完整封包。表頭或轉送區塊不合法、種類不對、長度不符、欄位不合法時回傳 nullopt
std::optional<TaskProgressBody> decodeTaskProgress(const std::uint8_t * data, std::size_t size);
inline std::optional<TaskProgressBody> decodeTaskProgress(const std::vector<std::uint8_t> & bytes)
{
  return decodeTaskProgress(bytes.data(), bytes.size());
}

}  // namespace cbba_core::wire
