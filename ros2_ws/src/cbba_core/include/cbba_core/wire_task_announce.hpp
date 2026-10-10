// TASK_ANNOUNCE（種類 3）：任務定義，轉送（協定版本 2，見 wire_header.hpp）
//
//   表頭 20 | 轉送區塊 14 | task_id 4 | task_type 1 | x、y、z 12 | value 4 | created_ms 4 | deadline_ms 4 |
//   duration_sec 2
//   共 65 B
//
//   task_id       建立者機號 × 65536 + 流水號（高 16 位元是建立者，不能是 0）。全隊共用的任務識別
//   task_type     1 空中偵察、2 地面處置、3 巡邏（和 types.hpp 的 TaskType 相同）
//   x、y、z       map 座標系（ENU，公尺）
//   value         任務價值
//   created_ms    任務建立的時刻（系統時鐘 ms 的低 32 位元）。沒有期限的任務，排隊的最大等待從這裡算，
//                 各節點、每輪競標都一樣，不會因為晚收到或重新競標而重新起算
//   deadline_ms   絕對截止時刻：系統時鐘的 ms 取低 32 位元，約 49.7 天循環；0 = 沒有期限
//   duration_sec  預估的現場執行時間（秒）
//
// 建立時送一次；補發（鄰居不認得）、保底重播時重送，內容原封不動、origin 沿用建立時的，
// 已收過的飛機會當成重複丟掉。截止時刻用絕對時刻，轉送或補發時期限不會往後推。
// 出價時才交換的得標者、得標價不在這裡（在 CBBA_STATE），不會每次出價都重送整份任務定義。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kTaskAnnounceSize = kHeaderSize + kRelaySize + 31;

struct TaskAnnounceBody
{
  RelayHeader relay;
  std::uint32_t task_id{0};
  std::uint8_t task_type{1};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float value{0.0f};
  std::uint32_t created_ms{0};
  std::uint32_t deadline_ms{0};
  std::uint16_t duration_sec{0};
};

// 編碼成完整封包；header.type 會設成 TASK_ANNOUNCE。
// task_id 的建立者是 0、task_type 不是 1～3、數值不是有限值、origin_id 不合法時丟出 std::invalid_argument。
std::vector<std::uint8_t> encodeTaskAnnounce(Header header, const TaskAnnounceBody & body);

// 解碼完整封包。表頭或轉送區塊不合法、不是 TASK_ANNOUNCE、長度不符、欄位不合法時回傳 nullopt。
std::optional<TaskAnnounceBody> decodeTaskAnnounce(const std::uint8_t * data, std::size_t size);
inline std::optional<TaskAnnounceBody> decodeTaskAnnounce(const std::vector<std::uint8_t> & bytes)
{
  return decodeTaskAnnounce(bytes.data(), bytes.size());
}

// 截止時刻 ↔ deadline_ms。toDeadlineMs 不會回傳 0（0 保留為「沒有期限資訊」）。
std::uint32_t toDeadlineMs(double deadline_seconds);
// 還原成離 now 最近的那個時刻（可以在過去或未來，範圍 ±24.8 天）
double fromDeadlineMs(std::uint32_t deadline_ms, double now);

}  // namespace cbba_core::wire
