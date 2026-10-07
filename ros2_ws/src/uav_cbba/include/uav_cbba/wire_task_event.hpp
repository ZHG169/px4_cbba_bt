// TASK_EVENT（代碼 4）的編解碼，對應「五機蜂群：機間通訊封包規格」2026-10-06 §2.6
//
// 版本 1（照規格）：14 + 27 + 名稱長度 = 42~49 B
//   表頭 14 | 轉送表頭 9 | task_index 1 | x、y、z 12 | value 4 | id_len 1 | task_id 1~8
//
//   task_index   任務編號 0~254（也是 AGENT_STATE 裡 z、y 陣列的位置）
//   x、y、z      任務位置（m）
//   value        任務價值
//   task_id      任務名稱，1~8 個 ASCII 字元
//
// 版本 2（我們的擴充）：版本 1 之後再加 7 B，共 49~56 B
//   task_type 1 | deadline_ms 4 | duration_sec 2
//
//   task_type     1 空中偵察、2 地面處置、3 巡邏（和 types.hpp 的 TaskType 相同）
//   deadline_ms   絕對截止時刻：系統時鐘的 ms 取低 32 位元，約 49.7 天循環；0 = 沒有期限資訊
//   duration_sec  預估的現場執行時間（秒）
//
// 和規格不同、以我們的情況為主的地方：
//   1. 出價公式需要任務類型、截止時刻、執行時間，規格沒有，所以加版本 2。
//      截止時刻用絕對時刻而不是「從現在起幾秒」：轉送或補發時內容原封不動，期限不會跟著往後移。
//      版本 1 仍可解碼（ext 為空）。
//   2. task_index = 255 保留為「無」，不接受。
//   3. 名稱只接受可列印的 ASCII（0x21~0x7E，不含空白）；位置與價值必須是有限的數值。
//   4. 規格沒定義座標系。我們的 x、y、z 一律是 map 座標系（ENU，公尺）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "uav_cbba/wire_header.hpp"

namespace uav_cbba::wire
{

constexpr std::size_t kTaskEventBaseSize = 41;   // 不含名稱
constexpr std::size_t kTaskExtensionSize = 7;
constexpr std::size_t kMaxTaskName = 8;
constexpr std::uint8_t kMaxTaskIndex = 254;

// 版本 2 的擴充欄位
struct TaskExtension
{
  std::uint8_t task_type{1};
  std::uint32_t deadline_ms{0};
  std::uint16_t duration_sec{0};
};

struct TaskEventBody
{
  RelayHeader relay;
  std::uint8_t task_index{0};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float value{0.0f};
  std::string name;
  std::optional<TaskExtension> ext;   // 有值時編成版本 2
};

constexpr std::size_t taskEventSize(std::size_t name_length, bool with_extension)
{
  return kTaskEventBaseSize + name_length + (with_extension ? kTaskExtensionSize : 0);
}

// 名稱是否合法：1~8 個可列印 ASCII 字元
bool validTaskName(const std::string & name);

// 編碼成完整封包；header.type 設成 TASK_EVENT，版本依 body.ext 設成 1 或 2。
// 名稱不合法、task_index 為 255、origin_id 為 0 時丟出 std::invalid_argument。
std::vector<std::uint8_t> encodeTaskEvent(Header header, const TaskEventBody & body);

// 解碼完整封包。表頭或轉送表頭不合法、不是 TASK_EVENT、長度不符、名稱或數值不合法時回傳 nullopt。
std::optional<TaskEventBody> decodeTaskEvent(const std::uint8_t * data, std::size_t size);
inline std::optional<TaskEventBody> decodeTaskEvent(const std::vector<std::uint8_t> & bytes)
{
  return decodeTaskEvent(bytes.data(), bytes.size());
}

// 截止時刻 ↔ deadline_ms。toDeadlineMs 不會回傳 0（0 保留為「沒有期限資訊」）。
std::uint32_t toDeadlineMs(double deadline_seconds);
// 還原成離 now 最近的那個時刻（可以在過去或未來，範圍 ±24.8 天）
double fromDeadlineMs(std::uint32_t deadline_ms, double now);

}  // namespace uav_cbba::wire
