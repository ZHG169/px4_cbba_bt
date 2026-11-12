// TASK_ANNOUNCE（種類 3）：任務定義，轉送（協定版本 4，見 wire_header.hpp；欄位見 doc/cbba_m0_contract.md 第二節 A 層）
//
//   表頭 20 | 轉送區塊 14 |
//   task_id 8 | revision 2 | task_type 1 | platform 1 | required_caps 8 | priority 1 | flags 1 |
//   x、y、z 12 | floor 1 | value 4 |
//   created_ms 4 | release_ms 4 | deadline_ms 4 | expires_ms 4 | duration_sec 2 | half_life_s 2 |
//   can_preempt 1 | preempt_margin_pct 1 | commit_radius_dm 2 | min_battery_reserve 1 |
//   group_id 8 | role_index 1 | role_count 1 | precedence 1 | sync_window_s 2 |
//   optional_no_capable_s 2 | optional_unassigned_s 2 | spec_length 4 | spec_crc 4
//   共 20 + 14 + 89 = 123 B（固定大小，逐欄序列化）
//
//   task_id       64 位元：建立者 16 ｜ 世代 16 ｜ 流水號 32；上半 32 位元是 0 的是舊型 ID（建立者 × 65536 + 流水號）
//   revision      同一個 task_id 的內容版本（建立者改內容時 + 1）
//   task_type     0 新型（平台、能力、優先級看欄位）；1 空中偵察、2 地面處置、3 巡邏（舊型，保留到 M3，由種類決定）
//   platform      0 不限、1 uav、2 quadruped；required_caps 能力遮罩（capabilities.hpp）；priority P0～P3
//   flags         bit 0 硬期限、bit 1 optional 角色、bit 2 all_or_nothing、bit 3 skip_satisfies
//   x、y、z       map 座標系（ENU，公尺）；floor 樓層（0 = 不分）
//   value         同級內的價值
//   created_ms    任務建立的時刻（系統時鐘 ms 的低 32 位元）。沒有期限的任務，排隊的最大等待從這裡算，
//                 各節點、每輪競標都一樣，不會因為晚收到或重新競標而重新起算
//   release_ms、deadline_ms、expires_ms   可以開始、截止、失效的絕對時刻（系統時鐘 ms 的低 32 位元，約 49.7 天循環）；0 = 沒有
//   duration_sec  預估的現場執行時間（秒）；half_life_s 價值衰減（0 = 不衰減）
//   can_preempt   可以搶占的優先級（位元 p = Pp，0 = 預設）；preempt_margin_pct 同級搶占要好多少；
//   commit_radius_dm  執行者離目標這麼近之後不可搶占（0.1 m）；min_battery_reserve 做完還要剩的電量（%）
//   group_id      任務組（格式同 task_id，0 = 單獨任務）；role_index／role_count（≤ 8）；precedence 要等的角色（位元 = role_index）；
//                 sync_window_s 同時到位的時間窗；optional_no_capable_s、optional_unassigned_s optional 關閉 O1、O2 的等待（0 = 預設）
//   spec_length、spec_crc  這個 revision 的 TaskSpec（TASK_SPEC 封包，壓縮後）大小與 CRC-32；0 = 沒有 TaskSpec
//
// 建立時送一次；補發（鄰居不認得）、保底重播時重送，內容原封不動、origin 沿用建立時的，
// 已收過的飛機會當成重複丟掉。時刻用絕對時刻，轉送或補發時期限不會往後推。
// 出價時才交換的得標者、得標價不在這裡（在 CBBA_STATE），不會每次出價都重送整份任務定義。
// M0：revision 的套用、過期、搶占、協同欄位只傳送、保存，行為在之後的步驟。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kTaskAnnounceSize = kHeaderSize + kRelaySize + 89;
constexpr std::uint8_t kMaxRoles = 8;   // precedence 是 8 位元的遮罩

struct TaskAnnounceBody
{
  RelayHeader relay;
  std::uint64_t task_id{0};
  std::uint16_t revision{0};
  std::uint8_t task_type{1};
  std::uint8_t platform{0};
  std::uint64_t required_caps{0};
  std::uint8_t priority{3};
  bool deadline_hard{false};
  bool optional{false};
  bool all_or_nothing{false};
  bool skip_satisfies{false};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  std::uint8_t floor{0};
  float value{0.0f};
  std::uint32_t created_ms{0};
  std::uint32_t release_ms{0};
  std::uint32_t deadline_ms{0};
  std::uint32_t expires_ms{0};
  std::uint16_t duration_sec{0};
  std::uint16_t half_life_s{0};
  std::uint8_t can_preempt{0};
  std::uint8_t preempt_margin_pct{0};
  std::uint16_t commit_radius_dm{0};
  std::uint8_t min_battery_reserve{0};
  std::uint64_t group_id{0};
  std::uint8_t role_index{0};
  std::uint8_t role_count{0};
  std::uint8_t precedence{0};
  std::uint16_t sync_window_s{0};
  std::uint16_t optional_no_capable_s{0};
  std::uint16_t optional_unassigned_s{0};
  std::uint32_t spec_length{0};
  std::uint32_t spec_crc{0};
};

// 編碼成完整封包；header.type 會設成 TASK_ANNOUNCE。欄位不合法時丟出 std::invalid_argument：
// task_id、group_id（非 0 時）不合法、task_type 超過 3、platform 超過 2、priority 超過 3、
// role_count 超過 8 或 role_index ≥ role_count（role_count > 0 時）、數值不是有限值、origin_id 不合法。
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
