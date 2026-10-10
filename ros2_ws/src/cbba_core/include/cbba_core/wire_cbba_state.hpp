// CBBA_STATE（種類 2）：共識資料，可分批（協定版本 2，見 wire_header.hpp）
//
//   表頭 20 | round_key 4 | exchange_step 2 | flags 1 | stable_steps 1 |
//   snapshot_id 2 | part_index 1 | part_count 1 |
//   stamp_count 2 | (agent_id 2 + stamp_ms 4) × N |
//   record_count 2 | (task_id 4 + winner_id 2 + winning_bid 4 + assign_version 8 + assignee_id 2 +
//                     assign_state 1) × R
//   大小 = 20 + 16 + 6N + 21R
//
//   round_key       已知任務與狀態（task_id、是否已結束）的 CRC-32 摘要；不同就互相補發
//   exchange_step   本機分配表變動的次數
//   flags           bit 0 參與出價（不參與時 N = R = 0）
//   stable_steps    分配連續幾個週期沒變（0～255，收斂偵測）
//   snapshot_id     同一次送出的各批共用；每次送出加 1
//   part_index      第幾批（0 起），part_count 共幾批
//   stamp           CBBA 的時間戳 s：機號 → 對它的資訊時間（系統時鐘 ms 的低 32 位元）。每一批都帶完整的 s
//   record          進行中、而且自己認得的任務：得標者（0 = 無人）、得標價，
//                   以及自己知道的最新「正式指派」：版本、執行者（0 = 撤銷或還沒指派過）、
//                   狀態（0 無、1 待接受、2 保留、3 執行中；保留和執行中的任務別台不出價）。
//                   不認得或已結束的任務不放（版本 1 用 y = −1 表示，版本 2 直接省略）
//
// 正式指派（2026-10-09）：請 BT 接受時產生新版本（已知的最新版本 + 1，狀態「待接受」），
// 接受後同一個版本變成「保留」或「執行中」；撤銷、拒絕、逾時時產生新版本（執行者 0、狀態「無」）。
// 各節點取（版本, 執行者, 狀態）較大的那筆，經由 CBBA_STATE 傳遍全隊，多跳、晚加入的節點也拿得到；
// 完成、失敗的回報以它判斷是不是舊的。
//
// 分批：一批（整個 UDP 封包）不超過 max_packet（預設 1200 B）。每一批都帶完整的 s，所以接收端
// 每收到一批就能直接交給 CBBA 規則（規則是逐任務判斷的，只看那個任務的 z、y 和 s）；
// 收斂判定、補發要比對「對方知道哪些任務」，只用收齊的那一次。缺批時等下一次（每 200 ms 一次），
// 不會把缺少的紀錄當成任務被刪掉。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kCbbaStateBaseSize = kHeaderSize + 16;   // N = R = 0
constexpr std::size_t kStampSize = 6;
constexpr std::size_t kRecordSize = 21;
constexpr std::size_t kDefaultMaxPacket = 1200;

struct Stamp
{
  std::uint16_t agent_id{0};
  std::uint32_t stamp_ms{0};
};

struct Record
{
  std::uint32_t task_id{0};
  std::uint16_t winner_id{0};
  float winning_bid{0.0f};
  std::uint64_t assign_version{0};
  std::uint16_t assignee_id{0};
  std::uint8_t assign_state{0};   // 0 無、1 待接受、2 保留、3 執行中
};

struct CbbaStateBody
{
  std::uint32_t round_key{0};
  std::uint16_t exchange_step{0};
  bool participating{false};
  std::uint8_t stable_steps{0};
  std::uint16_t snapshot_id{0};
  std::uint8_t part_index{0};
  std::uint8_t part_count{1};
  std::vector<Stamp> stamps;
  std::vector<Record> records;
};

constexpr std::size_t cbbaStateSize(std::size_t stamps, std::size_t records)
{
  return kCbbaStateBaseSize + kStampSize * stamps + kRecordSize * records;
}

// 把完整的一次分成幾批，每批（整個封包）不超過 max_packet。每批的 s 都完整；
// 沒有紀錄時也有一批。part_index、part_count 會填好，其他欄位照抄。
// s 本身就放不下，或超過 255 批時丟出 std::invalid_argument。
std::vector<CbbaStateBody> splitCbbaState(const CbbaStateBody & full,
  std::size_t max_packet = kDefaultMaxPacket);

// 編碼成完整封包；header.type 會設成 CBBA_STATE。欄位不合法時丟出 std::invalid_argument。
std::vector<std::uint8_t> encodeCbbaState(Header header, const CbbaStateBody & body);

// 解碼完整封包。表頭不合法、不是 CBBA_STATE、長度和筆數對不上、欄位不合法時回傳 nullopt：
// part_index ≥ part_count、機號不合法、task_id 的建立者是 0、得標價不是有限的非負數、執行者是 0xFFFF、
// 指派狀態不是 0～3，或狀態不是「無」卻沒有執行者。
std::optional<CbbaStateBody> decodeCbbaState(const std::uint8_t * data, std::size_t size);
inline std::optional<CbbaStateBody> decodeCbbaState(const std::vector<std::uint8_t> & bytes)
{
  return decodeCbbaState(bytes.data(), bytes.size());
}

}  // namespace cbba_core::wire
