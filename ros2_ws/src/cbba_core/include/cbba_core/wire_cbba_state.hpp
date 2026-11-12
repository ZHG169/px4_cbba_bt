// CBBA_STATE（種類 2）：共識資料，可分批（協定版本 4，見 wire_header.hpp）
//
//   表頭 20 | round_key 4 | exchange_step 2 | flags 1 | stable_steps 1 |
//   snapshot_id 2 | part_index 1 | part_count 1 |
//   digest_count 1 | (creator 2 + epoch 2 + min_seq 4 + max_seq 4 + known_count 4 + closed_count 4 + crc 4) × D |
//   stamp_count 2 | (agent_id 2 + stamp_ms 4) × N |
//   record_count 2 | (task_id 8 + revision 2 + winner_id 2 + winning_bid 4 + assign_version 8 + assignee_id 2 +
//                     assign_state 1) × R
//   大小 = 20 + 17 + 24D + 6N + 27R
//   （版本 4，2026-11-11：摘要每筆 24 B、紀錄每筆 27 B，固定大小、逐欄序列化，不用 sizeof(struct)）
//
//   round_key       已知任務與狀態（task_id、是否已結束）的 CRC-32 摘要；相同就不用比對摘要
//   exchange_step   本機分配表變動的次數
//   flags           bit 0 參與出價（不參與時 N = 0）；bit 1 有摘要（版本 3）
//   digest          壓縮的任務摘要（版本 3 起；版本 4 的鍵是（建立者, 世代），舊型 ID 是（建立者, 0））：
//                   每個（建立者, 世代）一筆，最小、最大流水號、知道幾個、已結束幾個、
//                   CRC-32（每個任務的流水號 4 B＋是否已結束 1 B，依流水號排序）。
//                   和 round_key 對應同一個狀態。比對哪個建立者不同，再用 GAP_REQUEST 問缺口（wire_sync.hpp）。
//                   不是每次都帶（flags bit 1）；有摘要而 D = 0 表示一個任務都不知道。有摘要時每一批都帶
//   stable_steps    分配連續幾個週期沒變（0～255，收斂偵測）
//   snapshot_id     同一次送出的各批共用；每次送出加 1
//   part_index      第幾批（0 起），part_count 共幾批
//   stamp           CBBA 的時間戳 s：機號 → 對它的資訊時間（系統時鐘 ms 的低 32 位元）。每一批都帶完整的 s
//   record          進行中、而且自己認得的任務：revision（送出者知道的任務定義版本；和收的一方不同時這筆不交給規則）、
//                   得標者（0 = 無人）、得標價（同級分數），
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

constexpr std::size_t kCbbaStateBaseSize = kHeaderSize + 17;   // D = N = R = 0
constexpr std::size_t kDigestSize = 24;
constexpr std::size_t kStampSize = 6;
constexpr std::size_t kRecordSize = 27;
constexpr std::size_t kDefaultMaxPacket = 1200;

// 一個（建立者, 世代）的摘要：known ≥ 1、closed ≤ known、min_seq ≤ max_seq；世代 0（舊型 ID）的流水號 ≤ 0xFFFF
struct DigestEntry
{
  std::uint16_t creator{0};
  std::uint16_t epoch{0};
  std::uint32_t min_seq{0};
  std::uint32_t max_seq{0};
  std::uint32_t known{0};
  std::uint32_t closed{0};
  std::uint32_t crc{0};
};

inline bool operator==(const DigestEntry & a, const DigestEntry & b)
{
  return a.creator == b.creator && a.epoch == b.epoch && a.min_seq == b.min_seq && a.max_seq == b.max_seq &&
         a.known == b.known && a.closed == b.closed && a.crc == b.crc;
}
inline bool operator!=(const DigestEntry & a, const DigestEntry & b) {return !(a == b);}

struct Stamp
{
  std::uint16_t agent_id{0};
  std::uint32_t stamp_ms{0};
};

struct Record
{
  std::uint64_t task_id{0};
  std::uint16_t revision{0};
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
  bool has_digest{false};
  std::vector<DigestEntry> digest;   // has_digest 時才有意義；最多 255 筆
  std::vector<Stamp> stamps;
  std::vector<Record> records;
};

constexpr std::size_t cbbaStateSize(std::size_t stamps, std::size_t records, std::size_t digests = 0)
{
  return kCbbaStateBaseSize + kDigestSize * digests + kStampSize * stamps + kRecordSize * records;
}

// 把完整的一次分成幾批，每批（整個封包）不超過 max_packet。每批的摘要、s 都完整；
// 沒有紀錄時也有一批。part_index、part_count 會填好，其他欄位照抄。
// 摘要和 s 本身就放不下，或超過 255 批時丟出 std::invalid_argument。
std::vector<CbbaStateBody> splitCbbaState(const CbbaStateBody & full,
  std::size_t max_packet = kDefaultMaxPacket);

// 編碼成完整封包；header.type 會設成 CBBA_STATE。欄位不合法時丟出 std::invalid_argument。
std::vector<std::uint8_t> encodeCbbaState(Header header, const CbbaStateBody & body);

// 解碼完整封包。表頭不合法、不是 CBBA_STATE、長度和筆數對不上、欄位不合法時回傳 nullopt：
// part_index ≥ part_count、機號不合法、task_id 不合法（types.hpp 的 validTaskId）、得標價不是有限的非負數、執行者是 0xFFFF、
// 指派狀態不是 0～3，或狀態不是「無」卻沒有執行者；摘要的建立者不合法、known 是 0、closed > known、min_seq > max_seq、
// 世代 0 的流水號超過 0xFFFF。
std::optional<CbbaStateBody> decodeCbbaState(const std::uint8_t * data, std::size_t size);
inline std::optional<CbbaStateBody> decodeCbbaState(const std::vector<std::uint8_t> & bytes)
{
  return decodeCbbaState(bytes.data(), bytes.size());
}

}  // namespace cbba_core::wire
