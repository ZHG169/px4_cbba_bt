// 補發（anti-entropy）的兩種封包，都不轉送（協定版本 3，2026-10-28；整體說明見 doc/protocol_v3_sync.md）
//
// GAP_REQUEST（種類 7，缺口請求）：20 + 12 + ⌈count / 4⌉ B（版本 4：加 epoch、first_seq 4 B）
//   表頭 20 | target_id 2 | creator 2 | epoch 2 | first_seq 4 | count 2 | bitmap ⌈count / 4⌉
//
//   target_id   要回答的鄰居（只有它回答；multicast，其他人收到就丟）
//   creator、epoch  （建立者, 世代），和 CBBA_STATE 的摘要同一個鍵；舊型 ID 的世代是 0
//   first_seq   點陣圖第一格的流水號（types.hpp 的 taskSeq），count 格（1～kMaxGapSeqs），流水號不繞回
//               （世代 0 不超過 0xFFFF，其他不超過 0xFFFFFFFF）
//   bitmap      每個流水號 2 bit，高位在前：0 不認得、1 進行中、2 已結束（3 保留，拒收）；最後一個位元組沒用到的位元填 0
//
//   回答的一方逐一比對：我知道、對方不認得 → 任務定義；我已結束、對方不認得或進行中 → 結束紀錄。
//   沒有可補的就不回。
//
// SYNC_BATCH（種類 8，分批回覆）：20 + 1 + Σ(3 + length) B，不超過 max_packet
//   表頭 20 | item_count 1 | (type 1 + length 2 + 內容 length) × M
//
//   type      只能是 TASK_ANNOUNCE（3）或 TASK_CLOSE（6）
//   內容      那種封包表頭之後的全部（轉送區塊＋欄位，origin 是原本的）。解碼時補一個表頭，交給原本的解碼函式，
//             驗證、權限檢查都和收到那種封包一樣
//
//   為什麼不另外定義一筆的格式：任務定義、結束紀錄的欄位、驗證已經在 wire_task_announce、wire_completion，
//   這裡只是把幾則裝進同一個 UDP 封包、省表頭；之後那兩種改格式，這裡不用跟著改。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kGapRequestBaseSize = kHeaderSize + 12;
constexpr std::size_t kMaxGapSeqs = 4000;            // 點陣圖 1000 B
constexpr std::size_t kSyncBatchBaseSize = kHeaderSize + 1;
constexpr std::size_t kSyncItemOverhead = 3;         // type 1 + length 2

// 點陣圖裡每個流水號的狀態
enum class SeqState : std::uint8_t { UNKNOWN = 0, OPEN = 1, CLOSED = 2 };

constexpr std::size_t gapRequestSize(std::size_t count)
{
  return kGapRequestBaseSize + (count + 3) / 4;
}

struct GapRequestBody
{
  std::uint16_t target_id{0};
  std::uint16_t creator{0};
  std::uint16_t epoch{0};
  std::uint32_t first_seq{0};
  std::vector<SeqState> states;   // states[i] 是流水號 first_seq + i 的狀態
};

// 編碼成完整封包；header.type 會設成 GAP_REQUEST。target、creator 不合法、states 是空的或超過 kMaxGapSeqs、
// first_seq + count 超過那個世代的流水號範圍時丟出 std::invalid_argument。
std::vector<std::uint8_t> encodeGapRequest(Header header, const GapRequestBody & body);
// 解碼完整封包。表頭不合法、種類不符、長度不符、欄位不合法（含點陣圖的值 3）時回傳 nullopt
std::optional<GapRequestBody> decodeGapRequest(const std::uint8_t * data, std::size_t size);
inline std::optional<GapRequestBody> decodeGapRequest(const std::vector<std::uint8_t> & bytes)
{
  return decodeGapRequest(bytes.data(), bytes.size());
}

struct SyncItem
{
  PacketType type{PacketType::TASK_ANNOUNCE};   // TASK_ANNOUNCE 或 TASK_CLOSE
  std::vector<std::uint8_t> payload;            // 那種封包表頭之後的全部
};

inline std::size_t syncItemSize(const SyncItem & item)
{
  return kSyncItemOverhead + item.payload.size();
}

// 完整封包（TASK_ANNOUNCE、TASK_CLOSE）→ 一筆：去掉表頭。種類不對或表頭不合法時回傳 nullopt
std::optional<SyncItem> toSyncItem(const std::vector<std::uint8_t> & packet);
// 一筆 → 完整封包（補一個表頭，sender 用 header 的），交給原本的解碼函式
std::vector<std::uint8_t> syncItemPacket(const SyncItem & item, Header header);

// 編碼成完整封包；header.type 會設成 SYNC_BATCH。沒有筆數、超過 255 筆、種類不對、內容超過 65535 B 時
// 丟出 std::invalid_argument（大小上限由呼叫端控制）。
std::vector<std::uint8_t> encodeSyncBatch(Header header, const std::vector<SyncItem> & items);
// 解碼完整封包：表頭不合法、種類不符、長度對不上、筆數是 0、item 種類不對時回傳 nullopt。
// 每一筆的內容在這裡不驗證，用 syncItemPacket() 補表頭後交給 decodeTaskAnnounce／decodeTaskClose
std::optional<std::vector<SyncItem>> decodeSyncBatch(const std::uint8_t * data, std::size_t size);
inline std::optional<std::vector<SyncItem>> decodeSyncBatch(const std::vector<std::uint8_t> & bytes)
{
  return decodeSyncBatch(bytes.data(), bytes.size());
}

}  // namespace cbba_core::wire
