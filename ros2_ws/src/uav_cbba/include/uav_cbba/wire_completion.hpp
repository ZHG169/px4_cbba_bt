// COMPLETION（代碼 6）與 COMPLETION_ACK（代碼 8）的編解碼，
// 對應「五機蜂群：機間通訊封包規格」2026-10-06 §2.8、§2.10
//
// COMPLETION：宣告 14 + 25 = 39 B；證明 14 + 26 + 9K B（K = 5 時 85 B）
//   表頭 14 | 轉送表頭 9 | kind 1 | task_index 1 | executor_id 1 | round_key 4 |
//   control_revision 8 | view_bits 1 | [ack_count 1 | (確認者 1 + 序號 8) × K]（只有證明）
//
//   kind              0 = 宣告，1 = 證明
//   executor_id       執行的機號
//   round_key         任務分配輪次編號
//   control_revision  任務分配版本號
//   view_bits         執行機認為目前存活的飛機（bit k = 機號 k+1）
//   acks              證明才有：每筆是確認者機號，以及那則 COMPLETION_ACK 的 origin_seq
//
// COMPLETION_ACK：14 + 25 = 39 B
//   表頭 14 | 轉送表頭 9 | task_index 1 | executor_id 1 | round_key 4 | control_revision 8 |
//   view_bits 1 | accepted 1
//
//   view_bits   確認者認為目前存活的飛機
//   accepted    1 = 接受，0 = 拒絕
//
// 流程（規格）：執行機到達後送宣告 → 各機回確認 → 收齊後送證明，全隊收到證明才標為完成。
//   宣告沒收齊確認時每 1 s 重送（最多 10 次）；收到重複的宣告時，原樣回送之前的確認。
//
// 和規格不同、以我們的情況為主的地方：
//   1. kind 只能是 0 或 1、accepted 只能是 0 或 1；宣告不能帶確認清單。其他值一律拒收。
//   2. task_index 不能是 255（保留為「無」）；executor_id 與確認清單裡的機號不能是 0。
//   3. 重送一律換新的序號（規格寫「原樣回送」，沒說序號要不要換）：
//        ・宣告每次重送都用新的 seq（origin_seq 跟著換）。
//        ・確認者「原樣回送」的意思是**決定不變**（第一次接受，之後也一定接受），序號換新。
//      原因：轉送的飛機以（種類, origin_id, origin_seq）去重複。重送若沿用舊序號，
//      多跳時第一次轉送掉了，之後的重送全會被當成重複擋掉，永遠補不回來：
//        uav1 ── uav2 ── uav3（執行機），uav1 的確認經 uav2 轉送時掉了
//        → uav1 原樣回送（序號相同）→ uav2 判定重複不轉送 → uav3 永遠收不到
//      換新序號後，轉送的飛機不需要任何特例，照一般去重複即可。
//      執行機以（確認者、任務、round_key）辨認確認，同一台的確認收到幾份都只算一次；
//      證明裡的序號填實際收到的那一份。
//   4. 規格沒說證明的 K 含不含執行機自己。格式上不限制，由 CBBA 節點決定。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "uav_cbba/wire_header.hpp"

namespace uav_cbba::wire
{

constexpr std::uint8_t kCompletionAnnounce = 0;
constexpr std::uint8_t kCompletionProof = 1;
constexpr std::size_t kCompletionAnnounceSize = 39;
constexpr std::size_t kCompletionAckSize = 39;

constexpr std::size_t completionProofSize(std::size_t ack_count)
{
  return 40 + 9 * ack_count;
}

struct AckRef
{
  std::uint8_t agent_id{0};    // 確認者
  std::uint64_t ack_seq{0};    // 那則 COMPLETION_ACK 的 origin_seq
};

struct CompletionBody
{
  RelayHeader relay;
  std::uint8_t kind{kCompletionAnnounce};
  std::uint8_t task_index{0};
  std::uint8_t executor_id{0};
  std::uint32_t round_key{0};
  std::uint64_t control_revision{0};
  std::uint8_t view_bits{0};
  std::vector<AckRef> acks;    // 只有證明才有
};

struct CompletionAckBody
{
  RelayHeader relay;
  std::uint8_t task_index{0};
  std::uint8_t executor_id{0};
  std::uint32_t round_key{0};
  std::uint64_t control_revision{0};
  std::uint8_t view_bits{0};
  bool accepted{true};
};

// 編碼成完整封包；header.type 會設成對應的種類。內容不合法時丟出 std::invalid_argument。
std::vector<std::uint8_t> encodeCompletion(Header header, const CompletionBody & body);
std::vector<std::uint8_t> encodeCompletionAck(Header header, const CompletionAckBody & body);

// 解碼完整封包。表頭或轉送表頭不合法、種類不符、長度不符、內容不合法時回傳 nullopt。
std::optional<CompletionBody> decodeCompletion(const std::uint8_t * data, std::size_t size);
inline std::optional<CompletionBody> decodeCompletion(const std::vector<std::uint8_t> & bytes)
{
  return decodeCompletion(bytes.data(), bytes.size());
}
std::optional<CompletionAckBody> decodeCompletionAck(const std::uint8_t * data, std::size_t size);
inline std::optional<CompletionAckBody> decodeCompletionAck(const std::vector<std::uint8_t> & bytes)
{
  return decodeCompletionAck(bytes.data(), bytes.size());
}

}  // namespace uav_cbba::wire
