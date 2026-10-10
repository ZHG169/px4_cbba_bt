// 任務完成的三種封包，都會轉送（協定版本 2，見 wire_header.hpp）
//
// COMPLETION（種類 4，完成宣告）：20 + 33 + 2V B
//   表頭 20 | 轉送區塊 14 | task_id 4 | executor_id 2 | round_key 4 | assign_version 8 |
//   view_count 1 | view_id 2 × V
//
// COMPLETION_ACK（種類 5，完成確認）：20 + 34 + 2V B
//   表頭 20 | 轉送區塊 14 | task_id 4 | executor_id 2 | round_key 4 | assign_version 8 | status 1 |
//   view_count 1 | view_id 2 × V
//
// TASK_CLOSE（種類 6，任務結束）：20 + 35 + 2V + 6K B
//   表頭 20 | 轉送區塊 14 | task_id 4 | reason 1 | actor_id 2 | round_key 4 | assign_version 8 |
//   view_count 1 | view_id 2 × V | ack_count 1 | (確認者 2 + ack_seq 4) × K
//
//   executor_id     執行（宣告完成）的機號
//   assign_version  執行者那次正式指派的版本（見 wire_cbba_state.hpp）。收到的節點知道更新的版本時，這是舊的回報
//   round_key       宣告當時的任務摘要（記錄用）
//   view            宣告者（確認者）認為目前存活的機號。版本 1 是 8 個位元，只放得下機號 1～8；
//                   版本 2 改成機號清單，機號超過 8 的載具（狗 50）也會被等待確認
//   status          1 = 接受；0 = 拒絕：不認得這個任務；2 = 拒絕：指派已過期（知道更新的版本）
//   reason          1 = 完成（DONE）、2 = 取消（CANCELLED），和 TaskStatus 相同
//   actor_id        結束任務的機號：完成時是執行者，取消時是發起者。必須等於 origin_id（自己發的）
//   acks            完成時：每筆是確認者，以及那則 COMPLETION_ACK 的 origin_seq
//
// 權限（收到 TASK_CLOSE 的節點檢查，不合格就不結束、記數）：
//   完成：actor 是執行者，assign_version 不比自己知道的最新正式指派舊。
//   取消：actor 是任務的建立者（task_id 的高 16 位元），或在 cancel_authorities（授權的管理端）裡。
//   執行失敗不送 TASK_CLOSE：任務保留，由 CBBA 重新分配。
//
// 流程：執行機送 COMPLETION → 各機回 COMPLETION_ACK → 收齊後送 TASK_CLOSE，全隊收到 TASK_CLOSE 才標為結束。
//   宣告沒收齊時每 1 s 重送（最多 10 次），之後以收到的確認送出 TASK_CLOSE；宣告後才失聯的不再等。
//   收到的 TASK_CLOSE 比任務定義先到（補發時兩者延遲不同）時先暫存。
//   已結束的任務一直記著（不回收），延遲的 TASK_ANNOUNCE、CBBA_STATE 不會把它重新加回來。
//
// 重送一律換新的序號（不是「原樣回送」）：
//   宣告每次重送都用新的 sequence（origin_seq 跟著換）；確認者對同一個宣告的決定不變，序號換新。
//   原因：轉送的飛機以（種類, origin_id, origin_session, origin_seq）去重複。重送若沿用舊序號，
//   多跳時第一次轉送掉了，之後的重送全會被當成重複擋掉，永遠補不回來：
//     uav1 ── uav2 ── uav3（執行機），uav1 的確認經 uav2 轉送時掉了
//     → uav1 原樣回送（序號相同）→ uav2 判定重複不轉送 → uav3 永遠收不到
//   執行機以確認者辨認確認，同一台的確認收到幾份都只算一次。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::uint8_t kCloseDone = 1;
constexpr std::uint8_t kCloseCancelled = 2;

constexpr std::size_t completionSize(std::size_t view)
{
  return kHeaderSize + kRelaySize + 19 + 2 * view;
}
constexpr std::size_t completionAckSize(std::size_t view)
{
  return kHeaderSize + kRelaySize + 20 + 2 * view;
}
constexpr std::size_t taskCloseSize(std::size_t view, std::size_t acks)
{
  return kHeaderSize + kRelaySize + 21 + 2 * view + 6 * acks;
}

struct AckRef
{
  std::uint16_t agent_id{0};   // 確認者
  std::uint32_t ack_seq{0};    // 那則 COMPLETION_ACK 的 origin_seq
};

struct CompletionBody
{
  RelayHeader relay;
  std::uint32_t task_id{0};
  std::uint16_t executor_id{0};
  std::uint32_t round_key{0};
  std::uint64_t assign_version{0};
  std::vector<std::uint16_t> view;
};

constexpr std::uint8_t kAckUnknownTask = 0;
constexpr std::uint8_t kAckAccepted = 1;
constexpr std::uint8_t kAckStaleAssignment = 2;

struct CompletionAckBody
{
  RelayHeader relay;
  std::uint32_t task_id{0};
  std::uint16_t executor_id{0};
  std::uint32_t round_key{0};
  std::uint64_t assign_version{0};
  std::uint8_t status{kAckAccepted};
  std::vector<std::uint16_t> view;
};

struct TaskCloseBody
{
  RelayHeader relay;
  std::uint32_t task_id{0};
  std::uint8_t reason{kCloseDone};
  std::uint16_t actor_id{0};
  std::uint32_t round_key{0};
  std::uint64_t assign_version{0};
  std::vector<std::uint16_t> view;
  std::vector<AckRef> acks;
};

// 編碼成完整封包；header.type 會設成對應的種類。內容不合法時丟出 std::invalid_argument：
// task_id 的建立者是 0、機號不合法、reason 不是 1 或 2、status 不是 0～2、清單超過 255 筆。
std::vector<std::uint8_t> encodeCompletion(Header header, const CompletionBody & body);
std::vector<std::uint8_t> encodeCompletionAck(Header header, const CompletionAckBody & body);
std::vector<std::uint8_t> encodeTaskClose(Header header, const TaskCloseBody & body);

// 解碼完整封包。表頭或轉送區塊不合法、種類不符、長度不符、內容不合法時回傳 nullopt。
std::optional<CompletionBody> decodeCompletion(const std::uint8_t * data, std::size_t size);
std::optional<CompletionAckBody> decodeCompletionAck(const std::uint8_t * data, std::size_t size);
std::optional<TaskCloseBody> decodeTaskClose(const std::uint8_t * data, std::size_t size);
inline std::optional<CompletionBody> decodeCompletion(const std::vector<std::uint8_t> & bytes)
{
  return decodeCompletion(bytes.data(), bytes.size());
}
inline std::optional<CompletionAckBody> decodeCompletionAck(const std::vector<std::uint8_t> & bytes)
{
  return decodeCompletionAck(bytes.data(), bytes.size());
}
inline std::optional<TaskCloseBody> decodeTaskClose(const std::vector<std::uint8_t> & bytes)
{
  return decodeTaskClose(bytes.data(), bytes.size());
}

}  // namespace cbba_core::wire
