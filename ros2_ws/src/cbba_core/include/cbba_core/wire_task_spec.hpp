// TASK_SPEC（種類 9）：TaskSpec 的一段，轉送（協定版本 4，2026-11-12；doc/cbba_m0_contract.md 第二節「TaskSpec 的傳送」）
//
//   表頭 20 | 轉送區塊 14 | task_id 8 | revision 2 | encoding 1 | part_index 1 | part_count 1 | data_length 2 | data
//   大小 = 20 + 14 + 15 + data_length（data_length 1～kMaxSpecChunk）
//
//   task_id、revision  這一段屬於哪個任務的哪個版本（要和 TASK_ANNOUNCE 的 revision 一樣才用）
//   encoding           0 = 原文 JSON（M0 只用這個）；1 = zlib（保留、暫不用，2026-11-12 使用者：先不加依賴）。
//                      不認得的編碼：解碼成功，但收的一方當成缺 TaskSpec（之後加新編碼不用升協定）
//   part_index、part_count  第幾段（0 起）、共幾段（1～255）
//   data               這一段的位元組。整份（各段依序接起來）的長度、CRC-32 要等於 TASK_ANNOUNCE 的 spec_length、spec_crc
//
// 只收建立者送出的：origin_id 要是 task_id 的建立者（版本 4 的 task_id 和舊型 ID 都一樣）。
// 建立時每段各一則（新的序號、origin），轉送一次；補發（缺口請求、SYNC_BATCH）沿用原本的 origin。
// CBBA 不解析內容：出價只用 TASK_ANNOUNCE（A 層），TaskSpec 只給執行端（current_task.spec）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kTaskSpecBaseSize = kHeaderSize + kRelaySize + 15;
constexpr std::size_t kMaxSpecChunk = 1100;   // 一段的資料上限：一段 + SYNC_BATCH 的表頭還在 1200 B 以內
constexpr std::size_t kMaxSpecParts = 255;
constexpr std::uint8_t kSpecEncodingJson = 0;
constexpr std::uint8_t kSpecEncodingZlib = 1;   // 保留、暫不用

constexpr std::size_t taskSpecSize(std::size_t data) {return kTaskSpecBaseSize + data;}

struct TaskSpecBody
{
  RelayHeader relay;
  std::uint64_t task_id{0};
  std::uint16_t revision{0};
  std::uint8_t encoding{kSpecEncodingJson};
  std::uint8_t part_index{0};
  std::uint8_t part_count{1};
  std::vector<std::uint8_t> data;
};

// 編碼成完整封包；header.type 會設成 TASK_SPEC。task_id 不合法、origin_id 不合法、part_count 是 0、
// part_index ≥ part_count、data 是空的或超過 kMaxSpecChunk 時丟出 std::invalid_argument。
std::vector<std::uint8_t> encodeTaskSpec(Header header, const TaskSpecBody & body);

// 解碼完整封包。表頭或轉送區塊不合法、不是 TASK_SPEC、長度不符、欄位不合法時回傳 nullopt。
std::optional<TaskSpecBody> decodeTaskSpec(const std::uint8_t * data, std::size_t size);
inline std::optional<TaskSpecBody> decodeTaskSpec(const std::vector<std::uint8_t> & bytes)
{
  return decodeTaskSpec(bytes.data(), bytes.size());
}

// 整份 TaskSpec → 各段的資料（每段 ≤ kMaxSpecChunk）。空的回傳空；超過 255 段丟出 std::invalid_argument
std::vector<std::vector<std::uint8_t>> splitSpec(const std::string & spec);

// CRC-32（IEEE 802.3），TASK_ANNOUNCE 的 spec_crc
std::uint32_t specCrc(const std::string & spec);

}  // namespace cbba_core::wire
