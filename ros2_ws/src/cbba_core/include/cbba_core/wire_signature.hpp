// 有權限意義的封包的簽章（任務欄位補齊 13「簽章與資安」，協定版本 5，2026-11-18；docs/planner_task_definition_plan.md）
//
// 簽的種類：TASK_ANNOUNCE（建立、改版）、TASK_SPEC、TASK_CLOSE（完成、取消、過期、略過）、INCIDENT_UPDATE。
// 簽章放在封包最後（算在表頭的 payload_length 裡），所以轉送（makeRelay）、補發（SYNC_BATCH 的一筆 = 表頭之後的全部）
// 都原樣帶著：
//   … 原本的封包 … | key_id 2 | issued_at_ms 8 | tag N | alg 1 | magic "CBSG" 4
//   alg 1 = HMAC-SHA256 取前 16 B（N = 16，共 31 B；PoC）；之後 Ed25519 用 alg 2（N = 64）
// 簽的內容（canonical：本來就是固定的大端序逐欄編碼，各節點算出一樣的位元組）：
//   協定版本 1 | 種類 1 | 表頭之後、簽章之前的全部（轉送區塊＋內容）| key_id 2 | issued_at_ms 8
// 不簽表頭（轉送時發送者、序號會換）。nonce 就是轉送區塊的 (origin_session, origin_seq)：同一則事件的重送、轉送、補發
// 都是同一組，重複收到 = 重放（去重複照舊擋下）。簽的人（issuer）＝ origin_id，金鑰用 (issuer, key_id) 找。
// 加密（機密性）不在這裡：交給 mesh／VPN（一般的 TLS 不能直接用在 UDP multicast）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::uint8_t kSigHmac128 = 1;
constexpr std::size_t kSigHmacTag = 16;
constexpr std::size_t kSigTrailerSize = 2 + 8 + kSigHmacTag + 1 + 4;   // 31
constexpr std::uint8_t kSigMagic[4] = {'C', 'B', 'S', 'G'};

bool isSignedType(PacketType type);

struct SignatureTrailer
{
  std::uint16_t key_id{0};
  std::uint64_t issued_ms{0};
  std::uint8_t alg{kSigHmac128};
  std::vector<std::uint8_t> tag;
};

// 封包最後有沒有簽章：有的話回傳（去掉簽章、payload_length 改好的封包, 簽章）；沒有回傳 nullopt。
// 表頭不合法、alg 不認得、長度不夠也是 nullopt（當成沒有簽章）
std::optional<std::pair<std::vector<std::uint8_t>, SignatureTrailer>> splitSignature(
  const std::vector<std::uint8_t> & packet);

// 簽：在沒有簽章的封包後面加 HMAC-SHA256 的簽章（key：issuer 的這把金鑰）
std::vector<std::uint8_t> appendHmacSignature(const std::vector<std::uint8_t> & packet, std::uint16_t key_id,
  std::uint64_t issued_ms, const std::vector<std::uint8_t> & key);

// 驗：stripped 是 splitSignature 的第一個（沒有簽章的封包）
bool verifyHmacSignature(const std::vector<std::uint8_t> & stripped, const SignatureTrailer & sig,
  const std::vector<std::uint8_t> & key);

}  // namespace cbba_core::wire
