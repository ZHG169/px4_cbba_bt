// AGENT_STATE（代碼 2）的編解碼，對應「五機蜂群：機間通訊封包規格」2026-10-06 §2.4
//
//   表頭 14 | round_key 4 | exchange_step 2 | M 1 | N 1 | Lt 1 | z[M] | y[M] 4M | s[N] 4N |
//   path[Lt] | soc 4 | flags 2 | progress_task 1 | progress 4
//   大小 = 14 + 20 + 5M + 4N + Lt；尚未分配（M = N = Lt = 0）為 34 B
//
//   round_key       任務分配輪次編號
//   exchange_step   CBBA 進行到第幾步
//   z[M]            第 i 格 = 任務編號 i 的得標者機號
//   y[M]            第 i 格 = 任務編號 i 的得標價
//   s[N]            第 k 格 = 機號 k+1 的資訊新舊程度
//   path[Lt]        自己負責的任務編號，依執行順序
//   soc             剩餘電量 0~1
//   flags           bit 0–7：直接聽得到的鄰居（bit k = 機號 k+1）
//                   bit 8–11：armed / offboard / 跟隨中 / 遙測正常
//                   bit 12–15：離最近一次分配變動幾步（0~15，收斂偵測）
//   progress_task   目前執行的任務編號（0xFF = 無）
//   progress        執行進度 0~1
//
// 這裡只處理格式。各欄位填什麼值（round_key、exchange_step、s 的單位等）由 CBBA 節點決定。
//
// 和我們的情況有關的限制：
//   flags 的鄰居位元只放得下機號 1~8；s 以「機號 − 1」當索引，機器狗（agent_id 50）若要放進來，
//   N 會變成 50（多 200 B）。neighborBit() 對 1~8 以外的機號回傳 0。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "uav_cbba/wire_header.hpp"

namespace uav_cbba::wire
{

constexpr std::uint8_t kNoTaskIndex = 0xFF;      // progress_task 的「無」
constexpr std::size_t kAgentStateBaseSize = 34;  // M = N = Lt = 0

struct AgentStateBody
{
  std::uint32_t round_key{0};
  std::uint16_t exchange_step{0};
  std::vector<std::uint8_t> z;      // M
  std::vector<float> y;             // M
  std::vector<std::uint32_t> s;     // N
  std::vector<std::uint8_t> path;   // Lt
  float soc{0.0f};
  std::uint16_t flags{0};
  std::uint8_t progress_task{kNoTaskIndex};
  float progress{0.0f};
};

// 規格的大小公式：14 + 20 + 5M + 4N + Lt
constexpr std::size_t agentStateSize(std::size_t m, std::size_t n, std::size_t lt)
{
  return kAgentStateBaseSize + 5 * m + 4 * n + lt;
}

// 編碼成完整封包（共用表頭 + 內容）；header.type 會設成 AGENT_STATE。
// z 和 y 長度不同，或任一陣列超過 255 筆時丟出 std::invalid_argument。
std::vector<std::uint8_t> encodeAgentState(Header header, const AgentStateBody & body);

// 解碼完整封包。表頭不合法、不是 AGENT_STATE、長度和 M、N、Lt 對不上時回傳 nullopt。
std::optional<AgentStateBody> decodeAgentState(const std::uint8_t * data, std::size_t size);
inline std::optional<AgentStateBody> decodeAgentState(const std::vector<std::uint8_t> & bytes)
{
  return decodeAgentState(bytes.data(), bytes.size());
}

// ---------------------------------------------------------------------------
// flags
// ---------------------------------------------------------------------------
struct StateFlags
{
  std::uint8_t neighbors{0};      // bit k = 機號 k+1
  bool armed{false};
  bool offboard{false};
  bool following{false};
  bool telemetry_ok{false};
  std::uint8_t stable_steps{0};   // 超過 15 時存成 15
};

std::uint16_t packFlags(const StateFlags & flags);
StateFlags unpackFlags(std::uint16_t flags);

// 機號在鄰居位元中的位置；1~8 以外回傳 0
inline std::uint8_t neighborBit(std::uint8_t agent_id)
{
  return agent_id >= 1 && agent_id <= 8 ? static_cast<std::uint8_t>(1u << (agent_id - 1)) : 0;
}

}  // namespace uav_cbba::wire
