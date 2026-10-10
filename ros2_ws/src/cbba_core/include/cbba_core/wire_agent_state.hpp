// AGENT_STATE（種類 1）：載具狀態，定期送（協定版本 2，見 wire_header.hpp）
//
//   表頭 20 | flags 1 | vehicle_type 1 | x、y、z 12 | battery 4 | progress 4 |
//   execution_state 1 | exec_flags 1 | remaining_time 4 |
//   active_task 4 | active_version 8 | queued_task 4 | queued_version 8 |
//   path_count 1 | path task_id 4 × L | neighbor_count 1 | neighbor_id 2 × K
//   大小 = 20 + 54 + 4L + 2K
//
//   flags            bit 0 參與出價、bit 1 遙測正常（robot_state 有在更新）、bit 2 飛行狀態有效、
//                    bit 3 armed、bit 4 offboard、bit 5 landed、bit 6 跟隨中（固定 0，等 BT）
//                    bit 3～5 只在 bit 2 為 1 時有意義（機器狗一律 0）
//   vehicle_type     1 無人機、2 地面載具（和 types.hpp 的 AgentType 相同）
//   x、y、z          map ENU（公尺）
//   battery          0～100 %
//   progress         執行進度 0～1（固定 0，等 BT）
//   execution_state  BT 的執行狀態（swarm_interfaces/ExecState：0 閒置、1 前往、2 處置中、3 不可中斷、4 停止中、5 故障）
//   exec_flags       bit 0 有 BT 的執行狀態（沒有時 execution_state、remaining_time 沒有意義）、
//                    bit 1 可以安全中斷、bit 2 有執行中的任務、bit 3 有保留（排隊）的任務
//   remaining_time   目前任務的預估剩餘時間（秒）；負值 = 不知道
//   active_*         目前正式指派、執行中的任務與版本（bit 2 為 0 時沒有意義）
//   queued_*         已接受、排隊中（保留）的任務與版本（bit 3 為 0 時沒有意義）
//   path             自己負責的任務，依執行順序
//   neighbor_id      直接聽得到的鄰居機號（完成確認要知道誰聽得到誰）
//
// 有沒有任務用旗標表示，不用 task_id = 0：用旗標比較明確，也不怕之後 task_id 的格式改變。
// 版本 1 把狀態和共識資料放在同一個封包；版本 2 分開，共識資料在 CBBA_STATE（wire_cbba_state.hpp）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cbba_core/wire_header.hpp"

namespace cbba_core::wire
{

constexpr std::size_t kAgentStateBaseSize = kHeaderSize + 54;   // L = K = 0
constexpr std::uint8_t kMaxExecutionState = 5;

struct StateFlags
{
  bool participating{false};
  bool telemetry_ok{false};
  bool flight_state_valid{false};
  bool armed{false};
  bool offboard{false};
  bool landed{false};
  bool following{false};
};

std::uint8_t packFlags(const StateFlags & flags);
StateFlags unpackFlags(std::uint8_t flags);

struct AgentStateBody
{
  StateFlags flags;
  std::uint8_t vehicle_type{1};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float battery{0.0f};
  float progress{0.0f};
  bool exec_known{false};
  std::uint8_t execution_state{0};
  bool preemptible{false};
  float remaining_time{-1.0f};
  bool has_active{false};
  std::uint32_t active_task{0};
  std::uint64_t active_version{0};
  bool has_queued{false};
  std::uint32_t queued_task{0};
  std::uint64_t queued_version{0};
  std::vector<std::uint32_t> path;
  std::vector<std::uint16_t> neighbors;
};

constexpr std::size_t agentStateSize(std::size_t path, std::size_t neighbors)
{
  return kAgentStateBaseSize + 4 * path + 2 * neighbors;
}

// 編碼成完整封包；header.type 會設成 AGENT_STATE。path 或 neighbors 超過 255 筆、
// vehicle_type 不是 1 或 2、execution_state 超過 5、鄰居機號不合法時丟出 std::invalid_argument。
std::vector<std::uint8_t> encodeAgentState(Header header, const AgentStateBody & body);

// 解碼完整封包。表頭不合法、不是 AGENT_STATE、長度和筆數對不上、欄位不合法時回傳 nullopt。
std::optional<AgentStateBody> decodeAgentState(const std::uint8_t * data, std::size_t size);
inline std::optional<AgentStateBody> decodeAgentState(const std::vector<std::uint8_t> & bytes)
{
  return decodeAgentState(bytes.data(), bytes.size());
}

}  // namespace cbba_core::wire
