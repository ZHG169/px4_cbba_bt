// 平台與能力標籤：CBBA 的能力過濾（2-1b M0 合約第一節，2026-11-11）
//
// 載具 i 可以對任務 j 出價 ⇔ task.platform ∈ {ANY, agent.platform} ∧ required_caps ⊆ available_caps。
// 不符合時出價 0（約束，不是成本），取代原本寫死的 isCompatible(AgentType, TaskType)。
//
// 能力在封包裡是 uint64 位元遮罩。位元位置是 2-1b 註冊表 tools.yaml 的 `bit:`，**只增不改**：
// 新增標籤用新的位元，刪除的標籤位元保留不重用。註冊表還沒建好之前，這裡的表就是 v1.0 的來源；
// 註冊表建好後要和它一致（之後加測試比對）。
//
// 舊任務種類（TaskType ≠ NONE，協定版本 4 之前的任務全是）：平台、能力需求、優先級由 TaskType 決定
// （effective*()），不讀 Task 的 platform／required_caps／priority 欄位，所以現有的 itest、demo 不用改。
// 載具沒設定能力（capabilities = 0）時，依 AgentType 給預設能力。
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cbba_core/types.hpp"

namespace cbba_core
{

// 能力標籤 v1.0（2-1b 第四節）。值 = 位元位置
enum class Capability : std::uint8_t
{
  MOBILITY_GROUND = 0,
  MOBILITY_AIR = 1,
  INDOOR_FLIGHT = 2,
  STAIRS = 3,
  RGB_CAMERA = 4,
  ZOOM = 5,
  THERMAL_CAMERA = 6,
  SPEAKER = 7,
  SPOTLIGHT = 8,
  GAS_SENSOR = 9,
  MANIPULATOR = 10,
  GRIPPER = 11,
  COMM_RELAY = 12,
};

constexpr Capabilities capBit(Capability c)
{
  return Capabilities{1} << static_cast<std::uint8_t>(c);
}

// 標籤名稱（註冊表、ROS 參數用的字串）→ 能力；不認得回傳 nullopt
std::optional<Capability> capabilityFromName(const std::string & name);
const char * capabilityName(Capability c);

// 字串陣列 → 遮罩；有不認得的名稱時回傳 nullopt，第一個不認得的寫進 bad（可為 nullptr）
std::optional<Capabilities> parseCapabilities(
  const std::vector<std::string> & names, std::string * bad = nullptr);
// 遮罩 → 名稱陣列（current_task 的 required_capabilities；不認得的位元是 "bitN"）
std::vector<std::string> capabilityNames(Capabilities caps);
// 遮罩 → "rgb_camera,thermal_camera"（log 用；0 → "none"；不認得的位元印成 bitN）
std::string capabilitiesToString(Capabilities caps);

// 平台：值和 AgentType 相同（UAV = 1、UGV = 2 = 四足），0 = 不限
constexpr std::uint8_t kPlatformAny = 0;
inline std::uint8_t platformOf(AgentType type) {return static_cast<std::uint8_t>(type);}
const char * platformName(std::uint8_t platform);

// 舊任務種類的對應（合約第一節的表）
std::uint8_t effectivePlatform(const Task & task);
Capabilities effectiveRequiredCaps(const Task & task);
// 載具目前可用的能力：設定的（沒設定時依 AgentType 的預設）扣掉回報故障的
Capabilities availableCaps(const AgentState & agent);
Capabilities defaultCaps(AgentType type);

enum class Fitness { OK, PLATFORM, CAPABILITY };

// 只看平台與能力（其他條件：OPEN、電量、排除、鎖定…在 CbbaAgent／scoring）
Fitness fitness(const AgentState & agent, const Task & task);

}  // namespace cbba_core
