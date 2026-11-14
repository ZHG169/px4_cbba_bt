// 2-1b 註冊表（REG-1，schema 1.0，2026-11-14）的載入與檢查：registry/ 的 tools.yaml、platforms.yaml、fleet.yaml
//
// 註冊表是唯一來源（2-1b）：能力的位元位置、平台代碼、每台載具的平台與能力。cbba_node 給 registry_dir 時載入，
// 從 fleet.yaml 拿自己的平台與能力。放在節點層（swarm_cbba），因為 cbba_core 不讀檔案（分層規則）。
// actions、objects、event_types 三份由規劃器、事件流程（PLN、EVT）讀，CBBA 不用。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cbba_core/types.hpp"

namespace swarm_cbba
{

constexpr const char * kRegistrySchema = "1.0";

struct ToolEntry
{
  std::string name;
  unsigned bit{0};
  bool deprecated{false};
  std::string description;
};

struct PlatformEntry
{
  std::string name;
  std::uint8_t code{0};
  std::string description;
  // 成本模型（M0 只載入、檢查；M1 才拿來出價）
  double cruise_speed{0.0};
  double energy_per_meter{0.0};
  double hover_energy_per_sec{0.0};
  double safety_reserve{0.0};
  double endurance_s{0.0};
  std::size_t max_bundle{0};
  bool must_return{true};
};

struct RobotEntry
{
  cbba_core::AgentId id{cbba_core::kNoAgent};
  std::string name;
  std::string platform;               // platforms.yaml 的 name
  std::string ns;                     // namespace
  std::vector<std::string> capabilities;
};

struct GatewayEntry
{
  cbba_core::AgentId id{cbba_core::kNoAgent};
  std::string name;
  std::string ns;
};

struct Registry
{
  std::string dir;
  std::vector<ToolEntry> tools;
  std::vector<PlatformEntry> platforms;
  std::vector<RobotEntry> robots;
  std::vector<GatewayEntry> gateways;

  const RobotEntry * robot(cbba_core::AgentId id) const;
  const PlatformEntry * platform(const std::string & name) const;
};

// 讀 dir 底下的 tools.yaml、platforms.yaml、fleet.yaml。檔案不在、不是 YAML、缺欄位、型別不對、schema_version 不是
// kRegistrySchema 時丟出 std::runtime_error（訊息帶檔名與哪一筆）
Registry loadRegistry(const std::string & dir);

// 內容的檢查，回傳所有錯誤（空 = 沒問題）：
//   tools：name、bit 不重複；bit < 64；**每個標籤的 bit 要和 cbba_core/capabilities.hpp 一樣**，capabilities.hpp 的
//          標籤都要在（註冊表比程式新、多出來的標籤只能是程式還不認得的，不算錯，見 unknownTools）
//   platforms：name、code 不重複；code 1～255（0 保留）；max_bundle ≥ 1
//   robots、gateways：機號 1～254 不重複（含兩者之間）；平台要在 platforms；能力要在 tools、不能是 deprecated
std::vector<std::string> checkRegistry(const Registry & registry);

// 註冊表有、但這版程式（capabilities.hpp）不認得的能力標籤：註冊表比程式新。載具宣告了這些能力也用不到
std::vector<std::string> unknownTools(const Registry & registry);

}  // namespace swarm_cbba
