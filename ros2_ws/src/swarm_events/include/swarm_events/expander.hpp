// 範本展開器（L6 任務生成 PLN-1，2026-11-23；docs/l5_l6_plan.md 第 3 節）
//
// 一個事件（Incident：類型、位置、區域、track_id、設備）＋一條規則 → 一組角色（Plan）：每個角色的 CBBA 任務欄位
// （平台、能力、優先級、位置、optional、precedence、distinct、all-or-nothing、sync_window）和它的 TaskSpec（JSON）。
// 只做規劃，不注入：注入由呼叫的人做（/gcs/expand_incident 只回傳 Plan；使用者 2026-11-23）。
//
// 展開的步驟：
//   1 位置：location（incident＋offset、poi、asset、zone、dock）；poi 依事件所在區域、平台挑，多個取最近，沒有就用 incident＋offset
//   2 preflight（2-1 第三節「目標點不可達 → 移到最近的可達觀察點」）：限平台的角色的觀察點落在這個平台的禁入區
//     → 移到最近的可達點；不限平台（any）的照原位置（能不能進由各台的出價決定，使用者 2026-11-22）
//   3 steps：代換 {incident.*}、wait_for 的角色名稱 → role_index、沒寫 location 的 go_to 填觀察點座標
//   4 檢查：TaskSpec（spec_validator：格式、詞彙、參數、位置、步數、平台、條件）；PLN-2 補的兩層：
//       機隊可行性：非 optional 角色要有載具（fleet.yaml，或呼叫的人給的可用清單）平台、能力、能做的動作都符合 → 否則 FLEET_INFEASIBLE
//       續航：每個可行的載具從自己的起降板／充電樁到觀察點來回＋作業時間都超過平台的續航 → 只警告 ENDURANCE
// 不依賴 ROS。
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "cbba_core/types.hpp"
#include "swarm_cbba/registry.hpp"
#include "swarm_events/event_rules.hpp"

namespace swarm_events
{

// 事件（L5 的輸出；registry/api.md 第 6 節的 Event 中，展開需要的部分）
struct Incident
{
  std::string event_id;
  std::string event_type;         // event_types.yaml 的名稱
  std::uint16_t revision{0};      // incident revision（UAV 定位、位置更新後 + 1）
  double x{0.0}, y{0.0}, z{0.0};  // map ENU
  double radius_m{0.0};
  std::string zone;               // 空 = 由位置判斷（包含它的區域裡面積最小的）
  std::string track_id;
  std::string asset;              // 設備 id（類別:實例；熱異常等 locate: asset 的事件）
};

struct PlannedRole
{
  std::string role;
  std::uint8_t role_index{0};
  std::uint8_t role_count{0};
  std::string platform;           // uav、quadruped、any
  std::uint8_t platform_code{0};  // 0 ANY、1 uav、2 quadruped（platforms.yaml 的 code）
  std::vector<std::string> required_capabilities;
  std::uint8_t priority{3};       // P0～P3
  double x{0.0}, y{0.0}, z{0.0};
  std::string location_source;    // incident、poi:OBS_HV_A_FRONT、asset:machine:TA1、zone:Z、dock:D、
                                  // 以及 preflight 移過的「… → anchor (x, y) out of Z_HV_B」
  bool optional{false};
  bool skip_satisfies{false};
  bool all_or_nothing{false};
  std::uint8_t precedence{0};     // 要等的角色（位元 = role_index）
  std::uint8_t distinct_agent_set{0};
  std::uint16_t sync_window_s{0};
  std::string fallback_template;
  bool requires_approval{false};
  double expires_s{0.0};
  double value{100.0};
  double service_time_s{30.0};
  bool retarget{true};            // false：changedRoles 不看位置的變化（規則的 retarget）
  std::string spec;               // 角色 TaskSpec（JSON）：{"template", "role", "steps", "report_policy"?}
};

struct Plan
{
  bool ok{false};
  std::string code;               // 錯誤碼：UNKNOWN_EVENT_TYPE、UNKNOWN_RULE、BAD_LOCATION、SPEC_INVALID、FLEET_INFEASIBLE
  std::string reason;
  std::string rule;
  std::string event_id;
  std::string event_type;
  std::uint16_t incident_revision{0};
  std::string zone;               // 事件所在區域
  std::vector<PlannedRole> roles;
  std::vector<std::string> warnings;   // TaskSpec 檢查的警告、ENDURANCE、optional 角色沒有能做的載具…
  std::string json;               // 整組（registry/api.md 第 4 節的格式，給日誌、控制中心）
};

struct ExpandOptions
{
  std::string rule;               // 空 = 事件類型的 default_rule
  std::vector<cbba_core::AgentId> available;   // 機隊可行性只看這些（空 = fleet.yaml 全部）
};

Plan expand(const Incident & incident, const EventRules & rules, const swarm_cbba::Registry & registry,
  const swarm_cbba::Site & site, const ExpandOptions & options = ExpandOptions{});

// 包含 (x, y) 的區域裡面積最小的（yard：Z_YARD 包住全部，高壓區在裡面）；沒有 → 空字串
std::string zoneAt(const swarm_cbba::Site & site, double x, double y);

// (x, y) 在這個平台（uav、quadruped）的禁入區 → 區域 id；不在、平台是 any → 空字串
std::string forbiddenZone(const swarm_cbba::Site & site, const std::string & platform, double x, double y);

// 離 (x, y) 最近、這個平台可以進的點（由內往外繞圈找，半徑每 0.5 m、最遠 30 m；找到後再往外 0.5 m，不停在邊線上）；
// 本來就可以 → 原點；找不到 → nullopt
std::optional<std::array<double, 2>> safeAnchor(const swarm_cbba::Site & site, const std::string & platform,
  double x, double y);

// 改版時要重新注入的角色：TaskSpec、位置（超過 retarget_m）或任務欄位不同的角色名稱（只動改了的角色，itest G6）。
// 規則寫 retarget: false 的角色不看位置與 go_to 座標的變化（追蹤目標的 UAV 靠 track_id 跟著走）
std::vector<std::string> changedRoles(const Plan & before, const Plan & after, double retarget_m = 3.0);

}  // namespace swarm_events
