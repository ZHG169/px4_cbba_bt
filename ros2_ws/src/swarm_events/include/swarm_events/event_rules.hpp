// 事件範本 event_rules.yaml（L6 任務生成 PLN-1，2026-11-23；docs/l5_l6_plan.md 第 3 節）
//
// site-config/<場域>/event_rules.yaml：規則名稱 → 一組角色（平台、能力、optional、觀察點、steps）＋協同約束。
// event_types.yaml 的 default_rule 指到這裡。格式說明寫在 yard 那份檔案的開頭。
// 這裡只讀檔、檢查；展開在 expander.hpp。不依賴 ROS。
#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "swarm_cbba/registry.hpp"

namespace swarm_events
{

constexpr const char * kRulesSchema = "1.0";

// 角色的觀察點
struct LocationRule
{
  std::string kind{"incident"};   // incident、poi、asset、zone、dock
  std::string value;              // poi：觀察點的 role；asset：設備 id（可以是 {incident.asset}）；zone、dock：id
};

struct RoleRule
{
  std::string role;
  std::string platform{"any"};    // uav、quadruped、any
  std::vector<std::string> required_capabilities;
  bool optional{false};
  bool skip_satisfies{false};
  LocationRule location;
  std::array<double, 2> offset{{0.0, 0.0}};
  double value{100.0};
  double service_time_s{30.0};
  bool retarget{true};            // false：事件位置更新（incident revision）時不跟著改位置（追蹤目標的 UAV、遠處的觀察員）
  YAML::Node steps;               // 原樣保留；展開時代換、轉成 JSON
};

struct Rule
{
  std::string name;
  std::optional<bool> requires_approval;   // 沒寫照事件類型
  std::optional<int> priority;             // 沒寫照事件類型
  std::string fallback_template;
  YAML::Node report_policy;                // 沒寫是 null
  bool all_or_nothing{false};
  std::vector<std::string> distinct;       // 角色名稱
  std::uint16_t sync_window_s{0};
  std::vector<std::pair<std::string, std::string>> precedence;   // (A, B)：B 等 A 完成
  std::vector<RoleRule> roles;

  // 角色名稱 → role_index；沒有 → -1
  int roleIndex(const std::string & role) const;
};

struct EventRules
{
  std::string path;
  std::map<std::string, Rule> rules;

  const Rule * rule(const std::string & name) const;
};

// 讀 event_rules.yaml；不是 YAML、缺欄位、型別不對、schema_version 不對、角色名稱重複時丟 std::runtime_error
EventRules loadEventRules(const std::string & path);

// 內容和註冊表、場域的一致性（回傳所有錯誤，空 = 沒問題）：
//   角色名稱在 task_types.yaml 的 roles；平台是 any 或 platforms.yaml 的 name；能力在 tools.yaml；
//   distinct、precedence、steps 的 wait_for 指的角色要在同一個規則裡、不能指自己；
//   steps 是 1 步以上的清單、每步有 action、action 在 actions.yaml；
//   location 的 kind 是規定的值；poi 的 role、zone、dock 在 site-config（有給 site 時）；
//   fallback_template 要是有的規則；event_types.yaml 的 default_rule（status open、category template）都要有規則
std::vector<std::string> checkEventRules(const EventRules & rules, const swarm_cbba::Registry & registry,
  const swarm_cbba::Site * site);

}  // namespace swarm_events
