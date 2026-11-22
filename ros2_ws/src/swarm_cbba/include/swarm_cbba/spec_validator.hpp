// 角色 TaskSpec 的檢查（任務欄位補齊 10「執行規格」，2026-11-18；registry/api.md 第 2 節的前幾層）
//
// 閘道注入前檢查規劃器給的 spec（還沒加 meta 的那份），有錯就不注入（不會變成 BT 才回 BAD_SPEC）：
//   format        JSON 物件、steps 是 1～kMaxSteps 步的清單、每步是物件、有 action
//   vocabulary    action 在 actions.yaml、是可規劃（plannable）而且開放（open）；object 的類別在這個動作的 objects
//                （「類別:實例」：類別要對；設備類〔locate 有 asset〕的實例要在 devices.csv）；location 要存在
//                （zone／poi／asset／dock／{x, y}；範本的 "{incident.zone}" 這類沒代換的字串 → 錯）；
//                 location、object 是 required 的動作一定要給
//   params        型別（float、int、string、bool、enum、string_list、waypoints、bbox）、範圍、enum 的值、required；
//                 不認得的參數只警告（BT 可能還在用舊名稱，例如狗的 snapshots）
//   platform      任務的平台（ANY = 至少一個平台）有這個動作的實作，而且 autonomy 不是 not_started
//   wait_for      role_index 在同組、不是自己；里程碑認得；sync 是非空字串
//   conditions    success_criteria（{"effect": 詞彙}，要有某一步會產生）、abort_conditions（{"failure_code": E_*}、
//                 {"battery_below_pct": 0～100}、{"elapsed_s": > 0}、{"zone_entered": 區域}）每個條件剛好一個鍵
//   timeout_s     任務層級的逾時（> 0，≤ 86400）；safety_notes 是字串清單（只顯示，真正的限制要轉成區域、standoff、
//                 能力、中止條件）
//   report_policy （任務欄位補齊 11，2026-11-18）{"stream": bool, "snapshot_period_s": 0～3600（0 = 不定期拍）,
//                  "channels": [gcs、vms、operator], "required_findings": [objects.yaml 的 findings 詞彙]}；不認得的鍵是錯
// 另外回傳核准要看的：風險 high 的動作、實作還沒 field_verified 的動作（人機與安全，任務欄位補齊 12）。
// 前置條件鏈（preconditions／effects 模擬）、續航、機隊可行性還沒做（api.md 的 preconditions、fleet、endurance 層）。
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "swarm_cbba/registry.hpp"

namespace swarm_cbba
{

constexpr std::size_t kMaxSteps = 20;          // api.md 的 scale 層：每角色 20 步
constexpr double kMaxTaskTimeout = 86400.0;

struct SpecIssue
{
  std::string code;      // api.md 的錯誤碼：SCHEMA、UNKNOWN_ACTION、ACTION_NOT_OPEN、OBJECT_NOT_ALLOWED、UNKNOWN_OBJECT、
                         // UNKNOWN_LOCATION、LOCATION_MISSING、OBJECT_MISSING、PARAM_TYPE、PARAM_OUT_OF_RANGE、PARAM_MISSING、
                         // PARAM_UNKNOWN、PLATFORM_NOT_SUPPORTED、BAD_WAIT_FOR、BAD_CONDITION、BAD_TIMEOUT、TOO_MANY_STEPS
  std::string path;      // steps[1].params.duration_s 這種
  std::string message;
};

struct SpecValidation
{
  std::vector<SpecIssue> errors;
  std::vector<SpecIssue> warnings;
  std::vector<std::string> high_risk_actions;     // risk: high
  std::vector<std::string> unverified_actions;    // 這個平台的實作還沒 field_verified
  bool ok() const {return errors.empty();}
  std::string summary() const;                   // 第一個錯誤（給拒絕原因）；沒有錯誤時空字串
};

// 檢查的選項（2026-11-19）
struct ValidateOptions
{
  // 任務的平台在 actions.yaml 沒有實作（或是 not_started）時：true = 錯誤 PLATFORM_NOT_SUPPORTED；false = 只警告。
  // 閘道用 false（docs/execution_backend_plan.md「not_started：不全域放行」：動作合法就照收，由各台由註冊表推導的
  // 能做的動作決定能不能出價，沒人能做時由孤兒任務、P0 看門狗處理）。不認得的動作、參數、location 等照樣是錯誤
  bool platform_support_is_error{true};
};

// spec：規劃器的 spec（空 = 沒有，直接過）；platform：任務的平台代碼（0 = 不限）；site 可以是 nullptr（不檢查位置、
// 實例是否存在）；role_index、role_count：任務 A 層的（單獨任務 count = 0）
SpecValidation validateSpec(const std::string & spec, const Registry & registry, const Site * site,
  std::uint8_t platform, std::uint8_t role_index, std::uint8_t role_count,
  const ValidateOptions & options = ValidateOptions{});

// 任務層級的 timeout_s（沒有、格式不對回 nullopt）：機器人的 cbba_node 用來看執行逾時
std::optional<double> specTimeout(const std::string & spec);

// report_policy（沒有時全是預設：不串流、不定期拍、沒有回寫頻道、沒有必要的 findings）；閘道整理結果時用
struct ReportPolicy
{
  bool stream{false};
  double snapshot_period_s{0.0};
  std::vector<std::string> channels;
  std::vector<std::string> required_findings;
};
ReportPolicy specReportPolicy(const std::string & spec);

}  // namespace swarm_cbba
