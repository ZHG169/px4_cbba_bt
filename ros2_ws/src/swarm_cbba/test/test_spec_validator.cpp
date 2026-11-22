// TaskSpec 的檢查（任務欄位補齊 10，2026-11-18）：用 repo 的 registry/ 與 site-config/yard
//   動作、參數、location／object、步數、平台支援、wait_for、成功／中止條件、timeout_s；核准要看的風險、實作狀態
#include <gtest/gtest.h>

#include <string>

#include "swarm_cbba/registry.hpp"
#include "swarm_cbba/spec_validator.hpp"

using namespace swarm_cbba;

namespace
{
constexpr std::uint8_t kAny = 0;
constexpr std::uint8_t kUav = 1;
constexpr std::uint8_t kDog = 2;

const Registry & reg()
{
  static const Registry r = loadRegistry(REGISTRY_DIR);
  return r;
}
const Site & site()
{
  static const Site s = loadSite(SITE_DIR);
  return s;
}

SpecValidation check(const std::string & spec, std::uint8_t platform = kDog, std::uint8_t role_index = 0,
  std::uint8_t role_count = 0)
{
  return validateSpec(spec, reg(), &site(), platform, role_index, role_count);
}

bool hasCode(const std::vector<SpecIssue> & issues, const std::string & code)
{
  for (const auto & i : issues) {
    if (i.code == code) {
      return true;
    }
  }
  return false;
}

std::string first(const SpecValidation & v)
{
  return v.summary();
}
}  // namespace

TEST(SpecValidator, GoodSpecPasses)
{
  // 狗：到前方觀察點 → 用熱像看設備 → 成功條件「有觀測」；中止條件、逾時；沒有錯也沒有警告
  const auto v = check(
    "{\"steps\": [{\"action\": \"go_to\", \"location\": {\"poi\": \"OBS_HV_A_FRONT\"}},"
    " {\"action\": \"inspect\", \"object\": \"machine:TA1\", \"params\": {\"mode\": \"thermal\", \"duration_s\": 10}}],"
    " \"success_criteria\": [{\"effect\": \"has_observation\"}],"
    " \"abort_conditions\": [{\"failure_code\": \"E_TARGET_LOST\"}, {\"battery_below_pct\": 25}],"
    " \"timeout_s\": 300, \"safety_notes\": [\"不進高壓區\"]}");
  EXPECT_TRUE(v.ok()) << first(v);
  EXPECT_TRUE(v.warnings.empty());
  EXPECT_TRUE(v.high_risk_actions.empty());
  EXPECT_EQ(specTimeout("{\"steps\": [], \"timeout_s\": 300}"), 300.0);
  EXPECT_FALSE(specTimeout("{\"steps\": []}"));
  EXPECT_TRUE(check("").ok());   // 沒有 spec：不檢查
}

TEST(SpecValidator, UnknownOrClosedActionIsRejected)
{
  // 10 驗收 1：不認得的動作 → 注入前就擋（不會到 BT 才回 BAD_SPEC）；平台內部動作（takeoff）不能規劃
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"dance\"}]}").errors, "UNKNOWN_ACTION"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"takeoff\"}]}", kUav).errors, "ACTION_NOT_OPEN"));
  EXPECT_TRUE(hasCode(check("{\"steps\": []}").errors, "SCHEMA"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"object\": \"person\"}]}").errors, "SCHEMA"));
  EXPECT_TRUE(hasCode(check("[1]").errors, "SCHEMA"));
  std::string many = "{\"steps\": [";
  for (int i = 0; i < 21; ++i) {
    many += std::string(i > 0 ? "," : "") + "{\"action\": \"inspect\"}";
  }
  EXPECT_TRUE(hasCode(check(many + "]}").errors, "TOO_MANY_STEPS"));
}

TEST(SpecValidator, ParamsAreChecked)
{
  // 10 驗收 2：缺少必要參數、型別、範圍、enum 不對 → 不注入；不認得的參數只警告（BT 的舊名稱）
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"wait\"}]}", kUav).errors, "PARAM_MISSING"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\", \"params\": {\"duration_s\": 90}}]}").errors,
    "PARAM_OUT_OF_RANGE"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\", \"params\": {\"duration_s\": \"long\"}}]}").errors,
    "PARAM_TYPE"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\", \"params\": {\"mode\": \"xray\"}}]}").errors,
    "PARAM_OUT_OF_RANGE"));
  const auto legacy = check("{\"steps\": [{\"action\": \"inspect\", \"params\": {\"snapshots\": 2}}]}");
  EXPECT_TRUE(legacy.ok()) << first(legacy);
  EXPECT_TRUE(hasCode(legacy.warnings, "PARAM_UNKNOWN"));
}

TEST(SpecValidator, LocationAndObject)
{
  // location：required 的要給、參照要存在、沒代換的範本字串擋下；object：類別要在動作的 objects、設備實例要在 devices.csv
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"go_to\"}]}").errors, "LOCATION_MISSING"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"go_to\", \"location\": \"{incident.zone}\"}]}").errors,
    "UNKNOWN_LOCATION"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"go_to\", \"location\": {\"poi\": \"NOWHERE\"}}]}").errors,
    "UNKNOWN_LOCATION"));
  EXPECT_TRUE(check("{\"steps\": [{\"action\": \"go_to\", \"location\": {\"x\": 1, \"y\": 2}}]}").ok());
  EXPECT_TRUE(check("{\"steps\": [{\"action\": \"go_to\", \"location\": {\"zone\": \"Z_YARD\"}}]}").ok());
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\", \"object\": \"car\"}]}").errors, "OBJECT_NOT_ALLOWED"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\", \"object\": \"machine:ZZ9\"}]}").errors,
    "UNKNOWN_OBJECT"));
  EXPECT_TRUE(check("{\"steps\": [{\"action\": \"inspect\", \"object\": \"person:trk_88\"}]}").ok());   // 追蹤實例不查
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"track\", \"params\": {}}]}", kAny).errors, "OBJECT_MISSING"));
}

TEST(SpecValidator, PlatformSupport)
{
  // 8 個可規劃動作的支援表（actions.yaml 的 implementations，not_started 不算）：狗沒有 patrol、track；
  // 平台不限時有任一個平台支援就好
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"patrol\", \"params\": {\"area\": \"Z_YARD\"}}]}").errors,
    "PLATFORM_NOT_SUPPORTED"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"wait\", \"params\": {\"duration_s\": 5}}]}", kDog).errors,
    "PLATFORM_NOT_SUPPORTED"));
  EXPECT_TRUE(check("{\"steps\": [{\"action\": \"wait\", \"params\": {\"duration_s\": 5}}]}", kUav).ok());
  EXPECT_TRUE(check("{\"steps\": [{\"action\": \"wait\", \"params\": {\"duration_s\": 5}}]}", kAny).ok());
  const auto v = check("{\"steps\": [{\"action\": \"inspect\"}]}", kDog);
  EXPECT_EQ(v.unverified_actions, std::vector<std::string>{"inspect"});   // sim_verified：嚴格的核准政策要核准
}

TEST(SpecValidator, WaitForAndConditions)
{
  // wait_for：同組的其他角色、認得的里程碑；成功條件要有某一步會產生；中止條件的格式
  const std::string wait_ok = "{\"steps\": [{\"action\": \"inspect\", \"wait_for\": "
    "[{\"role_index\": 0, \"milestone\": \"target_localized\"}]}]}";
  EXPECT_TRUE(check(wait_ok, kDog, 1, 2).ok()) << first(check(wait_ok, kDog, 1, 2));
  EXPECT_TRUE(hasCode(check(wait_ok, kDog, 0, 2).errors, "BAD_WAIT_FOR"));   // 等自己
  EXPECT_TRUE(hasCode(check(wait_ok, kDog, 0, 0).errors, "BAD_WAIT_FOR"));   // 單獨任務
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\", \"wait_for\": "
    "[{\"role_index\": 0, \"milestone\": \"teleported\"}]}]}", kDog, 1, 2).errors, "BAD_WAIT_FOR"));

  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"go_to\", \"location\": {\"x\": 1, \"y\": 1}}], "
    "\"success_criteria\": [{\"effect\": \"has_observation\"}]}").errors, "BAD_CONDITION"));   // 沒有人會產生
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\"}], "
    "\"success_criteria\": [{\"effect\": \"magic\"}]}").errors, "BAD_CONDITION"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\"}], "
    "\"abort_conditions\": [{\"failure_code\": \"E_BORED\"}]}").errors, "BAD_CONDITION"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\"}], "
    "\"abort_conditions\": [{\"battery_below_pct\": 25, \"elapsed_s\": 10}]}").errors, "BAD_CONDITION"));
  EXPECT_TRUE(hasCode(check("{\"steps\": [{\"action\": \"inspect\"}], \"timeout_s\": 0}").errors, "BAD_TIMEOUT"));
}

TEST(SpecValidator, HighRiskActionsAreReported)
{
  // 核准要看的：track 的風險是 medium，不算；actions.yaml 現在沒有 high 的可規劃動作 → 用暫存的註冊表改一個
  Registry r = reg();
  for (ActionEntry & a : r.actions) {
    if (a.name == "inspect") {
      a.risk = "high";
    }
  }
  const auto v = validateSpec("{\"steps\": [{\"action\": \"inspect\"}]}", r, &site(), kDog, 0, 0);
  EXPECT_EQ(v.high_risk_actions, std::vector<std::string>{"inspect"});
}

TEST(SpecValidator, ReportPolicy)
{
  // 11：report_policy 的格式；回寫頻道 gcs／vms／operator；required_findings 要是 objects.yaml 的 findings 詞彙
  const std::string steps = "{\"steps\": [{\"action\": \"inspect\"}], \"report_policy\": ";
  const auto good = check(steps + "{\"stream\": false, \"snapshot_period_s\": 5, \"channels\": [\"gcs\", \"vms\"], "
    "\"required_findings\": [\"image\"]}}");
  EXPECT_TRUE(good.ok()) << first(good);
  EXPECT_TRUE(hasCode(check(steps + "{\"channels\": [\"email\"]}}").errors, "BAD_REPORT_POLICY"));
  EXPECT_TRUE(hasCode(check(steps + "{\"required_findings\": [\"selfie\"]}}").errors, "BAD_REPORT_POLICY"));
  EXPECT_TRUE(hasCode(check(steps + "{\"snapshot_period_s\": -1}}").errors, "BAD_REPORT_POLICY"));
  EXPECT_TRUE(hasCode(check(steps + "{\"stream\": \"yes\"}}").errors, "BAD_REPORT_POLICY"));
  EXPECT_TRUE(hasCode(check(steps + "{\"push\": true}}").errors, "BAD_REPORT_POLICY"));

  const ReportPolicy p = specReportPolicy(steps + "{\"stream\": true, \"snapshot_period_s\": 5, \"channels\": [\"gcs\"], "
    "\"required_findings\": [\"image\", \"reading\"]}}");
  EXPECT_TRUE(p.stream);
  EXPECT_DOUBLE_EQ(p.snapshot_period_s, 5.0);
  EXPECT_EQ(p.channels, std::vector<std::string>{"gcs"});
  EXPECT_EQ(p.required_findings, (std::vector<std::string>{"image", "reading"}));
  const ReportPolicy none = specReportPolicy("{\"steps\": []}");
  EXPECT_FALSE(none.stream);
  EXPECT_TRUE(none.channels.empty());
}

TEST(SpecValidator, PlatformSupportCanBeAWarning)
{
  // 閘道（2026-11-19，docs/execution_backend_plan.md）：平台在 actions.yaml 還沒實作（UAV 的 track 是 not_started、UAV 沒有 speak）
  // → 只警告 PLATFORM_NOT_SUPPORTED，照收（各台由註冊表推導能做的動作決定能不能出價）；預設（假 adapter、其他用途）照樣是錯誤。
  // 不認得的動作、go_to 沒有 location 不受影響
  const std::string track = R"({"steps": [{"action": "track", "object": "person"}]})";
  EXPECT_TRUE(hasCode(check(track, kUav).errors, "PLATFORM_NOT_SUPPORTED"));
  ValidateOptions gw;
  gw.platform_support_is_error = false;
  const SpecValidation v = validateSpec(track, reg(), &site(), kUav, 0, 0, gw);
  EXPECT_TRUE(v.ok());
  EXPECT_TRUE(hasCode(v.warnings, "PLATFORM_NOT_SUPPORTED"));
  EXPECT_TRUE(validateSpec(R"({"steps": [{"action": "speak", "params": {"template": "x"}}]})", reg(), &site(),
    kUav, 0, 0, gw).ok());
  EXPECT_TRUE(hasCode(validateSpec(R"({"steps": [{"action": "go_to"}]})", reg(), &site(), kDog, 0, 0, gw).errors,
    "LOCATION_MISSING"));
  EXPECT_TRUE(hasCode(validateSpec(R"({"steps": [{"action": "fly_away"}]})", reg(), &site(), kDog, 0, 0, gw).errors,
    "UNKNOWN_ACTION"));
}
