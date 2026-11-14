// 2-1b 註冊表（REG-1，schema 1.0，2026-11-14）的載入與檢查：registry/ 的六份
//   tools.yaml、platforms.yaml、fleet.yaml、actions.yaml、objects.yaml、event_types.yaml
//
// 註冊表是唯一來源（2-1b）：能力的位元位置、平台代碼、每台載具的平台與能力、動作（steps 的 action）的參數與失敗碼、
// 對象的詞彙與場地上的設備、事件種類（偵測器、累積門檻、範本）。cbba_node 給 registry_dir 時載入，從 fleet.yaml 拿自己的
// 平台與能力；其他幾份只檢查（給規劃器、檢查器、事件流程、BT 用）。放在節點層（swarm_cbba），因為 cbba_core 不讀檔案。
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

struct ActionParam
{
  std::string name;
  std::string type;          // float、int、string、bool、point、point_list
  std::string unit;
  bool required{false};
  bool has_default{false};
  std::string default_value;  // 原文（YAML 的純量）
  std::string description;
};

struct ActionEntry
{
  std::string name;
  std::string description;
  std::string cls;            // plannable、system、internal
  std::vector<std::string> platforms;
  std::vector<std::string> required_capabilities;
  std::string location;       // required、optional、none
  std::string object;         // required、optional、none
  std::vector<ActionParam> params;
  std::vector<std::string> preconditions;
  std::vector<std::string> failure_codes;
  bool cancellable{true};
  std::string status;         // open、closed
};

struct ObjectClass
{
  std::string name;
  std::string category;      // agent、equipment、structure、hazard、item
  bool trackable{false};
  bool inspectable{false};
  std::string description;
};

struct ObjectInstance
{
  std::string id;
  std::string cls;           // ObjectClass 的 name
  std::string site;
  std::vector<double> position;   // x、y、z（map ENU）
  int floor{0};
};

struct EventDetector
{
  std::string detector;      // Detection 的 detector
  std::vector<std::string> labels;   // Detection 的 label
  double min_score{0.0};
  std::string status;        // available、planned
};

struct EventType
{
  std::string name;
  std::string description;
  int priority{3};
  std::string object_class;
  std::vector<EventDetector> detectors;
  int min_count{1};          // trigger
  double window{0.0};
  double merge_radius{0.0};
  bool vlm_enabled{false};
  std::string vlm_prompt;
  double dispatch_cost{1.0};
  std::string tmpl;          // template
  double expires{0.0};
  std::string status;        // open、closed
};

struct Registry
{
  std::string dir;
  std::vector<ToolEntry> tools;
  std::vector<PlatformEntry> platforms;
  std::vector<RobotEntry> robots;
  std::vector<GatewayEntry> gateways;
  std::vector<std::string> common_failure_codes;   // actions.yaml：每個動作都可能回的
  std::vector<ActionEntry> actions;
  std::vector<ObjectClass> object_classes;
  std::vector<ObjectInstance> object_instances;
  std::vector<EventType> event_types;

  const RobotEntry * robot(cbba_core::AgentId id) const;
  const ActionEntry * action(const std::string & name) const;
  const PlatformEntry * platform(const std::string & name) const;
};

// 讀 dir 底下的六份。檔案不在、不是 YAML、缺欄位、型別不對、schema_version 不是
// kRegistrySchema 時丟出 std::runtime_error（訊息帶檔名與哪一筆）
Registry loadRegistry(const std::string & dir);

// 內容的檢查，回傳所有錯誤（空 = 沒問題）：
//   tools：name、bit 不重複；bit < 64；**每個標籤的 bit 要和 cbba_core/capabilities.hpp 一樣**，capabilities.hpp 的
//          標籤都要在（註冊表比程式新、多出來的標籤只能是程式還不認得的，不算錯，見 unknownTools）
//   platforms：name、code 不重複；code 1～255（0 保留）；max_bundle ≥ 1
//   robots、gateways：機號 1～254 不重複（含兩者之間）；平台要在 platforms；能力要在 tools、不能是 deprecated
//   actions：name 不重複；class、location、object、status、參數的 type 是規定的值；平台、能力要在 platforms、tools；
//            失敗碼（含 common_failure_codes）要是 8 個 E_* 之一（cbba_core/failure_codes.hpp）；參數名稱不重複；
//            required 的參數不能有 default；preconditions 是規定的詞（at_location）
//   objects：種類名稱不重複、category 是規定的值；設備 id 不重複、種類要在 classes、position 3 個數
//   event_types：名稱不重複；priority 0～3；object_class 要在 objects；至少一個偵測器；detector、label ≤ 64 B
//            （DETECTION 封包的上限）；min_score 0～1；min_count ≥ 1、window > 0、merge_radius ≥ 0；VLM 開著要有 prompt；
//            dispatch_cost ≥ 0；要有 template；expires ≥ 0；status、偵測器的 status 是規定的值
std::vector<std::string> checkRegistry(const Registry & registry);

// 註冊表有、但這版程式（capabilities.hpp）不認得的能力標籤：註冊表比程式新。載具宣告了這些能力也用不到
std::vector<std::string> unknownTools(const Registry & registry);

}  // namespace swarm_cbba
