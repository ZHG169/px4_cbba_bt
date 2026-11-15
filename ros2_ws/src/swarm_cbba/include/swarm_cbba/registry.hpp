// 2-1b 註冊表（REG-1，schema 1.0）的載入與檢查：registry/ 的六份
//   tools.yaml、platforms.yaml、fleet.yaml、actions.yaml、objects.yaml、event_types.yaml
// 以及場域設定（2-1b 第五節的位置標準）：site-config/<site>/ 的 zones.geojson、devices.csv、pois.csv、docks.csv
//
// 註冊表是唯一來源（2-1b）：能力的位元位置、平台代碼、每台載具的平台與能力、動作（steps 的 action）的參數、前置條件、
// 效果與各平台實作、物件類別、事件類型。cbba_node 給 registry_dir 時載入，從 fleet.yaml 拿自己的平台與能力；
// 其他幾份與場域設定只檢查（給規劃器、檢查器、事件流程、BT 用）。放在節點層（swarm_cbba），因為 cbba_core 不讀檔案。
// 2026-11-15：actions、objects、event_types、fleet 的格式照 2-1b 原文改寫（名稱 → 內容的表，原文的欄位名稱）。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
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
  // 成本模型（CBBA 出價用，cbba_node 的 applyPlatform）
  double cruise_speed{0.0};
  double energy_per_meter{0.0};
  double hover_energy_per_sec{0.0};
  double safety_reserve{0.0};
  double endurance_s{0.0};
  std::size_t max_bundle{0};
  bool must_return{true};
};

// fleet.yaml 的 limits（原文：UAV 的 max_altitude_m、max_flight_s、landing_pads；我們加狗的 docks）
struct RobotLimits
{
  std::optional<double> max_altitude_m;
  std::optional<double> max_flight_s;
  std::vector<std::string> landing_pads;   // site-config 的 docks.csv（kind pad）
  std::vector<std::string> docks;          // site-config 的 docks.csv（kind dock）
};

struct RobotEntry
{
  cbba_core::AgentId id{cbba_core::kNoAgent};
  std::string name;
  std::string platform;               // platforms.yaml 的 name
  std::string ns;                     // namespace（實機無人機是 Node-RED 的介面前綴 /MAV1/api）
  std::vector<std::string> capabilities;
  RobotLimits limits;
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
  std::string type;            // float、int、string、bool、enum、string_list、waypoints、bbox
  std::vector<std::string> values;   // enum 的值
  std::optional<double> min;
  std::optional<double> max;
  bool has_default{false};
  std::string default_value;   // 原文（YAML 的純量）
  std::string unit;
  bool required{false};
  std::string description;
};

struct ActionImpl
{
  std::string platform;        // platforms.yaml 的 name
  std::string backend;         // task_manager、uav_adapter、gateway
  std::string interface;
  std::vector<std::string> sequence;   // 平台內部動作（class internal）
  std::vector<std::string> extra_preconditions;
  std::vector<std::string> extra_effects;
  std::string autonomy;        // not_started、in_development、sim_verified、field_verified
};

struct ActionEntry
{
  std::string name;
  std::string cls;             // plannable、system、internal
  std::string status;          // open、closed
  int version{1};
  std::vector<ActionParam> params;
  std::string location;        // required、optional、none
  std::string object;          // required、optional、none
  std::vector<std::string> objects;            // 物件類別（objects.yaml）
  std::vector<std::vector<std::string>> requires_any_of;   // 符合其中一組能力即可；空 = 不需要
  std::vector<std::string> preconditions;
  std::vector<std::string> effects;
  double timeout_s{0.0};       // 0 = 依參數算
  bool cancelable{true};
  std::string risk;            // low、medium、high
  std::vector<std::string> failure_codes;
  std::vector<ActionImpl> implementations;

  const ActionImpl * implementation(const std::string & platform) const;
  const ActionParam * param(const std::string & name) const;
};

// 角色的里程碑（CB-4，2026-11-16）：actions.yaml 的 milestones，bit 要和 cbba_core/milestones.hpp 一致
struct MilestoneEntry
{
  std::string name;
  unsigned bit{0};
  std::string kind;            // executor、event、completion
  std::string description;
};

struct ConditionEntry
{
  std::string name;
  std::string description;
};

struct ObjectClass
{
  std::string name;
  std::vector<std::string> locate;      // projection、onboard、asset、asset_pose、site_map、zone
  std::vector<std::string> allowed_actions;
  bool safety_per_instance{false};      // safety_distance_m: per_instance → devices.csv
  std::map<std::string, std::optional<double>> safety_distance_m;   // 平台 → 公尺；nullopt = 不適用
  std::vector<std::string> rules;
  std::vector<std::string> findings;
  std::string note;
};

struct DetectorEntry
{
  std::string name;            // Detection 的 detector
  std::vector<std::string> labels;          // Detection 的 label
  std::vector<std::string> source_kinds;    // fixed_camera、robot_camera、vms_analytics、scada、sensor
  double min_score{0.0};
  std::string status;          // available、planned
};

struct EventType
{
  std::string name;
  std::string category;        // template、open
  std::vector<std::string> detectors;
  std::string zone_attr{"any"};          // trigger
  std::string time_rule{"always"};
  bool has_accumulate{false};
  double lambda{0.0};
  double theta_on{0.0};
  double theta_off{0.0};
  double min_duration_s{0.0};
  bool has_associate{false};
  double associate_dt_s{0.0};
  double associate_r_m{0.0};
  bool has_vlm{false};
  std::string vlm_prompt_id;
  int vlm_samples{1};
  double vlm_timeout_s{0.0};
  bool has_cost{false};
  double cost_miss{1.0};
  double cost_false{1.0};
  std::string locate;          // projection、asset、zone
  std::string default_rule;    // event_rules.yaml 的範本
  int priority{3};             // P0～P3
  bool requires_approval{false};
  std::string object_class;    // 我們加的
  double expires_s{0.0};       // 我們加的
  std::string status;          // open、closed
};

struct Registry
{
  std::string dir;
  std::vector<ToolEntry> tools;
  std::vector<PlatformEntry> platforms;
  std::vector<RobotEntry> robots;
  std::vector<GatewayEntry> gateways;
  std::vector<std::string> common_failure_codes;   // actions.yaml：每個動作都可能回的
  std::vector<ConditionEntry> conditions;          // actions.yaml：前置條件、效果的詞彙
  std::vector<MilestoneEntry> milestones;          // actions.yaml：角色的里程碑（CB-4）
  std::vector<ActionEntry> actions;
  std::vector<ObjectClass> object_classes;
  std::vector<DetectorEntry> detectors;
  std::vector<EventType> event_types;

  const RobotEntry * robot(cbba_core::AgentId id) const;
  const ActionEntry * action(const std::string & name) const;
  const PlatformEntry * platform(const std::string & name) const;
  const ObjectClass * objectClass(const std::string & name) const;
  const EventType * eventType(const std::string & name) const;
};

// 讀 dir 底下的六份。檔案不在、不是 YAML、缺欄位、型別不對、schema_version 不是
// kRegistrySchema 時丟出 std::runtime_error（訊息帶檔名與哪一筆）
Registry loadRegistry(const std::string & dir);

// 內容的檢查，回傳所有錯誤（空 = 沒問題）：
//   tools：name、bit 不重複；bit < 64；**每個標籤的 bit 要和 cbba_core/capabilities.hpp 一樣**，capabilities.hpp 的
//          標籤都要在（註冊表比程式新、多出來的標籤只能是程式還不認得的，不算錯，見 unknownTools）
//   platforms：name、code 不重複；code 1～255（0 保留）；max_bundle ≥ 1
//   robots、gateways：機號 1～254 不重複（含兩者之間）；平台要在 platforms；能力要在 tools、不能是 deprecated；
//            limits 的數值 > 0
//   actions：class、status、location、object、risk、參數的 type、autonomy 是規定的值；enum 參數要有 values、default 在裡面；
//            min ≤ max、default 在範圍內；required 的參數不能有 default；能力（requires.any_of）要在 tools；
//            preconditions、effects 要在 conditions；失敗碼（含 common）要是 8 個 E_* 之一；必須可取消；
//            至少一個平台實作、平台要在 platforms；sequence 只能用這個平台有實作的 internal 動作；
//            **objects 和物件類別的 allowed_actions 要互相一致**
//   milestones：name、bit 不重複；bit < 16；**每個的 bit、kind 要和 cbba_core/milestones.hpp 一樣**，程式認得的都要在
//   objects：locate、rules、findings 是規定的詞；allowed_actions 要是有的動作；safety_distance_m 的平台要在 platforms、值 ≥ 0
//   detectors：labels 非空；名稱、label ≤ 64 B（DETECTION 封包的上限）；source_kinds 是規定的值；min_score 0～1
//   event_types：category template 要有 detectors、default_rule；open 要有 vlm、requires_approval；detectors 要在清單；
//            trigger 的 zone_attr（any、restricted、high_voltage）、time_rule；accumulate 0 < λ ≤ 1、theta_on > theta_off ≥ 0；
//            associate dt_s > 0、r_m ≥ 0；vlm 要有 prompt_id、samples ≥ 1、timeout_s > 0；cost 都 > 0；locate；
//            priority P0～P3；object_class 要在 objects；expires_s ≥ 0；status
std::vector<std::string> checkRegistry(const Registry & registry);

// 註冊表有、但這版程式（capabilities.hpp）不認得的能力標籤：註冊表比程式新。載具宣告了這些能力也用不到
std::vector<std::string> unknownTools(const Registry & registry);

// ============================================================
// 場域設定（2-1b 第五節：zone、poi、asset、dock／pad、pose）：site-config/<site>/
// ============================================================

struct Zone
{
  std::string id;
  std::string name;
  std::string floor;
  bool restricted{false};      // 人員禁入（入侵判定，event_types 的 trigger.zone_attr）
  bool high_voltage{false};
  std::vector<std::string> no_entry;     // 哪些平台不能進（uav = 禁飛）
  std::optional<double> cruise_altitude_m;
  std::optional<double> ceiling_m;
  std::vector<std::array<double, 2>> polygon;   // map ENU，頭尾同一點

  bool contains(double x, double y) const;
};

struct Device
{
  std::string id;              // 類別:實例（machine:TA1）
  std::string cls;
  double x{0.0}, y{0.0}, z{0.0};
  std::string floor;
  std::string zone;
  std::optional<double> safety_quadruped_m;
  std::optional<double> safety_uav_m;
  std::optional<std::array<double, 3>> obs_quadruped;   // x、y、yaw（度）
  std::optional<std::array<double, 3>> obs_uav;         // x、y、z
  std::string name;
};

struct Poi
{
  std::string id;
  std::string zone;
  std::string role;            // front、rear、overwatch…（範本用）
  std::string platform;
  double x{0.0}, y{0.0}, z{0.0};
  std::optional<double> yaw_deg;
  std::string floor;
};

struct Dock
{
  std::string id;
  std::string kind;            // dock、pad
  std::string platform;
  double x{0.0}, y{0.0}, z{0.0};
  std::string floor;
};

struct Site
{
  std::string dir;
  std::vector<Zone> zones;
  std::vector<Device> devices;
  std::vector<Poi> pois;
  std::vector<Dock> docks;

  const Zone * zone(const std::string & id) const;
  const Device * device(const std::string & id) const;
  const Dock * dock(const std::string & id) const;
};

// 讀 dir 底下的四份（zones.geojson、devices.csv、pois.csv、docks.csv）；格式錯誤丟 std::runtime_error
Site loadSite(const std::string & dir);

// 場域設定和註冊表的一致性：id 不重複；設備 id 是「類別:實例」、類別要在 objects.yaml、和 class 欄一致；
// 類別是 per_instance 的設備要有自己的安全距離；zone 要存在；no_entry、poi、dock 的平台要在 platforms.yaml；
// 狗的觀察姿態、狗的觀察點、充電樁不能在狗的禁入區（no_entry 有 quadruped）；UAV 的觀察點不能在禁飛區、
// 不能高於區域的天花板；fleet.yaml 的 landing_pads、docks 要在 docks.csv（kind 與平台要對）
std::vector<std::string> checkSite(const Site & site, const Registry & registry);

}  // namespace swarm_cbba
