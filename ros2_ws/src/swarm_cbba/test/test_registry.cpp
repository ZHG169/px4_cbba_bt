// 2-1b 註冊表（schema 1.0；2026-11-15 照 2-1b 原文改寫）：repo 的 registry/ 與 site-config/yard 載入、和 capabilities.hpp 一致；
// 原文的範例（inspect、perimeter_intrusion）原樣能載入；不合法的註冊表、場域設定被擋下
#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "cbba_core/capabilities.hpp"
#include "swarm_cbba/registry.hpp"

using namespace swarm_cbba;
namespace fs = std::filesystem;

namespace
{
std::string readFile(const fs::path & p)
{
  std::ifstream in(p);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// 暫存目錄：先複製 repo 的註冊表與 yard 的場域設定，再改其中一份
class TempRegistry
{
public:
  TempRegistry()
  : dir_(fs::temp_directory_path() / ("cbba_registry_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
      "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name()))
  {
    fs::remove_all(dir_);
    fs::create_directories(dir_ / "site");
    for (const char * f : {"tools.yaml", "platforms.yaml", "fleet.yaml", "actions.yaml", "objects.yaml",
        "event_types.yaml", "task_types.yaml"})
    {
      fs::copy_file(fs::path(REGISTRY_DIR) / f, dir_ / f);
    }
    for (const char * f : {"zones.geojson", "devices.csv", "pois.csv", "docks.csv"}) {
      fs::copy_file(fs::path(SITE_DIR) / f, dir_ / "site" / f);
    }
  }
  ~TempRegistry() {fs::remove_all(dir_);}

  void write(const std::string & file, const std::string & text) const
  {
    std::ofstream(dir_ / file) << text;
  }
  // 在 repo 的那一份後面接一段
  void append(const std::string & file, const std::string & text) const
  {
    write(file, readFile(dir_ / file) + text);
  }
  std::string dir() const {return dir_.string();}
  std::string site() const {return (dir_ / "site").string();}

private:
  fs::path dir_;
};

bool hasError(const std::vector<std::string> & errors, const std::string & part)
{
  for (const auto & e : errors) {
    if (e.find(part) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::string firstOf(const std::vector<std::string> & errors)
{
  return errors.empty() ? std::string() : errors.front();
}
}  // namespace

TEST(Registry, RepoRegistryLoadsAndMatchesCode)
{
  // repo 的 registry/：沒有錯誤；每個能力的 bit 和 capabilities.hpp 一樣；機隊照模擬的 docker（uav1～3、狗 50～52、閘道 200）
  const Registry reg = loadRegistry(REGISTRY_DIR);
  const auto errors = checkRegistry(reg);
  EXPECT_TRUE(errors.empty()) << firstOf(errors);
  EXPECT_TRUE(unknownTools(reg).empty());
  EXPECT_EQ(reg.tools.size(), 13u);
  for (const ToolEntry & t : reg.tools) {
    const auto cap = cbba_core::capabilityFromName(t.name);
    ASSERT_TRUE(cap) << t.name;
    EXPECT_EQ(t.bit, static_cast<unsigned>(*cap)) << t.name;
  }
  ASSERT_NE(reg.platform("uav"), nullptr);
  EXPECT_EQ(reg.platform("uav")->code, 1);
  EXPECT_EQ(reg.platform("uav")->max_bundle, 2u);
  EXPECT_EQ(reg.platform("quadruped")->code, 2);
  // 平台代碼和 AgentType 一樣（uav = UAV、quadruped = UGV）
  EXPECT_EQ(reg.platform("uav")->code, cbba_core::platformOf(cbba_core::AgentType::UAV));
  EXPECT_EQ(reg.platform("quadruped")->code, cbba_core::platformOf(cbba_core::AgentType::UGV));

  const RobotEntry * dog = reg.robot(50);
  ASSERT_NE(dog, nullptr);
  EXPECT_EQ(dog->platform, "quadruped");
  EXPECT_EQ(dog->ns, "/v60");
  EXPECT_EQ(dog->limits.docks, std::vector<std::string>{"DOCK_1"});
  // 宣告的能力和節點原本的預設一樣：載入註冊表前後出價不變
  EXPECT_EQ(cbba_core::parseCapabilities(dog->capabilities), cbba_core::defaultCaps(cbba_core::AgentType::UGV));
  EXPECT_EQ(cbba_core::parseCapabilities(reg.robot(1)->capabilities),
    cbba_core::defaultCaps(cbba_core::AgentType::UAV));
  // 原文的 limits：max_altitude_m、max_flight_s、landing_pads
  ASSERT_TRUE(reg.robot(1)->limits.max_flight_s);
  EXPECT_DOUBLE_EQ(*reg.robot(1)->limits.max_flight_s, 900.0);
  EXPECT_EQ(reg.robot(2)->limits.landing_pads, std::vector<std::string>{"PAD_2"});
  EXPECT_EQ(reg.robot(200), nullptr);   // 閘道不是載具
  ASSERT_EQ(reg.gateways.size(), 1u);
  EXPECT_EQ(reg.gateways[0].id, 200);

  // 2-1b 第二節的動作清單：8 個可規劃（開放）、3 個系統、6 個平台內部、5 個未開放
  EXPECT_EQ(reg.actions.size(), 22u);
  for (const char * name : {"go_to", "patrol", "inspect", "track", "speak", "wait", "return_to_dock", "alert_operator"}) {
    const ActionEntry * a = reg.action(name);
    ASSERT_NE(a, nullptr) << name;
    EXPECT_EQ(a->cls, "plannable") << name;
    EXPECT_EQ(a->status, "open") << name;
    EXPECT_TRUE(a->cancelable) << name;
    EXPECT_NE(a->implementation("quadruped"), nullptr) << name;
  }
  EXPECT_EQ(reg.action("speak")->implementation("uav"), nullptr);   // 原文：無人機不支援（除非有喇叭酬載）
  EXPECT_EQ(reg.action("emergency_stop")->cls, "system");
  EXPECT_EQ(reg.action("takeoff")->cls, "internal");
  EXPECT_EQ(reg.action("use_tool")->status, "closed");

  // 原文的 inspect：mode、duration_s、standoff_m；能力依模式（any_of）；無人機 = hover + capture、要先在空中
  const ActionEntry * inspect = reg.action("inspect");
  ASSERT_NE(inspect->param("mode"), nullptr);
  EXPECT_EQ(inspect->param("mode")->values, (std::vector<std::string>{"rgb", "thermal", "zoom"}));
  EXPECT_EQ(inspect->param("mode")->default_value, "rgb");
  ASSERT_NE(inspect->param("duration_s"), nullptr);
  EXPECT_DOUBLE_EQ(*inspect->param("duration_s")->max, 60.0);
  EXPECT_NE(inspect->param("standoff_m"), nullptr);
  EXPECT_EQ(inspect->requires_any_of.size(), 3u);
  EXPECT_EQ(inspect->preconditions, std::vector<std::string>{"at_location"});
  EXPECT_EQ(inspect->effects, std::vector<std::string>{"has_observation"});
  EXPECT_DOUBLE_EQ(inspect->timeout_s, 90.0);
  EXPECT_EQ(inspect->risk, "low");
  const ActionImpl * uav = inspect->implementation("uav");
  ASSERT_NE(uav, nullptr);
  EXPECT_EQ(uav->sequence, (std::vector<std::string>{"hover", "capture"}));
  EXPECT_EQ(uav->extra_preconditions, std::vector<std::string>{"airborne"});
  EXPECT_EQ(inspect->implementation("quadruped")->interface, "/ptz_inspect");
  EXPECT_EQ(reg.common_failure_codes.size(), 5u);
  // 角色的里程碑（CB-4）：9 個（第二批加 sync_reached），bit 和 cbba_core/milestones.hpp 一樣（checkRegistry 檢查）
  ASSERT_EQ(reg.milestones.size(), 9u);
  EXPECT_EQ(reg.milestones[4].name, "target_localized");
  EXPECT_EQ(reg.milestones[4].kind, "event");

  // 物件：原文的 8 類＋我們的 fire；person 不從頭上飛過、狗 1.5 m／UAV 3 m；machine 依設備
  EXPECT_EQ(reg.object_classes.size(), 9u);
  const ObjectClass * person = reg.objectClass("person");
  ASSERT_NE(person, nullptr);
  EXPECT_DOUBLE_EQ(*person->safety_distance_m.at("uav"), 3.0);
  EXPECT_EQ(person->allowed_actions, (std::vector<std::string>{"inspect", "track", "speak"}));
  EXPECT_TRUE(reg.objectClass("machine")->safety_per_instance);
  EXPECT_FALSE(reg.objectClass("gauge")->safety_distance_m.at("uav").has_value());   // 原文的「—」

  // 事件：原文的 perimeter_intrusion、unknown＋我們的 4 種；只有火警的偵測器（apriltag_fire_detector）已經有
  EXPECT_EQ(reg.event_types.size(), 6u);
  const EventType * intrusion = reg.eventType("perimeter_intrusion");
  ASSERT_NE(intrusion, nullptr);
  EXPECT_EQ(intrusion->priority, 0);
  EXPECT_DOUBLE_EQ(intrusion->theta_on, 3.0);
  EXPECT_EQ(intrusion->zone_attr, "restricted");
  EXPECT_EQ(intrusion->default_rule, "perimeter_intrusion");
  EXPECT_TRUE(reg.eventType("unknown")->requires_approval);
  for (const DetectorEntry & d : reg.detectors) {
    EXPECT_EQ(d.status, d.name == "apriltag_fire" ? "available" : "planned") << d.name;
  }
}

TEST(Registry, RepoSiteConfigIsConsistent)
{
  // site-config/yard（2-1b 第五節）：設備的類別在 objects.yaml、狗的觀察姿態不在狗的禁入區、fleet.yaml 的起降點都在
  const Registry reg = loadRegistry(REGISTRY_DIR);
  const Site site = loadSite(SITE_DIR);
  const auto errors = checkSite(site, reg);
  EXPECT_TRUE(errors.empty()) << firstOf(errors);
  EXPECT_EQ(site.devices.size(), 20u);
  const Device * tb3 = site.device("machine:TB3");
  ASSERT_NE(tb3, nullptr);
  EXPECT_EQ(tb3->zone, "Z_HV_B");
  ASSERT_TRUE(tb3->obs_quadruped);
  EXPECT_FALSE(site.device("machine:valve_skid")->obs_uav);   // 鋼構棚下，下視相機看不到
  const Zone * hv = site.zone("Z_HV_B");
  ASSERT_NE(hv, nullptr);
  EXPECT_TRUE(hv->high_voltage);
  EXPECT_TRUE(hv->contains(-10.0, 0.0));
  EXPECT_FALSE(hv->contains(-10.0, 7.5));
  ASSERT_NE(site.dock("PAD_1"), nullptr);
  EXPECT_EQ(site.dock("PAD_1")->kind, "pad");
  EXPECT_EQ(site.pois.size(), 9u);
}

TEST(Registry, PaperExamplesLoadAsIs)
{
  // 2-1b 原文的註冊表範例（inspect）與事件類型範例（perimeter_intrusion、unknown）原樣能載入、沒有錯誤
  //（我們加的欄位 location、object、status、object_class、expires_s 都有預設）。inspect 的 objects 多一個 fire：
  // objects.yaml 的 fire（我們加的類別）允許 inspect，兩邊要一致
  TempRegistry tmp;
  const std::string actions = readFile(fs::path(REGISTRY_DIR) / "actions.yaml");
  const auto start = actions.find("  inspect:\n");
  const auto end = actions.find("  track:\n");
  ASSERT_NE(start, std::string::npos);
  std::string text = actions.substr(0, start) +
    "  inspect:\n"
    "    class: plannable\n"
    "    version: 1\n"
    "    params:\n"
    "      mode:        {type: enum, values: [rgb, thermal, zoom], default: rgb}\n"
    "      duration_s:  {type: float, min: 2, max: 60, default: 5}\n"
    "      standoff_m:  {type: float, min: 1.0, max: 20.0}\n"
    "    objects: [person, bag, machine, gauge, door, puddle, smoke_region, fire]\n"
    "    requires:\n"
    "      any_of:\n"
    "        - [rgb_camera]            # mode=rgb\n"
    "        - [thermal_camera]        # mode=thermal\n"
    "    preconditions: [at_location]\n"
    "    effects: [has_observation]\n"
    "    timeout_s: 90\n"
    "    cancelable: true\n"
    "    risk: low\n"
    "    implementations:\n"
    "      quadruped:\n"
    "        backend: ros2_action\n"
    "        interface: /{robot}/inspect\n"
    "        autonomy: in_development\n"
    "      uav:\n"
    "        backend: uav_adapter\n"
    "        sequence: [hover, capture]  # 對應 /MAV1/api/hover + 拍攝\n"
    "        extra_preconditions: [airborne]\n"
    "        autonomy: in_development\n" +
    actions.substr(end);
  tmp.write("actions.yaml", text);
  tmp.write("event_types.yaml",
    "schema_version: \"1.0\"\n"
    "detectors:\n"
    "  person_yolo_v1: {labels: [person], source_kinds: [fixed_camera], min_score: 0.5, status: planned}\n"
    "  vms_line_crossing: {labels: [line_crossing], source_kinds: [vms_analytics], min_score: 0.5, status: planned}\n"
    "event_types:\n"
    "  perimeter_intrusion:\n"
    "    category: template\n"
    "    detectors: [person_yolo_v1, vms_line_crossing]\n"
    "    trigger: {zone_attr: restricted, time_rule: off_hours}\n"
    "    accumulate: {lambda: 0.8, theta_on: 3.0, theta_off: 1.0, min_duration_s: 1.5}\n"
    "    associate: {dt_s: 5, r_m: 3}\n"
    "    vlm: {prompt_id: intrusion_v2, samples: 2, timeout_s: 5}\n"
    "    cost: {miss: 10, false: 1}          # 決定派遣門檻\n"
    "    locate: projection                  # projection / asset / zone\n"
    "    default_rule: perimeter_intrusion   # 對應 event_rules.yaml\n"
    "    priority: P0\n"
    "  unknown:\n"
    "    category: open\n"
    "    vlm: {prompt_id: open_event_planner_v1, timeout_s: 8}\n"
    "    requires_approval: true\n"
    "    priority: P2\n");
  const Registry reg = loadRegistry(tmp.dir());
  const auto errors = checkRegistry(reg);
  EXPECT_TRUE(errors.empty()) << firstOf(errors);
  const ActionEntry * inspect = reg.action("inspect");
  ASSERT_NE(inspect, nullptr);
  EXPECT_EQ(inspect->object, "optional");
  EXPECT_EQ(inspect->status, "open");
  EXPECT_EQ(reg.eventType("perimeter_intrusion")->status, "open");
  EXPECT_DOUBLE_EQ(reg.eventType("perimeter_intrusion")->cost_miss, 10.0);
}

TEST(Registry, BitMismatchWithCodeIsError)
{
  // 位元位置只增不改：註冊表的 bit 和程式不一樣 → 錯誤（節點拒絕啟動）
  TempRegistry tmp;
  tmp.write("tools.yaml",
    "schema_version: \"1.0\"\n"
    "capabilities:\n"
    "  - {name: mobility_ground, bit: 1}\n"
    "  - {name: mobility_air, bit: 0}\n");
  const auto errors = checkRegistry(loadRegistry(tmp.dir()));
  EXPECT_TRUE(hasError(errors, "mobility_ground 的 bit 是 1"));
  EXPECT_TRUE(hasError(errors, "少了程式認得的能力 rgb_camera"));
}

TEST(Registry, NewerRegistryToolIsNotAnError)
{
  // 註冊表比程式新：多一個程式不認得的標籤（新的 bit）不算錯，列在 unknownTools
  TempRegistry tmp;
  tmp.append("tools.yaml", "  - {name: lidar, bit: 13}\n");
  const Registry reg = loadRegistry(tmp.dir());
  EXPECT_TRUE(checkRegistry(reg).empty());
  EXPECT_EQ(unknownTools(reg), std::vector<std::string>{"lidar"});
}

TEST(Registry, FleetErrors)
{
  // 重複的機號（含閘道）、不認得的平台、不在 tools 的能力、deprecated 的能力、limits 不合理
  TempRegistry tmp;
  tmp.append("tools.yaml", "  - {name: old_sensor, bit: 20, deprecated: true}\n");
  tmp.write("fleet.yaml",
    "schema_version: \"1.0\"\n"
    "robots:\n"
    "  - {id: 1, name: a, platform: uav, capabilities: [mobility_air], limits: {max_altitude_m: 0}}\n"
    "  - {id: 1, name: b, platform: uav}\n"
    "  - {id: 2, name: c, platform: boat}\n"
    "  - {id: 3, name: d, platform: uav, capabilities: [x_ray, old_sensor]}\n"
    "gateways:\n"
    "  - {id: 3, name: gcs}\n");
  const auto errors = checkRegistry(loadRegistry(tmp.dir()));
  EXPECT_TRUE(hasError(errors, "機號 1 重複"));
  EXPECT_TRUE(hasError(errors, "平台 boat 不在 platforms.yaml"));
  EXPECT_TRUE(hasError(errors, "能力 x_ray 不在 tools.yaml"));
  EXPECT_TRUE(hasError(errors, "能力 old_sensor 已經 deprecated"));
  EXPECT_TRUE(hasError(errors, "閘道 gcs 的機號 3 和別台重複"));
  EXPECT_TRUE(hasError(errors, "a 的 limits"));
}

TEST(Registry, LoadErrorsThrow)
{
  // 缺檔、缺欄位、型別不對、機號超出範圍、schema 版本不對、優先級不是 P0～P3：丟例外（節點拒絕啟動）
  EXPECT_THROW(loadRegistry("/nonexistent_registry_dir"), std::runtime_error);
  {
    TempRegistry tmp;
    tmp.write("fleet.yaml", "schema_version: \"1.0\"\nrobots:\n  - {id: 1, platform: uav}\n");   // 缺 name
    EXPECT_THROW(loadRegistry(tmp.dir()), std::runtime_error);
  }
  {
    TempRegistry tmp;
    tmp.write("fleet.yaml", "schema_version: \"1.0\"\nrobots:\n  - {id: 300, name: a, platform: uav}\n");
    EXPECT_THROW(loadRegistry(tmp.dir()), std::runtime_error);
  }
  {
    TempRegistry tmp;
    tmp.write("tools.yaml", "schema_version: \"1.0\"\ncapabilities:\n  - {name: a, bit: many}\n");
    EXPECT_THROW(loadRegistry(tmp.dir()), std::runtime_error);
  }
  {
    TempRegistry tmp;
    tmp.write("actions.yaml", "schema_version: \"1.0\"\nactions:\n  - {name: go_to}\n");   // 舊格式（清單）
    EXPECT_THROW(loadRegistry(tmp.dir()), std::runtime_error);
  }
  {
    TempRegistry tmp;
    tmp.write("event_types.yaml",
      "schema_version: \"1.0\"\nevent_types:\n  x: {category: open, priority: 0}\n");   // priority 要寫 P0
    EXPECT_THROW(loadRegistry(tmp.dir()), std::runtime_error);
  }
  {
    TempRegistry tmp;
    tmp.write("platforms.yaml", "schema_version: \"2.0\"\nplatforms: []\n");
    try {
      loadRegistry(tmp.dir());
      FAIL() << "應該丟例外";
    } catch (const std::runtime_error & e) {
      EXPECT_NE(std::string(e.what()).find("platforms.yaml"), std::string::npos) << e.what();
      EXPECT_NE(std::string(e.what()).find("schema_version 是 2.0"), std::string::npos) << e.what();
    }
  }
}

TEST(Registry, ActionErrors)
{
  // 動作：class／status／location／risk／autonomy 不認得、能力、失敗碼、前置條件與效果不在 conditions、參數（type、enum、
  // 範圍、required 又有 default、重複）、不可取消、沒有平台實作、sequence 不是平台內部動作、objects 和物件的 allowed_actions 不一致
  TempRegistry tmp;
  tmp.write("actions.yaml",
    "schema_version: \"1.0\"\n"
    "common_failure_codes: [E_HW, E_OOPS]\n"
    "conditions:\n"
    "  - {name: at_location}\n"
    "  - {name: at_location}\n"
    "actions:\n"
    "  go_to:\n"
    "    class: magic\n"
    "    status: maybe\n"
    "    version: 0\n"
    "    location: somewhere\n"
    "    params:\n"
    "      a: {type: float, required: true, default: 1}\n"
    "      b: {type: complex}\n"
    "      c: {type: enum, default: x}\n"
    "      d: {type: enum, values: [p, q], default: r}\n"
    "      e: {type: float, min: 5, max: 1}\n"
    "      f: {type: int, min: 0, max: 10, default: 20}\n"
    "    objects: [person, ufo]\n"
    "    requires: {any_of: [[x_ray], []]}\n"
    "    preconditions: [is_happy]\n"
    "    effects: [at_location]\n"
    "    failure_codes: [E_NOPE]\n"
    "    timeout_s: -1\n"
    "    cancelable: false\n"
    "    risk: extreme\n"
    "    implementations:\n"
    "      boat: {backend: x, interface: y, autonomy: done}\n"
    "      uav: {backend: uav_adapter, sequence: [go_to], extra_effects: [flying], autonomy: in_development}\n"
    "  wait:\n"
    "    class: plannable\n"
    "    version: 1\n"
    "    timeout_s: 0\n"
    "    cancelable: true\n"
    "    risk: low\n"
    "    implementations:\n"
    "      quadruped: {backend: task_manager, autonomy: not_started}\n");
  const auto errors = checkRegistry(loadRegistry(tmp.dir()));
  EXPECT_TRUE(hasError(errors, "common_failure_codes 的 E_OOPS"));
  EXPECT_TRUE(hasError(errors, "condition at_location 重複"));
  EXPECT_TRUE(hasError(errors, "class magic"));
  EXPECT_TRUE(hasError(errors, "status maybe"));
  EXPECT_TRUE(hasError(errors, "version 要 ≥ 1"));
  EXPECT_TRUE(hasError(errors, "location somewhere"));
  EXPECT_TRUE(hasError(errors, "risk extreme"));
  EXPECT_TRUE(hasError(errors, "timeout_s 不能是負的"));
  EXPECT_TRUE(hasError(errors, "不可取消"));
  EXPECT_TRUE(hasError(errors, "能力 x_ray 不在 tools.yaml"));
  EXPECT_TRUE(hasError(errors, "any_of 有空的一組"));
  EXPECT_TRUE(hasError(errors, "precondition is_happy 不在 conditions"));
  EXPECT_TRUE(hasError(errors, "extra_effect flying 不在 conditions"));
  EXPECT_TRUE(hasError(errors, "失敗碼 E_NOPE"));
  EXPECT_TRUE(hasError(errors, "參數 a 是 required，不能有 default"));
  EXPECT_TRUE(hasError(errors, "type complex"));
  EXPECT_TRUE(hasError(errors, "參數 c 是 enum，要有 values"));
  EXPECT_TRUE(hasError(errors, "參數 d 的 default r 不在 values"));
  EXPECT_TRUE(hasError(errors, "參數 e 的 min > max"));
  EXPECT_TRUE(hasError(errors, "參數 f 的 default 20 超出範圍"));
  EXPECT_TRUE(hasError(errors, "boat 實作：平台不在 platforms.yaml"));
  EXPECT_TRUE(hasError(errors, "autonomy done"));
  EXPECT_TRUE(hasError(errors, "sequence 的 go_to 不是平台內部動作"));
  EXPECT_TRUE(hasError(errors, "wait 的 quadruped 實作：要有 interface 或 sequence"));
  EXPECT_TRUE(hasError(errors, "objects ufo 不在 objects.yaml"));
  EXPECT_TRUE(hasError(errors, "objects 有 person，但 objects.yaml 的 person 的 allowed_actions 沒有 go_to"));
  // 物件 person 允許 inspect、track、speak，但這份 actions.yaml 沒有
  EXPECT_TRUE(hasError(errors, "person 的 allowed_actions inspect 不在 actions.yaml"));
}

TEST(Registry, AddingEventTypeIsJustOneEntry)
{
  // 之後要加事件類型、偵測器、物件類別（我們或伯宇）：在表的後面加一筆就好，不用改程式；偵測器還沒有就標 planned
  TempRegistry tmp;
  std::string events = readFile(fs::path(REGISTRY_DIR) / "event_types.yaml");
  const auto at = events.find("event_types:\n");
  ASSERT_NE(at, std::string::npos);
  events.insert(at, "  gas_sensor_v1: {labels: [gas], source_kinds: [sensor], min_score: 0.5, status: planned}\n\n");
  events +=
    "  gas_leak:\n"
    "    category: template\n"
    "    detectors: [gas_sensor_v1]\n"
    "    locate: zone\n"
    "    default_rule: gas_leak_check\n"
    "    priority: P0\n";
  tmp.write("event_types.yaml", events);
  const Registry reg = loadRegistry(tmp.dir());
  const auto errors = checkRegistry(reg);
  EXPECT_TRUE(errors.empty()) << firstOf(errors);
  ASSERT_NE(reg.eventType("gas_leak"), nullptr);
  EXPECT_FALSE(reg.eventType("gas_leak")->has_vlm);   // 沒寫的用預設
  EXPECT_EQ(reg.eventType("gas_leak")->zone_attr, "any");
  EXPECT_EQ(reg.detectors.back().name, "gas_sensor_v1");
}

TEST(Registry, MilestoneMismatchIsError)
{
  // 里程碑的 bit、kind 和程式不一樣、少了程式認得的 → 錯誤
  TempRegistry tmp;
  std::string actions = readFile(fs::path(REGISTRY_DIR) / "actions.yaml");
  const auto replace = [&actions](const std::string & from, const std::string & to) {
      const auto at = actions.find(from);
      ASSERT_NE(at, std::string::npos) << from;
      actions.replace(at, from.size(), to);
    };
  replace("{name: arrived, bit: 2, kind: executor", "{name: arrived, bit: 9, kind: executor");
  replace("{name: has_observation, bit: 6, kind: event", "{name: has_observation, bit: 6, kind: executor");
  replace("  - {name: completed, bit: 7", "  - {name: done, bit: 7");
  tmp.write("actions.yaml", actions);
  const auto errors = checkRegistry(loadRegistry(tmp.dir()));
  EXPECT_TRUE(hasError(errors, "里程碑 arrived 的 bit 是 9"));
  EXPECT_TRUE(hasError(errors, "里程碑 has_observation 的 kind 是 executor"));
  EXPECT_TRUE(hasError(errors, "少了程式認得的里程碑 completed"));
}

TEST(Registry, ObjectAndEventErrors)
{
  TempRegistry tmp;
  tmp.write("objects.yaml",
    "schema_version: \"1.0\"\n"
    "classes:\n"
    "  person:\n"
    "    locate: [telepathy]\n"
    "    allowed_actions: [inspect, track, speak, fly_away]\n"
    "    safety_distance_m: {quadruped: -1, boat: 2}\n"
    "    rules: [be_nice]\n"
    "    findings: [poem]\n");
  tmp.write("event_types.yaml",
    "schema_version: \"1.0\"\n"
    "detectors:\n"
    "  d: {labels: [], source_kinds: [satellite], min_score: 2, status: soon}\n"
    "event_types:\n"
    "  x:\n"
    "    category: magic\n"
    "    detectors: [nope]\n"
    "    trigger: {zone_attr: moon, time_rule: sometimes}\n"
    "    accumulate: {lambda: 1.5, theta_on: 1, theta_off: 2}\n"
    "    associate: {dt_s: 0, r_m: -1}\n"
    "    vlm: {prompt_id: \"\", samples: 0, timeout_s: 0}\n"
    "    cost: {miss: 0, false: 1}\n"
    "    locate: radar\n"
    "    priority: P1\n"
    "    object_class: ufo\n"
    "    expires_s: -1\n"
    "    status: maybe\n"
    "  y: {category: template, priority: P0}\n"
    "  z: {category: open, priority: P3}\n");
  const auto errors = checkRegistry(loadRegistry(tmp.dir()));
  EXPECT_TRUE(hasError(errors, "locate telepathy"));
  EXPECT_TRUE(hasError(errors, "allowed_actions fly_away 不在 actions.yaml"));
  EXPECT_TRUE(hasError(errors, "safety_distance_m 的平台 boat"));
  EXPECT_TRUE(hasError(errors, "safety_distance_m.quadruped 不能是負的"));
  EXPECT_TRUE(hasError(errors, "rule be_nice"));
  EXPECT_TRUE(hasError(errors, "findings poem"));
  // actions.yaml 的 inspect 的 objects 有 bag…，但這份 objects.yaml 沒有
  EXPECT_TRUE(hasError(errors, "objects bag 不在 objects.yaml"));
  EXPECT_TRUE(hasError(errors, "偵測器 d 沒有 labels"));
  EXPECT_TRUE(hasError(errors, "source_kinds satellite"));
  EXPECT_TRUE(hasError(errors, "偵測器 d 的 min_score 要在 0～1"));
  EXPECT_TRUE(hasError(errors, "偵測器 d 的 status"));
  EXPECT_TRUE(hasError(errors, "category magic"));
  EXPECT_TRUE(hasError(errors, "偵測器 nope 不在 detectors"));
  EXPECT_TRUE(hasError(errors, "zone_attr moon"));
  EXPECT_TRUE(hasError(errors, "time_rule sometimes"));
  EXPECT_TRUE(hasError(errors, "accumulate 要 0 < lambda"));
  EXPECT_TRUE(hasError(errors, "associate 要 dt_s > 0"));
  EXPECT_TRUE(hasError(errors, "vlm 要有 prompt_id"));
  EXPECT_TRUE(hasError(errors, "cost（miss、false）要 > 0"));
  EXPECT_TRUE(hasError(errors, "locate radar"));
  EXPECT_TRUE(hasError(errors, "object_class ufo 不在 objects.yaml"));
  EXPECT_TRUE(hasError(errors, "expires_s 不能是負的"));
  EXPECT_TRUE(hasError(errors, "x 的 status 要是 open／closed"));
  EXPECT_TRUE(hasError(errors, "y 是 template 事件，要有 detectors"));
  EXPECT_TRUE(hasError(errors, "y 是 template 事件，要有 default_rule"));
  EXPECT_TRUE(hasError(errors, "z 是 open 事件，要有 vlm 而且 requires_approval: true"));
}

TEST(Registry, SiteErrors)
{
  // 場域設定：id 格式與重複、類別不在 objects、per_instance 沒填安全距離、zone 不存在、狗的觀察點在狗的禁入區、
  // UAV 的觀察點高過天花板、dock 的 kind、fleet.yaml 指到不在或不對的起降點
  TempRegistry tmp;
  tmp.append("site/devices.csv",
    "TA9,machine,0,0,1,1F,Z_HV_A,1.5,3.0,,,,,,,沒有類別前綴\n"
    "bag:B1,machine,0,40,1,1F,,1.0,,,,,,,,前綴和類別不同\n"
    "ufo:U1,ufo,0,40,1,1F,Z_NOWHERE,,,,,,,,,不認得的類別\n"
    "machine:TX,machine,0,40,1,1F,,,,,,,,,,沒填安全距離\n"
    "machine:TA1,machine,-30,15,1.5,1F,Z_HV_A,1.5,3.0,-30,15,90,,,,重複、狗站在高壓區裡\n");
  tmp.append("site/pois.csv",
    "OBS_BAD,Z_HV_A,front,quadruped,-10.0,15.0,0.0,90,1F\n"
    "OBS_HIGH,Z_INDOOR,overwatch,uav,100.0,100.0,5.0,,1F\n");
  tmp.append("site/docks.csv", "PAD_9,helipad,uav,-40.0,-35.0,0.0,1F\n");
  std::string zones = readFile(fs::path(SITE_DIR) / "zones.geojson");
  const auto last = zones.rfind(']');
  ASSERT_NE(last, std::string::npos);
  zones.insert(last,
    ",{\"type\": \"Feature\", \"properties\": {\"id\": \"Z_INDOOR\", \"ceiling_m\": 3.0, \"no_entry\": [\"boat\"]},"
    " \"geometry\": {\"type\": \"Polygon\", \"coordinates\": [[[90, 90], [110, 90], [110, 110], [90, 110], [90, 90]]]}}");
  tmp.write("site/zones.geojson", zones);
  std::string fleet = readFile(fs::path(REGISTRY_DIR) / "fleet.yaml");
  const auto pad = fleet.find("landing_pads: [PAD_1]");
  ASSERT_NE(pad, std::string::npos);
  fleet.replace(pad, std::string("landing_pads: [PAD_1]").size(), "landing_pads: [PAD_7, DOCK_1]");
  tmp.write("fleet.yaml", fleet);
  const Registry reg = loadRegistry(tmp.dir());
  const auto errors = checkSite(loadSite(tmp.site()), reg);
  EXPECT_TRUE(hasError(errors, "TA9 的 id 要是「類別:實例」"));
  EXPECT_TRUE(hasError(errors, "bag:B1 的 id 前綴和 class machine 不一樣"));
  EXPECT_TRUE(hasError(errors, "ufo:U1 的 class ufo 不在 objects.yaml"));
  EXPECT_TRUE(hasError(errors, "ufo:U1 的 zone Z_NOWHERE 不在 zones.geojson"));
  EXPECT_TRUE(hasError(errors, "machine:TX：machine 的安全距離是 per_instance"));
  EXPECT_TRUE(hasError(errors, "machine:TA1 重複"));
  EXPECT_TRUE(hasError(errors, "machine:TA1 的狗觀察姿態在狗的禁入區 Z_HV_A"));
  EXPECT_TRUE(hasError(errors, "OBS_BAD 在 quadruped 的禁入區 Z_HV_A"));
  EXPECT_TRUE(hasError(errors, "OBS_HIGH 的高度 5 m 超過 Z_INDOOR 的天花板 3 m"));
  EXPECT_TRUE(hasError(errors, "Z_INDOOR 的 no_entry 平台 boat"));
  EXPECT_TRUE(hasError(errors, "PAD_9 的 kind helipad"));
  EXPECT_TRUE(hasError(errors, "uav1 的 landing_pads PAD_7 不在 docks.csv"));
  EXPECT_TRUE(hasError(errors, "uav1 的 landing_pads DOCK_1 是 quadruped 的 dock"));

  // 格式錯誤：欄數不對、數字欄不是數字 → 丟例外
  TempRegistry bad;
  bad.append("site/docks.csv", "PAD_X,pad,uav,-40.0\n");
  EXPECT_THROW(loadSite(bad.site()), std::runtime_error);
  bad.write("site/docks.csv", "id,kind,platform,x,y,z,floor\nPAD_X,pad,uav,abc,0,0,1F\n");
  EXPECT_THROW(loadSite(bad.site()), std::runtime_error);
}

TEST(Registry, SemanticIds)
{
  // 任務語意的編號（任務欄位補齊 02，2026-11-17）：repo 的 event_types、task_types、roles 都有編號、不重複；
  // 重複的 id、0、缺 id 是錯誤；事件類型可以不寫 id（編號 0）
  const Registry reg = loadRegistry(REGISTRY_DIR);
  EXPECT_TRUE(checkRegistry(reg).empty()) << firstOf(checkRegistry(reg));
  ASSERT_NE(reg.eventType("perimeter_intrusion"), nullptr);
  EXPECT_EQ(reg.eventType("perimeter_intrusion")->id, 1);
  ASSERT_NE(reg.taskType("confirm_intrusion"), nullptr);
  EXPECT_EQ(reg.taskType("confirm_intrusion")->id, 4);
  ASSERT_NE(reg.role("front"), nullptr);
  EXPECT_EQ(reg.role("front")->id, 2);
  EXPECT_EQ(reg.role("nobody"), nullptr);
  for (const EventType & e : reg.event_types) {
    EXPECT_GT(e.id, 0) << e.name;
  }

  TempRegistry dup;
  dup.append("task_types.yaml", "  hover_watch: {id: 4}\n");   // 接在 roles 後面：roles 的 id 4 重複
  EXPECT_TRUE(hasError(checkRegistry(loadRegistry(dup.dir())), "roles 的 id 4")) <<
    firstOf(checkRegistry(loadRegistry(dup.dir())));

  TempRegistry zero;
  zero.append("task_types.yaml", "  hover_watch: {id: 0}\n");
  EXPECT_THROW(loadRegistry(zero.dir()), std::runtime_error);
  TempRegistry missing;
  missing.append("task_types.yaml", "  hover_watch: {description: x}\n");
  EXPECT_THROW(loadRegistry(missing.dir()), std::runtime_error);

  TempRegistry events;
  std::string text = readFile(fs::path(REGISTRY_DIR) / "event_types.yaml");
  const auto at = text.find("  smoke:\n    id: 4\n");
  ASSERT_NE(at, std::string::npos);
  text.replace(at, std::string("  smoke:\n    id: 4\n").size(), "  smoke:\n    id: 3\n");   // 和 fire 重複
  events.write("event_types.yaml", text);
  EXPECT_TRUE(hasError(checkRegistry(loadRegistry(events.dir())), "id 3 同時給了 fire 和 smoke"));
}
