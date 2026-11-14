// 2-1b 註冊表（schema 1.0，2026-11-14）：repo 的 registry/ 載入、和 capabilities.hpp 一致；
// 不合法的註冊表（bit 不一致、重複、不認得的能力、缺欄位、schema 版本、動作的欄位）被擋下
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
// 暫存目錄：先複製 repo 的註冊表，再改其中一份
class TempRegistry
{
public:
  TempRegistry()
  : dir_(fs::temp_directory_path() / ("cbba_registry_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
      "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name()))
  {
    fs::remove_all(dir_);
    fs::create_directories(dir_);
    for (const char * f : {"tools.yaml", "platforms.yaml", "fleet.yaml", "actions.yaml", "objects.yaml",
        "event_types.yaml"})
    {
      fs::copy_file(fs::path(REGISTRY_DIR) / f, dir_ / f);
    }
  }
  ~TempRegistry() {fs::remove_all(dir_);}

  void write(const std::string & file, const std::string & text) const
  {
    std::ofstream(dir_ / file) << text;
  }
  std::string dir() const {return dir_.string();}

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
}  // namespace

TEST(Registry, RepoRegistryLoadsAndMatchesCode)
{
  // repo 的 registry/：沒有錯誤；每個能力的 bit 和 capabilities.hpp 一樣；機隊照模擬的 docker（uav1～3、狗 50～52、閘道 200）
  const Registry reg = loadRegistry(REGISTRY_DIR);
  const auto errors = checkRegistry(reg);
  EXPECT_TRUE(errors.empty()) << errors.front();
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
  // 宣告的能力和節點原本的預設一樣：載入註冊表前後出價不變
  EXPECT_EQ(cbba_core::parseCapabilities(dog->capabilities), cbba_core::defaultCaps(cbba_core::AgentType::UGV));
  EXPECT_EQ(cbba_core::parseCapabilities(reg.robot(1)->capabilities),
    cbba_core::defaultCaps(cbba_core::AgentType::UAV));
  EXPECT_EQ(reg.robot(200), nullptr);   // 閘道不是載具
  ASSERT_EQ(reg.gateways.size(), 1u);
  EXPECT_EQ(reg.gateways[0].id, 200);

  // 2-1b 第二節的 8 個可規劃動作，狗和無人機都能做
  ASSERT_EQ(reg.actions.size(), 8u);
  for (const char * name : {"go_to", "patrol", "inspect", "track", "speak", "wait", "return_to_dock", "alert_operator"}) {
    const ActionEntry * a = reg.action(name);
    ASSERT_NE(a, nullptr) << name;
    EXPECT_EQ(a->cls, "plannable") << name;
    EXPECT_EQ(a->platforms, (std::vector<std::string>{"uav", "quadruped"})) << name;
    EXPECT_TRUE(a->cancellable) << name;
  }
  // 狗的 BT 已經在用的參數名稱（task_manager_spec.patch）：inspect 的 snapshots（預設 1）、timeout
  const ActionEntry * inspect = reg.action("inspect");
  ASSERT_EQ(inspect->params.size(), 2u);
  EXPECT_EQ(inspect->params[0].name, "snapshots");
  EXPECT_EQ(inspect->params[0].default_value, "1");
  EXPECT_EQ(inspect->params[1].name, "timeout");
  EXPECT_EQ(inspect->location, "required");
  EXPECT_EQ(reg.action("speak")->required_capabilities, std::vector<std::string>{"speaker"});
  EXPECT_TRUE(reg.action("wait")->params[0].required);
  EXPECT_FALSE(reg.action("wait")->params[0].has_default);
  EXPECT_EQ(reg.common_failure_codes.size(), 5u);

  // objects：12 種、yard 的 21 個設備
  EXPECT_EQ(reg.object_classes.size(), 12u);
  EXPECT_EQ(reg.object_instances.size(), 21u);
  // event_types：先自訂的 5 種；只有火警的偵測器（apriltag_fire_detector）已經有
  ASSERT_EQ(reg.event_types.size(), 5u);
  const EventType & fire = reg.event_types.front();
  EXPECT_EQ(fire.name, "fire");
  EXPECT_EQ(fire.priority, 0);
  EXPECT_EQ(fire.detectors.front().detector, "apriltag_fire");
  EXPECT_EQ(fire.detectors.front().status, "available");
  EXPECT_DOUBLE_EQ(fire.detectors.front().min_score, 0.85);
  for (const EventType & e : reg.event_types) {
    if (e.name != "fire") {
      for (const EventDetector & d : e.detectors) {
        EXPECT_EQ(d.status, "planned") << e.name;
      }
    }
  }
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
  std::ifstream in(fs::path(REGISTRY_DIR) / "tools.yaml");
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  tmp.write("tools.yaml", text + "  - {name: lidar, bit: 13}\n");
  const Registry reg = loadRegistry(tmp.dir());
  EXPECT_TRUE(checkRegistry(reg).empty());
  EXPECT_EQ(unknownTools(reg), std::vector<std::string>{"lidar"});
}

TEST(Registry, FleetErrors)
{
  // 重複的機號（含閘道）、不認得的平台、不在 tools 的能力、deprecated 的能力
  TempRegistry tmp;
  std::ifstream in(fs::path(REGISTRY_DIR) / "tools.yaml");
  std::string tools((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  tmp.write("tools.yaml", tools + "  - {name: old_sensor, bit: 20, deprecated: true}\n");
  tmp.write("fleet.yaml",
    "schema_version: \"1.0\"\n"
    "robots:\n"
    "  - {id: 1, name: a, platform: uav, capabilities: [mobility_air]}\n"
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
}

TEST(Registry, LoadErrorsThrow)
{
  // 缺檔、缺欄位、型別不對、機號超出範圍、schema 版本不對：丟例外（節點拒絕啟動）
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
  // 動作：重複、不認得的 class／location、平台、能力、失敗碼、參數 type、required 又有 default、不可取消、不認得的 precondition
  TempRegistry tmp;
  tmp.write("actions.yaml",
    "schema_version: \"1.0\"\n"
    "common_failure_codes: [E_HW, E_OOPS]\n"
    "actions:\n"
    "  - {name: go_to, class: plannable, platforms: [uav], location: required, object: none, cancellable: true,"
    " status: open}\n"
    "  - {name: go_to, class: magic, platforms: [boat], required_capabilities: [x_ray], location: maybe, object: none,"
    " failure_codes: [E_NOPE], preconditions: [is_happy], cancellable: false, status: open,"
    " params: [{name: a, type: float, required: true, default: 1}, {name: a, type: complex}]}\n");
  const auto errors = checkRegistry(loadRegistry(tmp.dir()));
  EXPECT_TRUE(hasError(errors, "common_failure_codes 的 E_OOPS"));
  EXPECT_TRUE(hasError(errors, "actions.yaml：go_to 重複"));
  EXPECT_TRUE(hasError(errors, "class magic"));
  EXPECT_TRUE(hasError(errors, "location maybe"));
  EXPECT_TRUE(hasError(errors, "平台 boat 不在 platforms.yaml"));
  EXPECT_TRUE(hasError(errors, "能力 x_ray 不在 tools.yaml"));
  EXPECT_TRUE(hasError(errors, "失敗碼 E_NOPE"));
  EXPECT_TRUE(hasError(errors, "precondition is_happy"));
  EXPECT_TRUE(hasError(errors, "不可取消"));
  EXPECT_TRUE(hasError(errors, "參數 a 重複"));
  EXPECT_TRUE(hasError(errors, "type complex"));
  EXPECT_TRUE(hasError(errors, "參數 a 是 required，不能有 default"));

  // 缺欄位（class）→ 載入時就丟例外
  tmp.write("actions.yaml", "schema_version: \"1.0\"\nactions:\n  - {name: go_to, platforms: [uav]}\n");
  EXPECT_THROW(loadRegistry(tmp.dir()), std::runtime_error);
}

namespace
{
std::string readFile(const fs::path & p)
{
  std::ifstream in(p);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}
}  // namespace

TEST(Registry, AddingEventTypeAndObjectIsJustOneEntry)
{
  // 之後要加事件種類（我們或伯宇）：在清單後面加一筆就好，不用改程式；偵測器還沒有就標 planned
  TempRegistry tmp;
  tmp.write("objects.yaml", readFile(fs::path(REGISTRY_DIR) / "objects.yaml") +
    "  - {id: gate_1, class: door, site: yard, position: [0.0, -40.0, 1.0]}\n");
  tmp.write("event_types.yaml", readFile(fs::path(REGISTRY_DIR) / "event_types.yaml") +
    "  - name: gas_leak\n"
    "    priority: 0\n"
    "    object_class: valve\n"
    "    detectors: [{detector: gas_sensor, labels: [gas], min_score: 0.5, status: planned}]\n"
    "    trigger: {min_count: 2, window: 5.0}\n"
    "    template: gas_leak_check\n"
    "    status: open\n");
  const Registry reg = loadRegistry(tmp.dir());
  const auto errors = checkRegistry(reg);
  EXPECT_TRUE(errors.empty()) << errors.front();
  EXPECT_EQ(reg.event_types.size(), 6u);
  EXPECT_EQ(reg.event_types.back().name, "gas_leak");
  EXPECT_FALSE(reg.event_types.back().vlm_enabled);   // 沒寫的用預設
  EXPECT_EQ(reg.object_instances.back().id, "gate_1");
}

TEST(Registry, ObjectAndEventErrors)
{
  TempRegistry tmp;
  tmp.write("objects.yaml",
    "schema_version: \"1.0\"\n"
    "classes:\n"
    "  - {name: person, category: agent}\n"
    "  - {name: person, category: alien}\n"
    "instances:\n"
    "  - {id: a, class: person, position: [0, 0, 0]}\n"
    "  - {id: a, class: ghost, position: [0, 0]}\n");
  tmp.write("event_types.yaml",
    "schema_version: \"1.0\"\n"
    "event_types:\n"
    "  - {name: x, priority: 5, object_class: ufo, detectors: [], trigger: {min_count: 0, window: 0},"
    " vlm: {enabled: true}, dispatch_cost: -1, template: \"\", expires: -1, status: maybe}\n"
    "  - {name: x, priority: 0, object_class: person, trigger: {min_count: 1, window: 1}, template: t, status: open,"
    " detectors: [{detector: d, labels: [], min_score: 2, status: soon}]}\n");
  const auto errors = checkRegistry(loadRegistry(tmp.dir()));
  EXPECT_TRUE(hasError(errors, "種類 person 重複"));
  EXPECT_TRUE(hasError(errors, "category alien"));
  EXPECT_TRUE(hasError(errors, "設備 a 重複"));
  EXPECT_TRUE(hasError(errors, "種類 ghost 不在 classes"));
  EXPECT_TRUE(hasError(errors, "position 要是 [x, y, z]"));
  EXPECT_TRUE(hasError(errors, "event_types.yaml：x 重複"));
  EXPECT_TRUE(hasError(errors, "priority 要是 0～3"));
  EXPECT_TRUE(hasError(errors, "object_class ufo 不在 objects.yaml"));
  EXPECT_TRUE(hasError(errors, "沒有偵測器"));
  EXPECT_TRUE(hasError(errors, "trigger 要 min_count ≥ 1"));
  EXPECT_TRUE(hasError(errors, "VLM 開著但沒有 prompt"));
  EXPECT_TRUE(hasError(errors, "dispatch_cost 不能是負的"));
  EXPECT_TRUE(hasError(errors, "沒有 template"));
  EXPECT_TRUE(hasError(errors, "expires 不能是負的"));
  EXPECT_TRUE(hasError(errors, "status 要是 open／closed"));
  EXPECT_TRUE(hasError(errors, "偵測器 d 沒有 labels"));
  EXPECT_TRUE(hasError(errors, "min_score 要在 0～1"));
  EXPECT_TRUE(hasError(errors, "偵測器 d 的 status"));
}
