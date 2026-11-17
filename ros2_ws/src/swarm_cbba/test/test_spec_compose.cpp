// 角色 TaskSpec 的組合（任務欄位補齊第一批，2026-11-17）：閘道把識別、語意、空間寫進規劃器的 spec 最上層
#include <gtest/gtest.h>

#include <string>

#include <yaml-cpp/yaml.h>

#include "swarm_cbba/spec_compose.hpp"

using namespace swarm_cbba;
using cbba_core::TargetKind;
using cbba_core::TaskMeta;
using cbba_core::TaskSource;

namespace
{
TaskMeta meta()
{
  TaskMeta m;
  m.incident_id = "ev_0042";
  m.source = TaskSource::VMS;
  m.event_type = "perimeter_intrusion";
  m.event_type_id = 1;
  m.task_type = "confirm_intrusion";
  m.task_type_id = 4;
  m.role = "front";
  m.role_id = 2;
  m.target_kind = TargetKind::MOVING;
  m.target = {10.0, 0.0, 0.0};
  m.track_id = "trk_88";
  m.standoff_m = 3.0;
  return m;
}
}  // namespace

TEST(SpecCompose, InsertsMetaAndKeepsSteps)
{
  // 組好的是合法 JSON：最上層有 incident、來源、語意、觀察點、目標，規劃器的 steps 原封不動
  const std::string planner = "{\"steps\":[{\"action\":\"go_to\"},{\"action\":\"inspect\",\"object\":\"person:trk_88\"}]}";
  std::string error;
  const auto spec = composeSpec(planner, meta(), {6.0, 0.0, 0.0}, 1, &error);
  ASSERT_TRUE(spec) << error;
  const YAML::Node root = YAML::Load(*spec);
  EXPECT_EQ(root["schema_version"].as<std::string>(), "1.1");
  EXPECT_EQ(root["incident_id"].as<std::string>(), "ev_0042");
  EXPECT_EQ(root["source"].as<std::string>(), "VMS");
  EXPECT_EQ(root["source_id"].as<int>(), 4);
  EXPECT_EQ(root["event_type_id"].as<int>(), 1);
  EXPECT_EQ(root["task_type"].as<std::string>(), "confirm_intrusion");
  EXPECT_EQ(root["role"].as<std::string>(), "front");
  EXPECT_EQ(root["role_id"].as<int>(), 2);
  EXPECT_EQ(root["target_kind"].as<std::string>(), "moving");
  EXPECT_EQ(root["track_id"].as<std::string>(), "trk_88");
  EXPECT_DOUBLE_EQ(root["observe_pose"]["x"].as<double>(), 6.0);
  EXPECT_DOUBLE_EQ(root["target"]["x"].as<double>(), 10.0);
  EXPECT_EQ(root["steps"].size(), 2u);
  EXPECT_EQ(root["steps"][1]["object"].as<std::string>(), "person:trk_88");
  EXPECT_NE(spec->find(planner.substr(1)), std::string::npos);   // 規劃器的內容一字不改
  EXPECT_EQ(specSchemaVersion(*spec), "1.1");
}

TEST(SpecCompose, EmptySpecStaysEmptyAndEmptyObjectWorks)
{
  // 規劃器沒給 spec：照舊沒有 TaskSpec；給了空物件：只有 meta（沒有多出來的逗號）
  EXPECT_EQ(composeSpec("", meta(), {}, 0).value_or("x"), "");
  const auto spec = composeSpec(" { } ", meta(), {6.0, 0.0, 0.0}, 0);
  ASSERT_TRUE(spec);
  EXPECT_NO_THROW(YAML::Load(*spec));
  EXPECT_EQ(YAML::Load(*spec)["incident_id"].as<std::string>(), "ev_0042");
}

TEST(SpecCompose, SameRoleAndSchemaAreKeptOnce)
{
  // 規劃器自己寫了一樣的 role、schema_version（itest 的假 Planner 會寫 role）：照收、不重複
  std::string error;
  const auto spec = composeSpec("{\"schema_version\":\"1.1\",\"template\":\"t\",\"role\":\"front\",\"steps\":[]}", meta(),
    {6.0, 0.0, 0.0}, 0, &error);
  ASSERT_TRUE(spec) << error;
  EXPECT_EQ(spec->find("\"role\":\"front\""), spec->rfind("\"role\":\"front\""));
  EXPECT_EQ(spec->find("\"schema_version\""), spec->rfind("\"schema_version\""));
  EXPECT_EQ(YAML::Load(*spec)["role_id"].as<int>(), 2);
}

TEST(SpecCompose, RejectsConflictsAndNonObjects)
{
  // 規劃器帶了閘道的鍵（不一樣的 role、incident_id、target…）、不是 JSON 物件 → 拒絕
  std::string error;
  EXPECT_FALSE(composeSpec("{\"role\":\"rear\",\"steps\":[]}", meta(), {}, 0, &error));
  EXPECT_NE(error.find("role"), std::string::npos);
  EXPECT_FALSE(composeSpec("{\"incident_id\":\"other\"}", meta(), {}, 0, &error));
  EXPECT_FALSE(composeSpec("{\"target\":{\"x\":1}}", meta(), {}, 0, &error));
  EXPECT_FALSE(composeSpec("{\"schema_version\":\"2.0\"}", meta(), {}, 0, &error));
  EXPECT_FALSE(composeSpec("[1, 2]", meta(), {}, 0, &error));
  EXPECT_FALSE(composeSpec("steps: []", meta(), {}, 0, &error));   // YAML，不是 JSON 物件
  EXPECT_FALSE(composeSpec("{\"steps\": [", meta(), {}, 0, &error));
  EXPECT_EQ(specSchemaVersion("{\"steps\":[]}"), "");
  EXPECT_EQ(specSchemaVersion("not json {"), "");
}
