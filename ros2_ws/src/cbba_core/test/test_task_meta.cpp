// 任務的識別、語意與空間（任務欄位補齊第一批，2026-11-17）：欄位檢查、角色 TaskSpec 最上層的 JSON、schema 版本
#include <gtest/gtest.h>

#include <string>

#include "cbba_core/task_meta.hpp"

using namespace cbba_core;

namespace
{
TaskMeta pointMeta()
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
  m.description = "前方確認";
  m.target_kind = TargetKind::POINT;
  m.target = {10.0, 0.0, 0.0};
  m.standoff_m = 3.0;
  return m;
}
}  // namespace

TEST(TaskMeta, SourceAndTargetKindNames)
{
  // 來源的列舉是固定的（只增不改）：0 UNKNOWN … 6 SENSOR
  EXPECT_STREQ(toString(TaskSource::UNKNOWN), "UNKNOWN");
  EXPECT_STREQ(toString(TaskSource::ALARM), "ALARM");
  EXPECT_STREQ(toString(TaskSource::UAV), "UAV");
  EXPECT_STREQ(toString(TaskSource::QUADRUPED), "QUADRUPED");
  EXPECT_STREQ(toString(TaskSource::VMS), "VMS");
  EXPECT_STREQ(toString(TaskSource::OPERATOR), "OPERATOR");
  EXPECT_STREQ(toString(TaskSource::SENSOR), "SENSOR");
  EXPECT_EQ(static_cast<int>(TaskSource::SENSOR), 6);
  EXPECT_STREQ(toString(TargetKind::POINT), "point");
  EXPECT_STREQ(toString(TargetKind::AREA), "area");
  EXPECT_STREQ(toString(TargetKind::MOVING), "moving");
}

TEST(TaskMeta, StandoffIsCheckedAgainstObservePose)
{
  // 觀察點離目標要 ≥ standoff_m（容許 5 cm 的數值誤差）；area 從區域邊緣算
  TaskMeta m = pointMeta();
  EXPECT_EQ(checkMeta(m, {6.0, 0.0, 0.0}), "");            // 4 m
  EXPECT_EQ(checkMeta(m, {7.0, 0.0, 0.0}), "");            // 剛好 3 m
  EXPECT_EQ(checkMeta(m, {7.04, 0.0, 0.0}), "");           // 誤差內
  EXPECT_NE(checkMeta(m, {8.0, 0.0, 0.0}), "");            // 2 m
  EXPECT_NE(checkMeta(m, {10.0, 0.0, 5.0}), "");           // 正上方（水平 0 m）
  m.target_kind = TargetKind::AREA;
  m.uncertainty_m = 4.0;
  EXPECT_NEAR(standoffDistance(m, {3.0, 0.0, 0.0}), 3.0, 1e-9);
  EXPECT_EQ(checkMeta(m, {3.0, 0.0, 0.0}), "");            // 邊緣外 3 m
  EXPECT_NE(checkMeta(m, {4.0, 0.0, 0.0}), "");            // 邊緣外 2 m
  m.standoff_m = 0.0;
  EXPECT_EQ(checkMeta(m, {10.0, 0.0, 0.0}), "");           // 不要求安全距離
}

TEST(TaskMeta, KindSpecificFields)
{
  // moving 要 track_id、area 要半徑；沒有目標（NONE）時不能給安全距離、半徑、track；長度、範圍
  TaskMeta m = pointMeta();
  m.target_kind = TargetKind::MOVING;
  EXPECT_NE(checkMeta(m, {0.0, 0.0, 0.0}), "");
  m.track_id = "trk_88";
  EXPECT_EQ(checkMeta(m, {0.0, 0.0, 0.0}), "");
  m.track_id = std::string(65, 'x');
  EXPECT_NE(checkMeta(m, {0.0, 0.0, 0.0}), "");

  TaskMeta area = pointMeta();
  area.target_kind = TargetKind::AREA;
  EXPECT_NE(checkMeta(area, {0.0, 0.0, 0.0}), "");

  TaskMeta none;
  EXPECT_EQ(checkMeta(none, {0.0, 0.0, 0.0}), "");
  none.standoff_m = 1.0;
  EXPECT_NE(checkMeta(none, {0.0, 0.0, 0.0}), "");

  TaskMeta bad = pointMeta();
  bad.target_kind = static_cast<TargetKind>(4);
  EXPECT_NE(checkMeta(bad, {0.0, 0.0, 0.0}), "");
  bad = pointMeta();
  bad.source = static_cast<TaskSource>(7);
  EXPECT_NE(checkMeta(bad, {0.0, 0.0, 0.0}), "");
  bad = pointMeta();
  bad.description = std::string(201, 'd');
  EXPECT_NE(checkMeta(bad, {0.0, 0.0, 0.0}), "");
  bad = pointMeta();
  bad.uncertainty_m = -1.0;
  EXPECT_NE(checkMeta(bad, {0.0, 0.0, 0.0}), "");
}

TEST(TaskMeta, JsonMembersCarryNamesIdsAndPoses)
{
  // 角色 TaskSpec 最上層：schema_version、incident、來源（名稱＋編號）、事件／任務／角色（名稱＋編號）、說明、
  // 觀察點（＝ TASK_ANNOUNCE 的位置）與目標
  const std::string j = metaJsonMembers(pointMeta(), {6.0, 0.0, 0.0}, 1);
  for (const char * part : {
      "\"schema_version\":\"1.1\"", "\"incident_id\":\"ev_0042\"", "\"source\":\"VMS\"", "\"source_id\":4",
      "\"event_type\":\"perimeter_intrusion\"", "\"event_type_id\":1", "\"task_type\":\"confirm_intrusion\"",
      "\"task_type_id\":4", "\"role\":\"front\"", "\"role_id\":2", "\"description\":\"前方確認\"",
      "\"observe_pose\":{\"frame\":\"map\",\"floor_id\":1,\"x\":6.000,\"y\":0.000,\"z\":0.000}",
      "\"target_kind\":\"point\"", "\"target\":{\"frame\":\"map\",\"floor_id\":1,\"x\":10.000",
      "\"standoff_m\":3.000"})
  {
    EXPECT_NE(j.find(part), std::string::npos) << part << "\n" << j;
  }
  EXPECT_EQ(j.find("track_id"), std::string::npos);
  EXPECT_EQ(j.front(), '"');   // 不含大括號

  // 沒寫的欄位不出現；沒有目標時只有觀察點
  TaskMeta bare;
  bare.incident_id = "x";
  const std::string b = metaJsonMembers(bare, {1.0, 2.0, 3.0}, 0);
  for (const char * absent : {"event_type", "task_type", "\"role\"", "description", "target", "standoff_m"}) {
    EXPECT_EQ(b.find(absent), std::string::npos) << absent << "\n" << b;
  }
  EXPECT_NE(b.find("\"source\":\"UNKNOWN\""), std::string::npos);
  EXPECT_NE(b.find("\"observe_pose\""), std::string::npos);
}

TEST(TaskMeta, JsonStringEscapes)
{
  EXPECT_EQ(jsonString("a\"b\\c\nd"), "\"a\\\"b\\\\c\\nd\"");
  EXPECT_EQ(jsonString(std::string("\x01", 1)), "\"\\u0001\"");
  EXPECT_EQ(jsonString("入侵"), "\"入侵\"");
}

TEST(TaskMeta, SchemaMajor)
{
  EXPECT_EQ(specSchemaMajor("1.1"), 1);
  EXPECT_EQ(specSchemaMajor("1.0"), 1);
  EXPECT_EQ(specSchemaMajor("2"), 2);
  EXPECT_FALSE(specSchemaMajor(""));
  EXPECT_FALSE(specSchemaMajor("v1"));
  EXPECT_FALSE(specSchemaMajor("1."));
  EXPECT_FALSE(specSchemaMajor("1.x"));
}

TEST(TaskMeta, EvidenceReferenceOnly)
{
  // 03：TaskSpec 只帶證據的參照與信心值；URI 的 scheme、長度、空白；信心值 0～1
  TaskMeta m = pointMeta();
  m.evidence_uri = "evidence://ev_0042/snapshot/cam07_0001.jpg";
  m.evidence_confidence = 0.93;
  EXPECT_EQ(checkMeta(m, {6.0, 0.0, 0.0}), "");
  const std::string j = metaJsonMembers(m, {6.0, 0.0, 0.0}, 1);
  EXPECT_NE(j.find("\"evidence\":{\"uri\":\"evidence://ev_0042/snapshot/cam07_0001.jpg\",\"confidence\":0.930}"),
    std::string::npos) << j;
  EXPECT_TRUE(validEvidenceUri(""));
  EXPECT_TRUE(validEvidenceUri("https://vms.local/clip/1.mp4"));
  EXPECT_FALSE(validEvidenceUri("ftp://x/y"));
  EXPECT_FALSE(validEvidenceUri("evidence://"));
  EXPECT_FALSE(validEvidenceUri("evidence://a b"));
  EXPECT_FALSE(validEvidenceUri("evidence://" + std::string(600, 'x')));
  m.evidence_confidence = 1.5;
  EXPECT_NE(checkMeta(m, {6.0, 0.0, 0.0}), "");
  m.evidence_confidence = 0.5;
  m.evidence_uri = "C:\\images\\1.jpg";
  EXPECT_NE(checkMeta(m, {6.0, 0.0, 0.0}), "");
  TaskMeta none;
  EXPECT_EQ(metaJsonMembers(none, {}, 0).find("evidence"), std::string::npos);
}
