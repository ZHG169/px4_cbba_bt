// CB-4 第一批（2026-11-16）：TaskSpec 的 wait_for → step-level gate；precedence 沒有 wait_for 的角色預設第一步等 completed
#include <gtest/gtest.h>

#include <string>

#include "swarm_cbba/spec_gates.hpp"

using namespace swarm_cbba;
using cbba_core::Milestone;

TEST(SpecGates, WaitForOnStep)
{
  // 入侵範本的 front：第 1 步（inspect）等 role 0 的 target_localized；前面的 go_to 不用等（狗可以先出發）
  const std::string spec = R"({"steps": [
    {"action": "go_to", "location": {"poi": "OBS_HV_A_FRONT"}},
    {"action": "inspect", "object": "person", "wait_for": [{"role_index": 0, "milestone": "target_localized"}]},
    {"action": "speak", "params": {"template": "intrusion_warning"}}]})";
  const SpecGates g = parseSpecGates(spec, 0b001, 1);
  EXPECT_TRUE(g.warnings.empty());
  ASSERT_EQ(g.steps.size(), 1u);
  EXPECT_EQ(g.steps[0].step_index, 1);
  ASSERT_EQ(g.steps[0].wait_for.size(), 1u);
  EXPECT_EQ(g.steps[0].wait_for[0].role_index, 0);
  EXPECT_EQ(g.steps[0].wait_for[0].milestone, Milestone::TARGET_LOCALIZED);
}

TEST(SpecGates, DefaultFirstStepWaitsCompleted)
{
  // precedence 有 role 0、2，wait_for 只提到 role 0 → role 2 預設在第一步等 completed；沒有 spec 時全部在第一步等
  const std::string spec = R"({"steps": [{"action": "go_to"},
    {"action": "inspect", "wait_for": [{"role_index": 0, "milestone": "arrived"}]}]})";
  const SpecGates g = parseSpecGates(spec, 0b101, 1);
  ASSERT_EQ(g.steps.size(), 2u);
  EXPECT_EQ(g.steps[0].step_index, 0);
  ASSERT_EQ(g.steps[0].wait_for.size(), 1u);
  EXPECT_EQ(g.steps[0].wait_for[0].role_index, 2);
  EXPECT_EQ(g.steps[0].wait_for[0].milestone, Milestone::COMPLETED);
  EXPECT_EQ(g.steps[1].wait_for[0].milestone, Milestone::ARRIVED);

  const SpecGates empty = parseSpecGates("", 0b011, 2);
  ASSERT_EQ(empty.steps.size(), 1u);
  EXPECT_EQ(empty.steps[0].wait_for.size(), 2u);
  EXPECT_TRUE(parseSpecGates("", 0, 0).steps.empty());
  // 自己的位元不算
  EXPECT_TRUE(parseSpecGates("", 0b010, 1).steps.empty());
}

TEST(SpecGates, Warnings)
{
  // 不認得的里程碑、不在 precedence 的角色（照樣等）、等自己、格式錯
  const std::string spec = R"({"steps": [{"action": "go_to", "wait_for": [
    {"role_index": 0, "milestone": "flying"},
    {"role_index": 2, "milestone": "arrived"},
    {"role_index": 1, "milestone": "arrived"},
    {"milestone": "arrived"}]}]})";
  const SpecGates g = parseSpecGates(spec, 0b001, 1);
  EXPECT_EQ(g.warnings.size(), 4u);
  // role 2 照樣等；role 0 的 wait_for 不認得 → 沒被提到 → 預設第一步等 completed
  ASSERT_EQ(g.steps.size(), 1u);
  ASSERT_EQ(g.steps[0].wait_for.size(), 2u);
  EXPECT_EQ(g.steps[0].wait_for[0].role_index, 2);
  EXPECT_EQ(g.steps[0].wait_for[1].role_index, 0);
  EXPECT_EQ(g.steps[0].wait_for[1].milestone, Milestone::COMPLETED);
  EXPECT_FALSE(parseSpecGates("{not json", 0, 0).warnings.empty());
}
