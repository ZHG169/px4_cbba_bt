// 2-1b 的 8 個失敗碼 → 排除分類（介面 v2.0，2026-11-14；合約第三節的表）
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/failure_codes.hpp"

using namespace cbba_core;

// 合約的表逐條對照
TEST(FailureCodes, MapsAllEightCodes)
{
  const std::vector<std::pair<std::string, FailureClass>> expect{
    {"E_PRECONDITION", FailureClass::PERMANENT},
    {"E_UNREACHABLE", FailureClass::PERMANENT},
    {"E_TIMEOUT", FailureClass::TRANSIENT},
    {"E_TARGET_LOST", FailureClass::TRANSIENT},
    {"E_LOW_BATTERY", FailureClass::STOPPED},
    {"E_SAFETY_STOP", FailureClass::STOPPED},
    {"E_COMM", FailureClass::TRANSIENT},
    {"E_HW", FailureClass::PERMANENT},
  };
  for (const auto & [code, cls] : expect) {
    const auto info = failureCodeInfo(code);
    ASSERT_TRUE(info.has_value()) << code;
    EXPECT_EQ(info->cls, cls) << code;
    EXPECT_EQ(info->notify_orphan, code == "E_PRECONDITION") << code;
    EXPECT_EQ(info->drop_capability, code == "E_HW") << code;
  }
}

// 空字串（舊的 BT）、不認得的、大小寫不同的都不認：節點層照舊用 reason
TEST(FailureCodes, UnknownCodeFallsBackToReason)
{
  EXPECT_FALSE(failureCodeInfo("").has_value());
  EXPECT_FALSE(failureCodeInfo("E_FOO").has_value());
  EXPECT_FALSE(failureCodeInfo("e_hw").has_value());
}

// 數值和 TaskResult.reason 相同（節點層直接轉型）
TEST(FailureCodes, ClassValuesMatchTaskResultReason)
{
  EXPECT_EQ(static_cast<int>(FailureClass::PERMANENT), 1);
  EXPECT_EQ(static_cast<int>(FailureClass::TRANSIENT), 2);
  EXPECT_EQ(static_cast<int>(FailureClass::PREEMPTED), 3);
  EXPECT_EQ(static_cast<int>(FailureClass::STOPPED), 4);
}

// current_task 的 required_capabilities：遮罩 → 名稱陣列，和 parseCapabilities 互逆
TEST(CapabilityNames, RoundTrip)
{
  const Capabilities caps = capBit(Capability::MOBILITY_AIR) | capBit(Capability::THERMAL_CAMERA);
  const auto names = capabilityNames(caps);
  EXPECT_EQ(names, (std::vector<std::string>{"mobility_air", "thermal_camera"}));
  EXPECT_EQ(parseCapabilities(names), caps);
  EXPECT_TRUE(capabilityNames(0).empty());
  // 不認得的位元（註冊表比這版新）印成 bitN，不丟掉
  EXPECT_EQ(capabilityNames(Capabilities{1} << 40), (std::vector<std::string>{"bit40"}));
}
