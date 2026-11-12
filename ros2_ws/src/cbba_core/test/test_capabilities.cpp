// 能力過濾與 P0～P3 優先級（2-1b M0 合約第一、二節，2026-11-11）
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/cbba_agent.hpp"

using namespace cbba_core;

namespace
{
AgentState makeAgent(AgentId id, AgentType type, double x = 0.0)
{
  AgentState a;
  a.id = id;
  a.type = type;
  a.position = {x, 0.0, 0.0};
  a.home = a.position;
  a.battery = 200.0;
  a.safety_reserve = 20.0;
  a.energy_per_meter = 0.1;
  a.hover_energy_per_sec = 0.05;
  a.cruise_speed = 1.0;
  return a;
}

// 新型任務（type = NONE）：平台、能力、優先級看欄位
Task newTask(TaskId id, double x, std::uint8_t platform, Capabilities caps, std::uint8_t priority = 3)
{
  Task t;
  t.id = id;
  t.type = TaskType::NONE;
  t.position = {x, 0.0, 0.0};
  t.deadline_sec = 600.0;
  t.value = 80.0;
  t.platform = platform;
  t.required_caps = caps;
  t.priority = priority;
  return t;
}

Task legacyTask(TaskId id, TaskType type)
{
  Task t;
  t.id = id;
  t.type = type;
  return t;
}
}  // namespace

TEST(Capabilities, ParsesNamesAndRejectsUnknown)
{
  // 名稱 → 位元；不認得的名稱整份拒絕並回報是哪一個
  const auto caps = parseCapabilities({"rgb_camera", "thermal_camera"});
  ASSERT_TRUE(caps.has_value());
  EXPECT_EQ(*caps, capBit(Capability::RGB_CAMERA) | capBit(Capability::THERMAL_CAMERA));
  EXPECT_EQ(capabilitiesToString(*caps), "rgb_camera,thermal_camera");

  std::string bad;
  EXPECT_FALSE(parseCapabilities({"rgb_camera", "laser"}, &bad).has_value());
  EXPECT_EQ(bad, "laser");
  EXPECT_EQ(capabilitiesToString(0), "none");
  EXPECT_EQ(capabilitiesToString(Capabilities{1} << 40), "bit40");
}

TEST(Capabilities, BitPositionsAreFrozen)
{
  // v1.0 的位元位置只增不改：改了會和別台、封包、註冊表對不上
  EXPECT_EQ(capBit(Capability::MOBILITY_GROUND), Capabilities{1} << 0);
  EXPECT_EQ(capBit(Capability::MOBILITY_AIR), Capabilities{1} << 1);
  EXPECT_EQ(capBit(Capability::RGB_CAMERA), Capabilities{1} << 4);
  EXPECT_EQ(capBit(Capability::THERMAL_CAMERA), Capabilities{1} << 6);
  EXPECT_EQ(capBit(Capability::COMM_RELAY), Capabilities{1} << 12);
  for (const char * name : {"mobility_ground", "mobility_air", "indoor_flight", "stairs", "rgb_camera", "zoom",
      "thermal_camera", "speaker", "spotlight", "gas_sensor", "manipulator", "gripper", "comm_relay"})
  {
    const auto cap = capabilityFromName(name);
    ASSERT_TRUE(cap.has_value()) << name;
    EXPECT_STREQ(capabilityName(*cap), name);
  }
}

TEST(Capabilities, LegacyTaskTypesKeepOldCompatibility)
{
  // 舊型任務照原本的相容表：無人機只做空中偵察，狗只做地面處置、巡邏（能力用預設）
  const AgentState uav = makeAgent(1, AgentType::UAV);
  const AgentState dog = makeAgent(50, AgentType::UGV);
  EXPECT_EQ(fitness(uav, legacyTask(1, TaskType::AIR_RECON)), Fitness::OK);
  EXPECT_EQ(fitness(uav, legacyTask(2, TaskType::GROUND_INTERVENTION)), Fitness::PLATFORM);
  EXPECT_EQ(fitness(uav, legacyTask(3, TaskType::PATROL)), Fitness::PLATFORM);
  EXPECT_EQ(fitness(dog, legacyTask(1, TaskType::AIR_RECON)), Fitness::PLATFORM);
  EXPECT_EQ(fitness(dog, legacyTask(2, TaskType::GROUND_INTERVENTION)), Fitness::OK);
  EXPECT_EQ(fitness(dog, legacyTask(3, TaskType::PATROL)), Fitness::OK);
}

TEST(Capabilities, PlatformAndCapabilityFilter)
{
  // 新型任務：平台 ANY 時兩種都能做；缺任何一項能力就不行；故障的能力不算
  AgentState dog = makeAgent(50, AgentType::UGV);
  dog.capabilities = capBit(Capability::MOBILITY_GROUND) | capBit(Capability::RGB_CAMERA) |
    capBit(Capability::THERMAL_CAMERA);
  const AgentState uav = makeAgent(1, AgentType::UAV);

  const Task photo = newTask(1, 5.0, kPlatformAny, capBit(Capability::RGB_CAMERA));
  EXPECT_EQ(fitness(dog, photo), Fitness::OK);
  EXPECT_EQ(fitness(uav, photo), Fitness::OK);

  const Task thermal = newTask(2, 5.0, platformOf(AgentType::UGV), capBit(Capability::THERMAL_CAMERA));
  EXPECT_EQ(fitness(dog, thermal), Fitness::OK);
  EXPECT_EQ(fitness(uav, thermal), Fitness::PLATFORM);

  const Task speak = newTask(3, 5.0, kPlatformAny, capBit(Capability::SPEAKER) | capBit(Capability::RGB_CAMERA));
  EXPECT_EQ(fitness(dog, speak), Fitness::CAPABILITY);

  dog.failed_caps = capBit(Capability::THERMAL_CAMERA);   // E_HW
  EXPECT_EQ(fitness(dog, thermal), Fitness::CAPABILITY);
}

TEST(Capabilities, AgentDoesNotBidWithoutCapabilityAndSaysWhy)
{
  // 出價表的原因：平台不符 platform、缺能力 capability；有能力的照常得標
  AgentState dog = makeAgent(50, AgentType::UGV);
  dog.capabilities = capBit(Capability::MOBILITY_GROUND) | capBit(Capability::RGB_CAMERA);
  CbbaAgent agent(dog, ScoringParams{});
  agent.onTask(newTask(1, 5.0, kPlatformAny, capBit(Capability::THERMAL_CAMERA)), 0.0);
  agent.onTask(newTask(2, 5.0, platformOf(AgentType::UAV), capBit(Capability::RGB_CAMERA)), 0.0);
  agent.onTask(newTask(3, 5.0, kPlatformAny, capBit(Capability::RGB_CAMERA)), 0.0);

  EXPECT_EQ(agent.winner(1), kNoAgent);
  EXPECT_STREQ(agent.bidFor(1, 0.0).why, "capability");
  EXPECT_EQ(agent.winner(2), kNoAgent);
  EXPECT_STREQ(agent.bidFor(2, 0.0).why, "platform");
  EXPECT_EQ(agent.winner(3), 50);
}

TEST(Priority, LegacyFireIsP0OthersP3)
{
  // 舊型任務的優先級：火警處置 P0，空中偵察、巡邏 P3；新型看欄位，超出範圍當 P3
  EXPECT_EQ(priorityLevel(legacyTask(1, TaskType::GROUND_INTERVENTION)), 0);
  EXPECT_EQ(priorityLevel(legacyTask(1, TaskType::AIR_RECON)), 3);
  EXPECT_EQ(priorityLevel(legacyTask(1, TaskType::PATROL)), 3);
  EXPECT_EQ(taskPriority(legacyTask(1, TaskType::GROUND_INTERVENTION)), kPriorityFire);
  EXPECT_EQ(taskPriority(legacyTask(1, TaskType::PATROL)), kPriorityPatrol);
  EXPECT_EQ(priorityLevel(newTask(1, 0.0, kPlatformAny, 0, 1)), 1);
  EXPECT_EQ(priorityLevel(newTask(1, 0.0, kPlatformAny, 0, 9)), 3);
}

TEST(Priority, BundleTakesHigherPriorityFirstRegardlessOfDistance)
{
  // 只能接一個：近的 P3 分數高，遠的 P1 分數低 → 照優先級接 P1（任何距離差都不能跨級）
  ScoringParams params;
  params.max_bundle = 1;
  CbbaAgent agent(makeAgent(50, AgentType::UGV), params);
  agent.setActive(false, 0.0);   // 兩個任務都知道之後才一起挑（逐一收到時，先到的會先佔滿 bundle）
  agent.onTask(newTask(1, 2.0, kPlatformAny, 0, 3), 0.0);
  agent.onTask(newTask(2, 40.0, kPlatformAny, 0, 1), 0.0);
  agent.setActive(true, 0.0);
  ASSERT_EQ(agent.bundle().size(), 1u);
  EXPECT_EQ(agent.bundle().front(), 2u);
  EXPECT_EQ(agent.winner(1), kNoAgent);
}

TEST(Priority, SamePriorityPicksBetterScore)
{
  // 同級時照分數：近的先接
  ScoringParams params;
  params.max_bundle = 1;
  CbbaAgent agent(makeAgent(50, AgentType::UGV), params);
  agent.setActive(false, 0.0);
  agent.onTask(newTask(1, 40.0, kPlatformAny, 0, 2), 0.0);
  agent.onTask(newTask(2, 2.0, kPlatformAny, 0, 2), 0.0);
  agent.setActive(true, 0.0);
  ASSERT_EQ(agent.bundle().size(), 1u);
  EXPECT_EQ(agent.bundle().front(), 2u);
}
