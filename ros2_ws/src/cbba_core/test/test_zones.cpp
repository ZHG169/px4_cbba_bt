// 任務欄位補齊第二批（2026-11-18）：區域（05）、能力與高度（08）、同級搶占的遲滯（07）
//   區域註冊表的判斷：樓層、高度上限、天花板、no_entry、zone_policy_id、室內；出價表的原因；preempt_margin_pct
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/cbba_agent.hpp"
#include "cbba_core/zones.hpp"

using namespace cbba_core;

namespace
{
constexpr double kNow = 1000.0;
constexpr std::uint8_t kUav = 1;
constexpr std::uint8_t kDog = 2;

ZoneArea square(const std::string & id, double x0, double y0, double x1, double y1)
{
  ZoneArea z;
  z.id = id;
  z.floor = 1;
  z.polygon = {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}};
  return z;
}

// Z_HV（0～10）：狗不能進、天花板 5 m；Z_ROOM（20～30）：室內；政策 1：UAV 不進 Z_HV
std::shared_ptr<const ZoneRegistry> yard()
{
  auto r = std::make_shared<ZoneRegistry>();
  r->version = 3;
  ZoneArea hv = square("Z_HV", 0, 0, 10, 10);
  hv.no_entry = 1u << kDog;
  hv.ceiling = 5.0;
  ZoneArea room = square("Z_ROOM", 20, 0, 30, 10);
  room.indoor = true;
  r->zones = {hv, room};
  ZonePolicy p;
  p.id = 1;
  p.name = "uav_no_hv";
  p.forbid = {ZonePolicy::Rule{0, static_cast<std::uint8_t>(1u << kUav)}};
  r->policies = {p};
  return r;
}

AgentState agent(AgentType type)
{
  AgentState a;
  a.id = type == AgentType::UAV ? 1 : 50;
  a.type = type;
  a.floor = 1;
  a.battery = 100.0;
  a.safety_reserve = 10.0;
  a.energy_per_meter = 0.01;
  a.hover_energy_per_sec = 0.01;
  a.cruise_speed = 1.0;
  return a;
}

Task at(double x, double y, double z = 0.0, std::uint8_t platform = 0)
{
  Task t;
  t.id = 0x00C8000100000001ULL;
  t.type = TaskType::NONE;
  t.platform = platform;
  t.priority = 1;
  t.position = {x, y, z};
  t.created = kNow;
  t.value = 100.0;
  t.duration_sec = 5.0;
  t.ext.floor = 1;
  return t;
}
}  // namespace

TEST(Zones, PolygonContains)
{
  const ZoneArea z = square("Z", 0, 0, 10, 10);
  EXPECT_TRUE(z.contains(5, 5));
  EXPECT_TRUE(z.contains(0, 5));     // 邊上算在裡面（禁區寧可多擋）
  EXPECT_TRUE(z.contains(10, 10));
  EXPECT_FALSE(z.contains(10.01, 5));
  EXPECT_FALSE(z.contains(-1, -1));
}

TEST(Zones, FloorAndAltitude)
{
  // 樓層：任務、載具都知道而且不同才擋；高度：載具的上限、區域的天花板
  const auto zones = yard();
  AgentState uav = agent(AgentType::UAV);
  Task t = at(15, 5, 8);
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, t), ZoneVerdict::OK);
  t.ext.floor = 2;
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, t), ZoneVerdict::WRONG_FLOOR);
  uav.floor = 0;   // 載具的樓層不知道：不擋
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, t), ZoneVerdict::OK);
  t.ext.floor = 0;   // 任務不分樓層
  uav.floor = 1;
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, t), ZoneVerdict::OK);

  uav.max_altitude = 6.0;
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, t), ZoneVerdict::ALTITUDE_LIMIT);
  EXPECT_EQ(checkZones(nullptr, uav, kUav, t), ZoneVerdict::ALTITUDE_LIMIT);   // 沒有區域註冊表也看載具的上限
  uav.max_altitude = 0.0;
  std::string zone;
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, at(5, 5, 6), &zone), ZoneVerdict::ALTITUDE_LIMIT);   // 天花板 5 m
  EXPECT_EQ(zone, "Z_HV");
}

TEST(Zones, NoEntryPolicyAndIndoor)
{
  // no_entry 看平台；政策 1 另外禁止 UAV 進 Z_HV；不認得的政策只套用 no_entry；室內要 indoor_flight
  const auto zones = yard();
  const AgentState dog = agent(AgentType::UGV);
  AgentState uav = agent(AgentType::UAV);
  EXPECT_EQ(checkZones(zones.get(), dog, kDog, at(5, 5)), ZoneVerdict::ZONE_FORBIDDEN);
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, at(5, 5, 3)), ZoneVerdict::OK);
  Task policy = at(5, 5, 3);
  policy.ext.zone_policy_id = 1;
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, policy), ZoneVerdict::ZONE_FORBIDDEN);
  policy.ext.zone_policy_id = 9;   // 本機不認得
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, policy), ZoneVerdict::OK);

  EXPECT_EQ(checkZones(zones.get(), uav, kUav, at(25, 5, 2)), ZoneVerdict::INDOOR);
  uav.capabilities = capBit(Capability::MOBILITY_AIR) | capBit(Capability::RGB_CAMERA) |
    capBit(Capability::INDOOR_FLIGHT);
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, at(25, 5, 2)), ZoneVerdict::OK);
  uav.failed_caps = capBit(Capability::INDOOR_FLIGHT);   // 故障（E_HW）就不行
  EXPECT_EQ(checkZones(zones.get(), uav, kUav, at(25, 5, 2)), ZoneVerdict::INDOOR);
  EXPECT_EQ(checkZones(zones.get(), dog, kDog, at(25, 5)), ZoneVerdict::OK);   // 室內只限制 UAV
}

TEST(Zones, TargetInsideObservePoseOutside)
{
  // 05 驗收 3：事件在禁區裡、觀察點在外面 → 狗照樣可以接（只看觀察點，不看目標、不看路徑）
  ScoringParams params;
  params.zones = yard();
  CbbaAgent dog(agent(AgentType::UGV), params);
  Task outside = at(12, 5, 0, kDog);   // 觀察點在 Z_HV 外 2 m
  dog.onTask(outside, kNow);
  EXPECT_EQ(dog.path(), std::vector<TaskId>{outside.id});
  Task inside = at(5, 5, 0, kDog);
  inside.id = outside.id + 1;
  dog.onTask(inside, kNow);
  EXPECT_STREQ(dog.bidFor(inside.id, kNow).why, "zone_forbidden");
  EXPECT_DOUBLE_EQ(dog.bidFor(inside.id, kNow).bid, 0.0);
}

TEST(Zones, BidReasonsForFloorAltitudeIndoor)
{
  // 05、08 驗收：狗不接另一層的任務；UAV 不接禁飛區、超高、沒有 indoor_flight 的室內任務（出價表寫原因）
  ScoringParams params;
  params.zones = yard();
  CbbaAgent dog(agent(AgentType::UGV), params);
  Task upstairs = at(40, 5, 0, kDog);
  upstairs.ext.floor = 2;
  dog.onTask(upstairs, kNow);
  EXPECT_STREQ(dog.bidFor(upstairs.id, kNow).why, "wrong_floor");

  AgentState u = agent(AgentType::UAV);
  u.max_altitude = 10.0;
  CbbaAgent uav(u, params);
  Task high = at(40, 5, 12, kUav);
  Task room = at(25, 5, 2, kUav);
  room.id = high.id + 1;
  Task no_fly = at(5, 5, 3, kUav);
  no_fly.id = high.id + 2;
  no_fly.ext.zone_policy_id = 1;
  for (const Task & t : {high, room, no_fly}) {
    uav.onTask(t, kNow);
  }
  EXPECT_STREQ(uav.bidFor(high.id, kNow).why, "altitude_limit");
  EXPECT_STREQ(uav.bidFor(room.id, kNow).why, "indoor");
  EXPECT_STREQ(uav.bidFor(no_fly.id, kNow).why, "zone_forbidden");
  EXPECT_TRUE(uav.path().empty());
}

TEST(Zones, FailedCapabilityReleasesTaskAndRestoreBidsAgain)
{
  // 08 驗收 4：感測器故障（E_HW 拿掉能力）→ 下一次重新評估就放掉需要它的任務；恢復後又會出價
  CbbaAgent dog(agent(AgentType::UGV), ScoringParams{});
  Task thermal = at(10, 0, 0, kDog);
  thermal.required_caps = capBit(Capability::MOBILITY_GROUND) | capBit(Capability::THERMAL_CAMERA);
  AgentState s = agent(AgentType::UGV);
  s.capabilities = capBit(Capability::MOBILITY_GROUND) | capBit(Capability::THERMAL_CAMERA);
  dog.setState(s);
  dog.onTask(thermal, kNow);
  ASSERT_EQ(dog.path(), std::vector<TaskId>{thermal.id});

  s.failed_caps = capBit(Capability::THERMAL_CAMERA);
  dog.setState(s);
  dog.reevaluate(kNow + 1.0);
  EXPECT_TRUE(dog.path().empty());
  EXPECT_STREQ(dog.bidFor(thermal.id, kNow + 1.0).why, "capability");

  s.failed_caps = 0;   // capability_restored
  dog.setState(s);
  EXPECT_STREQ(dog.bidFor(thermal.id, kNow + 2.0).why, "bid");
}

namespace
{
// 狗手上有一個執行中的 P1（遠，分數低），來了一個同級、遮罩允許同級搶占的 P1（近）
std::vector<TaskId> sameLevelPreempt(std::uint8_t margin_pct)
{
  CbbaAgent dog(agent(AgentType::UGV), ScoringParams{});
  Task far = at(30, 0, 0, kDog);
  dog.onTask(far, kNow);
  ExecConstraints c;
  c.pinned = {far.id};
  c.active_pinned = true;
  c.preemptible = true;
  c.active_remaining = 60.0;
  dog.setExecConstraints(c, kNow);
  Task near = at(3, 0, 0, kDog);
  near.id = far.id + 1;
  near.ext.can_preempt = 0b0010;   // 可以搶 P1（同級）
  near.ext.preempt_margin_pct = margin_pct;
  dog.onTask(near, kNow);
  return dog.path();
}
}  // namespace

TEST(PreemptMargin, SameLevelNeedsToBeBetterByMargin)
{
  // 07 驗收 2：同級搶占要新分數 > 目前任務的分數 ×（1 + margin）。近的 P1 分數約是遠的 1.2 倍：
  // margin 0 → 搶（排到最前面）；margin 50 % → 不夠好，不排到執行中的前面（小幅差異不切換）
  const auto no_margin = sameLevelPreempt(0);
  ASSERT_FALSE(no_margin.empty());
  EXPECT_EQ(no_margin.front(), at(3, 0).id + 1);
  const auto with_margin = sameLevelPreempt(50);
  ASSERT_FALSE(with_margin.empty());
  EXPECT_EQ(with_margin.front(), at(30, 0).id);
}

TEST(PreemptMargin, CrossLevelIgnoresMargin)
{
  // 07 驗收 1：P0 搶 P3 不看分數（margin 只用在同級）
  CbbaAgent dog(agent(AgentType::UGV), ScoringParams{});
  Task patrol = at(30, 0, 0, kDog);
  patrol.priority = 3;
  dog.onTask(patrol, kNow);
  ExecConstraints c;
  c.pinned = {patrol.id};
  c.active_pinned = true;
  c.active_remaining = 60.0;
  dog.setExecConstraints(c, kNow);
  Task fire = at(40, 0, 0, kDog);
  fire.id = patrol.id + 1;
  fire.priority = 0;
  fire.ext.preempt_margin_pct = 100;
  dog.onTask(fire, kNow);
  ASSERT_FALSE(dog.path().empty());
  EXPECT_EQ(dog.path().front(), fire.id);
}
