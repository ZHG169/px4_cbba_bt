#include "cbba_core/tdr_experiment.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <sstream>
#include <stdexcept>

#include "cbba_core/capabilities.hpp"

namespace cbba_core
{

ScoringParams applyParamSet(ScoringParams base, const ParamSet & set)
{
  base.energy_weight = set.energy_weight;
  for (std::size_t p = 0; p < 4; ++p) {
    base.half_life_by_priority[p] = kBaseHalfLife[p] * set.gamma;
  }
  return base;
}

std::string halfLifeLabel(double gamma)
{
  std::ostringstream out;
  for (std::size_t p = 0; p < 4; ++p) {
    out << (p > 0 ? "/" : "") << kBaseHalfLife[p] * gamma;
  }
  return out.str();
}

const std::vector<std::string> & scenarioNames()
{
  static const std::vector<std::string> names{"S1", "S2", "S3", "S4", "S5", "S6"};
  return names;
}

namespace
{
constexpr double kPi = 3.14159265358979323846;

class Draw
{
public:
  explicit Draw(std::uint32_t seed) : rng_(seed) {}
  double uniform(double lo, double hi) {return std::uniform_real_distribution<double>(lo, hi)(rng_);}

private:
  std::mt19937 rng_;
};

Vec3 polar(double r, double angle) {return {r * std::cos(angle), r * std::sin(angle), 0.0};}
Vec3 add(const Vec3 & a, const Vec3 & b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}

MissionAgent dog(
  AgentId id, const Vec3 & position, double battery, double speed, double per_meter, double per_sec,
  double reserve, double error)
{
  MissionAgent a;
  a.state.id = id;
  a.state.type = AgentType::UGV;
  a.state.position = position;
  a.state.battery = battery;
  a.state.safety_reserve = reserve;
  a.state.cruise_speed = speed;
  a.state.energy_per_meter = per_meter;
  a.state.hover_energy_per_sec = per_sec;
  a.drain_rate = per_sec * (1.0 + error);
  a.drain_per_meter = per_meter * (1.0 + error);
  a.max_bundle = 5;
  return a;
}

// 和 cbba_node 的續航換算一致：每秒 100 / 續航、每公尺 = 每秒 / 速度；PX4 的模擬電池只依時間
MissionAgent uav(
  AgentId id, const Vec3 & position, double battery, const ScenarioOptions & opt, double error)
{
  MissionAgent a;
  a.state.id = id;
  a.state.type = AgentType::UAV;
  a.state.position = position;
  a.state.battery = battery;
  a.state.safety_reserve = opt.reserve;
  a.state.cruise_speed = opt.uav_speed;
  a.state.hover_energy_per_sec = 100.0 / opt.uav_endurance;
  a.state.energy_per_meter = a.state.hover_energy_per_sec / opt.uav_speed;
  a.drain_rate = a.state.hover_energy_per_sec * (1.0 + error);
  a.max_bundle = 2;
  return a;
}

// 新型任務（TaskType::NONE）：優先級、平台由情境給，沒有期限（軟期限不影響 TDR 的分數）
MissionTask task(
  AgentId creator, std::uint16_t seq, const std::string & label, const Vec3 & position,
  std::uint8_t priority, std::uint8_t platform, double duration, double insert_at, double value = 100.0)
{
  MissionTask m;
  m.label = label;
  m.insert_at = insert_at;
  m.task.id = makeTaskId(creator, seq);
  m.task.type = TaskType::NONE;
  m.task.position = position;
  m.task.priority = priority;
  m.task.platform = platform;
  m.task.value = value;
  m.task.duration_sec = duration;
  m.task.explicit_deadline = false;
  m.task.deadline_sec = 3600.0;
  return m;
}

constexpr std::uint8_t kUav = 1;
constexpr std::uint8_t kDog = 2;

ScenarioCase s1(Draw & d, std::uint32_t seed, const ScenarioOptions & opt)
{
  // 兩隻狗離任務一樣遠、每公尺耗電一樣、不算待機耗電 → 能量項相同，只有到達時間不同
  ScenarioCase c;
  c.max_time = 300.0;
  const double r = d.uniform(20.0, 40.0);
  const double a = d.uniform(0.0, 2.0 * kPi);
  const double b = a + d.uniform(0.5 * kPi, 1.5 * kPi);
  const double e1 = d.uniform(-opt.drain_error, opt.drain_error);
  const double e2 = d.uniform(-opt.drain_error, opt.drain_error);
  // 慢的機號小：平手時比機號會給慢的，要靠時間折扣才給對
  c.agents.push_back(dog(50, polar(r, a), 80.0, 0.7, 0.1, 0.0, opt.reserve, e1));
  c.agents.push_back(dog(51, polar(r, b), 80.0, 1.5, 0.1, 0.0, opt.reserve, e2));
  const std::uint8_t priority = seed % 2 == 0 ? 0 : 1;
  MissionTask m = task(50, 1, priority == 0 ? "S1_P0" : "S1_P1", {0.0, 0.0, 0.0}, priority, kDog, 10.0, opt.t0);
  m.expected = 51;
  m.ideal_eta = r / 1.5;
  c.tasks.push_back(m);
  return c;
}

ScenarioCase s2(Draw & d, const ScenarioOptions & opt)
{
  // 兩隻狗速度、距離、耗電模型都一樣，只有電量不同（可用 15 % vs 70 %）
  ScenarioCase c;
  c.max_time = 300.0;
  const double r = d.uniform(15.0, 35.0);
  const double a = d.uniform(0.0, 2.0 * kPi);
  const double e1 = d.uniform(-opt.drain_error, opt.drain_error);
  const double e2 = d.uniform(-opt.drain_error, opt.drain_error);
  c.agents.push_back(dog(50, polar(r, a), 35.0, 1.0, opt.dog_per_meter, opt.dog_per_sec, opt.reserve, e1));
  c.agents.push_back(dog(51, polar(r, a + kPi), 90.0, 1.0, opt.dog_per_meter, opt.dog_per_sec, opt.reserve, e2));
  MissionTask m = task(50, 1, "S2_P2", {0.0, 0.0, 0.0}, 2, kDog, 10.0, opt.t0);
  m.expected = 51;
  m.ideal_eta = r;
  c.tasks.push_back(m);
  return c;
}

ScenarioCase s3(Draw & d, std::uint32_t seed, const ScenarioOptions & opt)
{
  // A 快但耗電高、B 慢但省電，距離相同；P0 該給 A（急），P3 該給 B（不急、省電）
  ScenarioCase c;
  c.max_time = 300.0;
  const double r = d.uniform(20.0, 40.0);
  const double a = d.uniform(0.0, 2.0 * kPi);
  const double b = a + d.uniform(0.5 * kPi, 1.5 * kPi);
  const double e1 = d.uniform(-opt.drain_error, opt.drain_error);
  const double e2 = d.uniform(-opt.drain_error, opt.drain_error);
  c.agents.push_back(dog(50, polar(r, a), 80.0, 1.5, 0.3, 0.0, opt.reserve, e1));
  c.agents.push_back(dog(51, polar(r, b), 80.0, 0.7, 0.08, 0.0, opt.reserve, e2));
  const std::uint8_t priority = seed % 2 == 0 ? 0 : 3;
  MissionTask m = task(50, 1, priority == 0 ? "S3_P0" : "S3_P3", {0.0, 0.0, 0.0}, priority, kDog, 10.0, opt.t0);
  m.expected = priority == 0 ? 50 : 51;
  m.ideal_eta = priority == 0 ? r / 1.5 : r / 0.7;
  c.tasks.push_back(m);
  return c;
}

ScenarioCase s4(Draw & d, const ScenarioOptions & opt)
{
  // 狗 A 前往 P3（25 m 外）的途中，旁邊插入 P0；狗 B 在 P0 的 25～50 m 外
  ScenarioCase c;
  c.max_time = 400.0;
  const Vec3 urgent{d.uniform(5.0, 15.0), d.uniform(6.0, 14.0), 0.0};
  const Vec3 b = add(urgent, polar(d.uniform(25.0, 50.0), d.uniform(0.0, 2.0 * kPi)));
  const double e1 = d.uniform(-opt.drain_error, opt.drain_error);
  const double e2 = d.uniform(-opt.drain_error, opt.drain_error);
  c.agents.push_back(dog(50, {0.0, 0.0, 0.0}, 85.0, 1.0, opt.dog_per_meter, opt.dog_per_sec, opt.reserve, e1));
  c.agents.push_back(dog(51, b, 85.0, 1.0, opt.dog_per_meter, opt.dog_per_sec, opt.reserve, e2));
  MissionTask old = task(50, 1, "S4_old_P3", {25.0, 0.0, 0.0}, 3, kDog, 15.0, opt.t0);
  old.ideal_eta = 25.0;
  c.tasks.push_back(old);
  c.tasks.push_back(task(50, 2, "S4_new_P0", urgent, 0, kDog, 15.0, opt.t0 + 5.0));
  return c;
}

ScenarioCase s5(Draw & d, const ScenarioOptions & opt)
{
  // 1 架無人機＋1 隻狗，3 個平台不限的任務（P1、P2、P3）
  ScenarioCase c;
  c.max_time = 900.0;
  const double b1 = d.uniform(70.0, 100.0);
  const double b2 = d.uniform(70.0, 100.0);
  const Vec3 dog_at{d.uniform(-10.0, 10.0), d.uniform(-10.0, 10.0), 0.0};
  const double e1 = d.uniform(-opt.drain_error, opt.drain_error);
  const double e2 = d.uniform(-opt.drain_error, opt.drain_error);
  c.agents.push_back(uav(1, {0.0, 0.0, 0.0}, b1, opt, e1));
  c.agents.push_back(dog(50, dog_at, b2, opt.dog_speed, opt.dog_per_meter, opt.dog_per_sec, opt.reserve, e2));
  for (std::uint16_t k = 0; k < 3; ++k) {
    const Vec3 at{d.uniform(-30.0, 30.0), d.uniform(-30.0, 30.0), 0.0};
    const double when = k == 0 ? opt.t0 : opt.t0 + d.uniform(0.0, 30.0);
    const std::uint8_t priority = static_cast<std::uint8_t>(k + 1);
    c.tasks.push_back(task(1, static_cast<std::uint16_t>(k + 1), "S5_P" + std::to_string(priority), at,
      priority, kPlatformAny, 10.0, when));
  }
  return c;
}

ScenarioCase s6(Draw & d, const ScenarioOptions & opt)
{
  // 3 架無人機＋3 隻狗（itest yard 的出生點），12 個一般任務＋一組 3 個角色
  ScenarioCase c;
  c.max_time = 1500.0;
  const Vec3 uav_at[3] = {{-40.0, -30.0, 0.0}, {-40.0, -25.0, 0.0}, {-40.0, -20.0, 0.0}};
  const Vec3 dog_at[3] = {{-4.0, 0.0, 0.0}, {10.0, 6.0, 0.0}, {-18.0, -8.0, 0.0}};
  for (int i = 0; i < 3; ++i) {
    const double battery = d.uniform(60.0, 100.0);
    c.agents.push_back(uav(static_cast<AgentId>(i + 1), uav_at[i], battery, opt,
      d.uniform(-opt.drain_error, opt.drain_error)));
  }
  for (int i = 0; i < 3; ++i) {
    const double battery = d.uniform(60.0, 100.0);
    c.agents.push_back(dog(static_cast<AgentId>(50 + i), dog_at[i], battery, opt.dog_speed, opt.dog_per_meter,
      opt.dog_per_sec, opt.reserve, d.uniform(-opt.drain_error, opt.drain_error)));
  }
  const std::uint8_t platforms[12] = {kUav, kUav, kUav, kUav, kDog, kDog, kDog, kDog, 0, 0, 0, 0};
  for (int i = 0; i < 12; ++i) {
    const Vec3 at{d.uniform(-45.0, 30.0), d.uniform(-35.0, 20.0), 0.0};
    const double when = i < 6 ? opt.t0 : opt.t0 + d.uniform(0.0, 120.0);
    const std::uint8_t priority = static_cast<std::uint8_t>(i % 4);
    c.tasks.push_back(task(1, static_cast<std::uint16_t>(i + 1), "T" + std::to_string(i + 1), at, priority,
      platforms[i], platforms[i] == kDog ? 15.0 : 10.0, when));
  }
  // 角色組（入侵範本的簡化）：空中監看（無人機）＋前、後（狗，distinct_agent_set：不能同一隻）
  const Vec3 center{d.uniform(-35.0, 20.0), d.uniform(-25.0, 10.0), 0.0};
  const double when = opt.t0 + d.uniform(20.0, 90.0);
  const TaskId group = makeTaskId(1, 900);
  const char * names[3] = {"G_air", "G_front", "G_rear"};
  const Vec3 offsets[3] = {{0.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}};
  for (std::uint8_t r = 0; r < 3; ++r) {
    MissionTask m = task(1, static_cast<std::uint16_t>(101 + r), names[r], add(center, offsets[r]), 0,
      r == 0 ? kUav : kDog, r == 0 ? 10.0 : 15.0, when);
    m.task.ext.group_id = group;
    m.task.ext.role_index = r;
    m.task.ext.role_count = 3;
    m.task.ext.distinct_agent_set = 0x06;
    c.tasks.push_back(m);
  }
  return c;
}
}  // namespace

ScenarioCase makeScenario(const std::string & name, std::uint32_t seed, const ScenarioOptions & opt)
{
  Draw d(seed);
  ScenarioCase c;
  if (name == "S1") {
    c = s1(d, seed, opt);
  } else if (name == "S2") {
    c = s2(d, opt);
  } else if (name == "S3") {
    c = s3(d, seed, opt);
  } else if (name == "S4") {
    c = s4(d, opt);
  } else if (name == "S5") {
    c = s5(d, opt);
  } else if (name == "S6") {
    c = s6(d, opt);
  } else {
    throw std::invalid_argument("unknown scenario: " + name);
  }
  c.name = name;
  return c;
}

RunSummary summarize(const MissionResult & result)
{
  RunSummary s;
  s.tasks = static_cast<int>(result.tasks.size());
  s.preempted = result.preempted;
  s.wrong_accepts = result.wrong_accepts;
  double worst = 0.0;
  std::ostringstream map;
  for (const TaskRecord & t : result.tasks) {
    s.completed += t.done_at >= 0.0 ? 1 : 0;
    if (t.converged_at < 0.0) {
      ++s.unconverged;
    } else {
      worst = std::max(worst, t.converged_at - t.inserted);
    }
    if (t.duplicate_time > 0.0) {
      ++s.duplicate;
      s.duplicate_time += t.duplicate_time;
    }
    s.handoff += t.handoffs;
    s.handoff_unforced += t.handoffs - t.handoffs_forced;
    if (t.bidders >= 2) {
      s.score_gap = s.score_gap < 0.0 ? t.score_gap : std::min(s.score_gap, t.score_gap);
      s.near_tie += t.score_gap < 0.01 ? 1 : 0;
    }
    s.deadline_miss += t.deadline_miss ? 1 : 0;
    if (t.priority == 0 && t.eta() >= 0.0) {
      s.p0_eta = std::max(s.p0_eta, t.eta());
    }
    if (t.priority == 1 && t.eta() >= 0.0) {
      s.p1_eta = std::max(s.p1_eta, t.eta());
    }
    s.discounted_value += t.discounted_value;
    if (t.expected != kNoAgent) {
      ++s.choice_total;
      s.choice_ok += t.first_executor == t.expected ? 1 : 0;
    }
    s.old_delay = std::max(s.old_delay, t.delay);
    map << (map.tellp() > 0 ? ";" : "") << t.label << ":" << (t.done_at >= 0.0 ? t.done_by : 0);
  }
  s.converge_s = s.unconverged > 0 ? -1.0 : worst;
  s.conv_5s = s.unconverged == 0 && worst <= 5.0 && s.duplicate == 0;
  s.winner_map = map.str();
  bool first = true;
  for (const AgentRecord & a : result.agents) {
    s.energy_used += a.energy_used;
    s.bundle_drops += a.bundle_drops;
    if (first) {
      s.min_battery = a.min_battery;
      s.min_return_margin = a.return_margin;
      s.max_tasks = s.min_tasks = a.completed;
      first = false;
    } else {
      s.min_battery = std::min(s.min_battery, a.min_battery);
      s.min_return_margin = std::min(s.min_return_margin, a.return_margin);
      s.max_tasks = std::max(s.max_tasks, a.completed);
      s.min_tasks = std::min(s.min_tasks, a.completed);
    }
  }
  return s;
}

}  // namespace cbba_core
