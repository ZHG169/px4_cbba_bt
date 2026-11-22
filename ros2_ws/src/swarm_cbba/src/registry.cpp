#include "swarm_cbba/registry.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/failure_codes.hpp"
#include "cbba_core/milestones.hpp"

namespace swarm_cbba
{

namespace
{
// 錯誤訊息的位置：「fleet.yaml robots[3]」「actions.yaml inspect」
struct Where
{
  std::string file;
  std::string item;
  std::string text() const {return item.empty() ? file : file + " " + item;}
  Where sub(const std::string & s) const {return Where{file, item.empty() ? s : item + "." + s};}
};

[[noreturn]] void fail(const Where & w, const std::string & what)
{
  throw std::runtime_error("registry " + w.text() + "：" + what);
}

template<typename T>
T need(const YAML::Node & node, const char * key, const Where & w)
{
  const YAML::Node v = node[key];
  if (!v) {
    fail(w, std::string("缺欄位 ") + key);
  }
  try {
    return v.as<T>();
  } catch (const YAML::Exception &) {
    fail(w, std::string("欄位 ") + key + " 的型別不對");
  }
}

template<typename T>
T optionalField(const YAML::Node & node, const char * key, const T & fallback, const Where & w)
{
  const YAML::Node v = node[key];
  if (!v || v.IsNull()) {
    return fallback;
  }
  try {
    return v.as<T>();
  } catch (const YAML::Exception &) {
    fail(w, std::string("欄位 ") + key + " 的型別不對");
  }
}

std::optional<double> optionalNumber(const YAML::Node & node, const char * key, const Where & w)
{
  const YAML::Node v = node[key];
  if (!v || v.IsNull()) {
    return std::nullopt;
  }
  try {
    return v.as<double>();
  } catch (const YAML::Exception &) {
    fail(w, std::string("欄位 ") + key + " 要是數字");
  }
}

using Strings = std::vector<std::string>;

// 讀一份檔案、檢查 schema_version、回傳根節點
YAML::Node loadRoot(const std::string & dir, const std::string & file)
{
  const Where w{file, ""};
  YAML::Node root;
  try {
    root = YAML::LoadFile(dir + "/" + file);
  } catch (const YAML::BadFile &) {
    fail(w, "讀不到 " + dir + "/" + file);
  } catch (const YAML::Exception & e) {
    fail(w, std::string("不是合法的 YAML：") + e.what());
  }
  const auto version = need<std::string>(root, "schema_version", w);
  if (version != kRegistrySchema) {
    fail(w, "schema_version 是 " + version + "，這版程式要 " + kRegistrySchema);
  }
  return root;
}

// 根節點底下的清單（可以不在：空清單）
YAML::Node listOf(const YAML::Node & root, const char * key, const Where & w)
{
  const YAML::Node list = root[key];
  if (list && !list.IsNull() && !list.IsSequence()) {
    fail(w, std::string(key) + " 要是清單");
  }
  return (list && list.IsSequence()) ? list : YAML::Node(YAML::NodeType::Sequence);
}

// 根節點底下的「名稱 → 內容」表（可以不在：空表）
YAML::Node mapOf(const YAML::Node & root, const char * key, const Where & w)
{
  const YAML::Node map = root[key];
  if (map && !map.IsNull() && !map.IsMap()) {
    fail(w, std::string(key) + " 要是「名稱: 內容」的表");
  }
  return (map && map.IsMap()) ? map : YAML::Node(YAML::NodeType::Map);
}

Where itemOf(const std::string & file, const char * list, std::size_t i)
{
  return Where{file, std::string(list) + "[" + std::to_string(i) + "]"};
}

cbba_core::AgentId agentId(const YAML::Node & node, const Where & w)
{
  const int id = need<int>(node, "id", w);
  if (id < 1 || id > 254) {
    fail(w, "id 要是 1～254（RobotState.agent_id 是 uint8，0 = 無人）");
  }
  return static_cast<cbba_core::AgentId>(id);
}

// "P0"～"P3" → 0～3
int parsePriority(const std::string & text, const Where & w)
{
  if (text.size() == 2 && text[0] == 'P' && text[1] >= '0' && text[1] <= '3') {
    return text[1] - '0';
  }
  fail(w, "priority 要是 P0～P3（是 " + text + "）");
}

ActionParam loadParam(const std::string & name, const YAML::Node & n, const Where & w)
{
  if (!n.IsMap()) {
    fail(w, "參數要是表（type、default…）");
  }
  ActionParam p;
  p.name = name;
  p.type = need<std::string>(n, "type", w);
  p.values = optionalField<Strings>(n, "values", {}, w);
  p.min = optionalNumber(n, "min", w);
  p.max = optionalNumber(n, "max", w);
  if (n["default"] && !n["default"].IsNull()) {
    p.has_default = true;
    p.default_value = optionalField<std::string>(n, "default", "", w);
  }
  p.unit = optionalField<std::string>(n, "unit", "", w);
  p.required = optionalField<bool>(n, "required", false, w);
  p.description = optionalField<std::string>(n, "description", "", w);
  return p;
}

ActionImpl loadImpl(const std::string & platform, const YAML::Node & n, const Where & w)
{
  if (!n.IsMap()) {
    fail(w, "平台實作要是表（backend、interface／sequence、autonomy）");
  }
  ActionImpl i;
  i.platform = platform;
  i.backend = need<std::string>(n, "backend", w);
  i.interface = optionalField<std::string>(n, "interface", "", w);
  i.sequence = optionalField<Strings>(n, "sequence", {}, w);
  i.extra_preconditions = optionalField<Strings>(n, "extra_preconditions", {}, w);
  i.extra_effects = optionalField<Strings>(n, "extra_effects", {}, w);
  i.autonomy = need<std::string>(n, "autonomy", w);
  return i;
}

ActionEntry loadAction(const std::string & name, const YAML::Node & n, const Where & w)
{
  if (!n.IsMap()) {
    fail(w, "動作要是表");
  }
  ActionEntry a;
  a.name = name;
  a.cls = need<std::string>(n, "class", w);
  a.status = optionalField<std::string>(n, "status", "open", w);   // 原文的範例沒有：預設開放
  a.version = need<int>(n, "version", w);
  const YAML::Node params = n["params"];
  if (params && !params.IsNull()) {
    if (!params.IsMap()) {
      fail(w, "params 要是「參數名稱: {type…}」的表");
    }
    for (const auto & kv : params) {
      const std::string pname = kv.first.as<std::string>();
      a.params.push_back(loadParam(pname, kv.second, w.sub("params." + pname)));
    }
  }
  // location、object 是我們加的（原文的範例沒有）：沒寫時 location 選填、有 objects 時 object 選填
  a.objects = optionalField<Strings>(n, "objects", {}, w);
  a.location = optionalField<std::string>(n, "location", "optional", w);
  a.object = optionalField<std::string>(n, "object", a.objects.empty() ? "none" : "optional", w);
  const YAML::Node req = n["requires"];
  if (req && !req.IsNull()) {
    if (!req.IsMap()) {
      fail(w, "requires 要是表（any_of）");
    }
    a.requires_any_of = optionalField<std::vector<Strings>>(req, "any_of", {}, w);
  }
  a.preconditions = optionalField<Strings>(n, "preconditions", {}, w);
  a.effects = optionalField<Strings>(n, "effects", {}, w);
  a.timeout_s = need<double>(n, "timeout_s", w);
  a.cancelable = need<bool>(n, "cancelable", w);
  a.risk = need<std::string>(n, "risk", w);
  a.failure_codes = optionalField<Strings>(n, "failure_codes", {}, w);
  const YAML::Node impls = n["implementations"];
  if (!impls || !impls.IsMap()) {
    fail(w, "implementations 要是「平台: 實作」的表");
  }
  for (const auto & kv : impls) {
    const std::string platform = kv.first.as<std::string>();
    a.implementations.push_back(loadImpl(platform, kv.second, w.sub("implementations." + platform)));
  }
  return a;
}

ObjectClass loadObjectClass(const std::string & name, const YAML::Node & n, const Where & w)
{
  if (!n.IsMap()) {
    fail(w, "物件類別要是表");
  }
  ObjectClass c;
  c.name = name;
  c.locate = need<Strings>(n, "locate", w);
  c.allowed_actions = need<Strings>(n, "allowed_actions", w);
  const YAML::Node safety = n["safety_distance_m"];
  if (!safety) {
    fail(w, "缺欄位 safety_distance_m");
  }
  if (safety.IsScalar()) {
    if (safety.as<std::string>() != "per_instance") {
      fail(w, "safety_distance_m 要是 {平台: 公尺} 或 per_instance");
    }
    c.safety_per_instance = true;
  } else if (safety.IsMap()) {
    for (const auto & kv : safety) {
      const std::string platform = kv.first.as<std::string>();
      if (kv.second.IsNull()) {
        c.safety_distance_m[platform] = std::nullopt;
      } else {
        try {
          c.safety_distance_m[platform] = kv.second.as<double>();
        } catch (const YAML::Exception &) {
          fail(w, "safety_distance_m." + platform + " 要是數字或 null");
        }
      }
    }
  } else {
    fail(w, "safety_distance_m 要是 {平台: 公尺} 或 per_instance");
  }
  c.rules = optionalField<Strings>(n, "rules", {}, w);
  c.findings = optionalField<Strings>(n, "findings", {}, w);
  c.note = optionalField<std::string>(n, "note", "", w);
  return c;
}

EventType loadEventType(const std::string & name, const YAML::Node & n, const Where & w)
{
  if (!n.IsMap()) {
    fail(w, "事件類型要是表");
  }
  EventType e;
  e.name = name;
  const int id = optionalField<int>(n, "id", 0, w);
  if (id < 0 || id > 0xFFFF) {
    fail(w, "id 要是 1～65535（0 = 沒有編號）");
  }
  e.id = static_cast<std::uint16_t>(id);
  e.category = need<std::string>(n, "category", w);
  e.detectors = optionalField<Strings>(n, "detectors", {}, w);
  if (const YAML::Node t = n["trigger"]) {
    e.zone_attr = optionalField<std::string>(t, "zone_attr", "any", w.sub("trigger"));
    e.time_rule = optionalField<std::string>(t, "time_rule", "always", w.sub("trigger"));
  }
  if (const YAML::Node a = n["accumulate"]) {
    const Where aw = w.sub("accumulate");
    e.has_accumulate = true;
    e.lambda = need<double>(a, "lambda", aw);
    e.theta_on = need<double>(a, "theta_on", aw);
    e.theta_off = need<double>(a, "theta_off", aw);
    e.min_duration_s = optionalField<double>(a, "min_duration_s", 0.0, aw);
  }
  if (const YAML::Node a = n["associate"]) {
    const Where aw = w.sub("associate");
    e.has_associate = true;
    e.associate_dt_s = need<double>(a, "dt_s", aw);
    e.associate_r_m = need<double>(a, "r_m", aw);
  }
  if (const YAML::Node v = n["vlm"]) {
    const Where vw = w.sub("vlm");
    e.has_vlm = true;
    e.vlm_prompt_id = need<std::string>(v, "prompt_id", vw);
    e.vlm_samples = optionalField<int>(v, "samples", 1, vw);
    e.vlm_timeout_s = need<double>(v, "timeout_s", vw);
  }
  if (const YAML::Node c = n["cost"]) {
    const Where cw = w.sub("cost");
    e.has_cost = true;
    e.cost_miss = need<double>(c, "miss", cw);
    e.cost_false = need<double>(c, "false", cw);
  }
  e.locate = optionalField<std::string>(n, "locate", "", w);
  e.default_rule = optionalField<std::string>(n, "default_rule", "", w);
  e.priority = parsePriority(need<std::string>(n, "priority", w), w);
  e.requires_approval = optionalField<bool>(n, "requires_approval", false, w);
  e.object_class = optionalField<std::string>(n, "object_class", "", w);
  e.expires_s = optionalField<double>(n, "expires_s", 0.0, w);
  e.status = optionalField<std::string>(n, "status", "open", w);   // 我們加的：預設啟用
  return e;
}

bool oneOf(const std::string & v, std::initializer_list<const char *> allowed)
{
  for (const char * a : allowed) {
    if (v == a) {
      return true;
    }
  }
  return false;
}

bool contains(const Strings & list, const std::string & v)
{
  for (const auto & s : list) {
    if (s == v) {
      return true;
    }
  }
  return false;
}

std::string fmt(double v)
{
  std::ostringstream os;
  os << v;
  return os.str();
}
}  // namespace

const ActionImpl * ActionEntry::implementation(const std::string & platform) const
{
  for (const ActionImpl & i : implementations) {
    if (i.platform == platform) {
      return &i;
    }
  }
  return nullptr;
}

const ActionParam * ActionEntry::param(const std::string & n) const
{
  for (const ActionParam & p : params) {
    if (p.name == n) {
      return &p;
    }
  }
  return nullptr;
}

const RobotEntry * Registry::robot(cbba_core::AgentId id) const
{
  for (const RobotEntry & r : robots) {
    if (r.id == id) {
      return &r;
    }
  }
  return nullptr;
}

const ActionEntry * Registry::action(const std::string & name) const
{
  for (const ActionEntry & a : actions) {
    if (a.name == name) {
      return &a;
    }
  }
  return nullptr;
}

const PlatformEntry * Registry::platform(const std::string & name) const
{
  for (const PlatformEntry & p : platforms) {
    if (p.name == name) {
      return &p;
    }
  }
  return nullptr;
}

const ObjectClass * Registry::objectClass(const std::string & name) const
{
  for (const ObjectClass & c : object_classes) {
    if (c.name == name) {
      return &c;
    }
  }
  return nullptr;
}

const EventType * Registry::eventType(const std::string & name) const
{
  for (const EventType & e : event_types) {
    if (e.name == name) {
      return &e;
    }
  }
  return nullptr;
}

const SemanticEntry * Registry::taskType(const std::string & name) const
{
  for (const SemanticEntry & e : task_types) {
    if (e.name == name) {
      return &e;
    }
  }
  return nullptr;
}

const SemanticEntry * Registry::role(const std::string & name) const
{
  for (const SemanticEntry & e : roles) {
    if (e.name == name) {
      return &e;
    }
  }
  return nullptr;
}

namespace
{
// task_types.yaml 的一張表：「名稱 → {id, description, deprecated}」
std::vector<SemanticEntry> loadSemantic(const YAML::Node & root, const std::string & file, const std::string & table)
{
  std::vector<SemanticEntry> out;
  for (const auto & kv : mapOf(root, table.c_str(), Where{file, ""})) {
    SemanticEntry e;
    e.name = kv.first.as<std::string>();
    const Where w{file, table + "." + e.name};
    if (!kv.second.IsMap()) {
      fail(w, "要是表（{id: …, description: …}）");
    }
    const int id = need<int>(kv.second, "id", w);
    if (id < 1 || id > 0xFFFF) {
      fail(w, "id 要是 1～65535（0 保留給「不知道」）");
    }
    e.id = static_cast<std::uint16_t>(id);
    e.description = optionalField<std::string>(kv.second, "description", "", w);
    e.deprecated = optionalField<bool>(kv.second, "deprecated", false, w);
    out.push_back(e);
  }
  return out;
}
}  // namespace

Registry loadRegistry(const std::string & dir)
{
  Registry reg;
  reg.dir = dir;

  {
    const std::string file = "tools.yaml";
    const YAML::Node tools = listOf(loadRoot(dir, file), "capabilities", Where{file, ""});
    for (std::size_t i = 0; i < tools.size(); ++i) {
      const Where w = itemOf(file, "capabilities", i);
      ToolEntry t;
      t.name = need<std::string>(tools[i], "name", w);
      const int bit = need<int>(tools[i], "bit", w);
      if (bit < 0 || bit > 63) {
        fail(w, "bit 要是 0～63");
      }
      t.bit = static_cast<unsigned>(bit);
      t.deprecated = optionalField<bool>(tools[i], "deprecated", false, w);
      t.description = optionalField<std::string>(tools[i], "description", "", w);
      reg.tools.push_back(t);
    }
  }

  {
    const std::string file = "platforms.yaml";
    const YAML::Node platforms = listOf(loadRoot(dir, file), "platforms", Where{file, ""});
    for (std::size_t i = 0; i < platforms.size(); ++i) {
      const Where w = itemOf(file, "platforms", i);
      PlatformEntry p;
      p.name = need<std::string>(platforms[i], "name", w);
      const int code = need<int>(platforms[i], "code", w);
      if (code < 1 || code > 255) {
        fail(w, "code 要是 1～255（0 保留給 ANY／閘道）");
      }
      p.code = static_cast<std::uint8_t>(code);
      p.description = optionalField<std::string>(platforms[i], "description", "", w);
      p.cruise_speed = need<double>(platforms[i], "cruise_speed", w);
      p.energy_per_meter = need<double>(platforms[i], "energy_per_meter", w);
      p.hover_energy_per_sec = need<double>(platforms[i], "hover_energy_per_sec", w);
      p.safety_reserve = need<double>(platforms[i], "safety_reserve", w);
      p.endurance_s = need<double>(platforms[i], "endurance_s", w);
      const int bundle = need<int>(platforms[i], "max_bundle", w);
      if (bundle < 1) {
        fail(w, "max_bundle 至少 1");
      }
      p.max_bundle = static_cast<std::size_t>(bundle);
      p.must_return = need<bool>(platforms[i], "must_return", w);
      reg.platforms.push_back(p);
    }
  }

  {
    const std::string file = "fleet.yaml";
    const YAML::Node root = loadRoot(dir, file);
    const YAML::Node robots = listOf(root, "robots", Where{file, ""});
    for (std::size_t i = 0; i < robots.size(); ++i) {
      const Where w = itemOf(file, "robots", i);
      RobotEntry r;
      r.id = agentId(robots[i], w);
      r.name = need<std::string>(robots[i], "name", w);
      r.platform = need<std::string>(robots[i], "platform", w);
      r.ns = optionalField<std::string>(robots[i], "ns", "", w);
      r.capabilities = optionalField<Strings>(robots[i], "capabilities", {}, w);
      const int floor = optionalField<int>(robots[i], "floor", 0, w);
      if (floor < 0 || floor > 255) {
        fail(w, "floor 要是 0～255（0 = 不知道）");
      }
      r.floor = static_cast<std::uint8_t>(floor);
      if (const YAML::Node l = robots[i]["limits"]) {
        const Where lw = w.sub("limits");
        r.limits.max_altitude_m = optionalNumber(l, "max_altitude_m", lw);
        r.limits.max_flight_s = optionalNumber(l, "max_flight_s", lw);
        r.limits.landing_pads = optionalField<Strings>(l, "landing_pads", {}, lw);
        r.limits.docks = optionalField<Strings>(l, "docks", {}, lw);
      }
      reg.robots.push_back(r);
    }
    const YAML::Node gateways = listOf(root, "gateways", Where{file, ""});
    for (std::size_t i = 0; i < gateways.size(); ++i) {
      const Where w = itemOf(file, "gateways", i);
      GatewayEntry g;
      g.id = agentId(gateways[i], w);
      g.name = need<std::string>(gateways[i], "name", w);
      g.ns = optionalField<std::string>(gateways[i], "ns", "", w);
      reg.gateways.push_back(g);
    }
  }

  {
    const std::string file = "actions.yaml";
    const YAML::Node root = loadRoot(dir, file);
    const Where top{file, ""};
    reg.common_failure_codes = optionalField<Strings>(root, "common_failure_codes", {}, top);
    const YAML::Node conditions = listOf(root, "conditions", top);
    for (std::size_t i = 0; i < conditions.size(); ++i) {
      const Where w = itemOf(file, "conditions", i);
      reg.conditions.push_back(ConditionEntry{need<std::string>(conditions[i], "name", w),
          optionalField<std::string>(conditions[i], "description", "", w)});
    }
    const YAML::Node milestones = listOf(root, "milestones", top);
    for (std::size_t i = 0; i < milestones.size(); ++i) {
      const Where w = itemOf(file, "milestones", i);
      MilestoneEntry m;
      m.name = need<std::string>(milestones[i], "name", w);
      const int bit = need<int>(milestones[i], "bit", w);
      if (bit < 0 || bit > 15) {
        fail(w, "bit 要是 0～15（TASK_PROGRESS 的 milestones 是 16 位元）");
      }
      m.bit = static_cast<unsigned>(bit);
      m.kind = need<std::string>(milestones[i], "kind", w);
      m.description = optionalField<std::string>(milestones[i], "description", "", w);
      reg.milestones.push_back(m);
    }
    for (const auto & kv : mapOf(root, "actions", top)) {
      const std::string name = kv.first.as<std::string>();
      reg.actions.push_back(loadAction(name, kv.second, Where{file, name}));
    }
  }

  {
    const std::string file = "objects.yaml";
    const YAML::Node root = loadRoot(dir, file);
    for (const auto & kv : mapOf(root, "classes", Where{file, ""})) {
      const std::string name = kv.first.as<std::string>();
      reg.object_classes.push_back(loadObjectClass(name, kv.second, Where{file, name}));
    }
  }

  {
    const std::string file = "event_types.yaml";
    const YAML::Node root = loadRoot(dir, file);
    for (const auto & kv : mapOf(root, "detectors", Where{file, ""})) {
      const std::string name = kv.first.as<std::string>();
      const Where w{file, "detectors." + name};
      DetectorEntry d;
      d.name = name;
      d.labels = need<Strings>(kv.second, "labels", w);
      d.source_kinds = need<Strings>(kv.second, "source_kinds", w);
      d.min_score = need<double>(kv.second, "min_score", w);
      d.status = need<std::string>(kv.second, "status", w);
      reg.detectors.push_back(d);
    }
    for (const auto & kv : mapOf(root, "event_types", Where{file, ""})) {
      const std::string name = kv.first.as<std::string>();
      reg.event_types.push_back(loadEventType(name, kv.second, Where{file, name}));
    }
  }

  {
    const std::string file = "task_types.yaml";
    const YAML::Node root = loadRoot(dir, file);
    reg.task_types = loadSemantic(root, file, "task_types");
    reg.roles = loadSemantic(root, file, "roles");
  }
  return reg;
}

std::vector<std::string> checkRegistry(const Registry & reg)
{
  std::vector<std::string> errors;

  // tools：不重複，和 capabilities.hpp 一致
  std::map<std::string, const ToolEntry *> tool_by_name;
  std::map<unsigned, std::string> name_by_bit;
  for (const ToolEntry & t : reg.tools) {
    if (!tool_by_name.emplace(t.name, &t).second) {
      errors.push_back("tools.yaml：能力 " + t.name + " 重複");
    }
    const auto bit = name_by_bit.emplace(t.bit, t.name);
    if (!bit.second) {
      errors.push_back("tools.yaml：bit " + std::to_string(t.bit) + " 同時給了 " + bit.first->second + " 和 " + t.name);
    }
    if (const auto cap = cbba_core::capabilityFromName(t.name)) {
      const unsigned expect = static_cast<unsigned>(*cap);
      if (t.bit != expect) {
        errors.push_back("tools.yaml：" + t.name + " 的 bit 是 " + std::to_string(t.bit) +
          "，程式（cbba_core/capabilities.hpp）是 " + std::to_string(expect));
      }
    }
  }
  for (unsigned bit = 0; bit < 64; ++bit) {
    const auto caps = cbba_core::capabilityNames(cbba_core::Capabilities{1} << bit);
    const std::string & name = caps.front();
    if (name.rfind("bit", 0) == 0) {
      continue;   // 程式不認得的位元
    }
    if (tool_by_name.count(name) == 0) {
      errors.push_back("tools.yaml：少了程式認得的能力 " + name + "（bit " + std::to_string(bit) + "）");
    }
  }
  const auto capabilityOk = [&tool_by_name](const std::string & c) {
      const auto t = tool_by_name.find(c);
      return t != tool_by_name.end() && !t->second->deprecated;
    };

  // platforms
  std::set<std::string> platform_names;
  std::set<unsigned> codes;
  for (const PlatformEntry & p : reg.platforms) {
    if (!platform_names.insert(p.name).second) {
      errors.push_back("platforms.yaml：平台 " + p.name + " 重複");
    }
    if (!codes.insert(p.code).second) {
      errors.push_back("platforms.yaml：code " + std::to_string(p.code) + " 重複");
    }
  }

  // robots、gateways
  std::set<cbba_core::AgentId> ids;
  for (const RobotEntry & r : reg.robots) {
    if (!ids.insert(r.id).second) {
      errors.push_back("fleet.yaml：機號 " + std::to_string(r.id) + " 重複");
    }
    if (platform_names.count(r.platform) == 0) {
      errors.push_back("fleet.yaml：" + r.name + " 的平台 " + r.platform + " 不在 platforms.yaml");
    }
    for (const std::string & c : r.capabilities) {
      const auto t = tool_by_name.find(c);
      if (t == tool_by_name.end()) {
        errors.push_back("fleet.yaml：" + r.name + " 的能力 " + c + " 不在 tools.yaml");
      } else if (t->second->deprecated) {
        errors.push_back("fleet.yaml：" + r.name + " 的能力 " + c + " 已經 deprecated");
      }
    }
    if ((r.limits.max_altitude_m && !(*r.limits.max_altitude_m > 0.0)) ||
      (r.limits.max_flight_s && !(*r.limits.max_flight_s > 0.0)))
    {
      errors.push_back("fleet.yaml：" + r.name + " 的 limits（max_altitude_m、max_flight_s）要 > 0");
    }
  }
  for (const GatewayEntry & g : reg.gateways) {
    if (!ids.insert(g.id).second) {
      errors.push_back("fleet.yaml：閘道 " + g.name + " 的機號 " + std::to_string(g.id) + " 和別台重複");
    }
  }

  // actions
  for (const std::string & code : reg.common_failure_codes) {
    if (!cbba_core::failureCodeInfo(code)) {
      errors.push_back("actions.yaml：common_failure_codes 的 " + code + " 不是 8 個失敗碼之一");
    }
  }
  std::set<std::string> condition_names;
  for (const ConditionEntry & c : reg.conditions) {
    if (!condition_names.insert(c.name).second) {
      errors.push_back("actions.yaml：condition " + c.name + " 重複");
    }
  }
  std::set<std::string> action_names;
  for (const ActionEntry & a : reg.actions) {
    action_names.insert(a.name);
  }
  for (const ActionEntry & a : reg.actions) {
    const std::string at = "actions.yaml：" + a.name;
    if (!oneOf(a.cls, {"plannable", "system", "internal"})) {
      errors.push_back(at + " 的 class " + a.cls + " 要是 plannable／system／internal");
    }
    if (!oneOf(a.status, {"open", "closed"})) {
      errors.push_back(at + " 的 status " + a.status + " 要是 open／closed");
    }
    if (a.version < 1) {
      errors.push_back(at + " 的 version 要 ≥ 1");
    }
    if (!oneOf(a.location, {"required", "optional", "none"})) {
      errors.push_back(at + " 的 location " + a.location + " 要是 required／optional／none");
    }
    if (!oneOf(a.object, {"required", "optional", "none"})) {
      errors.push_back(at + " 的 object " + a.object + " 要是 required／optional／none");
    }
    if (a.object == "none" && !a.objects.empty()) {
      errors.push_back(at + " 的 object 是 none，objects 要是空的");
    }
    if (!oneOf(a.risk, {"low", "medium", "high"})) {
      errors.push_back(at + " 的 risk " + a.risk + " 要是 low／medium／high");
    }
    if (a.timeout_s < 0.0) {
      errors.push_back(at + " 的 timeout_s 不能是負的");
    }
    if (!a.cancelable) {
      errors.push_back(at + " 不可取消（2-1b：每個動作都必須可取消）");
    }
    for (const Strings & group : a.requires_any_of) {
      if (group.empty()) {
        errors.push_back(at + " 的 requires.any_of 有空的一組");
      }
      for (const std::string & c : group) {
        if (!capabilityOk(c)) {
          errors.push_back(at + " 的能力 " + c + " 不在 tools.yaml（或已經 deprecated）");
        }
      }
    }
    const auto checkConditions = [&](const Strings & list, const std::string & field) {
        for (const std::string & c : list) {
          if (condition_names.count(c) == 0) {
            errors.push_back(at + " 的 " + field + " " + c + " 不在 conditions");
          }
        }
      };
    checkConditions(a.preconditions, "precondition");
    checkConditions(a.effects, "effect");
    for (const std::string & code : a.failure_codes) {
      if (!cbba_core::failureCodeInfo(code)) {
        errors.push_back(at + " 的失敗碼 " + code + " 不是 8 個失敗碼之一");
      }
    }
    for (std::size_t i = 0; i < a.params.size(); ++i) {
      const ActionParam & p = a.params[i];
      const std::string pat = at + " 的參數 " + p.name;
      for (std::size_t k = 0; k < i; ++k) {
        if (a.params[k].name == p.name) {
          errors.push_back(pat + " 重複");
        }
      }
      if (!oneOf(p.type, {"float", "int", "string", "bool", "enum", "string_list", "waypoints", "bbox"})) {
        errors.push_back(pat + " 的 type " + p.type + " 不認得");
      }
      if (p.type == "enum" && p.values.empty()) {
        errors.push_back(pat + " 是 enum，要有 values");
      }
      if (p.type == "enum" && p.has_default && !contains(p.values, p.default_value)) {
        errors.push_back(pat + " 的 default " + p.default_value + " 不在 values");
      }
      if (p.min && p.max && *p.min > *p.max) {
        errors.push_back(pat + " 的 min > max");
      }
      if (p.has_default && (p.type == "float" || p.type == "int")) {
        try {
          const double v = std::stod(p.default_value);
          if ((p.min && v < *p.min) || (p.max && v > *p.max)) {
            errors.push_back(pat + " 的 default " + p.default_value + " 超出範圍");
          }
        } catch (const std::exception &) {
          errors.push_back(pat + " 的 default " + p.default_value + " 不是數字");
        }
      }
      if (p.required && p.has_default) {
        errors.push_back(pat + " 是 required，不能有 default");
      }
    }
    if (a.implementations.empty()) {
      errors.push_back(at + " 沒有任何平台實作");
    }
    for (const ActionImpl & impl : a.implementations) {
      const std::string iat = at + " 的 " + impl.platform + " 實作";
      if (platform_names.count(impl.platform) == 0) {
        errors.push_back(iat + "：平台不在 platforms.yaml");
      }
      if (!oneOf(impl.autonomy, {"not_started", "in_development", "sim_verified", "field_verified"})) {
        errors.push_back(iat + "：autonomy " + impl.autonomy + " 要是 not_started／in_development／sim_verified／field_verified");
      }
      if (impl.interface.empty() && impl.sequence.empty()) {
        errors.push_back(iat + "：要有 interface 或 sequence");
      }
      checkConditions(impl.extra_preconditions, impl.platform + " 的 extra_precondition");
      checkConditions(impl.extra_effects, impl.platform + " 的 extra_effect");
      for (const std::string & step : impl.sequence) {
        const ActionEntry * s = reg.action(step);
        if (s == nullptr || s->cls != "internal") {
          errors.push_back(iat + "：sequence 的 " + step + " 不是平台內部動作（class internal）");
        } else if (s->implementation(impl.platform) == nullptr) {
          errors.push_back(iat + "：sequence 的 " + step + " 沒有 " + impl.platform + " 的實作");
        }
      }
    }
    // 動作的 objects ⇔ 物件類別的 allowed_actions
    for (const std::string & cls : a.objects) {
      const ObjectClass * c = reg.objectClass(cls);
      if (c == nullptr) {
        errors.push_back(at + " 的 objects " + cls + " 不在 objects.yaml");
      } else if (!contains(c->allowed_actions, a.name)) {
        errors.push_back(at + " 的 objects 有 " + cls + "，但 objects.yaml 的 " + cls + " 的 allowed_actions 沒有 " + a.name);
      }
    }
  }

  // milestones（CB-4）：和 cbba_core/milestones.hpp 一致
  std::set<std::string> milestone_names;
  std::set<unsigned> milestone_bits;
  for (const MilestoneEntry & m : reg.milestones) {
    const std::string at = "actions.yaml：里程碑 " + m.name;
    if (!milestone_names.insert(m.name).second) {
      errors.push_back(at + " 重複");
    }
    if (!milestone_bits.insert(m.bit).second) {
      errors.push_back(at + " 的 bit " + std::to_string(m.bit) + " 重複");
    }
    if (!oneOf(m.kind, {"executor", "event", "completion"})) {
      errors.push_back(at + " 的 kind " + m.kind + " 要是 executor／event／completion");
    }
    if (const auto code = cbba_core::milestoneFromName(m.name)) {
      const unsigned expect = static_cast<unsigned>(*code);
      const cbba_core::MilestoneMask b = cbba_core::milestoneBit(*code);
      const char * kind = (b & cbba_core::kEventMilestones) != 0 ? "event" :
        *code == cbba_core::Milestone::COMPLETED ? "completion" : "executor";
      if (m.bit != expect) {
        errors.push_back(at + " 的 bit 是 " + std::to_string(m.bit) + "，程式（cbba_core/milestones.hpp）是 " +
          std::to_string(expect));
      }
      if (m.kind != kind) {
        errors.push_back(at + " 的 kind 是 " + m.kind + "，程式是 " + kind);
      }
    }
  }
  for (unsigned i = 0; i < cbba_core::kMilestoneCount; ++i) {
    const char * name = cbba_core::toString(static_cast<cbba_core::Milestone>(i));
    if (milestone_names.count(name) == 0) {
      errors.push_back(std::string("actions.yaml：少了程式認得的里程碑 ") + name + "（bit " + std::to_string(i) + "）");
    }
  }

  // objects
  for (const ObjectClass & c : reg.object_classes) {
    const std::string at = "objects.yaml：" + c.name;
    if (c.locate.empty()) {
      errors.push_back(at + " 沒有 locate");
    }
    for (const std::string & l : c.locate) {
      if (!oneOf(l, {"projection", "onboard", "asset", "asset_pose", "site_map", "zone"})) {
        errors.push_back(at + " 的 locate " + l + " 要是 projection／onboard／asset／asset_pose／site_map／zone");
      }
    }
    for (const std::string & name : c.allowed_actions) {
      const ActionEntry * a = reg.action(name);
      if (a == nullptr) {
        errors.push_back(at + " 的 allowed_actions " + name + " 不在 actions.yaml");
      } else if (!contains(a->objects, c.name)) {
        errors.push_back(at + " 的 allowed_actions 有 " + name + "，但 actions.yaml 的 " + name + " 的 objects 沒有 " + c.name);
      }
    }
    for (const auto & kv : c.safety_distance_m) {
      if (platform_names.count(kv.first) == 0) {
        errors.push_back(at + " 的 safety_distance_m 的平台 " + kv.first + " 不在 platforms.yaml");
      }
      if (kv.second && *kv.second < 0.0) {
        errors.push_back(at + " 的 safety_distance_m." + kv.first + " 不能是負的");
      }
    }
    for (const std::string & r : c.rules) {
      if (!oneOf(r, {"no_overfly", "no_contact", "no_enter", "avoid_ground", "approach_needs_approval",
          "high_voltage_zone"}))
      {
        errors.push_back(at + " 的 rule " + r + " 不認得");
      }
    }
    for (const std::string & f : c.findings) {
      if (!oneOf(f, {"image", "reading", "detection"})) {
        errors.push_back(at + " 的 findings " + f + " 要是 image／reading／detection");
      }
    }
  }

  // detectors
  constexpr std::size_t kMaxName = 64;   // DETECTION 封包的 detector、label 上限（wire_detection.hpp）
  std::set<std::string> detector_names;
  for (const DetectorEntry & d : reg.detectors) {
    const std::string at = "event_types.yaml：偵測器 " + d.name;
    detector_names.insert(d.name);
    if (d.name.empty() || d.name.size() > kMaxName) {
      errors.push_back(at + " 的名稱要是 1～64 B");
    }
    if (d.labels.empty()) {
      errors.push_back(at + " 沒有 labels");
    }
    for (const std::string & l : d.labels) {
      if (l.empty() || l.size() > kMaxName) {
        errors.push_back(at + " 的 label 要是 1～64 B");
      }
    }
    for (const std::string & k : d.source_kinds) {
      if (!oneOf(k, {"fixed_camera", "robot_camera", "vms_analytics", "scada", "sensor"})) {
        errors.push_back(at + " 的 source_kinds " + k + " 要是 fixed_camera／robot_camera／vms_analytics／scada／sensor");
      }
    }
    if (d.min_score < 0.0 || d.min_score > 1.0) {
      errors.push_back(at + " 的 min_score 要在 0～1");
    }
    if (!oneOf(d.status, {"available", "planned"})) {
      errors.push_back(at + " 的 status 要是 available／planned");
    }
  }

  // event_types
  for (const EventType & e : reg.event_types) {
    const std::string at = "event_types.yaml：" + e.name;
    if (!oneOf(e.category, {"template", "open"})) {
      errors.push_back(at + " 的 category " + e.category + " 要是 template／open");
    }
    if (e.category == "template") {
      if (e.detectors.empty()) {
        errors.push_back(at + " 是 template 事件，要有 detectors");
      }
      if (e.default_rule.empty()) {
        errors.push_back(at + " 是 template 事件，要有 default_rule");
      }
    }
    if (e.category == "open" && (!e.has_vlm || !e.requires_approval)) {
      errors.push_back(at + " 是 open 事件，要有 vlm 而且 requires_approval: true");
    }
    for (const std::string & d : e.detectors) {
      if (detector_names.count(d) == 0) {
        errors.push_back(at + " 的偵測器 " + d + " 不在 detectors");
      }
    }
    if (!oneOf(e.zone_attr, {"any", "restricted", "high_voltage"})) {
      errors.push_back(at + " 的 trigger.zone_attr " + e.zone_attr + " 要是 any／restricted／high_voltage");
    }
    if (!oneOf(e.time_rule, {"always", "off_hours", "on_hours"})) {
      errors.push_back(at + " 的 trigger.time_rule " + e.time_rule + " 要是 always／off_hours／on_hours");
    }
    if (e.has_accumulate && (!(e.lambda > 0.0) || e.lambda > 1.0 || !(e.theta_on > e.theta_off) ||
      e.theta_off < 0.0 || e.min_duration_s < 0.0))
    {
      errors.push_back(at + " 的 accumulate 要 0 < lambda ≤ 1、theta_on > theta_off ≥ 0、min_duration_s ≥ 0");
    }
    if (e.has_associate && (!(e.associate_dt_s > 0.0) || e.associate_r_m < 0.0)) {
      errors.push_back(at + " 的 associate 要 dt_s > 0、r_m ≥ 0");
    }
    if (e.has_vlm && (e.vlm_prompt_id.empty() || e.vlm_samples < 1 || !(e.vlm_timeout_s > 0.0))) {
      errors.push_back(at + " 的 vlm 要有 prompt_id、samples ≥ 1、timeout_s > 0");
    }
    if (e.has_cost && (!(e.cost_miss > 0.0) || !(e.cost_false > 0.0))) {
      errors.push_back(at + " 的 cost（miss、false）要 > 0");
    }
    if (!e.locate.empty() && !oneOf(e.locate, {"projection", "asset", "zone"})) {
      errors.push_back(at + " 的 locate " + e.locate + " 要是 projection／asset／zone");
    }
    if (!e.object_class.empty() && reg.objectClass(e.object_class) == nullptr) {
      errors.push_back(at + " 的 object_class " + e.object_class + " 不在 objects.yaml");
    }
    if (e.expires_s < 0.0) {
      errors.push_back(at + " 的 expires_s 不能是負的");
    }
    if (!oneOf(e.status, {"open", "closed"})) {
      errors.push_back(at + " 的 status 要是 open／closed");
    }
  }
  {
    // 編號（任務欄位補齊 02）：不重複；0 = 沒寫，可以多個
    std::map<std::uint16_t, std::string> by_id;
    for (const EventType & e : reg.event_types) {
      if (e.id == 0) {
        continue;
      }
      const auto it = by_id.emplace(e.id, e.name);
      if (!it.second) {
        errors.push_back("event_types.yaml：id " + std::to_string(e.id) + " 同時給了 " + it.first->second + " 和 " + e.name);
      }
    }
  }

  // task_types.yaml：名稱、編號不重複
  for (const auto & [table, entries] : {std::make_pair(std::string("task_types"), &reg.task_types),
      std::make_pair(std::string("roles"), &reg.roles)})
  {
    std::set<std::string> names;
    std::map<std::uint16_t, std::string> by_id;
    for (const SemanticEntry & e : *entries) {
      if (!names.insert(e.name).second) {
        errors.push_back("task_types.yaml：" + table + " 的 " + e.name + " 重複");
      }
      const auto it = by_id.emplace(e.id, e.name);
      if (!it.second) {
        errors.push_back("task_types.yaml：" + table + " 的 id " + std::to_string(e.id) + " 同時給了 " +
          it.first->second + " 和 " + e.name);
      }
    }
  }
  return errors;
}

std::vector<std::string> unknownTools(const Registry & reg)
{
  std::vector<std::string> out;
  for (const ToolEntry & t : reg.tools) {
    if (!t.deprecated && !cbba_core::capabilityFromName(t.name)) {
      out.push_back(t.name);
    }
  }
  return out;
}

// ============================================================
// 場域設定
// ============================================================

namespace
{
[[noreturn]] void failSite(const std::string & file, const std::string & what)
{
  throw std::runtime_error("site " + file + "：" + what);
}

// 簡單的 CSV：# 開頭與空行略過；第一個非註解行是欄位名稱；欄位內不能有逗號（不支援引號）
struct Csv
{
  std::vector<std::string> header;
  std::vector<std::map<std::string, std::string>> rows;
  std::vector<int> lines;
};

Csv readCsv(const std::string & dir, const std::string & file)
{
  std::ifstream in(dir + "/" + file);
  if (!in) {
    failSite(file, "讀不到 " + dir + "/" + file);
  }
  Csv csv;
  std::string line;
  int n = 0;
  while (std::getline(in, line)) {
    ++n;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      cells.push_back(cell);
    }
    if (!line.empty() && line.back() == ',') {
      cells.emplace_back();
    }
    if (csv.header.empty()) {
      csv.header = cells;
      continue;
    }
    if (cells.size() != csv.header.size()) {
      failSite(file, "第 " + std::to_string(n) + " 行有 " + std::to_string(cells.size()) + " 欄，欄位名稱有 " +
        std::to_string(csv.header.size()) + " 欄");
    }
    std::map<std::string, std::string> row;
    for (std::size_t i = 0; i < cells.size(); ++i) {
      row[csv.header[i]] = cells[i];
    }
    csv.rows.push_back(row);
    csv.lines.push_back(n);
  }
  return csv;
}

struct CsvRow
{
  const std::string & file;
  const std::map<std::string, std::string> & row;
  int line;

  std::string text(const char * key) const
  {
    const auto it = row.find(key);
    if (it == row.end()) {
      failSite(file, std::string("缺欄位 ") + key);
    }
    return it->second;
  }
  std::string need(const char * key) const
  {
    const std::string v = text(key);
    if (v.empty()) {
      failSite(file, "第 " + std::to_string(line) + " 行的 " + key + " 是空的");
    }
    return v;
  }
  std::optional<double> number(const char * key) const
  {
    const std::string v = text(key);
    if (v.empty()) {
      return std::nullopt;
    }
    try {
      std::size_t used = 0;
      const double d = std::stod(v, &used);
      if (used != v.size()) {
        throw std::invalid_argument(v);
      }
      return d;
    } catch (const std::exception &) {
      failSite(file, "第 " + std::to_string(line) + " 行的 " + key + " 要是數字（是 " + v + "）");
    }
  }
  double needNumber(const char * key) const
  {
    need(key);
    return *number(key);
  }
};
}  // namespace

bool Zone::contains(double x, double y) const
{
  // 射線法；邊界上的點不保證
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto & a = polygon[i];
    const auto & b = polygon[j];
    if ((a[1] > y) != (b[1] > y) && x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]) {
      inside = !inside;
    }
  }
  return inside;
}

const Zone * Site::zone(const std::string & id) const
{
  for (const Zone & z : zones) {
    if (z.id == id) {
      return &z;
    }
  }
  return nullptr;
}

const Device * Site::device(const std::string & id) const
{
  for (const Device & d : devices) {
    if (d.id == id) {
      return &d;
    }
  }
  return nullptr;
}

const Dock * Site::dock(const std::string & id) const
{
  for (const Dock & d : docks) {
    if (d.id == id) {
      return &d;
    }
  }
  return nullptr;
}

Site loadSite(const std::string & dir)
{
  Site site;
  site.dir = dir;

  {
    const std::string file = "zones.geojson";
    YAML::Node root;
    try {
      root = YAML::LoadFile(dir + "/" + file);   // JSON 是 YAML 的子集
    } catch (const YAML::BadFile &) {
      failSite(file, "讀不到 " + dir + "/" + file);
    } catch (const YAML::Exception & e) {
      failSite(file, std::string("不是合法的 JSON：") + e.what());
    }
    if (!root["type"] || root["type"].as<std::string>() != "FeatureCollection" || !root["features"].IsSequence()) {
      failSite(file, "要是 FeatureCollection（features 清單）");
    }
    const Where top{file, ""};
    const int version = optionalField<int>(root, "version", 0, top);
    if (version < 0 || version > 0xFFFF) {
      fail(top, "version 要是 0～65535");
    }
    site.version = static_cast<std::uint16_t>(version);
    if (const YAML::Node policies = root["policies"]) {
      if (!policies.IsSequence()) {
        fail(top, "policies 要是清單");
      }
      for (std::size_t i = 0; i < policies.size(); ++i) {
        const Where w = itemOf(file, "policies", i);
        ZonePolicyEntry p;
        const int id = need<int>(policies[i], "id", w);
        if (id < 1 || id > 0xFFFF) {
          fail(w, "id 要是 1～65535（0 保留給「只套用 no_entry」）");
        }
        p.id = static_cast<std::uint16_t>(id);
        p.name = optionalField<std::string>(policies[i], "name", "", w);
        p.description = optionalField<std::string>(policies[i], "description", "", w);
        const YAML::Node forbid = policies[i]["forbid"];
        if (!forbid || !forbid.IsSequence()) {
          fail(w, "要有 forbid 清單");
        }
        for (std::size_t k = 0; k < forbid.size(); ++k) {
          const Where fw = w.sub("forbid[" + std::to_string(k) + "]");
          p.forbid.push_back(ZonePolicyEntry::Forbid{need<std::string>(forbid[k], "zone", fw),
              need<Strings>(forbid[k], "platforms", fw)});
        }
        site.policies.push_back(p);
      }
    }
    const YAML::Node features = root["features"];
    for (std::size_t i = 0; i < features.size(); ++i) {
      const Where w = itemOf(file, "features", i);
      const YAML::Node props = features[i]["properties"];
      const YAML::Node geom = features[i]["geometry"];
      if (!props || !geom) {
        fail(w, "要有 properties、geometry");
      }
      Zone z;
      z.id = need<std::string>(props, "id", w);
      z.name = optionalField<std::string>(props, "name", "", w);
      z.floor = optionalField<std::string>(props, "floor", "", w);
      z.restricted = optionalField<bool>(props, "restricted", false, w);
      z.high_voltage = optionalField<bool>(props, "high_voltage", false, w);
      z.no_entry = optionalField<Strings>(props, "no_entry", {}, w);
      z.cruise_altitude_m = optionalNumber(props, "cruise_altitude_m", w);
      z.ceiling_m = optionalNumber(props, "ceiling_m", w);
      const int floor_id = optionalField<int>(props, "floor_id", 0, w);
      if (floor_id < 0 || floor_id > 255) {
        fail(w, "floor_id 要是 0～255（0 = 不分）");
      }
      z.floor_id = static_cast<std::uint8_t>(floor_id);
      z.indoor = optionalField<bool>(props, "indoor", false, w);
      if (need<std::string>(geom, "type", w) != "Polygon") {
        fail(w, "geometry 要是 Polygon");
      }
      const auto rings = need<std::vector<std::vector<std::vector<double>>>>(geom, "coordinates", w);
      if (rings.empty() || rings[0].size() < 4) {
        fail(w, "多邊形至少 3 個點（頭尾同一點，共 4 個）");
      }
      for (const auto & p : rings[0]) {
        if (p.size() < 2) {
          fail(w, "座標要是 [x, y]");
        }
        z.polygon.push_back({p[0], p[1]});
      }
      site.zones.push_back(z);
    }
  }

  {
    const std::string file = "devices.csv";
    const Csv csv = readCsv(dir, file);
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      const CsvRow r{file, csv.rows[i], csv.lines[i]};
      Device d;
      d.id = r.need("id");
      d.cls = r.need("class");
      d.x = r.needNumber("x");
      d.y = r.needNumber("y");
      d.z = r.needNumber("z");
      d.floor = r.text("floor");
      d.zone = r.text("zone");
      d.safety_quadruped_m = r.number("safety_quadruped_m");
      d.safety_uav_m = r.number("safety_uav_m");
      const auto qx = r.number("obs_quadruped_x");
      const auto qy = r.number("obs_quadruped_y");
      const auto qyaw = r.number("obs_quadruped_yaw_deg");
      if (qx && qy) {
        d.obs_quadruped = std::array<double, 3>{*qx, *qy, qyaw.value_or(0.0)};
      }
      const auto ux = r.number("obs_uav_x");
      const auto uy = r.number("obs_uav_y");
      const auto uz = r.number("obs_uav_z");
      if (ux && uy && uz) {
        d.obs_uav = std::array<double, 3>{*ux, *uy, *uz};
      }
      d.name = r.text("name");
      site.devices.push_back(d);
    }
  }

  {
    const std::string file = "pois.csv";
    const Csv csv = readCsv(dir, file);
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      const CsvRow r{file, csv.rows[i], csv.lines[i]};
      Poi p;
      p.id = r.need("id");
      p.zone = r.text("zone");
      p.role = r.text("role");
      p.platform = r.need("platform");
      p.x = r.needNumber("x");
      p.y = r.needNumber("y");
      p.z = r.needNumber("z");
      p.yaw_deg = r.number("yaw_deg");
      p.floor = r.text("floor");
      site.pois.push_back(p);
    }
  }

  {
    const std::string file = "docks.csv";
    const Csv csv = readCsv(dir, file);
    for (std::size_t i = 0; i < csv.rows.size(); ++i) {
      const CsvRow r{file, csv.rows[i], csv.lines[i]};
      Dock d;
      d.id = r.need("id");
      d.kind = r.need("kind");
      d.platform = r.need("platform");
      d.x = r.needNumber("x");
      d.y = r.needNumber("y");
      d.z = r.needNumber("z");
      d.floor = r.text("floor");
      site.docks.push_back(d);
    }
  }
  return site;
}

std::vector<std::string> checkSite(const Site & site, const Registry & reg)
{
  std::vector<std::string> errors;
  const auto platformOk = [&reg](const std::string & p) {return reg.platform(p) != nullptr;};
  // (x, y) 落在哪個不准 platform 進去的區域；沒有回傳空字串
  const auto forbiddenZone = [&site](double x, double y, const std::string & platform) -> std::string {
      for (const Zone & z : site.zones) {
        if (contains(z.no_entry, platform) && z.contains(x, y)) {
          return z.id;
        }
      }
      return "";
    };

  std::set<std::string> ids;
  for (const Zone & z : site.zones) {
    if (!ids.insert(z.id).second) {
      errors.push_back("zones.geojson：區域 " + z.id + " 重複");
    }
    if (z.indoor && !z.no_entry.empty() && contains(z.no_entry, "uav")) {
      errors.push_back("zones.geojson：" + z.id + " 是室內又禁飛：indoor 沒有意義（要嘛拿掉 uav 的 no_entry）");
    }
    for (const std::string & p : z.no_entry) {
      if (!platformOk(p)) {
        errors.push_back("zones.geojson：" + z.id + " 的 no_entry 平台 " + p + " 不在 platforms.yaml");
      }
    }
    const auto & first = z.polygon.front();
    const auto & last = z.polygon.back();
    if (first[0] != last[0] || first[1] != last[1]) {
      errors.push_back("zones.geojson：" + z.id + " 的多邊形頭尾要同一點");
    }
  }

  // 政策（任務欄位補齊 05）：id 不重複、指到的區域與平台要存在
  std::set<std::uint16_t> policy_ids;
  for (const ZonePolicyEntry & p : site.policies) {
    const std::string at = "zones.geojson：政策 " + std::to_string(p.id);
    if (!policy_ids.insert(p.id).second) {
      errors.push_back(at + " 重複");
    }
    for (const ZonePolicyEntry::Forbid & f : p.forbid) {
      if (site.zone(f.zone) == nullptr) {
        errors.push_back(at + " 的區域 " + f.zone + " 不存在");
      }
      for (const std::string & pl : f.platforms) {
        if (!platformOk(pl)) {
          errors.push_back(at + " 的平台 " + pl + " 不在 platforms.yaml");
        }
      }
    }
  }

  for (const Device & d : site.devices) {
    const std::string at = "devices.csv：" + d.id;
    if (!ids.insert(d.id).second) {
      errors.push_back(at + " 重複（id 在 zones、devices、pois、docks 之間都不能重複）");
    }
    const auto colon = d.id.find(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 == d.id.size()) {
      errors.push_back(at + " 的 id 要是「類別:實例」（例如 machine:TA1）");
    } else if (d.id.substr(0, colon) != d.cls) {
      errors.push_back(at + " 的 id 前綴和 class " + d.cls + " 不一樣");
    }
    const ObjectClass * c = reg.objectClass(d.cls);
    if (c == nullptr) {
      errors.push_back(at + " 的 class " + d.cls + " 不在 objects.yaml");
    } else if (c->safety_per_instance && !d.safety_quadruped_m && !d.safety_uav_m) {
      errors.push_back(at + "：" + d.cls + " 的安全距離是 per_instance，要填 safety_quadruped_m 或 safety_uav_m");
    }
    if (!d.zone.empty() && site.zone(d.zone) == nullptr) {
      errors.push_back(at + " 的 zone " + d.zone + " 不在 zones.geojson");
    }
    if (d.obs_quadruped) {
      const std::string z = forbiddenZone((*d.obs_quadruped)[0], (*d.obs_quadruped)[1], "quadruped");
      if (!z.empty()) {
        errors.push_back(at + " 的狗觀察姿態在狗的禁入區 " + z);
      }
    }
    if (d.obs_uav) {
      const std::string z = forbiddenZone((*d.obs_uav)[0], (*d.obs_uav)[1], "uav");
      if (!z.empty()) {
        errors.push_back(at + " 的無人機觀察點在禁飛區 " + z);
      }
    }
  }

  for (const Poi & p : site.pois) {
    const std::string at = "pois.csv：" + p.id;
    if (!ids.insert(p.id).second) {
      errors.push_back(at + " 重複（id 在 zones、devices、pois、docks 之間都不能重複）");
    }
    if (!platformOk(p.platform)) {
      errors.push_back(at + " 的平台 " + p.platform + " 不在 platforms.yaml");
    }
    if (!p.zone.empty() && site.zone(p.zone) == nullptr) {
      errors.push_back(at + " 的 zone " + p.zone + " 不在 zones.geojson");
    }
    const std::string z = forbiddenZone(p.x, p.y, p.platform);
    if (!z.empty()) {
      errors.push_back(at + " 在 " + p.platform + " 的禁入區 " + z);
    }
    for (const Zone & zone : site.zones) {
      if (zone.ceiling_m && p.platform == "uav" && zone.contains(p.x, p.y) && p.z > *zone.ceiling_m) {
        errors.push_back(at + " 的高度 " + fmt(p.z) + " m 超過 " + zone.id + " 的天花板 " + fmt(*zone.ceiling_m) + " m");
      }
    }
  }

  for (const Dock & d : site.docks) {
    const std::string at = "docks.csv：" + d.id;
    if (!ids.insert(d.id).second) {
      errors.push_back(at + " 重複（id 在 zones、devices、pois、docks 之間都不能重複）");
    }
    if (!oneOf(d.kind, {"dock", "pad"})) {
      errors.push_back(at + " 的 kind " + d.kind + " 要是 dock／pad");
    }
    if (!platformOk(d.platform)) {
      errors.push_back(at + " 的平台 " + d.platform + " 不在 platforms.yaml");
    }
    const std::string z = forbiddenZone(d.x, d.y, d.platform);
    if (!z.empty()) {
      errors.push_back(at + " 在 " + d.platform + " 的禁入區 " + z);
    }
  }

  // fleet.yaml 的 limits 指到這裡的起降點、充電樁
  for (const RobotEntry & r : reg.robots) {
    const auto checkRefs = [&](const Strings & refs, const char * kind) {
        for (const std::string & id : refs) {
          const Dock * d = site.dock(id);
          if (d == nullptr) {
            errors.push_back("fleet.yaml：" + r.name + " 的 " + kind + " " + id + " 不在 docks.csv");
          } else if (d->platform != r.platform || d->kind != (std::string(kind) == "landing_pads" ? "pad" : "dock")) {
            errors.push_back("fleet.yaml：" + r.name + " 的 " + kind + " " + id + " 是 " + d->platform + " 的 " + d->kind);
          }
        }
      };
    checkRefs(r.limits.landing_pads, "landing_pads");
    checkRefs(r.limits.docks, "docks");
  }
  return errors;
}

cbba_core::ZoneRegistry toZoneRegistry(const Site & site, const Registry * registry)
{
  const auto code = [registry](const std::string & name) -> std::uint8_t {
      if (registry != nullptr) {
        if (const PlatformEntry * p = registry->platform(name)) {
          return p->code;
        }
      } else if (name == "uav") {
        return 1;
      } else if (name == "quadruped") {
        return 2;
      }
      throw std::runtime_error("zones.geojson：不認得的平台 " + name);
    };
  const auto mask = [&code](const std::vector<std::string> & names) {
      std::uint8_t m = 0;
      for (const std::string & n : names) {
        const std::uint8_t c = code(n);
        if (c >= 8) {
          throw std::runtime_error("zones.geojson：平台 " + n + " 的代碼 " + std::to_string(c) + " 超過 7（遮罩只有 8 位元）");
        }
        m = static_cast<std::uint8_t>(m | (1u << c));
      }
      return m;
    };
  cbba_core::ZoneRegistry out;
  out.version = site.version;
  for (const Zone & z : site.zones) {
    cbba_core::ZoneArea a;
    a.id = z.id;
    a.floor = z.floor_id;
    a.no_entry = mask(z.no_entry);
    a.ceiling = z.ceiling_m.value_or(0.0);
    a.indoor = z.indoor;
    a.polygon = z.polygon;
    out.zones.push_back(a);
  }
  for (const ZonePolicyEntry & p : site.policies) {
    cbba_core::ZonePolicy policy;
    policy.id = p.id;
    policy.name = p.name;
    for (const ZonePolicyEntry::Forbid & f : p.forbid) {
      std::size_t index = site.zones.size();
      for (std::size_t i = 0; i < site.zones.size(); ++i) {
        if (site.zones[i].id == f.zone) {
          index = i;
        }
      }
      if (index == site.zones.size()) {
        throw std::runtime_error("zones.geojson：政策 " + std::to_string(p.id) + " 的區域 " + f.zone + " 不存在");
      }
      policy.forbid.push_back(cbba_core::ZonePolicy::Rule{index, mask(f.platforms)});
    }
    out.policies.push_back(policy);
  }
  return out;
}

std::set<std::string> supportedActions(const Registry & registry, const std::string & platform,
  const std::vector<std::string> & capabilities)
{
  const std::set<std::string> have(capabilities.begin(), capabilities.end());
  std::set<std::string> out;
  for (const ActionEntry & a : registry.actions) {
    if (a.status != "open" || a.cls == "internal" || a.implementation(platform) == nullptr) {
      continue;
    }
    bool ok = a.requires_any_of.empty();
    for (const auto & group : a.requires_any_of) {
      if (std::all_of(group.begin(), group.end(), [&have](const std::string & cap) {return have.count(cap) > 0;})) {
        ok = true;
        break;
      }
    }
    if (ok) {
      out.insert(a.name);
    }
  }
  return out;
}

}  // namespace swarm_cbba
