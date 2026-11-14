#include "swarm_cbba/registry.hpp"

#include <initializer_list>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/failure_codes.hpp"

namespace swarm_cbba
{

namespace
{
// 錯誤訊息的位置：「fleet.yaml robots[3]」
struct Where
{
  std::string file;
  std::string item;
  std::string text() const {return item.empty() ? file : file + " " + item;}
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
  if (!v) {
    return fallback;
  }
  try {
    return v.as<T>();
  } catch (const YAML::Exception &) {
    fail(w, std::string("欄位 ") + key + " 的型別不對");
  }
}

// 讀一份檔案、檢查 schema_version、回傳 list_key 的清單（list_key 可以不在：空清單）
YAML::Node loadFile(const std::string & dir, const std::string & file, const char * list_key)
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
  const YAML::Node list = root[list_key];
  if (list && !list.IsSequence()) {
    fail(w, std::string(list_key) + " 要是清單");
  }
  return list ? list : YAML::Node(YAML::NodeType::Sequence);
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
}  // namespace

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

Registry loadRegistry(const std::string & dir)
{
  Registry reg;
  reg.dir = dir;

  const YAML::Node tools = loadFile(dir, "tools.yaml", "capabilities");
  for (std::size_t i = 0; i < tools.size(); ++i) {
    const Where w = itemOf("tools.yaml", "capabilities", i);
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

  const YAML::Node platforms = loadFile(dir, "platforms.yaml", "platforms");
  for (std::size_t i = 0; i < platforms.size(); ++i) {
    const Where w = itemOf("platforms.yaml", "platforms", i);
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

  const YAML::Node robots = loadFile(dir, "fleet.yaml", "robots");
  for (std::size_t i = 0; i < robots.size(); ++i) {
    const Where w = itemOf("fleet.yaml", "robots", i);
    RobotEntry r;
    r.id = agentId(robots[i], w);
    r.name = need<std::string>(robots[i], "name", w);
    r.platform = need<std::string>(robots[i], "platform", w);
    r.ns = optionalField<std::string>(robots[i], "namespace", "", w);
    r.capabilities = optionalField<std::vector<std::string>>(robots[i], "capabilities", {}, w);
    reg.robots.push_back(r);
  }
  const YAML::Node gateways = loadFile(dir, "fleet.yaml", "gateways");
  for (std::size_t i = 0; i < gateways.size(); ++i) {
    const Where w = itemOf("fleet.yaml", "gateways", i);
    GatewayEntry g;
    g.id = agentId(gateways[i], w);
    g.name = need<std::string>(gateways[i], "name", w);
    g.ns = optionalField<std::string>(gateways[i], "namespace", "", w);
    reg.gateways.push_back(g);
  }

  // actions.yaml：common_failure_codes 在最上層，和 actions 清單同一份檔案
  const YAML::Node actions = loadFile(dir, "actions.yaml", "actions");
  {
    const YAML::Node root = YAML::LoadFile(dir + "/actions.yaml");
    reg.common_failure_codes = optionalField<std::vector<std::string>>(
      root, "common_failure_codes", {}, Where{"actions.yaml", ""});
  }
  for (std::size_t i = 0; i < actions.size(); ++i) {
    const Where w = itemOf("actions.yaml", "actions", i);
    const YAML::Node & n = actions[i];
    ActionEntry a;
    a.name = need<std::string>(n, "name", w);
    a.description = optionalField<std::string>(n, "description", "", w);
    a.cls = need<std::string>(n, "class", w);
    a.platforms = need<std::vector<std::string>>(n, "platforms", w);
    a.required_capabilities = optionalField<std::vector<std::string>>(n, "required_capabilities", {}, w);
    a.location = need<std::string>(n, "location", w);
    a.object = need<std::string>(n, "object", w);
    a.preconditions = optionalField<std::vector<std::string>>(n, "preconditions", {}, w);
    a.failure_codes = optionalField<std::vector<std::string>>(n, "failure_codes", {}, w);
    a.cancellable = need<bool>(n, "cancellable", w);
    a.status = need<std::string>(n, "status", w);
    const YAML::Node params = n["params"];
    if (params && !params.IsSequence()) {
      fail(w, "params 要是清單");
    }
    for (std::size_t k = 0; params && k < params.size(); ++k) {
      const Where pw{"actions.yaml", w.item + ".params[" + std::to_string(k) + "]"};
      ActionParam p;
      p.name = need<std::string>(params[k], "name", pw);
      p.type = need<std::string>(params[k], "type", pw);
      p.unit = optionalField<std::string>(params[k], "unit", "", pw);
      p.required = optionalField<bool>(params[k], "required", false, pw);
      p.description = optionalField<std::string>(params[k], "description", "", pw);
      if (params[k]["default"]) {
        p.has_default = true;
        p.default_value = optionalField<std::string>(params[k], "default", "", pw);
      }
      a.params.push_back(p);
    }
    reg.actions.push_back(a);
  }

  // objects.yaml：classes、instances 兩份清單在同一份檔案
  const YAML::Node classes = loadFile(dir, "objects.yaml", "classes");
  for (std::size_t i = 0; i < classes.size(); ++i) {
    const Where w = itemOf("objects.yaml", "classes", i);
    ObjectClass c;
    c.name = need<std::string>(classes[i], "name", w);
    c.category = need<std::string>(classes[i], "category", w);
    c.trackable = optionalField<bool>(classes[i], "trackable", false, w);
    c.inspectable = optionalField<bool>(classes[i], "inspectable", false, w);
    c.description = optionalField<std::string>(classes[i], "description", "", w);
    reg.object_classes.push_back(c);
  }
  const YAML::Node instances = loadFile(dir, "objects.yaml", "instances");
  for (std::size_t i = 0; i < instances.size(); ++i) {
    const Where w = itemOf("objects.yaml", "instances", i);
    ObjectInstance o;
    o.id = need<std::string>(instances[i], "id", w);
    o.cls = need<std::string>(instances[i], "class", w);
    o.site = optionalField<std::string>(instances[i], "site", "", w);
    o.position = need<std::vector<double>>(instances[i], "position", w);
    o.floor = optionalField<int>(instances[i], "floor", 0, w);
    reg.object_instances.push_back(o);
  }

  // event_types.yaml
  const YAML::Node events = loadFile(dir, "event_types.yaml", "event_types");
  for (std::size_t i = 0; i < events.size(); ++i) {
    const Where w = itemOf("event_types.yaml", "event_types", i);
    const YAML::Node & n = events[i];
    EventType e;
    e.name = need<std::string>(n, "name", w);
    e.description = optionalField<std::string>(n, "description", "", w);
    e.priority = need<int>(n, "priority", w);
    e.object_class = need<std::string>(n, "object_class", w);
    const YAML::Node detectors = n["detectors"];
    if (!detectors || !detectors.IsSequence()) {
      fail(w, "detectors 要是清單");
    }
    for (std::size_t k = 0; k < detectors.size(); ++k) {
      const Where dw{"event_types.yaml", w.item + ".detectors[" + std::to_string(k) + "]"};
      EventDetector d;
      d.detector = need<std::string>(detectors[k], "detector", dw);
      d.labels = need<std::vector<std::string>>(detectors[k], "labels", dw);
      d.min_score = need<double>(detectors[k], "min_score", dw);
      d.status = need<std::string>(detectors[k], "status", dw);
      e.detectors.push_back(d);
    }
    const YAML::Node trigger = n["trigger"];
    if (!trigger || !trigger.IsMap()) {
      fail(w, "缺 trigger");
    }
    e.min_count = need<int>(trigger, "min_count", w);
    e.window = need<double>(trigger, "window", w);
    e.merge_radius = optionalField<double>(trigger, "merge_radius", 0.0, w);
    const YAML::Node vlm = n["vlm"];
    if (vlm) {
      e.vlm_enabled = need<bool>(vlm, "enabled", w);
      e.vlm_prompt = optionalField<std::string>(vlm, "prompt", "", w);
    }
    e.dispatch_cost = optionalField<double>(n, "dispatch_cost", 1.0, w);
    e.tmpl = need<std::string>(n, "template", w);
    e.expires = optionalField<double>(n, "expires", 0.0, w);
    e.status = need<std::string>(n, "status", w);
    reg.event_types.push_back(e);
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
  }
  for (const GatewayEntry & g : reg.gateways) {
    if (!ids.insert(g.id).second) {
      errors.push_back("fleet.yaml：閘道 " + g.name + " 的機號 " + std::to_string(g.id) + " 和別台重複");
    }
  }

  // actions
  const auto oneOf = [](const std::string & v, std::initializer_list<const char *> allowed) {
      for (const char * a : allowed) {
        if (v == a) {
          return true;
        }
      }
      return false;
    };
  for (const std::string & code : reg.common_failure_codes) {
    if (!cbba_core::failureCodeInfo(code)) {
      errors.push_back("actions.yaml：common_failure_codes 的 " + code + " 不是 8 個失敗碼之一");
    }
  }
  std::set<std::string> action_names;
  for (const ActionEntry & a : reg.actions) {
    const std::string at = "actions.yaml：" + a.name;
    if (!action_names.insert(a.name).second) {
      errors.push_back(at + " 重複");
    }
    if (!oneOf(a.cls, {"plannable", "system", "internal"})) {
      errors.push_back(at + " 的 class " + a.cls + " 要是 plannable／system／internal");
    }
    if (!oneOf(a.location, {"required", "optional", "none"})) {
      errors.push_back(at + " 的 location " + a.location + " 要是 required／optional／none");
    }
    if (!oneOf(a.object, {"required", "optional", "none"})) {
      errors.push_back(at + " 的 object " + a.object + " 要是 required／optional／none");
    }
    if (!oneOf(a.status, {"open", "closed"})) {
      errors.push_back(at + " 的 status " + a.status + " 要是 open／closed");
    }
    if (a.platforms.empty()) {
      errors.push_back(at + " 沒有平台");
    }
    for (const std::string & p : a.platforms) {
      if (platform_names.count(p) == 0) {
        errors.push_back(at + " 的平台 " + p + " 不在 platforms.yaml");
      }
    }
    for (const std::string & c : a.required_capabilities) {
      const auto t = tool_by_name.find(c);
      if (t == tool_by_name.end() || t->second->deprecated) {
        errors.push_back(at + " 的能力 " + c + " 不在 tools.yaml（或已經 deprecated）");
      }
    }
    for (const std::string & code : a.failure_codes) {
      if (!cbba_core::failureCodeInfo(code)) {
        errors.push_back(at + " 的失敗碼 " + code + " 不是 8 個失敗碼之一");
      }
    }
    for (const std::string & pre : a.preconditions) {
      if (!oneOf(pre, {"at_location"})) {
        errors.push_back(at + " 的 precondition " + pre + " 不認得（目前只有 at_location）");
      }
    }
    if (!a.cancellable) {
      errors.push_back(at + " 不可取消（2-1b：每個動作都必須可取消）");
    }
    std::set<std::string> param_names;
    for (const ActionParam & p : a.params) {
      if (!param_names.insert(p.name).second) {
        errors.push_back(at + " 的參數 " + p.name + " 重複");
      }
      if (!oneOf(p.type, {"float", "int", "string", "bool", "point", "point_list"})) {
        errors.push_back(at + " 的參數 " + p.name + " 的 type " + p.type + " 不認得");
      }
      if (p.required && p.has_default) {
        errors.push_back(at + " 的參數 " + p.name + " 是 required，不能有 default");
      }
    }
  }

  // objects
  std::set<std::string> class_names;
  for (const ObjectClass & c : reg.object_classes) {
    if (!class_names.insert(c.name).second) {
      errors.push_back("objects.yaml：種類 " + c.name + " 重複");
    }
    if (!oneOf(c.category, {"agent", "equipment", "structure", "hazard", "item"})) {
      errors.push_back("objects.yaml：" + c.name + " 的 category " + c.category +
        " 要是 agent／equipment／structure／hazard／item");
    }
  }
  std::set<std::string> instance_ids;
  for (const ObjectInstance & o : reg.object_instances) {
    if (!instance_ids.insert(o.id).second) {
      errors.push_back("objects.yaml：設備 " + o.id + " 重複");
    }
    if (class_names.count(o.cls) == 0) {
      errors.push_back("objects.yaml：" + o.id + " 的種類 " + o.cls + " 不在 classes");
    }
    if (o.position.size() != 3) {
      errors.push_back("objects.yaml：" + o.id + " 的 position 要是 [x, y, z]");
    }
  }

  // event_types
  constexpr std::size_t kMaxName = 64;   // DETECTION 封包的 detector、label 上限（wire_detection.hpp）
  std::set<std::string> event_names;
  for (const EventType & e : reg.event_types) {
    const std::string at = "event_types.yaml：" + e.name;
    if (!event_names.insert(e.name).second) {
      errors.push_back(at + " 重複");
    }
    if (e.priority < 0 || e.priority > 3) {
      errors.push_back(at + " 的 priority 要是 0～3");
    }
    if (class_names.count(e.object_class) == 0) {
      errors.push_back(at + " 的 object_class " + e.object_class + " 不在 objects.yaml");
    }
    if (e.detectors.empty()) {
      errors.push_back(at + " 沒有偵測器");
    }
    for (const EventDetector & d : e.detectors) {
      if (d.detector.empty() || d.detector.size() > kMaxName) {
        errors.push_back(at + " 的偵測器名稱要是 1～64 B");
      }
      if (d.labels.empty()) {
        errors.push_back(at + " 的偵測器 " + d.detector + " 沒有 labels");
      }
      for (const std::string & l : d.labels) {
        if (l.empty() || l.size() > kMaxName) {
          errors.push_back(at + " 的 label 要是 1～64 B");
        }
      }
      if (d.min_score < 0.0 || d.min_score > 1.0) {
        errors.push_back(at + " 的偵測器 " + d.detector + " 的 min_score 要在 0～1");
      }
      if (!oneOf(d.status, {"available", "planned"})) {
        errors.push_back(at + " 的偵測器 " + d.detector + " 的 status 要是 available／planned");
      }
    }
    if (e.min_count < 1 || !(e.window > 0.0) || e.merge_radius < 0.0) {
      errors.push_back(at + " 的 trigger 要 min_count ≥ 1、window > 0、merge_radius ≥ 0");
    }
    if (e.vlm_enabled && e.vlm_prompt.empty()) {
      errors.push_back(at + " 的 VLM 開著但沒有 prompt");
    }
    if (e.dispatch_cost < 0.0) {
      errors.push_back(at + " 的 dispatch_cost 不能是負的");
    }
    if (e.tmpl.empty()) {
      errors.push_back(at + " 沒有 template");
    }
    if (e.expires < 0.0) {
      errors.push_back(at + " 的 expires 不能是負的");
    }
    if (!oneOf(e.status, {"open", "closed"})) {
      errors.push_back(at + " 的 status 要是 open／closed");
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

}  // namespace swarm_cbba
