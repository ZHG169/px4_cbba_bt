#include "swarm_cbba/registry.hpp"

#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "cbba_core/capabilities.hpp"

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
