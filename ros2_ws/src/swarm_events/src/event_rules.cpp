#include "swarm_events/event_rules.hpp"

#include <algorithm>
#include <set>
#include <stdexcept>

namespace swarm_events
{

namespace
{

[[noreturn]] void fail(const std::string & path, const std::string & where, const std::string & why)
{
  throw std::runtime_error(path + "：" + where + "：" + why);
}

std::vector<std::string> stringList(const YAML::Node & node, const std::string & path, const std::string & where)
{
  std::vector<std::string> out;
  if (!node) {
    return out;
  }
  if (!node.IsSequence()) {
    fail(path, where, "要是清單");
  }
  for (const auto & item : node) {
    out.push_back(item.as<std::string>());
  }
  return out;
}

LocationRule parseLocation(const YAML::Node & node, const std::string & path, const std::string & where)
{
  LocationRule loc;
  if (!node) {
    return loc;
  }
  if (node.IsScalar()) {
    loc.kind = node.as<std::string>();   // incident
    return loc;
  }
  if (!node.IsMap() || node.size() != 1) {
    fail(path, where + ".location", "要是 incident 或 {poi|asset|zone|dock: 值}");
  }
  const auto it = node.begin();
  loc.kind = it->first.as<std::string>();
  loc.value = it->second.as<std::string>();
  return loc;
}

RoleRule parseRole(const YAML::Node & node, const std::string & path, const std::string & where)
{
  if (!node.IsMap()) {
    fail(path, where, "角色要是物件");
  }
  RoleRule r;
  if (!node["role"]) {
    fail(path, where, "缺 role");
  }
  r.role = node["role"].as<std::string>();
  const std::string w = where + "(" + r.role + ")";
  if (node["platform"]) {
    r.platform = node["platform"].as<std::string>();
  }
  r.required_capabilities = stringList(node["required_capabilities"], path, w + ".required_capabilities");
  r.optional = node["optional"] ? node["optional"].as<bool>() : false;
  r.skip_satisfies = node["skip_satisfies"] ? node["skip_satisfies"].as<bool>() : false;
  r.location = parseLocation(node["location"], path, w);
  if (const auto off = node["offset"]) {
    if (!off.IsSequence() || off.size() != 2) {
      fail(path, w + ".offset", "要是 [dx, dy]");
    }
    r.offset = {{off[0].as<double>(), off[1].as<double>()}};
  }
  if (node["value"]) {
    r.value = node["value"].as<double>();
  }
  if (node["service_time_s"]) {
    r.service_time_s = node["service_time_s"].as<double>();
  }
  r.retarget = node["retarget"] ? node["retarget"].as<bool>() : true;
  if (!node["steps"] || !node["steps"].IsSequence()) {
    fail(path, w + ".steps", "要是清單");
  }
  r.steps = node["steps"];
  return r;
}

int priorityOf(const std::string & text)
{
  if (text.size() == 2 && text[0] == 'P' && text[1] >= '0' && text[1] <= '3') {
    return text[1] - '0';
  }
  return -1;
}

}  // namespace

int Rule::roleIndex(const std::string & role) const
{
  for (std::size_t i = 0; i < roles.size(); ++i) {
    if (roles[i].role == role) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

const Rule * EventRules::rule(const std::string & name) const
{
  const auto it = rules.find(name);
  return it == rules.end() ? nullptr : &it->second;
}

EventRules loadEventRules(const std::string & path)
{
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(path + "：讀不到或不是 YAML（" + std::string(e.what()) + "）");
  }
  EventRules out;
  out.path = path;
  try {
    if (!doc["schema_version"] || doc["schema_version"].as<std::string>() != kRulesSchema) {
      fail(path, "schema_version", std::string("要是 ") + kRulesSchema);
    }
    const auto rules = doc["rules"];
    if (!rules || !rules.IsMap()) {
      fail(path, "rules", "要是物件（規則名稱 → 內容）");
    }
    for (const auto & kv : rules) {
      Rule rule;
      rule.name = kv.first.as<std::string>();
      const YAML::Node & n = kv.second;
      const std::string where = "rules." + rule.name;
      if (!n.IsMap()) {
        fail(path, where, "要是物件");
      }
      if (n["requires_approval"]) {
        rule.requires_approval = n["requires_approval"].as<bool>();
      }
      if (n["priority"]) {
        const int p = priorityOf(n["priority"].as<std::string>());
        if (p < 0) {
          fail(path, where + ".priority", "要是 P0～P3");
        }
        rule.priority = p;
      }
      if (n["fallback_template"]) {
        rule.fallback_template = n["fallback_template"].as<std::string>();
      }
      // 不存在的欄位是無效節點，不能拿來指定（yaml-cpp 會丟 InvalidNode），先確認有
      if (n["report_policy"]) {
        rule.report_policy = n["report_policy"];
      }
      if (const auto c = n["coupling"]) {
        rule.all_or_nothing = c["all_or_nothing"] ? c["all_or_nothing"].as<bool>() : false;
        rule.distinct = stringList(c["distinct"], path, where + ".coupling.distinct");
        if (c["sync_window_s"]) {
          rule.sync_window_s = c["sync_window_s"].as<std::uint16_t>();
        }
        if (const auto pre = c["precedence"]) {
          if (!pre.IsSequence()) {
            fail(path, where + ".coupling.precedence", "要是 [[A, B], …]");
          }
          for (const auto & pair : pre) {
            if (!pair.IsSequence() || pair.size() != 2) {
              fail(path, where + ".coupling.precedence", "每一組是 [A, B]");
            }
            rule.precedence.emplace_back(pair[0].as<std::string>(), pair[1].as<std::string>());
          }
        }
      }
      const auto roles = n["roles"];
      if (!roles || !roles.IsSequence() || roles.size() == 0) {
        fail(path, where + ".roles", "要是 1 個以上角色的清單");
      }
      if (roles.size() > 8) {
        fail(path, where + ".roles", "最多 8 個角色（CBBA 的 role_count 上限）");
      }
      std::set<std::string> seen;
      for (std::size_t i = 0; i < roles.size(); ++i) {
        RoleRule r = parseRole(roles[i], path, where + ".roles[" + std::to_string(i) + "]");
        if (!seen.insert(r.role).second) {
          fail(path, where + ".roles", "角色 " + r.role + " 重複");
        }
        rule.roles.push_back(std::move(r));
      }
      out.rules.emplace(rule.name, std::move(rule));
    }
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(path + "：欄位型別不對（" + std::string(e.what()) + "）");
  }
  return out;
}

std::vector<std::string> checkEventRules(const EventRules & rules, const swarm_cbba::Registry & registry,
  const swarm_cbba::Site * site)
{
  std::vector<std::string> errors;
  std::set<std::string> tools;
  for (const auto & t : registry.tools) {
    tools.insert(t.name);
  }
  for (const auto & [name, rule] : rules.rules) {
    const std::string w = "規則 " + name;
    const auto has_role = [&rule](const std::string & role) {return rule.roleIndex(role) >= 0;};
    if (!rule.fallback_template.empty() && !rules.rule(rule.fallback_template)) {
      errors.push_back(w + "：fallback_template " + rule.fallback_template + " 不是有的規則");
    }
    for (const auto & d : rule.distinct) {
      if (!has_role(d)) {
        errors.push_back(w + "：distinct 的 " + d + " 不是這個規則的角色");
      }
    }
    for (const auto & [a, b] : rule.precedence) {
      if (!has_role(a) || !has_role(b) || a == b) {
        errors.push_back(w + "：precedence [" + a + ", " + b + "] 要是這個規則裡兩個不同的角色");
      }
    }
    for (const auto & r : rule.roles) {
      const std::string rw = w + " 的角色 " + r.role;
      if (!registry.role(r.role)) {
        errors.push_back(rw + "：不在 task_types.yaml 的 roles");
      }
      if (r.platform != "any" && !registry.platform(r.platform)) {
        errors.push_back(rw + "：平台 " + r.platform + " 不在 platforms.yaml（或寫 any）");
      }
      for (const auto & c : r.required_capabilities) {
        if (!tools.count(c)) {
          errors.push_back(rw + "：能力 " + c + " 不在 tools.yaml");
        }
      }
      static const std::set<std::string> kinds{"incident", "poi", "asset", "zone", "dock"};
      if (!kinds.count(r.location.kind)) {
        errors.push_back(rw + "：location 的種類 " + r.location.kind + " 不認得（incident、poi、asset、zone、dock）");
      } else if (site != nullptr) {
        if (r.location.kind == "poi" &&
          std::none_of(site->pois.begin(), site->pois.end(), [&r](const swarm_cbba::Poi & p) {
            return p.role == r.location.value;}))
        {
          errors.push_back(rw + "：場域沒有 role 是 " + r.location.value + " 的觀察點");
        }
        if (r.location.kind == "zone" && !site->zone(r.location.value)) {
          errors.push_back(rw + "：區域 " + r.location.value + " 不存在");
        }
        if (r.location.kind == "dock" && !site->dock(r.location.value)) {
          errors.push_back(rw + "：起降點 " + r.location.value + " 不存在");
        }
      }
      if (r.steps.size() == 0) {
        errors.push_back(rw + "：steps 是空的");
      }
      for (std::size_t i = 0; i < r.steps.size(); ++i) {
        const YAML::Node step = r.steps[i];
        const std::string sw = rw + " 的 steps[" + std::to_string(i) + "]";
        if (!step.IsMap() || !step["action"]) {
          errors.push_back(sw + "：要是有 action 的物件");
          continue;
        }
        const std::string action = step["action"].as<std::string>();
        if (!registry.action(action)) {
          errors.push_back(sw + "：動作 " + action + " 不在 actions.yaml");
        }
        if (const auto wf = step["wait_for"]) {
          for (const auto & item : wf) {
            const std::string target = item["role"] ? item["role"].as<std::string>() : "";
            if (!has_role(target) || target == r.role) {
              errors.push_back(sw + "：wait_for 的角色 " + target + " 要是這個規則裡別的角色");
            }
            if (!item["milestone"]) {
              errors.push_back(sw + "：wait_for 要有 milestone");
            }
          }
        }
      }
    }
  }
  for (const auto & e : registry.event_types) {
    if (e.status == "open" && e.category == "template" && !e.default_rule.empty() && !rules.rule(e.default_rule)) {
      errors.push_back("事件類型 " + e.name + " 的 default_rule " + e.default_rule + " 沒有規則");
    }
  }
  return errors;
}

}  // namespace swarm_events
