#include "swarm_events/expander.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <sstream>

#include "cbba_core/task_meta.hpp"
#include "swarm_cbba/spec_validator.hpp"

namespace swarm_events
{

namespace
{

constexpr double kAnchorStep = 0.5;
constexpr double kAnchorReach = 30.0;
// 找到的點再往外推這麼多：不停在禁區的邊線上（射線法在邊上的結果不一定，閘道可能判在區內；itest G6 2026-11-23 發現）
constexpr double kAnchorMargin = 0.5;

std::string fmt2(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.2f", v);
  return buf;
}

double polygonArea(const swarm_cbba::Zone & z)
{
  double a = 0.0;
  for (std::size_t i = 0; i + 1 < z.polygon.size(); ++i) {
    a += z.polygon[i][0] * z.polygon[i + 1][1] - z.polygon[i + 1][0] * z.polygon[i][1];
  }
  return std::fabs(a) / 2.0;
}

// ---------- {incident.*} 代換與 YAML → JSON ----------
struct Subst
{
  std::map<std::string, std::string> values;

  std::string apply(std::string text) const
  {
    for (const auto & [key, value] : values) {
      for (std::size_t pos = text.find(key); pos != std::string::npos; pos = text.find(key, pos + value.size())) {
        text.replace(pos, key.size(), value);
      }
    }
    if (!text.empty() && text.back() == ':') {
      text.pop_back();   // "person:{incident.track_id}" 沒有 track_id → "person"
    }
    return text;
  }
};

// 純量的型別：YAML 有引號的一律是字串；否則 true／false 是布林、整個是數字的是數字，其他是字串
std::string scalarJson(const YAML::Node & node, const Subst & subst)
{
  const std::string text = node.Scalar();
  if (node.Tag() != "!") {
    if (text == "true" || text == "false") {
      return text;
    }
    if (!text.empty()) {
      char * end = nullptr;
      const double v = std::strtod(text.c_str(), &end);
      if (end != nullptr && *end == '\0' && std::isfinite(v)) {
        if (text.find_first_of(".eE") == std::string::npos) {
          return std::to_string(std::llround(v));
        }
        std::ostringstream os;
        os.precision(12);
        os << v;
        return os.str();
      }
    }
  }
  return cbba_core::jsonString(subst.apply(text));
}

std::string toJson(const YAML::Node & node, const Subst & subst)
{
  if (node.IsMap()) {
    std::string out = "{";
    bool first = true;
    for (const auto & kv : node) {
      out += (first ? "" : ",") + cbba_core::jsonString(kv.first.as<std::string>()) + ":" + toJson(kv.second, subst);
      first = false;
    }
    return out + "}";
  }
  if (node.IsSequence()) {
    std::string out = "[";
    for (std::size_t i = 0; i < node.size(); ++i) {
      out += (i ? "," : "") + toJson(node[i], subst);
    }
    return out + "]";
  }
  if (node.IsScalar()) {
    return scalarJson(node, subst);
  }
  return "null";
}

// 一個角色的 steps → JSON 陣列。wait_for 的角色名稱換成 role_index；fill = true 時沒寫 location 的 go_to 填觀察點
std::string stepsJson(const RoleRule & r, const Rule & rule, const Subst & subst, bool fill, double x, double y,
  double z, std::vector<std::string> * actions, std::set<int> * waits)
{
  std::string out = "[";
  for (std::size_t i = 0; i < r.steps.size(); ++i) {
    const YAML::Node step = r.steps[i];
    std::string s = "{";
    bool first = true;
    bool has_location = false;
    std::string action;
    for (const auto & kv : step) {
      const std::string key = kv.first.as<std::string>();
      std::string value;
      if (key == "wait_for") {
        value = "[";
        for (std::size_t k = 0; k < kv.second.size(); ++k) {
          const YAML::Node w = kv.second[k];
          const int idx = rule.roleIndex(w["role"] ? w["role"].as<std::string>() : "");
          if (waits != nullptr && idx >= 0) {
            waits->insert(idx);
          }
          value += (k ? "," : "") + std::string("{\"role_index\":") + std::to_string(std::max(idx, 0)) +
            ",\"milestone\":" + cbba_core::jsonString(w["milestone"] ? w["milestone"].as<std::string>() : "") + "}";
        }
        value += "]";
      } else {
        value = toJson(kv.second, subst);
      }
      if (key == "action") {
        action = kv.second.as<std::string>();
      }
      if (key == "location") {
        has_location = true;
      }
      s += (first ? "" : ",") + cbba_core::jsonString(key) + ":" + value;
      first = false;
    }
    if (fill && action == "go_to" && !has_location) {
      s += ",\"location\":{\"x\":" + fmt2(x) + ",\"y\":" + fmt2(y) + ",\"z\":" + fmt2(z) + "}";
    }
    s += "}";
    if (actions != nullptr && !action.empty()) {
      actions->push_back(action);
    }
    out += (i ? "," : "") + s;
  }
  return out + "]";
}

std::string roleSpec(const Rule & rule, const RoleRule & r, const std::string & steps)
{
  std::string spec = "{\"template\":" + cbba_core::jsonString(rule.name) + ",\"role\":" + cbba_core::jsonString(r.role) +
    ",\"steps\":" + steps;
  if (rule.report_policy && !rule.report_policy.IsNull()) {
    spec += ",\"report_policy\":" + toJson(rule.report_policy, Subst{});
  }
  return spec + "}";
}

Plan failed(Plan plan, const std::string & code, const std::string & reason)
{
  plan.ok = false;
  plan.code = code;
  plan.reason = reason;
  return plan;
}

std::array<double, 2> homeOf(const swarm_cbba::RobotEntry & robot, const swarm_cbba::Site & site, bool * known)
{
  for (const auto * list : {&robot.limits.landing_pads, &robot.limits.docks}) {
    for (const auto & id : *list) {
      if (const auto * d = site.dock(id)) {
        *known = true;
        return {{d->x, d->y}};
      }
    }
  }
  *known = false;
  return {{0.0, 0.0}};
}

std::string planJson(const Plan & plan, const Incident & inc)
{
  std::string j = "{\"schema_version\":\"1.0\",\"event_id\":" + cbba_core::jsonString(plan.event_id) +
    ",\"event_type\":" + cbba_core::jsonString(plan.event_type) + ",\"rule\":" + cbba_core::jsonString(plan.rule) +
    ",\"incident\":{\"revision\":" + std::to_string(plan.incident_revision) + ",\"location\":{\"frame\":\"map\",\"x\":" +
    fmt2(inc.x) + ",\"y\":" + fmt2(inc.y) + ",\"z\":" + fmt2(inc.z) + ",\"radius_m\":" + fmt2(inc.radius_m) +
    ",\"zone\":" + cbba_core::jsonString(plan.zone) + "},\"track_id\":" + cbba_core::jsonString(inc.track_id) + "}";
  if (!plan.roles.empty()) {
    const auto & r0 = plan.roles.front();
    j += ",\"priority\":\"P" + std::to_string(r0.priority) + "\",\"expires_s\":" + fmt2(r0.expires_s) +
      ",\"requires_approval\":" + (r0.requires_approval ? "true" : "false") +
      ",\"fallback_template\":" + cbba_core::jsonString(r0.fallback_template);
  }
  j += ",\"roles\":[";
  for (std::size_t i = 0; i < plan.roles.size(); ++i) {
    const auto & r = plan.roles[i];
    std::string caps = "[";
    for (std::size_t k = 0; k < r.required_capabilities.size(); ++k) {
      caps += (k ? "," : "") + cbba_core::jsonString(r.required_capabilities[k]);
    }
    caps += "]";
    j += (i ? "," : "") + std::string("{\"role\":") + cbba_core::jsonString(r.role) + ",\"platform\":" +
      cbba_core::jsonString(r.platform) + ",\"required_capabilities\":" + caps + ",\"optional\":" +
      (r.optional ? "true" : "false") + ",\"position\":{\"x\":" + fmt2(r.x) + ",\"y\":" + fmt2(r.y) + ",\"z\":" +
      fmt2(r.z) + "},\"location_source\":" + cbba_core::jsonString(r.location_source) + ",\"precedence\":" +
      std::to_string(r.precedence) + ",\"spec\":" + r.spec + "}";
  }
  j += "]";
  if (!plan.roles.empty()) {
    const auto & r0 = plan.roles.front();
    j += ",\"coupling\":{\"all_or_nothing\":" + std::string(r0.all_or_nothing ? "true" : "false") +
      ",\"distinct_agent_set\":" + std::to_string(r0.distinct_agent_set) + ",\"sync_window_s\":" +
      std::to_string(r0.sync_window_s) + "}";
  }
  return j + "}";
}

}  // namespace

std::string zoneAt(const swarm_cbba::Site & site, double x, double y)
{
  const swarm_cbba::Zone * best = nullptr;
  double best_area = 0.0;
  for (const auto & z : site.zones) {
    if (z.contains(x, y)) {
      const double a = polygonArea(z);
      if (best == nullptr || a < best_area) {
        best = &z;
        best_area = a;
      }
    }
  }
  return best ? best->id : std::string();
}

std::string forbiddenZone(const swarm_cbba::Site & site, const std::string & platform, double x, double y)
{
  if (platform == "any" || platform.empty()) {
    return "";
  }
  for (const auto & z : site.zones) {
    if (std::find(z.no_entry.begin(), z.no_entry.end(), platform) != z.no_entry.end() && z.contains(x, y)) {
      return z.id;
    }
  }
  return "";
}

std::optional<std::array<double, 2>> safeAnchor(const swarm_cbba::Site & site, const std::string & platform,
  double x, double y)
{
  if (forbiddenZone(site, platform, x, y).empty()) {
    return std::array<double, 2>{{x, y}};
  }
  constexpr double kPi = 3.14159265358979323846;
  for (double r = kAnchorStep; r <= kAnchorReach + 1e-9; r += kAnchorStep) {
    const int n = std::max(8, static_cast<int>(2.0 * kPi * r / kAnchorStep));
    for (int k = 0; k < n; ++k) {
      const double a = 2.0 * kPi * k / n;
      const double cx = x + r * std::cos(a);
      const double cy = y + r * std::sin(a);
      const double ox = x + (r + kAnchorMargin) * std::cos(a);
      const double oy = y + (r + kAnchorMargin) * std::sin(a);
      if (forbiddenZone(site, platform, cx, cy).empty() && forbiddenZone(site, platform, ox, oy).empty()) {
        return std::array<double, 2>{{std::round(ox * 100.0) / 100.0, std::round(oy * 100.0) / 100.0}};
      }
    }
  }
  return std::nullopt;
}

Plan expand(const Incident & incident, const EventRules & rules, const swarm_cbba::Registry & registry,
  const swarm_cbba::Site & site, const ExpandOptions & options)
{
  Plan plan;
  plan.event_id = incident.event_id;
  plan.event_type = incident.event_type;
  plan.incident_revision = incident.revision;
  const swarm_cbba::EventType * et = registry.eventType(incident.event_type);
  if (et == nullptr) {
    return failed(plan, "UNKNOWN_EVENT_TYPE", "事件類型 " + incident.event_type + " 不在 event_types.yaml");
  }
  plan.rule = options.rule.empty() ? et->default_rule : options.rule;
  const Rule * rule = rules.rule(plan.rule);
  if (rule == nullptr) {
    return failed(plan, "UNKNOWN_RULE", "沒有規則 " + (plan.rule.empty() ? std::string("（事件類型沒有 default_rule）") :
      plan.rule));
  }
  plan.zone = incident.zone.empty() ? zoneAt(site, incident.x, incident.y) : incident.zone;

  Subst subst;
  subst.values = {{"{incident.track_id}", incident.track_id}, {"{incident.object}", et->object_class},
    {"{incident.asset}", incident.asset}, {"{incident.zone}", plan.zone}, {"{incident.event_id}", incident.event_id}};

  const int priority = rule->priority ? *rule->priority : et->priority;
  const bool approval = rule->requires_approval ? *rule->requires_approval : et->requires_approval;
  std::uint8_t distinct = 0;
  for (const auto & d : rule->distinct) {
    const int idx = rule->roleIndex(d);
    if (idx >= 0) {
      distinct |= static_cast<std::uint8_t>(1u << idx);
    }
  }
  const auto count = static_cast<std::uint8_t>(rule->roles.size());

  for (std::size_t i = 0; i < rule->roles.size(); ++i) {
    const RoleRule & r = rule->roles[i];
    PlannedRole p;
    p.role = r.role;
    p.role_index = static_cast<std::uint8_t>(i);
    p.role_count = count;
    p.platform = r.platform;
    if (r.platform != "any") {
      const auto * pe = registry.platform(r.platform);
      p.platform_code = pe ? pe->code : 0;
    }
    p.required_capabilities = r.required_capabilities;
    p.priority = static_cast<std::uint8_t>(priority);
    p.optional = r.optional;
    p.skip_satisfies = r.skip_satisfies;
    p.all_or_nothing = rule->all_or_nothing;
    p.distinct_agent_set = distinct;
    p.sync_window_s = rule->sync_window_s;
    p.fallback_template = rule->fallback_template;
    p.requires_approval = approval;
    p.expires_s = et->expires_s;
    p.value = r.value;
    p.service_time_s = r.service_time_s;
    p.retarget = r.retarget;

    // ---- 1 位置 ----
    double x = incident.x + r.offset[0];
    double y = incident.y + r.offset[1];
    double z = incident.z;
    p.location_source = "incident";
    const std::string kind = r.location.kind;
    const std::string value = subst.apply(r.location.value);
    if (kind == "poi") {
      const swarm_cbba::Poi * best = nullptr;
      double best_d = 0.0;
      for (const auto & poi : site.pois) {
        if (poi.role != value || poi.zone != plan.zone || (r.platform != "any" && poi.platform != r.platform)) {
          continue;
        }
        const double d = std::hypot(poi.x - incident.x, poi.y - incident.y);
        if (best == nullptr || d < best_d) {
          best = &poi;
          best_d = d;
        }
      }
      if (best != nullptr) {
        x = best->x;
        y = best->y;
        z = best->z;
        p.location_source = "poi:" + best->id;
      } else {
        p.location_source = "incident (no " + value + " poi in " + (plan.zone.empty() ? "?" : plan.zone) + ")";
      }
    } else if (kind == "asset") {
      const auto * dev = site.device(value);
      if (dev == nullptr) {
        return failed(plan, "BAD_LOCATION", "角色 " + r.role + " 的設備 " + value + " 不在 devices.csv");
      }
      x = dev->x;
      y = dev->y;
      z = dev->z;
      if (r.platform == "quadruped" && dev->obs_quadruped) {
        x = (*dev->obs_quadruped)[0];
        y = (*dev->obs_quadruped)[1];
        z = 0.0;
      } else if (r.platform == "uav" && dev->obs_uav) {
        x = (*dev->obs_uav)[0];
        y = (*dev->obs_uav)[1];
        z = (*dev->obs_uav)[2];
      }
      p.location_source = "asset:" + value;
    } else if (kind == "zone") {
      const auto * zn = site.zone(value);
      if (zn == nullptr || zn->polygon.empty()) {
        return failed(plan, "BAD_LOCATION", "角色 " + r.role + " 的區域 " + value + " 不存在");
      }
      double sx = 0.0, sy = 0.0;
      const std::size_t n = zn->polygon.size() > 1 ? zn->polygon.size() - 1 : 1;   // 頭尾同一點
      for (std::size_t k = 0; k < n; ++k) {
        sx += zn->polygon[k][0];
        sy += zn->polygon[k][1];
      }
      x = sx / n;
      y = sy / n;
      p.location_source = "zone:" + value;
    } else if (kind == "dock") {
      const auto * dk = site.dock(value);
      if (dk == nullptr) {
        return failed(plan, "BAD_LOCATION", "角色 " + r.role + " 的起降點 " + value + " 不存在");
      }
      x = dk->x;
      y = dk->y;
      z = dk->z;
      p.location_source = "dock:" + value;
    }

    // ---- 2 preflight：限平台的角色，觀察點在禁入區 → 最近的可達點 ----
    const std::string fz = forbiddenZone(site, r.platform, x, y);
    if (!fz.empty()) {
      const auto anchor = safeAnchor(site, r.platform, x, y);
      if (!anchor) {
        return failed(plan, "BAD_LOCATION", "角色 " + r.role + " 的觀察點在 " + fz + " 裡，30 m 內找不到可達點");
      }
      p.location_source += " -> anchor (" + fmt2((*anchor)[0]) + ", " + fmt2((*anchor)[1]) + ") out of " + fz;
      x = (*anchor)[0];
      y = (*anchor)[1];
    }
    p.x = x;
    p.y = y;
    p.z = z;

    // ---- 3 steps ----
    std::vector<std::string> actions;
    std::set<int> waits;
    p.spec = roleSpec(*rule, r, stepsJson(r, *rule, subst, true, x, y, z, &actions, &waits));
    for (const auto & [a, b] : rule->precedence) {
      if (b == r.role) {
        waits.insert(rule->roleIndex(a));
      }
    }
    for (const int w : waits) {
      if (w >= 0 && w != static_cast<int>(i)) {
        p.precedence |= static_cast<std::uint8_t>(1u << w);
      }
    }

    // ---- 4 檢查：TaskSpec ----
    swarm_cbba::ValidateOptions vopt;
    vopt.platform_support_is_error = false;   // 和閘道一樣：平台還沒實作只警告
    const auto v = swarm_cbba::validateSpec(p.spec, registry, &site, p.platform_code, p.role_index, p.role_count, vopt);
    for (const auto & w : v.warnings) {
      plan.warnings.push_back(r.role + "：" + w.code + " " + w.message + (w.path.empty() ? "" : "（" + w.path + "）"));
    }
    if (!v.ok()) {
      plan.roles.push_back(p);
      return failed(plan, "SPEC_INVALID", "角色 " + r.role + "：" + v.summary());
    }

    // ---- 4 檢查：機隊可行性、續航 ----
    std::vector<const swarm_cbba::RobotEntry *> capable;
    for (const auto & robot : registry.robots) {
      if (!options.available.empty() &&
        std::find(options.available.begin(), options.available.end(), robot.id) == options.available.end())
      {
        continue;
      }
      if (r.platform != "any" && robot.platform != r.platform) {
        continue;
      }
      const bool has_caps = std::all_of(r.required_capabilities.begin(), r.required_capabilities.end(),
        [&robot](const std::string & c) {
          return std::find(robot.capabilities.begin(), robot.capabilities.end(), c) != robot.capabilities.end();
        });
      if (!has_caps) {
        continue;
      }
      const auto can = swarm_cbba::supportedActions(registry, robot.platform, robot.capabilities);
      if (!std::all_of(actions.begin(), actions.end(), [&can](const std::string & a) {return can.count(a) > 0;})) {
        continue;
      }
      capable.push_back(&robot);
    }
    if (capable.empty()) {
      if (!r.optional) {
        plan.roles.push_back(p);
        return failed(plan, "FLEET_INFEASIBLE", "角色 " + r.role + "：機隊沒有平台、能力、能做的動作都符合的載具");
      }
      plan.warnings.push_back(r.role + "：NO_CAPABLE optional 角色目前沒有能做的載具（閘道會略過）");
    }
    bool any_known = false;
    bool any_ok = false;
    for (const auto * robot : capable) {
      const auto * pe = registry.platform(robot->platform);
      bool known = false;
      const auto home = homeOf(*robot, site, &known);
      if (pe == nullptr || pe->endurance_s <= 0.0 || pe->cruise_speed <= 0.0 || !known) {
        any_ok = true;   // 不知道續航或起點：不判
        continue;
      }
      any_known = true;
      const double need = 2.0 * std::hypot(x - home[0], y - home[1]) / pe->cruise_speed + r.service_time_s;
      any_ok = any_ok || need <= pe->endurance_s;
    }
    if (any_known && !any_ok) {
      plan.warnings.push_back(r.role + "：ENDURANCE 每台能做的載具從起點來回＋作業都超過續航");
    }
    plan.roles.push_back(std::move(p));
  }
  plan.ok = true;
  plan.json = planJson(plan, incident);
  return plan;
}

std::vector<std::string> changedRoles(const Plan & before, const Plan & after, double retarget_m)
{
  // TaskSpec 裡 go_to 的座標是填進去的：小位移（≤ retarget_m）不算改，只看去掉座標後的內容
  const auto strip = [](std::string spec) {
      for (std::size_t pos = spec.find("\"location\":{\"x\":"); pos != std::string::npos;
        pos = spec.find("\"location\":{\"x\":", pos))
      {
        const std::size_t end = spec.find('}', pos);
        if (end == std::string::npos) {
          break;
        }
        spec.erase(pos, end - pos + 1);
      }
      return spec;
    };
  std::vector<std::string> out;
  for (const auto & a : after.roles) {
    const auto it = std::find_if(before.roles.begin(), before.roles.end(),
      [&a](const PlannedRole & b) {return b.role == a.role;});
    if (it == before.roles.end()) {
      out.push_back(a.role);
      continue;
    }
    const PlannedRole & b = *it;
    const bool moved = a.retarget && std::hypot(a.x - b.x, a.y - b.y) > retarget_m;
    const bool fields = a.platform_code != b.platform_code || a.required_capabilities != b.required_capabilities ||
      a.priority != b.priority || a.optional != b.optional || a.precedence != b.precedence ||
      a.distinct_agent_set != b.distinct_agent_set || a.all_or_nothing != b.all_or_nothing;
    if (moved || fields || strip(a.spec) != strip(b.spec)) {
      out.push_back(a.role);
    }
  }
  return out;
}

}  // namespace swarm_events
