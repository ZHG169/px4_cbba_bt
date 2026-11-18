#include "swarm_cbba/spec_validator.hpp"

#include <algorithm>
#include <cmath>
#include <set>

#include <yaml-cpp/yaml.h>

#include "cbba_core/failure_codes.hpp"
#include "cbba_core/milestones.hpp"

namespace swarm_cbba
{

std::string SpecValidation::summary() const
{
  if (errors.empty()) {
    return {};
  }
  const SpecIssue & e = errors.front();
  std::string out = e.code + "：" + e.message + (e.path.empty() ? "" : "（" + e.path + "）");
  if (errors.size() > 1) {
    out += "，另外還有 " + std::to_string(errors.size() - 1) + " 個錯誤";
  }
  return out;
}

namespace
{
bool number(const YAML::Node & n, double * out = nullptr)
{
  if (!n.IsScalar()) {
    return false;
  }
  try {
    const double v = n.as<double>();
    if (!std::isfinite(v)) {
      return false;
    }
    if (out != nullptr) {
      *out = v;
    }
    return true;
  } catch (const YAML::Exception &) {
    return false;
  }
}

bool boolean(const YAML::Node & n)
{
  if (!n.IsScalar()) {
    return false;
  }
  const std::string s = n.Scalar();
  return s == "true" || s == "false";
}

bool stringScalar(const YAML::Node & n)
{
  // JSON 的字串、數字在 yaml-cpp 都是純量；數字、true/false 當成型別不對
  return n.IsScalar() && !number(n) && !boolean(n);
}

class Checker
{
public:
  Checker(const Registry & reg, const Site * site, std::uint8_t platform, std::uint8_t role_index,
    std::uint8_t role_count, SpecValidation & out)
  : reg_(reg), site_(site), role_index_(role_index), role_count_(role_count), out_(out)
  {
    for (const PlatformEntry & p : reg.platforms) {
      if (platform == 0 || p.code == platform) {
        platforms_.push_back(p.name);
      }
    }
  }

  void error(const std::string & code, const std::string & path, const std::string & message)
  {
    out_.errors.push_back(SpecIssue{code, path, message});
  }
  void warning(const std::string & code, const std::string & path, const std::string & message)
  {
    out_.warnings.push_back(SpecIssue{code, path, message});
  }

  void run(const YAML::Node & root)
  {
    const YAML::Node steps = root["steps"];
    if (!steps || !steps.IsSequence() || steps.size() == 0) {
      error("SCHEMA", "steps", "要有 steps（1 步以上的清單）");
      return;
    }
    if (steps.size() > kMaxSteps) {
      error("TOO_MANY_STEPS", "steps", std::to_string(steps.size()) + " 步，上限 " + std::to_string(kMaxSteps));
    }
    for (std::size_t i = 0; i < steps.size(); ++i) {
      step(steps[i], "steps[" + std::to_string(i) + "]");
    }
    conditions(root);
    reportPolicy(root["report_policy"]);
    if (const YAML::Node t = root["timeout_s"]) {
      double v = 0.0;
      if (!number(t, &v) || v <= 0.0 || v > kMaxTaskTimeout) {
        error("BAD_TIMEOUT", "timeout_s", "要是 0～" + std::to_string(static_cast<int>(kMaxTaskTimeout)) + " 秒（不含 0）");
      }
    }
    if (const YAML::Node notes = root["safety_notes"]) {
      bool ok = notes.IsSequence();
      for (std::size_t i = 0; ok && i < notes.size(); ++i) {
        ok = stringScalar(notes[i]);
      }
      if (!ok) {
        error("SCHEMA", "safety_notes", "要是字串清單");
      }
    }
  }

private:
  void step(const YAML::Node & s, const std::string & at)
  {
    if (!s.IsMap()) {
      error("SCHEMA", at, "每一步要是物件");
      return;
    }
    const YAML::Node a = s["action"];
    if (!a || !stringScalar(a) || a.Scalar().empty()) {
      error("SCHEMA", at + ".action", "要有 action");
      return;
    }
    const std::string name = a.Scalar();
    const ActionEntry * action = reg_.action(name);
    if (action == nullptr) {
      error("UNKNOWN_ACTION", at + ".action", "不認得的動作 " + name + "（不在 actions.yaml）");
      return;
    }
    if (action->cls != "plannable" || action->status != "open") {
      error("ACTION_NOT_OPEN", at + ".action", name + " 不是可規劃且開放的動作（class " + action->cls + "、status " +
        action->status + "）");
      return;
    }
    effects_.insert(action->effects.begin(), action->effects.end());
    platform(*action, at);
    location(*action, s["location"], at + ".location");
    object(*action, s["object"], at + ".object");
    params(*action, s["params"], at + ".params");
    waitFor(s["wait_for"], at + ".wait_for");
    if (const YAML::Node sync = s["sync"]) {
      if (!stringScalar(sync) || sync.Scalar().empty()) {
        error("BAD_WAIT_FOR", at + ".sync", "sync 要是非空的會合點名稱");
      }
    }
    if (action->risk == "high") {
      out_.high_risk_actions.push_back(name);
    }
  }

  void platform(const ActionEntry & action, const std::string & at)
  {
    bool any = false;
    for (const std::string & p : platforms_) {
      const ActionImpl * impl = action.implementation(p);
      if (impl == nullptr || impl->autonomy == "not_started") {
        continue;
      }
      any = true;
      for (const std::string & e : impl->extra_effects) {
        effects_.insert(e);
      }
      if (impl->autonomy != "field_verified" &&
        std::find(out_.unverified_actions.begin(), out_.unverified_actions.end(), action.name) ==
        out_.unverified_actions.end())
      {
        out_.unverified_actions.push_back(action.name);
      }
    }
    if (!any) {
      std::string list;
      for (const std::string & p : platforms_) {
        list += (list.empty() ? "" : "、") + p;
      }
      error("PLATFORM_NOT_SUPPORTED", at + ".action", action.name + " 在 " + (list.empty() ? "（沒有平台）" : list) +
        " 還沒有實作（actions.yaml 的 implementations；not_started 不算）");
    }
  }

  void location(const ActionEntry & action, const YAML::Node & loc, const std::string & at)
  {
    if (!loc) {
      if (action.location == "required") {
        error("LOCATION_MISSING", at, action.name + " 要給 location");
      }
      return;
    }
    if (action.location == "none") {
      warning("PARAM_UNKNOWN", at, action.name + " 不用 location（會被忽略）");
      return;
    }
    if (loc.IsScalar()) {
      error("UNKNOWN_LOCATION", at, "location 是字串「" + loc.Scalar() + "」：範本的代換（{incident.zone} 等）要在注入前做完");
      return;
    }
    if (!loc.IsMap()) {
      error("UNKNOWN_LOCATION", at, "location 要是物件");
      return;
    }
    const auto ref = [&](const char * key) -> std::optional<std::string> {
        const YAML::Node v = loc[key];
        if (!v) {
          return std::nullopt;
        }
        return v.IsScalar() ? v.Scalar() : std::string();
      };
    if (const auto zone = ref("zone"); zone && !loc["x"]) {
      if (site_ != nullptr && site_->zone(*zone) == nullptr) {
        error("UNKNOWN_LOCATION", at + ".zone", "區域 " + *zone + " 不在 zones.geojson");
      }
      return;
    }
    if (const auto poi = ref("poi")) {
      if (site_ != nullptr && std::none_of(site_->pois.begin(), site_->pois.end(),
        [&](const Poi & p) {return p.id == *poi;}))
      {
        error("UNKNOWN_LOCATION", at + ".poi", "觀察點 " + *poi + " 不在 pois.csv");
      }
      return;
    }
    if (const auto asset = ref("asset")) {
      if (site_ != nullptr && site_->device(*asset) == nullptr) {
        error("UNKNOWN_LOCATION", at + ".asset", "設備 " + *asset + " 不在 devices.csv");
      }
      return;
    }
    if (const auto dock = ref("dock")) {
      if (site_ != nullptr && site_->dock(*dock) == nullptr) {
        error("UNKNOWN_LOCATION", at + ".dock", dock->empty() ? "dock 要是字串" : "起降點 " + *dock + " 不在 docks.csv");
      }
      return;
    }
    if (!number(loc["x"]) || !number(loc["y"]) || (loc["z"] && !number(loc["z"]))) {
      error("UNKNOWN_LOCATION", at, "location 要是 {zone}、{poi}、{asset}、{dock} 或 {x, y[, z]}");
    }
  }

  void object(const ActionEntry & action, const YAML::Node & obj, const std::string & at)
  {
    if (!obj) {
      if (action.object == "required") {
        error("OBJECT_MISSING", at, action.name + " 要給 object");
      }
      return;
    }
    if (!stringScalar(obj) || obj.Scalar().empty()) {
      error("SCHEMA", at, "object 要是「類別」或「類別:實例」");
      return;
    }
    if (action.object == "none") {
      warning("PARAM_UNKNOWN", at, action.name + " 不用 object（會被忽略）");
      return;
    }
    const std::string text = obj.Scalar();
    const auto colon = text.find(':');
    const std::string cls = text.substr(0, colon);
    if (std::find(action.objects.begin(), action.objects.end(), cls) == action.objects.end()) {
      error("OBJECT_NOT_ALLOWED", at, action.name + " 不能用在 " + cls + "（actions.yaml 的 objects）");
      return;
    }
    const ObjectClass * oc = reg_.objectClass(cls);
    if (colon != std::string::npos && site_ != nullptr && oc != nullptr &&
      std::find(oc->locate.begin(), oc->locate.end(), "asset") != oc->locate.end() && site_->device(text) == nullptr)
    {
      error("UNKNOWN_OBJECT", at, "設備 " + text + " 不在 devices.csv");
    }
  }

  void params(const ActionEntry & action, const YAML::Node & ps, const std::string & at)
  {
    if (ps && !ps.IsMap()) {
      error("PARAM_TYPE", at, "params 要是物件");
      return;
    }
    for (const ActionParam & p : action.params) {
      if (p.required && (!ps || !ps[p.name])) {
        error("PARAM_MISSING", at + "." + p.name, action.name + " 要給 " + p.name);
      }
    }
    if (!ps) {
      return;
    }
    for (const auto & kv : ps) {
      const std::string key = kv.first.as<std::string>();
      const std::string path = at + "." + key;
      const ActionParam * p = action.param(key);
      if (p == nullptr) {
        warning("PARAM_UNKNOWN", path, action.name + " 沒有參數 " + key + "（actions.yaml；BT 可能還在用舊名稱）");
        continue;
      }
      param(*p, kv.second, path, action.name);
    }
  }

  void param(const ActionParam & p, const YAML::Node & v, const std::string & path, const std::string & action)
  {
    const std::string what = action + " 的 " + p.name;
    double x = 0.0;
    if (p.type == "float" || p.type == "int") {
      if (!number(v, &x) || (p.type == "int" && x != std::floor(x))) {
        error("PARAM_TYPE", path, what + " 要是" + (p.type == "int" ? "整數" : "數字"));
        return;
      }
      if ((p.min && x < *p.min) || (p.max && x > *p.max)) {
        error("PARAM_OUT_OF_RANGE", path, what + " 是 " + v.Scalar() + "，範圍 " +
          (p.min ? std::to_string(*p.min) : std::string("-∞")) + "～" + (p.max ? std::to_string(*p.max) : std::string("∞")));
      }
    } else if (p.type == "bool") {
      if (!boolean(v)) {
        error("PARAM_TYPE", path, what + " 要是 true／false");
      }
    } else if (p.type == "string") {
      if (!stringScalar(v)) {
        error("PARAM_TYPE", path, what + " 要是字串");
      }
    } else if (p.type == "enum") {
      if (!v.IsScalar() || std::find(p.values.begin(), p.values.end(), v.Scalar()) == p.values.end()) {
        std::string values;
        for (const std::string & s : p.values) {
          values += (values.empty() ? "" : "、") + s;
        }
        error("PARAM_OUT_OF_RANGE", path, what + " 要是 " + values + " 之一");
      }
    } else if (p.type == "string_list") {
      bool ok = v.IsSequence();
      for (std::size_t i = 0; ok && i < v.size(); ++i) {
        ok = stringScalar(v[i]);
      }
      if (!ok) {
        error("PARAM_TYPE", path, what + " 要是字串清單");
      }
    } else if (p.type == "waypoints") {
      bool ok = v.IsSequence() && v.size() > 0;
      for (std::size_t i = 0; ok && i < v.size(); ++i) {
        const YAML::Node w = v[i];
        ok = (w.IsSequence() && (w.size() == 2 || w.size() == 3) && number(w[0]) && number(w[1])) ||
          (w.IsMap() && number(w["x"]) && number(w["y"]));
      }
      if (!ok) {
        error("PARAM_TYPE", path, what + " 要是航點清單（[x, y] 或 {x, y}）");
      }
    } else if (p.type == "bbox") {
      bool ok = v.IsSequence() && v.size() == 4;
      for (std::size_t i = 0; ok && i < 4; ++i) {
        ok = number(v[i]);
      }
      if (!ok) {
        error("PARAM_TYPE", path, what + " 要是 [x, y, w, h]");
      }
    }
  }

  void waitFor(const YAML::Node & w, const std::string & at)
  {
    if (!w) {
      return;
    }
    if (!w.IsSequence()) {
      error("BAD_WAIT_FOR", at, "wait_for 要是清單");
      return;
    }
    for (std::size_t i = 0; i < w.size(); ++i) {
      const std::string path = at + "[" + std::to_string(i) + "]";
      double role = -1.0;
      if (!w[i].IsMap() || !number(w[i]["role_index"], &role) || role != std::floor(role)) {
        error("BAD_WAIT_FOR", path, "要有整數的 role_index");
        continue;
      }
      if (role_count_ == 0) {
        error("BAD_WAIT_FOR", path, "單獨任務不能 wait_for 別的角色");
      } else if (role < 0 || role >= role_count_ || role == role_index_) {
        error("BAD_WAIT_FOR", path, "role_index " + std::to_string(static_cast<int>(role)) + " 要是同組的其他角色（0～" +
          std::to_string(role_count_ - 1) + "，不是自己 " + std::to_string(role_index_) + "）");
      }
      const YAML::Node m = w[i]["milestone"];
      if (m && (!m.IsScalar() || !cbba_core::milestoneFromName(m.Scalar()))) {
        error("BAD_WAIT_FOR", path + ".milestone", "不認得的里程碑 " + (m.IsScalar() ? m.Scalar() : std::string("?")));
      }
    }
  }

  void reportPolicy(const YAML::Node & rp)
  {
    if (!rp) {
      return;
    }
    if (!rp.IsMap()) {
      error("BAD_REPORT_POLICY", "report_policy", "要是物件");
      return;
    }
    std::set<std::string> findings;
    for (const ObjectClass & c : reg_.object_classes) {
      findings.insert(c.findings.begin(), c.findings.end());
    }
    for (const auto & kv : rp) {
      const std::string key = kv.first.as<std::string>();
      const std::string at = "report_policy." + key;
      const YAML::Node v = kv.second;
      double x = 0.0;
      if (key == "stream") {
        if (!boolean(v)) {
          error("BAD_REPORT_POLICY", at, "要是 true／false");
        }
      } else if (key == "snapshot_period_s") {
        if (!number(v, &x) || x < 0.0 || x > 3600.0) {
          error("BAD_REPORT_POLICY", at, "要是 0～3600 秒（0 = 不定期拍）");
        }
      } else if (key == "channels" || key == "required_findings") {
        bool ok = v.IsSequence();
        for (std::size_t i = 0; ok && i < v.size(); ++i) {
          ok = stringScalar(v[i]);
          if (ok && key == "channels" && v[i].Scalar() != "gcs" && v[i].Scalar() != "vms" && v[i].Scalar() != "operator") {
            error("BAD_REPORT_POLICY", at + "[" + std::to_string(i) + "]", "回寫頻道要是 gcs、vms、operator");
          }
          if (ok && key == "required_findings" && findings.count(v[i].Scalar()) == 0) {
            error("BAD_REPORT_POLICY", at + "[" + std::to_string(i) + "]", "不認得的 findings " + v[i].Scalar() +
              "（objects.yaml 的 findings）");
          }
        }
        if (!ok) {
          error("BAD_REPORT_POLICY", at, "要是字串清單");
        }
      } else {
        error("BAD_REPORT_POLICY", at, "不認得的鍵（stream、snapshot_period_s、channels、required_findings）");
      }
    }
  }

  void conditions(const YAML::Node & root)
  {
    if (const YAML::Node sc = root["success_criteria"]) {
      if (!sc.IsSequence()) {
        error("BAD_CONDITION", "success_criteria", "要是清單");
      } else {
        for (std::size_t i = 0; i < sc.size(); ++i) {
          const std::string at = "success_criteria[" + std::to_string(i) + "]";
          if (!sc[i].IsMap() || sc[i].size() != 1 || !sc[i]["effect"] || !sc[i]["effect"].IsScalar()) {
            error("BAD_CONDITION", at, "每個成功條件是 {\"effect\": 名稱}");
            continue;
          }
          const std::string e = sc[i]["effect"].Scalar();
          const bool known = std::any_of(reg_.conditions.begin(), reg_.conditions.end(),
              [&](const ConditionEntry & c) {return c.name == e;});
          if (!known) {
            error("BAD_CONDITION", at + ".effect", "不認得的條件 " + e + "（actions.yaml 的 conditions）");
          } else if (effects_.count(e) == 0) {
            error("BAD_CONDITION", at + ".effect", "沒有任何一步會產生 " + e + "（動作的 effects）：永遠不會成功");
          }
        }
      }
    }
    if (const YAML::Node ac = root["abort_conditions"]) {
      if (!ac.IsSequence()) {
        error("BAD_CONDITION", "abort_conditions", "要是清單");
        return;
      }
      for (std::size_t i = 0; i < ac.size(); ++i) {
        const std::string at = "abort_conditions[" + std::to_string(i) + "]";
        const YAML::Node c = ac[i];
        if (!c.IsMap() || c.size() != 1) {
          error("BAD_CONDITION", at, "每個中止條件剛好一個鍵：failure_code、battery_below_pct、elapsed_s、zone_entered");
          continue;
        }
        double v = 0.0;
        if (const YAML::Node f = c["failure_code"]) {
          if (!f.IsScalar() || !cbba_core::failureCodeInfo(f.Scalar())) {
            error("BAD_CONDITION", at + ".failure_code", "要是 8 個 E_* 之一");
          }
        } else if (const YAML::Node b = c["battery_below_pct"]) {
          if (!number(b, &v) || v < 0.0 || v > 100.0) {
            error("BAD_CONDITION", at + ".battery_below_pct", "要是 0～100");
          }
        } else if (const YAML::Node e = c["elapsed_s"]) {
          if (!number(e, &v) || v <= 0.0) {
            error("BAD_CONDITION", at + ".elapsed_s", "要 > 0");
          }
        } else if (const YAML::Node z = c["zone_entered"]) {
          if (!z.IsScalar() || (site_ != nullptr && site_->zone(z.Scalar()) == nullptr)) {
            error("BAD_CONDITION", at + ".zone_entered", "區域不在 zones.geojson");
          }
        } else {
          error("BAD_CONDITION", at, "不認得的中止條件（failure_code、battery_below_pct、elapsed_s、zone_entered）");
        }
      }
    }
  }

  const Registry & reg_;
  const Site * site_;
  std::uint8_t role_index_;
  std::uint8_t role_count_;
  SpecValidation & out_;
  std::vector<std::string> platforms_;
  std::set<std::string> effects_;   // 這些步驟會產生的效果（success_criteria 要有人產生）
};
}  // namespace

SpecValidation validateSpec(const std::string & spec, const Registry & registry, const Site * site,
  std::uint8_t platform, std::uint8_t role_index, std::uint8_t role_count)
{
  SpecValidation out;
  if (spec.empty()) {
    return out;
  }
  YAML::Node root;
  try {
    root = YAML::Load(spec);
  } catch (const YAML::Exception & e) {
    out.errors.push_back(SpecIssue{"SCHEMA", "", std::string("不是合法的 JSON：") + e.what()});
    return out;
  }
  if (!root.IsMap()) {
    out.errors.push_back(SpecIssue{"SCHEMA", "", "spec 要是 JSON 物件"});
    return out;
  }
  Checker(registry, site, platform, role_index, role_count, out).run(root);
  return out;
}

ReportPolicy specReportPolicy(const std::string & spec)
{
  ReportPolicy out;
  if (spec.empty()) {
    return out;
  }
  try {
    const YAML::Node root = YAML::Load(spec);
    const YAML::Node rp = root.IsMap() ? root["report_policy"] : YAML::Node();
    if (!rp || !rp.IsMap()) {
      return out;
    }
    out.stream = rp["stream"] && boolean(rp["stream"]) && rp["stream"].Scalar() == "true";
    double v = 0.0;
    if (rp["snapshot_period_s"] && number(rp["snapshot_period_s"], &v)) {
      out.snapshot_period_s = v;
    }
    for (const char * key : {"channels", "required_findings"}) {
      const YAML::Node list = rp[key];
      if (!list || !list.IsSequence()) {
        continue;
      }
      for (std::size_t i = 0; i < list.size(); ++i) {
        if (list[i].IsScalar()) {
          (std::string(key) == "channels" ? out.channels : out.required_findings).push_back(list[i].Scalar());
        }
      }
    }
  } catch (const YAML::Exception &) {
  }
  return out;
}

std::optional<double> specTimeout(const std::string & spec)
{
  if (spec.empty()) {
    return std::nullopt;
  }
  try {
    const YAML::Node root = YAML::Load(spec);
    double v = 0.0;
    if (root.IsMap() && root["timeout_s"] && number(root["timeout_s"], &v) && v > 0.0) {
      return v;
    }
  } catch (const YAML::Exception &) {
  }
  return std::nullopt;
}

}  // namespace swarm_cbba
