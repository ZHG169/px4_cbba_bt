#include "swarm_cbba/fake_executor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <yaml-cpp/yaml.h>

#include "swarm_cbba/spec_validator.hpp"

namespace swarm_cbba
{

namespace
{
using Pos = std::array<double, 3>;

constexpr double kGroundTolerance = 0.2;   // 離地面這麼近算在地上（UAV）

double dist(const Pos & a, const Pos & b)
{
  return std::hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

std::string jsonString(const std::string & s)
{
  std::string out = "\"";
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out + "\"";
}

std::string jsonNumber(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3f", v);
  return buf;
}

// JSON 物件（yaml-cpp 解析）；空字串 = 空物件。不是物件時 nullopt
std::optional<YAML::Node> parseObject(const std::string & text)
{
  if (text.empty()) {
    return YAML::Node(YAML::NodeType::Map);
  }
  try {
    YAML::Node n = YAML::Load(text);
    if (n.IsNull()) {
      return YAML::Node(YAML::NodeType::Map);
    }
    if (!n.IsMap()) {
      return std::nullopt;
    }
    return n;
  } catch (const YAML::Exception &) {
    return std::nullopt;
  }
}

bool isSystemAction(const std::string & name)
{
  return name == "emergency_stop" || name == "emergency_land" || name == "report_state";
}
}  // namespace

const char * toString(StepStatus s)
{
  switch (s) {
    case StepStatus::RUNNING: return "RUNNING";
    case StepStatus::SUCCEEDED: return "SUCCEEDED";
    case StepStatus::FAILED: return "FAILED";
    case StepStatus::CANCELED: return "CANCELED";
    case StepStatus::TIMEOUT: return "TIMEOUT";
  }
  return "?";
}

const char * toString(PlatformState s)
{
  switch (s) {
    case PlatformState::LANDED: return "LANDED";
    case PlatformState::AIRBORNE: return "AIRBORNE";
    case PlatformState::READY: return "READY";
    case PlatformState::EMERGENCY: return "EMERGENCY";
  }
  return "?";
}

FakeExecutor::FakeExecutor(const Registry & registry, const Site * site, const FakeExecutorConfig & config)
: registry_(registry), site_(site), config_(config)
{
  uav_ = config_.platform == "uav";
  platform_ = uav_ ? PlatformState::LANDED : PlatformState::READY;
  pos_ = config_.home;
  last_snapshot_ = snapshot();
}

std::optional<StepFault> FakeExecutor::parseFault(const std::string & text)
{
  const auto eq = text.find('=');
  if (eq == std::string::npos) {
    return std::nullopt;
  }
  StepFault f;
  f.action = text.substr(0, eq);
  std::string rest = text.substr(eq + 1);
  const auto at = rest.find('@');
  if (at != std::string::npos) {
    try {
      f.at = std::stod(rest.substr(at + 1));
    } catch (const std::exception &) {
      return std::nullopt;
    }
    if (!(f.at >= 0.0 && f.at <= 1.0)) {
      return std::nullopt;
    }
    rest = rest.substr(0, at);
  }
  const auto colon = rest.find(':');
  f.failure_code = rest.substr(0, colon);
  if (colon != std::string::npos) {
    f.failed_capability = rest.substr(colon + 1);
  }
  if (f.failure_code.rfind("E_", 0) != 0) {
    return std::nullopt;
  }
  return f;
}

// ===========================================================================
// 開始：檢查 → 排出各段 → 逾時
// ===========================================================================
StepSnapshot FakeExecutor::start(const StepGoal & goal, double now)
{
  active_ = false;
  goal_ = goal;
  segments_.clear();
  seg_ = 0;
  seg_elapsed_ = 0.0;
  done_ = 0.0;
  total_ = 0.0;
  lands_ = false;
  findings_ = "[]";
  fault_.reset();
  warnings_.clear();
  started_ = now;
  last_ = now;
  if (!uav_ && platform_ == PlatformState::EMERGENCY) {
    platform_ = PlatformState::READY;   // 狗：急停之後的下一個 goal 恢復
  }

  const ActionEntry * entry = registry_.action(goal.action);
  const PlatformEntry * platform = registry_.platform(config_.platform);
  if (platform == nullptr) {
    return finish(StepStatus::FAILED, "E_PRECONDITION", "平台「" + config_.platform + "」不在 platforms.yaml");
  }
  if (entry != nullptr && isSystemAction(goal.action)) {
    // 系統動作不是 plannable（validateSpec 不收）：只看這個平台有沒有實作
    if (entry->implementation(config_.platform) == nullptr) {
      return finish(StepStatus::FAILED, "E_PRECONDITION",
        "PLATFORM_NOT_SUPPORTED：" + config_.platform + " 沒有 " + goal.action + " 的實作");
    }
  } else {
    // 和閘道注入前的檢查同一套：包成只有一步的 spec
    std::string step = "{\"action\": " + jsonString(goal.action);
    if (!goal.object.empty()) {
      step += ", \"object\": " + jsonString(goal.object);
    }
    if (!goal.location_json.empty()) {
      step += ", \"location\": " + goal.location_json;
    }
    if (!goal.params_json.empty()) {
      step += ", \"params\": " + goal.params_json;
    }
    step += "}";
    const SpecValidation v = validateSpec("{\"steps\": [" + step + "]}", registry_, site_, platform->code, 0, 0);
    const ActionImpl * impl = entry != nullptr ? entry->implementation(config_.platform) : nullptr;
    for (const SpecIssue & e : v.errors) {
      if (e.code == "PLATFORM_NOT_SUPPORTED" && config_.allow_not_started && impl != nullptr &&
        impl->autonomy == "not_started")
      {
        warnings_ += std::string(warnings_.empty() ? "警告：" : "；") + goal.action + " 在 " + config_.platform +
          " 的實作是 not_started（閘道會擋；這裡照樣模擬）";
        continue;
      }
      return finish(StepStatus::FAILED, "E_PRECONDITION", e.code + " " + e.path + "：" + e.message);
    }
    for (const SpecIssue & w : v.warnings) {
      warnings_ += (warnings_.empty() ? "警告：" : "；") + w.code + " " + w.path + " " + w.message;
    }
  }

  if (const auto why = plan(goal)) {
    return finish(StepStatus::FAILED, "E_PRECONDITION", *why);
  }
  total_ = estimate();
  if (goal.deadline > 0.0) {
    deadline_ = goal.deadline;
  } else if (entry != nullptr && entry->timeout_s > 0.0) {
    deadline_ = now + entry->timeout_s;
  } else {
    deadline_ = now + total_ * 1.5 + 10.0;
  }
  for (const StepFault & f : config_.faults) {
    if (f.action.empty() || f.action == goal.action) {
      fault_ = f;
      break;
    }
  }
  active_ = true;
  last_snapshot_ = snapshot();
  last_snapshot_.detail = warnings_;
  return last_snapshot_;
}

std::optional<std::string> FakeExecutor::plan(const StepGoal & goal)
{
  const auto params = parseObject(goal.params_json);
  if (!params) {
    return "params_json 不是 JSON 物件";
  }
  const ActionEntry * entry = registry_.action(goal.action);
  const auto number = [&](const std::string & name, double fallback) {
      if ((*params)[name] && (*params)[name].IsScalar()) {
        try {
          return (*params)[name].as<double>();
        } catch (const YAML::Exception &) {
        }
      }
      if (entry != nullptr) {
        if (const ActionParam * p = entry->param(name); p != nullptr && p->has_default) {
          try {
            return std::stod(p->default_value);
          } catch (const std::exception &) {
          }
        }
      }
      return fallback;
    };
  const auto text = [&](const std::string & name, const std::string & fallback) {
      if ((*params)[name] && (*params)[name].IsScalar()) {
        return (*params)[name].as<std::string>();
      }
      if (entry != nullptr) {
        if (const ActionParam * p = entry->param(name); p != nullptr && p->has_default) {
          return p->default_value;
        }
      }
      return fallback;
    };
  const std::string uri = "evidence://fake/" + std::to_string(goal.task_id) + "/" + std::to_string(goal.step_id);
  const std::string obj = goal.object.empty() ? "null" : jsonString(goal.object);
  cursor_ = pos_;
  plan_airborne_ = platform_ == PlatformState::AIRBORNE;

  const std::string & a = goal.action;
  if (a == "go_to") {
    std::string why;
    auto target = resolveLocation(goal.location_json, &why);
    if (!target) {
      return why;
    }
    const double standoff = number("standoff_m", 0.0);
    const double dx = (*target)[0] - cursor_[0];
    const double dy = (*target)[1] - cursor_[1];
    const double horizontal = std::hypot(dx, dy);
    if (standoff > 0.0 && horizontal > 1e-6) {
      const double keep = std::min(standoff, horizontal);
      (*target)[0] -= dx / horizontal * keep;
      (*target)[1] -= dy / horizontal * keep;
    }
    if (uav_) {
      double altitude = config_.cruise_altitude;
      if (const auto zone = zoneAltitude(goal.location_json)) {
        altitude = *zone;
      }
      if ((*params)["altitude_m"]) {
        altitude = number("altitude_m", altitude);
      }
      (*target)[2] = config_.home[2] + altitude;
      addTakeoffIfNeeded();
      if (std::fabs(cursor_[2] - (*target)[2]) > kGroundTolerance) {
        addMove("climbing", {{cursor_[0], cursor_[1], (*target)[2]}});
      }
    }
    addMove("moving", *target);
  } else if (a == "patrol") {
    std::vector<Pos> points;
    const YAML::Node wps = (*params)["waypoints"];
    if (wps && wps.IsSequence()) {
      for (const auto & w : wps) {
        if (w.IsSequence() && w.size() >= 2) {
          points.push_back({{w[0].as<double>(), w[1].as<double>(), w.size() > 2 ? w[2].as<double>() : 0.0}});
        } else if (w.IsMap() && w["x"] && w["y"]) {
          points.push_back({{w["x"].as<double>(), w["y"].as<double>(), w["z"] ? w["z"].as<double>() : 0.0}});
        } else {
          return "waypoints 的每一點要是 [x, y]、[x, y, z] 或 {x, y}";
        }
      }
    } else if ((*params)["area"]) {
      const std::string area = (*params)["area"].as<std::string>();
      const Zone * z = site_ != nullptr ? site_->zone(area) : nullptr;
      if (z == nullptr) {
        return "patrol 的 area「" + area + "」找不到（沒有 site_dir 或不在 zones.geojson）";
      }
      for (std::size_t k = 0; k + 1 < z->polygon.size(); ++k) {
        points.push_back({{z->polygon[k][0], z->polygon[k][1], 0.0}});
      }
    }
    if (points.empty()) {
      return "patrol 要 waypoints 或 area";
    }
    const int loops = std::max(1, static_cast<int>(number("loops", 1.0)));
    if (uav_) {
      addTakeoffIfNeeded();
    }
    for (int l = 0; l < loops; ++l) {
      for (Pos p : points) {
        if (uav_ && p[2] <= 0.0) {
          p[2] = config_.home[2] + config_.cruise_altitude;
        }
        addMove("patrolling", p);
      }
    }
  } else if (a == "inspect") {
    if (uav_) {
      addTakeoffIfNeeded();
    }
    const std::string mode = text("mode", "rgb");
    addHold("aiming", 1.0);
    addHold("capturing", number("duration_s", 5.0));
    findings_ = "[{\"type\": \"snapshot\", \"mode\": " + jsonString(mode) + ", \"object\": " + obj + ", \"uri\": " +
      jsonString(uri + ".jpg") + "}]";
  } else if (a == "track") {
    if (uav_) {
      addTakeoffIfNeeded();
    }
    const double duration = number("duration_s", 30.0);
    addHold("tracking", duration);
    findings_ = "[{\"type\": \"track\", \"object\": " + obj + ", \"duration_s\": " + jsonNumber(duration) +
      ", \"distance_m\": " + jsonNumber(number("distance_m", 2.0)) + "}]";
  } else if (a == "wait") {
    addHold("waiting", number("duration_s", 0.0));
  } else if (a == "speak") {
    addHold("speaking", 2.0);
    findings_ = "[{\"type\": \"speech\", \"template\": " + jsonString(text("template", "")) + "}]";
  } else if (a == "alert_operator") {
    addHold("notifying", 0.5);
    findings_ = "[{\"type\": \"alert\", \"severity\": " + jsonString(text("severity", "warning")) + "}]";
  } else if (a == "return_to_dock") {
    Pos dock = config_.home;
    if ((*params)["dock"]) {
      const std::string id = (*params)["dock"].as<std::string>();
      const Dock * d = site_ != nullptr ? site_->dock(id) : nullptr;
      if (d == nullptr) {
        return "dock「" + id + "」找不到（沒有 site_dir 或不在 docks.csv）";
      }
      dock = {{d->x, d->y, d->z}};
    }
    if (uav_) {
      const double hx = cursor_[0] - dock[0];
      const double hy = cursor_[1] - dock[1];
      if (std::hypot(hx, hy) > kGroundTolerance || plan_airborne_) {
        addTakeoffIfNeeded();
        addMove("moving", {{dock[0], dock[1], cursor_[2]}});
        addMove("landing", dock);
      }
      lands_ = true;
    } else {
      addMove("moving", dock);
    }
  } else if (a == "emergency_land") {
    if (uav_) {
      platform_ = PlatformState::EMERGENCY;
      addMove("landing", {{cursor_[0], cursor_[1], config_.home[2]}});
      lands_ = true;
    }
  } else if (a == "emergency_stop") {
    addHold("stopping", 0.0);
  } else if (a == "report_state") {
    addHold("reporting", 0.0);
    findings_ = "[{\"type\": \"state\", \"platform_state\": " + jsonString(toString(platform_)) +
      ", \"position\": [" + jsonNumber(pos_[0]) + ", " + jsonNumber(pos_[1]) + ", " + jsonNumber(pos_[2]) + "]}]";
  } else {
    addHold("executing", 1.0);
  }
  return std::nullopt;
}

std::optional<std::array<double, 3>> FakeExecutor::resolveLocation(const std::string & json, std::string * why) const
{
  const auto loc = parseObject(json);
  if (!loc || loc->size() == 0) {
    *why = "location 不是 JSON 物件或是空的";
    return std::nullopt;
  }
  const auto & l = *loc;
  if (l["x"] && l["y"]) {
    return Pos{{l["x"].as<double>(), l["y"].as<double>(), l["z"] ? l["z"].as<double>() : 0.0}};
  }
  if (site_ == nullptr) {
    *why = "沒有 site_dir：location 只能是座標（x、y）";
    return std::nullopt;
  }
  if (l["zone"]) {
    const Zone * z = site_->zone(l["zone"].as<std::string>());
    if (z != nullptr && z->polygon.size() > 1) {
      double sx = 0.0, sy = 0.0;
      const std::size_t n = z->polygon.size() - 1;   // 頭尾同一點
      for (std::size_t k = 0; k < n; ++k) {
        sx += z->polygon[k][0];
        sy += z->polygon[k][1];
      }
      return Pos{{sx / n, sy / n, 0.0}};
    }
  } else if (l["poi"]) {
    const std::string id = l["poi"].as<std::string>();
    for (const Poi & p : site_->pois) {
      if (p.id == id) {
        return Pos{{p.x, p.y, p.z}};
      }
    }
  } else if (l["asset"]) {
    const Device * d = site_->device(l["asset"].as<std::string>());
    if (d != nullptr) {
      if (uav_ && d->obs_uav) {
        return Pos{{(*d->obs_uav)[0], (*d->obs_uav)[1], (*d->obs_uav)[2]}};
      }
      if (!uav_ && d->obs_quadruped) {
        return Pos{{(*d->obs_quadruped)[0], (*d->obs_quadruped)[1], 0.0}};
      }
      return Pos{{d->x, d->y, d->z}};
    }
  } else if (l["dock"]) {
    const Dock * d = site_->dock(l["dock"].as<std::string>());
    if (d != nullptr) {
      return Pos{{d->x, d->y, d->z}};
    }
  }
  *why = "location " + json + " 在場域設定裡找不到";
  return std::nullopt;
}

std::optional<double> FakeExecutor::zoneAltitude(const std::string & json) const
{
  const auto loc = parseObject(json);
  if (!loc || site_ == nullptr || !(*loc)["zone"]) {
    return std::nullopt;
  }
  const Zone * z = site_->zone((*loc)["zone"].as<std::string>());
  if (z == nullptr || !z->cruise_altitude_m) {
    return std::nullopt;
  }
  return *z->cruise_altitude_m;
}

void FakeExecutor::addTakeoffIfNeeded()
{
  if (!uav_ || plan_airborne_) {
    return;
  }
  addMove("takeoff", {{cursor_[0], cursor_[1], config_.home[2] + config_.takeoff_altitude}});
  plan_airborne_ = true;
}

void FakeExecutor::addMove(const std::string & phase, std::array<double, 3> target)
{
  Segment s;
  s.phase = phase;
  s.move = true;
  s.target = target;
  s.duration = dist(cursor_, target) / std::max(config_.speed, 1e-3);
  segments_.push_back(s);
  cursor_ = target;
}

void FakeExecutor::addHold(const std::string & phase, double seconds)
{
  Segment s;
  s.phase = phase;
  s.duration = std::max(seconds, 0.0);
  segments_.push_back(s);
}

double FakeExecutor::estimate() const
{
  double t = 0.0;
  for (const Segment & s : segments_) {
    t += s.duration;
  }
  return t;
}

// ===========================================================================
// 執行
// ===========================================================================
StepSnapshot FakeExecutor::step(double now)
{
  if (!active_) {
    return last_snapshot_;
  }
  double dt = std::max(now - last_, 0.0);
  last_ = now;
  const double speed = std::max(config_.speed * config_.slowdown, 1e-3);
  while (seg_ < segments_.size() && dt >= 0.0) {
    Segment & s = segments_[seg_];
    bool finished = false;
    if (s.move) {
      const double d = dist(pos_, s.target);
      if (speed * dt >= d) {
        pos_ = s.target;
        dt -= d / speed;
        finished = true;
      } else {
        for (int k = 0; k < 3; ++k) {
          pos_[k] += (s.target[k] - pos_[k]) * (speed * dt / d);
        }
        dt = -1.0;
      }
    } else {
      if (seg_elapsed_ + dt >= s.duration) {
        dt -= s.duration - seg_elapsed_;
        finished = true;
      } else {
        seg_elapsed_ += dt;
        dt = -1.0;
      }
    }
    if (!finished) {
      break;
    }
    if (s.phase == "takeoff") {
      platform_ = PlatformState::AIRBORNE;
    }
    done_ += s.duration;
    seg_elapsed_ = 0.0;
    ++seg_;
  }

  StepSnapshot snap = snapshot();
  if (fault_ && snap.progress >= fault_->at) {
    return finish(StepStatus::FAILED, fault_->failure_code, "故障注入：" + goal_.action + " 在進度 " +
      jsonNumber(fault_->at) + " 時 " + fault_->failure_code, fault_->failed_capability);
  }
  if (seg_ >= segments_.size()) {
    if (lands_) {
      platform_ = PlatformState::LANDED;
    }
    if (goal_.action == "emergency_stop") {
      platform_ = PlatformState::EMERGENCY;
    }
    return finish(StepStatus::SUCCEEDED, "", "");
  }
  if (now > deadline_) {
    return finish(StepStatus::TIMEOUT, "E_TIMEOUT", "超過期限（估計 " + jsonNumber(total_) + " s，已經 " +
      jsonNumber(now - started_) + " s）");
  }
  last_snapshot_ = snap;
  return snap;
}

StepSnapshot FakeExecutor::cancel(const std::string & detail)
{
  if (!active_) {
    return last_snapshot_;
  }
  return finish(StepStatus::CANCELED, "", detail);
}

StepSnapshot FakeExecutor::finish(StepStatus status, const std::string & code, const std::string & detail,
  const std::string & capability)
{
  active_ = false;
  if (uav_ && platform_ != PlatformState::EMERGENCY && status != StepStatus::SUCCEEDED) {
    // 中途停下：在空中就是 AIRBORNE（懸停），在地上就是 LANDED
    platform_ = pos_[2] > config_.home[2] + kGroundTolerance ? PlatformState::AIRBORNE : PlatformState::LANDED;
  }
  StepSnapshot s = snapshot();
  s.status = status;
  if (status == StepStatus::SUCCEEDED) {
    s.progress = 1.0;
    s.findings_json = findings_;
  }
  s.failure_code = code;
  s.failed_capability = capability;
  s.detail = warnings_.empty() ? detail : (detail.empty() ? warnings_ : detail + "；" + warnings_);
  last_snapshot_ = s;
  return s;
}

StepSnapshot FakeExecutor::snapshot() const
{
  StepSnapshot s;
  s.platform = platform_;
  s.position = pos_;
  if (!active_ && segments_.empty()) {
    return s;
  }
  double partial = 0.0;
  if (seg_ < segments_.size()) {
    const Segment & cur = segments_[seg_];
    s.phase = cur.phase;
    if (cur.move) {
      const double remaining = dist(pos_, cur.target) / std::max(config_.speed, 1e-3);
      partial = std::max(cur.duration - remaining, 0.0);
    } else {
      partial = seg_elapsed_;
    }
  }
  s.progress = total_ > 0.0 ? std::clamp((done_ + partial) / total_, 0.0, 1.0) : 0.0;
  return s;
}

}  // namespace swarm_cbba
