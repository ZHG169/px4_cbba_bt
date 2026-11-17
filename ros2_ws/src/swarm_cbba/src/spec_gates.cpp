#include "swarm_cbba/spec_gates.hpp"

#include <map>
#include <set>

#include <yaml-cpp/yaml.h>

namespace swarm_cbba
{

using cbba_core::Milestone;
using cbba_core::WaitFor;

SpecGates parseSpecGates(const std::string & spec, std::uint8_t precedence, std::uint8_t self)
{
  SpecGates out;
  std::map<std::uint8_t, std::vector<WaitFor>> by_step;
  std::map<std::uint8_t, std::string> syncs;
  std::set<std::uint8_t> mentioned;

  if (!spec.empty()) {
    YAML::Node root;
    try {
      root = YAML::Load(spec);
    } catch (const YAML::Exception & e) {
      out.warnings.push_back(std::string("spec 不是合法的 JSON（wait_for 不看）：") + e.what());
    }
    const YAML::Node steps = root.IsMap() ? root["steps"] : YAML::Node();
    if (steps && steps.IsSequence()) {
      for (std::size_t i = 0; i < steps.size() && i < 255; ++i) {
        const YAML::Node sync = steps[i].IsMap() ? steps[i]["sync"] : YAML::Node();
        if (sync) {
          if (sync.IsScalar() && !sync.as<std::string>().empty()) {
            syncs[static_cast<std::uint8_t>(i)] = sync.as<std::string>();
          } else {
            out.warnings.push_back("steps[" + std::to_string(i) + "].sync 要是會合點的名稱（字串）");
          }
        }
        const YAML::Node wait = steps[i].IsMap() ? steps[i]["wait_for"] : YAML::Node();
        if (!wait) {
          continue;
        }
        const std::string at = "steps[" + std::to_string(i) + "].wait_for";
        if (!wait.IsSequence()) {
          out.warnings.push_back(at + " 要是清單");
          continue;
        }
        for (std::size_t k = 0; k < wait.size(); ++k) {
          const YAML::Node w = wait[k];
          int role = -1;
          std::string name;
          try {
            role = w["role_index"].as<int>();
            name = w["milestone"].as<std::string>("completed");
          } catch (const YAML::Exception &) {
            out.warnings.push_back(at + "[" + std::to_string(k) + "] 要有 role_index（整數）與 milestone");
            continue;
          }
          const auto m = cbba_core::milestoneFromName(name);
          if (role < 0 || role > 7 || !m) {
            out.warnings.push_back(at + "[" + std::to_string(k) + "]：role_index 要是 0～7、milestone 要認得（是 " +
              std::to_string(role) + "、" + name + "）");
            continue;
          }
          const auto r = static_cast<std::uint8_t>(role);
          if (r == self) {
            out.warnings.push_back(at + "：不能等自己（role_index " + std::to_string(role) + "）");
            continue;
          }
          if ((precedence & (1u << r)) == 0) {
            out.warnings.push_back(at + "：role " + std::to_string(role) + " 不在任務的 precedence（照 wait_for 等）");
          }
          by_step[static_cast<std::uint8_t>(i)].push_back(WaitFor{r, *m});
          mentioned.insert(r);
        }
      }
    }
  }

  // precedence 裡、沒有 wait_for 提到的角色：第一步等它 completed
  for (unsigned r = 0; r < 8; ++r) {
    if ((precedence & (1u << r)) != 0 && r != self && mentioned.count(static_cast<std::uint8_t>(r)) == 0) {
      by_step[0].push_back(WaitFor{static_cast<std::uint8_t>(r), Milestone::COMPLETED});
    }
  }
  for (const auto & [step, name] : syncs) {
    by_step[step];   // 只有會合點、沒有 wait_for 的步驟也列
  }
  for (auto & [step, waits] : by_step) {
    const auto s = syncs.find(step);
    out.steps.push_back(StepGateDef{step, std::move(waits), s != syncs.end() ? s->second : std::string()});
  }
  return out;
}

SyncResult evaluateSync(const std::vector<std::pair<std::uint8_t, SyncMember>> & others,
  std::optional<double> first_arrival, double now, double window)
{
  SyncResult r;
  for (const auto & [role, m] : others) {
    if (m == SyncMember::MISSING) {
      r.missing.push_back(role);
    }
  }
  if (r.missing.empty()) {
    r.state = cbba_core::GateState::OPEN;
  } else if (first_arrival && window > 0.0 && now - *first_arrival >= window) {
    r.state = cbba_core::GateState::OPEN;   // 時間窗過了：已到的先開始
    r.timed_out = true;
  }
  return r;
}

}  // namespace swarm_cbba
