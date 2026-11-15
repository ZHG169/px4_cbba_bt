#include "cbba_core/milestones.hpp"

namespace cbba_core
{

const char * toString(Milestone m)
{
  switch (m) {
    case Milestone::ACCEPTED: return "accepted";
    case Milestone::EN_ROUTE: return "en_route";
    case Milestone::ARRIVED: return "arrived";
    case Milestone::STARTED: return "started";
    case Milestone::TARGET_LOCALIZED: return "target_localized";
    case Milestone::TARGET_LOCKED: return "target_locked";
    case Milestone::HAS_OBSERVATION: return "has_observation";
    case Milestone::COMPLETED: return "completed";
  }
  return "?";
}

std::optional<Milestone> milestoneFromName(const std::string & name)
{
  for (unsigned i = 0; i < kMilestoneCount; ++i) {
    const auto m = static_cast<Milestone>(i);
    if (name == toString(m)) {
      return m;
    }
  }
  return std::nullopt;
}

std::vector<std::string> milestoneNames(MilestoneMask mask)
{
  std::vector<std::string> out;
  for (unsigned i = 0; i < 16; ++i) {
    if ((mask & (1u << i)) == 0) {
      continue;
    }
    out.push_back(i < kMilestoneCount ? toString(static_cast<Milestone>(i)) : "bit" + std::to_string(i));
  }
  return out;
}

MilestoneMask parseMilestones(const std::vector<std::string> & names, std::vector<std::string> * unknown)
{
  MilestoneMask mask = 0;
  for (const std::string & n : names) {
    if (const auto m = milestoneFromName(n)) {
      mask = static_cast<MilestoneMask>(mask | milestoneBit(*m));
    } else if (unknown != nullptr) {
      unknown->push_back(n);
    }
  }
  return mask;
}

const char * toString(ProgressPhase p)
{
  switch (p) {
    case ProgressPhase::UNKNOWN: return "UNKNOWN";
    case ProgressPhase::ACCEPTED: return "ACCEPTED";
    case ProgressPhase::EN_ROUTE: return "EN_ROUTE";
    case ProgressPhase::ARRIVED: return "ARRIVED";
    case ProgressPhase::EXECUTING: return "EXECUTING";
    case ProgressPhase::BLOCKED: return "BLOCKED";
    case ProgressPhase::ARRIVED_WAITING: return "ARRIVED_WAITING";
    case ProgressPhase::DONE: return "DONE";
  }
  return "?";
}

}  // namespace cbba_core
