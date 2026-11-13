#include "cbba_core/capabilities.hpp"

#include <array>
#include <utility>

namespace cbba_core
{

namespace
{
constexpr std::array<std::pair<Capability, const char *>, 13> kNames{{
  {Capability::MOBILITY_GROUND, "mobility_ground"},
  {Capability::MOBILITY_AIR, "mobility_air"},
  {Capability::INDOOR_FLIGHT, "indoor_flight"},
  {Capability::STAIRS, "stairs"},
  {Capability::RGB_CAMERA, "rgb_camera"},
  {Capability::ZOOM, "zoom"},
  {Capability::THERMAL_CAMERA, "thermal_camera"},
  {Capability::SPEAKER, "speaker"},
  {Capability::SPOTLIGHT, "spotlight"},
  {Capability::GAS_SENSOR, "gas_sensor"},
  {Capability::MANIPULATOR, "manipulator"},
  {Capability::GRIPPER, "gripper"},
  {Capability::COMM_RELAY, "comm_relay"},
}};
}  // namespace

std::optional<Capability> capabilityFromName(const std::string & name)
{
  for (const auto & [cap, n] : kNames) {
    if (name == n) {
      return cap;
    }
  }
  return std::nullopt;
}

const char * capabilityName(Capability c)
{
  for (const auto & [cap, n] : kNames) {
    if (cap == c) {
      return n;
    }
  }
  return "?";
}

std::optional<Capabilities> parseCapabilities(const std::vector<std::string> & names, std::string * bad)
{
  Capabilities caps = 0;
  for (const std::string & name : names) {
    const auto cap = capabilityFromName(name);
    if (!cap) {
      if (bad) {
        *bad = name;
      }
      return std::nullopt;
    }
    caps |= capBit(*cap);
  }
  return caps;
}

std::vector<std::string> capabilityNames(Capabilities caps)
{
  std::vector<std::string> out;
  for (unsigned bit = 0; bit < 64; ++bit) {
    if ((caps & (Capabilities{1} << bit)) == 0) {
      continue;
    }
    out.push_back(bit < kNames.size() ? capabilityName(static_cast<Capability>(bit)) :
      "bit" + std::to_string(bit));
  }
  return out;
}

std::string capabilitiesToString(Capabilities caps)
{
  if (caps == 0) {
    return "none";
  }
  std::string out;
  for (const std::string & name : capabilityNames(caps)) {
    if (!out.empty()) {
      out += ',';
    }
    out += name;
  }
  return out;
}

const char * platformName(std::uint8_t platform)
{
  switch (platform) {
    case kPlatformAny: return "any";
    case 1: return "uav";
    case 2: return "quadruped";
    default: return "?";
  }
}

std::uint8_t effectivePlatform(const Task & task)
{
  switch (task.type) {
    case TaskType::NONE: return task.platform;
    case TaskType::AIR_RECON: return platformOf(AgentType::UAV);
    case TaskType::GROUND_INTERVENTION:
    case TaskType::PATROL: return platformOf(AgentType::UGV);
  }
  return task.platform;
}

Capabilities effectiveRequiredCaps(const Task & task)
{
  switch (task.type) {
    case TaskType::NONE: return task.required_caps;
    case TaskType::AIR_RECON: return capBit(Capability::MOBILITY_AIR) | capBit(Capability::RGB_CAMERA);
    case TaskType::GROUND_INTERVENTION:
    case TaskType::PATROL: return capBit(Capability::MOBILITY_GROUND);
  }
  return task.required_caps;
}

Capabilities defaultCaps(AgentType type)
{
  switch (type) {
    case AgentType::UAV: return capBit(Capability::MOBILITY_AIR) | capBit(Capability::RGB_CAMERA);
    case AgentType::UGV: return capBit(Capability::MOBILITY_GROUND) | capBit(Capability::RGB_CAMERA);
  }
  return 0;
}

Capabilities availableCaps(const AgentState & agent)
{
  const Capabilities declared = agent.capabilities != 0 ? agent.capabilities : defaultCaps(agent.type);
  return declared & ~agent.failed_caps;
}

Fitness fitness(const AgentState & agent, const Task & task)
{
  const std::uint8_t platform = effectivePlatform(task);
  if (platform != kPlatformAny && platform != platformOf(agent.type)) {
    return Fitness::PLATFORM;
  }
  const Capabilities need = effectiveRequiredCaps(task);
  if ((need & ~availableCaps(agent)) != 0) {
    return Fitness::CAPABILITY;
  }
  return Fitness::OK;
}

}  // namespace cbba_core
