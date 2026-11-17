#include "cbba_core/zones.hpp"

#include <algorithm>

#include "cbba_core/capabilities.hpp"

namespace cbba_core
{

bool ZoneArea::contains(double x, double y) const
{
  // 射線法：點在邊上算在裡面（禁區寧可多擋）
  bool inside = false;
  const std::size_t n = polygon.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[i][0], yi = polygon[i][1];
    const double xj = polygon[j][0], yj = polygon[j][1];
    const double cross = (x - xi) * (yj - yi) - (y - yi) * (xj - xi);
    if (cross == 0.0 && x >= std::min(xi, xj) && x <= std::max(xi, xj) && y >= std::min(yi, yj) &&
      y <= std::max(yi, yj))
    {
      return true;
    }
    if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
      inside = !inside;
    }
  }
  return n >= 3 && inside;
}

const ZonePolicy * ZoneRegistry::policy(std::uint16_t id) const
{
  for (const ZonePolicy & p : policies) {
    if (p.id == id) {
      return &p;
    }
  }
  return nullptr;
}

const char * toString(ZoneVerdict v)
{
  switch (v) {
    case ZoneVerdict::OK: return "OK";
    case ZoneVerdict::WRONG_FLOOR: return "WRONG_FLOOR";
    case ZoneVerdict::ALTITUDE_LIMIT: return "ALTITUDE_LIMIT";
    case ZoneVerdict::ZONE_FORBIDDEN: return "ZONE_FORBIDDEN";
    case ZoneVerdict::INDOOR: return "INDOOR";
  }
  return "?";
}

ZoneVerdict checkZones(const ZoneRegistry * zones, const AgentState & agent, std::uint8_t platform,
  const Task & task, std::string * zone_id)
{
  const std::uint8_t floor = task.ext.floor;
  if (floor != 0 && agent.floor != 0 && floor != agent.floor) {
    return ZoneVerdict::WRONG_FLOOR;
  }
  const Vec3 & p = task.position;
  if (agent.max_altitude > 0.0 && p.z > agent.max_altitude + 1e-6) {
    return ZoneVerdict::ALTITUDE_LIMIT;
  }
  if (zones == nullptr) {
    return ZoneVerdict::OK;
  }
  const std::uint8_t bit = platform < 8 ? static_cast<std::uint8_t>(1u << platform) : 0;
  const ZonePolicy * policy = task.ext.zone_policy_id != 0 ? zones->policy(task.ext.zone_policy_id) : nullptr;
  const bool indoor_ok = platform != platformOf(AgentType::UAV) ||
    (availableCaps(agent) & capBit(Capability::INDOOR_FLIGHT)) != 0;
  for (std::size_t i = 0; i < zones->zones.size(); ++i) {
    const ZoneArea & z = zones->zones[i];
    if ((z.floor != 0 && floor != 0 && z.floor != floor) || !z.contains(p.x, p.y)) {
      continue;
    }
    const auto blocked = [&](ZoneVerdict v) {
        if (zone_id != nullptr) {
          *zone_id = z.id;
        }
        return v;
      };
    if (z.ceiling > 0.0 && p.z > z.ceiling + 1e-6) {
      return blocked(ZoneVerdict::ALTITUDE_LIMIT);
    }
    if ((z.no_entry & bit) != 0) {
      return blocked(ZoneVerdict::ZONE_FORBIDDEN);
    }
    if (policy != nullptr) {
      for (const ZonePolicy::Rule & r : policy->forbid) {
        if (r.zone == i && (r.platforms & bit) != 0) {
          return blocked(ZoneVerdict::ZONE_FORBIDDEN);
        }
      }
    }
    if (z.indoor && !indoor_ok) {
      return blocked(ZoneVerdict::INDOOR);
    }
  }
  return ZoneVerdict::OK;
}

}  // namespace cbba_core
