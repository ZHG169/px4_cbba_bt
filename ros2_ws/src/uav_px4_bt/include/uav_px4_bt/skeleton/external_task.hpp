// 機器狗端（伯宇）task_manager 骨架的外部任務佇列規則，給無人機用（2026-10-12）
// 來源：docs/handoff_uav_2026-10-12/bt/task_manager/include/task_manager/external_task.hpp。
// 只改了這些，其餘一字未改：
//   - makeExternalWaypoint（狗：站在目標前 1.5 m、雲台；無人機：目標正上方、air_recon）
//   - failureReason 加無人機的停止原因 BATTERY_RTL（低電量返航）、NOT_AIRBORNE（降落、上鎖）→ STOPPED（UAV_BT_GUIDE §2.1）
//     、PX4_FAILSAFE（PX4 自己的失效保護，2026-10-17）。被人接管照狗用 PREEMPTED_BY_MANUAL（原本就是 PREEMPTED）
#pragma once
// Queue rules for external (CBBA / manual) waypoints. Pure: no ROS, no blackboard. Spec 3.4.3-3.4.6.
#include "uav_px4_bt/skeleton/bt_nodes.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <utility>

namespace task_manager {

// 無人機版（伯宇 UAV_BT_GUIDE §4.1）：狗站在目標前 1.5 m 面向它、雲台盯目標；無人機飛到目標正上方
// z = max(altitude, 目標 z)，tasks = [air_recon（target、duration_sec）, report_result（task_id）]。id 一樣是 <prefix><task_id>
inline Waypoint makeExternalWaypoint(const char* prefix, uint32_t task_id, double tx, double ty, double tz,
                                     double altitude, double duration_sec) {
  Waypoint w;
  w.id = prefix + std::to_string(task_id);
  w.x = tx; w.y = ty; w.yaw = 0;
  w.on_nav_fail = "skip_waypoint";
  Task recon; recon.action = "air_recon";
  recon.params = {{"target.x", std::to_string(tx)}, {"target.y", std::to_string(ty)},
                  {"target.z", std::to_string(std::max(altitude, tz))}, {"duration_sec", std::to_string(duration_sec)}};
  Task rep; rep.action = "report_result";
  rep.params = {{"task_id", std::to_string(task_id)}};
  w.tasks = {recon, rep};
  return w;
}

// Id of the unreported external point with this prefix in hand: the one being worked on (`active`) or the queue head. "" if none.
// current_task_id == 0 means ReportTaskResult / HandleWaypointFailure already reported it: the point is only finishing (LoopTask needs
// one more tick), so it is nobody's to hand back, cancel or resend.
inline std::string externalInHand(const std::deque<Waypoint>& q, const Waypoint* active, const char* prefix, uint32_t current_task_id) {
  if (current_task_id == 0) return "";
  if (active && hasPrefix(active->id, prefix)) return active->id;
  if (!q.empty() && hasPrefix(q.front().id, prefix)) return q.front().id;
  return "";
}

// Put `ext` at the head: the sentinel halts what runs now, LoopWaypoint pops `ext` next. Invariant: at most one external point, always at the head.
inline void insertExternal(std::deque<Waypoint>& q, const Waypoint* active, const Waypoint& ext,
                           uint32_t task_id, uint32_t& current_task_id) {
  const bool head_external = !q.empty() && isExternal(q.front().id);
  // the interrupted patrol point resumes after the external one; skipped when an earlier insert already put it back (head still external: the sentinel has not halted yet)
  if (active && !isExternal(active->id) && !head_external) q.push_front(*active);
  if (head_external) q.pop_front();   // the old external point was handed back or replaced by the caller
  q.push_front(ext);
  current_task_id = task_id;
}

// Cancel the external point with this prefix. Running: current_task_id = 0 so the sentinel halts it. Queued: drop it. False if none in hand.
inline bool cancelExternal(std::deque<Waypoint>& q, const Waypoint* active, const char* prefix, uint32_t& current_task_id) {
  if (current_task_id == 0) return false;   // already reported (see externalInHand): a late cancel changes nothing
  if (active && hasPrefix(active->id, prefix)) { current_task_id = 0; return true; }
  if (!q.empty() && hasPrefix(q.front().id, prefix)) { q.pop_front(); current_task_id = 0; return true; }   // id cleared too, or a later re-dispatch of the same id reads as a resend
  return false;
}

// Reload the route into the live queue for a (re)start. An external point already at the head arrived between the last point and this
// restart: it stays, or it would be dropped with no result and CBBA would wait on this robot forever. Other leftovers are not kept.
inline void resetRoute(std::deque<Waypoint>& q, const std::deque<Waypoint>& route) {
  std::optional<Waypoint> ext;
  if (!q.empty() && isExternal(q.front().id)) ext = q.front();
  q = route;
  if (ext) q.push_front(*ext);
}

// The point in hand, read from the tree itself: {wp} while LoopWaypoint's child is RUNNING. A finished point leaves {wp} behind
// until the next tick pops the next one; the child is IDLE then, so the leftover is never mistaken for a running point.
inline std::optional<Waypoint> waypointInHand(BT::Tree& tree) {
  const BT::TreeNode* visit = nullptr;
  BT::applyRecursiveVisitor(tree.rootNode(), [&](BT::TreeNode* n) {
    if (!visit && n->registrationName() == "LoopWaypoint")
      if (auto* d = dynamic_cast<BT::DecoratorNode*>(n)) visit = d->child();
  });
  if (!visit || visit->status() != BT::NodeStatus::RUNNING) return std::nullopt;
  try { return tree.rootBlackboard()->get<Waypoint>("wp"); } catch (...) { return std::nullopt; }
}

// The tablet sent the task already in hand again (lossy link): same prefix and id, nothing to change
inline bool isResend(const std::deque<Waypoint>& q, const Waypoint* active, const char* prefix, uint32_t task_id, uint32_t current_task_id) {
  return externalInHand(q, active, prefix, current_task_id) == prefix + std::to_string(task_id);
}

// Failure class for TaskResult.reason (swarm_interfaces v1.2, spec 2.3 rule 3); values mirror the msg constants
namespace reason { constexpr uint8_t NONE = 0, PERMANENT = 1, TRANSIENT = 2, PREEMPTED = 3, STOPPED = 4; }
inline uint8_t failureReason(bool success, const std::string& detail) {
  if (success) return reason::NONE;
  auto starts = [&](const char* p) { return detail.rfind(p, 0) == 0; };
  if (starts("NAV_FAILED(204)") || starts("NAV_FAILED(208)") || starts("PTZ_OUT_OF_RANGE")) return reason::PERMANENT;
  if (starts("PREEMPTED_BY_MANUAL")) return reason::PREEMPTED;
  for (const char* s : {"OPERATOR_STOP", "EMERGENCY", "MISSION_TIMEOUT", "SHUTDOWN", "POLICY_ABORT", "INTERNAL_ERROR",
                        "BATTERY_RTL", "NOT_AIRBORNE", "PX4_FAILSAFE"})
    if (starts(s)) return reason::STOPPED;
  return reason::TRANSIENT;   // stuck, timeouts, no ack, anything unknown: penalise, keep it in the auction
}

}  // namespace task_manager
