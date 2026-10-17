// 機器狗端（伯宇）task_manager 骨架的通用部分，給無人機用（2026-10-12）
//
// 來源：docs/handoff_uav_2026-10-12/bt/task_manager/include/task_manager/bt_nodes.hpp（bt_ws 2026-10-12 工作樹）。
// 原樣切出，只改了這些（伯宇更新骨架時照這份清單重切）：
//   - 拿掉狗的葉節點 GoTo（Nav2 NavigateToPose）、PtzInspect（雲台 action）、fillPtzGoal 和它們的 include、註冊函式：
//     我們的映像沒有 nav2_msgs、task_manager_interfaces；無人機的葉節點在 skeleton/uav_leaves.hpp
//   - printReport 的結束原因（2026-10-17）：原本沒列出的原因一律印「逾時中止」；改成 MISSION_TIMEOUT 才印「逾時中止」，
//     其他印「中止（原因）」（無人機的 PREEMPTED_BY_MANUAL、PX4_FAILSAFE、BATTERY_RTL、NOT_AIRBORNE 會被當成逾時）
//   - 這段註解
// 其餘（Task、Waypoint、Ctx、哨兵 IsTaskValidAndUnchanged、IsEmergency、HandleWaypointFailure、LoadTasks、SplitTask、
// HandleTaskFailure、ReportTaskResult、MarkMissionAborted、UnknownTask、printReport 的其他部分、tickOrAbort、registerAll）一字未改。
#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace task_manager {

struct Task {
  std::string action;
  std::map<std::string, std::string> params;

  std::string get(const std::string& k, const std::string& def = "") const {
    auto it = params.find(k);
    return it == params.end() ? def : it->second;
  }
};

struct Waypoint {
  std::string id;
  double x = 0, y = 0, yaw = 0;
  std::string on_nav_fail;          // per-waypoint override of policy.on_nav_fail ("" = use global)
  std::vector<Task> tasks;
};

struct Ctx {
  int  tick      = 0;
  bool emergency = false;
  std::string frame_id = "map";

  std::string on_nav_fail  = "skip_waypoint";
  std::string on_task_fail = "skip_task";
  double mission_timeout   = 1800.0;

  int  force_nav_fail = 0;        // read only by the mocks in test/mock_goto.hpp
  std::string force_task_fail;

  std::string current_waypoint;
  std::string last_error;
  bool        aborted = false;
  bool        abort_requested = false;
  std::string abort_waypoint;
  std::string end_reason;
  bool   manual_active = false;                             // tablet override in force (spec 2.3 rule 2); cleared by ReportTaskResult / abortNow
  size_t wp_results_begin = 0;                              // results.size() when LoadTasks ran; ReportTaskResult judges only this waypoint's attempt
  std::function<void(uint32_t task_id, bool success, const std::string& detail)> task_result_pub;   // the node wires /<ns>/task_result here
  std::optional<std::chrono::steady_clock::time_point> preempt_t0;   // spec §3.7 t0: the callback wrote {current_task_id}; consumed by the next halt


  struct Record { std::string waypoint, action, outcome, detail; };
  std::vector<Record> results;
  std::vector<std::string> images;   // "<waypoint>  <path>", one per photo

  void reset() {           // emergency is owned by /emergency_stop, not cleared here
    tick = 0;
    current_waypoint.clear();
    last_error.clear();
    aborted = false;
    abort_requested = false;
    abort_waypoint.clear();
    end_reason.clear();
    results.clear();
    images.clear();
  }
};
inline Ctx g_ctx;

// External (CBBA / tablet) waypoints are named "<prefix><task_id>"; everything else came from the yaml
inline bool hasPrefix(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }
inline bool isExternal(const std::string& id) { return hasPrefix(id, "cbba_") || hasPrefix(id, "manual_"); }
inline uint32_t externalTaskId(const std::string& id) {
  try { return static_cast<uint32_t>(std::stoul(id.substr(id.find('_') + 1))); } catch (...) { return 0; }
}

// spec §3.7 t1: the halt that sends the cancel. Prints the latency once per external dispatch
inline void preemptLatencyNote(const std::string& who) {
  if (!g_ctx.preempt_t0) return;
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - *g_ctx.preempt_t0).count();
  g_ctx.preempt_t0.reset();
  RCLCPP_INFO(rclcpp::get_logger("task_manager"), "      │ %-14s │ ⏱ 中斷延遲 %.1f ms（callback 寫 id → 送 cancel）", who.c_str(), ms);
}

inline void say(const std::string& who, const std::string& msg) {
  std::ostringstream out;
  out << "  " << std::setw(3) << g_ctx.tick << " │ "
      << std::setw(14) << std::left << who << " │ " << msg;
  RCLCPP_INFO(rclcpp::get_logger("task_manager"), "%s", out.str().c_str());
}

// Sentinel (spec 3.1): a new {current_task_id} fails once so the ReactiveSequence halts what runs below, and is copied into {active_task_id}; the next tick passes.
class IsTaskValidAndUnchanged : public BT::ConditionNode {
public:
  IsTaskValidAndUnchanged(const std::string& n, const BT::NodeConfig& c) : ConditionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<uint32_t>("current"), BT::BidirectionalPort<uint32_t>("active") };
  }
  BT::NodeStatus tick() override {
    const uint32_t cur = getInput<uint32_t>("current").value_or(0);   // no key yet = no external task (tree_step / tree_dump set none)
    const uint32_t act = getInput<uint32_t>("active").value_or(0);
    if (cur == act) return BT::NodeStatus::SUCCESS;
    setOutput("active", cur);
    say("TaskSentinel", "★ 任務換了 " + std::to_string(act) + " → " + std::to_string(cur) + "，halt 手上的點");
    return BT::NodeStatus::FAILURE;
  }
};

class IsEmergency : public BT::ConditionNode {
public:
  IsEmergency(const std::string& n, const BT::NodeConfig& c) : ConditionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    if (g_ctx.emergency) say("IsEmergency", "★★ 緊急停止觸發");
    return g_ctx.emergency ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class HandleWaypointFailure : public BT::SyncActionNode {
public:
  HandleWaypointFailure(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<Waypoint>("waypoint"),
             BT::OutputPort<uint32_t>("current"), BT::OutputPort<uint32_t>("active") };   // cleared when an external point fails
  }
  BT::NodeStatus tick() override {
    auto in = getInput<Waypoint>("waypoint");
    if (!in) return BT::NodeStatus::FAILURE;
    auto wp = in.value();

    // task-level abort: pass through, not a nav failure
    if (g_ctx.abort_requested) return BT::NodeStatus::FAILURE;

    // only NAV_FAILED* is a nav failure; anything else (NAV_NO_ACK, bad definition) aborts the mission
    if (g_ctx.last_error.rfind("NAV_FAILED", 0) != 0) {
      const std::string why = g_ctx.last_error.empty() ? "INTERNAL_ERROR" : g_ctx.last_error;
      g_ctx.results.push_back({wp.id, "waypoint", "FAILED", why});
      say("HandleWpFail", "✗ " + wp.id + " 內部錯誤（非導航失敗）→ 中止整個任務");
      closeExternal(wp.id, why);   // hand the point back with the real reason now; abortNow finds nothing left to report
      g_ctx.last_error.clear();
      return BT::NodeStatus::FAILURE;
    }

    g_ctx.results.push_back({wp.id, "goto", "FAILED", g_ctx.last_error});
    closeExternal(wp.id, g_ctx.last_error);
    g_ctx.last_error.clear();

    // per-waypoint override beats the global policy
    const std::string& own   = wp.on_nav_fail;
    const std::string policy = own.empty() ? g_ctx.on_nav_fail : own;
    const std::string tag    = own.empty() ? "" : "（點自標）";
    if (policy == "abort") {
      g_ctx.abort_waypoint = wp.id;
      say("HandleWpFail", "✗ 去不了 " + wp.id + "，policy=abort" + tag + " → 中止整個任務");
      return BT::NodeStatus::FAILURE;
    }
    say("HandleWpFail", "⚠ 去不了 " + wp.id + "，policy=skip_waypoint" + tag + " → 記錄後跳過");
    return BT::NodeStatus::SUCCESS;   // skip_waypoint
  }

private:
  // An external point that cannot be reached is over: cbba goes back to the auction, manual drops the override, both clear the ids (spec 3.2)
  void closeExternal(const std::string& id, const std::string& why) {
    if (!isExternal(id)) return;
    if (hasPrefix(id, "cbba_")) {
      if (g_ctx.task_result_pub) g_ctx.task_result_pub(externalTaskId(id), false, why);
      say("HandleWpFail", "→ task_result  task " + std::to_string(externalTaskId(id)) + " 交還 CBBA：" + why);
    } else {
      g_ctx.manual_active = false;
    }
    setOutput("current", uint32_t{0});
    setOutput("active",  uint32_t{0});
  }
};

class LoadTasks : public BT::SyncActionNode {
public:
  LoadTasks(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<Waypoint>("waypoint"),
             BT::OutputPort<BT::SharedQueue<Task>>("tasks") };
  }
  BT::NodeStatus tick() override {
    auto in = getInput<Waypoint>("waypoint");
    if (!in) return BT::NodeStatus::FAILURE;
    auto wp = in.value();
    g_ctx.current_waypoint = wp.id;
    g_ctx.wp_results_begin = g_ctx.results.size();   // ReportTaskResult judges only what this attempt records from here on
    setOutput("tasks", std::make_shared<std::deque<Task>>(wp.tasks.begin(), wp.tasks.end()));
    say("LoadTasks", wp.id + " 有 " + std::to_string(wp.tasks.size()) + " 個任務");
    return BT::NodeStatus::SUCCESS;
  }
};

class SplitTask : public BT::SyncActionNode {
public:
  SplitTask(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<Task>("task"), BT::OutputPort<std::string>("action") };
  }
  BT::NodeStatus tick() override {
    auto in = getInput<Task>("task");
    if (!in || in.value().action.empty()) {
      g_ctx.last_error = "BAD_TASK_DEF";
      say("SplitTask", "✗ 這筆任務沒有 action");
      return BT::NodeStatus::FAILURE;
    }
    setOutput("action", in.value().action);
    return BT::NodeStatus::SUCCESS;
  }
};

class HandleTaskFailure : public BT::SyncActionNode {
public:
  HandleTaskFailure(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<Waypoint>("waypoint"), BT::InputPort<Task>("task") };
  }
  BT::NodeStatus tick() override {
    auto wi = getInput<Waypoint>("waypoint");
    auto ti = getInput<Task>("task");
    if (!wi || !ti) return BT::NodeStatus::FAILURE;
    auto wp = wi.value();
    auto t  = ti.value();

    std::string detail = g_ctx.last_error.empty() ? "UNKNOWN" : g_ctx.last_error;
    for (const auto& [k, v] : t.params)          // params go into the report
      if (k != "on_fail") detail += "  " + k + "=" + v;
    std::string action = t.action.empty() ? "(無action)" : t.action;
    g_ctx.results.push_back({wp.id, action, "FAILED", detail});
    g_ctx.last_error.clear();

    // per-task override beats the global policy
    const std::string own    = t.get("on_fail");
    const std::string policy = own.empty() ? g_ctx.on_task_fail : own;
    const std::string tag    = own.empty() ? "" : "（任務自標）";
    if (policy == "abort") {
      g_ctx.abort_requested = true;
      g_ctx.abort_waypoint = wp.id;
      say("HandleTaskFail", "✗ " + wp.id + " 的 " + action
                          + " 失敗，policy=abort" + tag + " → 中止整個任務");
      return BT::NodeStatus::FAILURE;
    }
    say("HandleTaskFail", "⚠ " + wp.id + " 的 " + action
                        + " 失敗，policy=skip_task" + tag + " → 記錄後繼續");
    return BT::NodeStatus::SUCCESS;
  }
};

// ReportTaskResult (spec 3.4.7): closes an external point. cbba_ points publish TaskResult; manual_ points just release the override.
class ReportTaskResult : public BT::SyncActionNode {
public:
  ReportTaskResult(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<Task>("task"),
             BT::BidirectionalPort<uint32_t>("current"), BT::BidirectionalPort<uint32_t>("active") };
  }
  BT::NodeStatus tick() override {
    auto in = getInput<Task>("task");
    if (!in) return BT::NodeStatus::FAILURE;
    const std::string wp = g_ctx.current_waypoint;
    uint32_t id = 0;
    try { id = static_cast<uint32_t>(std::stoul(in.value().get("task_id"))); } catch (...) {}

    // this attempt's records only (results are kept for the whole mission): one non-OK line fails the task
    bool ok = true; std::string detail;
    for (size_t i = g_ctx.wp_results_begin; i < g_ctx.results.size(); ++i) {
      const auto& r = g_ctx.results[i];
      if (r.waypoint == wp && r.outcome != "OK") { ok = false; detail = r.detail; break; }
    }
    if (hasPrefix(wp, "cbba_")) {
      if (g_ctx.task_result_pub) g_ctx.task_result_pub(id, ok, detail);
      say("ReportResult", "→ task_result  task " + std::to_string(id) + (ok ? "  success" : "  failed  " + detail));
    } else if (hasPrefix(wp, "manual_")) {
      g_ctx.manual_active = false;
      say("ReportResult", "平板任務 " + std::to_string(id) + (ok ? " 完成" : " 失敗  " + detail) + "，解除人工接管");
    }
    g_ctx.results.push_back({wp, "report_result", "OK", ok ? "success" : "failed: " + detail});
    // the point is closed: let the sentinel pass again. A newer id written meanwhile is left alone
    if (getInput<uint32_t>("current").value_or(0) == id) setOutput("current", uint32_t{0});
    if (getInput<uint32_t>("active").value_or(0)  == id) setOutput("active",  uint32_t{0});
    return BT::NodeStatus::SUCCESS;
  }
};

class MarkMissionAborted : public BT::SyncActionNode {
public:
  MarkMissionAborted(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    g_ctx.aborted = true;
    if (g_ctx.abort_waypoint.empty()) g_ctx.abort_waypoint = g_ctx.current_waypoint;
    say("MarkAborted", "■ 任務中止於 " + g_ctx.abort_waypoint);
    return BT::NodeStatus::SUCCESS;
  }
};

class UnknownTask : public BT::SyncActionNode {
public:
  UnknownTask(const std::string& n, const BT::NodeConfig& c) : SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("action") }; }
  BT::NodeStatus tick() override {
    auto a = getInput<std::string>("action");
    g_ctx.last_error = "UNKNOWN_ACTION";
    say("UnknownTask", "✗ mission.yaml 裡有不認識的動作："
                       + (a ? a.value() : std::string("(空)")));
    return BT::NodeStatus::FAILURE;
  }
};

inline void printReport() {
  int ok = 0, fail = 0;
  for (auto& r : g_ctx.results) (r.outcome == "OK" ? ok : fail)++;

  // one RCLCPP_INFO so the report is not interleaved with other log lines
  std::ostringstream out;
  out << "\n  ┌─ 巡檢報告 ─────────────────────────────────────\n";
  if (g_ctx.end_reason == "COMPLETED")
    out << "  │ 任務完成\n";
  else if (g_ctx.end_reason == "EMERGENCY")
    out << "  │ 急停中止　於 " << g_ctx.abort_waypoint << " 點\n";
  else if (g_ctx.end_reason == "POLICY_ABORT")
    out << "  │ 策略中止　於 " << g_ctx.abort_waypoint << " 點\n";
  else if (g_ctx.end_reason == "INTERNAL_ERROR")
    out << "  │ 程式錯誤中止　於 " << g_ctx.abort_waypoint << " 點：" << g_ctx.last_error << "\n";
  else if (g_ctx.end_reason == "OPERATOR_STOP")
    out << "  │ 操作員停止　於 " << g_ctx.abort_waypoint << " 點\n";
  else if (g_ctx.end_reason == "SHUTDOWN")
    out << "  │ 關機中止　於 " << g_ctx.abort_waypoint << " 點\n";
  else if (g_ctx.end_reason == "MISSION_TIMEOUT")
    out << "  │ 逾時中止　於 " << g_ctx.abort_waypoint << " 點\n";
  else
    out << "  │ 中止（" << g_ctx.end_reason << "）　於 " << g_ctx.abort_waypoint << " 點\n";
  out << "  ├────────────────────────────────────────────────\n";

  for (auto& r : g_ctx.results)
    out << "  │ " << std::setw(3) << std::left << r.waypoint
        << " " << std::setw(14) << r.action
        << " " << std::setw(7) << r.outcome << r.detail << "\n";

  if (!g_ctx.images.empty()) {
    out << "  ├─ 照片 ────────────────────────────────────────\n";
    for (auto& p : g_ctx.images) out << "  │ " << p << "\n";
  }
  out << "  ├────────────────────────────────────────────────\n"
      << "  │ 成功 " << ok << "  失敗 " << fail
      << "  共 " << g_ctx.tick << " ticks\n"
      << "  └────────────────────────────────────────────────";

  RCLCPP_INFO(rclcpp::get_logger("task_manager"), "%s", out.str().c_str());
}

// The only tick entry: a node exception halts the tree and ends the mission as INTERNAL_ERROR, the process lives
inline BT::NodeStatus tickOrAbort(BT::Tree& tree) {
  auto logger = rclcpp::get_logger("task_manager");
  try {
    return tree.tickOnce();
  } catch (const std::exception& e) {
    g_ctx.last_error = e.what();
  } catch (...) {
    g_ctx.last_error = "unknown exception";
  }
  g_ctx.end_reason = "INTERNAL_ERROR";
  RCLCPP_ERROR(logger, "樹丟例外：%s → halt 進行中的節點", g_ctx.last_error.c_str());
  try {
    tree.haltTree();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "halt 也丟例外：%s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger, "halt 也丟例外");
  }
  return BT::NodeStatus::FAILURE;
}

inline void registerAll(BT::BehaviorTreeFactory& f) {
  // GoTo / PtzInspect are registered by the node (real) or the tests (mock)
  f.registerNodeType<BT::SwitchNode<1>>("Switch1");   // BT.CPP ships Switch2..6 only
  f.registerNodeType<IsEmergency>("IsEmergency");
  f.registerNodeType<IsTaskValidAndUnchanged>("IsTaskValidAndUnchanged");
  f.registerNodeType<HandleWaypointFailure>("HandleWaypointFailure");
  f.registerNodeType<LoadTasks>("LoadTasks");
  f.registerNodeType<SplitTask>("SplitTask");
  f.registerNodeType<HandleTaskFailure>("HandleTaskFailure");
  f.registerNodeType<MarkMissionAborted>("MarkMissionAborted");
  f.registerNodeType<ReportTaskResult>("ReportTaskResult");
  f.registerNodeType<UnknownTask>("UnknownTask");
  f.registerNodeType<BT::LoopNode<Waypoint>>("LoopWaypoint");
  f.registerNodeType<BT::LoopNode<Task>>("LoopTask");
}

}  // namespace task_manager
