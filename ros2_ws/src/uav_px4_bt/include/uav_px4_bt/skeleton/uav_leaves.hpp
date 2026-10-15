// 無人機的葉節點，掛在機器狗端骨架（skeleton/bt_nodes.hpp）上（2026-10-12，照伯宇 UAV_BT_GUIDE v2 §2）
//
//   NavigateTo  取代狗的 GoTo（Nav2）：飛到 {wp} 上空（高度 altitude）。移動經過 UavContext::moveTo（CBF 避碰、運動平滑層、
//               位置＋速度前饋給 PX4）。飛行途中相機持續偵測：看到不是目的地的新火點（自己和別台都沒報過，別台的從
//               cbba_node 的 task_announced 來，2026-10-14）→ 發布地面處置任務（給 V60 競標）→
//               轉向飛到火點上空、懸停監控到那個地面任務結束（task_closed）或 monitor_timeout → 回到原本的航線繼續飛
//               （週計畫：看到火點要到上空懸停等待；2026-10-14 起，原本只發布、繼續飛）。監控中不檢查飛行逾時，
//               回到航線時重新計時。到點 → results 記 {wp, "goto", "OK"} → SUCCESS。
//               失敗的 last_error（HandleWaypointFailure 靠前綴決定）：飛不到（逾時）NAV_FAILED(TIMEOUT) → 這點跳過
//               （fly_timeout_abort = false 時逾時只印警告、繼續飛）；
//               沒有位置、掉出 offboard NAV_NO_ACK → 整趟中止。halt（哨兵換任務）：印中斷延遲，交給下一個動作
//   AirRecon    取代狗的 PtzInspect（Switch2 的 case air_recon）：在 target 上空懸停 duration_sec，懸停穩定 settle_sec 後
//               認 tag 101（confidence ≥ 0.85）；認到就發布地面處置任務、移到火點上空（2026-10-14），懸停時間到了
//               繼續懸停監控，等那個地面任務結束
//               （cbba_node 的 task_closed，狗處理完）或 monitor_timeout 才完成（週計畫，2026-10-13）。結束時 results 記
//               {wp, "air_recon", "OK"}，骨架的 ReportTaskResult 據此發 task_result
//
// 樹外（node 層，uav_task_manager_node.cpp）：起飛、待命時盤旋與讓路、低電量返航、送 setpoint、位置廣播。
// 共用的 UavContext（blackboard "uav_ctx"）和備用 BT 相同：CBF、運動平滑、火點去重複。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

#include <behaviortree_cpp/bt_factory.h>

#include "uav_px4_bt/bt_nodes.hpp"
#include "uav_px4_bt/skeleton/bt_nodes.hpp"

namespace uav_px4_bt
{

inline UavContext & uavCtx(const BT::TreeNode & node)
{
  return *node.config().blackboard->get<UavContext *>("uav_ctx");
}

inline std::string fmt1(double v)
{
  char b[32];
  std::snprintf(b, sizeof(b), "%.1f", v);
  return b;
}

class NavigateTo : public BT::StatefulActionNode
{
public:
  NavigateTo(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<task_manager::Waypoint>("waypoint"),
      BT::InputPort<double>("altitude", 5.0, "飛行高度（公尺）"),
      BT::InputPort<unsigned>("watch_fire_tag", 101u, "飛行途中看這個火警 tag（0 = 不看）"),
      BT::InputPort<double>("min_confidence", 0.85, ""),
      BT::InputPort<int>("min_frames", 5, ""),
      BT::InputPort<double>("max_age", 0.5, ""),
      BT::InputPort<bool>("divert", true, "路過發現火點時轉向火點上空懸停監控（false = 只發布、繼續飛）"),
      BT::InputPort<double>("monitor_timeout", 120.0,
        "轉向火點後懸停監控、等地面處置任務結束最多幾秒（0 = 不等，到了上空就回航線）"),
    };
  }

  BT::NodeStatus onStart() override
  {
    using task_manager::g_ctx;
    auto in = getInput<task_manager::Waypoint>("waypoint");
    if (!in) {
      g_ctx.last_error = "BAD_WAYPOINT_DEF";
      return BT::NodeStatus::FAILURE;
    }
    wp_ = in.value();
    g_ctx.current_waypoint = wp_.id;
    UavContext & ctx = uavCtx(*this);
    const VehicleState v = ctx.platform.vehicle();
    if (!v.valid || !v.armed || !v.offboard) {
      g_ctx.last_error = "NAV_NO_ACK";
      task_manager::say("NavigateTo", "✗ 沒有位置、沒解鎖或不在 offboard");
      return BT::NodeStatus::FAILURE;
    }
    goal_ = {wp_.x, wp_.y, getInput<double>("altitude").value_or(5.0)};
    ctx.waypoint = goal_;
    ctx.arrival = -1.0;
    const double d = norm(goal_ - v.position);
    start_ = ctx.platform.timeNow();
    deadline_ = start_ + d / std::max(ctx.config.cbf.max_speed, 0.1) * 2.0 + ctx.config.fly_timeout_margin;
    phase_ = Phase::ROUTE;
    task_manager::say("NavigateTo", "→ 去 " + wp_.id + "  (" + fmt1(goal_.x) + ", " + fmt1(goal_.y) + ", " +
      fmt1(goal_.z) + ")  " + fmt1(d) + " m");
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    using task_manager::g_ctx;
    UavContext & ctx = uavCtx(*this);
    const VehicleState v = ctx.platform.vehicle();
    if (!v.valid || !v.offboard) {
      g_ctx.last_error = "NAV_NO_ACK";
      task_manager::say("NavigateTo", "✗ 位置不見了或掉出 offboard");
      return BT::NodeStatus::FAILURE;
    }
    const double now = ctx.platform.timeNow();
    if (phase_ != Phase::ROUTE) {
      divertStep(ctx, now);
      return BT::NodeStatus::RUNNING;
    }
    const double d = ctx.moveTo(goal_);
    ctx.setExec(ExecMode::NAVIGATING, d / std::max(ctx.config.cbf.max_speed, 0.1), nullptr);
    if (watchFire(ctx, now)) {
      return BT::NodeStatus::RUNNING;
    }
    if (d < ctx.config.reach_tolerance) {
      ctx.arrival = now;
      g_ctx.results.push_back({wp_.id, "goto", "OK", ""});
      task_manager::say("NavigateTo", "✓ 抵達 " + wp_.id);
      return BT::NodeStatus::SUCCESS;
    }
    if (now > deadline_) {
      if (ctx.config.fly_timeout_abort) {
        g_ctx.last_error = "NAV_FAILED(TIMEOUT)";
        task_manager::say("NavigateTo", "✗ 飛不到 " + wp_.id + "（剩 " + fmt1(d) + " m）");
        return BT::NodeStatus::FAILURE;
      }
      // 調 CBF 時（fly_timeout_abort = false）：只印警告、繼續飛，每 kSlowWarnPeriod 秒印一次
      task_manager::say("NavigateTo", "⚠ 飛太久：去 " + wp_.id + " 已 " + fmt1(now - start_) + " s（逾時 " +
        fmt1(deadline_ - start_) + " s），剩 " + fmt1(d) + " m，繼續飛");
      deadline_ = now + kSlowWarnPeriod;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    task_manager::say("NavigateTo", "★ halt（換任務）→ 停在原地，交給下一個動作");
    task_manager::preemptLatencyNote("NavigateTo");
  }

private:
  enum class Phase {ROUTE, TO_FIRE, MONITOR};

  // 路過發現：不是這次目的地的新火點 → 發布地面處置任務（給 V60）；divert 時轉向火點上空（回傳 true）。
  // 目的地就是火點時到了由 AirRecon 判定
  bool watchFire(UavContext & ctx, double now)
  {
    const unsigned tag = getInput<unsigned>("watch_fire_tag").value_or(101u);
    if (tag == 0) {
      return false;
    }
    const auto f = ctx.newFire(tag, getInput<double>("min_confidence").value_or(0.85),
        getInput<int>("min_frames").value_or(5), getInput<double>("max_age").value_or(0.5));
    if (!f || std::hypot(f->position.x - goal_.x, f->position.y - goal_.y) <= ctx.config.fire_dedup_radius) {
      return false;
    }
    task_manager::say("NavigateTo", "🔥 飛行途中發現火警 tag " + std::to_string(tag) + "（confidence " +
      fmt1(f->confidence) + "）在 (" + fmt1(f->position.x) + ", " + fmt1(f->position.y) + ")");
    fire_task_ = ctx.reportFire(f->position);
    if (!getInput<bool>("divert").value_or(true)) {
      return false;
    }
    fire_goal_ = {f->position.x, f->position.y, goal_.z};
    phase_ = Phase::TO_FIRE;
    monitor_start_ = -1.0;
    task_manager::say("NavigateTo", "↪ 轉向火點上空 (" + fmt1(fire_goal_.x) + ", " + fmt1(fire_goal_.y) +
      ")，懸停監控後再回去 " + wp_.id);
    divertStep(ctx, now);
    return true;
  }

  // 轉向火點：飛到上空 → 懸停監控到地面任務結束或 monitor_timeout → 回到原本的航線（重新計時飛行逾時）
  void divertStep(UavContext & ctx, double now)
  {
    const double timeout = getInput<double>("monitor_timeout").value_or(120.0);
    const double d = ctx.moveTo(fire_goal_);
    if (phase_ == Phase::TO_FIRE) {
      ctx.setExec(ExecMode::EXECUTING, d / std::max(ctx.config.cbf.max_speed, 0.1) + std::max(timeout, 0.0),
        nullptr);
      if (d >= ctx.config.reach_tolerance) {
        return;
      }
      phase_ = Phase::MONITOR;
      monitor_start_ = now;
      task_manager::say("NavigateTo", "👁 在火點上空懸停監控，等地面處置任務 " + taskName(fire_task_) +
        " 結束（最多 " + fmt1(timeout) + " s）");
    }
    const double waited = now - monitor_start_;
    ctx.setExec(ExecMode::EXECUTING, std::max(timeout - waited, 0.0), nullptr);
    std::string why;
    if (ctx.platform.taskClosed(fire_task_)) {
      why = "地面處置任務 " + taskName(fire_task_) + " 結束（" + fmt1(waited) + " s）";
    } else if (waited >= timeout) {
      why = "等了 " + fmt1(timeout) + " s，地面處置任務 " + taskName(fire_task_) + " 還沒結束";
    } else {
      return;
    }
    phase_ = Phase::ROUTE;
    const double left = norm(goal_ - ctx.platform.vehicle().position);
    start_ = now;
    deadline_ = now + left / std::max(ctx.config.cbf.max_speed, 0.1) * 2.0 + ctx.config.fly_timeout_margin;
    task_manager::say("NavigateTo", "↩ " + why + "，回去 " + wp_.id + "（剩 " + fmt1(left) + " m）");
  }

  static std::string taskName(std::uint32_t id)
  {
    char b[16];
    std::snprintf(b, sizeof(b), "%08X", id);
    return b;
  }

  static constexpr double kSlowWarnPeriod = 10.0;

  task_manager::Waypoint wp_;
  Vec3 goal_;
  double start_{0.0};
  double deadline_{0.0};
  Phase phase_{Phase::ROUTE};
  Vec3 fire_goal_;
  std::uint32_t fire_task_{0};
  double monitor_start_{-1.0};
};

class AirRecon : public BT::StatefulActionNode
{
public:
  AirRecon(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<task_manager::Task>("task"),
      BT::InputPort<unsigned>("fire_tag", 101u, "火警的 tag 編號"),
      BT::InputPort<double>("min_confidence", 0.85, ""),
      BT::InputPort<int>("min_frames", 5, ""),
      BT::InputPort<double>("max_age", 0.5, ""),
      BT::InputPort<double>("settle_sec", 1.0, "懸停穩定這麼久才判斷（減速、傾斜時的影像位置不準）"),
      BT::InputPort<double>("view_radius", 6.0,
        "只認離懸停點這麼近的火點（水平，公尺；下視相機 5 m 高約看 12 m × 8 m）"),
      BT::InputPort<double>("monitor_timeout", 120.0,
        "看到火警後懸停監控、等地面處置任務結束最多幾秒（0 = 不等）"),
    };
  }

  BT::NodeStatus onStart() override
  {
    using task_manager::g_ctx;
    auto in = getInput<task_manager::Task>("task");
    double tx = 0, ty = 0, tz = 0, dur = 0;
    if (!in || !num(in.value(), "target.x", tx) || !num(in.value(), "target.y", ty) ||
      !num(in.value(), "target.z", tz) || !num(in.value(), "duration_sec", dur))
    {
      g_ctx.last_error = "BAD_TASK_DEF(air_recon 要 target.x/y/z、duration_sec)";
      task_manager::say("AirRecon", "✗ " + g_ctx.last_error);
      return BT::NodeStatus::FAILURE;
    }
    UavContext & ctx = uavCtx(*this);
    hold_ = {tx, ty, tz};
    duration_ = std::max(0.0, dur);
    start_ = ctx.platform.timeNow();
    fire_task_.reset();
    others_fire_ = false;
    monitor_start_ = -1.0;
    task_manager::say("AirRecon", "→ 在 " + g_ctx.current_waypoint + " 上空懸停 " + fmt1(duration_) + " s、認 tag");
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    using task_manager::g_ctx;
    UavContext & ctx = uavCtx(*this);
    ctx.moveTo(hold_);
    const double now = ctx.platform.timeNow();
    if (!fire_task_ && !others_fire_ && now - start_ >= getInput<double>("settle_sec").value_or(1.0)) {
      look(ctx, now);
    }
    if (now - start_ < duration_) {
      ctx.setExec(ExecMode::EXECUTING, duration_ - (now - start_), nullptr);
      return BT::NodeStatus::RUNNING;
    }
    // 週計畫：看到火警 → 發布地面處置任務 → 懸停監控到地面處置（狗）完成，才完成這個巡檢點（2026-10-13）
    const double timeout = getInput<double>("monitor_timeout").value_or(120.0);
    if (fire_task_ && timeout > 0.0) {
      const std::string id = taskName(*fire_task_);
      if (monitor_start_ < 0.0) {
        monitor_start_ = now;
        task_manager::say("AirRecon", "👁 懸停監控火點，等地面處置任務 " + id + " 結束（最多 " + fmt1(timeout) + " s）");
      }
      if (ctx.platform.taskClosed(*fire_task_)) {
        return done(ctx, "fire handled", "✓ 地面處置任務 " + id + " 結束（" + fmt1(now - monitor_start_) +
          " s），完成這個點");
      }
      if (now - monitor_start_ >= timeout) {
        return done(ctx, "fire unattended", "⚠ 等了 " + fmt1(timeout) + " s，地面處置任務 " + id +
          " 還沒結束，完成這個點、離開");
      }
      ctx.setExec(ExecMode::EXECUTING, timeout - (now - monitor_start_), nullptr);
      return BT::NodeStatus::RUNNING;
    }
    return done(ctx, "", "✓ 完成");
  }

  void onHalted() override
  {
    task_manager::say("AirRecon", "★ halt（換任務）");
    task_manager::preemptLatencyNote("AirRecon");
  }

private:
  // 認 tag：符合條件的火點。自己已經發布過的（例如飛過來的路上看到的）不再發布，改成監控那個任務；
  // 別台發布的不再發布、也不監控（它在監控）
  void look(UavContext & ctx, double now)
  {
    const unsigned tag = getInput<unsigned>("fire_tag").value_or(101u);
    const double min_conf = getInput<double>("min_confidence").value_or(0.85);
    const int min_frames = getInput<int>("min_frames").value_or(5);
    const double max_age = getInput<double>("max_age").value_or(0.5);
    const double radius = getInput<double>("view_radius").value_or(6.0);
    for (const FireObservation & o : ctx.platform.fireObservations()) {
      if (o.tag_id != tag || now - o.received > max_age || o.frames < min_frames || o.confidence < min_conf ||
        std::hypot(o.position.x - hold_.x, o.position.y - hold_.y) > radius)
      {
        continue;
      }
      if (const auto known = ctx.fireTaskNear(o.position)) {
        if (!ctx.ownFireTask(*known)) {
          // 別台報的、由它在監控（2026-10-14）：不再發布、不過去搶同一個火點上空，照常完成這個點
          task_manager::say("AirRecon", "🔥 看到的火點別台已經發布（地面處置任務 " + taskName(*known) + "），不監控");
          others_fire_ = true;
          return;
        }
        fire_task_ = *known;
        task_manager::say("AirRecon", "🔥 看到的火點已經發布過（地面處置任務 " + taskName(*known) + "）");
      } else {
        task_manager::say("AirRecon", "🔥 火警成立：tag " + std::to_string(tag) + "（confidence " +
          fmt1(o.confidence) + "）在 (" + fmt1(o.position.x) + ", " + fmt1(o.position.y) + ")");
        fire_task_ = ctx.reportFire(o.position);
      }
      // 週計畫：到火點上空懸停等待（2026-10-14；原本留在巡檢點上空）。已經處理完的火點不用過去
      if (!ctx.platform.taskClosed(*fire_task_)) {
        hold_ = {o.position.x, o.position.y, hold_.z};
        task_manager::say("AirRecon", "↪ 移到火點上空 (" + fmt1(hold_.x) + ", " + fmt1(hold_.y) + ") 懸停監控");
      }
      return;
    }
  }

  BT::NodeStatus done(UavContext & ctx, const std::string & detail, const std::string & msg)
  {
    task_manager::g_ctx.results.push_back({task_manager::g_ctx.current_waypoint, "air_recon", "OK",
      fire_task_ ? (detail.empty() ? "fire" : detail) : ""});
    ctx.setExec(ExecMode::IDLE, -1.0, nullptr);
    task_manager::say("AirRecon", msg);
    return BT::NodeStatus::SUCCESS;
  }

  static std::string taskName(std::uint32_t id)
  {
    char b[16];
    std::snprintf(b, sizeof(b), "%08X", id);
    return b;
  }

  static bool num(const task_manager::Task & t, const std::string & k, double & out)
  {
    const std::string v = t.get(k);
    try {
      std::size_t n = 0;
      out = std::stod(v, &n);
      return n == v.size() && std::isfinite(out);
    } catch (...) {
      return false;
    }
  }

  Vec3 hold_;
  double duration_{0.0};
  double start_{0.0};
  std::optional<std::uint32_t> fire_task_;
  bool others_fire_{false};       // 看到的是別台報的火（只印一次）
  double monitor_start_{-1.0};
};

inline void registerUavLeaves(BT::BehaviorTreeFactory & f)
{
  f.registerNodeType<NavigateTo>("NavigateTo");
  f.registerNodeType<AirRecon>("AirRecon");
}

}  // namespace uav_px4_bt
