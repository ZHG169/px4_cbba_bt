#include "uav_px4_bt/bt_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>

namespace uav_px4_bt
{

namespace
{
std::string taskName(std::uint32_t id)
{
  char buf[9];
  std::snprintf(buf, sizeof(buf), "%08X", id);
  return buf;
}

std::string fmt(const char * f, double a, double b, double c)
{
  char buf[160];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}

UavContext & ctxOf(const BT::TreeNode & node)
{
  return *node.config().blackboard->get<UavContext *>("ctx");
}

// 任務點上方 altitude（任務的 z 比較高時用任務的 z）
Vec3 waypointOf(const AssignedTask & t, double altitude)
{
  return {t.position.x, t.position.y, std::max(altitude, t.position.z)};
}

double remainingHover(const UavContext & ctx, const AssignedTask & t)
{
  const double elapsed = ctx.arrival >= 0.0 ? ctx.platform.timeNow() - ctx.arrival : 0.0;
  return std::max(0.0, t.duration - elapsed);
}
}  // namespace

// ===========================================================================
// UavContext
// ===========================================================================
double UavContext::moveTo(const Vec3 & goal)
{
  const VehicleState v = platform.vehicle();
  const CpfOutput out = cpfVelocity(v.position, v.velocity, goal, platform.neighbors(), config.cpf);
  platform.setVelocity(out.velocity);
  // 一次「接近」（最近鄰機 < 1.5 × safe_radius）結束時印這次的最近距離：懸停時鄰機在影響範圍內、
  // 互相推開到平衡點是正常的，不印
  const bool close = out.min_distance >= 0.0 && out.min_distance < 1.5 * config.cpf.safe_radius;
  if (close) {
    encounter_min_ = encounter_min_ < 0.0 ? out.min_distance : std::min(encounter_min_, out.min_distance);
  } else if (encounter_min_ >= 0.0) {
    platform.log(fmt("避碰：這次最接近鄰機 %.2f m（安全距離 %.1f m）", encounter_min_, config.cpf.safe_radius, 0));
    encounter_min_ = -1.0;
  }
  return norm(goal - v.position);
}

void UavContext::setExec(ExecMode mode, double remaining, const AssignedTask * active)
{
  ExecStatus s;
  s.mode = mode;
  s.preemptible = true;     // 無人機隨時可以改飛別的任務
  s.remaining = remaining;
  if (active != nullptr && active->task_id != 0) {
    s.active_task = active->task_id;
    s.active_version = active->version;
  }
  platform.setExecStatus(s);
}

namespace
{
// ===========================================================================
// 電量
// ===========================================================================
class IsUAVBatteryCritical : public BT::ConditionNode
{
public:
  IsUAVBatteryCritical(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("min_battery", 20.0, "電量（%）低於等於這個值就返航")};
  }

  BT::NodeStatus tick() override
  {
    UavContext & ctx = ctxOf(*this);
    const double min = getInput<double>("min_battery").value_or(20.0);
    const double battery = ctx.platform.vehicle().battery;
    // 起飛前不判斷（還沒用到電，返航也沒意義）；一旦觸發就不再恢復，電量讀數上下跳時不會來回切換
    if (!ctx.battery_critical && ctx.airborne && battery >= 0.0 && battery <= min) {
      ctx.battery_critical = true;
      ctx.platform.log(fmt("電量 %.1f %% ≤ %.1f %%：返航", battery, min, 0.0));
    }
    return ctx.battery_critical ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class ExecuteReturnToLaunch : public BT::StatefulActionNode
{
public:
  ExecuteReturnToLaunch(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<AssignedTask>("task", "執行中的任務：回報失敗（BATTERY_RTL）"),
      BT::InputPort<std::string>("mode", "AUTO_RTL", "PX4 的模式（目前只支援 AUTO_RTL）"),
    };
  }

  BT::NodeStatus onStart() override
  {
    UavContext & ctx = ctxOf(*this);
    const auto task = getInput<AssignedTask>("task");
    if (task && task->task_id != 0 && !ctx.reported(*task)) {
      ctx.platform.reportResult(*task, false, "BATTERY_RTL");
      ctx.markReported(*task);
    }
    // 故障：cbba_node 不再給新任務、撤回還沒接受的出價
    ctx.setExec(ExecMode::FAULT, -1.0, nullptr);
    ctx.platform.stopOffboard();
    ctx.platform.requestReturnToLaunch();
    sent_ = 1;
    last_ = ctx.platform.timeNow();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    UavContext & ctx = ctxOf(*this);
    ctx.setExec(ExecMode::FAULT, -1.0, nullptr);
    // 指令可能掉：每 2 s 再送一次，最多 3 次
    if (sent_ < 3 && ctx.platform.timeNow() - last_ >= 2.0) {
      ctx.platform.requestReturnToLaunch();
      ++sent_;
      last_ = ctx.platform.timeNow();
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  int sent_{0};
  double last_{0.0};
};

// ===========================================================================
// 起飛
// ===========================================================================
class TakeOff : public BT::StatefulActionNode
{
public:
  TakeOff(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("altitude", 5.0, "起飛高度（公尺，map z）")};
  }

  BT::NodeStatus onStart() override
  {
    UavContext & ctx = ctxOf(*this);
    const VehicleState v = ctx.platform.vehicle();
    if (ctx.airborne && v.armed && !v.landed) {
      return BT::NodeStatus::SUCCESS;
    }
    if (ctx.airborne) {
      ctx.platform.log("已經不在空中（解鎖狀態不見了）：重新起飛");
    }
    ctx.airborne = false;
    start_ = ctx.platform.timeNow();
    last_request_ = -1e9;
    have_climb_xy_ = false;
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    UavContext & ctx = ctxOf(*this);
    const VehicleState v = ctx.platform.vehicle();
    const double altitude = getInput<double>("altitude").value_or(5.0);
    const double now = ctx.platform.timeNow();
    ctx.setExec(ExecMode::IDLE, -1.0, nullptr);
    if (!v.valid || !v.ready) {
      ctx.platform.setVelocity({});
      return BT::NodeStatus::RUNNING;
    }
    if (!(v.armed && v.offboard)) {
      // PX4 要先收到 setpoint 才能切 offboard：先送 1 s 的零速度，再每秒要求一次 offboard＋解鎖
      ctx.platform.setVelocity({});
      if (now - start_ >= 1.0 && now - last_request_ >= 1.0) {
        ctx.platform.requestOffboardAndArm();
        last_request_ = now;
      }
      return BT::NodeStatus::RUNNING;
    }
    if (!have_climb_xy_) {
      climb_xy_ = v.position;
      have_climb_xy_ = true;
      ctx.platform.log(fmt("解鎖、offboard：爬升到 %.1f m", altitude, 0.0, 0.0));
    }
    ctx.moveTo({climb_xy_.x, climb_xy_.y, altitude});
    if (std::fabs(v.position.z - altitude) < ctx.config.takeoff_tolerance) {
      ctx.airborne = true;
      ctx.platform.log(fmt("到達 %.1f m，等待指派", altitude, 0.0, 0.0));
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  double start_{0.0};
  double last_request_{0.0};
  bool have_climb_xy_{false};
  Vec3 climb_xy_;
};

// ===========================================================================
// 任務
// ===========================================================================
class IsTaskValidAndUnchanged : public BT::ConditionNode
{
public:
  IsTaskValidAndUnchanged(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<AssignedTask>("task_in_execution", "執行中的任務（這個節點負責更新）"),
      BT::InputPort<AssignedTask>("blackboard_task", "cbba_node 目前的指派（assigned_task）"),
    };
  }

  BT::NodeStatus tick() override
  {
    UavContext & ctx = ctxOf(*this);
    const AssignedTask current = getInput<AssignedTask>("blackboard_task").value_or(AssignedTask{});
    const AssignedTask running = getInput<AssignedTask>("task_in_execution").value_or(AssignedTask{});
    if (current.task_id == 0 || ctx.reported(current)) {
      // 沒有任務，或已經回報過（cbba_node 確認之前可能還會短暫指派同一個）
      if (running.task_id != 0) {
        setOutput("task_in_execution", AssignedTask{});
      }
      return BT::NodeStatus::FAILURE;
    }
    if (current != running) {
      // 換了任務（或同一個任務的新版本）：這個 tick 回傳 FAILURE 讓 ReactiveSequence 中斷舊的序列，下個 tick 從頭執行
      setOutput("task_in_execution", current);
      ctx.arrival = -1.0;
      ctx.platform.log("新指派 " + taskName(current.task_id) + " v" + std::to_string(current.version) +
        fmt("：map (%.1f, %.1f, %.1f)", current.position.x, current.position.y, current.position.z));
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

class FlyToWaypoint : public BT::StatefulActionNode
{
public:
  FlyToWaypoint(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<AssignedTask>("task"),
      BT::InputPort<double>("altitude", 5.0, "飛行高度（公尺）"),
    };
  }

  BT::NodeStatus onStart() override
  {
    UavContext & ctx = ctxOf(*this);
    const auto task = getInput<AssignedTask>("task");
    if (!task || task->task_id == 0) {
      return BT::NodeStatus::FAILURE;
    }
    task_ = *task;
    ctx.waypoint = waypointOf(task_, getInput<double>("altitude").value_or(5.0));
    const double d = norm(ctx.waypoint - ctx.platform.vehicle().position);
    deadline_ = ctx.platform.timeNow() + d / std::max(ctx.config.cpf.max_speed, 0.1) * 2.0 +
      ctx.config.fly_timeout_margin;
    ctx.platform.log("前往 " + taskName(task_.task_id) +
      fmt("：(%.1f, %.1f, %.1f)", ctx.waypoint.x, ctx.waypoint.y, ctx.waypoint.z));
    last_log_ = ctx.platform.timeNow();
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    UavContext & ctx = ctxOf(*this);
    const double d = ctx.moveTo(ctx.waypoint);
    const double now = ctx.platform.timeNow();
    ctx.setExec(ExecMode::NAVIGATING, d / std::max(ctx.config.cpf.max_speed, 0.1) + task_.duration, &task_);
    if (d < ctx.config.reach_tolerance) {
      ctx.arrival = now;
      ctx.platform.log("到達 " + taskName(task_.task_id) + fmt("，停留 %.1f s", task_.duration, 0, 0));
      return BT::NodeStatus::SUCCESS;
    }
    if (now > deadline_) {
      ctx.platform.log("前往 " + taskName(task_.task_id) + fmt(" 逾時（剩 %.1f m）", d, 0, 0));
      return BT::NodeStatus::FAILURE;
    }
    if (now - last_log_ >= 2.0) {
      last_log_ = now;
      ctx.platform.log("前往 " + taskName(task_.task_id) + fmt("：剩 %.1f m", d, 0, 0));
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  AssignedTask task_;
  double deadline_{0.0};
  double last_log_{0.0};
};

class DetectAprilTagFire : public BT::StatefulActionNode
{
public:
  DetectAprilTagFire(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<AssignedTask>("task"),
      BT::InputPort<unsigned>("expected_tag_id", 101u, "火警的 tag 編號（tag36h11）"),
      BT::InputPort<double>("min_confidence", 0.85, "confidence 達到才判定火警"),
      BT::InputPort<int>("min_frames", 5, "confidence 至少要用幾張影像算"),
      BT::InputPort<double>("observe_sec", 3.0, "到達後最多觀察幾秒（含 settle_sec）"),
      BT::InputPort<double>("settle_sec", 1.0,
        "到達後先懸停穩定這麼久才判斷（偵測的 confidence 視窗裡不要有減速、傾斜時的影像，位置才準）"),
      BT::InputPort<double>("max_age", 0.5, "偵測結果多舊以內才算"),
      BT::OutputPort<Vec3>("fire_pose", "火點（map ENU）"),
      BT::OutputPort<unsigned>("fire_tag"),
    };
  }

  BT::NodeStatus onStart() override
  {
    UavContext & ctx = ctxOf(*this);
    task_ = getInput<AssignedTask>("task").value_or(AssignedTask{});
    start_ = ctx.platform.timeNow();
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    UavContext & ctx = ctxOf(*this);
    ctx.moveTo(ctx.waypoint);
    ctx.setExec(ExecMode::EXECUTING, remainingHover(ctx, task_), &task_);
    const unsigned tag = getInput<unsigned>("expected_tag_id").value_or(101u);
    const double min_conf = getInput<double>("min_confidence").value_or(0.85);
    const int min_frames = getInput<int>("min_frames").value_or(5);
    const double max_age = getInput<double>("max_age").value_or(0.5);
    const double now = ctx.platform.timeNow();
    const double settle = getInput<double>("settle_sec").value_or(1.0);
    if (ctx.arrival >= 0.0 && now - ctx.arrival < settle) {
      return BT::NodeStatus::RUNNING;
    }
    for (const FireObservation & o : ctx.platform.fireObservations()) {
      if (o.tag_id != tag || now - o.received > max_age || o.frames < min_frames ||
        o.confidence < min_conf)
      {
        continue;
      }
      if (ctx.reported_fire_tags.count(tag) > 0) {
        ctx.platform.log("看到火警 tag " + std::to_string(tag) + "，已經回報過，不再建立任務");
        return BT::NodeStatus::FAILURE;
      }
      setOutput("fire_pose", o.position);
      setOutput("fire_tag", tag);
      ctx.platform.log("火警成立：tag " + std::to_string(tag) +
        fmt("，confidence %.2f，位置 (%.1f, %.1f)", o.confidence, o.position.x, o.position.y));
      return BT::NodeStatus::SUCCESS;
    }
    if (now - start_ >= getInput<double>("observe_sec").value_or(3.0)) {
      return BT::NodeStatus::FAILURE;   // 沒有火警
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  AssignedTask task_;
  double start_{0.0};
};

class ReportFireEvent : public BT::SyncActionNode
{
public:
  ReportFireEvent(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<Vec3>("fire_pose"), BT::InputPort<unsigned>("fire_tag")};
  }

  BT::NodeStatus tick() override
  {
    UavContext & ctx = ctxOf(*this);
    const auto pose = getInput<Vec3>("fire_pose");
    const auto tag = getInput<unsigned>("fire_tag");
    if (!pose || !tag) {
      return BT::NodeStatus::FAILURE;
    }
    // 地面處置任務：地面上的點（z = 0），無人機不出價、機器狗接
    const std::uint32_t id = ctx.platform.createTask(kTaskGroundIntervention, {pose->x, pose->y, 0.0},
        ctx.config.fire_task_value, ctx.config.fire_task_duration, ctx.config.fire_task_deadline);
    ctx.reported_fire_tags.insert(*tag);
    ctx.platform.log("廣播地面處置任務 " + taskName(id) + fmt("：(%.1f, %.1f)", pose->x, pose->y, 0));
    return BT::NodeStatus::SUCCESS;
  }
};

class HoverAndMonitor : public BT::StatefulActionNode
{
public:
  HoverAndMonitor(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {BT::InputPort<AssignedTask>("task")};}

  BT::NodeStatus onStart() override
  {
    task_ = getInput<AssignedTask>("task").value_or(AssignedTask{});
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    UavContext & ctx = ctxOf(*this);
    ctx.moveTo(ctx.waypoint);
    const double remaining = remainingHover(ctx, task_);
    ctx.setExec(ExecMode::EXECUTING, remaining, &task_);
    return remaining <= 0.0 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  AssignedTask task_;
};

class ReportTaskResult : public BT::SyncActionNode
{
public:
  ReportTaskResult(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<AssignedTask>("task"),
      BT::InputPort<bool>("success", true, ""),
      BT::InputPort<std::string>("detail", "", "失敗原因（只記 log）"),
    };
  }

  BT::NodeStatus tick() override
  {
    UavContext & ctx = ctxOf(*this);
    const auto task = getInput<AssignedTask>("task");
    if (!task || task->task_id == 0) {
      return BT::NodeStatus::FAILURE;
    }
    if (!ctx.reported(*task)) {
      const bool success = getInput<bool>("success").value_or(true);
      const std::string detail = getInput<std::string>("detail").value_or("");
      ctx.platform.reportResult(*task, success, detail);
      ctx.markReported(*task);
      ctx.platform.log("回報 " + taskName(task->task_id) + " v" + std::to_string(task->version) +
        (success ? " 完成" : " 失敗 " + detail));
    }
    return BT::NodeStatus::SUCCESS;
  }
};

class LandOrLoiter : public BT::StatefulActionNode
{
public:
  LandOrLoiter(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    UavContext & ctx = ctxOf(*this);
    hold_ = ctx.platform.vehicle().position;
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    UavContext & ctx = ctxOf(*this);
    // 讓路：影響範圍內有執行任務中的鄰機（例如它的任務點就在這裡）時不拉回待命點，只剩排斥把自己推開；
    // 推開後的位置就是新的待命點。否則兩台都被吸向同一點、繞圈，執行中的那台會到不了
    const VehicleState v = ctx.platform.vehicle();
    bool yield = false;
    for (const Neighbor & n : ctx.platform.neighbors()) {
      yield = yield || (n.busy && norm(n.position - v.position) < ctx.config.cpf.influence_radius);
    }
    if (yield) {
      hold_ = v.position;
      if (!yielding_) {
        ctx.platform.log("待命中：讓路給執行任務的鄰機");
      }
    }
    yielding_ = yield;
    ctx.moveTo(hold_);
    ctx.setExec(ExecMode::IDLE, -1.0, nullptr);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  Vec3 hold_;
  bool yielding_{false};
};
}  // namespace

void registerUavNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<IsUAVBatteryCritical>("IsUAVBatteryCritical");
  factory.registerNodeType<ExecuteReturnToLaunch>("ExecuteReturnToLaunch");
  factory.registerNodeType<TakeOff>("TakeOff");
  factory.registerNodeType<IsTaskValidAndUnchanged>("IsTaskValidAndUnchanged");
  factory.registerNodeType<FlyToWaypoint>("FlyToWaypoint");
  factory.registerNodeType<DetectAprilTagFire>("DetectAprilTagFire");
  factory.registerNodeType<ReportFireEvent>("ReportFireEvent");
  factory.registerNodeType<HoverAndMonitor>("HoverAndMonitor");
  factory.registerNodeType<ReportTaskResult>("ReportTaskResult");
  factory.registerNodeType<LandOrLoiter>("LandOrLoiter");
}

}  // namespace uav_px4_bt
