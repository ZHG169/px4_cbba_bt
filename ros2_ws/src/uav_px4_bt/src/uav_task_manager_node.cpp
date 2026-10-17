// 無人機的正式 BT 節點（2026-10-13）：機器狗端（伯宇）task_manager 骨架的 node 層，換成無人機的葉節點與 PX4。
//
// 來源：docs/handoff_uav_2026-10-12/bt/task_manager/src/task_manager_node.cpp。照搬的部分（行為、log 字串相同）：
//   onTick（哨兵 FAILURE 一拍不算結束）、onCurrentTask（callback A）、dispatchExternal、kick（寫黑板後立刻 tick）、
//   beginExternalRun、releaseExternal、abortNow、finish、waypointInHand、關機訊號（先停任務再關）
// 和狗不同（UAV_BT_GUIDE v2 §2、§4）：
//   - 外部任務的幾何 makeExternalWaypoint：目標正上方、air_recon（skeleton/external_task.hpp）
//   - 葉節點 NavigateTo／AirRecon（skeleton/uav_leaves.hpp）；PX4、setpoint、位置廣播、task_result、new_task 用
//     Px4Platform（和備用 BT 共用）
//   - 樹外（skeleton/uav_flight.hpp）：起飛完成才 tick 樹；樹沒在跑時待命（return_home_delay 秒後飛回起飛點上空）與讓路；低電量返航（BATTERY_RTL）、
//     降落（NOT_AIRBORNE）時結束任務、交還 CBBA
//   - robot_state 不由這裡發（px4_state_bridge 發，兩邊都發會互相蓋，GUIDE §4）；沒有 TF、markers、/mission_status。
//     mode 由這裡決定（2026-10-17）：/<ns>/bt_mode（std_msgs/UInt8，值同 RobotState 的 AUTO／MANUAL／ESTOP），
//     bridge 照抄進 robot_state。AUTO = 起飛完成、BT 在控制（AIRBORNE）；ESTOP = 急停；其他（地上、爬升中、
//     被接管、PX4 失效保護、低電量返航）MANUAL。改變時立刻送，平常每 mode_period 送一次（bridge 逾時當 MANUAL）
//   - 被接管（offboard 掉了，UavFlight 的 OVERRIDE）：照狗的平板接管，手上的任務以 PREEMPTED_BY_MANUAL 交還；
//     PX4 失效保護時以 PX4_FAILSAFE（STOPPED）交還。期間收到的任務直接用同一個原因交還
//   - 沒有巡檢 yaml、/start_mission、/stop_mission：巡檢點是 CBBA 的 AIR_RECON 任務，佇列裡只有外部點
//   - 沒有 manual_override（平板不接無人機；無人機的接管是遙控器／地面站切 PX4 模式）、Nav2 reaper
//   - 急停話題 /<ns>/emergency_stop（參數 emergency_topic）：狗的 /emergency_stop 是全域的，無人機各台分開；
//     急停時樹 halt、原地懸停（不是降落）
//
//   ROS  /<ns>/current_task（cbba_node → 這裡，reliable、transient_local）
//        /<ns>/task_result、/<ns>/new_task、/<ns>/fire_detection、PX4 話題：Px4Platform
//
// 每 tick_period_ms：beginTick → 起飛／返航（UavFlight）→ 在空中而且佇列有點：tick 樹，否則待命 → endTick（送 setpoint）。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <swarm_interfaces/msg/robot_state.hpp>
#include <swarm_interfaces/msg/task.hpp>
#include <swarm_interfaces/msg/task_result.hpp>

#include "uav_px4_bt/px4_platform.hpp"
#include "uav_px4_bt/skeleton/bt_nodes.hpp"
#include "uav_px4_bt/skeleton/external_task.hpp"
#include "uav_px4_bt/skeleton/uav_flight.hpp"
#include "uav_px4_bt/skeleton/uav_leaves.hpp"

namespace
{
std::atomic<bool> g_shutdown{false};
void onSignal(int) {g_shutdown = true;}
}  // namespace

namespace uav_px4_bt
{

using SwarmTask = swarm_interfaces::msg::Task;
using RobotStateMsg = swarm_interfaces::msg::RobotState;
using task_manager::g_ctx;
using task_manager::Waypoint;

static_assert(task_manager::reason::PERMANENT == swarm_interfaces::msg::TaskResult::PERMANENT &&
  task_manager::reason::TRANSIENT == swarm_interfaces::msg::TaskResult::TRANSIENT &&
  task_manager::reason::PREEMPTED == swarm_interfaces::msg::TaskResult::PREEMPTED &&
  task_manager::reason::STOPPED == swarm_interfaces::msg::TaskResult::STOPPED,
  "external_task.hpp reason:: must match swarm_interfaces/msg/TaskResult.msg");

class UavTaskManagerNode : public rclcpp::Node
{
public:
  UavTaskManagerNode()
  : Node("uav_task_manager")
  {
    tick_period_ = declare_parameter<int>("tick_period_ms", 50);   // 20 Hz：哨兵最壞延遲一個週期（spec §3.1）
    bt_file_ = declare_parameter<std::string>("bt_file",
        ament_index_cpp::get_package_share_directory("uav_px4_bt") + "/trees/uav_mission.xml");
    const double min_battery = declare_parameter<double>("min_battery", 20.0);
    const double mission_timeout = declare_parameter<double>("mission_timeout", 0.0);   // 0 = 骨架預設 1800 s
    // 任務做完、待命這麼久後飛回起飛點上空（負值 = 原地懸停）
    const double return_home_delay = declare_parameter<double>("return_home_delay", 2.0);
    // 看到火警後懸停監控、等地面處置任務結束最多幾秒（0 = 不等）
    fire_monitor_timeout_ = declare_parameter<double>("fire_task.monitor_timeout", 120.0);
    log_transitions_ = declare_parameter<bool>("log_transitions", false);
    mode_period_ = declare_parameter<double>("mode_period", 0.5);   // bt_mode 平常多久送一次（s）
    const UavContextConfig cfg = Px4Platform::declareConfig(*this);
    platform_ = std::make_unique<Px4Platform>(*this);
    const std::string emergency_topic = declare_parameter<std::string>("emergency_topic",
        "/" + platform_->ns() + "/emergency_stop");
    const std::string mode_topic = declare_parameter<std::string>("bt_mode_topic",
        "/" + platform_->ns() + "/bt_mode");

    ctx_ = std::make_unique<UavContext>(*platform_, cfg);
    flight_ = std::make_unique<UavFlight>(*ctx_, platform_->altitude(), min_battery, return_home_delay);
    if (mission_timeout > 0.0) {
      mission_timeout_ = mission_timeout;
    }

    task_manager::registerAll(factory_);
    registerUavLeaves(factory_);

    g_ctx.task_result_pub = [this](uint32_t id, bool ok, const std::string & detail) {
        platform_->publishResult(id, ok, detail, task_manager::failureReason(ok, detail));
      };
    current_task_sub_ = create_subscription<SwarmTask>("/" + platform_->ns() + "/current_task",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](SwarmTask::SharedPtr m) {onCurrentTask(*m);});
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(emergency_topic, 10,
        [this](std_msgs::msg::Bool::SharedPtr m) {
          if (m->data != g_ctx.emergency) {
            RCLCPP_WARN(get_logger(), "緊急停止：%s", m->data ? "TRUE" : "FALSE");
          }
          g_ctx.emergency = m->data;
        });

    mode_pub_ = create_publisher<std_msgs::msg::UInt8>(mode_topic, rclcpp::QoS(1).reliable());
    timer_ = create_wall_timer(std::chrono::milliseconds(tick_period_), [this]() {onTick();});
    RCLCPP_INFO(get_logger(), "uav%u：樹 %s，速度 %.1f m/s，返航電量 %.0f %%；%s", platform_->agentId(),
      bt_file_.c_str(), cfg.cbf.max_speed, min_battery, platform_->describe().c_str());
  }

  // 黑板和樹建一次、用到關機（spec 3.1）。XML 壞了樹是空的、節點待命，收到任務時再試
  bool buildTree()
  {
    auto bb = BT::Blackboard::create();
    waypoints_ = std::make_shared<std::deque<Waypoint>>();
    bb->set("waypoints", waypoints_);
    bb->set("current_task_id", uint32_t{0});   // 哨兵的兩個 key（spec §3.3）
    bb->set("active_task_id", uint32_t{0});
    bb->set<UavContext *>("uav_ctx", ctx_.get());
    bb->set<double>("altitude", platform_->altitude());
    bb->set<double>("fire_monitor_timeout", fire_monitor_timeout_);
    try {
      tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(bt_file_, bb));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "建不了行為樹 %s：%s", bt_file_.c_str(), e.what());
      return false;
    }
    if (log_transitions_) {
      cout_logger_ = std::make_unique<BT::StdCoutLogger>(*tree_);
    }
    return true;
  }

private:
  void onTick()
  {
    if (g_shutdown) {   // Ctrl-C／SIGTERM：先停任務（交還 CBBA）再關
      if (running_) {
        RCLCPP_WARN(get_logger(), "收到關機訊號，任務進行中 → 先停");
        abortNow("SHUTDOWN");
      }
      std_msgs::msg::UInt8 m;   // 不用等 bridge 逾時
      m.data = RobotStateMsg::MANUAL;
      mode_pub_->publish(m);
      rclcpp::shutdown();
      return;
    }
    platform_->beginTick();
    switch (flight_->update()) {
      case FlightEvent::LANDED:
        if (running_) {
          abortNow("NOT_AIRBORNE");
        }
        break;
      case FlightEvent::BATTERY_RTL:
      case FlightEvent::TAKEOVER:
      case FlightEvent::FAILSAFE: {
          const std::string why = flight_->phase() == FlightPhase::RTL ? "BATTERY_RTL" : flight_->overrideReason();
          if (running_) {
            abortNow(why);
          } else {
            rejectQueued(why);
          }
          break;
        }
      default:
        break;
    }
    if (flight_->phase() == FlightPhase::AIRBORNE) {
      tickMission();
      if (!running_ && !g_ctx.emergency) {
        flight_->loiter();
      }
    }
    publishMode();
    platform_->endTick();
  }

  // 給 px4_state_bridge 填 robot_state.mode（照機器狗端：BT 決定能不能接任務）
  void publishMode()
  {
    const std::uint8_t mode = g_ctx.emergency ? RobotStateMsg::ESTOP :
      flight_->phase() == FlightPhase::AIRBORNE ? RobotStateMsg::AUTO : RobotStateMsg::MANUAL;
    const double t = platform_->timeNow();
    if (mode == mode_ && t - mode_sent_ < mode_period_) {
      return;
    }
    if (mode != mode_) {
      static const char * const kNames[] = {"AUTO", "MANUAL", "ESTOP"};
      RCLCPP_INFO(get_logger(), "mode → %s", mode < 3 ? kNames[mode] : "?");
    }
    mode_ = mode;
    mode_sent_ = t;
    std_msgs::msg::UInt8 m;
    m.data = mode;
    mode_pub_->publish(m);
  }

  // 伯宇 onTick 的任務部分（起飛完成才會到這裡）
  void tickMission()
  {
    if (!running_) {
      if (!tree_ || waypoints_->empty()) {
        return;
      }
      beginExternalRun();   // 待命中來了外部點：這個佇列就是這次的任務
    }
    ++g_ctx.tick;
    const auto st = task_manager::tickOrAbort(*tree_);
    const double elapsed = (now() - mission_start_).seconds();

    // 新的外部任務來時哨兵 FAILURE 一次：下面的子樹 halt，下一拍樹照常（spec §3.4.1）
    const bool sentinel = st == BT::NodeStatus::FAILURE && !g_ctx.emergency && g_ctx.end_reason != "INTERNAL_ERROR";
    if (st != BT::NodeStatus::RUNNING && !sentinel) {
      if (g_ctx.end_reason == "INTERNAL_ERROR") {
      } else if (st == BT::NodeStatus::SUCCESS && !g_ctx.aborted) {
        g_ctx.end_reason = "COMPLETED";
      } else if (g_ctx.emergency) {
        g_ctx.end_reason = "EMERGENCY";
      } else {
        g_ctx.end_reason = "POLICY_ABORT";
      }
      if (g_ctx.end_reason != "COMPLETED" && g_ctx.abort_waypoint.empty()) {
        g_ctx.abort_waypoint = g_ctx.current_waypoint;
      }
      finish();   // in_hand_ 還是上一拍的：執行中的點帶著原因交還
      return;
    }
    in_hand_ = task_manager::waypointInHand(*tree_);
    if (elapsed > g_ctx.mission_timeout) {
      abortNow("MISSION_TIMEOUT");
    }
  }

  void beginExternalRun()
  {
    g_ctx.reset();
    if (mission_timeout_) {
      g_ctx.mission_timeout = *mission_timeout_;
    }
    mission_start_ = now();
    running_ = true;
    flight_->leaveHold();
    RCLCPP_INFO(get_logger(), "▶ 外部任務插隊，閒置中開跑：%s", waypoints_->front().id.c_str());
  }

  // ── CBBA（spec §3.4.3–3.4.5）───────────────────────────────────────────

  // 上一拍結束時手上的點：callback 在兩拍之間執行，finish() 要的是 halt 之前的狀態
  const Waypoint * activeWaypoint(Waypoint & buf)
  {
    if (!in_hand_) {
      return nullptr;
    }
    buf = *in_hand_;
    return &buf;
  }

  // callback 寫了 {current_task_id}：立刻 tick 一次，halt 不等計時器（spec §3.7 的 t0 從這裡開始），計時器重新起算
  void kick()
  {
    g_ctx.preempt_t0 = std::chrono::steady_clock::now();
    onTick();
    timer_->reset();
  }

  // callback A：/<ns>/current_task（spec §3.4.5）
  void onCurrentTask(const SwarmTask & t)
  {
    if (!tree_ && !buildTree()) {
      return;
    }
    auto bb = tree_->rootBlackboard();
    uint32_t cur = bb->get<uint32_t>("current_task_id");
    Waypoint buf;
    const Waypoint * active = activeWaypoint(buf);
    if (t.task_id == 0) {   // 取消
      const bool was_running = active && task_manager::hasPrefix(active->id, "cbba_");
      if (task_manager::cancelExternal(*waypoints_, active, "cbba_", cur)) {
        bb->set("current_task_id", cur);
        RCLCPP_INFO(get_logger(), "CBBA 取消任務 → %s", was_running ? "哨兵下一拍 halt" : "從佇列移除");
        kick();
      } else {
        RCLCPP_INFO(get_logger(), "CBBA 取消任務：手上沒有 CBBA 任務，忽略");
      }
      return;
    }
    if (t.task_id == cur) {   // 重送／transient_local 重播
      return;
    }
    if (g_ctx.emergency) {
      g_ctx.task_result_pub(t.task_id, false, "EMERGENCY");
      RCLCPP_WARN(get_logger(), "急停中收到 CBBA 任務 %u → 回 EMERGENCY 交還", t.task_id);
      return;
    }
    if (flight_->phase() == FlightPhase::RTL) {
      g_ctx.task_result_pub(t.task_id, false, "BATTERY_RTL");
      RCLCPP_WARN(get_logger(), "返航中收到 CBBA 任務 %u → 回 BATTERY_RTL 交還", t.task_id);
      return;
    }
    if (flight_->phase() == FlightPhase::OVERRIDE) {   // mode MANUAL 送到之前派下來的
      g_ctx.task_result_pub(t.task_id, false, flight_->overrideReason());
      RCLCPP_WARN(get_logger(), "被接管中收到 CBBA 任務 %u → 回 %s 交還", t.task_id, flight_->overrideReason());
      return;
    }
    // 手上另一個 cbba 點直接換掉：CBBA 知道
    const Waypoint wp = task_manager::makeExternalWaypoint("cbba_", t.task_id, t.position.x, t.position.y,
        t.position.z, platform_->altitude(), std::max(0.0f, t.duration_sec));
    task_manager::insertExternal(*waypoints_, active, wp, t.task_id, cur);
    bb->set("current_task_id", cur);
    RCLCPP_INFO(get_logger(), "%s → 插隊  目標 (%.2f, %.2f, %.2f)  停留 %.1f s%s", wp.id.c_str(), t.position.x,
      t.position.y, t.position.z, t.duration_sec,
      flight_->phase() == FlightPhase::AIRBORNE ? "" : "  （還沒起飛完成，起飛後執行）");
    kick();
  }

  // 任務結束（spec §3.4.1 abortNow）：手上還沒回報的 cbba 點帶著原因交還，哨兵的 key 歸零
  void releaseExternal(const std::string & detail)
  {
    if (!tree_) {
      return;
    }
    auto bb = tree_->rootBlackboard();
    // 兩個 id 都是 0 = ReportTaskResult／HandleWaypointFailure 已經回報過這個點，{wp} 只是殘值
    if (bb->get<uint32_t>("current_task_id") == 0 && bb->get<uint32_t>("active_task_id") == 0) {
      g_ctx.manual_active = false;
      return;
    }
    Waypoint buf;
    const Waypoint * active = activeWaypoint(buf);
    const std::string cbba = task_manager::externalInHand(*waypoints_, active, "cbba_",
        bb->get<uint32_t>("current_task_id"));
    if (!cbba.empty()) {
      g_ctx.task_result_pub(task_manager::externalTaskId(cbba), false, detail);
      RCLCPP_WARN(get_logger(), "任務結束，%s 交還 CBBA：%s", cbba.c_str(), detail.c_str());
    }
    g_ctx.manual_active = false;
    bb->set("current_task_id", uint32_t{0});
    bb->set("active_task_id", uint32_t{0});
  }

  // 樹沒在跑時佇列裡等著的點（還沒起飛完成就收到的）：返航了不會做，交還 CBBA
  void rejectQueued(const std::string & detail)
  {
    if (!tree_ || waypoints_->empty()) {
      return;
    }
    releaseExternal(detail);
    waypoints_->clear();
  }

  // 從外面停掉執行中的任務：halt，再照一般流程回報
  void abortNow(const std::string & reason)
  {
    tree_->haltTree();
    g_ctx.end_reason = reason;
    if (g_ctx.abort_waypoint.empty()) {
      g_ctx.abort_waypoint = g_ctx.current_waypoint;
    }
    finish();
  }

  void finish()
  {
    task_manager::printReport();
    RCLCPP_INFO(get_logger(), "■ 任務結束：%s（%d ticks）", g_ctx.end_reason.c_str(), g_ctx.tick);
    running_ = false;
    releaseExternal(g_ctx.end_reason);
    waypoints_->clear();   // 中止後留下的點不能在下次開跑時執行；樹本身留著
    in_hand_.reset();
    ctx_->setExec(ExecMode::IDLE, -1.0, nullptr);
    RCLCPP_INFO(get_logger(), "待命中 — 等 CBBA 的下一個任務");
  }

  int tick_period_{50};
  std::string bt_file_;
  bool log_transitions_{false};
  double fire_monitor_timeout_{120.0};
  double mode_period_{0.5};
  std::uint8_t mode_{255};   // 上一次送的 bt_mode（255 = 還沒送過）
  double mode_sent_{0.0};
  std::optional<double> mission_timeout_;

  std::unique_ptr<Px4Platform> platform_;
  std::unique_ptr<UavContext> ctx_;
  std::unique_ptr<UavFlight> flight_;

  rclcpp::Subscription<SwarmTask>::SharedPtr current_task_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool running_{false};
  std::optional<Waypoint> in_hand_;   // 上一拍結束時正在做的點（見 activeWaypoint）
  rclcpp::Time mission_start_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  std::shared_ptr<std::deque<Waypoint>> waypoints_ = std::make_shared<std::deque<Waypoint>>();
  std::unique_ptr<BT::StdCoutLogger> cout_logger_;
};

}  // namespace uav_px4_bt

int main(int argc, char ** argv)
{
  // 自己處理訊號：設旗標，onTick 先 halt 樹、交還任務再關
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  try {
    auto node = std::make_shared<uav_px4_bt::UavTaskManagerNode>();
    node->buildTree();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("uav_task_manager"), "啟動失敗：%s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
