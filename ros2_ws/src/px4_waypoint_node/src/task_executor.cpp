// 代替 BT 的簡易任務執行者（測試 cbba_node 用），每台無人機在自己的 uav 容器裡各跑一個。
//
// 和 takeoff_hover 一樣起飛，之後照 cbba_node 的指派飛：
//   WAIT_READY → STREAM → ENGAGE → CLIMB   同 takeoff_hover（離地後 cbba_node 才開始參與出價）
//   IDLE        懸停在目前位置，等 /uavN/assigned_task
//   GOTO        以 cruise_speed 把 setpoint 往任務點移（任務點正上方 altitude 公尺）
//   WORK        到達後停留 duration_sec，結束時發 /uavN/task_result（DONE）一次，回到 IDLE
//
// 指派中途換成別的任務就直接改飛新任務；變成 task_id = 0 就原地懸停。
// 已回報過的任務不再執行（cbba_node 確認完成之前可能還會短暫指派同一個）。
// 要測「失敗交回」時，手動對 /uavN/task_result 發 CANCELLED（docker/scripts/cbba_task.sh fail）。
//
// 座標：任務是 map ENU；PX4 local 是 NED、以出生點為原點。出生點預設和 px4_sitl.sh、cbba_node 一樣，
// 沿 y 方向每台間隔 UAV_SPAWN_SPACING 公尺。
//
// 注意：offboard 需要持續的 setpoint，停掉這個節點 PX4 會觸發 failsafe。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/task.hpp>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using TaskMsg = swarm_interfaces::msg::Task;

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

std::string envOr(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr && *v != '\0' ? v : fallback;
}

std::string taskName(std::uint32_t id)
{
  char buf[9];
  std::snprintf(buf, sizeof(buf), "%08X", id);
  return buf;
}
}  // namespace

class TaskExecutor : public rclcpp::Node
{
public:
  TaskExecutor()
  : Node("task_executor")
  {
    const int agent_id = declare_parameter<int>("agent_id", std::stoi(envOr("UAV_ID", "1")));
    px4_ns_ = declare_parameter<std::string>("px4_ns", envOr("UAV_NS", ""));
    altitude_ = declare_parameter<double>("altitude", 5.0);            // 飛行高度（公尺，相對出生點）
    // setpoint 移動速度（m/s）；要和 cbba_node 的 cruise_speed 一致，出價估的時間才準
    cruise_speed_ = declare_parameter<double>("cruise_speed", 5.0);
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);
    reach_tol_ = declare_parameter<double>("reach_tolerance", 0.5);    // 視為到達的距離
    const double spacing = std::stod(envOr("UAV_SPAWN_SPACING", "2.0"));
    const auto spawn = declare_parameter<std::vector<double>>(
      "spawn_enu", {0.0, (agent_id - 1) * spacing, 0.0});
    if (spawn.size() != 3) {
      throw std::invalid_argument("spawn_enu must have 3 elements");
    }
    spawn_x_ = spawn[0];
    spawn_y_ = spawn[1];
    spawn_z_ = spawn[2];

    const std::string ns = px4_ns_.empty() ? "" : "/" + px4_ns_;
    const std::string fmu = ns + "/fmu";
    const auto px4_qos = rclcpp::SensorDataQoS();   // PX4 的話題是 best effort

    offboard_pub_ = create_publisher<OffboardControlMode>(fmu + "/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>(fmu + "/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>(fmu + "/in/vehicle_command", 10);
    status_sub_ = create_subscription<VehicleStatus>(
      fmu + "/out/vehicle_status_v1", px4_qos,
      [this](VehicleStatus::ConstSharedPtr msg) {status_ = *msg; have_status_ = true;});
    position_sub_ = create_subscription<VehicleLocalPosition>(
      fmu + "/out/vehicle_local_position_v1", px4_qos,
      [this](VehicleLocalPosition::ConstSharedPtr msg) {position_ = *msg; have_position_ = true;});

    // 和 cbba_node 的 QoS 一致
    result_pub_ = create_publisher<TaskMsg>(ns + "/task_result", rclcpp::QoS(20).reliable());
    assigned_sub_ = create_subscription<TaskMsg>(
      ns + "/assigned_task", rclcpp::QoS(1).reliable().transient_local(),
      [this](TaskMsg::ConstSharedPtr msg) {onAssigned(*msg);});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_), [this]() {step();});

    RCLCPP_INFO(get_logger(), "uav%d：PX4 %s，飛行高度 %.1f m，速度 %.1f m/s，出生點 map (%.1f, %.1f, %.1f)",
      agent_id, fmu.c_str(), altitude_, cruise_speed_, spawn_x_, spawn_y_, spawn_z_);
  }

private:
  enum class Phase { WAIT_READY, STREAM, ENGAGE, CLIMB, IDLE, GOTO, WORK };

  static const char * name(Phase p)
  {
    switch (p) {
      case Phase::WAIT_READY: return "WAIT_READY";
      case Phase::STREAM: return "STREAM";
      case Phase::ENGAGE: return "ENGAGE";
      case Phase::CLIMB: return "CLIMB";
      case Phase::IDLE: return "IDLE";
      case Phase::GOTO: return "GOTO";
      case Phase::WORK: return "WORK";
    }
    return "?";
  }

  void enter(Phase next)
  {
    RCLCPP_INFO(get_logger(), "%s -> %s", name(phase_), name(next));
    phase_ = next;
    phase_ticks_ = 0;
  }

  bool airborne() const
  {
    return phase_ == Phase::IDLE || phase_ == Phase::GOTO || phase_ == Phase::WORK;
  }

  bool engaged() const
  {
    return status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
           status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  void onAssigned(const TaskMsg & msg)
  {
    have_assigned_ = true;
    assigned_ = msg;
    if (msg.task_id == 0) {
      RCLCPP_INFO(get_logger(), "指派：無");
    } else {
      RCLCPP_INFO(get_logger(), "指派：%s map (%.1f, %.1f, %.1f)，停留 %.1f s",
        taskName(msg.task_id).c_str(), msg.position.x, msg.position.y, msg.position.z,
        msg.duration_sec);
    }
    if (airborne()) {
      follow();
    }
  }

  // 依目前的指派決定 IDLE 或 GOTO
  void follow()
  {
    const bool has_task = have_assigned_ && assigned_.task_id != 0 &&
      done_.count(assigned_.task_id) == 0;
    if (!has_task) {
      if (phase_ != Phase::IDLE) {
        // 原地懸停：setpoint 停在目前位置
        sp_x_ = position_.x;
        sp_y_ = position_.y;
        enter(Phase::IDLE);
      }
      active_ = 0;
      return;
    }
    if (assigned_.task_id == active_ && (phase_ == Phase::GOTO || phase_ == Phase::WORK)) {
      return;   // 同一個任務，繼續
    }
    active_ = assigned_.task_id;
    active_duration_ = std::max(0.0f, assigned_.duration_sec);
    // map ENU → local NED（以出生點為原點）
    goal_x_ = static_cast<float>(assigned_.position.y - spawn_y_);
    goal_y_ = static_cast<float>(assigned_.position.x - spawn_x_);
    goal_z_ = -static_cast<float>(std::max(altitude_, assigned_.position.z - spawn_z_));
    RCLCPP_INFO(get_logger(), "前往 %s：NED (%.1f, %.1f, %.1f)", taskName(active_).c_str(),
      goal_x_, goal_y_, goal_z_);
    enter(Phase::GOTO);
  }

  void step()
  {
    ++phase_ticks_;
    const double dt = 1.0 / rate_hz_;
    switch (phase_) {
      case Phase::WAIT_READY:
        if (have_status_ && have_position_ && status_.pre_flight_checks_pass &&
          position_.xy_valid && position_.z_valid)
        {
          sp_x_ = position_.x;
          sp_y_ = position_.y;
          sp_z_ = position_.z - static_cast<float>(altitude_);
          yaw_ = position_.heading;
          enter(Phase::STREAM);
        } else if (phase_ticks_ % static_cast<int>(rate_hz_ * 5) == 0) {
          RCLCPP_INFO(get_logger(), "等待 PX4：status %d position %d preflight %d",
            have_status_, have_position_, have_status_ && status_.pre_flight_checks_pass);
        }
        return;   // 還沒有 setpoint

      case Phase::STREAM:
        if (phase_ticks_ >= static_cast<int>(rate_hz_)) {
          enter(Phase::ENGAGE);
        }
        break;

      case Phase::ENGAGE:
        if (engaged()) {
          enter(Phase::CLIMB);
        } else if (phase_ticks_ % static_cast<int>(rate_hz_) == 1) {
          sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);   // custom mode, OFFBOARD
          sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        break;

      case Phase::CLIMB:
        if (std::fabs(position_.z - sp_z_) < reach_tol_) {
          RCLCPP_INFO(get_logger(), "到達 %.1f m，等待指派", altitude_);
          enter(Phase::IDLE);
          follow();
        }
        break;

      case Phase::IDLE:
        break;

      case Phase::GOTO: {
        moveSetpoint(dt);
        const double d = std::sqrt(sq(position_.x - goal_x_) + sq(position_.y - goal_y_) +
            sq(position_.z - goal_z_));
        if (d < reach_tol_) {
          RCLCPP_INFO(get_logger(), "到達 %s，停留 %.1f s", taskName(active_).c_str(), active_duration_);
          enter(Phase::WORK);
        } else if (phase_ticks_ % static_cast<int>(rate_hz_ * 2) == 0) {
          RCLCPP_INFO(get_logger(), "前往 %s：剩 %.1f m", taskName(active_).c_str(), d);
        }
        break;
      }

      case Phase::WORK:
        if (phase_ticks_ * dt >= active_duration_) {
          reportDone();
          follow();
        }
        break;
    }
    publishSetpoint();
  }

  static double sq(double v) {return v * v;}

  // setpoint 以 cruise_speed 往目標移動，避免一次跳太遠讓 PX4 全速衝過去
  void moveSetpoint(double dt)
  {
    const double dx = goal_x_ - sp_x_;
    const double dy = goal_y_ - sp_y_;
    const double dz = goal_z_ - sp_z_;
    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    const double step = cruise_speed_ * dt;
    if (d <= step) {
      sp_x_ = goal_x_;
      sp_y_ = goal_y_;
      sp_z_ = goal_z_;
      return;
    }
    sp_x_ += static_cast<float>(dx / d * step);
    sp_y_ += static_cast<float>(dy / d * step);
    sp_z_ += static_cast<float>(dz / d * step);
  }

  void reportDone()
  {
    TaskMsg msg = assigned_;
    msg.task_id = active_;
    msg.status = TaskMsg::DONE;
    msg.status_stamp = get_clock()->now();
    result_pub_->publish(msg);
    done_.insert(active_);
    RCLCPP_INFO(get_logger(), "回報 %s 完成", taskName(active_).c_str());
    active_ = 0;
  }

  std::uint64_t stampUs() const
  {
    return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  void publishSetpoint()
  {
    OffboardControlMode mode{};
    mode.timestamp = stampUs();
    mode.position = true;
    offboard_pub_->publish(mode);

    TrajectorySetpoint sp{};
    sp.timestamp = mode.timestamp;
    sp.position = {sp_x_, sp_y_, sp_z_};
    sp.velocity = {kNaN, kNaN, kNaN};
    sp.acceleration = {kNaN, kNaN, kNaN};
    sp.jerk = {kNaN, kNaN, kNaN};
    sp.yaw = yaw_;
    sp.yawspeed = kNaN;
    setpoint_pub_->publish(sp);
  }

  void sendCommand(std::uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand cmd{};
    cmd.timestamp = stampUs();
    cmd.command = command;
    cmd.param1 = param1;
    cmd.param2 = param2;
    // SITL 實例 -i N 的 system id 是 N+1，所以從 vehicle_status 讀，不寫死
    cmd.target_system = status_.system_id;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    command_pub_->publish(cmd);
  }

  std::string px4_ns_;
  double altitude_{5.0};
  double cruise_speed_{5.0};
  double rate_hz_{10.0};
  double reach_tol_{0.5};
  double spawn_x_{0.0};
  double spawn_y_{0.0};
  double spawn_z_{0.0};

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<TaskMsg>::SharedPtr result_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<TaskMsg>::SharedPtr assigned_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_{};
  VehicleLocalPosition position_{};
  bool have_status_{false};
  bool have_position_{false};

  TaskMsg assigned_{};
  bool have_assigned_{false};
  std::uint32_t active_{0};
  float active_duration_{0.0f};
  std::set<std::uint32_t> done_;

  Phase phase_{Phase::WAIT_READY};
  int phase_ticks_{0};
  float sp_x_{0.0f};
  float sp_y_{0.0f};
  float sp_z_{0.0f};
  float yaw_{0.0f};
  float goal_x_{0.0f};
  float goal_y_{0.0f};
  float goal_z_{0.0f};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskExecutor>());
  rclcpp::shutdown();
  return 0;
}
