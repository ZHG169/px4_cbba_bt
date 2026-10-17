#include "uav_px4_bt/px4_platform.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <random>
#include <stdexcept>

namespace uav_px4_bt
{

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using FireDetectionMsg = swarm_interfaces::msg::FireDetection;
using TaskMsg = swarm_interfaces::msg::Task;
using TaskResultMsg = swarm_interfaces::msg::TaskResult;

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

std::string envOr(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr && *v != '\0' ? v : fallback;
}

double steadyNow()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

Vec3 toEnu(double n, double e, double d) {return {e, n, -d};}
}  // namespace

Px4Platform::Px4Platform(rclcpp::Node & node)
: node_(node)
{
  const int agent_id = node.declare_parameter<int>("agent_id", std::stoi(envOr("UAV_ID", "1")));
  if (agent_id < 1 || agent_id > 254) {
    throw std::invalid_argument("agent_id must be 1~254");
  }
  agent_id_ = static_cast<std::uint16_t>(agent_id);
  const std::string px4_ns = node.declare_parameter<std::string>("px4_ns", envOr("UAV_NS", ""));
  ns_ = node.declare_parameter<std::string>("ns", envOr("UAV_NS", "uav" + std::to_string(agent_id)));
  altitude_ = node.declare_parameter<double>("altitude", 5.0);
  position_timeout_ = node.declare_parameter<double>("position_timeout", 0.5);
  neighbor_timeout_ = node.get_parameter_or<double>("cbf.neighbor_timeout", CbfParams{}.neighbor_timeout);

  const double spacing = std::stod(envOr("UAV_SPAWN_SPACING", "2.0"));
  const auto spawn = node.declare_parameter<std::vector<double>>("spawn_enu", {0.0, (agent_id - 1) * spacing, 0.0});
  if (spawn.size() != 3) {
    throw std::invalid_argument("spawn_enu must have 3 elements");
  }
  spawn_ = {spawn[0], spawn[1], spawn[2]};

  // ---------- 位置廣播 ----------
  udp_.group = node.declare_parameter<std::string>("udp_group", udp_.group);
  udp_.port = static_cast<std::uint16_t>(node.declare_parameter<int>("beacon_port", 14601));
  udp_.interface_ip = node.declare_parameter<std::string>("mesh_ip", envOr("MESH_IP", ""));
  beacon_period_ = 1.0 / node.declare_parameter<double>("beacon_hz", 10.0);
  link_ = std::make_unique<cbba_core::UdpLink>(udp_);
  table_ = std::make_unique<NeighborTable>(agent_id_);
  std::random_device rd;
  session_ = (static_cast<std::uint64_t>(rd()) << 32) | rd();
  next_seq_ = 0x8000u + static_cast<std::uint32_t>(std::time(nullptr) % 0x8000);

  // ---------- PX4 ----------
  fmu_ = (px4_ns.empty() ? "" : "/" + px4_ns) + "/fmu";
  const auto px4_qos = rclcpp::SensorDataQoS();
  offboard_pub_ = node.create_publisher<OffboardControlMode>(fmu_ + "/in/offboard_control_mode", 10);
  setpoint_pub_ = node.create_publisher<TrajectorySetpoint>(fmu_ + "/in/trajectory_setpoint", 10);
  command_pub_ = node.create_publisher<VehicleCommand>(fmu_ + "/in/vehicle_command", 10);
  status_sub_ = node.create_subscription<VehicleStatus>(fmu_ + "/out/vehicle_status_v1", px4_qos,
      [this](VehicleStatus::ConstSharedPtr m) {status_ = *m; have_status_ = true;});
  position_sub_ = node.create_subscription<VehicleLocalPosition>(
    fmu_ + "/out/vehicle_local_position_v1", px4_qos,
    [this](VehicleLocalPosition::ConstSharedPtr m) {
      position_ = *m;
      position_rx_ = steadyNow();
      if (!have_position_ && std::isfinite(m->heading)) {
        yaw_ = m->heading;   // 航向固定在第一次收到的方向
      }
      have_position_ = true;
    });
  battery_sub_ = node.create_subscription<BatteryStatus>(fmu_ + "/out/battery_status_v1", px4_qos,
      [this](BatteryStatus::ConstSharedPtr m) {
        battery_ = m->connected && m->remaining >= 0.0f ? m->remaining * 100.0 : -1.0;
      });
  land_sub_ = node.create_subscription<VehicleLandDetected>(fmu_ + "/out/vehicle_land_detected", px4_qos,
      [this](VehicleLandDetected::ConstSharedPtr m) {landed_ = m->landed;});

  // ---------- cbba_node（QoS 和 cbba_node 一致）、偵測 ----------
  const std::string prefix = "/" + ns_;
  result_pub_ = node.create_publisher<TaskResultMsg>(prefix + "/task_result", rclcpp::QoS(20).reliable());
  new_task_pub_ = node.create_publisher<TaskMsg>(prefix + "/new_task", rclcpp::QoS(20).reliable());
  closed_sub_ = node.create_subscription<TaskMsg>(prefix + "/task_closed",
      rclcpp::QoS(100).reliable().transient_local(),
      [this](TaskMsg::ConstSharedPtr m) {closed_.insert(m->task_id);});
  // 全隊已知的任務：只留地面處置任務（火點），每個 task_id 一筆
  announced_sub_ = node.create_subscription<TaskMsg>(prefix + "/task_announced",
      rclcpp::QoS(256).reliable().transient_local(),
      [this](TaskMsg::ConstSharedPtr m) {
        if (m->type != kTaskGroundIntervention) {
          return;
        }
        for (const KnownGroundTask & g : ground_) {
          if (g.task_id == m->task_id) {
            return;
          }
        }
        ground_.push_back({m->task_id, {m->position.x, m->position.y, m->position.z}});
      });
  fire_sub_ = node.create_subscription<FireDetectionMsg>(prefix + "/fire_detection", rclcpp::QoS(20),
      [this](FireDetectionMsg::ConstSharedPtr m) {
        FireObservation o;
        o.tag_id = m->tag_id;
        o.confidence = m->confidence;
        o.frames = m->frames;
        o.position = {m->position.x, m->position.y, m->position.z};
        o.received = steadyNow();
        fires_[m->tag_id] = o;
      });
}

UavContextConfig Px4Platform::declareConfig(rclcpp::Node & node)
{
  UavContextConfig cfg;
  // 和 cbba_node 的 cruise_speed 一致，出價估的時間才準
  cfg.cbf.max_speed = node.declare_parameter<double>("cruise_speed", cfg.cbf.max_speed);
  cfg.cbf.max_vertical_speed = node.declare_parameter<double>("max_vertical_speed", cfg.cbf.max_vertical_speed);
  cfg.cbf.k_att = node.declare_parameter<double>("cbf.k_att", cfg.cbf.k_att);
  cfg.cbf.brake_accel = node.declare_parameter<double>("cbf.brake_accel", cfg.cbf.brake_accel);
  cfg.cbf.brake_vertical_accel = node.declare_parameter<double>("cbf.brake_vertical_accel",
      cfg.cbf.brake_vertical_accel);
  cfg.cbf.safe_radius = node.declare_parameter<double>("cbf.safe_radius", cfg.cbf.safe_radius);
  cfg.cbf.gamma = node.declare_parameter<double>("cbf.gamma", cfg.cbf.gamma);
  cfg.cbf.influence_radius = node.declare_parameter<double>("cbf.influence_radius", cfg.cbf.influence_radius);
  cfg.cbf.yield_radius = node.declare_parameter<double>("cbf.yield_radius", cfg.cbf.yield_radius);
  cfg.cbf.k_tan = node.declare_parameter<double>("cbf.k_tan", cfg.cbf.k_tan);
  cfg.cbf.neighbor_timeout = node.declare_parameter<double>("cbf.neighbor_timeout", cfg.cbf.neighbor_timeout);
  cfg.cbf.max_extrapolation = node.declare_parameter<double>("cbf.max_extrapolation", cfg.cbf.max_extrapolation);
  cfg.motion.max_accel = node.declare_parameter<double>("motion.max_accel", cfg.motion.max_accel);
  cfg.motion.max_vertical_accel = node.declare_parameter<double>("motion.max_vertical_accel",
      cfg.motion.max_vertical_accel);
  cfg.motion.leash = node.declare_parameter<double>("motion.leash", cfg.motion.leash);
  cfg.motion.hold_radius = node.declare_parameter<double>("motion.hold_radius", cfg.motion.hold_radius);
  cfg.motion.reset_gap = node.declare_parameter<double>("motion.reset_gap", cfg.motion.reset_gap);
  cfg.reach_tolerance = node.declare_parameter<double>("reach_tolerance", cfg.reach_tolerance);
  cfg.takeoff_tolerance = node.declare_parameter<double>("takeoff_tolerance", cfg.takeoff_tolerance);
  cfg.fly_timeout_margin = node.declare_parameter<double>("fly_timeout_margin", cfg.fly_timeout_margin);
  cfg.fly_timeout_abort = node.declare_parameter<bool>("fly_timeout_abort", cfg.fly_timeout_abort);
  cfg.fire_task_value = node.declare_parameter<double>("fire_task.value", cfg.fire_task_value);
  cfg.fire_task_duration = node.declare_parameter<double>("fire_task.duration", cfg.fire_task_duration);
  cfg.fire_task_deadline = node.declare_parameter<double>("fire_task.deadline", cfg.fire_task_deadline);
  cfg.fire_dedup_radius = node.declare_parameter<double>("fire_task.dedup_radius", cfg.fire_dedup_radius);
  cfg.fire_divert_on_sight = node.declare_parameter<bool>("fire_task.divert_on_sight", cfg.fire_divert_on_sight);
  cfg.fire_monitor_value = node.declare_parameter<double>("fire_task.monitor_value", cfg.fire_monitor_value);
  cfg.fire_monitor_sec = node.declare_parameter<double>("fire_task.monitor_sec", cfg.fire_monitor_sec);
  return cfg;
}

std::string Px4Platform::describe() const
{
  char buf[256];
  std::snprintf(buf, sizeof(buf), "PX4 %s，CBBA /%s，高度 %.1f m；位置廣播 %s:%u；出生點 map (%.1f, %.1f, %.1f)",
    fmu_.c_str(), ns_.c_str(), altitude_, udp_.group.c_str(), udp_.port, spawn_.x, spawn_.y, spawn_.z);
  return buf;
}

void Px4Platform::beginTick()
{
  const double t = steadyNow();
  for (const auto & bytes : link_->receiveAll()) {
    table_->receive(bytes.data(), bytes.size(), t);
  }
  commanded_ = false;
}

void Px4Platform::endTick()
{
  // 這個 tick 沒有人下命令：保持上一次的位置（速度 0）；還沒有位置命令時速度 0
  if (!commanded_) {
    velocity_cmd_ = {};
  }
  if (offboard_ && have_position_) {
    publishSetpoint();
  }
  const double t = steadyNow();
  if (t - last_beacon_ >= beacon_period_) {
    last_beacon_ = t;
    sendBeacon();
  }
}

double Px4Platform::timeNow() const {return steadyNow();}

VehicleState Px4Platform::vehicle() const
{
  VehicleState v;
  v.valid = have_position_ && steadyNow() - position_rx_ <= position_timeout_ &&
    position_.xy_valid && position_.z_valid;
  v.ready = v.valid && have_status_ && status_.pre_flight_checks_pass;
  v.position = toEnu(position_.x, position_.y, position_.z) + spawn_;
  v.velocity = toEnu(position_.vx, position_.vy, position_.vz);
  v.battery = battery_;
  v.armed = have_status_ && status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
  v.offboard = have_status_ && status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  v.landed = landed_;
  v.failsafe = have_status_ && status_.failsafe;
  v.user_took_over = have_status_ && status_.failsafe_and_user_took_over;
  return v;
}

std::vector<Neighbor> Px4Platform::neighbors() const {return table_->neighbors(steadyNow(), neighbor_timeout_);}

std::vector<FireObservation> Px4Platform::fireObservations() const
{
  std::vector<FireObservation> out;
  for (const auto & [id, o] : fires_) {
    out.push_back(o);
  }
  return out;
}

void Px4Platform::setVelocity(const Vec3 & v)
{
  velocity_cmd_ = v;
  has_position_cmd_ = false;
  commanded_ = true;
}

void Px4Platform::setSetpoint(const Vec3 & position, const Vec3 & velocity)
{
  position_cmd_ = position;
  velocity_cmd_ = velocity;
  has_position_cmd_ = true;
  commanded_ = true;
}

void Px4Platform::requestOffboardAndArm()
{
  sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);   // custom mode, OFFBOARD
  sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void Px4Platform::requestReturnToLaunch() {sendCommand(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);}

void Px4Platform::reportResult(const AssignedTask & task, bool success, const std::string & detail)
{
  // 失敗分類（spec §2.3 規則 3）：低電量返航 = 載具本身停了；路過發現火警、改去監控 = 被搶占；
  // 其他（飛行逾時等）可以再試
  const std::uint8_t reason = success ? TaskResultMsg::NONE :
    detail == "BATTERY_RTL" ? TaskResultMsg::STOPPED :
    detail == "FIRE_FOUND" ? TaskResultMsg::PREEMPTED : TaskResultMsg::TRANSIENT;
  publishResult(task.task_id, success, detail, reason);
}

void Px4Platform::publishResult(std::uint32_t task_id, bool success, const std::string & detail, std::uint8_t reason)
{
  TaskResultMsg msg;
  msg.task_id = task_id;
  msg.success = success;
  msg.detail = detail;
  msg.reason = reason;
  result_pub_->publish(msg);
}

std::uint32_t Px4Platform::createTask(std::uint8_t type, const Vec3 & p, double value, double duration,
  double deadline)
{
  TaskMsg msg;
  msg.task_id = static_cast<std::uint32_t>(agent_id_) * 65536u + next_seq_;
  next_seq_ = next_seq_ >= 0xFFFFu ? 0x8000u : next_seq_ + 1;
  msg.type = type;
  msg.position.x = p.x;
  msg.position.y = p.y;
  msg.position.z = p.z;
  msg.value = static_cast<float>(value);
  msg.duration_sec = static_cast<float>(duration);
  msg.deadline_sec = static_cast<float>(deadline);
  msg.status = TaskMsg::OPEN;
  new_task_pub_->publish(msg);
  return msg.task_id;
}

void Px4Platform::log(const std::string & text) {RCLCPP_INFO(node_.get_logger(), "%s", text.c_str());}

void Px4Platform::publishSetpoint()
{
  const std::uint64_t stamp = static_cast<std::uint64_t>(node_.get_clock()->now().nanoseconds() / 1000);
  OffboardControlMode mode{};
  mode.timestamp = stamp;
  mode.position = has_position_cmd_;      // 位置控制＋速度前饋（一般的移動、懸停）
  mode.velocity = !has_position_cmd_;     // 只有速度（起飛前的零速度）
  offboard_pub_->publish(mode);

  TrajectorySetpoint sp{};
  sp.timestamp = stamp;
  if (has_position_cmd_) {
    // map ENU → PX4 local NED（以出生點為原點）
    const Vec3 local = position_cmd_ - spawn_;
    sp.position = {static_cast<float>(local.y), static_cast<float>(local.x), static_cast<float>(-local.z)};
  } else {
    sp.position = {kNaN, kNaN, kNaN};
  }
  sp.velocity = {static_cast<float>(velocity_cmd_.y), static_cast<float>(velocity_cmd_.x),
    static_cast<float>(-velocity_cmd_.z)};
  sp.acceleration = {kNaN, kNaN, kNaN};
  sp.jerk = {kNaN, kNaN, kNaN};
  sp.yaw = yaw_;
  sp.yawspeed = kNaN;
  setpoint_pub_->publish(sp);
}

void Px4Platform::sendBeacon()
{
  const VehicleState v = vehicle();
  Beacon b;
  b.sender = agent_id_;
  b.session = session_;
  b.sequence = beacon_seq_++;
  b.airborne = v.armed && !v.landed;
  b.position_valid = v.valid;
  b.busy = exec_.mode == ExecMode::NAVIGATING || exec_.mode == ExecMode::EXECUTING;
  b.position = v.position;
  b.velocity = v.velocity;
  link_->send(encodeBeacon(b));
}

void Px4Platform::sendCommand(std::uint32_t command, float param1, float param2)
{
  VehicleCommand cmd{};
  cmd.timestamp = static_cast<std::uint64_t>(node_.get_clock()->now().nanoseconds() / 1000);
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

}  // namespace uav_px4_bt
