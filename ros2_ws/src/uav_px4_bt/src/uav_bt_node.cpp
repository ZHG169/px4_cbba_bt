// 無人機的備用 BT 節點（2026-10-09）：取代 px4_waypoint_node/task_executor，每台無人機的機上電腦各跑一個。
//
//   PX4   <px4_ns>/fmu/out/{vehicle_status_v1, vehicle_local_position_v1, battery_status_v1, vehicle_land_detected}
//         <px4_ns>/fmu/in/{offboard_control_mode, trajectory_setpoint（速度）, vehicle_command}
//   CBBA  <ns>/assigned_task → BT；BT → <ns>/task_result、<ns>/exec_state（2 Hz＋改變時）、<ns>/new_task（火警）
//   偵測  <ns>/fire_detection（apriltag_fire_detector）
//   機間  飛行層的位置廣播（UDP multicast，beacon_port，10 Hz），給 CPF 避碰（格式見 flight_beacon.hpp）
//
// 每 1/tick_hz 秒：把 assigned_task 寫進 blackboard（current_uav_task）→ tick 一次樹 → 把樹設定的速度送給 PX4。
// 樹的節點不碰 ROS，透過 UavPlatform（這個檔案實作）讀狀態、下指令。
//
// 座標：BT 用 map ENU；PX4 local 是 NED、以出生點為原點。出生點預設和 px4_sitl.sh、px4_state_bridge 一樣，
// 沿 y 方向每台間隔 UAV_SPAWN_SPACING 公尺。航向固定在起飛時的方向（下視相機的影像方向不會一直轉）。
//
// 建立的火警任務 task_id = agent_id × 65536 + 流水號；流水號從 0x8000 + (Unix 秒 mod 0x8000) 開始遞增，
// BT 重開機後不會和之前建立過的重複（除非剛好隔 9.1 小時的整數倍），也不會和 cbba_task.sh 手動用的小流水號撞。
//
// 注意：offboard 需要持續的 setpoint，停掉這個節點 PX4 會觸發 failsafe。
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/exec_state.hpp>
#include <swarm_interfaces/msg/fire_detection.hpp>
#include <swarm_interfaces/msg/task.hpp>
#include <swarm_interfaces/msg/task_result.hpp>

#include "cbba_core/udp_link.hpp"
#include "uav_px4_bt/bt_nodes.hpp"
#include "uav_px4_bt/flight_beacon.hpp"
#include "uav_px4_bt/uav_platform.hpp"

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using ExecStateMsg = swarm_interfaces::msg::ExecState;
using FireDetectionMsg = swarm_interfaces::msg::FireDetection;
using TaskMsg = swarm_interfaces::msg::Task;
using TaskResultMsg = swarm_interfaces::msg::TaskResult;

namespace uav_px4_bt
{

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
}  // namespace

class UavBtNode : public rclcpp::Node, public UavPlatform
{
public:
  UavBtNode()
  : Node("uav_bt_node")
  {
    const int agent_id = declare_parameter<int>("agent_id", std::stoi(envOr("UAV_ID", "1")));
    if (agent_id < 1 || agent_id > 254) {
      throw std::invalid_argument("agent_id must be 1~254");
    }
    agent_id_ = static_cast<std::uint16_t>(agent_id);
    const std::string px4_ns = declare_parameter<std::string>("px4_ns", envOr("UAV_NS", ""));
    const std::string ns = declare_parameter<std::string>("ns", envOr("UAV_NS", "uav" + std::to_string(agent_id)));
    altitude_ = declare_parameter<double>("altitude", 5.0);
    const double tick_hz = declare_parameter<double>("tick_hz", 20.0);
    position_timeout_ = declare_parameter<double>("position_timeout", 0.5);
    const std::string tree_file = declare_parameter<std::string>("tree_file",
        ament_index_cpp::get_package_share_directory("uav_px4_bt") + "/trees/main_uav_tree.xml");

    UavContextConfig cfg;
    // 和 cbba_node 的 cruise_speed 一致，出價估的時間才準
    cfg.cpf.max_speed = declare_parameter<double>("cruise_speed", cfg.cpf.max_speed);
    cfg.cpf.max_vertical_speed = declare_parameter<double>("max_vertical_speed", cfg.cpf.max_vertical_speed);
    cfg.cpf.k_att = declare_parameter<double>("cpf.k_att", cfg.cpf.k_att);
    cfg.cpf.influence_radius = declare_parameter<double>("cpf.influence_radius", cfg.cpf.influence_radius);
    cfg.cpf.safe_radius = declare_parameter<double>("cpf.safe_radius", cfg.cpf.safe_radius);
    cfg.cpf.k_rep = declare_parameter<double>("cpf.k_rep", cfg.cpf.k_rep);
    cfg.cpf.k_tan = declare_parameter<double>("cpf.k_tan", cfg.cpf.k_tan);
    cfg.cpf.horizon = declare_parameter<double>("cpf.horizon", cfg.cpf.horizon);
    cfg.cpf.neighbor_timeout = declare_parameter<double>("cpf.neighbor_timeout", cfg.cpf.neighbor_timeout);
    cfg.reach_tolerance = declare_parameter<double>("reach_tolerance", cfg.reach_tolerance);
    cfg.fire_task_value = declare_parameter<double>("fire_task.value", cfg.fire_task_value);
    cfg.fire_task_duration = declare_parameter<double>("fire_task.duration", cfg.fire_task_duration);
    cfg.fire_task_deadline = declare_parameter<double>("fire_task.deadline", cfg.fire_task_deadline);
    neighbor_timeout_ = cfg.cpf.neighbor_timeout;

    const double spacing = std::stod(envOr("UAV_SPAWN_SPACING", "2.0"));
    const auto spawn = declare_parameter<std::vector<double>>(
      "spawn_enu", {0.0, (agent_id - 1) * spacing, 0.0});
    if (spawn.size() != 3) {
      throw std::invalid_argument("spawn_enu must have 3 elements");
    }
    spawn_ = {spawn[0], spawn[1], spawn[2]};

    // ---------- 位置廣播 ----------
    cbba_core::UdpConfig udp;
    udp.group = declare_parameter<std::string>("udp_group", udp.group);
    udp.port = static_cast<std::uint16_t>(declare_parameter<int>("beacon_port", 14601));
    udp.interface_ip = declare_parameter<std::string>("mesh_ip", envOr("MESH_IP", ""));
    beacon_period_ = 1.0 / declare_parameter<double>("beacon_hz", 10.0);
    link_ = std::make_unique<cbba_core::UdpLink>(udp);
    table_ = std::make_unique<NeighborTable>(agent_id_);
    std::random_device rd;
    session_ = (static_cast<std::uint64_t>(rd()) << 32) | rd();
    next_seq_ = 0x8000u + static_cast<std::uint32_t>(std::time(nullptr) % 0x8000);

    // ---------- PX4 ----------
    const std::string fmu = (px4_ns.empty() ? "" : "/" + px4_ns) + "/fmu";
    const auto px4_qos = rclcpp::SensorDataQoS();
    offboard_pub_ = create_publisher<OffboardControlMode>(fmu + "/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>(fmu + "/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>(fmu + "/in/vehicle_command", 10);
    status_sub_ = create_subscription<VehicleStatus>(fmu + "/out/vehicle_status_v1", px4_qos,
        [this](VehicleStatus::ConstSharedPtr m) {status_ = *m; have_status_ = true;});
    position_sub_ = create_subscription<VehicleLocalPosition>(
      fmu + "/out/vehicle_local_position_v1", px4_qos,
      [this](VehicleLocalPosition::ConstSharedPtr m) {
        position_ = *m;
        position_rx_ = steadyNow();
        if (!have_position_ && std::isfinite(m->heading)) {
          yaw_ = m->heading;   // 航向固定在第一次收到的方向
        }
        have_position_ = true;
      });
    battery_sub_ = create_subscription<BatteryStatus>(fmu + "/out/battery_status_v1", px4_qos,
        [this](BatteryStatus::ConstSharedPtr m) {
          battery_ = m->connected && m->remaining >= 0.0f ? m->remaining * 100.0 : -1.0;
        });
    land_sub_ = create_subscription<VehicleLandDetected>(fmu + "/out/vehicle_land_detected", px4_qos,
        [this](VehicleLandDetected::ConstSharedPtr m) {landed_ = m->landed;});

    // ---------- cbba_node（QoS 和 cbba_node 一致）----------
    const std::string prefix = "/" + ns;
    result_pub_ = create_publisher<TaskResultMsg>(prefix + "/task_result", rclcpp::QoS(20).reliable());
    new_task_pub_ = create_publisher<TaskMsg>(prefix + "/new_task", rclcpp::QoS(20).reliable());
    exec_pub_ = create_publisher<ExecStateMsg>(prefix + "/exec_state", rclcpp::QoS(10).best_effort());
    assigned_sub_ = create_subscription<TaskMsg>(prefix + "/assigned_task",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](TaskMsg::ConstSharedPtr m) {onAssigned(*m);});
    fire_sub_ = create_subscription<FireDetectionMsg>(prefix + "/fire_detection", rclcpp::QoS(20),
        [this](FireDetectionMsg::ConstSharedPtr m) {
          FireObservation o;
          o.tag_id = m->tag_id;
          o.confidence = m->confidence;
          o.frames = m->frames;
          o.position = {m->position.x, m->position.y, m->position.z};
          o.received = steadyNow();
          fires_[m->tag_id] = o;
        });

    // ---------- 樹 ----------
    ctx_ = std::make_unique<UavContext>(*this, cfg);
    registerUavNodes(factory_);
    blackboard_ = BT::Blackboard::create();
    blackboard_->set<UavContext *>("ctx", ctx_.get());
    blackboard_->set<AssignedTask>("current_uav_task", AssignedTask{});
    blackboard_->set<AssignedTask>("active_uav_task", AssignedTask{});
    blackboard_->set<double>("altitude", altitude_);
    tree_ = factory_.createTreeFromFile(tree_file, blackboard_);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_hz), [this]() {step();});
    RCLCPP_INFO(get_logger(),
      "uav%u：樹 %s，PX4 %s，CBBA %s，高度 %.1f m、速度 %.1f m/s；位置廣播 %s:%u；出生點 map (%.1f, %.1f, %.1f)",
      agent_id_, tree_file.c_str(), fmu.c_str(), prefix.c_str(), altitude_, cfg.cpf.max_speed,
      udp.group.c_str(), udp.port, spawn_.x, spawn_.y, spawn_.z);
  }

  // ===========================================================================
  // UavPlatform
  // ===========================================================================
  double timeNow() const override {return steadyNow();}

  VehicleState vehicle() const override
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
    return v;
  }

  std::vector<Neighbor> neighbors() const override {return table_->neighbors(steadyNow(), neighbor_timeout_);}

  std::vector<FireObservation> fireObservations() const override
  {
    std::vector<FireObservation> out;
    for (const auto & [id, o] : fires_) {
      out.push_back(o);
    }
    return out;
  }

  void setVelocity(const Vec3 & v) override {velocity_cmd_ = v;}

  void requestOffboardAndArm() override
  {
    sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);   // custom mode, OFFBOARD
    sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
  }

  void requestReturnToLaunch() override {sendCommand(VehicleCommand::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH);}

  void stopOffboard() override {offboard_ = false;}

  void reportResult(const AssignedTask & task, bool success, const std::string & detail) override
  {
    TaskResultMsg msg;
    msg.task_id = task.task_id;
    msg.success = success;
    msg.detail = detail;
    msg.assignment_version = task.version;
    result_pub_->publish(msg);
  }

  std::uint32_t createTask(std::uint8_t type, const Vec3 & p, double value, double duration,
    double deadline) override
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

  void setExecStatus(const ExecStatus & s) override {exec_ = s;}

  void log(const std::string & text) override {RCLCPP_INFO(get_logger(), "%s", text.c_str());}

private:
  static Vec3 toEnu(double n, double e, double d) {return {e, n, -d};}

  void onAssigned(const TaskMsg & m)
  {
    AssignedTask t;
    if (m.task_id != 0) {
      t.task_id = m.task_id;
      t.type = m.type;
      t.position = {m.position.x, m.position.y, m.position.z};
      t.duration = std::max(0.0f, m.duration_sec);
      t.version = m.assignment_version;
    }
    assigned_ = t;
  }

  void step()
  {
    const double t = steadyNow();
    for (const auto & bytes : link_->receiveAll()) {
      table_->receive(bytes.data(), bytes.size(), t);
    }
    // 樹：沒有任何動作設定速度時保持 0（懸停）
    velocity_cmd_ = {};
    blackboard_->set<AssignedTask>("current_uav_task", assigned_);
    tree_.tickOnce();

    if (offboard_ && have_position_) {
      publishSetpoint();
    }
    if (t - last_beacon_ >= beacon_period_) {
      last_beacon_ = t;
      sendBeacon();
    }
    publishExec(t);
  }

  void publishSetpoint()
  {
    const std::uint64_t stamp = static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
    OffboardControlMode mode{};
    mode.timestamp = stamp;
    mode.velocity = true;
    offboard_pub_->publish(mode);

    TrajectorySetpoint sp{};
    sp.timestamp = stamp;
    sp.position = {kNaN, kNaN, kNaN};
    // map ENU → NED
    sp.velocity = {static_cast<float>(velocity_cmd_.y), static_cast<float>(velocity_cmd_.x),
      static_cast<float>(-velocity_cmd_.z)};
    sp.acceleration = {kNaN, kNaN, kNaN};
    sp.jerk = {kNaN, kNaN, kNaN};
    sp.yaw = yaw_;
    sp.yawspeed = kNaN;
    setpoint_pub_->publish(sp);
  }

  void sendBeacon()
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

  void publishExec(double t)
  {
    const bool changed = exec_.mode != last_exec_.mode || exec_.active_task != last_exec_.active_task ||
      exec_.active_version != last_exec_.active_version;
    if (!changed && t - last_exec_pub_ < 0.5) {
      return;
    }
    last_exec_pub_ = t;
    last_exec_ = exec_;
    ExecStateMsg msg;
    msg.header.stamp = get_clock()->now();
    msg.execution_state = static_cast<std::uint8_t>(exec_.mode);
    msg.preemptible = exec_.preemptible;
    msg.estimated_remaining_time = static_cast<float>(exec_.remaining);
    msg.has_active = exec_.active_task != 0;
    msg.active_task_id = exec_.active_task;
    msg.active_assignment_version = exec_.active_version;
    exec_pub_->publish(msg);
  }

  void sendCommand(std::uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand cmd{};
    cmd.timestamp = static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
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

  std::uint16_t agent_id_{1};
  double altitude_{5.0};
  double position_timeout_{0.5};
  double neighbor_timeout_{1.0};
  Vec3 spawn_;

  std::unique_ptr<cbba_core::UdpLink> link_;
  std::unique_ptr<NeighborTable> table_;
  std::uint64_t session_{0};
  std::uint32_t beacon_seq_{0};
  double beacon_period_{0.1};
  double last_beacon_{0.0};
  std::uint32_t next_seq_{0x8000};

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<TaskResultMsg>::SharedPtr result_pub_;
  rclcpp::Publisher<TaskMsg>::SharedPtr new_task_pub_;
  rclcpp::Publisher<ExecStateMsg>::SharedPtr exec_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<TaskMsg>::SharedPtr assigned_sub_;
  rclcpp::Subscription<FireDetectionMsg>::SharedPtr fire_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_{};
  VehicleLocalPosition position_{};
  bool have_status_{false};
  bool have_position_{false};
  double position_rx_{0.0};
  double battery_{-1.0};
  bool landed_{true};
  float yaw_{kNaN};

  AssignedTask assigned_;
  std::map<std::uint32_t, FireObservation> fires_;
  Vec3 velocity_cmd_;
  bool offboard_{true};
  ExecStatus exec_;
  ExecStatus last_exec_;
  double last_exec_pub_{-1e9};

  std::unique_ptr<UavContext> ctx_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
};

}  // namespace uav_px4_bt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_px4_bt::UavBtNode>());
  rclcpp::shutdown();
  return 0;
}
