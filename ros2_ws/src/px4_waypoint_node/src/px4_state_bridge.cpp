// 無人機的轉接節點：PX4 的話題 → 通用 cbba_node 的標準介面 RobotState。每台無人機的機上電腦各跑一個。
//
//   PX4 → 轉接   <px4_ns>/fmu/out/vehicle_local_position_v1   位置（NED、以出生點為原點）
//                <px4_ns>/fmu/out/vehicle_status_v1           arming_state、nav_state
//                <px4_ns>/fmu/out/battery_status_v1           電量
//                <px4_ns>/fmu/out/vehicle_land_detected       是否在地面
//   轉接 → CBBA  <px4_ns>/robot_state (swarm_interfaces/RobotState)   rate_hz
//
// PX4 的位置有在更新（position_timeout 內）就一直送，地面上、未解鎖時也送，讓 AGENT_STATE 的旗標跟著真實狀態變。
// 位置沒在更新（PX4 斷線）就停止送，cbba_node 在 state_timeout（1.5 s）後停止參與出價、釋放任務。
// 「要不要出價」（已解鎖、離地）由 cbba_node 依這裡送的欄位決定。
//
// 飛行狀態：
//   armed     = arming_state == ARMING_STATE_ARMED
//   offboard  = nav_state == NAVIGATION_STATE_OFFBOARD
//   landed    = vehicle_land_detected.landed
//   flight_state_valid = 收到過 vehicle_status 和 vehicle_land_detected，而且 vehicle_status 在
//     status_timeout 內有更新。「收到過」和「現在有效」分開記：逾時後 flight_state_valid = false，
//     armed、offboard、landed 保留最後的值但不算數。vehicle_land_detected 不保證固定頻率，不檢查它是否逾時。
//
// 座標：NED（以出生點為原點）→ map ENU，加出生點。出生點預設和 px4_sitl.sh、task_executor 一樣，
// 沿 y 方向每台間隔 UAV_SPAWN_SPACING 公尺。
//
// 電量：PX4 的 remaining（0～1）× 100；電池沒連上或不知道時送 −1（cbba_node 沿用上一筆）。
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/robot_state.hpp>

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using RobotStateMsg = swarm_interfaces::msg::RobotState;

namespace
{
std::string envOr(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr && *v != '\0' ? v : fallback;
}

// 判斷有沒有在更新用單調時鐘，不受對時、use_sim_time 影響
double steadyNow()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class Px4StateBridge : public rclcpp::Node
{
public:
  Px4StateBridge()
  : Node("px4_state_bridge")
  {
    const int agent_id = declare_parameter<int>("agent_id", std::stoi(envOr("UAV_ID", "1")));
    if (agent_id < 1 || agent_id > 254) {
      throw std::invalid_argument("agent_id must be 1~254");
    }
    agent_id_ = static_cast<std::uint8_t>(agent_id);
    const std::string ns = declare_parameter<std::string>("px4_ns", envOr("UAV_NS", ""));
    position_timeout_ = declare_parameter<double>("position_timeout", 0.5);
    status_timeout_ = declare_parameter<double>("status_timeout", 2.0);
    const double rate_hz = declare_parameter<double>("rate_hz", 10.0);

    const double spacing = std::stod(envOr("UAV_SPAWN_SPACING", "2.0"));
    const auto spawn = declare_parameter<std::vector<double>>(
      "spawn_enu", {0.0, (agent_id - 1) * spacing, 0.0});
    if (spawn.size() != 3) {
      throw std::invalid_argument("spawn_enu must have 3 elements");
    }
    spawn_x_ = spawn[0];
    spawn_y_ = spawn[1];
    spawn_z_ = spawn[2];

    const std::string prefix = ns.empty() ? "" : "/" + ns;
    const std::string fmu = prefix + "/fmu/out";
    const auto qos = rclcpp::SensorDataQoS();   // PX4 的話題是 best effort
    position_sub_ = create_subscription<VehicleLocalPosition>(
      fmu + "/vehicle_local_position_v1", qos,
      [this](VehicleLocalPosition::ConstSharedPtr msg) {onPosition(*msg);});
    status_sub_ = create_subscription<VehicleStatus>(
      fmu + "/vehicle_status_v1", qos,
      [this](VehicleStatus::ConstSharedPtr msg) {
        armed_ = msg->arming_state == VehicleStatus::ARMING_STATE_ARMED;
        offboard_ = msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        have_status_ = true;
        last_status_ = steadyNow();
      });
    land_sub_ = create_subscription<VehicleLandDetected>(
      fmu + "/vehicle_land_detected", qos,
      [this](VehicleLandDetected::ConstSharedPtr msg) {
        landed_ = msg->landed;
        have_land_ = true;
      });
    battery_sub_ = create_subscription<BatteryStatus>(
      fmu + "/battery_status_v1", qos,
      [this](BatteryStatus::ConstSharedPtr msg) {
        battery_ = msg->connected && msg->remaining >= 0.0f ? 100.0f * msg->remaining : -1.0f;
      });

    // cbba_node 用 best effort 訂閱，這裡用 reliable 發也收得到
    state_pub_ = create_publisher<RobotStateMsg>(prefix + "/robot_state", rclcpp::QoS(10));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {publish();});

    RCLCPP_INFO(get_logger(), "uav%d：%s → %s/robot_state（%.0f Hz），出生點 map (%.1f, %.1f, %.1f)",
      agent_id, fmu.c_str(), prefix.c_str(), rate_hz, spawn_x_, spawn_y_, spawn_z_);
  }

private:
  void onPosition(const VehicleLocalPosition & p)
  {
    if (!p.xy_valid || !p.z_valid) {
      return;
    }
    // NED（以出生點為原點）→ map ENU
    x_ = spawn_x_ + p.y;
    y_ = spawn_y_ + p.x;
    z_ = spawn_z_ - p.z;
    have_position_ = true;
    last_position_ = steadyNow();
  }

  void publish()
  {
    const double now = steadyNow();
    const bool fresh = have_position_ && now - last_position_ < position_timeout_;
    const bool valid = have_status_ && have_land_ && now - last_status_ < status_timeout_;
    if (fresh != sending_) {
      sending_ = fresh;
      if (fresh) {
        RCLCPP_INFO(get_logger(), "開始送 robot_state");
      } else {
        RCLCPP_WARN(get_logger(), "PX4 的位置沒有更新，停止送 robot_state（cbba_node 1.5 s 後停止出價）");
      }
    }
    if (valid != valid_) {
      valid_ = valid;
      if (valid) {
        RCLCPP_INFO(get_logger(), "飛行狀態有效");
      } else {
        RCLCPP_WARN(get_logger(), "飛行狀態無效：vehicle_status %s、vehicle_land_detected %s",
          !have_status_ ? "還沒收到" : (now - last_status_ < status_timeout_ ? "正常" : "逾時"),
          have_land_ ? "收到過" : "還沒收到");
      }
    }
    if (!fresh) {
      return;
    }
    RobotStateMsg msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.agent_id = agent_id_;
    msg.position.x = x_;
    msg.position.y = y_;
    msg.position.z = z_;
    msg.battery = battery_;
    msg.flight_state_valid = valid;
    msg.armed = armed_;
    msg.offboard = offboard_;
    msg.landed = landed_;
    state_pub_->publish(msg);
  }

  std::uint8_t agent_id_{1};
  double position_timeout_{0.5};
  double status_timeout_{2.0};
  double spawn_x_{0.0};
  double spawn_y_{0.0};
  double spawn_z_{0.0};

  bool armed_{false};
  bool offboard_{false};
  bool landed_{true};
  bool have_status_{false};
  bool have_land_{false};
  double last_status_{0.0};
  float battery_{-1.0f};
  bool have_position_{false};
  double last_position_{0.0};
  double x_{0.0};
  double y_{0.0};
  double z_{0.0};
  bool sending_{false};
  bool valid_{false};

  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Publisher<RobotStateMsg>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4StateBridge>());
  rclcpp::shutdown();
  return 0;
}
