// 單機起飛並懸停（PX4 offboard），每台無人機在自己的 uav 容器裡各跑一個，彼此不依賴。
//
// 流程：
//   WAIT_READY  等 vehicle_status 與 vehicle_local_position：飛行前檢查通過、位置估計有效，
//               記下目前位置與航向當作懸停點
//   STREAM      先送 1 秒 setpoint（PX4 要先收到串流才允許切 offboard）
//   ENGAGE      送 DO_SET_MODE(offboard) 與 ARM，每秒重送直到 nav_state=OFFBOARD 且已解鎖
//   CLIMB       爬升到懸停點正上方 altitude 公尺（NED，z = 起始 z - altitude）
//   HOVER       到達後持續送同一個 setpoint，原地懸停
//
// 注意：offboard 需要持續的 setpoint 串流，節點停止後 PX4 會觸發 failsafe（不是正常降落流程）。
// 這個節點刻意不做降落。
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

std::string defaultNamespace()
{
  const char * ns = std::getenv("UAV_NS");  // uav 容器裡由 docker-compose 設定，例如 uav1
  return ns ? ns : "";
}
}  // namespace

class TakeoffHover : public rclcpp::Node
{
public:
  TakeoffHover()
  : Node("takeoff_hover")
  {
    px4_ns_ = declare_parameter<std::string>("px4_ns", defaultNamespace());
    altitude_ = declare_parameter<double>("altitude", 5.0);           // 起飛高度（公尺）
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);            // setpoint 頻率
    reach_tol_ = declare_parameter<double>("reach_tolerance", 0.3);   // 到達高度的容許誤差
    auto_start_ = declare_parameter<bool>("auto_start", true);

    const std::string prefix = px4_ns_.empty() ? "/fmu" : "/" + px4_ns_ + "/fmu";
    // PX4 發出的話題是 best effort，要用 sensor data QoS 才收得到
    const auto px4_qos = rclcpp::SensorDataQoS();

    offboard_pub_ = create_publisher<OffboardControlMode>(prefix + "/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>(prefix + "/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>(prefix + "/in/vehicle_command", 10);

    // PX4 v1.16 起這兩個話題名稱帶版本後綴 _v1
    status_sub_ = create_subscription<VehicleStatus>(
      prefix + "/out/vehicle_status_v1", px4_qos,
      [this](VehicleStatus::ConstSharedPtr msg) {status_ = *msg; have_status_ = true;});
    position_sub_ = create_subscription<VehicleLocalPosition>(
      prefix + "/out/vehicle_local_position_v1", px4_qos,
      [this](VehicleLocalPosition::ConstSharedPtr msg) {position_ = *msg; have_position_ = true;});

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz_), [this]() {step();});

    RCLCPP_INFO(get_logger(), "PX4 話題前綴 %s，起飛高度 %.1f m，setpoint %.0f Hz",
      prefix.c_str(), altitude_, rate_hz_);
  }

private:
  enum class Phase { WAIT_READY, STREAM, ENGAGE, CLIMB, HOVER };

  static const char * name(Phase p)
  {
    switch (p) {
      case Phase::WAIT_READY: return "WAIT_READY";
      case Phase::STREAM: return "STREAM";
      case Phase::ENGAGE: return "ENGAGE";
      case Phase::CLIMB: return "CLIMB";
      case Phase::HOVER: return "HOVER";
    }
    return "?";
  }

  void enter(Phase next)
  {
    RCLCPP_INFO(get_logger(), "%s -> %s", name(phase_), name(next));
    phase_ = next;
    phase_ticks_ = 0;
  }

  bool ready() const
  {
    return have_status_ && have_position_ && status_.pre_flight_checks_pass &&
           position_.xy_valid && position_.z_valid;
  }

  bool engaged() const
  {
    return status_.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
           status_.arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  void step()
  {
    ++phase_ticks_;
    switch (phase_) {
      case Phase::WAIT_READY:
        if (auto_start_ && ready()) {
          hold_x_ = position_.x;
          hold_y_ = position_.y;
          hold_z_ = position_.z - static_cast<float>(altitude_);
          hold_yaw_ = position_.heading;
          RCLCPP_INFO(get_logger(), "system_id %d，懸停點 NED (%.2f, %.2f, %.2f)，航向 %.2f rad",
            status_.system_id, hold_x_, hold_y_, hold_z_, hold_yaw_);
          enter(Phase::STREAM);
        } else if (phase_ticks_ % static_cast<int>(rate_hz_ * 5) == 0) {
          RCLCPP_INFO(get_logger(), "等待 PX4：status %d position %d preflight %d",
            have_status_, have_position_, have_status_ && status_.pre_flight_checks_pass);
        }
        return;  // 還沒有懸停點，不送 setpoint

      case Phase::STREAM:
        if (phase_ticks_ >= static_cast<int>(rate_hz_)) {
          enter(Phase::ENGAGE);
        }
        break;

      case Phase::ENGAGE:
        if (engaged()) {
          engage_time_ = now();
          enter(Phase::CLIMB);
        } else if (phase_ticks_ % static_cast<int>(rate_hz_) == 1) {
          // 先切 offboard 再解鎖；每秒重送一次直到兩者都成立
          sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);  // custom mode, OFFBOARD
          sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        break;

      case Phase::CLIMB:
        if (std::fabs(position_.z - hold_z_) < reach_tol_) {
          RCLCPP_INFO(get_logger(), "到達 %.1f m，用時 %.1f s，開始懸停",
            altitude_, (now() - engage_time_).seconds());
          enter(Phase::HOVER);
        }
        break;

      case Phase::HOVER:
        if (phase_ticks_ % static_cast<int>(rate_hz_ * 5) == 0) {
          const double dx = position_.x - hold_x_;
          const double dy = position_.y - hold_y_;
          RCLCPP_INFO(get_logger(), "懸停中：高度 %.2f m，水平偏移 %.2f m，offboard %d",
            hold_z_ + altitude_ - position_.z, std::hypot(dx, dy), engaged());
        }
        break;
    }
    publishSetpoint();
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
    sp.position = {hold_x_, hold_y_, hold_z_};
    sp.velocity = {kNaN, kNaN, kNaN};
    sp.acceleration = {kNaN, kNaN, kNaN};
    sp.jerk = {kNaN, kNaN, kNaN};
    sp.yaw = hold_yaw_;
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
  double rate_hz_{10.0};
  double reach_tol_{0.3};
  bool auto_start_{true};

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  VehicleStatus status_{};
  VehicleLocalPosition position_{};
  bool have_status_{false};
  bool have_position_{false};

  Phase phase_{Phase::WAIT_READY};
  int phase_ticks_{0};
  rclcpp::Time engage_time_;
  float hold_x_{0.0f};
  float hold_y_{0.0f};
  float hold_z_{0.0f};
  float hold_yaw_{0.0f};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffHover>());
  rclcpp::shutdown();
  return 0;
}
