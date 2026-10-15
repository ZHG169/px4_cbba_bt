// UavPlatform 的 PX4＋ROS 實作（2026-10-12 從 uav_bt_node.cpp 抽出）：備用 BT（uav_bt_node）和照機器狗端骨架的
// uav_task_manager_node 共用，PX4 介面、運動控制、位置廣播只有一份。
//
//   PX4   <px4_ns>/fmu/out/{vehicle_status_v1, vehicle_local_position_v1, battery_status_v1, vehicle_land_detected}
//         <px4_ns>/fmu/in/{offboard_control_mode, trajectory_setpoint（位置＋速度前饋）, vehicle_command}
//   CBBA  <ns>/task_result（TaskResult）、<ns>/new_task（火警）、<ns>/task_closed（結束的任務，火警監控用）、
//         <ns>/task_announced（全隊已知的任務，火點去重複用）；
//         <ns>/current_task 由各節點自己訂（處理方式不同）
//   偵測  <ns>/fire_detection（apriltag_fire_detector）
//   機間  飛行層的位置廣播（UDP multicast，beacon_port，10 Hz），給 CBF 避碰（格式見 flight_beacon.hpp）
//
// 每個 tick：beginTick()（收位置廣播、這個 tick 還沒有命令）→ 樹或節點下命令 → endTick()（沒有命令時保持上一次的位置；
// 送 setpoint、位置廣播）。座標：BT 用 map ENU；PX4 local 是 NED、以出生點為原點（UAV_SPAWN_SPACING 沿 y）。
// 航向固定在第一次收到位置時的方向（下視相機的影像方向不會一直轉）。
// 建立的任務 task_id = agent_id × 65536 + 流水號；流水號從 0x8000 + (Unix 秒 mod 0x8000) 開始，重開機不會和之前的重複。
// 注意：offboard 需要持續的 setpoint，停掉節點 PX4 會觸發 failsafe。
#pragma once

#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <memory>
#include <string>
#include <vector>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/fire_detection.hpp>
#include <swarm_interfaces/msg/task.hpp>
#include <swarm_interfaces/msg/task_result.hpp>

#include "cbba_core/udp_link.hpp"
#include "uav_px4_bt/bt_nodes.hpp"
#include "uav_px4_bt/flight_beacon.hpp"
#include "uav_px4_bt/uav_platform.hpp"

namespace uav_px4_bt
{

class Px4Platform : public UavPlatform
{
public:
  // 在 node 上宣告參數（agent_id、px4_ns、ns、altitude、position_timeout、spawn_enu、udp_group、beacon_port、
  // mesh_ip、beacon_hz），建立 PX4、cbba_node（task_result、new_task）、偵測、位置廣播的介面
  explicit Px4Platform(rclcpp::Node & node);

  // CBF、運動平滑、到達、火警的參數（兩個節點相同）
  static UavContextConfig declareConfig(rclcpp::Node & node);

  void beginTick();
  void endTick();

  std::uint16_t agentId() const {return agent_id_;}
  double altitude() const {return altitude_;}
  const std::string & ns() const {return ns_;}
  const std::string & fmu() const {return fmu_;}
  Vec3 spawn() const {return spawn_;}
  std::string describe() const;     // 啟動 log 用

  // UavPlatform
  double timeNow() const override;
  VehicleState vehicle() const override;
  std::vector<Neighbor> neighbors() const override;
  std::vector<FireObservation> fireObservations() const override;
  void setVelocity(const Vec3 & v) override;
  void setSetpoint(const Vec3 & position, const Vec3 & velocity) override;
  void requestOffboardAndArm() override;
  void requestReturnToLaunch() override;
  void stopOffboard() override {offboard_ = false;}
  void reportResult(const AssignedTask & task, bool success, const std::string & detail) override;
  std::uint32_t createTask(std::uint8_t type, const Vec3 & p, double value, double duration,
    double deadline) override;
  void setExecStatus(const ExecStatus & s) override {exec_ = s;}
  bool taskClosed(std::uint32_t task_id) const override {return closed_.count(task_id) > 0;}
  std::vector<KnownGroundTask> groundTasks() const override {return ground_;}
  void log(const std::string & text) override;

  // task_result（失敗分類用 reason，見 cpp）
  void publishResult(std::uint32_t task_id, bool success, const std::string & detail, std::uint8_t reason);

private:
  void publishSetpoint();
  void sendBeacon();
  void sendCommand(std::uint32_t command, float param1 = 0.0f, float param2 = 0.0f);

  rclcpp::Node & node_;
  std::uint16_t agent_id_{1};
  std::string ns_;
  std::string fmu_;
  double altitude_{5.0};
  double position_timeout_{0.5};
  double neighbor_timeout_{1.0};
  Vec3 spawn_;

  cbba_core::UdpConfig udp_;
  std::unique_ptr<cbba_core::UdpLink> link_;
  std::unique_ptr<NeighborTable> table_;
  std::uint64_t session_{0};
  std::uint32_t beacon_seq_{0};
  double beacon_period_{0.1};
  double last_beacon_{0.0};
  std::uint32_t next_seq_{0x8000};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<swarm_interfaces::msg::TaskResult>::SharedPtr result_pub_;
  rclcpp::Publisher<swarm_interfaces::msg::Task>::SharedPtr new_task_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<swarm_interfaces::msg::FireDetection>::SharedPtr fire_sub_;
  rclcpp::Subscription<swarm_interfaces::msg::Task>::SharedPtr closed_sub_;
  std::set<std::uint32_t> closed_;
  rclcpp::Subscription<swarm_interfaces::msg::Task>::SharedPtr announced_sub_;
  std::vector<KnownGroundTask> ground_;

  px4_msgs::msg::VehicleStatus status_{};
  px4_msgs::msg::VehicleLocalPosition position_{};
  bool have_status_{false};
  bool have_position_{false};
  double position_rx_{0.0};
  double battery_{-1.0};
  bool landed_{true};
  float yaw_{std::numeric_limits<float>::quiet_NaN()};

  std::map<std::uint32_t, FireObservation> fires_;
  Vec3 velocity_cmd_;
  Vec3 position_cmd_;
  bool has_position_cmd_{false};
  bool commanded_{false};
  bool offboard_{true};
  ExecStatus exec_;
};

}  // namespace uav_px4_bt
