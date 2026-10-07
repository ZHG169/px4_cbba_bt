// 無人機端的 CBBA 節點：每台無人機的機上電腦各跑一個
//
// 機間（mesh）：UDP multicast，封包依「機間通訊封包規格」2026-10-06（見 cbba_comm.hpp）
// 機內（DDS）：
//   PX4 → CBBA   /uavN/fmu/out/vehicle_local_position_v1   位置（NED、以出生點為原點 → map ENU）
//                /uavN/fmu/out/vehicle_status_v1           armed、offboard
//                /uavN/fmu/out/battery_status_v1           電量
//                /uavN/fmu/out/vehicle_land_detected       是否在空中（參與出價的條件）
//   BT → CBBA    /uavN/new_task     (swarm_interfaces/Task)        本機發現的新任務
//                /uavN/task_result  (swarm_interfaces/Task)        任務結束時一次：
//                                   status = DONE 完成；CANCELLED 失敗，交回競標池、自己不再接
//   CBBA → BT    /uavN/assigned_task (swarm_interfaces/Task)       目前要執行的任務（見 cbba_node_base.hpp）
//
// 巡檢任務（AIR_RECON）的 z 是無人機巡檢的高度（map ENU）。BT 常給地面的座標（z = 0），
// 低於 recon_altitude 時改成 recon_altitude，否則每段距離都多算到地面的高度差，耗電也估錯。
// 要和執行者（task_executor 的 altitude）一致。地面處置任務（給機器狗）不改。
//
// 參與出價（participate）：airborne = 解鎖且離地（預設）、armed = 解鎖、always = 一律參與。
// use_px4:=false 時不訂閱 PX4，位置與電量用參數 initial_position、battery，一律參與（測試協商用）。
//
// seq 紀錄（seq_file，預設 ~/.cbba/seq_uavN）：見 cbba_node_base.hpp。
#include <memory>
#include <string>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include "uav_cbba/cbba_node_base.hpp"

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace uav_cbba
{

class CbbaNode : public CbbaNodeBase
{
public:
  CbbaNode()
  : CbbaNodeBase("cbba_node")
  {
    const int agent_id = declare_parameter<int>("agent_id", std::stoi(envOr("UAV_ID", "1")));
    if (agent_id < 1 || agent_id > 8) {
      throw std::invalid_argument("agent_id must be 1~8 (AGENT_STATE neighbor bits hold 8 agents)");
    }
    const std::string ns = declare_parameter<std::string>("px4_ns", envOr("UAV_NS", ""));
    use_px4_ = declare_parameter<bool>("use_px4", true);
    participate_ = declare_parameter<std::string>("participate", "airborne");
    if (participate_ != "airborne" && participate_ != "armed" && participate_ != "always") {
      throw std::invalid_argument("participate must be airborne, armed or always");
    }
    if (!use_px4_) {
      participate_ = "always";
    }
    recon_altitude_ = declare_parameter<double>("recon_altitude", 5.0);

    // 出生點（map ENU）：PX4 的 local position 以出生點為原點。預設和 px4_sitl.sh 一樣沿 y 排開
    const double spacing = std::stod(envOr("UAV_SPAWN_SPACING", "2.0"));
    spawn_ = toVec(declare_parameter<std::vector<double>>(
        "spawn_enu", {0.0, (agent_id - 1) * spacing, 0.0}), "spawn_enu");

    AgentState s;
    s.id = static_cast<AgentId>(agent_id);
    s.type = AgentType::UAV;
    s.position = toVec(declare_parameter<std::vector<double>>(
        "initial_position", {spawn_.x, spawn_.y, spawn_.z}), "initial_position");
    s.home = s.position;
    s.battery = declare_parameter<double>("battery", 100.0);
    s.cruise_speed = 5.0;   // PX4 多旋翼的 MPC_XY_CRUISE
    const CommonParams p = declareCommon(s, "seq_uav" + std::to_string(agent_id));

    const std::string prefix = ns.empty() ? "" : "/" + ns;
    start(s, p, prefix);

    // ---------- BT ----------
    new_task_sub_ = create_subscription<TaskMsg>(
      prefix + "/new_task", rclcpp::QoS(20).reliable(),
      [this](TaskMsg::ConstSharedPtr msg) {onNewTask(*msg);});
    result_sub_ = create_subscription<TaskMsg>(
      prefix + "/task_result", rclcpp::QoS(20).reliable(),
      [this](TaskMsg::ConstSharedPtr msg) {onTaskResult(*msg);});

    // ---------- PX4 ----------
    if (use_px4_) {
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
        });
      land_sub_ = create_subscription<VehicleLandDetected>(
        fmu + "/vehicle_land_detected", qos,
        [this](VehicleLandDetected::ConstSharedPtr msg) {landed_ = msg->landed;});
      battery_sub_ = create_subscription<BatteryStatus>(
        fmu + "/battery_status_v1", qos,
        [this](BatteryStatus::ConstSharedPtr msg) {
          if (msg->connected && msg->remaining >= 0.0f) {
            state_.battery = 100.0 * msg->remaining;
          }
        });
    }

    RCLCPP_INFO(get_logger(),
      "uav%d：UDP %s:%u（網卡 %s）；話題 %s/{new_task,task_result,assigned_task}；%s；參與出價：%s",
      agent_id, p.udp.group.c_str(), p.udp.port,
      p.udp.interface_ip.empty() ? "系統預設" : p.udp.interface_ip.c_str(), prefix.c_str(),
      use_px4_ ? "位置與電量來自 PX4" : "位置與電量來自參數", participate_.c_str());
  }

private:
  void beforeStep(double now) override
  {
    bool participate = true;
    if (use_px4_) {
      comm_->setStatus(armed_, offboard_, now - last_position_ < 0.5);
      participate = participate_ == "always" || (participate_ == "armed" && armed_) ||
        (participate_ == "airborne" && armed_ && !landed_);
    } else {
      comm_->setStatus(false, false, true);
    }
    setParticipating(participate, now);
  }

  void onPosition(const VehicleLocalPosition & p)
  {
    if (!p.xy_valid || !p.z_valid) {
      return;
    }
    // NED（以出生點為原點）→ map ENU
    state_.position = {spawn_.x + p.y, spawn_.y + p.x, spawn_.z - p.z};
    if (!have_home_) {
      state_.home = state_.position;   // 第一次有效位置當返航點
      have_home_ = true;
      RCLCPP_INFO(get_logger(), "返航點 map (%.1f, %.1f, %.1f)",
        state_.home.x, state_.home.y, state_.home.z);
    }
    last_position_ = wallNow();
  }

  void onNewTask(const TaskMsg & msg)
  {
    const double now = wallNow();
    Task t;
    t.id = msg.task_id;
    t.type = static_cast<TaskType>(msg.type);
    t.position = {msg.position.x, msg.position.y, msg.position.z};
    if (t.type == TaskType::AIR_RECON && t.position.z < recon_altitude_) {
      t.position.z = recon_altitude_;   // 地面座標 → 巡檢高度
    }
    t.created = msg.created.sec == 0 && msg.created.nanosec == 0 ? now : fromRosTime(msg.created);
    t.deadline_sec = msg.deadline_sec > 0.0f ? msg.deadline_sec : 300.0;
    t.value = msg.value;
    t.duration_sec = msg.duration_sec;
    t.status_stamp = t.created;
    if (comm_->addLocalTask(t, now)) {
      RCLCPP_INFO(get_logger(), "新任務 %s (%.1f, %.1f, %.1f) value %.0f，期限 %.0f s",
        taskName(t.id).c_str(), t.position.x, t.position.y, t.position.z, t.value, t.deadline_sec);
    } else {
      RCLCPP_WARN(get_logger(),
        "新任務 %s 沒有加入：task_id 要是 %u × 65536 + 流水號、不能重複，或本機的任務編號已用完",
        taskName(msg.task_id).c_str(), comm_->id());
    }
  }

  // status = DONE：完成；CANCELLED：失敗，交回競標池
  void onTaskResult(const TaskMsg & msg)
  {
    if (msg.status != TaskMsg::DONE && msg.status != TaskMsg::CANCELLED) {
      RCLCPP_WARN(get_logger(), "任務 %s 的結果 status = %u，要是 DONE(1) 或 CANCELLED(2)",
        taskName(msg.task_id).c_str(), msg.status);
      return;
    }
    reportResult(msg.task_id, msg.status == TaskMsg::DONE, "", wallNow());
  }

  bool use_px4_{true};
  std::string participate_{"airborne"};
  double recon_altitude_{5.0};
  Vec3 spawn_{};

  bool armed_{false};
  bool offboard_{false};
  bool landed_{true};
  bool have_home_{false};
  double last_position_{0.0};

  rclcpp::Subscription<TaskMsg>::SharedPtr new_task_sub_;
  rclcpp::Subscription<TaskMsg>::SharedPtr result_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
};

}  // namespace uav_cbba

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_cbba::CbbaNode>());
  rclcpp::shutdown();
  return 0;
}
