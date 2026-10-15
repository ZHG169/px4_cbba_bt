// 無人機的備用 BT 節點（2026-10-09）：取代 px4_waypoint_node/task_executor，每台無人機的機上電腦各跑一個。
//
//   PX4   <px4_ns>/fmu/out/{vehicle_status_v1, vehicle_local_position_v1, battery_status_v1, vehicle_land_detected}
//         <px4_ns>/fmu/in/{offboard_control_mode, trajectory_setpoint（速度）, vehicle_command}
//   CBBA  <ns>/current_task → BT；BT → <ns>/task_result（TaskResult：success、detail、reason）、<ns>/new_task（火警）
//         2026-10-12 照機器狗端的介面：不帶指派版本、不發 exec_state（任務換了只看 task_id）
//   偵測  <ns>/fire_detection（apriltag_fire_detector）
//   機間  飛行層的位置廣播（UDP multicast，beacon_port，10 Hz），給 CBF 避碰（格式見 flight_beacon.hpp）
//
// 每 1/tick_hz 秒：把 current_task 寫進 blackboard（current_uav_task）→ tick 一次樹 → 把樹設定的速度送給 PX4。
// 樹的節點不碰 ROS，透過 UavPlatform（Px4Platform，px4_platform.hpp，和 uav_task_manager_node 共用）讀狀態、下指令。
//
// 座標：BT 用 map ENU；PX4 local 是 NED、以出生點為原點。出生點預設和 px4_sitl.sh、px4_state_bridge 一樣，
// 沿 y 方向每台間隔 UAV_SPAWN_SPACING 公尺。航向固定在起飛時的方向（下視相機的影像方向不會一直轉）。
//
// 建立的火警任務 task_id = agent_id × 65536 + 流水號；流水號從 0x8000 + (Unix 秒 mod 0x8000) 開始遞增，
// BT 重開機後不會和之前建立過的重複（除非剛好隔 9.1 小時的整數倍），也不會和 cbba_task.sh 手動用的小流水號撞。
//
// 注意：offboard 需要持續的 setpoint，停掉這個節點 PX4 會觸發 failsafe。
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/task.hpp>

#include "uav_px4_bt/bt_nodes.hpp"
#include "uav_px4_bt/px4_platform.hpp"

using TaskMsg = swarm_interfaces::msg::Task;

namespace uav_px4_bt
{

class UavBtNode : public rclcpp::Node
{
public:
  UavBtNode()
  : Node("uav_bt_node")
  {
    const double tick_hz = declare_parameter<double>("tick_hz", 20.0);
    const std::string tree_file = declare_parameter<std::string>("tree_file",
        ament_index_cpp::get_package_share_directory("uav_px4_bt") + "/trees/main_uav_tree.xml");
    const UavContextConfig cfg = Px4Platform::declareConfig(*this);
    platform_ = std::make_unique<Px4Platform>(*this);

    const std::string prefix = "/" + platform_->ns();
    assigned_sub_ = create_subscription<TaskMsg>(prefix + "/current_task",
        rclcpp::QoS(1).reliable().transient_local(),
        [this](TaskMsg::ConstSharedPtr m) {onAssigned(*m);});

    // ---------- 樹 ----------
    ctx_ = std::make_unique<UavContext>(*platform_, cfg);
    registerUavNodes(factory_);
    blackboard_ = BT::Blackboard::create();
    blackboard_->set<UavContext *>("ctx", ctx_.get());
    blackboard_->set<AssignedTask>("current_uav_task", AssignedTask{});
    blackboard_->set<AssignedTask>("active_uav_task", AssignedTask{});
    blackboard_->set<double>("altitude", platform_->altitude());
    tree_ = factory_.createTreeFromFile(tree_file, blackboard_);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tick_hz), [this]() {step();});
    RCLCPP_INFO(get_logger(), "uav%u：樹 %s，速度 %.1f m/s；%s", platform_->agentId(), tree_file.c_str(),
      cfg.cbf.max_speed, platform_->describe().c_str());
  }

private:
  void onAssigned(const TaskMsg & m)
  {
    AssignedTask t;
    if (m.task_id != 0) {
      t.task_id = m.task_id;
      t.type = m.type;
      t.position = {m.position.x, m.position.y, m.position.z};
      t.duration = std::max(0.0f, m.duration_sec);
    }
    assigned_ = t;
  }

  void step()
  {
    platform_->beginTick();
    blackboard_->set<AssignedTask>("current_uav_task", assigned_);
    tree_.tickOnce();
    platform_->endTick();
  }

  std::unique_ptr<Px4Platform> platform_;
  rclcpp::Subscription<TaskMsg>::SharedPtr assigned_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  AssignedTask assigned_;
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
