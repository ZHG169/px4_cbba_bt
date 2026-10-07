// 機器狗（Ghost V60）端的 CBBA 節點：狗的機上電腦跑一個
//
// 和無人機的 cbba_node 共用協定層與骨架（uav_cbba 套件的 cbba_node_base.hpp），機間封包完全一樣；只有機內的介面不同
// （介面依 uav_cbba/doc/image.png）：
//   狗的 BT → CBBA   /v60/robot_state  (swarm_interfaces/RobotState)   位置（map ENU）、電量，2 Hz
//                    /v60/task_result  (swarm_interfaces/TaskResult)   任務結束時一次：
//                                      success = false 時交回競標池、自己不再接；detail 只記 log
//   CBBA → 狗的 BT   /v60/assigned_task (swarm_interfaces/Task)        目前要執行的任務（同無人機）
//
// 參與出價：robot_state 在 state_timeout 內有更新就參與。狗只接 GROUND_INTERVENTION、PATROL，
// 這些任務是無人機確認火情後建立的，所以實際上是「收到無人機傳來的火災地點才出價」。
// 狗的 BT 停掉、robot_state 斷掉時停止參與，手上的任務立刻釋放。
//
// 狗不建立任務（沒有 new_task）：ManualOverride（平板 → BT）只在狗的機內，不會送上網路。
//
// 機號（agent_id）用介面規格的 50。協定上的影響（見 wire_agent_state.hpp）：
//   - AGENT_STATE 的鄰居位元只有 1～8：狗沒有位元。無人機完成任務時不等狗的確認（狗照樣收得到證明）；
//     狗完成任務時照常等無人機的確認。
//   - 時間戳 s 的長度變成 50：每則 AGENT_STATE 多 200 B。
//   - 任務編號依機號交錯，50 會和 uav2 的編號撞（49 = 1 + 8×6）。狗不建立任務，所以不會發生。
//
// 能量模型的預設值是佔位用的，還沒校正（見 doc/ugv_tuning.md）。
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/robot_state.hpp>
#include <swarm_interfaces/msg/task_result.hpp>

#include "uav_cbba/cbba_node_base.hpp"

using RobotStateMsg = swarm_interfaces::msg::RobotState;
using TaskResultMsg = swarm_interfaces::msg::TaskResult;

namespace uav_cbba
{

class UgvCbbaNode : public CbbaNodeBase
{
public:
  UgvCbbaNode()
  : CbbaNodeBase("ugv_cbba_node")
  {
    const int agent_id = declare_parameter<int>("agent_id", std::stoi(envOr("UGV_ID", "50")));
    if (agent_id < 1 || agent_id > 254) {
      throw std::invalid_argument("agent_id must be 1~254");
    }
    if (agent_id > 8) {
      RCLCPP_WARN(get_logger(),
        "agent_id %d 超過 8：AGENT_STATE 沒有它的鄰居位元，無人機完成任務時不會等它的確認", agent_id);
    }
    agent_id_ = static_cast<AgentId>(agent_id);
    const std::string ns = declare_parameter<std::string>("ns", envOr("UGV_NS", "v60"));
    state_timeout_ = declare_parameter<double>("state_timeout", 1.5);

    AgentState s;
    s.id = agent_id_;
    s.type = AgentType::UGV;
    // 能量模型的佔位值（未校正）：步行約 1 m/s；續航、耗電要用狗的實測資料換掉
    s.safety_reserve = 20.0;
    s.energy_per_meter = 0.1;
    s.hover_energy_per_sec = 0.05;
    s.cruise_speed = 1.0;
    const CommonParams p = declareCommon(s, "seq_agent" + std::to_string(agent_id));

    const std::string prefix = ns.empty() ? "" : "/" + ns;
    start(s, p, prefix);

    // 狗的 BT 用什麼 QoS 發都收得到：best effort 的訂閱可以接 reliable 和 best effort 的發布
    state_sub_ = create_subscription<RobotStateMsg>(
      prefix + "/robot_state", rclcpp::QoS(10).best_effort(),
      [this](RobotStateMsg::ConstSharedPtr msg) {onRobotState(*msg);});
    // 任務結果只有一則，掉了就沒有了：用 reliable（狗的 BT 也要用 reliable 發）
    result_sub_ = create_subscription<TaskResultMsg>(
      prefix + "/task_result", rclcpp::QoS(20).reliable(),
      [this](TaskResultMsg::ConstSharedPtr msg) {
        reportResult(msg->task_id, msg->success, msg->detail, wallNow());
      });

    RCLCPP_INFO(get_logger(),
      "agent %d（UGV）：UDP %s:%u（網卡 %s）；話題 %s/{robot_state,task_result,assigned_task}；"
      "robot_state %.1f s 內有更新才參與出價",
      agent_id, p.udp.group.c_str(), p.udp.port,
      p.udp.interface_ip.empty() ? "系統預設" : p.udp.interface_ip.c_str(), prefix.c_str(),
      state_timeout_);
  }

private:
  void beforeStep(double now) override
  {
    const bool fresh = have_state_ && now - last_state_ <= state_timeout_;
    comm_->setStatus(false, false, fresh);
    setParticipating(fresh, now);
  }

  void onRobotState(const RobotStateMsg & msg)
  {
    if (msg.agent_id != agent_id_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "robot_state 的 agent_id 是 %u，不是 %u：略過", msg.agent_id, agent_id_);
      return;
    }
    state_.position = {msg.position.x, msg.position.y, msg.position.z};
    if (msg.battery >= 0.0f) {
      state_.battery = msg.battery;
    }
    if (!have_state_) {
      state_.home = state_.position;   // 第一次收到的位置當返航點
      RCLCPP_INFO(get_logger(), "返航點 map (%.1f, %.1f, %.1f)，電量 %.0f%%",
        state_.home.x, state_.home.y, state_.home.z, state_.battery);
    }
    have_state_ = true;
    last_state_ = wallNow();
  }

  AgentId agent_id_{50};
  double state_timeout_{1.5};
  bool have_state_{false};
  double last_state_{0.0};

  rclcpp::Subscription<RobotStateMsg>::SharedPtr state_sub_;
  rclcpp::Subscription<TaskResultMsg>::SharedPtr result_sub_;
};

}  // namespace uav_cbba

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<uav_cbba::UgvCbbaNode>());
  rclcpp::shutdown();
  return 0;
}
