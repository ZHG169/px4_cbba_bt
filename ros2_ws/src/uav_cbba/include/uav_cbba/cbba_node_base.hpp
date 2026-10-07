// CBBA 節點的共用骨架（無人機 cbba_node、機器狗套件 ugv_cbba 的 ugv_cbba_node 都繼承它）
//
// 負責和載具無關的部分：
//   - 宣告兩邊共用的 ROS 參數（能量模型、出價、協定、UDP、seq 紀錄、交給 BT）
//   - 每 tick_ms：收 UDP → 協定層 → 送 UDP
//   - 交給 BT：目前排第一的任務連續 assign_hold 秒沒變才發布 <ns>/assigned_task（swarm_interfaces/Task）
//   - seq 紀錄（見 cbba_comm.hpp）、log（路徑、任務結束、收斂、編號衝突）
// 子類別負責：自己的感測輸入（位置、電量）、參與出價的條件、BT 的任務結果怎麼接。
#pragma once

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/task.hpp>

#include "uav_cbba/cbba_comm.hpp"
#include "uav_cbba/udp_link.hpp"

namespace uav_cbba
{

std::string envOr(const char * name, const std::string & fallback);
double wallNow();   // 系統時鐘（秒），各機對時
builtin_interfaces::msg::Time toRosTime(double t);
double fromRosTime(const builtin_interfaces::msg::Time & t);
Vec3 toVec(const std::vector<double> & v, const char * name);
swarm_interfaces::msg::Task toMsg(const Task & t);

class CbbaNodeBase : public rclcpp::Node
{
public:
  using TaskMsg = swarm_interfaces::msg::Task;

  explicit CbbaNodeBase(const std::string & name);

protected:
  struct CommonParams
  {
    ScoringParams scoring;
    CommConfig comm;
    UdpConfig udp;
    double assign_hold{0.6};
    int tick_ms{10};
    std::string seq_file;
  };

  // 宣告兩邊共用的參數；能量模型的預設值取自 s（子類別先填好自己的預設）
  CommonParams declareCommon(AgentState & s, const std::string & seq_name);

  // 建立協定層與 UDP，開始 tick。prefix 是話題前綴（例如 /uav1、/v60）
  void start(const AgentState & s, const CommonParams & p, const std::string & prefix);

  // 每個 tick 一開始呼叫：子類別更新 state_、comm_->setStatus()、setParticipating()
  virtual void beforeStep(double now) = 0;

  void setParticipating(bool participate, double now);   // 有變才送、並印 log
  // BT 回報結果；detail 是失敗原因（只記 log，協定裡沒有地方放）
  void reportResult(TaskId task_id, bool success, const std::string & detail, double now);

  AgentState state_;
  std::unique_ptr<CbbaComm> comm_;

private:
  void setupSeqStore(const std::string & path);
  void step();
  void updateAssigned(double now);
  void publishAssigned(const std::optional<Task> & task, double now);
  void report(double now);

  std::unique_ptr<UdpLink> udp_;
  double assign_hold_{0.6};

  TaskId candidate_{0};
  double candidate_since_{0.0};
  TaskId published_{0};
  std::vector<TaskId> last_path_;
  bool last_converged_{false};
  std::map<TaskId, double> reported_at_;   // 自己回報完成的時刻
  std::set<TaskId> closed_;                // 已經印過結束的任務
  int last_conflicts_{0};

  rclcpp::Publisher<TaskMsg>::SharedPtr assigned_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uav_cbba
