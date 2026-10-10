// 無人機備用 BT 的節點（BehaviorTree.CPP v4，2026-10-09）。樹在 trees/main_uav_tree.xml，結構照週計畫的 MainUAVTree：
//
//   IsUAVBatteryCritical / ExecuteReturnToLaunch   電量 ≤ min_battery → 回報目前任務失敗、執行狀態故障、PX4 返航
//   TakeOff                                        offboard＋解鎖、爬升到 altitude（週計畫沒有，前台樹要先起飛）
//   IsTaskValidAndUnchanged                        比對 cbba_node 的 assigned_task（task_id＋assignment_version）
//                                                  和執行中的任務：換了 → 回傳 FAILURE 一次（中斷舊的序列）並換成新的
//   FlyToWaypoint                                  CPF 避碰、飛到任務點上方 altitude
//   DetectAprilTagFire / ReportFireEvent           到達、懸停穩定 settle_sec 後觀察到 observe_sec：
//                                                  expected_tag_id 的 confidence ≥ min_confidence
//                                                  → 對 cbba_node 建立 GROUND_INTERVENTION 任務（同一個 tag 只報一次）
//   HoverAndMonitor                                停留到任務的 duration_sec（從到達算起，含觀察的時間）
//   ReportTaskResult                               task_result（帶 assignment_version）
//   LandOrLoiter                                   沒有任務：原地懸停（目前只做 loiter；降落後 cbba_node 會停止出價）；
//                                                  執行任務中的鄰機靠近時讓路（不拉回待命點，被推開後在新位置待命）
//
// 所有移動都經過 UavContext::moveTo（CPF），懸停時也會閃避飛過來的鄰機。
// 和週計畫 XML 不同的地方：
//   - 加 TakeOff；頂層用 ReactiveFallback，電量檢查每個 tick 都做
//   - HoverAndMonitor 不是無限期：任務的 duration_sec 到了就回報完成，否則 cbba_node 一直認為它在執行
//   - FlyToWaypoint 逾時（距離 / 速度 × 2 + fly_timeout_margin）回報失敗（交回競標池）
#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <utility>

#include <behaviortree_cpp/bt_factory.h>

#include "uav_px4_bt/cpf.hpp"
#include "uav_px4_bt/uav_platform.hpp"

namespace uav_px4_bt
{

struct UavContextConfig
{
  CpfParams cpf;
  double reach_tolerance{0.5};        // 到達任務點的距離
  double takeoff_tolerance{0.3};      // 爬升到 altitude 的誤差
  double fly_timeout_margin{30.0};
  double fire_task_value{100.0};      // ReportFireEvent 建立的地面任務（介面規格的火警預設）
  double fire_task_duration{20.0};
  double fire_task_deadline{120.0};
};

// 樹裡共用的狀態，放在 blackboard 的 "ctx"（UavContext*）
class UavContext
{
public:
  UavContext(UavPlatform & platform, const UavContextConfig & config)
  : platform(platform), config(config) {}

  // 往 goal 飛（CPF 避碰），回傳現在離 goal 的距離
  double moveTo(const Vec3 & goal);
  void setExec(ExecMode mode, double remaining, const AssignedTask * active);
  bool reported(const AssignedTask & t) const {return reported_.count({t.task_id, t.version}) > 0;}
  void markReported(const AssignedTask & t) {reported_.insert({t.task_id, t.version});}

  UavPlatform & platform;
  UavContextConfig config;

  bool airborne{false};               // TakeOff 完成（解鎖狀態不見了就重來）
  bool battery_critical{false};       // 一旦觸發就不再恢復
  Vec3 waypoint;                      // 目前任務的航點
  double arrival{-1.0};               // 到達目前任務的時刻；< 0 還沒到
  std::set<std::uint32_t> reported_fire_tags;

private:
  std::set<std::pair<std::uint32_t, std::uint64_t>> reported_;
  double encounter_min_{-1.0};       // 這次接近的最近距離；< 0 沒有在接近
};

// 註冊全部節點（blackboard 要先放 "ctx"）
void registerUavNodes(BT::BehaviorTreeFactory & factory);

}  // namespace uav_px4_bt
