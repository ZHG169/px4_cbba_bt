// 無人機備用 BT 的節點（BehaviorTree.CPP v4，2026-10-09）。樹在 trees/main_uav_tree.xml，結構照週計畫的 MainUAVTree：
//
//   IsUAVBatteryCritical / ExecuteReturnToLaunch   電量 ≤ min_battery → 回報目前任務失敗、執行狀態故障、PX4 返航
//   TakeOff                                        offboard＋解鎖、爬升到 altitude（週計畫沒有，前台樹要先起飛）
//   IsTaskValidAndUnchanged                        比對 cbba_node 的 current_task（task_id；2026-10-12 起介面不帶指派版本）
//                                                  和執行中的任務：換了 → 回傳 FAILURE 一次（中斷舊的序列）並換成新的
//   FlyToWaypoint                                  CBF 避碰、飛到任務點上方 altitude；飛行途中相機持續偵測（watch_fire_tag，
//                                                  2026-10-12）：路過看到新的火點 → 建立地面處置任務（給 V60 競標），
//                                                  繼續飛自己的任務。fire_divert_on_sight = true 時另外建立火點監控任務
//                                                  （AIR_RECON，停留 fire_monitor_sec）、手上的任務回報 FIRE_FOUND 交回 CBBA，
//                                                  自己改去火點懸停（週計畫沒有，預設關閉）
//   DetectAprilTagFire / ReportFireEvent           到達、懸停穩定 settle_sec 後觀察到 observe_sec：
//                                                  expected_tag_id 的 confidence ≥ min_confidence
//                                                  → 對 cbba_node 建立 GROUND_INTERVENTION 任務（同一個火點只報一次：
//                                                  離已回報的火點 fire_dedup_radius 以內不再報；2026-10-12 前只看
//                                                  tag 編號，火點都貼 tag 101，第二個火點就不會報）
//   HoverAndMonitor                                停留到任務的 duration_sec（從到達算起，含觀察的時間）
//   ReportTaskResult                               task_result（success、detail、reason）
//   LandOrLoiter                                   沒有任務：原地懸停（目前只做 loiter；降落後 cbba_node 會停止出價）；
//                                                  執行任務中的鄰機靠近時讓路（不拉回待命點，被推開後在新位置待命）
//
// 所有移動都經過 UavContext::moveTo：避碰（avoidance.hpp：預測衝突 → 路權 → VO 繞行 → CBF 保底，2026-10-24）→
// 運動平滑層（加速度上限、剎車曲線）→
// 位置＋速度前饋的 setpoint 給 PX4（2026-10-12 前只送速度，SITL 很晃）。懸停時也會閃避飛過來的鄰機。
// 和週計畫 XML 不同的地方：
//   - 加 TakeOff；頂層用 ReactiveFallback，電量檢查每個 tick 都做
//   - HoverAndMonitor 不是無限期：任務的 duration_sec 到了就回報完成，否則 cbba_node 一直認為它在執行
//   - FlyToWaypoint 逾時（距離 / 速度 × 2 + fly_timeout_margin）回報失敗（交回競標池）；
//     fly_timeout_abort = false 時只印警告、繼續飛
#pragma once

#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "uav_px4_bt/avoidance.hpp"
#include "uav_px4_bt/cbf.hpp"
#include "uav_px4_bt/motion.hpp"
#include "uav_px4_bt/uav_platform.hpp"

namespace uav_px4_bt
{

struct UavContextConfig
{
  CbfParams cbf;
  MotionParams motion;
  double reach_tolerance{0.5};        // 到達任務點的距離
  double takeoff_tolerance{0.3};      // 爬升到 altitude 的誤差
  double fly_timeout_margin{30.0};
  bool fly_timeout_abort{true};       // false：逾時只印警告、繼續飛（調 CBF 時用；2026-10-14）
  double fire_task_value{100.0};      // ReportFireEvent 建立的地面任務（介面規格的火警預設）
  double fire_task_duration{20.0};
  double fire_task_deadline{120.0};
  double fire_dedup_radius{3.0};      // 同一個火點：離已回報的火點這麼近（公尺，水平）就不再建立任務
  bool fire_divert_on_sight{false};   // 路過發現火警時，自己放下手上的任務、改去火點懸停監控（預設不要）
  double fire_monitor_value{100.0};   // 改去懸停時建立的火點監控任務（AIR_RECON）：價值、在火點上空停留多久
  double fire_monitor_sec{20.0};
};

// 樹裡共用的狀態，放在 blackboard 的 "ctx"（UavContext*）
class UavContext
{
public:
  UavContext(UavPlatform & platform, const UavContextConfig & config)
  : platform(platform), config(config), motion_(config.motion)
  {
    // 二階 CBF 的加速度上限＝平滑層的（不一樣時平滑層會改掉 CBF 的結果）
    this->config.cbf.accel_max = config.motion.max_accel;
  }

  // 往 goal 飛（避碰），回傳現在離 goal 的距離。
  // 路權（avoidance.hpp）：執行中 = 目前的 ExecMode 是 NAVIGATING／EXECUTING（和位置廣播的 busy 同一個來源）；
  // 任務中懸停 = 執行中而且已經到 goal（水平 < kTaskHoverRadius）——BT 正在任務點停留、辨識或監控，不是看速度猜
  // yield_to_busy：2026-10-24 起不用（待命讓執行中的由路權決定），留著參數讓呼叫的地方不用改
  double moveTo(const Vec3 & goal, bool yield_to_busy = false);
  // 運動平滑層從目前位置重新開始（被接管後切回 offboard：參考位置不要停在接管前的點）
  void resetMotion() {motion_.reset();}
  void setExec(ExecMode mode, double remaining, const AssignedTask * active);
  bool reported(const AssignedTask & t) const {return reported_.count({t.task_id, t.version}) > 0;}
  void markReported(const AssignedTask & t) {reported_.insert({t.task_id, t.version});}

  UavPlatform & platform;
  UavContextConfig config;

  bool airborne{false};               // TakeOff 完成（解鎖狀態不見了就重來）
  bool battery_critical{false};       // 一旦觸發就不再恢復
  Vec3 waypoint;                      // 目前任務的航點
  double arrival{-1.0};               // 到達目前任務的時刻；< 0 還沒到
  std::vector<Vec3> reported_fires;   // 已回報的火點（map ENU）
  std::vector<std::uint32_t> fire_tasks;   // 和 reported_fires 對應的地面處置任務

  // 這個位置的火已經有地面處置任務（自己報的，或 cbba_node 轉來的別台報的；離 fire_dedup_radius 以內）
  bool fireReported(const Vec3 & p) const;
  // 這個地面處置任務是自己建立的（不是別台報的）
  bool ownFireTask(std::uint32_t task_id) const;
  // 符合火警條件（tag、confidence ≥ min_conf、張數 ≥ min_frames、max_age 秒內）、還沒回報過的火點；沒有就 nullopt
  std::optional<FireObservation> newFire(unsigned tag, double min_conf, int min_frames, double max_age) const;
  // 建立地面處置任務（地面的點，z = 0），記下火點和任務；回傳 task_id
  std::uint32_t reportFire(const Vec3 & p);
  // 離 p fire_dedup_radius 以內的地面處置任務：先找自己建立的，再找別台的（task_announced）；沒有就 nullopt
  std::optional<std::uint32_t> fireTaskNear(const Vec3 & p) const;

private:
  void logAvoidEvents(const std::vector<AvoidEvent> & events);

  std::set<std::pair<std::uint32_t, std::uint64_t>> reported_;
  double encounter_min_{-1.0};       // 這次接近的最近距離；< 0 沒有在接近
  MotionShaper motion_;              // CBF 的速度 → 平順的位置＋速度 setpoint
  CollisionAvoider avoider_;         // 避碰：每台鄰機的交會狀態（誰讓、往哪邊）
  ExecMode exec_mode_{ExecMode::IDLE};   // setExec 最後一次的狀態（路權的 busy）
};

// 註冊全部節點（blackboard 要先放 "ctx"）
void registerUavNodes(BT::BehaviorTreeFactory & factory);

}  // namespace uav_px4_bt
