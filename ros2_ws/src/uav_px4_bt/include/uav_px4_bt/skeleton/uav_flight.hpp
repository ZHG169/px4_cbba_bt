// 無人機在骨架的樹外要做的飛行（2026-10-13，UAV_BT_GUIDE v2 §2：TakeOff 樹外、LandOrLoiter 是待命、電量返航）
//
//   起飛     PX4 起飛前檢查通過 → 先送 1 s 零速度 → 每秒要求 offboard＋解鎖 → 原地爬升到 altitude（誤差 takeoff_tolerance）
//            → AIRBORNE。node 層到這時才 tick 樹（px4_state_bridge 爬到 altitude − 0.5 m 才送 AUTO，cbba_node 才出價）
//   待命     佇列空、樹沒在跑：先原地懸停，return_home_delay 秒後飛回起飛點上空（返航點：第一次起飛的位置、altitude）。
//            待命點的高度一律是 altitude（2026-10-17：原本用當時的高度，起飛完成時離 altitude 還差 takeoff_tolerance 以內，
//            SITL 實測一直停在 4.71 m；做完任務、被接管後也會停在當時的高度）
//            待命（2026-10-13 使用者：任務做完要回到初始位置；負值 = 一直原地懸停）。影響範圍內有執行任務中的鄰機時讓路
//            （停在被推開的位置，鄰機離開後再繼續飛回去）。GUIDE §3 的 DefaultBehavior 不放進樹，樹跑完就結束、回報照骨架的流程
//   返航     在空中、電量 ≤ min_battery → 停止 offboard、PX4 RTL（指令可能掉：每 2 s 再送，最多 3 次），之後一直是 RTL
//   不在空中 起飛完成後上鎖或降落 → 回到起飛（和備用 BT 的 TakeOff 一樣會重新起飛）
//   被接管   起飛完成後 offboard 掉了（不是自己的返航）→ OVERRIDE（2026-10-17，照機器狗端的平板接管）：
//            PX4 的 failsafe = 0（或失效保護中 user_took_over）＝ 人用遙控器／地面站切了模式 → TAKEOVER
//            （node 層把手上的任務以 PREEMPTED_BY_MANUAL 交還）；failsafe = 1 ＝ PX4 自己的失效保護（setpoint 斷了、
//            地理圍欄、PX4 的低電量）→ FAILSAFE（PX4_FAILSAFE，STOPPED）。2026-10-17 SITL 實測：commander mode auto:loiter
//            → user_intention 跟著變、failsafe 0；停掉 BT → nav_state AUTO_RTL、user_intention 仍是 OFFBOARD、failsafe 1。
//            OVERRIDE 期間不 tick 樹、mode 送 MANUAL（node 層），持續送「目前位置」的 setpoint（PX4 要有 offboard 訊號
//            才能切回）。切回 offboard → AIRBORNE（RESUMED，照狗：直接放行），待命點改成目前的水平位置、巡檢高度
//            （不飛回接管前的點），高度和 altitude 差超過 takeoff_tolerance 時警告
//   高度警告（2026-10-17，只印 log）：爬升 kClimbWarnSec 秒還沒到 altitude ± takeoff_tolerance：一直不算起飛完成、不 tick 樹
//
// 不依賴 ROS：透過 UavContext（UavPlatform）讀狀態、下命令，test_skeleton 用假平台測。
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

#include "uav_px4_bt/bt_nodes.hpp"

namespace uav_px4_bt
{

// OVERRIDE：在空中，但不是 BT 在控制（被接管或 PX4 失效保護）
enum class FlightPhase { GROUND, AIRBORNE, RTL, OVERRIDE };

// update() 這個 tick 發生的事（node 層據此結束任務、開始 tick 樹）
enum class FlightEvent { NONE, TOOK_OFF, LANDED, BATTERY_RTL, TAKEOVER, FAILSAFE, RESUMED };

class UavFlight
{
public:
  UavFlight(UavContext & ctx, double altitude, double min_battery, double return_home_delay = -1.0)
  : ctx_(ctx), altitude_(altitude), min_battery_(min_battery), return_home_delay_(return_home_delay) {}

  FlightPhase phase() const {return phase_;}
  const Vec3 & home() const {return home_;}
  double altitude() const {return altitude_;}
  // OVERRIDE 的原因：true = 人接管（PREEMPTED_BY_MANUAL），false = PX4 失效保護（PX4_FAILSAFE）
  bool overrideByUser() const {return override_user_;}
  const char * overrideReason() const {return override_user_ ? "PREEMPTED_BY_MANUAL" : "PX4_FAILSAFE";}

  // 每個 tick 先呼叫：起飛中、返航中自己下命令；在空中時只檢查電量、是否還在空中
  FlightEvent update()
  {
    const VehicleState v = ctx_.platform.vehicle();
    const double now = ctx_.platform.timeNow();
    switch (phase_) {
      case FlightPhase::RTL:
        ctx_.setExec(ExecMode::FAULT, -1.0, nullptr);
        if (rtl_sent_ < 3 && now - rtl_last_ >= 2.0) {
          ctx_.platform.requestReturnToLaunch();
          ++rtl_sent_;
          rtl_last_ = now;
        }
        return FlightEvent::NONE;
      case FlightPhase::OVERRIDE:
        return updateOverride(v);
      case FlightPhase::AIRBORNE:
        if (!v.armed || v.landed) {
          return landed();
        }
        if (!v.offboard) {
          return enterOverride(v);
        }
        // 一旦觸發就不再恢復，電量讀數上下跳時不會來回切換
        if (v.battery >= 0.0 && v.battery <= min_battery_) {
          char b[96];
          std::snprintf(b, sizeof(b), "電量 %.1f %% ≤ %.1f %%：返航", v.battery, min_battery_);
          ctx_.platform.log(b);
          ctx_.battery_critical = true;
          ctx_.setExec(ExecMode::FAULT, -1.0, nullptr);
          ctx_.platform.stopOffboard();
          ctx_.platform.requestReturnToLaunch();
          phase_ = FlightPhase::RTL;
          rtl_sent_ = 1;
          rtl_last_ = now;
          return FlightEvent::BATTERY_RTL;
        }
        return FlightEvent::NONE;
      case FlightPhase::GROUND:
        break;
    }
    return takeOff(v, now) ? FlightEvent::TOOK_OFF : FlightEvent::NONE;
  }

  // 待命：原地懸停，return_home_delay 秒後飛回返航點上空；讓路給執行任務中的鄰機。樹跑完後的第一次從那時的位置開始
  void loiter()
  {
    const VehicleState v = ctx_.platform.vehicle();
    const double now = ctx_.platform.timeNow();
    if (!hold_valid_) {
      hold_ = atAltitude(v.position);
      hold_valid_ = true;
      idle_since_ = now;
      homing_ = false;
    }
    if (!homing_ && have_home_ && return_home_delay_ >= 0.0 && now - idle_since_ >= return_home_delay_ &&
      norm(home_ - v.position) > ctx_.config.reach_tolerance)
    {
      homing_ = true;
      char b[96];
      std::snprintf(b, sizeof(b), "待命：飛回起飛點上空 (%.1f, %.1f, %.1f)", home_.x, home_.y, home_.z);
      ctx_.platform.log(b);
    }
    bool yield = false;
    for (const Neighbor & n : ctx_.platform.neighbors()) {
      yield = yield || (n.busy && norm(n.position - v.position) < ctx_.config.cbf.influence_radius);
    }
    if (yield) {
      hold_ = atAltitude(v.position);
      if (!yielding_) {
        ctx_.platform.log("待命中：讓路給執行任務的鄰機");
      }
    } else if (homing_) {
      hold_ = home_;
    }
    yielding_ = yield;
    ctx_.moveTo(hold_, true);
    ctx_.setExec(ExecMode::IDLE, -1.0, nullptr);
  }

  // 樹開始跑時呼叫：下次待命從那時的位置開始，不飛回舊的待命點
  void leaveHold() {hold_valid_ = false;}

private:
  Vec3 atAltitude(const Vec3 & p) const {return {p.x, p.y, altitude_};}

  FlightEvent landed()
  {
    ctx_.platform.log("已經不在空中（上鎖或降落）：重新起飛");
    ctx_.airborne = false;
    phase_ = FlightPhase::GROUND;
    takeoff_start_ = -1.0;
    return FlightEvent::LANDED;
  }

  static bool byUser(const VehicleState & v) {return !v.failsafe || v.user_took_over;}

  FlightEvent enterOverride(const VehicleState & v)
  {
    phase_ = FlightPhase::OVERRIDE;
    override_user_ = byUser(v);
    char b[256];
    std::snprintf(b, sizeof(b), override_user_ ?
      "警告：被接管（不是 offboard，failsafe 0），高度 %.2f m：交還手上的任務（PREEMPTED_BY_MANUAL）、mode MANUAL，"
      "切回 offboard 才接任務" :
      "警告：PX4 失效保護（不是 offboard，failsafe 1），高度 %.2f m：交還手上的任務（PX4_FAILSAFE）、mode MANUAL",
      v.position.z);
    ctx_.platform.log(b);
    ctx_.setExec(ExecMode::FAULT, -1.0, nullptr);
    holdHere(v);
    return override_user_ ? FlightEvent::TAKEOVER : FlightEvent::FAILSAFE;
  }

  FlightEvent updateOverride(const VehicleState & v)
  {
    if (!v.armed || v.landed) {
      return landed();
    }
    if (v.offboard) {
      phase_ = FlightPhase::AIRBORNE;
      hold_valid_ = false;   // 待命點改成目前的水平位置（巡檢高度）：接管期間被飛到別處，不要飛回接管前的點
      ctx_.resetMotion();
      char b[256];
      const double dz = v.position.z - altitude_;
      if (std::fabs(dz) >= ctx_.config.takeoff_tolerance) {
        std::snprintf(b, sizeof(b), "警告：切回 offboard，高度 %.2f m 和巡檢高度 %.1f m 差 %+.2f m："
          "不重新起飛，從目前的水平位置接回、回到巡檢高度、mode AUTO", v.position.z, altitude_, dz);
      } else {
        std::snprintf(b, sizeof(b), "切回 offboard（高度 %.2f m）：從目前位置接回、mode AUTO", v.position.z);
      }
      ctx_.platform.log(b);
      return FlightEvent::RESUMED;
    }
    // 失效保護中人接手、或人放手後 PX4 失效保護：只改原因（之後收到的任務用它交還）；手上的任務進來時已經交還
    const bool user = byUser(v);
    if (user != override_user_) {
      override_user_ = user;
      ctx_.platform.log(user ? "被接管中：人接手（failsafe 0 或 user_took_over）" : "被接管中：PX4 失效保護（failsafe 1）");
    }
    ctx_.setExec(ExecMode::FAULT, -1.0, nullptr);
    holdHere(v);
    return FlightEvent::NONE;
  }

  // PX4 要一直收到 offboard 的 setpoint 才能切回 offboard；送目前位置，切回時從這裡開始、不會往接管前的點衝
  void holdHere(const VehicleState & v) {ctx_.platform.setSetpoint(v.position, {});}

  bool takeOff(const VehicleState & v, double now)
  {
    ctx_.setExec(ExecMode::IDLE, -1.0, nullptr);
    if (takeoff_start_ < 0.0) {
      takeoff_start_ = now;
      last_request_ = -1e9;
      have_climb_xy_ = false;
      climb_warned_ = false;
    }
    if (!v.valid || !v.ready) {
      ctx_.platform.setVelocity({});
      return false;
    }
    if (!(v.armed && v.offboard)) {
      // PX4 要先收到 setpoint 才能切 offboard：先送 1 s 的零速度，再每秒要求一次 offboard＋解鎖
      ctx_.platform.setVelocity({});
      if (now - takeoff_start_ >= 1.0 && now - last_request_ >= 1.0) {
        ctx_.platform.requestOffboardAndArm();
        last_request_ = now;
      }
      return false;
    }
    if (!have_climb_xy_) {
      climb_xy_ = v.position;
      have_climb_xy_ = true;
      climb_start_ = now;
      char b[64];
      std::snprintf(b, sizeof(b), "解鎖、offboard：爬升到 %.1f m", altitude_);
      ctx_.platform.log(b);
    }
    ctx_.moveTo({climb_xy_.x, climb_xy_.y, altitude_}, true);   // 起飛也算待命：旁邊一起爬升的不提早互推
    if (std::fabs(v.position.z - altitude_) >= ctx_.config.takeoff_tolerance) {
      if (!climb_warned_ && now - climb_start_ >= kClimbWarnSec) {
        climb_warned_ = true;
        char w[256];
        std::snprintf(w, sizeof(w), "警告：爬升 %.0f s 還沒到 %.1f m（現在 %.2f m，誤差 %.1f m 內才算起飛完成）："
          "不 tick 樹", kClimbWarnSec, altitude_, v.position.z, ctx_.config.takeoff_tolerance);
        ctx_.platform.log(w);
      }
      return false;
    }
    char b[64];
    std::snprintf(b, sizeof(b), "到達 %.1f m，等待指派", altitude_);
    ctx_.platform.log(b);
    if (!have_home_) {
      home_ = {climb_xy_.x, climb_xy_.y, altitude_};   // 返航點：第一次起飛的位置
      have_home_ = true;
    }
    ctx_.airborne = true;
    phase_ = FlightPhase::AIRBORNE;
    hold_valid_ = false;
    return true;
  }

  static constexpr double kClimbWarnSec = 20.0;   // 5 m 的巡檢高度正常幾秒就到

  UavContext & ctx_;
  double altitude_;
  double min_battery_;
  double return_home_delay_;
  FlightPhase phase_{FlightPhase::GROUND};

  double takeoff_start_{-1.0};
  double last_request_{-1e9};
  bool have_climb_xy_{false};
  Vec3 climb_xy_;
  double climb_start_{0.0};
  bool climb_warned_{false};
  bool override_user_{true};

  int rtl_sent_{0};
  double rtl_last_{0.0};

  bool hold_valid_{false};
  bool yielding_{false};
  Vec3 hold_;
  double idle_since_{0.0};
  bool homing_{false};
  bool have_home_{false};
  Vec3 home_;
};

}  // namespace uav_px4_bt
