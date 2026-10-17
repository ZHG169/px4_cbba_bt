// 無人機在骨架的樹外要做的飛行（2026-10-13，UAV_BT_GUIDE v2 §2：TakeOff 樹外、LandOrLoiter 是待命、電量返航）
//
//   起飛     PX4 起飛前檢查通過 → 先送 1 s 零速度 → 每秒要求 offboard＋解鎖 → 原地爬升到 altitude（誤差 takeoff_tolerance）
//            → AIRBORNE。node 層到這時才 tick 樹（px4_state_bridge 爬到 altitude − 0.5 m 才送 AUTO，cbba_node 才出價）
//   待命     佇列空、樹沒在跑：先原地懸停，return_home_delay 秒後飛回起飛點上空（返航點：第一次起飛的位置、altitude）
//            待命（2026-10-13 使用者：任務做完要回到初始位置；負值 = 一直原地懸停）。影響範圍內有執行任務中的鄰機時讓路
//            （停在被推開的位置，鄰機離開後再繼續飛回去）。GUIDE §3 的 DefaultBehavior 不放進樹，樹跑完就結束、回報照骨架的流程
//   返航     在空中、電量 ≤ min_battery → 停止 offboard、PX4 RTL（指令可能掉：每 2 s 再送，最多 3 次），之後一直是 RTL
//   不在空中 起飛完成後上鎖或降落 → 回到起飛（和備用 BT 的 TakeOff 一樣會重新起飛）
//   高度警告（2026-10-17，只印 log、行為不變；被接管時 mode 怎麼送還在和機器狗端討論）：
//            - 爬升 kClimbWarnSec 秒還沒到 altitude ± takeoff_tolerance：一直不算起飛完成、不 tick 樹
//              （px4_state_bridge 的門檻是 altitude − 0.5 m，兩邊可能不一致）
//            - 在空中被切出 offboard（遙控器、地面站接管）：BT 照舊當作在空中，robot_state 仍是 AUTO
//            - 切回 offboard：不重新起飛、不重新確認高度，直接接回；高度和 altitude 差超過 takeoff_tolerance 時警告
//
// 不依賴 ROS：透過 UavContext（UavPlatform）讀狀態、下命令，test_skeleton 用假平台測。
#pragma once

#include <cmath>
#include <cstdio>
#include <string>

#include "uav_px4_bt/bt_nodes.hpp"

namespace uav_px4_bt
{

enum class FlightPhase { GROUND, AIRBORNE, RTL };

// update() 這個 tick 發生的事（node 層據此結束任務、開始 tick 樹）
enum class FlightEvent { NONE, TOOK_OFF, LANDED, BATTERY_RTL };

class UavFlight
{
public:
  UavFlight(UavContext & ctx, double altitude, double min_battery, double return_home_delay = -1.0)
  : ctx_(ctx), altitude_(altitude), min_battery_(min_battery), return_home_delay_(return_home_delay) {}

  FlightPhase phase() const {return phase_;}
  const Vec3 & home() const {return home_;}
  double altitude() const {return altitude_;}

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
      case FlightPhase::AIRBORNE:
        if (!v.armed || v.landed) {
          ctx_.platform.log("已經不在空中（上鎖或降落）：重新起飛");
          ctx_.airborne = false;
          phase_ = FlightPhase::GROUND;
          takeoff_start_ = -1.0;
          return FlightEvent::LANDED;
        }
        noticeOffboard(v);
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
      hold_ = v.position;
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
      hold_ = v.position;
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
    offboard_ = true;
    return true;
  }

  // 被接管的現象先印出來，行為不變（見開頭「高度警告」）
  void noticeOffboard(const VehicleState & v)
  {
    if (v.offboard == offboard_) {
      return;
    }
    offboard_ = v.offboard;
    char b[256];
    const double dz = v.position.z - altitude_;
    if (!v.offboard) {
      std::snprintf(b, sizeof(b), "警告：在空中但不是 offboard（遙控器或地面站接管？），高度 %.2f m："
        "BT 照舊當作在空中，robot_state 仍是 AUTO", v.position.z);
    } else if (std::fabs(dz) >= ctx_.config.takeoff_tolerance) {
      std::snprintf(b, sizeof(b), "警告：切回 offboard，高度 %.2f m 和巡檢高度 %.1f m 差 %+.2f m："
        "不重新起飛、直接接回", v.position.z, altitude_, dz);
    } else {
      std::snprintf(b, sizeof(b), "切回 offboard（高度 %.2f m）：直接接回", v.position.z);
    }
    ctx_.platform.log(b);
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
  bool offboard_{true};               // 在空中時上一次看到的 offboard（只用來印警告）

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
