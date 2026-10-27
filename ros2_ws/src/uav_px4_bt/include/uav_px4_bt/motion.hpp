// 運動平滑層（2026-10-12）：把 CBF 算出的「想要的速度」變成平順的位置＋速度 setpoint 給 PX4
//
// 原本直接把 CPF（2026-10-14 換成 CBF）的速度送給 PX4（只有速度、沒有位置）：任務開始時速度從 0 跳到巡航速度、排斥出現時方向瞬間改變，
// PX4 只能大角度傾斜去追；懸停時靠「速度 = k × 距離」拉回目標，位置一直小幅漂移、來回修正。SITL 看起來很晃。
//
//   參考速度 v_ref  每個週期往想要的速度靠近，變化量限制在 max_accel × dt（水平）、max_vertical_accel × dt（垂直）
//   參考位置 p_ref  由 v_ref 積分；離實際位置超過 leash 時拉回（PX4 追不上時不要越拉越遠）
//   到達保持        離目標 < hold_radius、附近沒有要閃的鄰機時，p_ref = 目標、v_ref = 0：交給 PX4 的位置保持；
//                   p_ref 越過目標（PX4 落後、實際位置還沒到）時也一樣（2026-10-13）
//   送給 PX4        TrajectorySetpoint 的 position = p_ref、velocity = v_ref（前饋）：PX4 位置控制器追 p_ref，
//                   速度前饋讓它不落後
// 長時間沒呼叫（> reset_gap，例如換了動作節點、剛起飛）時從目前的位置、速度重新開始，不會跳。
#pragma once

#include "uav_px4_bt/cbf.hpp"

namespace uav_px4_bt
{

struct MotionParams
{
  double max_accel{1.5};             // m/s²（水平）：0 → 3 m/s 約 2 s
  double max_vertical_accel{1.0};    // m/s²
  double leash{1.5};                 // p_ref 離實際位置最多這麼遠（公尺）
  double hold_radius{0.2};           // 離目標這麼近、沒有鄰機要閃時直接位置保持
  double reset_gap{0.5};             // 超過這麼久沒呼叫就從目前狀態重新開始（秒）
};

struct Setpoint
{
  Vec3 position;
  Vec3 velocity;
};

class MotionShaper
{
public:
  explicit MotionShaper(const MotionParams & params = MotionParams{}) : params_(params) {}

  // now：秒（單調）；position、velocity：目前狀態；desired：CBF 的速度；goal：目標；
  // avoiding：CBF 有鄰機在作用（不要直接位置保持，讓它閃）
  Setpoint step(double now, const Vec3 & position, const Vec3 & velocity, const Vec3 & desired,
    const Vec3 & goal, bool avoiding);

  void reset() {started_ = false;}
  // 下一次 step 會從哪個參考速度起算（長時間沒呼叫、剛開始時是實際速度），二階 CBF 用（2026-10-24）
  Vec3 referenceVelocity(double now, const Vec3 & velocity) const
  {
    const double dt = now - last_;
    return (!started_ || dt > params_.reset_gap || dt < 0.0) ? velocity : ref_vel_;
  }
  // 距離上一次 step 多久（沒開始或太久時回傳 fallback）
  double sinceLast(double now, double fallback) const
  {
    const double dt = now - last_;
    return (!started_ || dt > params_.reset_gap || dt <= 0.0) ? fallback : dt;
  }
  const MotionParams & params() const {return params_;}

private:
  MotionParams params_;
  bool started_{false};
  double last_{0.0};
  Vec3 ref_pos_;
  Vec3 ref_vel_;
};

}  // namespace uav_px4_bt
