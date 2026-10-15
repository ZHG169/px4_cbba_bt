#include "uav_px4_bt/motion.hpp"

#include <algorithm>
#include <cmath>

namespace uav_px4_bt
{

Setpoint MotionShaper::step(double now, const Vec3 & position, const Vec3 & velocity, const Vec3 & desired,
  const Vec3 & goal, bool avoiding)
{
  const double dt = now - last_;
  if (!started_ || dt > params_.reset_gap || dt < 0.0) {
    started_ = true;
    ref_pos_ = position;
    ref_vel_ = velocity;
    last_ = now;
    return {ref_pos_, ref_vel_};
  }
  last_ = now;

  if (!avoiding && norm(goal - position) < params_.hold_radius) {
    ref_pos_ = goal;
    ref_vel_ = {};
    return {ref_pos_, ref_vel_};
  }

  // 參考速度往想要的速度靠近：水平、垂直分開限制加速度
  Vec3 dv = desired - ref_vel_;
  const double h = std::hypot(dv.x, dv.y);
  const double max_h = params_.max_accel * dt;
  if (h > max_h && h > 0.0) {
    dv.x *= max_h / h;
    dv.y *= max_h / h;
  }
  dv.z = std::clamp(dv.z, -params_.max_vertical_accel * dt, params_.max_vertical_accel * dt);
  ref_vel_ = ref_vel_ + dv;
  ref_pos_ = ref_pos_ + ref_vel_ * dt;

  // 參考位置不越過目標（2026-10-13 SITL：起飛衝到 6.8 m）：CBF 的速度是用「實際」位置算的，PX4 落後時實際位置還沒到、
  // 參考位置卻已經積分過頭，最多到 leash。越過時停在目標、交給 PX4 位置保持
  if (!avoiding && dot(goal - ref_pos_, goal - position) <= 0.0) {
    ref_pos_ = goal;
    ref_vel_ = {};
    return {ref_pos_, ref_vel_};
  }

  // PX4 追不上（風、避碰、限速）時參考位置不要越拉越遠
  const Vec3 lead = ref_pos_ - position;
  const double d = norm(lead);
  if (d > params_.leash) {
    ref_pos_ = position + lead * (params_.leash / d);
  }
  return {ref_pos_, ref_vel_};
}

}  // namespace uav_px4_bt
