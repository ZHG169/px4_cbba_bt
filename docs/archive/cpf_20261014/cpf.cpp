#include "uav_px4_bt/cpf.hpp"

#include <algorithm>
#include <cmath>

namespace uav_px4_bt
{

double norm(const Vec3 & v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

namespace
{
Vec3 limit(const Vec3 & v, double max)
{
  const double n = norm(v);
  return n > max && n > 0.0 ? v * (max / n) : v;
}
}  // namespace

CpfOutput cpfVelocity(
  const Vec3 & position, const Vec3 & velocity, const Vec3 & goal,
  const std::vector<Neighbor> & neighbors, const CpfParams & p, bool yield_to_busy)
{
  CpfOutput out;
  // 吸引：剎車曲線（以 brake_accel 減速剛好停在目標）
  const Vec3 to_goal = goal - position;
  const double gd = norm(to_goal);
  Vec3 v;
  if (gd > 1e-6) {
    double speed = std::min({p.max_speed, p.k_att * gd, std::sqrt(2.0 * p.brake_accel * gd)});
    // 垂直分量的剎車曲線：整個向量等比例縮小（方向不變）
    const double vz = std::fabs(to_goal.z) * speed / gd;
    const double vz_max = std::sqrt(2.0 * p.brake_vertical_accel * std::fabs(to_goal.z));
    if (vz > vz_max) {
      speed *= vz_max / vz;
    }
    v = to_goal * (speed / gd);
  }

  const double R = p.influence_radius;
  const double safe = std::min(p.safe_radius, R * 0.99);
  for (const Neighbor & n : neighbors) {
    if (n.age > p.neighbor_timeout) {
      continue;
    }
    const Vec3 other = n.position + n.velocity * std::min(n.age, p.max_extrapolation);
    const Vec3 rel = position - other;
    const double now_dist = norm(rel);
    if (out.min_distance < 0.0 || now_dist < out.min_distance) {
      out.min_distance = now_dist;
    }
    // horizon 內最接近時的相對位置：rel + relv × t*，t* = −rel·relv / |relv|²（限制在 0～horizon）
    const Vec3 relv = velocity - n.velocity;
    const double vv = dot(relv, relv);
    double t = vv > 1e-6 ? -dot(rel, relv) / vv : 0.0;
    t = std::clamp(t, 0.0, p.horizon);
    Vec3 d = rel + relv * t;
    double dist = norm(d);
    // 沒有在靠近（預測不會更近）：只在 safe_radius 內排斥，旁邊懸停、平行飛的鄰機不互推；
    // 待命時對執行任務中的鄰機照 R（讓路給它）
    const bool closing = now_dist > 1e-3 && -dot(rel, relv) / now_dist >= p.closing_speed;
    // 自己在待命（yield_to_busy）時：執行任務中的鄰機照 R 讓路；同樣在待命的鄰機只在 safe_radius 內推開，
    // 「正在靠近」不提早反應（2026-10-13 SITL：懸停的漂移一到 0.5 m/s 就觸發 R 內的排斥＋環流，兩台互繞停不下來）
    const bool early = yield_to_busy ? n.busy : closing;
    if (dist >= (early ? R : safe)) {
      continue;
    }
    if (dist < 1e-3) {
      // 預測會正面相撞：用現在的相對位置決定方向。連現在都完全重疊（出生點不同，實際上不會發生）時
      // 用固定方向，只為了不除以 0
      d = now_dist > 1e-3 ? rel : Vec3{1.0, 0.0, 0.0};
      dist = std::max(norm(d), 1e-3);
    }
    ++out.active_neighbors;
    const double mag = std::min(
      p.k_rep * (1.0 / std::max(dist, 0.05) - 1.0 / R) / (1.0 / safe - 1.0 / R), p.max_repulsion);
    // 方向只取水平（2026-10-13 SITL：3D 的方向在高度差一點點時把兩台互相往上、往下推，懸停時高度在 3.2～6.8 m 擺；
    // 全隊飛同一個高度層，垂直閃避沒有好處）。正上、正下方（水平幾乎重合）時用現在的水平相對位置
    double hx = d.x, hy = d.y;
    if (std::hypot(hx, hy) < 1e-3) {
      hx = rel.x;
      hy = rel.y;
    }
    const double h = std::hypot(hx, hy);
    if (h < 1e-3) {
      hx = 1.0;
      hy = 0.0;
    }
    const double hn = std::max(std::hypot(hx, hy), 1e-3);
    const Vec3 dir{hx / hn, hy / hn, 0.0};
    v = v + dir * mag;
    // 環流：遠離方向逆時針轉 90°（ENU：(x, y) → (−y, x)）＝面向鄰機時的右手邊
    v = v + Vec3{-dir.y, dir.x, 0.0} * (p.k_tan * mag);
  }
  v.z = std::clamp(v.z, -p.max_vertical_speed, p.max_vertical_speed);
  out.velocity = limit(v, p.max_speed);
  return out;
}

}  // namespace uav_px4_bt
