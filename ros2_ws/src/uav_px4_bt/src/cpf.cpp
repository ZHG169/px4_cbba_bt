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
  const std::vector<Neighbor> & neighbors, const CpfParams & p)
{
  CpfOutput out;
  Vec3 v = limit((goal - position) * p.k_att, p.max_speed);

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
    if (dist >= R) {
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
    const Vec3 dir = d * (1.0 / dist);
    v = v + dir * mag;
    // 環流：遠離方向逆時針轉 90°（ENU：(x, y) → (−y, x)）＝面向鄰機時的右手邊
    const double h = std::hypot(dir.x, dir.y);
    if (h > 1e-3) {
      v = v + Vec3{-dir.y / h, dir.x / h, 0.0} * (p.k_tan * mag);
    }
  }
  v.z = std::clamp(v.z, -p.max_vertical_speed, p.max_vertical_speed);
  out.velocity = limit(v, p.max_speed);
  return out;
}

}  // namespace uav_px4_bt
