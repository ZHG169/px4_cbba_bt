#include "uav_px4_bt/cbf.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

// 水平的半平面約束 a·u ≥ b
struct Constraint
{
  double ax{0.0};
  double ay{0.0};
  double b{0.0};
};

constexpr int kMaxIterations = 100;
constexpr double kTolerance = 1e-9;

// Dykstra 交替投影：u0 投影到所有半平面的交集（最近點）。交集是空的（鄰機從兩側擠進 safe_radius）時不收斂，
// 迭代上限後回傳最後一次的結果（盡量往各約束靠）
void project(double & ux, double & uy, const std::vector<Constraint> & cs)
{
  std::vector<double> qx(cs.size(), 0.0), qy(cs.size(), 0.0);
  for (int it = 0; it < kMaxIterations; ++it) {
    double change = 0.0;
    for (std::size_t k = 0; k < cs.size(); ++k) {
      const Constraint & c = cs[k];
      const double yx = ux + qx[k], yy = uy + qy[k];
      double zx = yx, zy = yy;
      const double s = c.ax * yx + c.ay * yy;
      if (s < c.b) {
        const double aa = c.ax * c.ax + c.ay * c.ay;
        zx += c.ax * (c.b - s) / aa;
        zy += c.ay * (c.b - s) / aa;
      }
      qx[k] = yx - zx;
      qy[k] = yy - zy;
      change += std::fabs(zx - ux) + std::fabs(zy - uy);
      ux = zx;
      uy = zy;
    }
    if (change < kTolerance) {
      return;
    }
  }
}
}  // namespace

Vec3 nominalVelocity(const Vec3 & position, const Vec3 & goal, const CbfParams & p)
{
  // 剎車曲線（以 brake_accel 減速剛好停在目標）
  const Vec3 to_goal = goal - position;
  const double gd = norm(to_goal);
  Vec3 u;
  if (gd > 1e-6) {
    double speed = std::min({p.max_speed, p.k_att * gd, std::sqrt(2.0 * p.brake_accel * gd)});
    // 垂直分量的剎車曲線：整個向量等比例縮小（方向不變）
    const double vz = std::fabs(to_goal.z) * speed / gd;
    const double vz_max = std::sqrt(2.0 * p.brake_vertical_accel * std::fabs(to_goal.z));
    if (vz > vz_max) {
      speed *= vz_max / vz;
    }
    u = to_goal * (speed / gd);
  }
  return u;
}

CbfFilterResult cbfFilter(const Vec3 & position, const Vec3 & u_ref, const std::vector<CbfPair> & pairs,
  const CbfParams & p)
{
  std::vector<Constraint> cs;
  for (const CbfPair & pr : pairs) {
    double rx = position.x - pr.neighbor.position.x, ry = position.y - pr.neighbor.position.y;
    double dist = std::hypot(rx, ry);
    if (dist < 1e-3) {
      rx = 1e-3;
      ry = 0.0;
      dist = 1e-3;
    }
    const double h = dist * dist - p.safe_radius * p.safe_radius;
    Constraint c;
    c.ax = 2.0 * rx;
    c.ay = 2.0 * ry;
    c.b = pr.lambda * (c.ax * pr.neighbor.velocity.x + c.ay * pr.neighbor.velocity.y) - pr.rho * p.gamma * h;
    cs.push_back(c);
  }
  double ux = u_ref.x, uy = u_ref.y;
  if (!cs.empty()) {
    project(ux, uy, cs);
  }
  CbfFilterResult out;
  out.velocity = limit({ux, uy, std::clamp(u_ref.z, -p.max_vertical_speed, p.max_vertical_speed)}, p.max_speed);
  for (const Constraint & c : cs) {
    // 違反量換成 m/s（除以 |a|），和約束的尺度無關
    const double gap = (c.b - (c.ax * out.velocity.x + c.ay * out.velocity.y)) / std::hypot(c.ax, c.ay);
    out.worst = std::max(out.worst, gap);
  }
  out.feasible = out.worst <= 1e-3;
  return out;
}

namespace
{
// 二階 CBF 的一列約束 n·a ≥ b（水平）；d ≤ r 時 valid = false（已經在安全距離內）
struct Row
{
  bool valid{false};
  double nx{0.0};
  double ny{0.0};
  double b{0.0};
};

Row cbf2Row(const Vec3 & position, const Vec3 & velocity, const Neighbor & n, double rho, const CbfParams & p,
  bool clamp_inside = false)
{
  Row row;
  double rx = position.x - n.position.x, ry = position.y - n.position.y;
  double d = std::hypot(rx, ry);
  if (d < 1e-3) {
    rx = 1e-3;
    ry = 0.0;
    d = 1e-3;
  }
  row.nx = rx / d;
  row.ny = ry / d;
  const double dvx = velocity.x - n.velocity.x, dvy = velocity.y - n.velocity.y;
  const double ddot = row.nx * dvx + row.ny * dvy;
  const double tang2 = std::max(0.0, dvx * dvx + dvy * dvy - ddot * ddot);
  const double A = p.accel_max / std::max(rho, 1e-3);
  double r = p.safe_radius + p.response_delay * std::max(0.0, -ddot);
  if (d <= r) {
    if (!clamp_inside) {
      row.b = std::numeric_limits<double>::infinity();
      return row;
    }
    r = d - 0.05;             // 緊急時：當成剛好在邊上（b 很大，要往外）
  }
  const double s = std::max(std::sqrt(2.0 * A * (d - r)), 1e-3);
  const double h = s + ddot;
  row.valid = true;
  row.b = rho * (-tang2 / d - A * ddot / s - p.gamma * h);
  return row;
}

// Dykstra：(ax, ay) 投影到所有半平面 n·a ≥ b 和圓盤 |a| ≤ amax 的交集
void projectAccel(double & ax, double & ay, const std::vector<Row> & rows, double amax)
{
  const std::size_t m = rows.size() + 1;    // 最後一個是圓盤
  std::vector<double> qx(m, 0.0), qy(m, 0.0);
  for (int it = 0; it < kMaxIterations; ++it) {
    double change = 0.0;
    for (std::size_t k = 0; k < m; ++k) {
      const double yx = ax + qx[k], yy = ay + qy[k];
      double zx = yx, zy = yy;
      if (k < rows.size()) {
        const double sdot = rows[k].nx * yx + rows[k].ny * yy;
        if (sdot < rows[k].b) {
          zx += rows[k].nx * (rows[k].b - sdot);
          zy += rows[k].ny * (rows[k].b - sdot);
        }
      } else {
        const double len = std::hypot(yx, yy);
        if (len > amax && len > 0.0) {
          zx *= amax / len;
          zy *= amax / len;
        }
      }
      qx[k] = yx - zx;
      qy[k] = yy - zy;
      change += std::fabs(zx - ax) + std::fabs(zy - ay);
      ax = zx;
      ay = zy;
    }
    if (change < kTolerance) {
      return;
    }
  }
}
}  // namespace

Cbf2Result cbf2Filter(const Vec3 & position, const Vec3 & velocity, const Vec3 & a_ref,
  const std::vector<Cbf2Pair> & pairs, const CbfParams & p)
{
  Cbf2Result out;
  std::vector<Row> rows;
  bool inside = false;
  for (const Cbf2Pair & pr : pairs) {
    const Row row = cbf2Row(position, velocity, pr.neighbor, pr.rho, p);
    inside = inside || !row.valid;
    if (row.valid) {
      rows.push_back(row);
    }
  }
  double ax = a_ref.x, ay = a_ref.y;
  projectAccel(ax, ay, rows, p.accel_max);
  out.accel = {ax, ay, 0.0};
  out.worst = std::max(0.0, std::hypot(ax, ay) - p.accel_max);
  for (const Row & r : rows) {
    out.worst = std::max(out.worst, r.b - (r.nx * ax + r.ny * ay));
  }
  out.feasible = !inside && out.worst <= 1e-3;
  return out;
}

Vec3 cbf2Escape(const Vec3 & position, const Vec3 & velocity, const std::vector<Cbf2Pair> & pairs,
  const CbfParams & p)
{
  std::vector<Row> rows;
  for (const Cbf2Pair & pr : pairs) {
    rows.push_back(cbf2Row(position, velocity, pr.neighbor, pr.rho, p, true));
  }
  constexpr int kDirs = 48;
  Vec3 best;
  double best_score = -std::numeric_limits<double>::infinity();
  for (int k = 0; k < kDirs; ++k) {
    const double th = 2.0 * 3.14159265358979323846 * k / kDirs;
    const double ax = p.accel_max * std::cos(th), ay = p.accel_max * std::sin(th);
    double score = std::numeric_limits<double>::infinity();
    for (const Row & r : rows) {
      score = std::min(score, r.nx * ax + r.ny * ay - r.b);
    }
    if (score > best_score) {
      best_score = score;
      best = {ax, ay, 0.0};
    }
  }
  return best;
}

double brakingMargin(const Vec3 & position, const Vec3 & velocity, const Neighbor & n, double rho,
  const CbfParams & p)
{
  const double rx = position.x - n.position.x, ry = position.y - n.position.y;
  const double d = std::max(std::hypot(rx, ry), 1e-3);
  const double closing = std::max(0.0, -((velocity.x - n.velocity.x) * rx + (velocity.y - n.velocity.y) * ry) / d);
  const double A = p.accel_max / std::max(rho, 1e-3);
  return d - p.safe_radius - p.response_delay * closing - closing * closing / (2.0 * A);
}

CbfOutput cbfVelocity(
  const Vec3 & position, const Vec3 & goal, const std::vector<Neighbor> & neighbors, const CbfParams & p,
  bool yield_to_busy)
{
  CbfOutput out;
  const Vec3 u = nominalVelocity(position, goal, p);

  // 每台鄰機一個約束 a·u ≥ b（水平）
  std::vector<Constraint> cs;
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
    double rx = rel.x, ry = rel.y;
    double dist = std::hypot(rx, ry);
    if (dist >= p.influence_radius) {
      continue;
    }
    if (dist < 1e-3) {
      // 水平完全重疊（出生點不同，實際上不會發生）：用固定方向，只為了不除以 0
      rx = 1e-3;
      ry = 0.0;
      dist = 1e-3;
    }
    // 讓路：自己待命、鄰機在執行任務 → 把它當不會讓的障礙物（算進它的速度、全額負責）
    const bool yield = yield_to_busy && n.busy;
    const double lambda = yield ? 1.0 : 0.0;
    const double rho = yield ? 1.0 : 0.5;
    const double r = yield ? std::max(p.yield_radius, p.safe_radius) : p.safe_radius;
    const double h = dist * dist - r * r;
    Constraint c;
    c.ax = 2.0 * rx;
    c.ay = 2.0 * ry;
    c.b = lambda * (c.ax * n.velocity.x + c.ay * n.velocity.y) - rho * p.gamma * h;
    cs.push_back(c);
  }

  // 往右繞：名目速度違反的約束，加「違反量 × k_tan」的右手邊分量（面向鄰機時的右邊 = 遠離方向逆時針轉 90°）
  double ux = u.x, uy = u.y;
  for (const Constraint & c : cs) {
    const double an = std::hypot(c.ax, c.ay);
    const double deficit = (c.b - (c.ax * u.x + c.ay * u.y)) / an;   // m/s：名目速度在遠離方向還差多少
    if (deficit <= 0.0) {
      continue;
    }
    ++out.active_neighbors;
    ux += -c.ay / an * p.k_tan * deficit;
    uy += c.ax / an * p.k_tan * deficit;
  }
  if (out.active_neighbors > 0) {
    project(ux, uy, cs);
  }

  const Vec3 v{ux, uy, std::clamp(u.z, -p.max_vertical_speed, p.max_vertical_speed)};
  out.velocity = limit(v, p.max_speed);
  return out;
}

}  // namespace uav_px4_bt
