#include "uav_px4_bt/avoidance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace uav_px4_bt
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kStillSpeed = 0.05;          // 名目速度小於這個（m/s）當作自己沒在動
constexpr double kMinRelSpeed2 = 1e-4;        // 相對速度平方小於這個（< 1 cm/s）：沒有相對運動，不算靠近
constexpr double kStillMargin = 0.5;          // 自己沒在動時，要求的距離最多 safe_radius + 這個
constexpr int kSeparatingCycles = 3;          // 已經通過、距離連續變大這麼多個週期才解除（避免雜訊）
constexpr double kVoStep = 3.0 * kPi / 180.0; // VO 搜尋方向的間隔（3°）
constexpr int kVoSteps = 60;                  // 往左右各找到 180°
constexpr double kSpeedLevels[] = {1.0, 0.75, 0.5, 0.25};   // 保持速度找不到時依序降速
constexpr double kStillYieldFrac = 0.3;       // 在動的讓路、保持速度找不到時的讓路速度至少 max_speed × 這個
constexpr double kMinSidestep = 0.2;          // 懸停的讓路時，往旁邊移的最低速度（m/s）
constexpr double kResidualSlack = 0.05;       // VO 的結果離要求的距離差這麼多以內（m）還算達成
constexpr double kBrakeMargin = 0.5;          // 二階：剎車距離的餘裕小於這個（m）就加 CBF 約束
constexpr double kStillNeighbor = 0.2;        // 二階：鄰機往自己的速度小於這個（m/s）就不靠它分擔剎車

// 一台鄰機在這一步的資料（位置已經外推，水平）
struct Other
{
  const Neighbor * n{nullptr};
  Vec3 position;            // 外推後
  double rx{0.0};           // 自己 − 鄰機
  double ry{0.0};
  double dist{0.0};         // 水平距離
  bool yields{false};       // 對這台，自己是讓的那台
};
}  // namespace

Cpa predictCpa(double rx, double ry, double vx, double vy, double horizon)
{
  Cpa c;
  const double v2 = vx * vx + vy * vy;
  const double closing = -(rx * vx + ry * vy);
  c.approaching = v2 >= kMinRelSpeed2 && closing > 0.0;
  c.t = c.approaching ? std::clamp(closing / v2, 0.0, std::max(horizon, 0.0)) : 0.0;
  c.mx = rx + vx * c.t;
  c.my = ry + vy * c.t;
  c.d = std::hypot(c.mx, c.my);
  return c;
}

bool hasRightOfWay(const RightOfWayKey & self, const RightOfWayKey & other)
{
  if (self.task_hover != other.task_hover) {
    return self.task_hover;
  }
  if (self.busy != other.busy) {
    return self.busy;
  }
  return self.id < other.id;
}

AvoidOutput CollisionAvoider::step(const AvoidSelf & self, const std::vector<Neighbor> & neighbors,
  const CbfParams & p)
{
  AvoidOutput out;
  const Vec3 u_nom = nominalVelocity(self.position, self.goal, p);
  const double sp = std::hypot(u_nom.x, u_nom.y);
  const bool still = sp <= kStillSpeed;
  const double goal_dist = std::hypot(self.goal.x - self.position.x, self.goal.y - self.position.y);
  const double t_goal = still ? p.ttc_horizon : goal_dist / sp;
  const double horizon = std::min(p.ttc_horizon, t_goal);

  // ---------- 鄰機（外推）與要求的最近距離 ----------
  std::vector<Other> others;
  std::set<std::uint16_t> seen;
  for (const Neighbor & n : neighbors) {
    if (n.age > p.neighbor_timeout) {
      continue;
    }
    seen.insert(n.id);
    Other o;
    o.n = &n;
    o.position = n.position + n.velocity * std::min(n.age, p.max_extrapolation);
    const double d3 = norm(self.position - o.position);
    if (out.min_distance < 0.0 || d3 < out.min_distance) {
      out.min_distance = d3;
    }
    o.rx = self.position.x - o.position.x;
    o.ry = self.position.y - o.position.y;
    o.dist = std::hypot(o.rx, o.ry);
    others.push_back(o);
  }
  // 要求的最近距離（讓的對象；其他鄰機只要 safe_radius）：
  //   - miss_clearance；自己沒在動時 ≤ safe_radius + 0.5 m（鄰機的航線幾乎穿過自己才讓）
  //   - 最近點就是自己到目標的那一刻（目標就在鄰機旁，例如起飛點只隔 2 m）：只要 safe_radius——做不到的距離不要求
  //     （2026-10-24：3 m/s 時 miss_clearance 比起飛點間隔大，回起飛點的那台一直繞，兩台在起降板附近相撞）。
  //     最近點在到目標之前（對方從路上經過）照常要 miss_clearance
  //   - 現在已經比要求的近：不要再更近就好（往外、維持距離的方向都可以）
  const bool goal_in_window = !still && t_goal <= p.ttc_horizon;
  auto required = [&](const Cpa & c, const Other & o, bool yield_target) {
      double r = p.safe_radius;
      if (yield_target) {
        r = still ? std::min(p.miss_clearance, p.safe_radius + kStillMargin) :
          (goal_in_window && c.t >= t_goal - 1e-6 ? p.safe_radius : p.miss_clearance);
      }
      return std::min(r, o.dist);
    };

  // ---------- ConflictPredictor＋RightOfWayPolicy＋EncounterState ----------
  for (Other & o : others) {
    const Neighbor & n = *o.n;
    const Cpa c = predictCpa(o.rx, o.ry, u_nom.x - n.velocity.x, u_nom.y - n.velocity.y, horizon);
    // 衝突：正在靠近，而且預測最近距離比要求的近（現在已經更近時，還在靠近就算）
    const bool conflict = c.approaching && c.d < required(c, o, true) + 1e-9 && c.d < p.miss_clearance;
    auto it = encounters_.find(n.id);
    if (it != encounters_.end()) {
      Encounter & e = it->second;
      if (c.approaching) {
        e.separating = 0;
      } else if (e.last_distance >= 0.0 && o.dist > e.last_distance) {
        ++e.separating;
      }
      e.last_distance = o.dist;
      e.t_cpa = c.t;
      e.d_cpa = c.d;
      if (e.separating >= kSeparatingCycles || (!conflict && o.dist > p.yield_exit)) {
        out.events.push_back({AvoidEvent::Kind::RELEASE, n.id, e.self_yields, e.side, c.t, c.d, o.dist, 1.0});
        encounters_.erase(it);
        continue;
      }
    } else if (conflict) {
      Encounter e;
      e.self_yields = !hasRightOfWay(self.key, {n.task_hover, n.busy, n.id});
      e.last_distance = o.dist;
      e.t_cpa = c.t;
      e.d_cpa = c.d;
      it = encounters_.emplace(n.id, e).first;
      out.events.push_back({AvoidEvent::Kind::START, n.id, e.self_yields, 0, c.t, c.d, o.dist, 1.0});
    } else {
      continue;
    }
    o.yields = it->second.self_yields;
  }
  for (auto it = encounters_.begin(); it != encounters_.end(); ) {
    if (seen.count(it->first) == 0) {
      out.events.push_back({AvoidEvent::Kind::RELEASE, it->first, it->second.self_yields, it->second.side,
          0.0, -1.0, -1.0, 1.0});
      it = encounters_.erase(it);
    } else {
      ++it;
    }
  }

  // ---------- AvoidancePlanner（VO）：只有讓的那台 ----------
  // 候選速度 (cx, cy) 的餘裕：對每台鄰機，預測最近距離 − 要求的距離（required）；取最小
  auto margin = [&](double cx, double cy) {
      double m = std::numeric_limits<double>::infinity();
      for (const Other & o : others) {
        const Cpa c = predictCpa(o.rx, o.ry, cx - o.n->velocity.x, cy - o.n->velocity.y, horizon);
        m = std::min(m, c.d - required(c, o, o.yields));
      }
      return m;
    };
  const bool yielding = std::any_of(others.begin(), others.end(), [](const Other & o) {return o.yields;});
  double ux = u_nom.x, uy = u_nom.y;
  bool vo_ok = true;
  if (!yielding) {
    slowed_ = false;
  } else {
    if (!still) {
      // 鎖定的一側：讓的對象都同一側才照它（不同側時不限制）
      int lock = 0;
      bool mixed = false;
      for (const Other & o : others) {
        const int side = o.yields ? encounters_.at(o.n->id).side : 0;
        if (side != 0) {
          mixed = mixed || (lock != 0 && lock != side);
          lock = side;
        }
      }
      if (mixed) {
        lock = 0;
      }
      const double th0 = std::atan2(u_nom.y, u_nom.x);
      bool found = false;
      int chosen_side = 0;
      double chosen_frac = 1.0;
      // 讓路速度：保持速度找不到時先用它試（名目速度很小時——例如待命的讓開後要飛回待命點——讓得太慢，
      // 對方進到 guard_radius、被 CBF 砍到停下來，2026-10-24 單元測試發現），還找不到才降速
      double escape = kStillYieldFrac * p.max_speed;
      for (const Other & o : others) {
        if (o.yields) {
          escape = std::max(escape, std::hypot(o.n->velocity.x, o.n->velocity.y));
        }
      }
      escape = std::min(p.max_speed, escape);
      std::vector<double> speeds{sp};
      if (escape > sp) {
        speeds.push_back(escape);
      }
      for (double frac : kSpeedLevels) {
        if (frac < 1.0) {
          speeds.push_back(sp * frac);
        }
      }
      for (double s : speeds) {
        const double frac = sp > 1e-9 ? s / sp : 1.0;
        for (int k = 0; k <= kVoSteps && !found; ++k) {
          for (int sign : {-1, 1}) {          // 同樣偏轉角時先試右邊（−1）
            if ((k == 0 && sign == 1) || (k > 0 && lock != 0 && sign != lock)) {
              continue;
            }
            const double th = th0 + sign * k * kVoStep;
            const double cx = s * std::cos(th), cy = s * std::sin(th);
            if (margin(cx, cy) >= 0.0) {
              ux = cx;
              uy = cy;
              chosen_side = k == 0 ? 0 : sign;
              chosen_frac = frac;
              found = true;
              break;
            }
          }
        }
        if (found) {
          break;
        }
      }
      if (found) {
        for (const Other & o : others) {
          if (!o.yields) {
            continue;                       // 沒有交會、或自己直行的鄰機不在 encounters_ 或不用記方向
          }
          Encounter & e = encounters_.at(o.n->id);
          if (e.side == 0 && chosen_side != 0) {
            e.side = chosen_side;
          }
        }
        if (chosen_frac < 1.0 && !slowed_) {
          out.events.push_back({AvoidEvent::Kind::VO_SLOW, 0, true, chosen_side, 0.0, 0.0, 0.0, chosen_frac});
        }
        slowed_ = chosen_frac < 1.0;
      } else {
        // 都找不到：取餘裕最大的（含停下來），交給 CBF 和緊急處理
        vo_ok = false;
        double best = margin(0.0, 0.0);
        ux = uy = 0.0;
        for (double frac : kSpeedLevels) {
          for (int k = -kVoSteps; k <= kVoSteps; ++k) {
            const double th = th0 + k * kVoStep;
            const double cx = sp * frac * std::cos(th), cy = sp * frac * std::sin(th);
            const double m = margin(cx, cy);
            if (m > best) {
              best = m;
              ux = cx;
              uy = cy;
            }
          }
        }
      }
    } else if (margin(0.0, 0.0) >= 0.0) {
      ux = uy = 0.0;                          // 懸停的讓路：留在原地就夠
    } else {
      // 懸停的讓路：往離「最危險的那台的最近點」最近的方向移開，速度只要「剛好來得及讓開」：
      //   1.5 × (要求的距離 − 預測最近距離) / 距離交會的時間，最低 0.2 m/s；這個速度找不到安全方向才逐步加快到 max_speed
      // （2026-10-24 SITL：原本用 max(0.3 × max_speed, 對方的速度) = 3 m/s，起降板上的 uav2 只要讓 1 m，衝出去 4.6 m 再繞回來）
      double worst = std::numeric_limits<double>::infinity(), ax = 1.0, ay = 0.0, gap = 0.0, t_hit = 0.0;
      for (const Other & o : others) {
        if (!o.yields) {
          continue;
        }
        const Cpa c = predictCpa(o.rx, o.ry, -o.n->velocity.x, -o.n->velocity.y, horizon);
        if (c.d < worst) {
          worst = c.d;
          const double len = c.d > 1e-3 ? c.d : std::max(o.dist, 1e-3);
          ax = (c.d > 1e-3 ? c.mx : o.rx) / len;
          ay = (c.d > 1e-3 ? c.my : o.ry) / len;
          gap = std::max(0.0, required(c, o, true) - c.d);
          t_hit = c.t;
        }
      }
      const double s0 = std::min(p.max_speed, std::max(kMinSidestep, 1.5 * gap / std::max(t_hit, 0.5)));
      const double th0 = std::atan2(ay, ax);
      ux = s0 * ax;
      uy = s0 * ay;
      vo_ok = false;
      for (double s = s0; s <= p.max_speed + 1e-9 && !vo_ok; s = (s >= p.max_speed ? 2.0 * p.max_speed : std::min(
          p.max_speed, s * 1.5)))
      {
        for (int k = 0; k <= kVoSteps && !vo_ok; ++k) {
          for (int sign : {-1, 1}) {
            if (k == 0 && sign == 1) {
              continue;
            }
            const double th = th0 + sign * k * kVoStep;
            if (margin(s * std::cos(th), s * std::sin(th)) >= 0.0) {
              ux = s * std::cos(th);
              uy = s * std::sin(th);
              vo_ok = true;
              break;
            }
          }
        }
      }
    }
  }

  // ---------- CBF：只對需要的鄰機加約束 ----------
  // 讓的對象：VO 找不到安全速度、繞完預測還不夠遠、距離 < guard_radius → ρ = 1（全額負責）
  // 直行時的對象、沒有衝突的鄰機：距離 < guard_radius → ρ = 0.5
  // 二階時另外加：剎車距離的餘裕 < kBrakeMargin（速度越快越早加；取代 3 m/s 時加大的 guard_radius）
  std::vector<Neighbor> cbf_n;
  std::vector<double> cbf_rho;
  for (const Other & o : others) {
    Neighbor n = *o.n;
    n.position = o.position;
    // ρ：讓的那台全額負責。其他：二階時對方沒在靠近（往自己的速度 < kStillNeighbor，停著或正在離開）也全額負責——
    // 它不會替你剎那一半（2026-10-24 單元測試：直行的 uav2 以為對方會剎一半，剎太晚，靠到 0.57 m）
    const double toward_me = o.dist > 1e-3 ? (o.rx * n.velocity.x + o.ry * n.velocity.y) / o.dist : 0.0;
    const double rho = (o.yields || (p.second_order && toward_me < kStillNeighbor)) ? 1.0 : 0.5;
    bool active = o.dist < p.guard_radius;
    if (o.yields) {
      const Cpa c = predictCpa(o.rx, o.ry, ux - n.velocity.x, uy - n.velocity.y, horizon);
      active = active || !vo_ok || c.d < required(c, o, true) - kResidualSlack;
    }
    if (p.second_order) {
      active = active || brakingMargin(self.position, self.velocity, n, rho, p) < kBrakeMargin;
    }
    if (active) {
      cbf_n.push_back(n);
      cbf_rho.push_back(rho);
    }
  }
  out.active_neighbors = static_cast<int>(std::max(encounters_.size(), cbf_n.size()));
  double nearest = std::numeric_limits<double>::infinity();
  for (const Other & o : others) {
    nearest = std::min(nearest, o.dist);
  }
  // 緊急的方向：離近的鄰機遠一點（距離平方反比加總）
  auto away = [&](double & ex, double & ey) {
      ex = ey = 0.0;
      for (const Other & o : others) {
        if (o.dist < p.guard_radius || o.dist <= nearest + 1e-9) {
          const double d = std::max(o.dist, 1e-3);
          ex += (o.dist > 1e-3 ? o.rx : 1e-3) / (d * d);
          ey += (o.dist > 1e-3 ? o.ry : 0.0) / (d * d);
        }
      }
      const double len = std::hypot(ex, ey);
      if (len > 1e-9) {
        ex /= len;
        ey /= len;
      }
      return len > 1e-9;
    };
  const double vz = std::clamp(u_nom.z, -p.max_vertical_speed, p.max_vertical_speed);
  bool emergency = false;
  double worst = 0.0;
  if (p.second_order) {
    // 二階：VO 的速度 → 想要的加速度（從平滑層上一個週期的參考速度起算、限制在 accel_max）→ CBF 過濾加速度 →
    // 積分回速度（平滑層的加速度上限就是 accel_max，不會再改掉這個結果）
    const double dt = std::max(self.dt, 1e-3);
    double arx = (ux - self.v_ref.x) / dt, ary = (uy - self.v_ref.y) / dt;
    const double an = std::hypot(arx, ary);
    if (an > p.accel_max) {
      arx *= p.accel_max / an;
      ary *= p.accel_max / an;
    }
    std::vector<Cbf2Pair> pairs;
    for (std::size_t k = 0; k < cbf_n.size(); ++k) {
      pairs.push_back({cbf_n[k], cbf_rho[k]});
    }
    const Cbf2Result f = cbf2Filter(self.position, self.velocity, {arx, ary, 0.0}, pairs, p);
    worst = f.worst;
    emergency = nearest < p.safe_radius || !f.feasible;
    double ax = f.accel.x, ay = f.accel.y;
    if (emergency) {
      // 不用投影的結果：最大加速度、對所有鄰機的約束違反最少的方向（不是只看最近那台：最近的是停著的鄰機時，
      // 會被推向另一台高速飛來的，2026-10-24 單元測試 0.31 m）
      std::vector<Cbf2Pair> all;
      for (const Other & o : others) {
        Neighbor n = *o.n;
        n.position = o.position;
        const double toward_me = o.dist > 1e-3 ? (o.rx * n.velocity.x + o.ry * n.velocity.y) / o.dist : 0.0;
        all.push_back({n, (o.yields || toward_me < kStillNeighbor) ? 1.0 : 0.5});
      }
      const Vec3 a = cbf2Escape(self.position, self.velocity, all, p);
      ax = a.x;
      ay = a.y;
    }
    Vec3 v{self.v_ref.x + ax * dt, self.v_ref.y + ay * dt, vz};
    const double hs = std::hypot(v.x, v.y);
    if (hs > p.max_speed) {
      v.x *= p.max_speed / hs;
      v.y *= p.max_speed / hs;
    }
    out.velocity = v;
  } else {
    std::vector<CbfPair> pairs;
    for (std::size_t k = 0; k < cbf_n.size(); ++k) {
      // 一階照原本：讓的對象 λ = 1、ρ = 1，其他 λ = 0、ρ = 0.5（上面的「沒在靠近就全額」只用在二階）
      pairs.push_back({cbf_n[k], cbf_rho[k] >= 1.0 ? 1.0 : 0.0, cbf_rho[k]});
    }
    const CbfFilterResult f = cbfFilter(self.position, {ux, uy, u_nom.z}, pairs, p);
    worst = f.worst;
    emergency = nearest < p.safe_radius || !f.feasible;
    if (emergency) {
      double ex = 0.0, ey = 0.0, closing = 0.0;
      for (const Other & o : others) {
        if (o.dist <= nearest + 1e-9 && o.dist > 1e-3) {
          // 最近那台往自己靠近的速度（r 從對方指向自己）
          closing = std::max(closing, (o.rx * o.n->velocity.x + o.ry * o.n->velocity.y) / o.dist);
        }
      }
      const bool ok = away(ex, ey);
      const double v = std::min(p.max_speed, std::max(p.emergency_speed, closing));
      out.velocity = {ok ? ex * v : 0.0, ok ? ey * v : 0.0, vz};
    } else {
      out.velocity = f.velocity;
    }
  }
  if (emergency) {
    out.active_neighbors = std::max(out.active_neighbors, 1);
  }
  if (emergency != emergency_) {
    AvoidEvent e;
    e.kind = emergency ? AvoidEvent::Kind::EMERGENCY_ON : AvoidEvent::Kind::EMERGENCY_OFF;
    e.distance = std::isfinite(nearest) ? nearest : -1.0;
    e.worst = worst;
    out.events.push_back(e);
    emergency_ = emergency;
  }
  out.emergency = emergency;
  return out;
}

}  // namespace uav_px4_bt
