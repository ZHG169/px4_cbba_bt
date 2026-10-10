// CPF：無人機之間的避碰（人工位勢場＋環流項），飛行層使用，和 CBBA 無關（2026-10-09）
//
// 每個控制週期由目前位置、目標、鄰機的位置與速度算出速度命令（map ENU，m/s），交給 PX4 的 offboard 速度控制：
//
//   吸引  v_att = k_att × (目標 − 自己)，大小限制在 max_speed（接近目標時自然減速，到了就是位置保持）
//   排斥  對每台鄰機，先預測 horizon 內最接近時的相對位置 d（考慮雙方速度，對向飛時提早反應）；
//         |d| < influence_radius 時
//           大小 = k_rep × (1/|d| − 1/R) / (1/safe_radius − 1/R)   （|d| = safe_radius 時等於 k_rep，越近越大）
//           方向 = d / |d|（遠離鄰機）
//   環流  排斥方向的水平分量轉 90°、指向「面向鄰機時的右手邊」，大小 = k_tan × 排斥大小
//         單純位勢場在「正對面飛來」或「目標在鄰機後面」時排斥和吸引抵銷，卡在原地（局部極小值）。
//         全隊都往自己的右邊繞，兩台對飛時會往相反方向錯開，不會同時往同一邊閃
//   合成  v = v_att + Σ 排斥 + Σ 環流；垂直速度限制在 max_vertical_speed，總和限制在 max_speed
//
// 鄰機資料來自機間的位置廣播（flight_beacon，10 Hz）：位置用收到時的速度外推 age 秒（最多 max_extrapolation），
// 超過 neighbor_timeout 沒更新的不算（交給上層：鄰機失聯時只能靠高度分層）。
#pragma once

#include <cstdint>
#include <vector>

namespace uav_px4_bt
{

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline Vec3 operator+(const Vec3 & a, const Vec3 & b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
inline Vec3 operator-(const Vec3 & a, const Vec3 & b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
inline Vec3 operator*(const Vec3 & a, double k) {return {a.x * k, a.y * k, a.z * k};}
double norm(const Vec3 & v);
double dot(const Vec3 & a, const Vec3 & b);

struct Neighbor
{
  std::uint16_t id{0};
  Vec3 position;      // map ENU
  Vec3 velocity;
  double age{0.0};    // 資料的年齡（秒）：收到之後經過的時間
  bool busy{false};   // 正在執行任務（前往或停留）：待命的要讓路給它
};

struct CpfParams
{
  double max_speed{5.0};            // 和 cbba_node 的 cruise_speed 一致，出價估的時間才準
  double max_vertical_speed{2.0};
  double k_att{1.0};                // 1/s：離目標 1 m 時 1 m/s
  double influence_radius{6.0};     // R：鄰機在這個距離內才排斥
  double safe_radius{2.5};          // 期望保持的最小距離（x500 約 0.7 m＋位置誤差＋反應時間）
  double k_rep{4.0};                // m/s：距離 = safe_radius 時的排斥速度
  double max_repulsion{8.0};        // 單一鄰機的排斥上限（太近時不要暴衝）
  double k_tan{0.8};                // 環流／排斥
  double horizon{1.0};              // 預測最接近的時間範圍（秒）
  double max_extrapolation{0.5};    // 鄰機位置最多外推幾秒
  double neighbor_timeout{1.0};     // 超過就不算
};

struct CpfOutput
{
  Vec3 velocity;
  double min_distance{-1.0};        // 目前離最近鄰機的距離（沒有鄰機時 −1），log 用
  int active_neighbors{0};          // 在影響範圍內的鄰機數
};

CpfOutput cpfVelocity(
  const Vec3 & position, const Vec3 & velocity, const Vec3 & goal,
  const std::vector<Neighbor> & neighbors, const CpfParams & params);

}  // namespace uav_px4_bt
