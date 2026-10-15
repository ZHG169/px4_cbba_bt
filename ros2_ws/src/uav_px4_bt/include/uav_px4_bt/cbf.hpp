// CBF：無人機之間的避碰（控制障礙函數，Control Barrier Function），飛行層使用，和 CBBA 無關（2026-10-14 取代 CPF；
// CPF（人工位勢場）的原始碼備份在 docs/archive/cpf_20261014/）
//
// 每個控制週期由目前位置、目標、鄰機的位置與速度算出速度命令（map ENU，m/s），交給運動平滑層、再給 PX4：
//
//   名目速度  u_nom：方向朝目標，大小 = min(max_speed, k_att × 距離, √(2 × brake_accel × 距離))；垂直分量另外照
//            brake_vertical_accel 的剎車曲線限制（整個向量等比例縮小，方向不變）。和 CBF 的吸引相同
//   障礙函數  對每台 influence_radius 內的鄰機 j（只看水平；全隊飛同一個高度層，垂直閃避沒有好處，CBF 2026-10-13 的教訓）：
//              h_j = |p − p_j|² − safe_radius²          （h ≥ 0 = 安全）
//            要求  ḣ_j ≥ −γ h_j，ḣ_j = 2 (p − p_j)·(u − v_j)
//            分攤（兩台都跑同一套，各負責一部分）：
//              a_j·u ≥ b_j，a_j = 2 (p − p_j)，b_j = λ_j a_j·v_j − ρ_j γ h_j
//              同樣狀態（都在執行或都在待命）  λ = 0、ρ = 0.5（互相各讓一半；不看對方速度，否則兩台都全額補償會來回擺）
//              自己待命、鄰機執行中（讓路）    λ = 1、ρ = 1，而且 h 改用 yield_radius（> safe_radius）：
//                                            把它當不會讓的障礙物、全額閃開，讓出的距離比 safe_radius 大。
//                                            只用 safe_radius 時，執行中的那台自己也減速（ρ = 0.5），兩台在 safe_radius 外
//                                            互等，它到不了停在待命機位置上的任務點（2026-10-14 單元測試發現）
//              自己執行中、鄰機待命           λ = 0、ρ = 0.5（對方會全額讓；自己仍留一半，對方飛不動時還有保障）
//            只限制「往鄰機靠近」的速度，平行飛、遠離不受影響；懸停的不會被推（u = 0 在 h ≥ 0 時滿足）
//   往右繞    約束被名目速度違反時（u_nom 會撞上去），名目速度先加 k_tan × 違反量 的「面向鄰機的右手邊」分量：
//            單純的 CBF 在正對面飛來時只會減速到停（死結），全隊都往自己的右邊繞，兩台對飛時往相反方向錯開（同 CBF 的環流）
//   求解      u = argmin |u − u_nom'|²，s.t. 所有 a_j·u ≥ b_j（水平 2D，Dykstra 交替投影；鄰機只有幾台，迭代很快）；
//            垂直速度用名目值、限制在 max_vertical_speed；總和限制在 max_speed（等比例縮小，h ≥ 0 時仍滿足約束）
//
// 鄰機資料來自機間的位置廣播（flight_beacon，10 Hz）：位置用收到時的速度外推 age 秒（最多 max_extrapolation），
// 超過 neighbor_timeout 沒更新的不算。
// 限制：速度命令還要經過運動平滑層（加速度上限）與 PX4，實際速度會落後；γ 要配合加速度上限（doc/parameters.md 第五節）。
// 不跑這套的鄰機（PX4 自己 RTL、手動）也被當成會分攤一半。
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

struct CbfParams
{
  double max_speed{5.0};            // 和 cbba_node 的 cruise_speed 一致，出價估的時間才準
  double max_vertical_speed{2.0};
  double k_att{1.0};                // 1/s：離目標 1 m 時 1 m/s
  double brake_accel{1.2};          // m/s²：接近目標時的減速度（剎車曲線，和運動平滑層的加速度上限配合）
  double brake_vertical_accel{0.5}; // m/s²：垂直方向的剎車曲線
  double safe_radius{1.2};          // d_s：h = |p − p_j|² − d_s²，水平距離不小於這個。要比出生點間隔（2 m）小，否則一起飛就在不安全區
  double gamma{1.0};                // γ（1/s）：ḣ ≥ −γ h。大 = 允許靠得快、晚剎；要 ≤ 2 × motion.max_accel / (ρ × max_speed)
  double influence_radius{21.0};    // 只對這個水平距離內的鄰機加約束；要 ≥ (V + √(V² + (ρ γ d_s)²)) / (ρ γ)（ρ = 0.5），
                                    // 在這個距離允許的靠近速度才 ≥ max_speed，否則一進來就急剎
  double yield_radius{2.5};         // 讓路時（待命對執行中）保持的水平距離；> safe_radius。執行中的鄰機停下來（懸停做任務）時
                                    // 待命的也會被推到這個距離外
  double k_tan{0.8};                // 往右繞：名目速度違反約束時加「違反量 × k_tan」的右手邊分量（0 = 不繞，正對面會停住）
  double max_extrapolation{0.5};    // 鄰機位置最多外推幾秒
  double neighbor_timeout{1.0};     // 超過就不算
};

struct CbfOutput
{
  Vec3 velocity;
  double min_distance{-1.0};        // 目前離最近鄰機的距離（3D；沒有鄰機時 −1），log 用
  int active_neighbors{0};          // 約束有作用（名目速度會違反）的鄰機數；> 0 時運動平滑層不直接位置保持
};

// yield_to_busy：自己在待命、起飛（讓路給執行任務中的鄰機）
CbfOutput cbfVelocity(
  const Vec3 & position, const Vec3 & goal, const std::vector<Neighbor> & neighbors, const CbfParams & params,
  bool yield_to_busy = false);

}  // namespace uav_px4_bt
