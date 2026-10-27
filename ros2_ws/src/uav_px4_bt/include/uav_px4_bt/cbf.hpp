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
//
// 2026-10-24：正式 BT（UavContext::moveTo）改用 avoidance.hpp 的 CollisionAvoider（預測衝突 → 路權 → VO 繞行 →
// cbfFilter 只對需要的鄰機加約束）；cbfVelocity（約束一直開著＋往右繞）保留給單元測試和對照。
// 共用的部分拆成 nominalVelocity（名目速度）、cbfFilter（指定鄰機與 λ、ρ 的約束＋投影）。
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
  bool task_hover{false};   // 在任務點懸停（BT 已經到點、正在停留／辨識／監控；位置廣播 flags bit 3，2026-10-24）
};

struct CbfParams
{
  double max_speed{5.0};            // 和 cbba_node 的 cruise_speed 一致，出價估的時間才準
  double max_vertical_speed{2.0};
  double k_att{1.0};                // 1/s：離目標 1 m 時 1 m/s
  double brake_accel{1.2};          // m/s²：接近目標時的減速度（剎車曲線，和運動平滑層的加速度上限配合）
  double brake_vertical_accel{0.5}; // m/s²：垂直方向的剎車曲線
  double safe_radius{1.0};          // d_s（2026-10-24 預設改成和 yaml 一樣的 1.0）：h = |p − p_j|² − d_s²，水平距離不小於這個。要比出生點間隔（2 m）小，否則一起飛就在不安全區
  double gamma{1.0};                // γ（1/s）：ḣ ≥ −γ h。大 = 允許靠得快、晚剎；要 ≤ 2 × motion.max_accel / (ρ × max_speed)
  double influence_radius{21.0};    // 只對這個水平距離內的鄰機加約束；要 ≥ (V + √(V² + (ρ γ d_s)²)) / (ρ γ)（ρ = 0.5），
                                    // 在這個距離允許的靠近速度才 ≥ max_speed，否則一進來就急剎
  double yield_radius{2.0};         // （只有 cbfVelocity 用；預設 2026-10-24 改成和 yaml 一樣）讓路時（待命對執行中）保持的水平距離；> safe_radius。執行中的鄰機停下來（懸停做任務）時
                                    // 待命的也會被推到這個距離外
  double k_tan{0.8};                // 往右繞：名目速度違反約束時加「違反量 × k_tan」的右手邊分量（0 = 不繞，正對面會停住）
  double max_extrapolation{0.5};    // 鄰機位置最多外推幾秒
  double neighbor_timeout{1.0};     // 超過就不算
  // ---------- 避碰的路權與繞行（avoidance.hpp，2026-10-24）----------
  double ttc_horizon{5.0};          // 預測最近點只看這麼多秒內（也不超過自己到目標的時間）
  double miss_clearance{1.5};       // 預測最近距離 < 這個才算衝突；讓的那台用 VO 繞到至少這麼遠
  double guard_radius{1.3};         // 現在距離 < 這個時，不管預測、不管路權，CBF 一定對這台鄰機加約束
  double yield_exit{2.0};           // 衝突解除：預測不衝突而且距離 > 這個（或已經通過、距離一直變大）
  double emergency_speed{0.3};      // 一階：緊急退讓（距離 < safe_radius 或 CBF 無解）的水平速度（m/s）
  // ---------- 二階 CBF（2026-10-24）----------
  bool second_order{true};          // true：CBF 管加速度（考慮剎車距離）；false：一階（管速度，假設 ṗ = u）
  double accel_max{1.5};            // 水平加速度上限（m/s²）＝ motion.max_accel（declareConfig 自動帶入，兩個要一樣）
  double response_delay{0.3};       // 從下命令到真的開始剎的延遲（s：PX4 追 setpoint、位置廣播）；二階的安全距離加上
                                    // 延遲 × 靠近速度（2026-10-24 單元測試：3 m/s 時延遲多飛 0.9 m，靠到 0.57 m）
};

struct CbfOutput
{
  Vec3 velocity;
  double min_distance{-1.0};        // 目前離最近鄰機的距離（3D；沒有鄰機時 −1），log 用
  int active_neighbors{0};          // 約束有作用（名目速度會違反）的鄰機數；> 0 時運動平滑層不直接位置保持
};

// 名目速度：往目標、速度 = min(max_speed, k_att × 距離, √(2 × brake_accel × 距離))，垂直照 brake_vertical_accel 縮小
Vec3 nominalVelocity(const Vec3 & position, const Vec3 & goal, const CbfParams & params);

// cbfFilter 的一個約束：鄰機（位置已經外推）、λ、ρ（a·u ≥ λ a·v_j − ρ γ h，h 用 safe_radius）
struct CbfPair
{
  Neighbor neighbor;
  double lambda{0.0};
  double rho{0.5};
};

struct CbfFilterResult
{
  Vec3 velocity;
  bool feasible{true};              // 投影＋速度上限之後，所有約束都成立
  double worst{0.0};                // 最大的違反量（m/s，≤ 0 = 都成立）
};

// u_ref 投影到所有約束的交集（水平，Dykstra）；垂直限制在 max_vertical_speed，總和限制在 max_speed。
// 沒有約束時就是 u_ref（加上速度上限）
CbfFilterResult cbfFilter(const Vec3 & position, const Vec3 & u_ref, const std::vector<CbfPair> & pairs,
  const CbfParams & params);

// ---------- 二階 CBF（2026-10-24）----------
// 模型 ṗ = v、v̇ = a、|a| ≤ accel_max（水平）。對每台鄰機（位置已經外推）：
//   d = |p − p_j|、n = (p − p_j) / d、ḋ = n·(v − v_j)（< 0 = 在靠近）、A = accel_max / ρ（相對減速度：ρ = 1 只有自己剎、
//   ρ = 0.5 兩台各剎一半 → 2 × accel_max）
//   h = √(2 A (d − r)) + ḋ          照 A 全力剎車，靠近速度在剩下的距離內剎得住才 ≥ 0
//                                   r = safe_radius ＋ response_delay × 靠近速度（反應延遲期間還會往前飛）
//   ḣ = n·(a − a_j) + (|v − v_j|² − ḋ²) / d + A ḋ / √(2 A (d − r))
//   約束 ḣ ≥ −γ h，鄰機的加速度 a_j 當 0、自己負責 ρ：  n·a ≥ ρ [−(|Δv|² − ḋ²) / d − A ḋ / √(2A(d − r)) − γ h]
// 速度越快，h 越早變小、越早開始剎（一階只看距離，3 m/s 時剎不住）
struct Cbf2Pair
{
  Neighbor neighbor;
  double rho{0.5};
};

struct Cbf2Result
{
  Vec3 accel;                       // 水平（z = 0）
  bool feasible{true};              // 投影＋加速度上限之後，所有約束都成立（d ≤ safe_radius 時一定是 false）
  double worst{0.0};                // 最大的違反量（m/s²）
};

// a_ref（水平）投影到所有約束和 |a| ≤ accel_max 的交集（Dykstra）
Cbf2Result cbf2Filter(const Vec3 & position, const Vec3 & velocity, const Vec3 & a_ref,
  const std::vector<Cbf2Pair> & pairs, const CbfParams & params);

// 緊急時的加速度（2026-10-24）：在 |a| = accel_max 的一圈方向（每 7.5°）裡，挑對所有鄰機的約束違反最少的
// （max over a of min_j (n_j·a − b_j)；已經在安全距離內的鄰機當成剛好在邊上，b 會很大）。
// 原本只往「遠離最近那台」退：最近的是停著的鄰機時，會被推向另一台高速飛來的（單元測試 0.31 m）
Vec3 cbf2Escape(const Vec3 & position, const Vec3 & velocity, const std::vector<Cbf2Pair> & pairs,
  const CbfParams & params);

// 剎車距離的餘裕：d − r − 靠近速度² / (2A)（A = accel_max / ρ）；< 0 = 現在開始全力剎也來不及
double brakingMargin(const Vec3 & position, const Vec3 & velocity, const Neighbor & n, double rho,
  const CbfParams & params);

// yield_to_busy：自己在待命、起飛（讓路給執行任務中的鄰機）
CbfOutput cbfVelocity(
  const Vec3 & position, const Vec3 & goal, const std::vector<Neighbor> & neighbors, const CbfParams & params,
  bool yield_to_busy = false);

}  // namespace uav_px4_bt
