// 無人機之間的避碰（2026-10-24 定案）：每台鄰機先預測會不會碰，會碰才決定誰讓、讓的那台用 VO 繞，最後 CBF 保底。
// 和 CBBA、機器狗無關（只用位置廣播的資料：機號、位置、速度、busy、task_hover）。
//
//   ConflictPredictor  有限時間的最近點（CPA）：自己用名目速度 u_nom（意圖）、鄰機用廣播的速度
//                        r = p − p_j，v_r = u − v_j，t* = clamp(−r·v_r / |v_r|², 0, T)，d* = |r + v_r t*|
//                        T = min(ttc_horizon, 自己到目標的時間)（自己沒在動時 T = ttc_horizon）
//                      正在靠近而且 d* < 要求的距離才算衝突。要求的距離 = miss_clearance；最近點就是自己到目標的
//                      那一刻（目標就在鄰機旁）只要 safe_radius（不要求做不到的距離）；自己沒在動時 ≤ safe_radius + 0.5 m；
//                      現在已經比要求的近時，不要再更近就好（VO 才找得到往外的方向）
//   RightOfWayPolicy   路權鍵 K = (task_hover, busy, −機號)，大的直行、小的讓；兩台用同一份位置廣播算，結果一定唯一
//   EncounterState     每台鄰機一筆：誰讓、往哪邊繞。開始後不換角色（交會途中任務完成、busy 改變、速度變 0 都不換），
//                      解除：已經通過而且距離連續變大，或預測不衝突而且距離 > yield_exit，或沒有位置廣播
//   AvoidancePlanner   讓的那台：VO（Velocity Obstacle）找離 u_nom 最近的安全速度——先保持速度只轉方向，左右都行時選
//                      偏轉小的，一樣時往自己的右邊；選定後鎖定那一側；保持速度找不到時才降速（75、50、25 %）；
//                      自己沒在動（懸停的讓路）時，能留在原地就不動，不能才往旁邊移。讓行不是設成零速度
//   CBF（二階 cbf2Filter，預設；或一階 cbfFilter）只對需要的鄰機加約束：
//                        讓的那台：VO 找不到安全速度、或距離 < guard_radius → λ = 1、ρ = 1（算進對方的速度、全額負責）
//                        直行的那台、沒有衝突的鄰機：只有距離 < guard_radius 才加 → λ = 0、ρ = 0.5
//                      預測不會碰的鄰機不加約束（原本的 CBF 一直開著：錯開 2.36 m 也被砍速度）
//                      二階（2026-10-24）：CBF 管加速度（ṗ = v、v̇ = a、|a| ≤ accel_max），h = √(2A(d − r)) + ḋ（照最大減速度
//                      剎得住）；VO 的速度先換成加速度（從平滑層的參考速度起算），過濾後積分回速度。另外剎車距離的餘裕
//                      < 0.5 m 時也加約束（速度越快越早，取代一階時 3 m/s 要加大的 guard_radius）
//   緊急               距離 < safe_radius，或 CBF 投影後還有約束不成立 → 不用投影的結果：二階時用最大加速度、
//                      對所有鄰機的約束違反最少的方向（cbf2Escape）；一階時往外退，速度 = max(emergency_speed, 最近那台往自己靠近的速度)（高度不變）
//
// 和 cbfVelocity（cbf.hpp）不同：那個對每台鄰機一直加約束、約束違反後才往右繞（看起來是停下來等人過）。
#pragma once

#include <cstdint>
#include <map>
#include <vector>

#include "uav_px4_bt/cbf.hpp"

namespace uav_px4_bt
{

// ===========================================================================
// ConflictPredictor
// ===========================================================================
struct Cpa
{
  bool approaching{false};  // 正在靠近（−r·v_r > 0，而且有相對運動）
  double t{0.0};            // t*（s）
  double d{0.0};            // d*（m，水平）
  double mx{0.0};           // 最近點上：對方指向自己（水平）
  double my{0.0};
};

// r：自己 − 鄰機（水平）；v_rel：自己的速度 − 鄰機的速度；horizon：T
Cpa predictCpa(double rx, double ry, double vx, double vy, double horizon);

// ===========================================================================
// RightOfWayPolicy
// ===========================================================================
struct RightOfWayKey
{
  bool task_hover{false};
  bool busy{false};
  std::uint16_t id{0};
};

// 自己有路權（直行）：K = (task_hover, busy, −機號) 比較大
bool hasRightOfWay(const RightOfWayKey & self, const RightOfWayKey & other);

// ===========================================================================
// CollisionAvoider（EncounterState、AvoidancePlanner、CBF、緊急）
// ===========================================================================
struct AvoidSelf
{
  Vec3 position;
  Vec3 goal;
  RightOfWayKey key;
  Vec3 velocity;            // 實際量到的速度（二階 CBF 的安全計算用）
  Vec3 v_ref;               // 平滑層上一個週期的參考速度（二階：加速度從這裡積分，送出去的命令才平順）
  double dt{0.05};          // 控制週期（s）
};

struct AvoidEvent
{
  enum class Kind : std::uint8_t { START, RELEASE, VO_SLOW, EMERGENCY_ON, EMERGENCY_OFF };
  Kind kind{Kind::START};
  std::uint16_t neighbor{0};
  bool self_yields{false};
  int side{0};              // +1 往自己的左邊、−1 往右（START 時還沒決定是 0，VO 決定後的 log 在第一次繞時）
  double t_cpa{0.0};
  double d_cpa{0.0};
  double distance{0.0};     // 當時的水平距離；< 0 = 沒有位置廣播
  double speed_frac{1.0};   // VO_SLOW：降到名目速度的幾成
  double worst{0.0};        // EMERGENCY_ON：CBF 投影後最大的違反量（m/s）
};

struct AvoidOutput
{
  Vec3 velocity;
  double min_distance{-1.0};        // 離最近鄰機的距離（3D；沒有鄰機 −1），log 用
  int active_neighbors{0};          // 正在交會（有衝突）或有 CBF 約束的鄰機數；> 0 時運動平滑層不直接位置保持
  bool emergency{false};
  std::vector<AvoidEvent> events;
};

class CollisionAvoider
{
public:
  AvoidOutput step(const AvoidSelf & self, const std::vector<Neighbor> & neighbors, const CbfParams & params);

  struct Encounter
  {
    bool self_yields{false};
    int side{0};                    // 讓的那台繞的方向（+1 左、−1 右、0 還沒決定）
    double last_distance{-1.0};
    int separating{0};              // 已經通過、距離連續變大的週期數
    double t_cpa{0.0};
    double d_cpa{0.0};
  };
  const std::map<std::uint16_t, Encounter> & encounters() const {return encounters_;}

private:
  std::map<std::uint16_t, Encounter> encounters_;
  bool emergency_{false};
  bool slowed_{false};              // VO 保持速度找不到、降速中（log 只在開始時印）
};

}  // namespace uav_px4_bt
