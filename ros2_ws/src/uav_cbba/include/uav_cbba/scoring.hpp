// 火場版的成本與分數計算
//
// 成本沿用 Velhal 2022 的「空間成本 x 時間成本」形式，再乘上電池因子：
//   每個任務   cost_j = Ls_j * Lt_j
//              Ls_j   = 從前一個位置飛到任務點的距離
//              Lt_j   = 到達時間 / 距離期限剩餘的時間（1 以上表示逾期）
//   整條路徑   C_ST    = sum(cost_j)
//              C_total = C_ST * (1 + battery_weight * 路徑耗能 / 可用電量)
//   電量不足以做完並返航時，路徑不可行。
//
// 對外使用分數（越高越好，0 表示做不到）：
//   score = value / (1 + 邊際成本 / cost_ref)
#pragma once

#include <cstddef>
#include <vector>

#include "uav_cbba/types.hpp"

namespace uav_cbba
{

struct ScoringParams
{
  double cruise_speed{5.0};      // 標稱速度 (m/s)；載具自己有設定時以載具的為準
  double speed_margin{0.8};      // 規劃時用 cruise_speed * speed_margin，預留餘裕
  double battery_weight{1.0};    // 電池因子的權重；1 表示用完可用電量的路徑成本變兩倍
  double cost_ref{50.0};         // 成本轉分數的基準（公尺）
  double min_deadline{1.0};      // 剩餘期限的下限（秒），避免除以零
  double score_epsilon{1e-3};    // 兩個分數相差在此以內視為平手
  std::size_t max_bundle{5};     // 一次最多接幾個任務
  double rebid_threshold{0.1};   // 重新評估時，分數變動超過此比例才更新
};

struct LegEval
{
  TaskId task_id{0};
  double distance{0.0};            // Ls
  double arrival{0.0};             // 到達時間（從現在起算，秒）
  double completion{0.0};          // 完成時間（從現在起算，秒）
  double remaining_deadline{0.0};  // 距離期限還有多久（秒）
  double temporal{0.0};            // Lt = arrival / remaining_deadline
  double cost{0.0};                // Ls * Lt
  bool overdue{false};             // 到達時已超過期限
};

struct PathEval
{
  bool feasible{true};       // 目前只有電量會造成不可行（期限是軟限制）
  double st_cost{0.0};       // C_ST
  double path_energy{0.0};   // 含回程
  double battery_after{0.0}; // 做完並返航後的電量
  double battery_cost{0.0};  // 路徑耗能 / 可用電量
  double total_cost{0.0};    // C_total；不可行時為無限大
  std::vector<LegEval> legs;
};

// 依給定順序執行 tasks 的成本與可行性
PathEval evaluatePath(
  const AgentState & agent, const std::vector<Task> & ordered_tasks, double now,
  const ScoringParams & params);

// 邊際成本轉分數；不可行時回傳 0
double costToScore(double value, double marginal_cost, double cost_ref);

struct Insertion
{
  bool feasible{false};
  std::size_t position{0};      // 插入 path 的位置
  double marginal_cost{kInf};
  double score{0.0};
  PathEval eval;                // 插入後整條路徑的評估
};

// 嘗試把 task 插入 path 的每個位置，回傳邊際成本最小的可行位置。
// 載具類型不相容、任務不是 OPEN、或沒有可行位置時，feasible 為 false、score 為 0。
Insertion bestInsertion(
  const AgentState & agent, const std::vector<Task> & path, const Task & task, double now,
  const ScoringParams & params);

}  // namespace uav_cbba
