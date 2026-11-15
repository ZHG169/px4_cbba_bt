// 出價：時間折扣報酬（TDR；M1，2026-11-15；設計與數值例子見 doc/formula/tdr_design.md）
//
// CBBA 原論文（Choi, Brunet, How 2009）的報酬形式：任務的價值隨到達時間打折，出價是「加入這個任務後整條路徑的報酬增加多少」。
//   路徑報酬   S(p) = Σ_j  p_j × v_j × 2^(−τ_j / h_j)  −  w_E × E(p) / (電量 − 存量)
//   出價       c_j  = max_n [ S(p ⊕_n j) − S(p) ]；≤ 0 不出價（做這個任務不划算）
//     v_j   任務的價值
//     τ_j   照這條路徑到達 j 的時間（從現在起算）：距離 / (速度 × speed_margin)，加上 release 的等待、失敗過的期望重試時間
//     h_j   half_life：任務的 ext.half_life_s，沒帶用 half_life_by_priority[P]（P0～P3）；0 = 不打折
//     p_j   這台做 j 的估計成功率 1 / (1 + TRANSIENT 失敗次數)（2026-10-17；只在本機）
//     E(p)  路徑耗電（含返航，energy_model.hpp）；存量 = max(載具的 safety_reserve, 路徑上各任務的 min_battery_reserve)
//     w_E   energy_weight：用掉全部可用電量相當於少多少分（和價值同單位）
//   插入的任務讓路徑上其他任務晚到，它們少掉的報酬直接算在出價裡（舊公式只算時間成本的變化）。
//
// 可行性（不可行 = 出價 0）：平台與能力（capabilities.hpp 的 fitness）、做完返航後電量 ≥ 存量、
//   硬期限（ext.deadline_hard 且有期限）來不及；執行中的任務（exempt）不檢查硬期限。軟期限不影響分數（「急」由 half_life 表達）。
//
// 失敗過的任務（2026-10-17，宜臻的實作說明需求 1）：期望價值 p × v；期望到達時間多 (1 − p)/p ×（前往＋執行），
//   路徑後面的任務一起延後；執行中、有 BT 剩餘時間（head_remaining）的第一個任務不加。
//
// 取代 2026-10-05 的「空間成本 × 時間成本 × 電池因子」（doc/formula/formula_design.md，舊版）：
//   cost_ref、battery_weight、time_ref、min_deadline 拿掉
#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <vector>

#include "cbba_core/types.hpp"

namespace cbba_core
{

struct ScoringParams
{
  double cruise_speed{5.0};      // 標稱速度 (m/s)；載具自己有設定時以載具的為準
  double speed_margin{0.8};      // 估到達時間用 cruise_speed * speed_margin，預留餘裕
  double energy_weight{50.0};    // w_E：用掉全部可用電量相當於少多少分（暫定，docs/sim_scenario_plan.md 第八節的實驗調）
  // 任務沒帶 half_life 時依優先級 P0～P3 的預設（秒）；0 = 不打折
  std::array<double, 4> half_life_by_priority{{60.0, 120.0, 300.0, 600.0}};
  double score_epsilon{1e-3};    // 兩個分數相差在此以內視為平手
  std::size_t max_bundle{5};     // 一次最多接幾個任務
  double rebid_threshold{0.1};   // 重新評估時，分數變動超過此比例才更新
  int max_failures{0};           // 同一個任務 TRANSIENT 失敗到這個次數就永久不出價；0 = 不啟用
};

struct LegEval
{
  TaskId task_id{0};
  double distance{0.0};
  double arrival{0.0};             // 到達時間（從現在起算，秒）
  double completion{0.0};          // 完成時間（從現在起算，秒）
  double half_life{0.0};           // 用的 half_life（秒；0 = 不打折）
  double reward{0.0};              // p × v × 2^(−到達 / half_life)
  bool overdue{false};             // 有期限、到達時已超過（硬期限時不可行）
};

struct PathEval
{
  bool feasible{true};             // 電量不夠、或硬期限來不及時不可行
  double reward{0.0};              // Σ 各任務的報酬
  double path_energy{0.0};         // 含回程
  double battery_after{0.0};       // 做完並返航後的電量
  double battery_fraction{0.0};    // 路徑耗電 / 可用電量
  double energy_cost{0.0};         // energy_weight × battery_fraction
  double score{0.0};               // reward − energy_cost；不可行時為 −無限大，空路徑為 0
  std::vector<LegEval> legs;
};

// 這個任務用的 half_life（秒）：任務的 ext.half_life_s，沒有就照優先級的預設；0 = 不打折
double halfLifeOf(const Task & task, const ScoringParams & params);
// 到達時間（從現在起算，秒）時的價值：v × 2^(−到達 / half_life)（不含成功率）；不會超過 v
double decayedValue(const Task & task, double arrival, const ScoringParams & params);

// 依給定順序執行 tasks 的報酬與可行性。
// head_remaining ≥ 0：第一個任務已經在執行，還要這麼多秒才完成（BT 回報的預估），用它代替估計的到達＋執行時間
// exempt：執行中的任務，不檢查它的硬期限（0 = 沒有）
PathEval evaluatePath(
  const AgentState & agent, const std::vector<Task> & ordered_tasks, double now,
  const ScoringParams & params, double head_remaining = -1.0, TaskId exempt = 0);

struct Insertion
{
  bool feasible{false};         // 有可行的插入位置（報酬增加可能 ≤ 0，那時 score 是 0）
  std::size_t position{0};      // 插入 path 的位置
  double gain{-kInf};           // 路徑報酬增加多少
  double score{0.0};            // 出價 = max(gain, 0)
  PathEval eval;                // 插入後整條路徑的評估
};

// 插入位置的限制（執行狀態造成的，見 CbbaAgent::ExecConstraints）
struct InsertionLimits
{
  std::size_t min_position{0};   // 前面是已接受、不能被插隊的任務
  double head_remaining{-1.0};   // 見 evaluatePath
  // 插在最前面＝搶占路徑第一個（執行中、可中斷、優先級較低）：被搶占的任務會交回 CBBA 重新競標，
  // 評分時把它從路徑拿掉（不算「延後它」的損失），否則巡檢中的 V60 對身邊火警的出價永遠比不上閒著、但遠很多的（2026-10-12）
  bool preempt_drops_head{false};
  TaskId exempt{0};              // 執行中的任務：不檢查硬期限（見 evaluatePath）
  // 這個位置可不可以（插入後的路徑評估、位置、插入後的路徑）；沒設定時都可以
  std::function<bool(const PathEval &, std::size_t, const std::vector<Task> &)> allow;
};

// 嘗試把 task 插入 path 的每個位置（min_position 以後、allow 允許的），回傳路徑報酬增加最多的可行位置。
// 平台或能力不符（capabilities.hpp 的 fitness）、任務不是 OPEN、或沒有可行位置時，feasible 為 false、score 為 0。
Insertion bestInsertion(
  const AgentState & agent, const std::vector<Task> & path, const Task & task, double now,
  const ScoringParams & params, const InsertionLimits & limits = InsertionLimits{});

}  // namespace cbba_core
