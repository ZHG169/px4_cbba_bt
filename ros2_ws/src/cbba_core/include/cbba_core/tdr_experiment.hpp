// TDR 參數實驗（2026-11-16，docs/tdr_experiment_plan.md；依使用者的〈TDR 參數設定與測試計畫〉）：
// 情境 S1～S6、參數組（w_E × half_life 倍率 γ）、每次試驗的摘要。離線測試與 cbba_sim tdr 共用，不是機上程式。
//
// 情境（每個 seed 的幾何、電量、耗電誤差不同；同一個 seed 在每個參數組、掉包率都一樣，才能配對比較）：
//   S1 時間差    兩隻狗耗電相同、速度不同（1.5 vs 0.7 m/s）、離任務一樣遠 → 緊急任務（P0／P1）該給快的
//   S2 能量差    兩隻狗速度、距離相同，可用電量不同（35 % vs 90 %）→ 該給電量多的（w_E = 0 時平手比機號，會給錯）
//   S3 時能取捨  A 快但耗電高（1.5 m/s、0.3 %/m）、B 慢但省電（0.7 m/s、0.08 %/m）；P0 該給 A、P3 該給 B
//   S4 路徑插入  狗 A 正在前往 P3 任務，途中旁邊插入 P0；看插入位置、舊任務延後多少、P0 到達時間
//   S5 異質平台  1 架無人機＋1 隻狗，3 個平台不限的任務（P1～P3）；看平台選擇、耗電、完成時間（沒有標準答案）
//   S6 多任務機隊 3 架無人機＋3 隻狗（itest 的位置），12 個一般任務（無人機、狗、不限各 4，P0～P3）＋一組 3 個角色
//               （無人機 1、狗 2，distinct_agent_set）；看工作量分布、換手、總報酬
// 「該給誰」只在 S1～S3 有（MissionTask::expected），是情境設計的判讀，不是 CBBA 的保證
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "cbba_core/mission_sim.hpp"

namespace cbba_core
{

// 基準的預設 half_life（P0～P3，秒）：參數組只乘一個共同倍率 γ，保留各級的相對急迫程度
constexpr std::array<double, 4> kBaseHalfLife{{60.0, 120.0, 300.0, 600.0}};

struct ParamSet
{
  double energy_weight{50.0};   // w_E
  double gamma{1.0};            // half_life = kBaseHalfLife × gamma
};
ScoringParams applyParamSet(ScoringParams base, const ParamSet & set);
std::string halfLifeLabel(double gamma);   // "60/120/300/600"

struct ScenarioOptions
{
  double uav_speed{0.5};        // 和無人機 BT 一致（uav_bt.yaml）
  double uav_endurance{900.0};  // 秒：每秒 100 / 續航、每公尺 = 每秒 / 速度
  double dog_speed{1.0};        // S5、S6 的狗（S1～S4 照情境自己的設定）
  double dog_per_meter{0.1};    // 狗的佔位值（%/m、%/s），等伯宇實測
  double dog_per_sec{0.05};
  double reserve{20.0};
  double drain_error{0.1};      // 實際耗電 = 模型 × (1 + 誤差)，每次、每台在 ±這個比例內隨機
  double t0{3.0};               // 第一批任務的插入時刻（先讓節點完成開機同步）
};

struct ScenarioCase
{
  std::string name;
  std::vector<MissionAgent> agents;
  std::vector<MissionTask> tasks;
  double max_time{900.0};
};

const std::vector<std::string> & scenarioNames();   // S1～S6
// 不認得的名稱丟 std::invalid_argument
ScenarioCase makeScenario(const std::string & name, std::uint32_t seed, const ScenarioOptions & opt = {});

// 一次試驗的摘要（runs.csv 一列）。時間都是秒，−1 = 沒有
struct RunSummary
{
  int tasks{0};
  int completed{0};
  double converge_s{-1.0};      // 最晚收斂的任務的收斂時間；有任務沒收斂時 −1
  int unconverged{0};
  bool conv_5s{false};          // 每個任務都在 5 s 內收斂，而且沒有重複認領
  int duplicate{0};             // 有重複認領的任務數
  double duplicate_time{0.0};
  int handoff{0};
  int handoff_unforced{0};      // 不是因為失敗、返航、搶占的換手
  int preempted{0};
  double score_gap{-1.0};       // 各任務收斂時分數差距的最小值（只算有 2 台以上出價的）
  int near_tie{0};              // 分數差距 < 1 % 的任務數
  int deadline_miss{0};
  double energy_used{0.0};      // 全部載具的耗電總和（%）
  double min_battery{0.0};      // 過程中所有載具的最低電量
  double min_return_margin{0.0};
  double p0_eta{-1.0};          // P0 任務的到達時間（最大值）
  double p1_eta{-1.0};
  double discounted_value{0.0}; // 實際折扣任務價值總和（固定的 eval_half_life）
  int choice_total{0};          // 有 expected 的任務數
  int choice_ok{0};             // 其中第一位執行者 = expected
  int bundle_drops{0};
  int wrong_accepts{0};
  int max_tasks{0};             // 每台完成數的最大、最小（分配均衡）
  int min_tasks{0};
  double old_delay{-1.0};       // 有 ideal_eta 的任務延後的最大值（S4 的舊任務）
  std::string winner_map;       // label:完成者;…（沒完成的是 0）
};
RunSummary summarize(const MissionResult & result);

}  // namespace cbba_core
