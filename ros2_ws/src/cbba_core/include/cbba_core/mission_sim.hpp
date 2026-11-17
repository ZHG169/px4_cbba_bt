// 離線的任務模擬（2026-10-10，docs/sim_scenario_plan.md 的電池權重測試）：測試與分析用，不是機上程式
//
// network_sim 的 negotiate() 只模擬「一開始就給全部任務」的一次協商；這裡模擬整個任務過程：
//   - 每台跑和 cbba_node 一樣的 CbbaComm（協定版本 2 的封包）＋ AssignmentManager（無人機 require_accept = false），
//     封包經過模擬的廣播網路（掉包、延遲）
//   - 假的 BT（和 uav_px4_bt 的行為一致）：飛向 assigned_task、到達後停留 duration、回報 task_result；
//     電量 ≤ critical_battery 時返航（BATTERY_RTL）：手上的任務回報失敗，之後不參與出價（降落）
//
// 「接了沒做完」（wrong_accepts）有兩種，都算：
//   - 返航：電量降到 critical_battery 時手上還有任務
//   - 撤銷：核心每 reevaluate_period 用目前電量重新評估，發現做不完就釋放，執行中的指派被撤銷
//     （通常比返航早發生；無人機的任務都是同優先級、執行中鎖定，所以不會是被搶占或被別台拿走）
//   - 任務照 insert_at 插入（由建立者的 addLocalTask，等於 BT 發 new_task）
//
// 電池照 PX4 SITL 的模擬電池：**依時間線性下降**，飛行、懸停、待命都一樣（SIM_BAT_DRAIN），
// 降到 critical_battery 時 BT 返航（PX4 的 SIM_BAT_MIN_PCT 也停在這裡）。
// 出價用的能量模型（AgentState 的 energy_per_meter、hover_energy_per_sec）只估「飛行＋停留」，
// 不含等待新任務的時間，加上 drain_error（實際耗電 = 模型 × (1 + drain_error)），
// 所以會有「出價時可行、實際上做不完」的情況，這正是電池權重要減少的。
//
// TDR 參數實驗（2026-11-16，docs/tdr_experiment_plan.md）加的量測：
//   - 狗：實際耗電另外有每公尺的部分（drain_per_meter），出價的能量模型照 AgentState
//   - 收斂：插入後，存活的節點（還在參與的）都認得這個任務、得標者一致、得標者的路徑上有它，持續 converge_hold
//     才算；收斂時刻記這段一致的開始（和 itest loss 的「只剩一位持有者」同一件事，只是用核心的狀態直接看）
//   - 收斂當下記每台的出價（第一、第二名、分數差距；record_bids 時加分項明細）
//   - 重複認領：同一個任務同時有兩台以上的假 BT 在執行（正式指派重疊）的時間
//   - 換手：執行者換人的次數；前一位是失敗、返航、被搶占而停下的算「被迫」，其他的是重新競標造成的
//   - 搶占：執行中的任務被較高優先級的換掉，不算接錯（原本假設不會發生，異質情境會）
//   - 實際折扣價值：完成的任務 v × 2^(−(到達 − 插入) / h)，h 用固定的 eval_half_life（不是出價參數），
//     不同參數組才能比較；不含能量懲罰
#pragma once

#include <array>

#include <cstdint>
#include <string>
#include <vector>

#include "cbba_core/assignment_manager.hpp"
#include "cbba_core/cbba_comm.hpp"
#include "cbba_core/scoring.hpp"
#include "cbba_core/types.hpp"

namespace cbba_core
{

struct MissionAgent
{
  AgentState state;          // 出生點（= 返航點）、初始電量、出價用的能量模型、速度
  double drain_rate{0.2};    // 實際耗電（%／秒，依時間），PX4 的 100 / SIM_BAT_DRAIN
  double drain_per_meter{0.0};  // 實際耗電（%／公尺）：狗用；無人機的模擬電池只依時間，0
  std::size_t max_bundle{0};    // 這台的 bundle 上限（cbba_node 依平台：無人機 2、狗 5）；0 = 照 config.params
};

struct MissionTask
{
  Task task;                 // task.id 的建立者要是某一台；created 由模擬在插入時填
  double insert_at{0.0};     // 從模擬開始算起的秒數
  std::string label;         // 例如 G1、R3（輸出用）
  AgentId expected{kNoAgent};  // 情境設計上應該由誰做（只用來算選擇對不對；0 = 沒有）
  double ideal_eta{-1.0};    // 不受其他任務影響時的到達時間（秒，從插入起算）；≥ 0 時記實際的延遲
};

struct MissionConfig
{
  double loss{0.0};               // 封包掉包率
  double delay_min{0.02};         // 延遲 50 ± 30 ms
  double delay_max{0.08};
  double dt{0.01};                // 模擬步長（cbba_node 的 tick 是 10 ms）
  double max_time{1800.0};        // 最長模擬時間（秒）
  double critical_battery{20.0};  // BT 的 IsUAVBatteryCritical（min_battery）
  double reach_tolerance{0.5};    // 和 uav_px4_bt 一致
  ScoringParams params;
  CommConfig comm;
  AssignmentConfig assign;        // 預設 require_accept = false（無人機）
  double check_period{0.05};      // 收斂、重複認領、路徑變動的取樣週期（秒）
  double converge_hold{0.5};      // 一致要持續這麼久才算收斂（避開瞬間一致）
  // 實際折扣價值用的 half_life（P0～P3，秒）：固定的評估基準，和出價參數無關
  std::array<double, 4> eval_half_life{{60.0, 120.0, 300.0, 600.0}};
  bool record_bids{false};        // 收斂當下記每台的出價分項（MissionResult::bids）
  MissionConfig() {assign.require_accept = false;}
};

enum class TaskOutcome : std::uint8_t
{
  DONE,          // 在期限內完成
  LATE,          // 完成，但超過期限
  FAILED,        // 有人接過、電量不夠沒做完（返航或撤銷），最後沒有完成
  UNASSIGNED,    // 最後沒有人接（大家都電量不足、不出價；包含接了、還沒做就因電量釋放的）
  UNFINISHED,    // 時限到了還在執行（max_time 太短）
};
const char * toString(TaskOutcome outcome);

struct TaskRecord
{
  TaskId id{0};
  std::string label;
  double inserted{-1.0};        // 秒（從模擬開始）
  double first_assigned{-1.0};  // 第一次發到 assigned_task
  AgentId done_by{kNoAgent};
  double done_at{-1.0};
  int failures{0};              // 接了但沒做完的次數（返航或撤銷）
  TaskOutcome outcome{TaskOutcome::UNASSIGNED};

  std::uint8_t priority{3};     // P0～P3
  std::uint8_t platform{0};     // 任務的平台（0 = 不限）
  double value{0.0};
  AgentId expected{kNoAgent};
  AgentId first_executor{kNoAgent};
  double converged_at{-1.0};    // 秒（從模擬開始）；< 0 = 沒收斂。收斂時間 = converged_at − inserted
  AgentId converged_winner{kNoAgent};
  double top_bid{0.0};          // 收斂當下第一、第二高的出價（各台重算，不看鎖定，見 mission_sim.cpp 的 snapshotBids）
  double second_bid{0.0};
  double score_gap{-1.0};       // (第一 − 第二) / 第一；只有一台出價時 1；沒人出價 −1
  int bidders{0};
  double arrived_at{-1.0};      // 完成的那一次到達任務點的時刻
  int handoffs{0};              // 執行者換人的次數
  int handoffs_forced{0};       // 其中前一位是失敗、返航、被搶占而停下的
  int preemptions{0};           // 執行中被較高優先級的任務換掉的次數
  double duplicate_time{0.0};   // 兩台以上同時執行的時間（秒）
  bool deadline_miss{false};    // 有期限：沒完成，或到達超過期限
  double discounted_value{0.0}; // 完成時 v × 2^(−(到達 − 插入) / eval_half_life)
  double delay{-1.0};           // max(0, 到達 − 插入 − ideal_eta)（ideal_eta ≥ 0 而且有到達時）；−1 = 沒有
  double eta() const {return arrived_at >= 0.0 ? arrived_at - inserted : -1.0;}
};

// 收斂當下一台對一個任務的出價（record_bids）。分項用 bestInsertion 在「這台目前的路徑去掉這個任務」上重算；
// bid、why 是核心的 bidFor（含鎖定、bundle 上限、搶占等限制；已正式指派時別台是 0、locked）
struct BidRecord
{
  TaskId task{0};
  AgentId agent{kNoAgent};
  double bid{0.0};
  std::string why;              // bidFor 的原因（in_bundle、infeasible、platform…）
  bool feasible{false};
  std::size_t position{0};      // 最佳插入位置
  double eta{-1.0};             // 估計的到達時間（秒）
  double time_reward{0.0};      // 這個任務的折扣報酬 ρ v 2^(−τ/h)
  double energy_penalty{0.0};   // 插入後能量項增加多少 w_E ΔE / (B − R)
  double gain{0.0};             // 路徑報酬增加多少（= 邊際出價，可能 ≤ 0）
  bool winner{false};
};

struct AgentRecord
{
  AgentId id{kNoAgent};
  double battery_start{0.0};
  double battery_end{0.0};
  int completed{0};
  double distance{0.0};         // 飛行距離（公尺）
  double rtl_at{-1.0};          // 返航的時刻；< 0 = 沒有返航
  TaskId rtl_task{0};           // 返航時手上的任務（0 = 沒有）
  int released{0};              // 執行中的任務因電量被撤銷的次數
  std::uint8_t platform{1};
  double min_battery{0.0};      // 過程中最低的電量
  double energy_used{0.0};      // battery_start − battery_end
  double work_time{0.0};        // 手上有任務（前往＋執行）的時間
  double return_margin{0.0};    // 結束時：電量 − 返航耗電（模型）− 安全存量
  int bundle_drops{0};          // 路徑上還開著的任務被拿掉的次數（被出價蓋過、撤銷、重新排程）
};

struct MissionResult
{
  std::vector<TaskRecord> tasks;     // 和輸入的順序相同
  std::vector<AgentRecord> agents;
  double end_time{0.0};              // 全部完成、全部返航，或 max_time
  int wrong_accepts{0};              // 接了但沒做完的次數（返航＋撤銷；同一個任務可能不只一次）
  int released{0};                   // 其中撤銷的次數
  int preempted{0};                  // 執行中被較高優先級換掉的次數（不算接錯）
  std::vector<BidRecord> bids;       // record_bids 時
  int count(TaskOutcome o) const;
};

// 跑一次任務。seed 決定網路的掉包、延遲（任務的插入時刻在 tasks 裡，由呼叫者決定）
MissionResult runMission(const std::vector<MissionAgent> & agents, const std::vector<MissionTask> & tasks,
  const MissionConfig & config, std::uint32_t seed);

}  // namespace cbba_core
