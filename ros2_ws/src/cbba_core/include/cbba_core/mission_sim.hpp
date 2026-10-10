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
#pragma once

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
};

struct MissionTask
{
  Task task;                 // task.id 的建立者要是某一台；created 由模擬在插入時填
  double insert_at{0.0};     // 從模擬開始算起的秒數
  std::string label;         // 例如 G1、R3（輸出用）
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
};

struct MissionResult
{
  std::vector<TaskRecord> tasks;     // 和輸入的順序相同
  std::vector<AgentRecord> agents;
  double end_time{0.0};              // 全部完成、全部返航，或 max_time
  int wrong_accepts{0};              // 接了但沒做完的次數（返航＋撤銷；同一個任務可能不只一次）
  int released{0};                   // 其中撤銷的次數
  int count(TaskOutcome o) const;
};

// 跑一次任務。seed 決定網路的掉包、延遲（任務的插入時刻在 tasks 裡，由呼叫者決定）
MissionResult runMission(const std::vector<MissionAgent> & agents, const std::vector<MissionTask> & tasks,
  const MissionConfig & config, std::uint32_t seed);

}  // namespace cbba_core
