// CB-4 第一批（2026-11-16，doc/cb4_design.md）：角色進度的合併與 gate 的判斷（不依賴網路、ROS）
//
// 每個任務留一筆最新的進度（ProgressRecord）。合併規則（使用者 11/16 定）：
//   執行者相關的部分（phase、step、accepted…target_locked）：指派版本較新的取代；同一個版本用 progress_seq 取大的，
//     里程碑只增不減。較舊的指派版本不影響（換人後清掉執行者相關的里程碑）
//   事件結果的部分（target_localized、has_observation）：跟著 revision，不跟指派版本：revision 較新的取代、相同的 OR、
//     較舊的不算。換人後保留，但只在「同一個 revision、任務沒過期」時算數
//   completed：任務完成（TASK_CLOSE reason 1）時成立，視為全部里程碑都達成
#pragma once

#include <cstdint>

#include "cbba_core/milestones.hpp"
#include "cbba_core/types.hpp"
#include "cbba_core/wire_task_progress.hpp"

namespace cbba_core
{

struct ProgressRecord
{
  wire::RelayHeader origin;          // 最新那份（執行者相關部分）的轉送區塊：快照沿用
  std::uint64_t assign_version{0};
  AgentId executor{kNoAgent};
  std::uint16_t progress_seq{0};
  ProgressPhase phase{ProgressPhase::UNKNOWN};
  std::uint8_t step_index{wire::kUnknownStep};
  MilestoneMask executor_bits{0};    // 這個指派版本的執行者相關里程碑
  MilestoneMask event_bits{0};       // 事件結果型（revision 那一個 revision）
  std::uint16_t revision{0};         // event_bits 屬於哪個 revision
  double updated{0.0};               // 最後一次有變化的時刻
};

// 合併一份回報（rec 還沒有資料時 has = false）。回傳 true = 有變化
bool mergeProgress(ProgressRecord & rec, bool has, const wire::TaskProgressBody & b, double now);

// 快照：把一筆紀錄寫成封包內容（轉送區塊沿用 rec.origin）
wire::TaskProgressBody progressBody(TaskId task, const ProgressRecord & rec);

// gate 用的里程碑：
//   rec               這個任務的進度（沒有時 nullptr）
//   current_version   目前已知的正式指派版本：執行者相關的里程碑只在 rec.assign_version ≥ 它時算
//                    （換人之後、新的執行者還沒回報前，舊的不算）
//   current_revision  任務目前的 revision：事件結果型只在相同時算
//   accepted          目前的正式指派是保留或執行中（accepted 由 cbba_node 判定，不用 BT 回報）
//   expired           任務已過期：事件結果型不算
//   completed         TASK_CLOSE reason 1：全部
MilestoneMask effectiveMilestones(const ProgressRecord * rec, std::uint64_t current_version,
  std::uint16_t current_revision, bool accepted, bool expired, bool completed);

// step-level gate（第一批）：每個 wait_for 一個
struct WaitFor
{
  std::uint8_t role_index{0};
  Milestone milestone{Milestone::COMPLETED};
};

enum class GateState : std::uint8_t
{
  OPEN = 0,            // 都滿足了
  WAITING = 1,         // 還沒滿足，前面的角色還開著（或還不認得）
  UNSATISFIABLE = 2,   // 前面的角色已結束但不會滿足：取消、過期、略過而 skip_satisfies = false（第三批由閘道處理）
};
const char * toString(GateState s);

}  // namespace cbba_core
