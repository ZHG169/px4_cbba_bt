// CB-4 第一批（2026-11-16，cbba_core/doc/cb4_design.md）：從 TaskSpec 讀出 step-level gate
//
// spec 步驟可以寫 wait_for：[{"role_index": 0, "milestone": "target_localized"}, …]，放在要等的那一步。
// 任務 A 層的 precedence（要等哪幾個角色）裡、但沒有任何步驟的 wait_for 提到的角色：預設在第一步（step 0）等它 completed
//（使用者 11/16：template-level precedence 沒指定 step gate 時，後方角色的第一步等前方角色 completed）。
// 第二批（2026-11-16）：步驟的 "sync": "名稱" 是會合點：同一組裡 spec 有同名會合點的角色都到了（ARRIVED_WAITING）才開始；
// 第一個到的開始算，超過任務的 sync_window_s 還沒到齊 → 照樣開始（已到的先開始）、標 timed_out（使用者 11/16 的建議）。
// JSON 用 yaml-cpp 解析（JSON 是 YAML 的子集），不加新依賴。放在節點層：cbba_core 不解析 spec。
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cbba_core/group_progress.hpp"

namespace swarm_cbba
{

struct StepGateDef
{
  std::uint8_t step_index{0};
  std::vector<cbba_core::WaitFor> wait_for;
  std::string sync;                   // 會合點的名稱；空 = 不是
};

struct SpecGates
{
  std::vector<StepGateDef> steps;     // 照 step_index；沒有 gate 的步驟不列
  std::vector<std::string> warnings;  // 格式錯、不認得的里程碑、role_index 不在 precedence、指到自己（給 log）
};

// TaskSpec 裡用到的動作（steps[*].action，照順序、可能重複；格式不對回空清單）。2026-11-19：cbba_node 比對這台能做的動作
//（由註冊表推導，docs/execution_backend_plan.md）
std::vector<std::string> specActions(const std::string & spec);

// spec：current_task.spec（空 = 沒有 TaskSpec）；precedence：任務 A 層的遮罩（位元 = role_index）；self：自己的 role_index
SpecGates parseSpecGates(const std::string & spec, std::uint8_t precedence, std::uint8_t self);

// 會合點：同名會合點的其他角色各自的狀況
enum class SyncMember : std::uint8_t
{
  ARRIVED,    // 到了（sync_reached）或已完成
  MISSING,    // 還沒到（含還沒收到它的 TaskSpec）
  EXCLUDED,   // 不用等：被略過、取消、過期
};
struct SyncResult
{
  cbba_core::GateState state{cbba_core::GateState::WAITING};
  bool timed_out{false};
  std::vector<std::uint8_t> missing;   // 沒到的 role_index
};
// first_arrival：第一個到的時刻（自己或別人；nullopt = 還沒有人到）；window：任務的 sync_window_s（0 = 不限時，一直等）
SyncResult evaluateSync(const std::vector<std::pair<std::uint8_t, SyncMember>> & others,
  std::optional<double> first_arrival, double now, double window);

}  // namespace swarm_cbba
