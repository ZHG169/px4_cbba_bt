// CB-4 第一批（2026-11-16，doc/cb4_design.md）：角色的里程碑與執行階段
//
// 里程碑用固定的位元表示（TASK_PROGRESS 封包的 milestones 欄位），**位元位置只增不改**（和能力的 bit 一樣）；
// 名稱與 bit 也寫在註冊表 registry/actions.yaml 的 milestones，swarm_cbba 的 test_registry 檢查兩邊一致。
//
// 兩類（使用者 11/16 定）：
//   執行者相關（accepted、en_route、arrived、started、target_locked）：跟著那一次正式指派；換人（新的指派版本）就清掉
//   事件結果（target_localized、has_observation）：關於目標本身，換人後保留，但只在「同一個 incident revision」而且
//     任務還沒過期時算數（目標位置改版了，舊的定位就不算）。我們用前一個角色任務的 revision 代表 incident revision：
//     第三批的 IncidentUpdate 會讓同組的任務一起改版
//   completed：任務完成（TASK_CLOSE reason 1）時成立，視為全部里程碑都達成；BT 不用回報
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace cbba_core
{

enum class Milestone : std::uint8_t
{
  ACCEPTED = 0,           // 正式指派、發出 current_task（cbba_node）
  EN_ROUTE = 1,           // 開始前往
  ARRIVED = 2,            // 到達該角色第一個 go_to 的位置
  STARTED = 3,            // 開始第一個非移動的 step
  TARGET_LOCALIZED = 4,   // 已定位目標（事件結果）
  TARGET_LOCKED = 5,      // 追蹤已鎖定
  HAS_OBSERVATION = 6,    // 已取得觀測（事件結果）
  COMPLETED = 7,          // 任務完成（TASK_CLOSE reason 1）
};
constexpr unsigned kMilestoneCount = 8;

using MilestoneMask = std::uint16_t;

constexpr MilestoneMask milestoneBit(Milestone m)
{
  return static_cast<MilestoneMask>(1u << static_cast<unsigned>(m));
}

constexpr MilestoneMask kAllMilestones = static_cast<MilestoneMask>((1u << kMilestoneCount) - 1);
// 事件結果型：換人後保留（同一個 revision、沒過期）
constexpr MilestoneMask kEventMilestones =
  milestoneBit(Milestone::TARGET_LOCALIZED) | milestoneBit(Milestone::HAS_OBSERVATION);
// 執行者相關：跟著指派版本
constexpr MilestoneMask kExecutorMilestones =
  kAllMilestones & static_cast<MilestoneMask>(~kEventMilestones) &
  static_cast<MilestoneMask>(~milestoneBit(Milestone::COMPLETED));

const char * toString(Milestone m);
std::optional<Milestone> milestoneFromName(const std::string & name);
// 位元 → 名稱（照 bit 順序）；不認得的位元寫成 "bitN"
std::vector<std::string> milestoneNames(MilestoneMask mask);
// 名稱 → 位元；不認得的名稱放進 unknown（可以是 nullptr），不算進結果
MilestoneMask parseMilestones(const std::vector<std::string> & names, std::vector<std::string> * unknown = nullptr);

// 角色的執行階段（TASK_PROGRESS 的 phase、swarm_interfaces/TaskProgress 的常數）
enum class ProgressPhase : std::uint8_t
{
  UNKNOWN = 0,
  ACCEPTED = 1,
  EN_ROUTE = 2,
  ARRIVED = 3,
  EXECUTING = 4,
  BLOCKED = 5,           // 停在 wait_for 還沒滿足的 step（保持正式指派、不計入執行逾時）
  ARRIVED_WAITING = 6,   // 第二批：到了 sync 的那一步、等同伴
  DONE = 7,
};
constexpr std::uint8_t kMaxPhase = static_cast<std::uint8_t>(ProgressPhase::DONE);
const char * toString(ProgressPhase p);

}  // namespace cbba_core
