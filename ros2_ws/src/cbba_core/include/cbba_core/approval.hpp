// 閘道的核准（任務欄位補齊 12「人機與安全」，2026-11-18；docs/planner_task_definition_plan.md）
//
// 要核准的注入不直接建 CBBA 任務，先放在這裡：
//   PROPOSED → WAITING_APPROVAL → APPROVED → INJECTED（閘道照常注入）／REJECTED（值班拒絕、注入失敗）
//                               └→ EXPIRED（等核准時已過 expires、或硬期限）
// 核准前不建 CBBA 任務（不配 task_id、不廣播）。誰要核准由節點層決定（規劃器要求、事件類型 requires_approval、
// 風險 high 的動作；approval_policy），這裡只管狀態、冪等、時間換算。
// 冪等：同一個 (incident_id, role, revision) 再送 → 回同一筆（不管狀態；被拒絕的要換 revision 才能再送）。
// 時間：請求裡的 deadline_sec、release_sec、expires_sec 是「從送出起算」；核准時扣掉等待的時間再注入
//（期限不會因為等核准而往後推）。不依賴 ROS、不寫檔：狀態變化由節點層發話題、寫稽核紀錄
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "cbba_core/gateway.hpp"

namespace cbba_core
{

enum class ApprovalState : std::uint8_t
{
  PROPOSED = 0,
  WAITING_APPROVAL = 1,
  APPROVED = 2,
  INJECTED = 3,
  REJECTED = 4,
  EXPIRED = 5,
};
const char * toString(ApprovalState s);

struct ApprovalRecord
{
  std::uint32_t id{0};
  InjectRequest request;
  std::vector<std::string> reasons;   // 為什麼要核准
  double proposed_at{0.0};
  ApprovalState state{ApprovalState::PROPOSED};
  std::string operator_name;          // 誰核准／拒絕
  std::string note;                   // 值班的備註；注入失敗、過期時的原因
  double decided_at{0.0};
  TaskId task_id{0};                  // INJECTED 之後
  TaskId group_id{0};
};

class ApprovalQueue
{
public:
  struct Proposed
  {
    std::uint32_t id{0};
    bool duplicate{false};
  };
  // 新的進 WAITING_APPROVAL（PROPOSED 只出現在狀態變化的紀錄裡）
  Proposed propose(const InjectRequest & request, const std::vector<std::string> & reasons, double now);

  // 值班決定：id ≠ 0 只決定那一筆；id = 0 決定 incident_id 裡所有在等的。回傳決定了哪些（核准的變 APPROVED，
  // 節點接著注入、再 markInjected／markFailed）。沒有可決定的回傳空的、error 寫原因
  std::vector<std::uint32_t> decide(std::uint32_t id, const std::string & incident_id, bool approve,
    const std::string & operator_name, const std::string & note, double now, std::string * error = nullptr);

  // APPROVED 的請求、時間扣掉等待的；已經過期（expires、硬期限）回 nullopt 並寫原因
  std::optional<InjectRequest> injectable(std::uint32_t id, double now, std::string * why = nullptr) const;
  void markInjected(std::uint32_t id, const InjectResult & result, double now);
  void markFailed(std::uint32_t id, ApprovalState state, const std::string & why, double now);   // REJECTED、EXPIRED

  // 等核准時 expires 到了 → EXPIRED（每個 tick 呼叫）
  void expire(double now);

  const ApprovalRecord * find(std::uint32_t id) const;
  const std::map<std::uint32_t, ApprovalRecord> & records() const {return records_;}
  std::vector<std::uint32_t> waiting() const;
  // 上次之後狀態有變化的（含新提出的 PROPOSED → WAITING_APPROVAL：同一筆可能出現兩次，依序）
  std::vector<std::pair<std::uint32_t, ApprovalState>> takeChanges();

private:
  void set(ApprovalRecord & r, ApprovalState s);

  std::uint32_t next_id_{1};
  std::map<std::uint32_t, ApprovalRecord> records_;
  std::map<std::tuple<std::string, std::string, std::uint16_t>, std::uint32_t> by_key_;
  std::vector<std::pair<std::uint32_t, ApprovalState>> changes_;
};

}  // namespace cbba_core
