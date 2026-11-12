// 閘道（GCS）的任務注入（2-1b M0 第 4a 步，2026-11-12；合約 doc/cbba_m0_contract.md 第三節「注入」、第六節「統一由閘道建立任務」）
//
// 閘道跑一個不出價的 cbba_node（participate: never，機號例如 200），規劃器、檢查器經由 inject_task 把任務交給它；
// 這裡負責冪等、欄位檢查、配 task_id／group_id，再交給 CbbaComm::addLocalTask 廣播（TASK_ANNOUNCE＋TASK_SPEC）。
// 不依賴 ROS：節點層把 InjectTask.srv 轉成 InjectRequest。
//
// 冪等鍵 (incident_id, role, revision)：
//   同一個鍵重送           → DUPLICATE：回同一個 task_id，不重發（規劃器逾時重試不會建出兩個任務）
//   同 (incident, role)、revision 較大 → 改版（第 4b 步才做；現在回 REJECTED）
//   revision 較小          → REJECTED
// 對照表只在記憶體裡：閘道重開後同一個事件再注入會變成新任務（世代不同，task_id 不會撞）。
// group_id：同一個 incident 有角色組（role_count > 0）時配一個，格式同 task_id（和任務共用流水號）。
//
// 時間欄位是「從注入時刻起算的秒數」（0 = 沒有），方便 ros2 service call；轉成絕對時刻後放進任務定義，
// 轉送、補發都不會再往後推。
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>

#include "cbba_core/cbba_comm.hpp"

namespace cbba_core
{

struct InjectRequest
{
  std::string incident_id;     // 事件識別（規劃器給），不能是空的
  std::string role;            // 角色名稱；空 = 單獨任務
  std::uint16_t revision{0};
  // 任務內容：platform、required_caps、priority、position、value、duration_sec、ext（role_index、role_count、
  // optional、precedence…）。id、type、created、期限、status 由閘道填
  Task task;
  double deadline_sec{0.0};    // 從現在起的期限；0 = 沒有（出價用預設 300 s）
  double release_sec{0.0};     // 從現在起多久之後才能開始；0 = 立即
  double expires_sec{0.0};     // 從現在起多久之後失效；0 = 不失效
  std::string spec;            // TaskSpec（JSON，B 層）；空 = 沒有
};

enum class InjectStatus { CREATED, DUPLICATE, REJECTED };
const char * toString(InjectStatus status);

struct InjectResult
{
  InjectStatus status{InjectStatus::REJECTED};
  std::string reason;          // REJECTED 的原因（給規劃器、log）
  TaskId task_id{0};
  TaskId group_id{0};
  std::uint16_t revision{0};
};

class Gateway
{
public:
  explicit Gateway(CbbaComm & comm) : comm_(comm) {}

  InjectResult inject(const InjectRequest & req, double now);

  // 已注入的 (incident_id, role) → task_id（查詢、測試用）
  struct Injected
  {
    TaskId task_id{0};
    std::uint16_t revision{0};
  };
  const std::map<std::pair<std::string, std::string>, Injected> & injected() const {return injected_;}

private:
  std::string check(const InjectRequest & req) const;   // 空字串 = 合法

  CbbaComm & comm_;
  std::map<std::pair<std::string, std::string>, Injected> injected_;
  std::map<std::string, TaskId> groups_;   // incident_id → group_id
};

}  // namespace cbba_core
