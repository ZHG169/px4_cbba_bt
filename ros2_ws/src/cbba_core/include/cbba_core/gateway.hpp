// 閘道（GCS）的任務注入（2-1b M0 第 4a 步，2026-11-12；合約 doc/cbba_m0_contract.md 第三節「注入」、第六節「統一由閘道建立任務」）
//
// 閘道跑一個不出價的 cbba_node（participate: never，機號例如 200），規劃器、檢查器經由 inject_task 把任務交給它；
// 這裡負責冪等、欄位檢查、配 task_id／group_id，再交給 CbbaComm::addLocalTask 廣播（TASK_ANNOUNCE＋TASK_SPEC）。
// 不依賴 ROS：節點層把 InjectTask.srv 轉成 InjectRequest。
//
// 冪等鍵 (incident_id, role, revision)：
//   同一個鍵重送           → DUPLICATE：回同一個 task_id，不重發（規劃器逾時重試不會建出兩個任務）
//   同 (incident, role)、revision 較大 → 改版（REVISED，同一個 task_id；4b，2026-11-13）：CbbaComm::reviseLocalTask，
//                            只改位置、時間、價值時指派保留，改了平台、能力、優先級重新競標；角色組（role_index、role_count）不能改
//   revision 較小          → REJECTED
// 對照表只在記憶體裡：閘道重開後同一個事件再注入會變成新任務（世代不同，task_id 不會撞）。
// group_id：同一個 incident 有角色組（role_count > 0）時配一個，格式同 task_id（和任務共用流水號）。
//
// 時間欄位是「從注入時刻起算的秒數」（0 = 沒有），方便 ros2 service call；轉成絕對時刻後放進任務定義，
// 轉送、補發都不會再往後推。
//
// optional 角色的自動關閉（4e，2026-11-13，合約第三節）：step() 定期檢查自己建立、還開著的 optional 角色，
// 符合條件就送 TASK_CLOSE reason 4（略過），不自動重開：
//   O1 全隊沒有能做的載具（capableAgents 是空的；閘道不算）：membership_ready 之後連續 optional_no_capable_s（預設 10 s）
//   O2 有人能做但沒有正式指派：釋放（建立或持有者交還）後 optional_unassigned_s（預設 15 s）
//   O3 同一組的必要角色都結束了（完成、取消、過期）：馬上關，執行中的也關（閘道的正式關閉）
//   O1、O2 只關沒有正式指派的。O4（組取消、過期）就是各任務自己的取消、過期，不用另外做
// membership_ready：閘道知道「現在全隊有誰」才判斷 O1，避免剛啟動、剛從斷線恢復時還沒收到各台的 AGENT_STATE 就誤關。
//   開機同步完成，而且從連上（第一次聽到任何鄰居，或斷線後恢復）起過了 lost_timeout ＋ status_period，期間每個還聽得到的
//   鄰居都收過 AGENT_STATE。聽不到任何鄰居 = 閘道斷線，重置
// skip_satisfies（略過時 precedence 算不算滿足）只帶在任務定義裡：precedence 是 CB-4（M1～M2）
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

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

enum class InjectStatus { CREATED, DUPLICATE, REVISED, REJECTED };

// optional 角色被略過的原因
enum class OptionalDrop : std::uint8_t { NO_CAPABLE = 1, UNASSIGNED = 2, GROUP_DONE = 3 };
const char * toString(OptionalDrop reason);
struct DroppedOptional
{
  TaskId task{0};
  OptionalDrop reason{OptionalDrop::NO_CAPABLE};
  double waited{0.0};   // O1、O2：條件成立了多久
};
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

  // optional 角色的自動關閉：每個 tick 呼叫（內部每 0.2 s 檢查一次），回傳這次關掉的
  std::vector<DroppedOptional> step(double now);
  bool membershipReady(double now);

private:
  std::string check(const InjectRequest & req) const;   // 空字串 = 合法
  // 請求 → 任務內容（id、group_id、created 由呼叫端填）；期限從 created 起算
  static Task build(const InjectRequest & req, double created, double now);
  InjectResult revise(const InjectRequest & req, Injected & known, double now);

  CbbaComm & comm_;
  std::map<std::pair<std::string, std::string>, Injected> injected_;
  std::map<std::string, TaskId> groups_;   // incident_id → group_id

  double connected_since_{-1.0};   // 這次連上（聽得到鄰居）的時刻；< 0 = 斷線
  double next_check_{0.0};
  std::map<TaskId, double> no_capable_since_;   // O1 計時
  std::map<TaskId, double> unassigned_since_;   // O2 計時
};

}  // namespace cbba_core
