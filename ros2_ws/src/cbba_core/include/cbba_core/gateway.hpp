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
// 對照表存檔（任務欄位補齊 01，2026-11-17）：saveState()／loadState() 由節點層寫到 <state_dir>，閘道重開後同一個
//   (incident_id, role) 沿用原本的 task_id（PDF 01 的冪等鍵是 (incident_id, role_index)；role 名稱在同一個 incident
//   裡和 role_index 一一對應，所以照舊用名稱）。重開時任務本身還要從鄰居補回來（閘道是建立者，鄰居一直記著）：
//   讀回來的對照先標「待確認」，任務補回來就確認；restore_confirm（10 s）內還沒補回來的鍵再注入 → 拒絕、請規劃器稍後重送；
//   過了還沒補回來 = 全隊都不記得這個任務（整隊重開），丟掉那筆對照，之後再注入就是新任務（新世代，task_id 不會撞）
// 識別、語意、空間（任務欄位補齊 01、02、04）：meta 由節點層寫進 TaskSpec（cbba_core/task_meta.hpp），這裡只檢查欄位、
//   記住名稱與編號（assignments、log 用）。改版時規劃器給的相對時間（deadline_sec 等）和上一版一樣 → 沿用原本的絕對時刻，
//   只動位置的改版才認得出來（CbbaComm 的 retarget_threshold：小幅移動不重新評估）
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
//
// 連鎖撤銷（CB-4 第三批，2026-11-16，doc/cb4_design.md）：stepGroups() 定期檢查自己建立的角色組裡的必要角色（不是 optional 的）：
//   確定終止（立即）：合法取消、任務／事件過期
//   暫時不完整（使用者 11/16）：目前沒有存活的、平台與能力符合的載具；沒有得標者也沒有正式指派（撤銷、重新競標中）
//     → group_recovery_timeout（10 s）。期間恢復（有人得標或有正式指派）就繼續；
//     逾時以 RECOVERY_TIMEOUT 收尾，保留最初的觸發原因（recovery trigger）。gateway_timeout（3 s）只用在存活判斷
//   執行者失聯（做法 A，使用者 11/16）：超過 lost_timeout 先標記（holder_lost），不算恢復期限；照任務優先級等
//     takeover_by_priority（P0 10、P1 15、P2／P3 30 s）讓原執行者恢復。時限到、撤銷舊指派並重新開放競標時，
//     才開始算 group_recovery_timeout（觸發原因 HOLDER_LOST）。例 P1：1.5 s 判定失聯 → 再 15 s 撤銷、重新競標 →
//     再 10 s 內沒有新的指派才 RECOVERY_TIMEOUT。避免接手還沒輪到，組就先被恢復期限取消
//   有新的失效 → group_failure_version + 1；all_or_nothing 的組取消整組；不是的取消失效的角色和「依賴它的角色」
//  （A 層 precedence 遞移），不依賴的照做。有取消角色而且組有 fallback_template → 發 FallbackRequest（冪等鍵
//   (group_id, group_failure_version)）；組資料的 fallback_template 由規劃器在 InjectTask 明確帶入，閘道不猜
//   閘道聽不到任何鄰居（membership 還沒好）時不判斷
#pragma once

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "cbba_core/cbba_comm.hpp"
#include "cbba_core/task_meta.hpp"

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
  // CB-4 第三批：組資料。fallback_template 同一組的角色要一樣（第一個角色決定，之後不同 → 拒絕）；
  // incident_revision 是規劃器展開時依據的 incident revision，不能比閘道目前的新
  std::string fallback_template;
  std::uint16_t incident_revision{0};
  // 任務欄位補齊第一批（2026-11-17）：識別、語意、空間。incident_id、role 以上面的為準（閘道覆寫）；
  // spec 已經由節點層把它寫進最上層（metaJsonMembers）
  TaskMeta meta;
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

// 連鎖撤銷（第三批）
// 3 保留（11/16 初版的 NO_CAPABLE，改成暫時不完整的觸發原因）
enum class GroupFailure : std::uint8_t { CANCELLED = 1, EXPIRED = 2, RECOVERY_TIMEOUT = 4 };
const char * toString(GroupFailure f);
// 暫時不完整的觸發原因（第一次進入暫時不完整時的，恢復前不改）
enum class RecoveryTrigger : std::uint8_t { NONE = 0, NO_CAPABLE = 1, NO_WINNER = 2, HOLDER_LOST = 3 };
const char * toString(RecoveryTrigger t);
struct GroupEvent
{
  TaskId group_id{0};
  std::string incident_id;
  std::uint32_t failure_version{0};      // FallbackRequest 的冪等鍵（和 group_id 一起）
  GroupFailure reason{GroupFailure::CANCELLED};
  std::vector<std::uint8_t> failed_roles;       // 這次新失效的必要角色
  std::vector<TaskId> failed_tasks;
  std::vector<RecoveryTrigger> triggers;        // 和 failed_roles 對應：RECOVERY_TIMEOUT 的最初觸發原因（取消、過期是 NONE）
  std::vector<std::uint8_t> cancelled_roles;    // 閘道因此取消的角色（含失效的、依賴它的；all_or_nothing 是整組）
  bool all_or_nothing{false};
  bool fallback{false};                 // 要發 FallbackRequest（有取消角色而且有 fallback_template）
  std::string fallback_template;
  std::uint16_t incident_revision{0};
  bool position_valid{false};
  Vec3 position{};
};

struct GatewayConfig
{
  double group_recovery_timeout{10.0};  // 暫時不完整等多久（使用者 11/16：group_recovery_timeout_s = 10）
  double restore_confirm{10.0};         // 重開後讀回的對照，等任務從鄰居補回來多久（任務欄位補齊 01）
};

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
  explicit Gateway(CbbaComm & comm, GatewayConfig config = GatewayConfig{}) : comm_(comm), config_(config) {}

  InjectResult inject(const InjectRequest & req, double now);
  const GatewayConfig & config() const {return config_;}

  // 已注入的 (incident_id, role) → task_id（查詢、測試用）
  struct Injected
  {
    TaskId task_id{0};
    std::uint16_t revision{0};
    // 上一版請求的相對時間（改版時一樣就沿用絕對時刻）
    double deadline_sec{0.0};
    double release_sec{0.0};
    double expires_sec{0.0};
    TaskMeta meta;               // 存檔只存識別與語意（名稱、編號、來源）；空間在任務、TaskSpec 裡
    bool restored{false};        // 從存檔讀回、任務還沒從鄰居補回來
  };
  const std::map<std::pair<std::string, std::string>, Injected> & injected() const {return injected_;}
  const Injected * findTask(TaskId task_id) const;

  // 存檔（任務欄位補齊 01）：文字格式、一行一筆；節點層內容變了才寫檔
  std::string saveState() const;
  // 讀回（啟動時、注入之前）。格式不對回 false 並寫 error，什麼都不改
  bool loadState(const std::string & text, double now, std::string * error = nullptr);
  // 讀回、等不到任務補回來而丟掉的對照（給節點層 log）；取走
  std::vector<std::pair<std::string, std::string>> takePurged();

  // optional 角色的自動關閉：每個 tick 呼叫（內部每 0.2 s 檢查一次），回傳這次關掉的
  std::vector<DroppedOptional> step(double now);
  bool membershipReady(double now);
  // 連鎖撤銷：每個 tick 呼叫（內部每 0.2 s 檢查一次），回傳這次新的失效事件
  std::vector<GroupEvent> stepGroups(double now);

  struct GroupData
  {
    std::string incident_id;
    std::string fallback_template;
    std::uint32_t failure_version{0};
    std::set<std::uint8_t> failed_roles;
    bool terminated{false};             // all_or_nothing 已經整組取消
    struct Recovery
    {
      double since{0.0};
      RecoveryTrigger trigger{RecoveryTrigger::NONE};
    };
    std::map<TaskId, Recovery> recovering;   // 暫時不完整的角色：開始時刻、最初的觸發原因
    // 執行者失聯、還在等失聯接手的角色（開始標記的時刻）：這段期間不算恢復期限（做法 A）
    std::map<TaskId, double> holder_lost;
    bool restored{false};               // 從存檔讀回、組裡的任務還沒補回來
  };
  const std::map<TaskId, GroupData> & groups() const {return group_data_;}

private:
  std::string check(const InjectRequest & req) const;   // 空字串 = 合法
  // 請求 → 任務內容（id、group_id、created 由呼叫端填）；期限從 created 起算
  static Task build(const InjectRequest & req, double created, double now);
  InjectResult revise(const InjectRequest & req, Injected & known, double now);
  // 讀回的對照：任務補回來就確認；restore_confirm 之後還沒補回來的丟掉
  void confirmRestored(double now);

  CbbaComm & comm_;
  GatewayConfig config_;
  std::map<TaskId, GroupData> group_data_;
  std::set<TaskId> cascade_cancelled_;   // 連鎖撤銷取消的（之後看到它們被取消不再算成新的失效）
  double next_group_check_{0.0};
  std::map<std::pair<std::string, std::string>, Injected> injected_;
  std::map<std::string, TaskId> groups_;   // incident_id → group_id
  std::map<TaskId, std::pair<std::string, std::string>> by_task_;   // task_id → 對照的鍵
  double restored_at_{-1.0};   // 讀回存檔的時刻；< 0 = 沒有待確認的
  std::vector<std::pair<std::string, std::string>> purged_;

  double connected_since_{-1.0};   // 這次連上（聽得到鄰居）的時刻；< 0 = 斷線
  double next_check_{0.0};
  std::map<TaskId, double> no_capable_since_;   // O1 計時
  std::map<TaskId, double> unassigned_since_;   // O2 計時
};

}  // namespace cbba_core
