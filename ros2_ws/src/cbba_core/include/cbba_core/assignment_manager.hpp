// 正式指派的流程（不依賴 ROS，節點層把 BT 的訊息接進來）：2026-10-09
//
// CBBA 得標只是候選；BT（地面端）保有最終的接受與中斷權。流程：
//
//   路徑第一個任務連續 assign_hold 秒沒變
//     → 待接受：beginAssignment() 預先分配版本（這次請求的識別碼），送 ACTIVATE 請求給 BT。
//       舊的任務照常執行，assigned_task 不變（黑板不覆寫）
//     → BT 停止舊的、完成交接後回覆接受 → 版本變成「執行中」，發到 assigned_task
//     → 拒絕：結束這個版本；暫時（不可中斷階段等）→ 撤回出價，BT 的執行狀態改變時重新評估；
//             永久（能力、故障、不可達）→ 不再出價
//     → accept_timeout 內沒回覆：當成暫時拒絕，送 RELEASE 讓 BT 知道這個請求作廢；之後才到的接受回覆忽略
//
//   執行中任務之後的第一個火警（排隊）同樣經過接受，變成「保留」（RESERVE），全隊看得到、別台不出價。
//   執行中的任務結束、保留的任務排到第一時，先重新檢查可行性（核心的期限、電量），通過才變成「執行中」，
//   不再問一次 BT（它已經接受過保留）。
//
//   搶占：核心只在「可以安全中斷、新任務優先級較高」時，把新任務排到執行中任務的前面；
//   接受後，被中斷的任務結束這次指派（新版本），任務保留、重新開放競標，不送 TASK_CLOSE。
//
// require_accept = false（無人機的 BT 還沒好）：跳過和 BT 的握手（請求當下就算接受），
// 可行性檢查、版本、回報檢查照常。
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "cbba_core/cbba_comm.hpp"

namespace cbba_core
{

// BT 的執行狀態（swarm_interfaces/ExecState），fresh 由節點用本機收到的單調時間判斷
struct ExecInput
{
  bool fresh{false};
  std::uint8_t execution_state{0};
  bool preemptible{false};
  double remaining_time{-1.0};
  bool has_active{false};
  TaskId active_task{0};
  std::uint64_t active_version{0};
};
constexpr std::uint8_t kExecFault = 5;

struct AssignmentConfig
{
  bool require_accept{true};
  double assign_hold{0.6};        // 路徑第一個任務連續這麼久沒變才請 BT 接受
  double accept_timeout{3.0};     // 故障處理的上限，不是正常的反應時間
  std::size_t max_queued_fires{1};
  double max_queue_wait{120.0};
};

enum class RequestKind : std::uint8_t { ACTIVATE = 1, RESERVE = 2, RELEASE = 3 };
enum class RejectReason : std::uint8_t { NONE = 0, TEMPORARY = 1, PERMANENT = 2 };

struct AssignmentRequest
{
  RequestKind kind{RequestKind::ACTIVATE};
  bool preempt{false};
  Task task;
  std::uint64_t version{0};
};

struct HeldAssignment
{
  TaskId task{0};
  std::uint64_t version{0};
};

// 分段量測（系統時鐘的秒；created 是建立者的時鐘，要對時）。running < 0 表示 BT 還沒回報在執行
struct AssignmentTiming
{
  TaskId task{0};
  std::uint64_t version{0};
  bool preempt{false};
  double created{0.0};    // 任務建立
  double head{0.0};       // 開始排在路徑第一（含 assign_hold 的等待）
  double request{0.0};    // 送出請求
  double accept{-1.0};    // BT 接受
  double assigned{-1.0};  // 發到 assigned_task
  double running{-1.0};   // BT 的 ExecState 回報在執行這個任務、這個版本
};

class AssignmentManager
{
public:
  // level：0 一般、1 警告、2 錯誤
  using Logger = std::function<void(int level, const std::string & text)>;

  AssignmentManager(CbbaComm & comm, const AssignmentConfig & config, Logger log);

  // 每個 tick 都給（BT 沒在回報時 fresh = false）
  void setExec(const ExecInput & exec, double now);
  // 每個 tick（comm.poll 之後）：檢查持有的指派、逾時，決定要送的請求與 assigned_task
  void step(double now);
  // BT 的回覆：版本不是目前在等的那一個就忽略（延遲的接受不能啟動任務）
  void onResponse(TaskId task, std::uint64_t version, bool accepted, RejectReason reason,
    const std::string & detail, double now);
  // BT 的任務結果：要是目前執行中的任務、版本
  ReportCheck onResult(TaskId task, std::uint64_t version, bool success, double now);

  std::vector<AssignmentRequest> takeRequests();
  bool takeAssignedChanged();   // assigned_task 要重發（內容是 active()）
  std::vector<AssignmentTiming> takeTimings();   // 量測完成的（BT 回報在執行；不用接受時是發布時）

  const std::optional<HeldAssignment> & active() const {return active_;}
  const std::optional<HeldAssignment> & reserved() const {return reserved_;}
  std::optional<HeldAssignment> pending() const;

private:
  struct Pending
  {
    RequestKind kind{RequestKind::ACTIVATE};
    bool preempt{false};
    TaskId task{0};
    std::uint64_t version{0};
    double sent{0.0};
  };

  void checkHeld(double now);
  void startRequest(RequestKind kind, TaskId task, bool preempt, double head_since, double now);
  void accept(const Pending & p, double now);
  void release(TaskId task, std::uint64_t version);
  void abandonPending(const std::string & why);
  void updateConstraints(double now);
  bool inPath(TaskId task) const;
  std::string name(TaskId task, std::uint64_t version) const;
  void info(const std::string & text) {log_(0, text);}
  void warn(const std::string & text) {log_(1, text);}

  CbbaComm & comm_;
  AssignmentConfig config_;
  Logger log_;

  ExecInput exec_;
  std::tuple<bool, std::uint8_t, bool, bool, TaskId> exec_signature_{};

  std::optional<HeldAssignment> active_;
  std::optional<HeldAssignment> reserved_;
  std::optional<Pending> pending_;
  double active_since_{0.0};
  double last_mismatch_warn_{-1e9};

  TaskId candidate_{0};
  double candidate_since_{0.0};
  TaskId queue_candidate_{0};
  double queue_since_{0.0};

  std::vector<AssignmentRequest> requests_;
  bool assigned_changed_{true};   // 一開始發一次「沒有任務」
  std::map<std::pair<TaskId, std::uint64_t>, AssignmentTiming> timings_;
  std::vector<AssignmentTiming> finished_timings_;
};

}  // namespace cbba_core
