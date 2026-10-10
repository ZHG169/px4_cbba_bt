// CBBA 的機間協定層（不依賴 ROS，也不碰 socket）：協定版本 2（2026-10-09）
//
// 把 CbbaAgent 接上機間封包（格式見 wire_*.hpp，整體說明見 doc/protocol.md）。
// 呼叫端只負責：收到的封包交給 receive()，poll() 回傳的封包廣播出去。
//
// 任務識別
//   封包裡每筆任務資料都帶 uint32 task_id（建立者機號 × 65536 + 流水號），各節點自己維護
//   task_id → 任務的對照。版本 1 用 AGENT_STATE 裡 z、y 陣列的位置（uint8 任務編號）當全隊共用的識別，
//   要各機切份、只能 1～8 號建立任務、每台約 32 個；版本 2 沒有這些限制，容量只受記憶體與封包大小限制。
//   剛啟動的節點先和鄰居同步（摘要相同）才廣播自己建立的任務，重開機後 BT 重用了開機前的 task_id 時
//   才查得出來（記在 stats().rejected_local_tasks）：
//     摘要和任一鄰居相同 → 開始；join_wait 內沒聽到任何鄰居 → 單獨一台，開始；
//     有鄰居但 join_timeout 內一直對不上 → 照樣開始。
//
// 傳送
//   AGENT_STATE   每 status_period（0.5 s）：位置、電量、真實的 armed／offboard／landed、鄰居、路徑
//   CBBA_STATE    每 200 ms；表有變動時立即；和鄰居的得標者不一致時每 50 ms（同一步最多 60 次）。
//                 任務多時分批，每批不超過 max_packet（1200 B）
//   TASK_ANNOUNCE 建立時一次。補發：鄰居的 round_key 和我不同時，比對它上一次收齊的 CBBA_STATE，
//                 補它缺的任務定義或 TASK_CLOSE（每個每秒最多一次）；不參與出價的鄰居（還在地面、
//                 剛開機）沒有附紀錄，當成什麼都不知道，全部補一次。保底：每 5 s 輪流重播一個已知任務
//   完成          COMPLETION → COMPLETION_ACK → TASK_CLOSE（見 wire_completion.hpp）
//
// 收到 CBBA_STATE
//   每一批都直接交給 CBBA 規則（規則逐任務判斷；每批都帶完整的 s）。還不認得的任務略過，不暫存：
//   200 ms 後的下一次會再帶，任務定義由對方補發（它看到我的 round_key 不同、紀錄裡沒有這個任務）。
//   收斂判定、補發只用收齊的那一次。缺批時等下一次，不會把缺少的紀錄當成任務被刪掉。
//
// 正式指派與回報（2026-10-09）
//   節點請 BT 接受時呼叫 beginAssignment()：產生新版本（已知的最新版本 + 1，執行者是自己，狀態「待接受」）。
//   BT 接受後 confirmAssignment() 把同一個版本改成「保留」或「執行中」；撤銷、拒絕、逾時、執行失敗時
//   endAssignment() 產生新版本（執行者 0、狀態「無」），舊的版本作廢、不再使用。
//   版本放在 CBBA_STATE 的紀錄裡傳遍全隊，各節點取（版本, 執行者, 狀態）較大的那筆。X→Y→X 時 X 第二次的版本一定比第一次大。
//   鎖定：別台「保留」或「執行中」的任務不出價（待接受不鎖）。持有者撤銷 → 重新開放；完成、取消 → 永久結束；
//   持有者失聯 → 照樣鎖著（lockedByLostAgents() 列出來待確認），不重新指派，避免原載具還在執行。
//   回報（reportResult）要帶版本，必須是自己目前的正式指派；完成宣告、TASK_CLOSE 也帶版本，
//   收到的節點知道更新的版本就拒絕（ACK status 2、不結束任務）。
//   執行失敗：撤銷指派、任務保留，由 CBBA 重新分配；不送 TASK_CLOSE。
//   取消（cancelTask）：只有建立者或 cancel_authorities 裡的機號能發；收到的節點也檢查。
//
// 失聯
//   某台的 s 超過 lost_timeout 沒更新，各機自己判定失聯，釋放它得標的任務。
//   某台從參與出價變成不參與時，立即釋放它的任務。
//
// 重開機：每次建立 CbbaComm 都用新的 session_id（隨機），鄰居看到新的 session 就知道重開機過，
//   重開機前在網路上延遲的封包會被丟掉；不需要把序號存在非揮發性儲存（版本 1 的 seq_file）。
//
// 時間一律是系統時鐘的秒數（各機需對時）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "cbba_core/cbba_agent.hpp"
#include "cbba_core/wire_agent_state.hpp"
#include "cbba_core/wire_cbba_state.hpp"
#include "cbba_core/wire_completion.hpp"
#include "cbba_core/wire_header.hpp"
#include "cbba_core/wire_task_announce.hpp"

namespace cbba_core
{

using Bytes = std::vector<std::uint8_t>;

struct CommConfig
{
  double state_period{0.2};       // CBBA_STATE 週期
  double status_period{0.5};      // AGENT_STATE 週期
  double fast_period{0.05};       // 和鄰居不一致時的重送間隔
  int fast_max{60};               // 同一步最多快速重送幾次
  double min_gap{0.01};           // 立即發送的最短間隔
  double neighbor_timeout{1.5};   // 多久沒收到某台的封包就不算直接鄰居
  double lost_timeout{1.5};       // 多久沒有某台的新資訊視為失聯（規格：1.5 s）
  double completion_retry{1.0};   // 完成宣告的重送間隔
  int completion_max{10};         // 完成宣告最多重送幾次
  double resend_gap{1.0};         // 補發同一個 TASK_ANNOUNCE／TASK_CLOSE 的最短間隔
  double replay_period{5.0};      // 保底重播的間隔
  double reevaluate_period{1.0};  // 依目前位置、電量重新評估手上的任務
  int stable_steps{3};            // 收斂判定：連續幾個週期分配不變
  double join_wait{1.5};          // 啟動後這麼久沒聽到任何鄰居：單獨一台，開始廣播自己的任務
  double join_timeout{5.0};       // 有鄰居但摘要一直對不上時最多等多久
  std::size_t max_packet{wire::kDefaultMaxPacket};   // CBBA_STATE 每批（整個封包）的上限
  std::vector<AgentId> cancel_authorities;   // 除了建立者，可以取消任何任務的機號（授權的管理端），全隊要一樣
};

// 正式指派的狀態（CBBA_STATE 紀錄的 assign_state）
enum class AssignState : std::uint8_t { NONE = 0, PENDING = 1, RESERVED = 2, ACTIVE = 3 };

// 任務的正式指派：版本、執行者（0 = 撤銷或還沒指派過）、狀態。（版本, 狀態, 執行者）較大的比較新：
// 同一台的同一個版本，狀態只會往前走（待接受 → 保留 → 執行中）。兩台同時產生同一個版本時（不知道別台已接受），
// 已接受的（保留、執行中）勝過待接受的，同樣狀態才比機號
struct Assignment
{
  std::uint64_t version{0};
  AgentId assignee{kNoAgent};
  AssignState state{AssignState::NONE};
};

inline bool newerThan(const Assignment & a, const Assignment & b)
{
  if (a.version != b.version) {
    return a.version > b.version;
  }
  if (a.state != b.state) {
    return static_cast<int>(a.state) > static_cast<int>(b.state);
  }
  return a.assignee > b.assignee;
}

inline bool isLocking(AssignState s)
{
  return s == AssignState::RESERVED || s == AssignState::ACTIVE;
}

// BT 的執行狀態（AGENT_STATE 用；節點從 ExecState 填）
struct ExecReport
{
  bool known{false};
  std::uint8_t execution_state{0};
  bool preemptible{false};
  double remaining_time{-1.0};
};

// reportResult、cancelTask 的結果
enum class ReportCheck
{
  ACCEPTED,
  NOT_OPEN,          // 不認得、已結束
  STALE,             // 版本不是自己目前的正式指派（舊的回報，或指派已被撤銷、被別台取代）
  PENDING,           // 已經在確認完成
  NOT_AUTHORIZED,    // 取消：不是建立者，也不在 cancel_authorities 裡
};
const char * toString(ReportCheck check);

// 本機的飛行狀態（AGENT_STATE 的 flags）。flight_state_valid 為 false 時 armed、offboard、landed 不送
struct VehicleStatus
{
  bool telemetry_ok{false};
  bool flight_state_valid{false};
  bool armed{false};
  bool offboard{false};
  bool landed{false};
};

struct NeighborInfo
{
  double last_heard{0.0};
  std::uint64_t session{0};          // 換了就是重開機過：下面的新舊判斷重來
  // CBBA_STATE（snapshot_id 比上一次舊的整批丟掉：亂序到達的舊狀態）
  bool has_snapshot{false};
  std::uint16_t snapshot{0};
  bool participating{false};
  std::uint32_t round_key{0};
  int stable_steps{0};
  bool has_view{false};
  std::map<TaskId, AgentId> view;   // 上一次收齊的：它認得、進行中的任務 → 得標者
  std::uint16_t pending_snapshot{0};
  std::uint8_t pending_count{0};
  std::set<std::uint8_t> pending_parts;
  std::map<TaskId, AgentId> pending_view;
  // AGENT_STATE（sequence 比上一則舊的丟掉）
  bool has_status{false};
  std::uint32_t status_seq{0};
  wire::StateFlags flags;
  AgentType vehicle_type{AgentType::UAV};
  Vec3 position{};
  double battery{0.0};
  bool exec_known{false};           // 它的 BT 有在回報執行狀態
  std::uint8_t execution_state{0};
  bool preemptible{false};
  double remaining_time{-1.0};
  std::optional<std::pair<TaskId, std::uint64_t>> active;   // 執行中的正式指派
  std::optional<std::pair<TaskId, std::uint64_t>> queued;   // 保留（排隊）的
  std::set<AgentId> neighbors;      // 它直接聽得到的
};

struct CommStats
{
  std::map<wire::PacketType, int> sent;
  std::map<wire::PacketType, std::size_t> sent_bytes;
  std::map<wire::PacketType, int> received;
  int malformed{0};             // 解不開的封包
  int stale{0};                 // 重複、比視窗還舊、重開機前的 session、比上一次舊的狀態
  int duplicates{0};            // 已處理過的轉送事件
  int relayed{0};
  int task_conflicts{0};        // 同一個 task_id 由不同的 origin 宣布（兩台用了同一個機號，或 BT 重用 task_id）
  int rejected_local_tasks{0};  // 本機 BT 建立的 task_id 已經存在（同步後才發現）
  int rejected_acks{0};         // 收到 accepted = 0 的確認
  int join_timeouts{0};         // 啟動時沒和鄰居同步就開始（等滿 join_timeout）
  int incomplete_snapshots{0};  // CBBA_STATE 缺批，換下一次之前沒收齊
  int cbba_parts{0};            // 送出的 CBBA_STATE 批數
  int stale_acks{0};            // 收到 status = 2（指派已過期）的確認：完成宣告作廢
  int aborted_completions{0};   // 因為指派過期而作廢的完成宣告
  int rejected_closes{0};       // 不合權限的 TASK_CLOSE（舊版本的完成、沒有授權的取消）
};

// task_id 的顯示用名稱（8 位十六進位），例如 "00030005"
std::string taskName(TaskId id);

// 時間戳 s：系統時鐘 ms 的低 32 位元。fromStampMs 取「不晚於 now」最近的時間。
std::uint32_t toStampMs(double t);
double fromStampMs(std::uint32_t ms, double now);

class CbbaComm
{
public:
  // session_id = 0 時隨機產生（每次啟動都不同）
  CbbaComm(const AgentState & state, const ScoringParams & params, const CommConfig & config,
    std::uint64_t session_id = 0);

  // ---------- 本機輸入 ----------
  void setState(const AgentState & state) {agent_.setState(state);}
  void setStatus(const VehicleStatus & status) {status_ = status;}
  void setParticipating(bool participating, double now);
  bool participating() const {return agent_.active();}

  // BT 發現的新任務。task.id 的建立者必須是自己（task_id >> 16 == 機號）。
  // task_id 已知、建立者不對時回傳 false。還沒和鄰居同步時先排隊（回傳 true），同步後才發現重複的
  // 記在 stats().rejected_local_tasks。
  bool addLocalTask(const Task & task, double now);

  // 正式指派：請 BT 接受時呼叫，回傳新版本（已知的最新版本 + 1，狀態「待接受」）
  std::uint64_t beginAssignment(TaskId id);
  // BT 接受：同一個版本改成「保留」或「執行中」。最新的紀錄已經不是這個版本（被取代、已作廢）時回傳 false
  bool confirmAssignment(TaskId id, std::uint64_t version, AssignState state);
  // 結束自己的指派（撤銷、拒絕、逾時、執行失敗）：最新的紀錄是自己、任務還開著、沒在確認完成時，
  // 產生新版本（執行者 0、狀態「無」）。回傳 true 表示有結束
  bool endAssignment(TaskId id);
  Assignment assignment(TaskId id) const;
  // 鎖著、但持有者失聯（s 超過 lost_timeout 沒更新）的任務：任務 → 持有者。只列出、不處理
  std::vector<std::pair<TaskId, AgentId>> lockedByLostAgents(double now) const;

  // BT 的執行狀態（AGENT_STATE 用）與造成的限制（交給核心）
  void setExecReport(const ExecReport & report) {exec_report_ = report;}
  void setExecConstraints(const ExecConstraints & c, double now);
  bool suspendTask(TaskId id, double now);
  void resumeSuspended(double now);
  void excludeTask(TaskId id, double now);   // BT 永久拒絕：不再出價

  // BT 回報任務結果。version 必須是自己目前「執行中」的正式指派。
  // success：開始完成確認；失敗：結束指派、釋放回競標池，自己不再對它出價（任務保留）。
  ReportCheck reportResult(TaskId id, bool success, std::uint64_t version, double now);

  // 取消任務：只有建立者或 cancel_authorities 裡的機號能發。送 TASK_CLOSE（reason = 取消）
  ReportCheck cancelTask(TaskId id, double now);

  void receive(const std::uint8_t * data, std::size_t size, double now);
  void receive(const Bytes & packet, double now) {receive(packet.data(), packet.size(), now);}

  // 定時呼叫（建議 10 ms）：回傳現在要廣播的封包
  std::vector<Bytes> poll(double now);

  // ---------- 狀態查詢 ----------
  AgentId id() const {return agent_.state().id;}
  std::uint64_t sessionId() const {return session_;}
  const CbbaAgent & agent() const {return agent_;}
  // 目前要交給 BT 的任務：路徑上第一個還沒回報完成的任務
  std::optional<Task> currentTask() const;
  bool knows(TaskId id) const {return origins_.count(id) > 0;}
  bool taskOpen(TaskId id) const {return isOpen(id);}
  std::size_t taskCount() const {return origins_.size();}
  bool completionPending(TaskId id) const;
  bool joined() const {return joined_;}
  bool converged(double now) const;
  std::uint32_t roundKey() const;
  std::uint16_t exchangeStep() const {return exchange_step_;}
  std::vector<AgentId> aliveNeighbors(double now) const;
  const std::map<AgentId, NeighborInfo> & neighbors() const {return neighbors_;}
  const CommStats & stats() const {return stats_;}
  wire::AgentStateBody agentStateBody(double now) const;
  wire::CbbaStateBody cbbaStateBody(double now) const;   // 分批之前的完整內容

private:
  struct PendingTask
  {
    Task task;
    double queued{0.0};
  };
  struct PendingCompletion
  {
    std::uint32_t round_key{0};
    std::uint64_t version{0};
    std::vector<AgentId> view;
    std::map<AgentId, std::uint32_t> acks;   // 確認者 → 收到的那份確認的 origin_seq
    int tries{0};
    double last_sent{-1e9};
  };

  // 送出
  wire::Header newHeader(wire::PacketType type);
  void send(Bytes bytes, wire::PacketType type);
  void sendCbbaState(double now);
  void sendAgentState(double now);

  // 收到
  void onAgentState(const wire::Header & h, const Bytes & bytes);
  NeighborInfo & neighborOf(const wire::Header & h);
  void onCbbaState(const wire::Header & h, const Bytes & bytes, double now);
  void onTaskAnnounce(const Bytes & bytes, double now);
  void onCompletion(const Bytes & bytes, double now);
  void onCompletionAck(const Bytes & bytes, double now);
  void onTaskClose(const Bytes & bytes, double now);
  void relay(const Bytes & bytes);
  void resendMissing(const NeighborInfo & n, double now);

  // 任務
  bool assign(const Task & task, double now);
  bool addTask(const wire::RelayHeader & origin, const Task & task, double now);
  void resendTask(TaskId id, double now);
  void resendClose(TaskId id, double now);
  bool isOpen(TaskId id) const;

  // 完成
  void sendAnnounce(TaskId id, double now);
  bool allAcked(const PendingCompletion & pc, double now) const;
  void finishCompletion(TaskId id, double now);
  void markClosed(TaskId id, std::uint8_t reason, double now);
  bool mergeAssignment(TaskId id, const Assignment & a);
  bool closeAllowed(const wire::TaskCloseBody & close) const;
  bool canCancel(AgentId actor, TaskId id) const;

  // 狀態
  std::set<AgentId> aliveSet(double now) const;   // 自己 + 直接鄰居 + 鄰居的鄰居
  std::map<TaskId, AgentId> winnerMap() const;    // 進行中的任務 → 得標者
  bool disagrees(double now) const;
  void markChanged(bool changed);
  void updateJoined(double now);

  CbbaAgent agent_;
  CommConfig config_;
  std::uint64_t session_{0};

  std::map<TaskId, wire::RelayHeader> origins_;   // 任務 → TASK_ANNOUNCE 的 origin（補發時沿用）
  std::vector<PendingTask> queued_;
  std::map<TaskId, PendingCompletion> completions_;
  std::map<std::tuple<AgentId, TaskId, std::uint64_t>, std::uint8_t> decisions_;   // (執行機, 任務, 版本) → ACK status
  std::map<TaskId, Assignment> assignments_;   // 各任務已知的最新正式指派
  std::map<TaskId, std::uint64_t> used_versions_;   // 自己發過的最大版本：作廢的版本不再使用
  ExecReport exec_report_;
  void updateLocks(double now);
  std::map<TaskId, wire::TaskCloseBody> closes_;
  // 比任務定義先到的 TASK_CLOSE（補發時兩者延遲不同），等任務到了再套用
  std::map<TaskId, wire::TaskCloseBody> early_closes_;
  std::map<std::pair<int, TaskId>, double> last_resend_;
  std::map<AgentId, NeighborInfo> neighbors_;
  std::map<AgentId, std::uint32_t> core_seq_;   // 交給核心的訊息序號（封包層已過濾舊封包）

  wire::SeqCounter seq_;
  wire::SessionFilter filter_;
  wire::RelayDeduper dedup_;
  std::vector<Bytes> outbox_;

  VehicleStatus status_;

  double start_{-1.0};
  bool joined_{false};
  bool dirty_{false};
  std::uint16_t exchange_step_{0};
  std::uint16_t snapshot_id_{0};
  int fast_count_{0};
  double last_state_sent_{-1e9};
  double next_periodic_{0.0};
  double next_status_{0.0};
  double next_reevaluate_{0.0};
  double next_replay_{0.0};
  std::optional<TaskId> replay_cursor_;
  std::map<TaskId, AgentId> last_winners_;
  int stable_steps_{0};

  CommStats stats_;
};

}  // namespace cbba_core
