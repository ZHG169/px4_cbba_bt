// CBBA 的機間協定層（不依賴 ROS，也不碰 socket）
//
// 把 CbbaAgent 接上「機間通訊封包規格」的 AGENT_STATE、TASK_EVENT、COMPLETION、COMPLETION_ACK。
// 呼叫端只負責：收到的封包交給 receive()，poll() 回傳的封包廣播出去。
//
// 任務識別
//   ROS 與核心用 uint32 task_id（建立者機號 × 65536 + 流水號）；封包用 uint8 任務編號（0~254）。
//   任務編號各機自編、依機號交錯：第 k 台用 (k−1) + index_slots·j，彼此不會撞號。
//   task_id 寫成 8 位十六進位放在 TASK_EVENT 的名稱欄（規格稱 task_id），例如 "00030005"。
//   剛啟動的節點先和鄰居同步（摘要相同）才自編，避免重開機後撞到還沒補到的任務：
//     摘要和任一鄰居相同 → 開始；join_wait 內沒聽到任何鄰居 → 單獨一台，開始；
//     有鄰居但 join_timeout 內一直對不上 → 照樣開始，自編時跳過鄰居 M 以下的編號。
//
// AGENT_STATE 的欄位
//   round_key       已知任務與狀態（編號、是否已結束）的 CRC-32 摘要
//   exchange_step   本機分配表變動的次數
//   z、y            進行中的任務填得標者與得標價（無人得標為 0、0）；不認得或已結束填 0、−1
//   s               機號 k+1 的資訊時間：系統時鐘 ms 的低 32 位元；未知為 0
//   path            自己的任務編號，依執行順序
//   flags           鄰居、armed、offboard、遙測正常、連續幾個週期分配不變（跟隨中固定 0）
//   progress_task   目前指派給 BT 的任務編號（進度固定 0）
//   不參與出價時（例如還沒起飛）M、N、Lt 都是 0。
//
// 傳送時機
//   AGENT_STATE 每 200 ms；表有變動時立即；和鄰居的 z 不一致時每 50 ms（同一步最多 60 次）。
//   補發：鄰居的 round_key 和我不同時，逐格比對，補它缺的 TASK_EVENT 或完成證明（每個每秒最多一次）。
//         不參與出價的鄰居（還在地面、剛開機）沒有附 y，當成什麼都不知道，全部補一次。
//   保底：每 5 s 輪流重播一個已知任務（看不到出價表的飛機也補得到）。
//
// 任務完成（規格 §2.8、§2.10）
//   宣告 → 各機確認 → 收齊後送證明，全隊收到證明才標為 DONE。宣告沒收齊時每 1 s 重送，最多 10 次，
//   之後以收到的確認送出證明。宣告後才失聯的飛機不再等。重送一律換新序號；
//   確認者對同一個宣告的決定不變。執行機以確認者辨認確認，同一台收到幾份都只算一次。
//
// 失聯
//   某台的 s 超過 lost_timeout 沒更新，各機自己判定失聯，釋放它得標的任務。
//   某台從參與出價變成不參與時，立即釋放它的任務。
//
// seq 區塊預約（見 wire_header.hpp 第 7 點）
//   重開機後 seq 不能重複用開機前用過的值，否則鄰居的去重複（記 60 s）會把新事件當成舊的丟掉。
//   呼叫端把「預約的上限」存在非揮發性儲存：開機時 setSeqStore(上次的上限, persist)，
//   之後 seq 每超過上限就先呼叫 persist(新上限 = seq + seq_block) 寫入，封包才離開 outbox。
//   沒有紀錄（第一次開機、檔案不見）時用系統時鐘 ms 當起點。
//
// 時間一律是系統時鐘的秒數（各機需對時）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "uav_cbba/cbba_agent.hpp"
#include "uav_cbba/wire_agent_state.hpp"
#include "uav_cbba/wire_completion.hpp"
#include "uav_cbba/wire_header.hpp"
#include "uav_cbba/wire_task_event.hpp"

namespace uav_cbba
{

using Bytes = std::vector<std::uint8_t>;

struct CommConfig
{
  double state_period{0.2};       // AGENT_STATE 週期
  double fast_period{0.05};       // 和鄰居不一致時的重送間隔
  int fast_max{60};               // 同一步最多快速重送幾次
  double min_gap{0.01};           // 立即發送的最短間隔
  double neighbor_timeout{1.5};   // 多久沒收到 AGENT_STATE 就不算直接鄰居
  double lost_timeout{1.5};       // 多久沒有某台的新資訊視為失聯（規格：1.5 s）
  double completion_retry{1.0};   // 完成宣告的重送間隔
  int completion_max{10};         // 完成宣告最多重送幾次
  double resend_gap{1.0};         // 補發同一個 TASK_EVENT／證明的最短間隔
  double replay_period{5.0};      // 保底重播的間隔
  double reevaluate_period{1.0};  // 依目前位置、電量重新評估手上的任務
  int stable_steps{3};            // 收斂判定：連續幾個週期分配不變
  int index_slots{8};             // 任務編號空間切成幾份（≥ 機數）
  double join_wait{1.5};          // 啟動後這麼久沒聽到任何鄰居：單獨一台，開始自編任務
  double join_timeout{5.0};       // 有鄰居但摘要一直對不上時最多等多久（之後跳過鄰居用過的編號）
  double default_deadline{300.0}; // 沒有期限資訊的任務（版本 1 TASK_EVENT）
  std::uint32_t seq_block{10000}; // seq 每次預約多少號（AGENT_STATE 每秒最多 20 則：約 8 分鐘寫一次）
};

struct NeighborInfo
{
  double last_heard{0.0};
  bool participating{false};
  std::uint8_t neighbor_bits{0};
  std::uint32_t round_key{0};
  std::vector<std::uint8_t> z;
  std::vector<float> y;
  int stable_steps{0};
};

struct CommStats
{
  std::map<wire::PacketType, int> sent;
  std::map<wire::PacketType, std::size_t> sent_bytes;
  std::map<wire::PacketType, int> received;
  int malformed{0};          // 解不開的封包
  int stale{0};              // seq 較舊而丟掉的
  int duplicates{0};         // 已處理過的轉送事件
  int relayed{0};
  int index_conflicts{0};    // 同一個編號對到不同 task_id（或反過來）
  int foreign_tasks{0};      // 名稱不是 8 位十六進位 task_id 的 TASK_EVENT（忽略）
  int rejected_acks{0};      // 收到 accepted = 0 的確認
  int join_timeouts{0};      // 啟動時沒和鄰居同步就開始自編（等滿 join_timeout）
  int seq_reserves{0};       // seq 預約寫入次數
  int seq_reserve_failures{0};   // 寫入失敗（照樣送出，重開機後可能撞號）
};

// task_id ↔ TASK_EVENT 的名稱（8 位十六進位）
std::string taskName(TaskId id);
std::optional<TaskId> parseTaskName(const std::string & name);

// 時間戳 s：系統時鐘 ms 的低 32 位元。fromStampMs 取「不晚於 now」最近的時間。
std::uint32_t toStampMs(double t);
double fromStampMs(std::uint32_t ms, double now);

class CbbaComm
{
public:
  CbbaComm(const AgentState & state, const ScoringParams & params, const CommConfig & config);

  // ---------- 本機輸入 ----------
  void setState(const AgentState & state) {agent_.setState(state);}
  void setStatus(bool armed, bool offboard, bool telemetry_ok);
  void setParticipating(bool participating, double now);

  // seq 區塊預約。stored：上次存下的上限（沒有紀錄給 nullopt，改用時鐘起點）；
  // persist(上限)：寫進非揮發性儲存，成功回傳 true。要在第一次 receive()／poll() 之前呼叫。
  void setSeqStore(std::optional<std::uint32_t> stored, std::function<bool(std::uint32_t)> persist);
  std::uint32_t seqLimit() const {return seq_limit_;}   // 目前預約到的上限
  bool participating() const {return agent_.active();}

  // BT 發現的新任務。task.id 的建立者必須是自己（task_id >> 16 == 機號）。
  // task_id 重複、建立者不對、自己的編號用完時回傳 false。
  bool addLocalTask(const Task & task, double now);

  // BT 回報任務結果。success：開始完成確認；失敗：釋放回競標池，自己不再對它出價。
  bool reportResult(TaskId id, bool success, double now);

  void receive(const std::uint8_t * data, std::size_t size, double now);
  void receive(const Bytes & packet, double now) {receive(packet.data(), packet.size(), now);}

  // 定時呼叫（建議 10 ms）：回傳現在要廣播的封包
  std::vector<Bytes> poll(double now);

  // ---------- 狀態查詢 ----------
  AgentId id() const {return agent_.state().id;}
  const CbbaAgent & agent() const {return agent_;}
  // 目前要交給 BT 的任務：路徑上第一個還沒回報完成的任務
  std::optional<Task> currentTask() const;
  std::optional<std::uint8_t> indexOf(TaskId id) const;
  std::size_t taskCount() const {return index_of_.size();}
  bool completionPending(TaskId id) const;
  bool joined() const {return joined_;}   // 已和鄰居同步，可以自編任務
  bool converged(double now) const;
  std::uint32_t roundKey() const;
  std::uint16_t exchangeStep() const {return exchange_step_;}
  std::vector<AgentId> aliveNeighbors(double now) const;
  const std::map<AgentId, NeighborInfo> & neighbors() const {return neighbors_;}
  const CommStats & stats() const {return stats_;}
  wire::AgentStateBody agentStateBody(double now) const;

private:
  struct TaskMeta
  {
    TaskId id{0};
    wire::RelayHeader origin;   // TASK_EVENT 的原始發送者與序號（補發時沿用）
  };
  struct PendingTask
  {
    Task task;
    double queued{0.0};
  };
  struct PendingCompletion
  {
    std::uint8_t index{0};
    std::uint32_t round_key{0};
    std::uint64_t revision{0};
    std::uint8_t view_bits{0};
    std::map<AgentId, std::uint64_t> acks;   // 確認者 → 收到的那份確認的序號
    int tries{0};
    double last_sent{-1e9};
  };

  // 送出
  wire::Header newHeader(wire::PacketType type, double now);
  void send(Bytes bytes, wire::PacketType type);
  void sendAgentState(double now);

  // 收到
  void onAgentState(const wire::Header & h, const Bytes & bytes, double now);
  void onTaskEvent(const Bytes & bytes, double now);
  void onCompletion(const Bytes & bytes, double now);
  void onCompletionAck(const Bytes & bytes, double now);
  void relay(const Bytes & bytes, double now);
  void seedSeq(double now);
  void resendMissing(const wire::AgentStateBody & b, double now);

  // 任務
  std::optional<std::uint8_t> nextIndex() const;
  bool assign(const Task & task, double now);
  bool addTask(std::uint8_t index, const TaskMeta & meta, const Task & task, double now);
  Bytes taskEventPacket(std::uint8_t index, double now);
  void resendTask(std::uint8_t index, double now);
  void resendProof(std::uint8_t index, double now);
  bool isOpen(std::uint8_t index) const;
  std::size_t indexCount() const;   // M = 最大編號 + 1

  // 完成
  void sendAnnounce(TaskId id, double now);
  bool allAcked(const PendingCompletion & pc, double now) const;
  void finishCompletion(TaskId id, double now);
  void markDone(std::uint8_t index, double now);

  // 狀態
  std::uint8_t neighborBits(double now) const;
  std::uint8_t aliveBits(double now) const;   // 自己 + 直接鄰居 + 鄰居的鄰居
  std::vector<std::uint8_t> zVector() const;
  bool disagrees(double now) const;
  void markChanged(bool changed);
  void updateJoined(double now);

  CbbaAgent agent_;
  CommConfig config_;

  std::map<std::uint8_t, TaskMeta> meta_;
  std::map<TaskId, std::uint8_t> index_of_;
  std::vector<PendingTask> queued_;
  std::map<TaskId, PendingCompletion> completions_;
  std::map<std::pair<AgentId, std::uint8_t>, bool> decisions_;   // (執行機, 編號) → 是否接受
  std::map<std::uint8_t, wire::CompletionBody> proofs_;
  // 比任務先到的完成證明（補發時兩者延遲不同），等任務到了再套用
  std::map<std::uint8_t, wire::CompletionBody> early_proofs_;
  std::map<std::pair<int, std::uint8_t>, double> last_resend_;
  std::map<AgentId, NeighborInfo> neighbors_;
  std::map<AgentId, std::uint32_t> core_seq_;   // 交給核心的訊息序號（封包層已過濾舊封包）

  wire::SeqCounter seq_;
  bool seq_seeded_{false};   // 第一次送封包前設定 seq 起點
  std::optional<std::uint32_t> seq_stored_;   // 上次預約的上限（從非揮發性儲存讀回）
  std::function<bool(std::uint32_t)> seq_persist_;
  std::uint32_t seq_limit_{0};
  wire::SeqFilter seq_filter_;
  wire::RelayDeduper dedup_;
  std::vector<Bytes> outbox_;

  bool armed_{false};
  bool offboard_{false};
  bool telemetry_ok_{false};

  double start_{-1.0};
  bool joined_{false};
  std::size_t index_floor_{0};   // 沒同步就開始自編時，自己的編號從這裡以上找
  bool dirty_{false};
  std::uint16_t exchange_step_{0};
  int fast_count_{0};
  double last_state_sent_{-1e9};
  double next_periodic_{0.0};
  double next_reevaluate_{0.0};
  double next_replay_{0.0};
  int replay_cursor_{-1};
  std::vector<std::uint8_t> last_z_;
  int stable_steps_{0};

  CommStats stats_;
};

}  // namespace uav_cbba
