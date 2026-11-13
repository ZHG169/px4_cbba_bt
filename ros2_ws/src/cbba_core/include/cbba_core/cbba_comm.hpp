// CBBA 的機間協定層（不依賴 ROS，也不碰 socket）：協定版本 3（2026-10-28；版本 2 是 2026-10-09）
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
//   TASK_ANNOUNCE 建立時一次。保底：每 5 s 輪流重播一個已知任務
//   補發（版本 3，2026-10-28，見 doc/protocol_v3_sync.md）：
//                 CBBA_STATE 帶壓縮摘要（每個建立者一筆），收到的一方逐筆比對，某個建立者不同就送
//                 GAP_REQUEST（那段流水號的點陣圖）給那個鄰居，鄰居只把我缺的放進補發佇列；同時把自己那個建立者
//                 最近 push_recent_sec 內新增、結束的任務直接放進佇列（不等請求）。佇列用 SYNC_BATCH 分批送出，
//                 token bucket 限速（repair_rate）。版本 2 是「round_key 不同就把所有已結束的任務各補一次」，
//                 流量和已結束的任務數成正比（弱網測試 13 → 78 kB/s）
//   完成          COMPLETION → COMPLETION_ACK → TASK_CLOSE（見 wire_completion.hpp）
//
// 收到 CBBA_STATE
//   每一批都直接交給 CBBA 規則（規則逐任務判斷；每批都帶完整的 s）。還不認得的任務略過，不暫存：
//   200 ms 後的下一次會再帶，任務定義靠補發（摘要比對出缺口）。
//   收斂判定、補發只用收齊的那一次。缺批時等下一次，不會把缺少的紀錄當成任務被刪掉。
//
// 正式指派與回報（2026-10-09）
//   節點請 BT 接受時呼叫 beginAssignment()：產生新版本（已知的最新版本 + 1，執行者是自己，狀態「待接受」）。
//   BT 接受後 confirmAssignment() 把同一個版本改成「保留」或「執行中」；撤銷、拒絕、逾時、執行失敗時
//   endAssignment() 產生新版本（執行者 0、狀態「無」），舊的版本作廢、不再使用。
//   版本放在 CBBA_STATE 的紀錄裡傳遍全隊，各節點取（版本, 執行者, 狀態）較大的那筆。X→Y→X 時 X 第二次的版本一定比第一次大。
//   鎖定：別台「保留」或「執行中」的任務不出價（待接受不鎖）。持有者撤銷 → 重新開放；完成、取消 → 永久結束；
//   持有者失聯 → 先照樣鎖著（lockedByLostAgents() 列出來待確認），避免原載具還在執行；
//   失聯超過 takeover 時限（依優先級 takeover_by_priority）才撤銷（失聯接手，見下面）。
//   回報（reportResult）要帶版本，必須是自己目前的正式指派；完成宣告、TASK_CLOSE 也帶版本，
//   收到的節點知道更新的版本就拒絕（ACK status 2、不結束任務）。
//   執行失敗：撤銷指派、任務保留，由 CBBA 重新分配；不送 TASK_CLOSE。
//   取消（cancelTask）：只有建立者或 cancel_authorities 裡的機號能發；收到的節點也檢查。
//
// 失聯
//   某台的 s 超過 lost_timeout 沒更新，各機自己判定失聯，釋放它得標的任務。
//   某台從參與出價變成不參與時，立即釋放它的任務。
//
// 失聯接手（2026-10-11 定案、2026-10-12 實作）
//   「通訊斷了但還在做」和「墜毀」從外面分不出來，所以已接受（保留、執行中）的任務在持有者失聯後先鎖著，
//   失聯（s + lost_timeout）之後再過 takeover_by_priority[P]（2026-11-13 起依優先級：P0 10 s 寧可重複 … P2／P3 30 s
//   寧可等；舊型的火警是 P0、巡檢是 P3，和原本的 10／30 s 相同）才撤銷：產生新版本（執行者 0、狀態「無」），和持有者自己撤銷一樣，
//   各機各自判定、產生的紀錄相同（版本取大的），任務重新競標。
//   失聯的持有者照樣把手上的做完；恢復連線後收到新版本，它的指派被取代（AssignmentManager 撤下），
//   它的回報、完成宣告帶舊版本，被拒絕（ACK status 2），不會把已改派的任務關掉。
//
// 重開機：每次建立 CbbaComm 都用新的 session_id（隨機），鄰居看到新的 session 就知道重開機過，
//   重開機前在網路上延遲的封包會被丟掉；不需要把序號存在非揮發性儲存（版本 1 的 seq_file）。
//
// 時間一律是系統時鐘的秒數（各機需對時）。
#pragma once

#include <array>
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
#include "cbba_core/wire_sync.hpp"
#include "cbba_core/wire_task_announce.hpp"
#include "cbba_core/wire_task_spec.hpp"

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
  // 失聯接手：持有者失聯後多久撤銷它已接受的任務，依優先級 P0～P3（合約第三節；原本的 takeover_fire 10 s＝P0、
  // takeover_patrol 30 s＝P3）
  std::array<double, 4> takeover_by_priority{{10.0, 15.0, 30.0, 30.0}};
  double completion_retry{1.0};   // 完成宣告的重送間隔
  int completion_max{10};         // 完成宣告最多重送幾次
  double resend_gap{1.0};         // 同一個任務重送（保底重播、直接推送）的最短間隔
  // 補發（版本 3，doc/protocol_v3_sync.md）
  double digest_period{1.0};      // 收斂後多久在 CBBA_STATE 帶一次摘要（不一致時每次都帶）
  double gap_request_gap{0.5};    // 同一個（鄰居, 建立者）的缺口請求最短間隔
  double push_recent_sec{10.0};   // 摘要不同時，直接推送這麼久之內新增、結束的任務
  double repair_rate{8000.0};     // 補發（SYNC_BATCH）的平均上限（B/s）
  double repair_burst{2400.0};    // token bucket 的容量（B）；至少一個 max_packet
  double replay_period{5.0};      // 保底重播的間隔
  double reevaluate_period{1.0};  // 依目前位置、電量重新評估手上的任務
  int stable_steps{3};            // 收斂判定：連續幾個週期分配不變
  double join_wait{1.5};          // 啟動後這麼久沒聽到任何鄰居：單獨一台，開始廣播自己的任務
  double join_timeout{5.0};       // 有鄰居但摘要一直對不上時最多等多久
  std::size_t max_packet{wire::kDefaultMaxPacket};   // CBBA_STATE 每批（整個封包）的上限
  std::vector<AgentId> cancel_authorities;   // 除了建立者，可以取消任何任務的機號（授權的管理端），全隊要一樣
  // 協定版本 4（2026-11-11，cbba_m0_contract.md 第六節）
  Epoch epoch{0};                 // 這次啟動的世代（節點層讀存檔、+ 1、寫回後給）；0 = 不能產生版本 4 的 task_id
  // 同機號偵測：同一個機號的兩個 session，較早的那個在較晚的出現 conflict_grace 秒之後還在講話 → 兩台用了同一個機號
  //（真的重開機時，舊 session 只會有在網路上延遲的封包，不會超過幾百 ms）
  double conflict_grace{1.0};
  double conflict_release{5.0};   // 留下的（較早的）session 這麼久沒聲音 → 解除，改接受較晚的（較早的那台關掉了）
  // TaskSpec（版本 4，2026-11-12）
  double spec_pull_period{1.0};   // 缺 TaskSpec 的進行中任務：每這麼久向一個鄰居要一次（WAITING_SPEC 超時改成每 gap_request_gap 向每個鄰居要）
  double spec_keep_sec{60.0};     // 任務結束這麼久之後丟掉 TaskSpec 的內容（CRC 留在任務定義裡）
  // 閘道（2026-11-13 使用者定義）：非競標的 CBBA 協定節點，負責建立、同步、監控與關閉任務；不屬於任何載具平台，
  // 不列入任務可行性與 optional 角色的能力統計。永遠不出價（setParticipating(true) 不理會），AGENT_STATE 的
  // vehicle_type = 0、capabilities = 0。轉送、補發、完成確認、TASK_CLOSE 照常
  bool gateway{false};
};

// 同機號衝突（版本 4）：兩台用了同一個機號。留下較早出現的 session，較晚的 session 的封包、它建立的任務都拒收
struct AgentConflict
{
  AgentId agent{kNoAgent};
  std::uint64_t kept_session{0};
  std::uint64_t rejected_session{0};
  double since{0.0};
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
  bool committed{false};   // 執行中的任務已接近（commit_radius 內），不可再被搶占（AssignmentManager 算）
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

// 摘要的鍵（版本 4）：（建立者, 世代）；舊型 ID 的世代是 0
using DigestKey = std::pair<AgentId, Epoch>;

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
  // 摘要（版本 3）：digest_key 是帶摘要那一則的 round_key；它最新的 round_key 等於 digest_key 時摘要才是最新的
  bool has_digest{false};
  std::uint32_t digest_key{0};
  std::map<DigestKey, wire::DigestEntry> digest;   // （建立者, 世代）→ 摘要
  std::uint16_t pending_snapshot{0};
  std::uint8_t pending_count{0};
  std::set<std::uint8_t> pending_parts;
  std::map<TaskId, AgentId> pending_view;
  // AGENT_STATE（sequence 比上一則舊的丟掉）
  bool has_status{false};
  double last_status{0.0};           // 上一則 AGENT_STATE 的時刻（閘道的 membership_ready 用）
  bool gateway{false};               // 它是閘道（AGENT_STATE 的 vehicle_type = 0）：不是載具，不算進能力統計
  bool waiting_spec{false};          // 它排第一的任務在等 TaskSpec（版本 4 的 AGENT_STATE）
  bool committed{false};             // 它執行中的任務已接近目標（commit_radius 內），不可再被搶占
  std::uint32_t status_seq{0};
  wire::StateFlags flags;
  AgentType vehicle_type{AgentType::UAV};   // 載具的平台（閘道時沒有意義）
  Capabilities capabilities{0};      // 它目前可用的能力（版本 4 的 AGENT_STATE；閘道一定是 0）
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

// 送出封包的分類（2026-11-09，第 5 週頻寬圖：丟包率提高後重傳增加多少）：
//   BASE   週期的 AGENT_STATE／CBBA_STATE、變動時立即送的 CBBA_STATE、第一次的任務公告／完成宣告／結束、確認
//   RETRY  為了補掉包而多送的：CBBA_STATE 的快速重送（和鄰居不一致）、任務定義與結束的重送（resendTask／resendClose）、
//          完成宣告的第 2 次以後、缺口請求、補發批次
//   RELAY  替別台轉送（多跳）
enum class TxKind : std::uint8_t { BASE, RETRY, RELAY };

struct CommStats
{
  std::map<wire::PacketType, int> sent;
  std::map<wire::PacketType, std::size_t> sent_bytes;
  std::size_t tx_bytes{0};        // 送出的位元組（全部，送進 UDP 之前；tc 丟掉的也算）
  std::size_t tx_retry_bytes{0};  // 其中 RETRY
  std::size_t tx_relay_bytes{0};  // 其中 RELAY
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
  int takeovers{0};             // 失聯接手：撤銷失聯持有者的鎖定
  // 補發（版本 3）
  int gap_requests{0};          // 送出的缺口請求
  int gap_answers{0};           // 回答了的缺口請求（有東西可補）
  int repair_items_sent{0};     // SYNC_BATCH 送出的筆數（任務定義＋結束紀錄）
  int repair_items_applied{0};  // 收到的 SYNC_BATCH 筆數
  int repair_deferred{0};       // 補發額度不夠、留到下次的次數
  int spec_parts_sent{0};       // 送出的 TaskSpec 段（建立時；補發算在 repair_items_sent）
  int spec_parts_received{0};   // 收到的新段（不含重複）
  int spec_bad{0};              // 收齊卻長度、CRC 對不上，或 origin 不是建立者的段
  int spec_requests{0};         // 為了缺 TaskSpec 送出的缺口請求
  int id_conflicts{0};          // 偵測到的同機號衝突（含別台用了自己的機號）
  int conflict_dropped{0};      // 因為同機號衝突拒收的封包、任務定義
  // 改版（revision，2026-11-13）
  int revisions{0};             // 套用的改版（含自己建立的）
  int reauctions{0};            // 其中改了平台、能力、優先級，重新競標的
  int rejected_revisions{0};    // 不是建立者送的改版（不套用、不轉送）
  int revision_pulls{0};        // 看到鄰居的紀錄是較新的 revision，向它要任務定義的次數
  // 過期（2026-11-13）
  int expired_local{0};         // 本機判定過期（停止出價、撤銷自己的指派）
  int expired_closes{0};        // 自己是建立者、送出過期的 TASK_CLOSE
  // optional 角色（2026-11-13）
  int optional_drops{0};        // 送出的略過（TASK_CLOSE reason 4，閘道）
  int optional_local_drops{0};  // O3 本機判定：同組必要角色都結束了，自己不再對 optional 角色出價（不關任務）
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
  bool isGateway() const {return config_.gateway;}

  // BT 發現的新任務。task.id 的建立者必須是自己（taskCreator(task_id) == 機號）。
  // task_id 已知、建立者不對時回傳 false。還沒和鄰居同步時先排隊（回傳 true），同步後才發現重複的
  // 記在 stats().rejected_local_tasks。
  // spec：TaskSpec（JSON，原文；版本 4）。不是空的時填好任務定義的 spec_length、spec_crc，任務定義之後送 TASK_SPEC 各段；
  // 超過 255 段時回傳 false
  bool addLocalTask(const Task & task, double now, const std::string & spec = std::string());

  // 改版（2026-11-13，合約第三節「版本」）：自己建立、還開著的任務換成新內容。task.revision 要比已知的大；
  // spec 是這個 revision 的 TaskSpec（空 = 沒有）。送新的 TASK_ANNOUNCE（新的 origin）與 TASK_SPEC。
  // 只改位置、時間、價值 → 正式指派保留；改了平台、能力、優先級（舊型的種類）→ 重新競標，持有者撤銷自己的指派。
  // 不是建立者、不認得、已結束、revision 沒有變大、spec 太大時回傳 false
  bool reviseLocalTask(const Task & task, double now, const std::string & spec = std::string());

  // TaskSpec（版本 4）：這個任務可以開始執行了嗎（不需要 TaskSpec，或這個 revision 的已經收齊、長度與 CRC 對）
  bool specReady(TaskId id) const;
  // 收齊的 TaskSpec（不需要時是空字串；還沒收齊時 nullopt）
  std::optional<std::string> taskSpec(TaskId id) const;
  // 指派流程告訴通訊層：哪個任務排第一在等 TaskSpec（0 = 沒有）；urgent = 等超過 spec_wait，改向每個鄰居要
  void setWaitingSpec(TaskId id, bool urgent) {waiting_spec_ = id; spec_urgent_ = urgent;}

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
  // 失聯接手的時限（秒）：takeover_by_priority[任務的優先級]
  double takeoverLimit(TaskId id) const;
  // 上次呼叫之後，因失聯接手撤銷的任務（任務 → 原持有者），給節點印 log
  std::vector<std::pair<TaskId, AgentId>> takeTakeovers();

  // BT 的執行狀態（AGENT_STATE 用）與造成的限制（交給核心）
  void setExecReport(const ExecReport & report) {exec_report_ = report;}
  void setExecConstraints(const ExecConstraints & c, double now);
  bool suspendTask(TaskId id, double now);
  void resumeSuspended(double now);
  void excludeTask(TaskId id, double now, double duration = kInf);   // 不再出價（duration 秒；kInf = 永久）
  void countFailure(TaskId id, double now);   // TRANSIENT 失敗：之後對它的出價打折（CbbaAgent::countFailure）

  // BT 回報任務結果。version 必須是自己目前「執行中」的正式指派。
  // success：開始完成確認；失敗：結束指派、釋放回競標池，自己 exclude_sec 秒內不再對它出價（任務保留；kInf = 永久）。
  ReportCheck reportResult(TaskId id, bool success, std::uint64_t version, double now,
    double exclude_sec = kInf);

  // 取消任務：只有建立者或 cancel_authorities 裡的機號能發。送 TASK_CLOSE（reason = 取消）
  ReportCheck cancelTask(TaskId id, double now);

  // 過期（2026-11-13，合約第三節「過期」）：任務的 expires 到了（絕對時刻，全隊結果一樣，不靠建立者在線）。
  // 各節點本機：永久不再出價、撤銷自己的指派（已回報完成、在等確認的不撤）；建立者另外送 TASK_CLOSE（reason 3 = 過期），
  // 建立者不在線時任務先開著、沒人出價，建立者回來後看到就關。poll() 裡判斷
  bool expired(TaskId id, double now) const;
  std::vector<TaskId> takeExpired();   // 上次呼叫之後本機判定過期的（給節點印 log）
  // 已結束任務的 TASK_CLOSE reason（1 完成、2 取消、3 過期、4 略過）；還開著、或不知道原因時 nullopt
  std::optional<std::uint8_t> closeReason(TaskId id) const;

  // optional 角色（2026-11-13，合約第三節「optional 角色的自動關閉」）
  // 略過：送 TASK_CLOSE reason 4（閘道的 O1～O3 用）。要是 optional 角色、還開著、自己能取消（建立者或 cancel_authorities）
  ReportCheck dropOptional(TaskId id, double now);
  // 同一組的必要角色（不是 optional 的）都結束了（完成、取消、過期）。這一組 role_count 個角色要都認得才判斷
  bool groupMandatoryDone(TaskId group_id, std::uint8_t role_count) const;

  void receive(const std::uint8_t * data, std::size_t size, double now);
  void receive(const Bytes & packet, double now) {receive(packet.data(), packet.size(), now);}

  // 定時呼叫（建議 10 ms）：回傳現在要廣播的封包
  std::vector<Bytes> poll(double now);

  // ---------- 狀態查詢 ----------
  AgentId id() const {return agent_.state().id;}
  std::uint64_t sessionId() const {return session_;}
  Epoch epoch() const {return config_.epoch;}
  // 版本 4 的新 task_id（建立者 = 自己、這次的世代、流水號從 1 起）。沒有世代（epoch = 0）或流水號用完時 nullopt。
  // 給閘道注入任務用（第 4 步）；機器人的 BT 照舊用舊型 ID（new_task）
  std::optional<TaskId> newTaskId();
  // 同機號衝突：別台（key = 機號），以及別台用了自己的機號（它的 session；最近 conflict_release 秒內還聽得到才算）
  const std::map<AgentId, AgentConflict> & conflicts() const {return conflicts_;}
  std::optional<std::uint64_t> selfConflict(double now) const;
  const CbbaAgent & agent() const {return agent_;}
  const CommConfig & config() const {return config_;}
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
  // 能做這個任務的載具（平台、能力符合）：自己（不是閘道時）＋收過 AGENT_STATE、還聽得到的直接鄰居。
  // 閘道一律不算（2026-11-13 的定義）。不看是否參與出價、電量、排除：給閘道的機隊可行性、optional 關閉 O1、
  // 孤兒任務的原因用（O1 要不要只算參與出價的，第 4e 步再定）
  std::vector<AgentId> capableAgents(const Task & task, double now) const;
  const std::map<AgentId, NeighborInfo> & neighbors() const {return neighbors_;}
  const CommStats & stats() const {return stats_;}
  wire::AgentStateBody agentStateBody(double now) const;
  wire::CbbaStateBody cbbaStateBody(double now) const;   // 分批之前的完整內容（不含摘要）
  // 壓縮摘要（版本 3）：建立者 → 最小、最大流水號、知道幾個、已結束幾個、CRC
  std::map<DigestKey, wire::DigestEntry> digestMap() const;
  std::size_t repairQueued() const {return repair_open_.size() + repair_spec_.size() + repair_closed_.size();}

private:
  struct PendingTask
  {
    Task task;
    double queued{0.0};
    std::string spec;
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
  void send(Bytes bytes, wire::PacketType type, TxKind kind = TxKind::BASE);
  void sendCbbaState(double now, bool retry = false);
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
  void applyAnnounce(const wire::TaskAnnounceBody & b, double now);
  void applyClose(const wire::TaskCloseBody & b, double now);
  // 改版套用到本機（自己建立的、TASK_ANNOUNCE、SYNC_BATCH 共用；呼叫前已確認 origin 是建立者）
  void applyRevision(const wire::RelayHeader & origin, const Task & task, double now);
  // 鄰居的紀錄是較新的 revision：向它要那一個任務的定義（缺口請求，那個流水號標成不認得）
  void pullRevisions(double now);
  std::map<TaskId, AgentId> newer_revision_;     // 任務 → 有較新 revision 的鄰居
  std::map<TaskId, double> last_revision_pull_;

  // 補發（版本 3）
  bool wantDigest(double now) const;
  void compareDigest(AgentId neighbor, const NeighborInfo & n, double now);
  void sendGapRequest(AgentId target, DigestKey stream, std::uint32_t lo, std::uint32_t hi, double now);
  void onGapRequest(const Bytes & bytes);
  void onSyncBatch(const wire::Header & h, const Bytes & bytes, double now);
  struct RepairNeed
  {
    bool announce{false};
    bool close{false};
  };
  void queueRepair(TaskId id, bool announce, bool close);
  void drainRepair(double now);
  wire::SeqState seqState(TaskId id) const;

  // 任務
  wire::TaskAnnounceBody announceBody(TaskId id) const;
  bool assign(const Task & task, double now, const std::string & spec = std::string());
  void sendSpecParts(TaskId id, std::uint16_t revision, const std::string & spec, double now);

  // TaskSpec（版本 4）
  struct SpecStore
  {
    std::uint16_t revision{0};
    std::uint8_t encoding{wire::kSpecEncodingJson};
    std::uint8_t part_count{0};                    // 0 = 還沒有任何一段
    std::vector<std::vector<std::uint8_t>> data;   // 每段的資料
    std::vector<wire::RelayHeader> origin;         // 每段原本的 origin（補發沿用）
    std::vector<bool> have;
    bool complete{false};                          // 收齊、長度與 CRC 都對（對照任務定義）
    std::string json;
  };
  void onTaskSpec(const Bytes & bytes, double now);
  void applySpecPart(const wire::TaskSpecBody & b);
  void completeSpec(TaskId id);
  void pullSpecs(double now);
  void forgetSpecs(double now);
  void queueSpecRepair(TaskId id);
  wire::TaskSpecBody specPartBody(TaskId id, std::uint8_t part) const;
  std::map<TaskId, SpecStore> specs_;
  std::set<std::pair<TaskId, std::uint8_t>> repair_spec_;   // 補發佇列：TaskSpec 的段（進行中任務的定義之後送）
  std::map<TaskId, double> last_spec_pull_;
  std::size_t spec_pull_cursor_{0};
  TaskId waiting_spec_{0};
  bool spec_urgent_{false};
  bool addTask(const wire::RelayHeader & origin, const Task & task, double now);
  void resendTask(TaskId id, double now);
  void resendClose(TaskId id, double now);
  bool isOpen(TaskId id) const;

  // 完成
  void sendAnnounce(TaskId id, double now);
  bool allAcked(const PendingCompletion & pc, double now) const;
  void finishCompletion(TaskId id, double now);
  void markClosed(TaskId id, std::uint8_t reason, double now);
  void sendClose(TaskId id, std::uint8_t reason, double now);   // 自己發的取消、過期、略過
  void expireTasks(double now);
  void dropOptionalLocally(double now);   // O3 的本機判定（不是閘道的節點）
  std::set<TaskId> optional_local_;       // 已經本機停止出價的 optional 角色
  double next_optional_check_{0.0};
  std::set<TaskId> expired_;          // 本機判定過期、還開著的（建立者要送 TASK_CLOSE）
  std::vector<TaskId> newly_expired_;
  double next_expiry_check_{0.0};
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
  std::uint32_t next_seq_{0};   // newTaskId 上一次用的流水號

  // 同機號偵測（版本 4）
  struct SessionSeen
  {
    double first{0.0};
    double last{0.0};
  };
  bool trackSession(const wire::Header & h, double now);   // true：這個封包來自被拒收的 session
  bool rejectedOrigin(const wire::RelayHeader & origin) const;
  void releaseConflicts(double now);
  std::map<AgentId, std::map<std::uint64_t, SessionSeen>> sessions_;
  std::map<AgentId, AgentConflict> conflicts_;
  std::uint64_t self_conflict_session_{0};
  double self_conflict_last_{-1e9};

  std::map<TaskId, wire::RelayHeader> origins_;   // 任務 → TASK_ANNOUNCE 的 origin（補發時沿用）
  std::vector<PendingTask> queued_;
  std::map<TaskId, PendingCompletion> completions_;
  std::map<std::tuple<AgentId, TaskId, std::uint64_t>, std::uint8_t> decisions_;   // (執行機, 任務, 版本) → ACK status
  std::map<TaskId, Assignment> assignments_;   // 各任務已知的最新正式指派
  std::map<TaskId, std::uint64_t> used_versions_;   // 自己發過的最大版本：作廢的版本不再使用
  ExecReport exec_report_;
  void updateLocks(double now);
  void takeOverLost(double now);
  std::map<TaskId, double> lost_since_;   // 鎖著、持有者失聯的任務 → 持有者失聯的時刻
  std::vector<std::pair<TaskId, AgentId>> takeovers_;
  std::map<TaskId, wire::TaskCloseBody> closes_;
  // 比任務定義先到的 TASK_CLOSE（補發時兩者延遲不同），等任務到了再套用
  std::map<TaskId, wire::TaskCloseBody> early_closes_;
  std::map<std::pair<int, TaskId>, double> last_resend_;
  // 補發（版本 3）
  std::map<TaskId, double> added_at_;    // 本機知道這個任務的時刻（直接推送用）
  std::map<TaskId, double> closed_at_;   // 本機知道這個任務結束的時刻
  std::map<std::pair<AgentId, DigestKey>, double> last_gap_request_;   // (鄰居, (建立者, 世代)) → 上次請求
  std::map<std::pair<AgentId, DigestKey>, std::uint64_t> gap_cursor_;  // 範圍太大時下次從哪個流水號問
  std::map<TaskId, RepairNeed> repair_open_;     // 補發佇列：進行中的任務（先送）
  std::map<TaskId, RepairNeed> repair_closed_;   // 已結束的
  double repair_tokens_{0.0};
  double repair_last_{-1.0};
  double last_digest_sent_{-1e9};
  std::uint32_t last_digest_key_{0};
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
