// 機間通訊封包的表頭（不依賴 ROS）：協定版本 2（2026-10-09）
//
// 版本 2 不再照「機間通訊封包規格」2026-10-06 的格式。原因：規格用 AGENT_STATE 裡 z、y 陣列的位置
// （uint8 任務編號 0～254）當全隊共用的任務識別，所有節點必須先對「第 i 格是哪個任務」有共識：
//   - 任務編號要各機切份（依機號交錯），機號超過 8 的載具（狗 50）沒有自己的一份，不能建立任務；
//   - 每台一輩子只能建立約 32 個任務；鄰居位元只放得下機號 1～8；
//   - 重開機要先和鄰居同步才敢自編號碼。
// 版本 2 每筆任務資料都帶 uint32 task_id，各節點自己維護 task_id → 本機索引的對照，上面的限制都不存在。
// 封包種類、欄位見各 wire_*.hpp；整體說明見 doc/protocol.md。
//
// 共用表頭（每個封包都有）：20 B，網路位元組順序（大端序），逐欄位寫入
//   magic 2 | version 1 | type 1 | sender_id 2 | session_id 8 | sequence 4 | payload_length 2
//
//   magic           0x4342（"CB"），不是這個值就不是本協定的封包
//   version         2
//   type            封包種類（PacketType）
//   sender_id       直接發送者機號（轉送時為轉送者）。0 保留為「無人」，拒收
//   session_id      每次節點啟動隨機產生；(sender_id, session_id) 不同就是重開機過
//   sequence        同一個 session 內的封包序號，所有種類共用一個計數器
//   payload_length  表頭之後的位元組數，必須等於實際長度
//
// session_id + sequence 只用來丟掉重複或過期的封包、辨識轉送事件，不是任務指派版本，也不取代 CBBA 的時間戳 s。
// 取代版本 1 的「seq 區塊預約」（seq_file）：重開機換新的 session，(origin_id, origin_session, origin_seq)
// 不會和開機前的撞在一起，不需要把 seq 存在非揮發性儲存，也不受時鐘往回跳影響。
//
// 轉送區塊（TASK_ANNOUNCE、COMPLETION、COMPLETION_ACK、TASK_CLOSE 才有，接在表頭之後、算在 payload 裡）：14 B
//   origin_id 2 | origin_session 8 | origin_seq 4
//
//   轉送 = 換上自己的表頭（sender_id、session_id、sequence），payload（含轉送區塊）原封不動。
//   去重複的鍵是（種類, origin_id, origin_session, origin_seq）。
//   重送（不是轉送）一律用新的 sequence，origin_seq 也跟著換：沿用舊序號的重送會被轉送的飛機當成重複擋掉，
//   多跳時補不回來（詳見 wire_completion.hpp）。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

namespace cbba_core::wire
{

enum class PacketType : std::uint8_t
{
  AGENT_STATE = 1,     // 載具狀態（位置、電量、真實的 armed／offboard、鄰居、路徑），定期
  CBBA_STATE = 2,      // 共識資料（各任務的得標者、得標價、時間戳 s），可分批
  TASK_ANNOUNCE = 3,   // 任務定義，轉送
  COMPLETION = 4,      // 完成宣告，轉送
  COMPLETION_ACK = 5,  // 完成確認，轉送（只在需要時）
  TASK_CLOSE = 6,      // 任務結束（完成證明或取消），轉送
};

constexpr std::uint16_t kMagic = 0x4342;
constexpr std::uint8_t kVersion = 2;
constexpr std::size_t kHeaderSize = 20;
constexpr std::size_t kRelaySize = 14;
constexpr std::uint16_t kMaxAgentId = 0xFFFE;   // 0xFFFF 保留

struct Header
{
  PacketType type{PacketType::AGENT_STATE};
  std::uint16_t sender_id{0};
  std::uint64_t session_id{0};
  std::uint32_t sequence{0};
  std::uint16_t payload_length{0};
};

struct RelayHeader
{
  std::uint16_t origin_id{0};
  std::uint64_t origin_session{0};
  std::uint32_t origin_seq{0};
};

inline bool operator==(const RelayHeader & a, const RelayHeader & b)
{
  return a.origin_id == b.origin_id && a.origin_session == b.origin_session &&
         a.origin_seq == b.origin_seq;
}
inline bool operator!=(const RelayHeader & a, const RelayHeader & b) {return !(a == b);}

// 機號是否合法：1～0xFFFE
inline bool validAgentId(std::uint32_t id) {return id >= 1 && id <= kMaxAgentId;}

// 先寫表頭（payload_length 先填 0），內容寫完後用 finishPacket() 補上長度
void appendHeader(std::vector<std::uint8_t> & out, const Header & header);
// 依實際長度填 payload_length。payload 超過 65535 B 時丟出 std::invalid_argument
void finishPacket(std::vector<std::uint8_t> & packet);

// 解析表頭。長度不足、magic 或版本不對、種類不認得、sender_id 不合法、
// payload_length 和實際長度不符時回傳 nullopt。
std::optional<Header> parseHeader(const std::uint8_t * data, std::size_t size);
inline std::optional<Header> parseHeader(const std::vector<std::uint8_t> & bytes)
{
  return parseHeader(bytes.data(), bytes.size());
}

// 這種封包有沒有轉送區塊
bool isRelayed(PacketType type);

void appendRelay(std::vector<std::uint8_t> & out, const RelayHeader & relay);
// 解析表頭之後的轉送區塊（data 指向整個封包）。種類沒有轉送區塊、長度不足、origin_id 不合法時回傳 nullopt
std::optional<RelayHeader> parseRelay(const std::uint8_t * data, std::size_t size);
inline std::optional<RelayHeader> parseRelay(const std::vector<std::uint8_t> & bytes)
{
  return parseRelay(bytes.data(), bytes.size());
}

// 自己發出的事件：origin 就是自己
inline RelayHeader originOf(const Header & header)
{
  return RelayHeader{header.sender_id, header.session_id, header.sequence};
}

const char * toString(PacketType type);

// 發送端：同一個 session 的封包序號，第一則為 1
class SeqCounter
{
public:
  std::uint32_t next() {return ++seq_;}

private:
  std::uint32_t seq_{0};
};

// 轉送：換上自己的表頭（sender_id、session_id、sequence），payload 原封不動。
// 封包不合法或這種封包不轉送時回傳 nullopt。
std::optional<std::vector<std::uint8_t>> makeRelay(
  const std::vector<std::uint8_t> & packet, std::uint16_t my_id, std::uint64_t my_session,
  SeqCounter & counter);

// 收端：丟掉重複或過期的封包（每個發送者一個滑動視窗，像 IPsec 的 anti-replay）
//   同一個 session：sequence 在視窗內（最新的往回 window 號）而且沒收過就接受；收過的、比視窗還舊的丟掉。
//     亂序到達的照樣接受：一次送出很多則（大量 TASK_ANNOUNCE、分批的 CBBA_STATE）時，
//     只接受「比較新的」會把先送、晚到的那些全丟掉。狀態類封包的新舊由內容判斷
//     （CBBA_STATE 看 snapshot_id、AGENT_STATE 看 sequence，見 CbbaComm）。
//   新的 session：發送者重開機了，接受，舊的 session 記成已退役。
//   已退役的 session：重開機前、在網路上延遲的封包，丟掉。
class SessionFilter
{
public:
  explicit SessionFilter(std::uint32_t window = 4096)
  : window_(window) {}

  bool accept(const Header & header);

  // 某台目前的 session 有沒有換過（重開機）。第一次聽到不算
  bool restarted(std::uint16_t sender) const;

private:
  static constexpr std::size_t kRetiredMemory = 8;
  struct Sender
  {
    std::uint64_t session{0};
    std::uint32_t highest{0};
    std::set<std::uint32_t> seen;   // 視窗內收過的
    std::deque<std::uint64_t> retired;
    bool restarted{false};
  };
  void start(Sender & s, const Header & header);

  std::uint32_t window_;
  std::map<std::uint16_t, Sender> senders_;
};

// 轉送事件的去重複：同一則事件（種類, origin_id, origin_session, origin_seq）只處理、轉送一次。
// 記 memory 秒（預設 60 s）後忘記，記憶體不會無限增加；忘記之後收到同一則，會再當成新的處理。
class RelayDeduper
{
public:
  explicit RelayDeduper(double memory = 60.0)
  : memory_(memory) {}

  // 第一次看到回傳 true（應處理並轉送一次），之後回傳 false。自己發出的事件也要先登記。
  bool firstSeen(PacketType type, const RelayHeader & relay, double now);
  std::size_t size() const {return seen_.size();}

private:
  using Key = std::tuple<std::uint8_t, std::uint16_t, std::uint64_t, std::uint32_t>;
  void forget(double now);

  double memory_;
  std::map<Key, double> seen_;
  double last_cleanup_{0.0};
};

}  // namespace cbba_core::wire
