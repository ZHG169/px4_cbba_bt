// 機間通訊封包的表頭（不依賴 ROS），對應「五機蜂群：機間通訊封包規格」2026-10-06 §2.1、§2.2
//
// 共用表頭（每個封包都有）
//   type 1 B | agent_id 1 B | seq 4 B | stamp_us 8 B   共 14 B，小端序、不補位
//
//   type      低 5 bit = 封包種類代碼，高 3 bit = 版本（目前 1）
//   agent_id  直接發送者機號（轉送時為轉送者）
//   seq       每個發送者、每種封包各自遞增
//   stamp_us  發送時間（µs，系統時鐘；各機需對時）
//
// 轉送表頭（TASK_EVENT、LOST_EVENT、COMPLETION、COMPLETION_ACK、FORMATION_RELAY 才有，接在共用表頭之後）
//   origin_id 1 B | origin_seq 8 B   共 9 B
//
//   origin_id   原始發送者機號；轉送過程中不變
//   origin_seq  原始發送者送出時的 seq；轉送過程中不變
//   轉送 = 換上自己的共用表頭（agent_id、seq、stamp_us），轉送表頭與內容原封不動
//
// 和規格不同、以我們的情況為主的地方：
//   0. TASK_EVENT 另有版本 2（尾端加出價需要的欄位，見 wire_task_event.hpp），其他封包只接受版本 1。
//   1. agent_id、origin_id = 0 視為不合法（介面規格 v1.0：0 保留為「無人」），解碼時拒絕。
//   2. 規格沒寫收端怎麼用 seq。SeqFilter 以（發送者, 封包種類）分開記錄，丟掉舊的或重複的；
//      超過 reset_timeout 沒聽到某台的某種封包時，接受它較小的 seq（節點重開機後 seq 從 1 重算）。
//   3. 規格以（origin_id, origin_seq）去重複，但 seq 是「每種封包各自遞增」，同一台的第 5 則
//      TASK_EVENT 和第 5 則 COMPLETION 會撞在一起。RelayDeduper 的鍵加上封包種類。
//   4. 規格沒寫去重複要記多久。RelayDeduper 記 memory 秒（預設 60 s）後忘記，記憶體不會無限增加；
//      忘記之後收到同一則（例如補發），會再當成新的處理。
//   5. origin_seq 照規格用 8 B（uint64），雖然共用表頭的 seq 只有 4 B，高 4 B 永遠是 0。
//      保持和規格的封包大小一致。
//   6. 重送（不是轉送）一律用新的 seq，origin_seq 也跟著換。沿用舊序號的重送會被轉送的飛機
//      當成重複擋掉，多跳時補不回來（詳見 wire_completion.hpp）。「是不是同一件事」由收端看內容判斷。
//   7. 規格沒寫 seq 從多少開始。從 0 開始的話，重開機後的 (origin_id, origin_seq) 會和開機前的
//      一樣，鄰居的 RelayDeduper 還記得（60 s），新的事件會被當成重複丟掉；多跳時遠端也看不出
//      重開機，無法靠清記憶解決。所以重開機後 seq 要接著開機前的繼續編：
//      區塊預約（Zigbee、LoRaWAN 的封包計數器也這樣做）：把「預約的上限」存在非揮發性儲存，
//      先寫入新上限才使用這段號碼，開機時從上次的上限之後開始（見 CbbaComm::setSeqStore）。
//      沒有紀錄時退回系統時鐘 ms 的低 32 位元當起點（時鐘往回跳、沒對時會失效）。
//   8. SeqFilter 用循環序號比較（RFC 1982）：seq 從時鐘起算，執行約 49.7 天會繞回 0，
//      直接比大小會在繞回時誤丟 reset_timeout 秒的封包。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace uav_cbba::wire
{

enum class PacketType : std::uint8_t
{
  FORMATION = 1,
  AGENT_STATE = 2,
  ELECTION = 3,
  TASK_EVENT = 4,
  LOST_EVENT = 5,
  COMPLETION = 6,
  DOWNLINK = 7,
  COMPLETION_ACK = 8,
  FORMATION_RELAY = 9,
};

constexpr std::uint8_t kVersion = 1;
constexpr std::uint8_t kTaskEventVersion2 = 2;   // 我們的擴充，只用於 TASK_EVENT
constexpr std::size_t kHeaderSize = 14;
constexpr std::size_t kRelayHeaderSize = 9;

struct Header
{
  PacketType type{PacketType::AGENT_STATE};
  std::uint8_t version{kVersion};
  std::uint8_t agent_id{0};
  std::uint32_t seq{0};
  std::uint64_t stamp_us{0};
};

struct RelayHeader
{
  std::uint8_t origin_id{0};
  std::uint64_t origin_seq{0};
};

// type 位元組 = 代碼 | (版本 << 5)
inline std::uint8_t typeByte(PacketType type, std::uint8_t version = kVersion)
{
  return static_cast<std::uint8_t>(static_cast<std::uint8_t>(type) | (version << 5));
}

// 把 14 B 表頭接在 out 後面
void appendHeader(std::vector<std::uint8_t> & out, const Header & header);

// 解析開頭的 14 B。長度不足、代碼不在 1~9、版本不支援、agent_id 為 0 時回傳 nullopt。
// 支援的版本：所有封包的版本 1，以及 TASK_EVENT 的版本 2。
std::optional<Header> parseHeader(const std::uint8_t * data, std::size_t size);
inline std::optional<Header> parseHeader(const std::vector<std::uint8_t> & bytes)
{
  return parseHeader(bytes.data(), bytes.size());
}

// 這種封包有沒有轉送表頭
bool hasRelayHeader(PacketType type);

// 把 9 B 轉送表頭接在 out 後面
void appendRelayHeader(std::vector<std::uint8_t> & out, const RelayHeader & relay);

// 解析共用表頭之後的 9 B（data 指向整個封包）。種類沒有轉送表頭、長度不足、origin_id 為 0 時回傳 nullopt。
std::optional<RelayHeader> parseRelayHeader(const std::uint8_t * data, std::size_t size);
inline std::optional<RelayHeader> parseRelayHeader(const std::vector<std::uint8_t> & bytes)
{
  return parseRelayHeader(bytes.data(), bytes.size());
}

// 自己發出的事件：origin 就是自己，origin_seq = 這則的共用表頭 seq
inline RelayHeader originOf(const Header & header)
{
  return RelayHeader{header.agent_id, header.seq};
}

// 系統時鐘的秒數 → stamp_us
std::uint64_t toStampUs(double seconds);

const char * toString(PacketType type);

// 發送端：每種封包各自的流水號，第一則為 1
class SeqCounter
{
public:
  SeqCounter() = default;
  // 每種封包的第一個 seq 是 start + 1（開機時用上次預約的上限，見上面第 7 點）
  explicit SeqCounter(std::uint32_t start) {seq_.fill(start);}

  std::uint32_t next(PacketType type);

private:
  std::array<std::uint32_t, 32> seq_{};
};

// 轉送：把收到的封包換上自己的共用表頭（agent_id = my_id、seq 取自 counter、stamp_us = now），
// 轉送表頭與內容原封不動。封包不合法或這種封包沒有轉送表頭時回傳 nullopt。
std::optional<std::vector<std::uint8_t>> makeRelay(
  const std::vector<std::uint8_t> & packet, std::uint8_t my_id, SeqCounter & counter, double now);

// 收端：丟掉舊的或重複的封包（以發送者、封包種類分開記錄）
class SeqFilter
{
public:
  explicit SeqFilter(double reset_timeout = 3.0)
  : reset_timeout_(reset_timeout) {}

  // now 為收到的時間（秒）。回傳 false 表示應丟掉。seq 用循環比較（差值當成有號數）。
  bool accept(const Header & header, double now);

private:
  struct Last
  {
    std::uint32_t seq{0};
    double heard{0.0};
  };
  double reset_timeout_;
  std::map<std::pair<std::uint8_t, std::uint8_t>, Last> last_;  // (agent_id, 代碼)
};

// 轉送事件的去重複：同一則事件（種類, origin_id, origin_seq）只處理、轉送一次
class RelayDeduper
{
public:
  explicit RelayDeduper(double memory = 60.0)
  : memory_(memory) {}

  // 第一次看到回傳 true（應處理並轉送一次），之後回傳 false。自己發出的事件也要先登記。
  bool firstSeen(PacketType type, const RelayHeader & relay, double now);
  std::size_t size() const {return seen_.size();}

private:
  using Key = std::tuple<std::uint8_t, std::uint8_t, std::uint64_t>;
  void forget(double now);

  double memory_;
  std::map<Key, double> seen_;
  double last_cleanup_{0.0};
};

}  // namespace uav_cbba::wire
