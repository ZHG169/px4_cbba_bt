// 機器人的偵測送閘道（2-1b M0 第 5 步，2026-11-14；doc/cbba_m0_contract.md「統一由閘道建立任務」）
//
// 機器人不自己建任務：偵測（2-1b 第七節的 Detection）送給閘道，由閘道的事件流程決定要不要注入任務。
// 機器人端（DetectionOutbox）：
//   - 每筆偵測一個流水號，留在佇列裡直到閘道確認（DETECTION_ACK）。閘道在線時送出、沒確認就每 retry 秒重送；
//     閘道斷線時不送、只暫存（標成「暫存過」），恢復後由舊到新送出。同一個機制處理弱網的掉包和斷線
//   - 合併：同一個偵測器、同一個類別，track_id 相同（都不是 0），或都沒有 track_id、位置都有效而且相距 ≤ merge_radius
//     → 新的取代佇列裡還沒確認的舊的（偵測器 10 Hz 送同一個目標時，佇列裡只留最新的一筆）
//   - 滿了（capacity）丟最舊的
//   - 限速：token bucket，每秒最多 rate 則（含重送），突發 burst 則
// 閘道端（DetectionInbox）：用 (機器人, session, 流水號) 去重複；重複的照樣回確認（上一次的確認可能掉了）
//
// 不依賴 ROS、socket（CbbaComm 呼叫）。
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "cbba_core/types.hpp"

namespace cbba_core
{

// 一筆偵測（2-1b 第七節；EVT-1 之後只加欄位）
struct Detection
{
  std::string detector;        // 偵測器（event_types.yaml 的名稱，例如 apriltag_fire）
  std::string source;          // 感測器（例如 uav1/down_cam）
  std::string label;           // 類別（fire、person…）
  double t{0.0};               // 偵測的時刻（系統時鐘，秒）
  double score{0.0};           // 0～1
  std::array<float, 4> bbox{};  // 影像上的 x、y、w、h（px）；全 0 = 沒有
  std::uint32_t track_id{0};   // 0 = 沒有
  bool position_valid{false};
  Vec3 position{};             // map ENU
  std::uint8_t floor{0};
  std::string attrs;           // 其他屬性（JSON），CBBA 不解析
};

// 字串長度符合封包的限制（detector、source、label ≤ 64 B，attrs ≤ 900 B）
bool validDetection(const Detection & d);

struct DetectionOutboxConfig
{
  std::size_t capacity{64};    // 佇列上限（還沒確認的）；滿了丟最舊的
  double merge_radius{2.0};    // 沒有 track_id 時，同一個偵測器、類別相距這麼近的合併（m）；0 = 不用位置合併
  double retry{1.0};           // 送出後這麼久沒確認就重送（s）
  double rate{5.0};            // 每秒最多送幾則（含重送）
  double burst{10.0};          // token bucket 的容量
};

class DetectionOutbox
{
public:
  enum class AddResult { ADDED, MERGED, EVICTED };   // EVICTED：加進來了，但丟掉了最舊的一筆

  struct Item
  {
    std::uint32_t seq{0};
    Detection detection;
    bool buffered{false};      // 閘道斷線時產生（或合併了斷線時產生的）
    double last_sent{-1.0};    // < 0 = 還沒送過
    int tries{0};
  };

  explicit DetectionOutbox(const DetectionOutboxConfig & config = DetectionOutboxConfig{})
  : config_(config) {}

  // gateway_online：產生時閘道在不在線（不在線的標成暫存過）
  AddResult add(const Detection & d, bool gateway_online);
  // 現在要送的（由舊到新；沒送過的、或送過 retry 秒還沒確認的），扣限速額度、記下送出時刻。閘道在線時才呼叫
  std::vector<Item> due(double now);
  // 閘道確認了：從佇列拿掉。不在佇列裡（已經確認過、被合併掉）回傳 false
  bool ack(std::uint32_t seq);

  std::size_t size() const {return items_.size();}
  std::size_t bufferedCount() const;
  const std::deque<Item> & items() const {return items_;}

private:
  bool sameTarget(const Detection & a, const Detection & b) const;

  DetectionOutboxConfig config_;
  std::deque<Item> items_;     // 依流水號（由舊到新）
  std::uint32_t next_seq_{1};
  double tokens_{-1.0};        // < 0 = 第一次呼叫 due 時補滿
  double last_refill_{0.0};
};

// 閘道收到的偵測
struct ReceivedDetection
{
  AgentId agent{kNoAgent};     // 偵測的機器人（閘道本機的偵測是閘道自己）
  std::uint64_t session{0};
  std::uint32_t seq{0};
  Detection detection;
  bool buffered{false};
  double received_at{0.0};
};

class DetectionInbox
{
public:
  explicit DetectionInbox(double keep_sec = 300.0)
  : keep_sec_(keep_sec) {}

  // 第一次收到回傳 true；keep_sec 之前收過的忘掉（機器人的重送不會隔這麼久）
  bool firstSeen(AgentId agent, std::uint64_t session, std::uint32_t seq, double now);

private:
  double keep_sec_;
  std::map<std::tuple<AgentId, std::uint64_t, std::uint32_t>, double> seen_;
  double next_prune_{0.0};
};

}  // namespace cbba_core
