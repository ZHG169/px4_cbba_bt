// 任務的執行結果送閘道（任務欄位補齊 11「回報」，協定版本 5，2026-11-18；docs/planner_task_definition_plan.md）
//
// BT 回報 task_result（成功、失敗都算）時，執行者的 cbba_node 把結果的摘要與參照（failure_code、做完幾步、最後位置、
// findings_uri、附件 URI）放進 ReportOutbox，閘道在線時送 TASK_REPORT、等 TASK_REPORT_ACK；沒確認每 retry 秒重送；
// 閘道斷線時只暫存（標成「暫存過」），恢復後由舊到新送出（結果 outbox）。和偵測的 outbox 一樣的機制，但不合併
//（每個結果都要送到）；滿了丟最舊的。閘道端用 (機器人, session, 流水號) 去重複（DetectionInbox）。
// 影像、截圖本身不上 mesh：findings、附件是 URI（控制中心的證據庫，任務欄位補齊 03）。不依賴 ROS、socket
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include "cbba_core/types.hpp"

namespace cbba_core
{

struct TaskReport
{
  TaskId task{0};
  std::uint16_t revision{0};
  std::uint64_t assign_version{0};
  double t{0.0};                       // 回報的時刻（系統時鐘，秒）
  bool success{false};
  std::uint16_t completed_steps{0};
  bool position_valid{false};
  Vec3 final_position{};
  std::string failure_code;            // E_*；成功時空
  std::string detail;                  // ≤ 200 B
  std::string findings_uri;
  std::vector<std::string> attachments;
};

// 長度符合封包的限制（failure_code ≤ 32、detail ≤ 200、URI ≤ 512、附件 ≤ 8 個、整個封包 ≤ 1200 B）
bool validTaskReport(const TaskReport & r);

struct ReportOutboxConfig
{
  std::size_t capacity{128};   // 佇列上限（還沒確認的）
  double retry{1.0};           // 送出後這麼久沒確認就重送（s）
  double rate{5.0};            // 每秒最多送幾則（含重送）
  double burst{10.0};
};

class ReportOutbox
{
public:
  struct Item
  {
    std::uint32_t seq{0};
    TaskReport report;
    bool buffered{false};
    double last_sent{-1.0};
    int tries{0};
  };

  explicit ReportOutbox(const ReportOutboxConfig & config = ReportOutboxConfig{})
  : config_(config) {}

  // 回傳 true 表示佇列滿了、丟掉最舊的一筆
  bool add(const TaskReport & r, bool gateway_online);
  std::vector<Item> due(double now);
  bool ack(std::uint32_t seq);

  std::size_t size() const {return items_.size();}
  std::size_t bufferedCount() const;
  const std::deque<Item> & items() const {return items_;}

private:
  ReportOutboxConfig config_;
  std::deque<Item> items_;
  std::uint32_t next_seq_{1};
  double tokens_{-1.0};
  double last_refill_{0.0};
};

// 閘道收到的結果
struct ReceivedReport
{
  AgentId agent{kNoAgent};
  std::uint64_t session{0};
  std::uint32_t seq{0};
  TaskReport report;
  bool buffered{false};
  double received_at{0.0};
};

}  // namespace cbba_core
