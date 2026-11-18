#include "cbba_core/task_report.hpp"

#include <algorithm>
#include <cmath>

#include "cbba_core/wire_task_report.hpp"

namespace cbba_core
{

bool validTaskReport(const TaskReport & r)
{
  wire::TaskReportBody b;
  b.failure_code = r.failure_code;
  b.detail = r.detail;
  b.findings_uri = r.findings_uri;
  b.attachments = r.attachments;
  if (b.failure_code.size() > wire::kMaxReportCode || b.detail.size() > wire::kMaxReportDetail ||
    b.findings_uri.size() > wire::kMaxReportUri || b.attachments.size() > wire::kMaxReportAttachments)
  {
    return false;
  }
  for (const std::string & a : r.attachments) {
    if (a.size() > wire::kMaxReportUri) {
      return false;
    }
  }
  return validTaskId(r.task) && wire::taskReportSize(b) <= wire::kMaxReportPacket &&
         (!r.position_valid || (std::isfinite(r.final_position.x) && std::isfinite(r.final_position.y) &&
         std::isfinite(r.final_position.z)));
}

bool ReportOutbox::add(const TaskReport & r, bool gateway_online)
{
  Item item;
  item.seq = next_seq_++;
  item.report = r;
  item.buffered = !gateway_online;
  items_.push_back(std::move(item));
  if (config_.capacity > 0 && items_.size() > config_.capacity) {
    items_.pop_front();
    return true;
  }
  return false;
}

std::vector<ReportOutbox::Item> ReportOutbox::due(double now)
{
  if (tokens_ < 0.0) {
    tokens_ = config_.burst;
  } else {
    tokens_ = std::min(config_.burst, tokens_ + (now - last_refill_) * config_.rate);
  }
  last_refill_ = now;
  std::vector<Item> out;
  for (Item & item : items_) {
    if (tokens_ < 1.0) {
      break;
    }
    if (item.last_sent >= 0.0 && now - item.last_sent < config_.retry) {
      continue;
    }
    item.last_sent = now;
    ++item.tries;
    tokens_ -= 1.0;
    out.push_back(item);
  }
  return out;
}

bool ReportOutbox::ack(std::uint32_t seq)
{
  const auto it = std::find_if(items_.begin(), items_.end(), [seq](const Item & i) {return i.seq == seq;});
  if (it == items_.end()) {
    return false;
  }
  items_.erase(it);
  return true;
}

std::size_t ReportOutbox::bufferedCount() const
{
  return static_cast<std::size_t>(std::count_if(items_.begin(), items_.end(),
    [](const Item & i) {return i.buffered;}));
}

}  // namespace cbba_core
