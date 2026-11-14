#include "cbba_core/detection.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "cbba_core/wire_detection.hpp"

namespace cbba_core
{

bool validDetection(const Detection & d)
{
  return d.detector.size() <= wire::kMaxDetectionName && d.source.size() <= wire::kMaxDetectionName &&
         d.label.size() <= wire::kMaxDetectionName && d.attrs.size() <= wire::kMaxDetectionAttrs;
}

// ===========================================================================
// 機器人端
// ===========================================================================
bool DetectionOutbox::sameTarget(const Detection & a, const Detection & b) const
{
  if (a.detector != b.detector || a.label != b.label) {
    return false;
  }
  if (a.track_id != 0 || b.track_id != 0) {
    return a.track_id == b.track_id;   // 有追蹤編號就只看編號（兩個人走在一起也不會被合併）
  }
  if (!(config_.merge_radius > 0.0) || !a.position_valid || !b.position_valid) {
    return false;
  }
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  const double dz = a.position.z - b.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz) <= config_.merge_radius;
}

DetectionOutbox::AddResult DetectionOutbox::add(const Detection & d, bool gateway_online)
{
  Item item;
  item.seq = next_seq_++;
  item.detection = d;
  item.buffered = !gateway_online;
  AddResult result = AddResult::ADDED;
  // 合併：拿掉同一個目標的舊的（暫存過的標記留著：閘道要知道這個目標在斷線期間就看到了）
  for (auto it = items_.begin(); it != items_.end(); ) {
    if (sameTarget(it->detection, d)) {
      item.buffered = item.buffered || it->buffered;
      it = items_.erase(it);
      result = AddResult::MERGED;
    } else {
      ++it;
    }
  }
  items_.push_back(std::move(item));
  if (config_.capacity > 0 && items_.size() > config_.capacity) {
    items_.pop_front();
    if (result == AddResult::ADDED) {
      result = AddResult::EVICTED;
    }
  }
  return result;
}

std::vector<DetectionOutbox::Item> DetectionOutbox::due(double now)
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

bool DetectionOutbox::ack(std::uint32_t seq)
{
  const auto it = std::find_if(items_.begin(), items_.end(), [seq](const Item & i) {return i.seq == seq;});
  if (it == items_.end()) {
    return false;
  }
  items_.erase(it);
  return true;
}

std::size_t DetectionOutbox::bufferedCount() const
{
  return static_cast<std::size_t>(std::count_if(items_.begin(), items_.end(),
    [](const Item & i) {return i.buffered;}));
}

// ===========================================================================
// 閘道端
// ===========================================================================
bool DetectionInbox::firstSeen(AgentId agent, std::uint64_t session, std::uint32_t seq, double now)
{
  if (now >= next_prune_) {
    next_prune_ = now + 10.0;
    for (auto it = seen_.begin(); it != seen_.end(); ) {
      it = now - it->second > keep_sec_ ? seen_.erase(it) : std::next(it);
    }
  }
  return seen_.emplace(std::make_tuple(agent, session, seq), now).second;
}

}  // namespace cbba_core
