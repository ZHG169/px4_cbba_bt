#include "cbba_core/failure_codes.hpp"

#include <array>

namespace cbba_core
{

namespace
{
struct Entry
{
  const char * code;
  FailureCodeInfo info;
};

// 合約第三節的表：可預期再試也沒用的 → PERMANENT；條件可能變好的 → TRANSIENT（成功率打折已涵蓋「重試一次」）；
// 載具本身不能繼續的 → STOPPED（failure_exclude_sec 後恢復）
constexpr std::array<Entry, 8> kCodes{{
  {"E_PRECONDITION", {FailureClass::PERMANENT, true, false}},
  {"E_UNREACHABLE", {FailureClass::PERMANENT, false, false}},
  {"E_TIMEOUT", {FailureClass::TRANSIENT, false, false}},
  {"E_TARGET_LOST", {FailureClass::TRANSIENT, false, false}},
  {"E_LOW_BATTERY", {FailureClass::STOPPED, false, false}},
  {"E_SAFETY_STOP", {FailureClass::STOPPED, false, false}},
  {"E_COMM", {FailureClass::TRANSIENT, false, false}},
  {"E_HW", {FailureClass::PERMANENT, false, true}},
}};
}  // namespace

const char * toString(FailureClass cls)
{
  switch (cls) {
    case FailureClass::PERMANENT: return "PERMANENT";
    case FailureClass::TRANSIENT: return "TRANSIENT";
    case FailureClass::PREEMPTED: return "PREEMPTED";
    case FailureClass::STOPPED: return "STOPPED";
  }
  return "?";
}

std::optional<FailureCodeInfo> failureCodeInfo(const std::string & code)
{
  for (const auto & e : kCodes) {
    if (code == e.code) {
      return e.info;
    }
  }
  return std::nullopt;
}

}  // namespace cbba_core
