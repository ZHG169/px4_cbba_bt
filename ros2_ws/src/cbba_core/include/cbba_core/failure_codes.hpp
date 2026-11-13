// 2-1b 的 8 個失敗碼（E_*）→ 失敗後的排除分類（介面 v2.0，2026-11-14；合約 cbba_m0_contract.md 第三節的表）
//
// 執行端在 task_result 帶 failure_code（字串）時，cbba_node 用這張表決定 reason（排除多久），取代 BT 自己填的 reason；
// 沒帶 failure_code（舊的 BT）時照舊用 reason。失敗碼的介面由伯宇定（REG-2），對照由 CBBA 定，之後也寫進註冊表
// actions.yaml 的 failure_codes。
#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace cbba_core
{

// 值和 swarm_interfaces/TaskResult 的 reason 常數相同（節點層直接轉型）
enum class FailureClass : std::uint8_t { PERMANENT = 1, TRANSIENT = 2, PREEMPTED = 3, STOPPED = 4 };
const char * toString(FailureClass cls);

struct FailureCodeInfo
{
  FailureClass cls{FailureClass::TRANSIENT};
  bool notify_orphan{false};     // E_PRECONDITION：規劃錯誤，發 task_orphaned 通知閘道（升級值班）
  bool drop_capability{false};   // E_HW：拿掉 failed_capability（這台之後不再對需要它的任務出價）
};

// 不認得的失敗碼（含空字串）回傳 nullopt：節點層照舊用 reason，並印警告
std::optional<FailureCodeInfo> failureCodeInfo(const std::string & code);

}  // namespace cbba_core
