// 角色 TaskSpec 的組合（任務欄位補齊第一批，2026-11-17）：閘道把識別、語意、空間（cbba_core/task_meta.hpp）寫進
// 規劃器給的 spec 最上層，再交給 CbbaComm 廣播（TASK_SPEC）。
//
// 規則
//   - 規劃器的 spec 是空的：照舊沒有 TaskSpec（舊型、單點任務；meta 只留在閘道，assignments、log 看得到）。
//     不自己造一份只有 meta 的 spec：狗的 BT 看到沒有 steps 的 spec 會回 BAD_SPEC
//   - 不是空的：要是 JSON 物件；reservedSpecKeys() 是閘道的，規劃器不能自己帶。例外：role、schema_version 和閘道的一樣
//     就照收（itest、舊的規劃器會寫 role），不一樣 → 拒絕
//   - 不重新序列化（yaml-cpp 輸出不是 JSON）：成員字串插在第一個 { 後面，規劃器的內容原封不動
#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "cbba_core/task_meta.hpp"
#include "cbba_core/types.hpp"

namespace swarm_cbba
{

// 成功回組好的 spec（空的 spec 回空字串）；失敗回 nullopt、error 寫原因
std::optional<std::string> composeSpec(const std::string & planner_spec, const cbba_core::TaskMeta & meta,
  const cbba_core::Vec3 & observe, std::uint8_t floor, std::string * error = nullptr);

// spec 最上層的 schema_version（沒有、解析不了回空字串）：機器人的 cbba_node 收到 TaskSpec 時看主版本
std::string specSchemaVersion(const std::string & spec);

}  // namespace swarm_cbba
