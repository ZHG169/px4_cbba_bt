// 區域註冊表（任務欄位補齊 05、08，協定版本 5，2026-11-18；docs/planner_task_definition_plan.md）
//
// 各節點啟動時讀同一份 site-config/<site>/zones.geojson（節點層轉成這裡的結構，核心不讀檔），出價前檢查：
//   WRONG_FLOOR     任務的樓層 ≠ 0、載具的樓層 ≠ 0（不知道就不擋）而且不同
//   ALTITUDE_LIMIT  任務高度（觀察點的 z）超過載具的 max_altitude（fleet.yaml 的 limits），或超過觀察點所在區域的 ceiling_m
//   ZONE_FORBIDDEN  觀察點所在的區域對這個平台 no_entry，或任務的 zone_policy_id 禁止這個平台進那個區域
//   INDOOR          觀察點在室內區域（indoor），載具是 UAV 而且沒有 indoor_flight 能力
// 只看觀察點（task.position），不看路徑：路徑可不可達交給 Nav2／UAV 的規劃，走不到回報失敗碼（使用者 11/17）。
// 目標本身可以在禁區裡：規劃器把觀察點放在禁區外，照樣可以得標。
// 區域的樓層 0 = 不分；任務的樓層 0 = 不分（每一層的區域都看）。
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "cbba_core/types.hpp"

namespace cbba_core
{

struct ZoneArea
{
  std::string id;
  std::uint8_t floor{0};              // zones.geojson 的 floor_id；0 = 不分
  std::uint8_t no_entry{0};           // 不能進的平台（位元 p = 平台代碼 p：1 uav、2 quadruped）
  double ceiling{0.0};                // 天花板（m）；0 = 沒有
  bool indoor{false};
  std::vector<std::array<double, 2>> polygon;   // map ENU，頭尾同一點

  bool contains(double x, double y) const;
};

struct ZonePolicy
{
  std::uint16_t id{0};                // 1～65535；0 保留給「只套用區域的 no_entry」
  std::string name;
  struct Rule
  {
    std::size_t zone{0};              // zones 的索引
    std::uint8_t platforms{0};        // 禁止的平台（位元同 no_entry）
  };
  std::vector<Rule> forbid;
};

struct ZoneRegistry
{
  std::uint16_t version{0};           // zones.geojson 的 version；內容改了就遞增。0 = 沒有版本
  std::vector<ZoneArea> zones;
  std::vector<ZonePolicy> policies;

  const ZonePolicy * policy(std::uint16_t id) const;
};

enum class ZoneVerdict : std::uint8_t { OK, WRONG_FLOOR, ALTITUDE_LIMIT, ZONE_FORBIDDEN, INDOOR };
const char * toString(ZoneVerdict v);

// zones 是 nullptr 時只檢查樓層、載具的高度上限。zone_id 有給時寫入擋下的區域
ZoneVerdict checkZones(const ZoneRegistry * zones, const AgentState & agent, std::uint8_t platform,
  const Task & task, std::string * zone_id = nullptr);

}  // namespace cbba_core
