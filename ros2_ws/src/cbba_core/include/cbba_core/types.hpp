// 共用型別：載具、任務、相容表（對應介面規格 v0.1）
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace cbba_core
{

using AgentId = std::uint16_t;   // 協定版本 2 的機號是 uint16（1～0xFFFE）
using TaskId = std::uint32_t;

constexpr AgentId kNoAgent = 0;  // 得標者為「無人」
constexpr double kInf = std::numeric_limits<double>::infinity();

enum class AgentType : std::uint8_t { UAV = 1, UGV = 2 };
enum class TaskType : std::uint8_t { AIR_RECON = 1, GROUND_INTERVENTION = 2, PATROL = 3 };
enum class TaskStatus : std::uint8_t { OPEN = 0, DONE = 1, CANCELLED = 2 };

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline double distance(const Vec3 & a, const Vec3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

// 任務（時間一律用秒，節點層負責和 ROS 的 Time 互轉）
struct Task
{
  TaskId id{0};
  TaskType type{TaskType::AIR_RECON};
  Vec3 position{};            // map 座標系 (ENU)，公尺
  double created{0.0};        // 發現或建立的時間
  double deadline_sec{60.0};  // 期限，從 created 起算（軟限制）
  double value{80.0};         // 任務價值
  double duration_sec{0.0};   // 預估的現場執行時間
  TaskStatus status{TaskStatus::OPEN};
  double status_stamp{0.0};
  bool explicit_deadline{true};   // false：建立時沒給期限（deadline_sec 是出價用的預設值），排隊用最大等待時間
};

// 任務優先級（2026-10-09）：本機安全處置（BT 內部，不是 CBBA 任務）＞ 火警處置 ＞ 一般巡檢。
// 同級不互相搶占。之後要分火警嚴重度時再加欄位
constexpr int kPriorityPatrol = 1;
constexpr int kPriorityFire = 2;
inline int taskPriority(TaskType type)
{
  return type == TaskType::GROUND_INTERVENTION ? kPriorityFire : kPriorityPatrol;
}

// 任務 ID = 建立者編號 * 65536 + 流水號
inline TaskId makeTaskId(AgentId origin, std::uint16_t seq)
{
  return (static_cast<TaskId>(origin) << 16) | seq;
}

// 相容表：不相容的組合分數固定為 0
inline bool isCompatible(AgentType agent, TaskType task)
{
  switch (agent) {
    case AgentType::UAV:
      return task == TaskType::AIR_RECON;
    case AgentType::UGV:
      return task == TaskType::GROUND_INTERVENTION || task == TaskType::PATROL;
  }
  return false;
}

// 載具目前的狀態（出價時使用）
struct AgentState
{
  AgentId id{1};
  AgentType type{AgentType::UAV};
  Vec3 position{};                  // 目前位置
  Vec3 home{};                      // 返航點
  double battery{100.0};            // 目前電量
  double safety_reserve{20.0};      // 安全存量：做完並返航後至少要剩這麼多
  double energy_per_meter{0.5};     // 每公尺耗能
  double hover_energy_per_sec{0.2}; // 現場懸停每秒耗能
  double cruise_speed{0.0};         // 標稱速度 (m/s)；0 表示使用 ScoringParams 的預設值
};

}  // namespace cbba_core
