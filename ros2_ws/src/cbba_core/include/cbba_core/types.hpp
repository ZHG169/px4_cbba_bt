// 共用型別：載具、任務（對應介面規格 v0.1）；能力過濾在 capabilities.hpp（2026-11-11 取代原本的相容表 isCompatible）
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace cbba_core
{

using AgentId = std::uint16_t;   // 協定版本 2 的機號是 uint16（1～0xFFFE）
// 協定版本 4（2026-11-11，doc/cbba_m0_contract.md 第六節）：64 位元 = 建立者 16 ｜ 世代 16 ｜ 流水號 32。
// 舊型 ID 相容：上半 32 位元是 0 的是舊型（建立者 × 65536 + 流水號，數值同版本 3），世代視為 0
using TaskId = std::uint64_t;
using Epoch = std::uint16_t;
// 能力標籤的位元遮罩（位元位置見 capabilities.hpp，2-1b M0 合約，2026-11-11）
using Capabilities = std::uint64_t;

constexpr AgentId kNoAgent = 0;  // 得標者為「無人」
constexpr double kInf = std::numeric_limits<double>::infinity();

enum class AgentType : std::uint8_t { UAV = 1, UGV = 2 };
// NONE：新型任務（2-1b，閘道建立）：平台、能力、優先級看 Task 的欄位。其他：舊型任務，由種類決定（capabilities.hpp），
// 協定版本 4 之前的任務全是舊型；舊型保留到 M3
enum class TaskType : std::uint8_t { NONE = 0, AIR_RECON = 1, GROUND_INTERVENTION = 2, PATROL = 3 };
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

// 任務定義裡 CBBA 目前只傳送、保存的欄位（協定版本 4，合約第二、三節；時刻都是絕對時間、秒，0 = 沒有）
struct TaskExt
{
  TaskId group_id{0};                 // 同一個事件拆出的角色；0 = 單獨任務
  std::uint8_t role_index{0};         // 第幾個角色（0 起）
  std::uint8_t role_count{0};         // 共幾個角色；0 = 不屬於任務組。最多 8（precedence 是 8 位元的遮罩）
  std::uint8_t floor{0};              // 樓層編號；0 = 不分
  double release{0.0};                // 可以開始的時刻
  bool deadline_hard{false};          // 期限是硬限制
  double expires{0.0};                // 到了自動失效
  std::uint16_t half_life_s{0};       // 價值衰減的半衰期；0 = 不衰減（同級分數暫時不用）
  std::uint8_t can_preempt{0};        // 可以搶占的優先級（位元 p = Pp）；0 = 預設（比自己低的級）
  std::uint8_t preempt_margin_pct{0}; // 同級搶占要好多少（%）；0 = 同級不搶
  double commit_radius{0.0};          // 執行者離目標這麼近之後不可搶占（m）；0 = 不用
  std::uint8_t min_battery_reserve{0};  // 做完還要剩的電量（%）；0 = 用載具的設定
  bool optional{false};               // 角色可省略
  bool all_or_nothing{false};         // 湊不齊就全部不做
  bool skip_satisfies{false};         // 這個 optional 角色被略過時，等它的角色視為條件已滿足
  std::uint8_t precedence{0};         // 要等哪幾個角色（位元 = role_index）
  std::uint16_t sync_window_s{0};     // 要同時到位的時間窗；0 = 不用
  std::uint16_t optional_no_capable_s{0};   // optional 關閉 O1 的等待；0 = 預設
  std::uint16_t optional_unassigned_s{0};   // optional 關閉 O2 的等待；0 = 預設
  std::uint32_t spec_length{0};       // 這個 revision 的 TaskSpec（壓縮後）大小；0 = 沒有 TaskSpec
  std::uint32_t spec_crc{0};
};

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
  // 本機估計的成功率 p_ij = 1 / (1 + 這台 TRANSIENT 失敗的次數)（2026-10-17）。只在本機（CbbaAgent::countFailure 設定），
  // 不上網路：封包格式不帶、收到的任務一律是 1
  double success_prob{1.0};
  // 2-1b（2026-11-11）：只有 type == NONE 的新型任務看這三個；舊型由 TaskType 決定（capabilities.hpp 的 effective*）
  std::uint8_t platform{0};           // 0 = 不限、1 = uav、2 = quadruped（和 AgentType 同值）
  Capabilities required_caps{0};      // 需要的能力（全部都要有）
  std::uint8_t priority{3};           // P0～P3，0 最高
  // 協定版本 4 的任務定義（合約第二節 A 層）。M0 只傳送、保存；revision 的套用、過期、搶占、協同在之後的步驟
  std::uint16_t revision{0};          // 同一個 task_id 的內容版本，只接受較大的
  TaskExt ext{};
};

// 任務優先級（2026-10-09；2026-11-11 改成 P0～P3）：本機安全處置（BT 內部，不是 CBBA 任務）＞ P0 ＞ P1 ＞ P2 ＞ P3。
// 同級不互相搶占。2-1：煙霧、入侵 P0；熱異常 P1；遺留物 P2；例行巡檢 P3。
// 舊型任務：火警處置（GROUND_INTERVENTION）P0，空中偵察、巡邏 P3（和原本「火警＞巡檢」相同）。
// taskPriority() 回傳「越大越優先」（3 − P），方便比較；kPriorityFire 是 P0（火警的排隊、失聯接手規則用）
constexpr int kPriorityPatrol = 0;   // P3
constexpr int kPriorityFire = 3;     // P0
inline std::uint8_t priorityLevel(const Task & task)
{
  switch (task.type) {
    case TaskType::NONE: return task.priority <= 3 ? task.priority : 3;
    case TaskType::GROUND_INTERVENTION: return 0;
    case TaskType::AIR_RECON:
    case TaskType::PATROL: return 3;
  }
  return 3;
}
inline int taskPriority(const Task & task)
{
  return 3 - static_cast<int>(priorityLevel(task));
}

// ===========================================================================
// task_id（協定版本 4）
// ===========================================================================
// 舊型 ID：建立者 × 65536 + 流水號（BT 自己算，保留到 M3）
inline TaskId makeLegacyTaskId(AgentId creator, std::uint16_t seq)
{
  return (static_cast<TaskId>(creator) << 16) | seq;
}
inline TaskId makeTaskId(AgentId creator, std::uint16_t seq) {return makeLegacyTaskId(creator, seq);}   // 舊型（同上）
// 版本 4 ID：只由 cbba_node／閘道產生。epoch = 0 時是舊型（seq 取低 16 位元）
inline TaskId makeTaskId(AgentId creator, Epoch epoch, std::uint32_t seq)
{
  if (epoch == 0) {
    return makeLegacyTaskId(creator, static_cast<std::uint16_t>(seq));
  }
  return (static_cast<TaskId>(creator) << 48) | (static_cast<TaskId>(epoch) << 32) | seq;
}
inline bool isLegacyTaskId(TaskId id) {return (id >> 32) == 0;}
inline AgentId taskCreator(TaskId id)
{
  return static_cast<AgentId>(isLegacyTaskId(id) ? (id >> 16) & 0xFFFFu : id >> 48);
}
inline Epoch taskEpoch(TaskId id)
{
  return isLegacyTaskId(id) ? 0 : static_cast<Epoch>((id >> 32) & 0xFFFFu);
}
inline std::uint32_t taskSeq(TaskId id)
{
  return static_cast<std::uint32_t>(isLegacyTaskId(id) ? id & 0xFFFFu : id & 0xFFFFFFFFu);
}
// 合法：建立者不是 0；版本 4 ID 的世代不是 0（上半 32 位元不是 0 而建立者是 0 的，例如世代在、建立者 0，也不合法）
inline bool validTaskId(TaskId id)
{
  if (isLegacyTaskId(id)) {
    return taskCreator(id) != 0;
  }
  return taskCreator(id) != 0 && taskEpoch(id) != 0;
}
// 同一個（建立者, 世代）的流水號上限：舊型 16 位元、版本 4 32 位元
inline std::uint64_t seqLimit(Epoch epoch) {return epoch == 0 ? 0x10000ull : 0x100000000ull;}
// log 用：舊型印十進位（和版本 3 的 log 一樣），版本 4 印 creator.epoch.seq
inline std::string formatTaskId(TaskId id)
{
  if (isLegacyTaskId(id)) {
    return std::to_string(id);
  }
  return std::to_string(taskCreator(id)) + "." + std::to_string(taskEpoch(id)) + "." + std::to_string(taskSeq(id));
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
  // 2-1b（2026-11-11）：宣告的能力（fleet.yaml）；0 = 沒設定，依 type 用預設（capabilities.hpp 的 defaultCaps）
  Capabilities capabilities{0};
  Capabilities failed_caps{0};      // 回報故障（E_HW）的能力：不能用
};

}  // namespace cbba_core
