// BT 節點和外界之間的介面（2026-10-09）：ROS 節點（uav_bt_node）實作它，測試用假的實作。
// BT 節點本身不碰 ROS、PX4，只透過這裡讀狀態、下指令，所以整棵樹可以不開模擬器測（test_tree）。
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "uav_px4_bt/cpf.hpp"

namespace uav_px4_bt
{

// 自己的狀態（map ENU）
struct VehicleState
{
  bool valid{false};          // 有 PX4 的位置、而且還在更新
  bool ready{false};          // PX4 起飛前檢查通過、位置有效（可以解鎖）
  Vec3 position;
  Vec3 velocity;
  double battery{-1.0};       // 0～100 %；負值 = 不知道
  bool armed{false};
  bool offboard{false};
  bool landed{true};
};

// cbba_node 的 assigned_task（task_id = 0 表示沒有任務）
struct AssignedTask
{
  std::uint32_t task_id{0};
  std::uint8_t type{0};
  Vec3 position;
  double duration{0.0};
  std::uint64_t version{0};

  bool operator==(const AssignedTask & o) const {return task_id == o.task_id && version == o.version;}
  bool operator!=(const AssignedTask & o) const {return !(*this == o);}
};

// apriltag_fire_detector 的結果（每個 tag 最新的一筆）
struct FireObservation
{
  std::uint32_t tag_id{0};
  double confidence{0.0};
  int frames{0};
  Vec3 position;
  double received{0.0};       // 收到的時間（platform 的 now）
};

// swarm_interfaces/ExecState 的值
enum class ExecMode : std::uint8_t { IDLE = 0, NAVIGATING = 1, EXECUTING = 2, NON_INTERRUPTIBLE = 3, STOPPING = 4, FAULT = 5 };

struct ExecStatus
{
  ExecMode mode{ExecMode::IDLE};
  bool preemptible{true};
  double remaining{-1.0};
  std::uint32_t active_task{0};    // 0 = 沒有
  std::uint64_t active_version{0};
};

constexpr std::uint8_t kTaskGroundIntervention = 2;

class UavPlatform
{
public:
  virtual ~UavPlatform() = default;

  virtual double timeNow() const = 0;                   // 秒，單調
  virtual VehicleState vehicle() const = 0;
  virtual std::vector<Neighbor> neighbors() const = 0;  // 位置廣播收到的鄰機
  virtual std::vector<FireObservation> fireObservations() const = 0;

  // 速度命令（map ENU）：每個 tick 由目前在跑的動作節點設定；節點層照 setpoint 頻率送給 PX4
  virtual void setVelocity(const Vec3 & velocity) = 0;
  virtual void requestOffboardAndArm() = 0;             // 呼叫的人負責節流
  virtual void requestReturnToLaunch() = 0;
  virtual void stopOffboard() = 0;                      // 不再送 offboard setpoint（交給 PX4 的自動模式）

  virtual void reportResult(const AssignedTask & task, bool success, const std::string & detail) = 0;
  // 建立任務（ReportFireEvent）：回傳 task_id
  virtual std::uint32_t createTask(std::uint8_t type, const Vec3 & position, double value,
    double duration, double deadline) = 0;
  virtual void setExecStatus(const ExecStatus & status) = 0;
  virtual void log(const std::string & text) = 0;
};

}  // namespace uav_px4_bt
