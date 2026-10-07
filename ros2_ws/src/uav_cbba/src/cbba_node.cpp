// 無人機端的 CBBA 節點：每台無人機的機上電腦各跑一個
//
// 機間（mesh）：UDP multicast，封包依「機間通訊封包規格」2026-10-06（見 cbba_comm.hpp）
// 機內（DDS）：
//   PX4 → CBBA   /uavN/fmu/out/vehicle_local_position_v1   位置（NED、以出生點為原點 → map ENU）
//                /uavN/fmu/out/vehicle_status_v1           armed、offboard
//                /uavN/fmu/out/battery_status_v1           電量
//                /uavN/fmu/out/vehicle_land_detected       是否在空中（參與出價的條件）
//   BT → CBBA    /uavN/new_task     (swarm_interfaces/Task)        本機發現的新任務
//                /uavN/task_result  (swarm_interfaces/Task)        任務結束時一次：
//                                   status = DONE 完成；CANCELLED 失敗，交回競標池、自己不再接
//   CBBA → BT    /uavN/assigned_task (swarm_interfaces/Task)       目前要執行的任務
//                reliable、transient_local；換任務時發布。沒有任務時 task_id = 0、status = CANCELLED。
//                同一個任務要連續 assign_hold 秒都排在第一個才發布，避免協商中途來回換。
//
// 巡檢任務（AIR_RECON）的 z 是無人機巡檢的高度（map ENU）。BT 常給地面的座標（z = 0），
// 低於 recon_altitude 時改成 recon_altitude，否則每段距離都多算到地面的高度差，耗電也估錯。
// 要和執行者（task_executor 的 altitude）一致。
//
// 參與出價（participate）：airborne = 解鎖且離地（預設）、armed = 解鎖、always = 一律參與。
// use_px4:=false 時不訂閱 PX4，位置與電量用參數 initial_position、battery，一律參與（測試協商用）。
//
// seq 紀錄（seq_file，預設 ~/.cbba/seq_uavN）：存 seq 預約的上限，重開機後 seq 接著繼續編，
// 不會撞到開機前用過的值（見 cbba_comm.hpp）。先寫暫存檔、fsync 後再改名，斷電不會留下半個檔案。
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/task.hpp>

#include "uav_cbba/cbba_comm.hpp"
#include "uav_cbba/udp_link.hpp"

using px4_msgs::msg::BatteryStatus;
using px4_msgs::msg::VehicleLandDetected;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;
using TaskMsg = swarm_interfaces::msg::Task;

namespace
{
// seq 紀錄：檔案內容是十進位的上限。沒有檔案或內容不對時回傳 nullopt
std::optional<std::uint32_t> readSeqFile(const std::string & path)
{
  std::FILE * f = std::fopen(path.c_str(), "r");
  if (f == nullptr) {
    return std::nullopt;
  }
  unsigned long value = 0;
  const bool ok = std::fscanf(f, "%lu", &value) == 1 && value <= 0xFFFFFFFFul;
  std::fclose(f);
  return ok ? std::optional<std::uint32_t>(static_cast<std::uint32_t>(value)) : std::nullopt;
}

// 先寫暫存檔、fsync，再改名蓋過去（改名是原子的），最後 fsync 目錄讓改名本身落地
bool writeSeqFile(const std::string & path, std::uint32_t value)
{
  const std::string tmp = path + ".tmp";
  std::FILE * f = std::fopen(tmp.c_str(), "w");
  if (f == nullptr) {
    return false;
  }
  bool ok = std::fprintf(f, "%u\n", value) > 0 && std::fflush(f) == 0 && ::fsync(::fileno(f)) == 0;
  ok = std::fclose(f) == 0 && ok;
  if (!ok || std::rename(tmp.c_str(), path.c_str()) != 0) {
    return false;
  }
  const std::string dir = std::filesystem::path(path).parent_path().string();
  const int fd = ::open(dir.empty() ? "." : dir.c_str(), O_RDONLY);
  if (fd >= 0) {
    ::fsync(fd);
    ::close(fd);
  }
  return true;
}

std::string envOr(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr && *v != '\0' ? v : fallback;
}

double wallNow()
{
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

builtin_interfaces::msg::Time toRosTime(double t)
{
  builtin_interfaces::msg::Time out;
  const double sec = std::floor(t);
  out.sec = static_cast<std::int32_t>(sec);
  out.nanosec = static_cast<std::uint32_t>((t - sec) * 1e9);
  return out;
}

double fromRosTime(const builtin_interfaces::msg::Time & t)
{
  return t.sec + t.nanosec * 1e-9;
}

uav_cbba::Vec3 toVec(const std::vector<double> & v, const char * name)
{
  if (v.size() != 3) {
    throw std::invalid_argument(std::string(name) + " needs 3 values");
  }
  return {v[0], v[1], v[2]};
}

TaskMsg toMsg(const uav_cbba::Task & t)
{
  TaskMsg m;
  m.task_id = t.id;
  m.type = static_cast<std::uint8_t>(t.type);
  m.position.x = t.position.x;
  m.position.y = t.position.y;
  m.position.z = t.position.z;
  m.created = toRosTime(t.created);
  m.deadline_sec = static_cast<float>(t.deadline_sec);
  m.value = static_cast<float>(t.value);
  m.duration_sec = static_cast<float>(t.duration_sec);
  m.status = static_cast<std::uint8_t>(t.status);
  m.status_stamp = toRosTime(t.status_stamp);
  return m;
}
}  // namespace

class CbbaNode : public rclcpp::Node
{
public:
  CbbaNode()
  : Node("cbba_node")
  {
    const int agent_id = declare_parameter<int>("agent_id", std::stoi(envOr("UAV_ID", "1")));
    if (agent_id < 1 || agent_id > 8) {
      throw std::invalid_argument("agent_id must be 1~8 (AGENT_STATE neighbor bits hold 8 agents)");
    }
    const std::string ns = declare_parameter<std::string>("px4_ns", envOr("UAV_NS", ""));
    use_px4_ = declare_parameter<bool>("use_px4", true);
    participate_ = declare_parameter<std::string>("participate", "airborne");
    if (participate_ != "airborne" && participate_ != "armed" && participate_ != "always") {
      throw std::invalid_argument("participate must be airborne, armed or always");
    }
    if (!use_px4_) {
      participate_ = "always";
    }
    assign_hold_ = declare_parameter<double>("assign_hold", 0.6);
    recon_altitude_ = declare_parameter<double>("recon_altitude", 5.0);

    // 出生點（map ENU）：PX4 的 local position 以出生點為原點。預設和 px4_sitl.sh 一樣沿 y 排開
    const double spacing = std::stod(envOr("UAV_SPAWN_SPACING", "2.0"));
    spawn_ = toVec(declare_parameter<std::vector<double>>(
        "spawn_enu", {0.0, (agent_id - 1) * spacing, 0.0}), "spawn_enu");

    uav_cbba::AgentState s;
    s.id = static_cast<uav_cbba::AgentId>(agent_id);
    s.type = uav_cbba::AgentType::UAV;
    s.position = toVec(declare_parameter<std::vector<double>>(
        "initial_position", {spawn_.x, spawn_.y, spawn_.z}), "initial_position");
    s.home = s.position;
    s.battery = declare_parameter<double>("battery", 100.0);
    s.safety_reserve = declare_parameter<double>("safety_reserve", s.safety_reserve);
    s.energy_per_meter = declare_parameter<double>("energy_per_meter", s.energy_per_meter);
    s.hover_energy_per_sec = declare_parameter<double>("hover_energy_per_sec", s.hover_energy_per_sec);
    s.cruise_speed = declare_parameter<double>("cruise_speed", 5.0);

    uav_cbba::ScoringParams sp;
    sp.battery_weight = declare_parameter<double>("battery_weight", sp.battery_weight);
    sp.cost_ref = declare_parameter<double>("cost_ref", sp.cost_ref);
    sp.max_bundle = static_cast<std::size_t>(
      declare_parameter<int>("max_bundle", static_cast<int>(sp.max_bundle)));

    uav_cbba::CommConfig cc;
    cc.index_slots = declare_parameter<int>("index_slots", cc.index_slots);
    cc.lost_timeout = declare_parameter<double>("lost_timeout", cc.lost_timeout);

    uav_cbba::UdpConfig uc;
    uc.group = declare_parameter<std::string>("udp_group", uc.group);
    uc.port = static_cast<std::uint16_t>(declare_parameter<int>("udp_port", uc.port));
    uc.interface_ip = declare_parameter<std::string>("mesh_ip", envOr("MESH_IP", ""));

    state_ = s;
    comm_ = std::make_unique<uav_cbba::CbbaComm>(s, sp, cc);
    setupSeqStore(declare_parameter<std::string>(
        "seq_file", envOr("HOME", ".") + "/.cbba/seq_uav" + std::to_string(agent_id)));
    comm_->setParticipating(participate_ == "always", wallNow());
    udp_ = std::make_unique<uav_cbba::UdpLink>(uc);

    // ---------- BT ----------
    const std::string prefix = ns.empty() ? "" : "/" + ns;
    assigned_pub_ = create_publisher<TaskMsg>(
      prefix + "/assigned_task", rclcpp::QoS(1).reliable().transient_local());
    new_task_sub_ = create_subscription<TaskMsg>(
      prefix + "/new_task", rclcpp::QoS(20).reliable(),
      [this](TaskMsg::ConstSharedPtr msg) {onNewTask(*msg);});
    result_sub_ = create_subscription<TaskMsg>(
      prefix + "/task_result", rclcpp::QoS(20).reliable(),
      [this](TaskMsg::ConstSharedPtr msg) {onTaskResult(*msg);});

    // ---------- PX4 ----------
    if (use_px4_) {
      const std::string fmu = prefix + "/fmu/out";
      const auto qos = rclcpp::SensorDataQoS();   // PX4 的話題是 best effort
      position_sub_ = create_subscription<VehicleLocalPosition>(
        fmu + "/vehicle_local_position_v1", qos,
        [this](VehicleLocalPosition::ConstSharedPtr msg) {onPosition(*msg);});
      status_sub_ = create_subscription<VehicleStatus>(
        fmu + "/vehicle_status_v1", qos,
        [this](VehicleStatus::ConstSharedPtr msg) {
          armed_ = msg->arming_state == VehicleStatus::ARMING_STATE_ARMED;
          offboard_ = msg->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
        });
      land_sub_ = create_subscription<VehicleLandDetected>(
        fmu + "/vehicle_land_detected", qos,
        [this](VehicleLandDetected::ConstSharedPtr msg) {landed_ = msg->landed;});
      battery_sub_ = create_subscription<BatteryStatus>(
        fmu + "/battery_status_v1", qos,
        [this](BatteryStatus::ConstSharedPtr msg) {
          if (msg->connected && msg->remaining >= 0.0f) {
            state_.battery = 100.0 * msg->remaining;
          }
        });
    }

    publishAssigned(std::nullopt, wallNow());   // 一開始：沒有任務
    const int tick_ms = declare_parameter<int>("tick_ms", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(tick_ms), [this]() {step();});

    RCLCPP_INFO(get_logger(),
      "uav%d：UDP %s:%u（網卡 %s）；話題 %s/{new_task,task_result,assigned_task}；%s；參與出價：%s",
      agent_id, uc.group.c_str(), uc.port,
      uc.interface_ip.empty() ? "系統預設" : uc.interface_ip.c_str(), prefix.c_str(),
      use_px4_ ? "位置與電量來自 PX4" : "位置與電量來自參數", participate_.c_str());
  }

private:
  void setupSeqStore(const std::string & path)
  {
    if (path.empty()) {
      RCLCPP_WARN(get_logger(), "seq_file 是空的：seq 用時鐘起點，時鐘往回跳時重開機可能撞號");
      return;
    }
    std::error_code ec;
    std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
    const auto stored = readSeqFile(path);
    if (stored) {
      RCLCPP_INFO(get_logger(), "seq 紀錄 %s：從 %u 之後開始", path.c_str(), *stored);
    } else {
      RCLCPP_WARN(get_logger(), "沒有 seq 紀錄 %s（第一次開機或檔案不見）：這次用時鐘起點", path.c_str());
    }
    comm_->setSeqStore(stored, [this, path](std::uint32_t limit) {
        const bool ok = writeSeqFile(path, limit);
        if (!ok) {
          RCLCPP_ERROR(get_logger(), "seq 紀錄 %s 寫入失敗：重開機後可能撞號", path.c_str());
        }
        return ok;
      });
  }

  void onPosition(const VehicleLocalPosition & p)
  {
    if (!p.xy_valid || !p.z_valid) {
      return;
    }
    // NED（以出生點為原點）→ map ENU
    state_.position = {spawn_.x + p.y, spawn_.y + p.x, spawn_.z - p.z};
    if (!have_home_) {
      state_.home = state_.position;   // 第一次有效位置當返航點
      have_home_ = true;
      RCLCPP_INFO(get_logger(), "返航點 map (%.1f, %.1f, %.1f)",
        state_.home.x, state_.home.y, state_.home.z);
    }
    last_position_ = wallNow();
  }

  void onNewTask(const TaskMsg & msg)
  {
    const double now = wallNow();
    uav_cbba::Task t;
    t.id = msg.task_id;
    t.type = static_cast<uav_cbba::TaskType>(msg.type);
    t.position = {msg.position.x, msg.position.y, msg.position.z};
    if (t.type == uav_cbba::TaskType::AIR_RECON && t.position.z < recon_altitude_) {
      t.position.z = recon_altitude_;   // 地面座標 → 巡檢高度
    }
    t.created = msg.created.sec == 0 && msg.created.nanosec == 0 ? now : fromRosTime(msg.created);
    t.deadline_sec = msg.deadline_sec > 0.0f ? msg.deadline_sec : 300.0;
    t.value = msg.value;
    t.duration_sec = msg.duration_sec;
    t.status_stamp = t.created;
    if (comm_->addLocalTask(t, now)) {
      RCLCPP_INFO(get_logger(), "新任務 %s (%.1f, %.1f, %.1f) value %.0f，期限 %.0f s",
        uav_cbba::taskName(t.id).c_str(), t.position.x, t.position.y, t.position.z, t.value,
        t.deadline_sec);
    } else {
      RCLCPP_WARN(get_logger(),
        "新任務 %s 沒有加入：task_id 要是 %u × 65536 + 流水號、不能重複，或本機的任務編號已用完",
        uav_cbba::taskName(msg.task_id).c_str(), comm_->id());
    }
  }

  // status = DONE：完成；CANCELLED：失敗，交回競標池
  void onTaskResult(const TaskMsg & msg)
  {
    const double now = wallNow();
    if (msg.status != TaskMsg::DONE && msg.status != TaskMsg::CANCELLED) {
      RCLCPP_WARN(get_logger(), "任務 %s 的結果 status = %u，要是 DONE(1) 或 CANCELLED(2)",
        uav_cbba::taskName(msg.task_id).c_str(), msg.status);
      return;
    }
    const bool success = msg.status == TaskMsg::DONE;
    if (comm_->reportResult(msg.task_id, success, now)) {
      if (success) {
        reported_at_[msg.task_id] = now;
        RCLCPP_INFO(get_logger(), "任務 %s 完成，等待全隊確認", uav_cbba::taskName(msg.task_id).c_str());
      } else {
        RCLCPP_WARN(get_logger(), "任務 %s 失敗，交回競標池", uav_cbba::taskName(msg.task_id).c_str());
      }
    } else {
      RCLCPP_WARN(get_logger(), "任務 %s 的結果無法處理：不認得、已結束，或正在確認中",
        uav_cbba::taskName(msg.task_id).c_str());
    }
  }

  void step()
  {
    const double now = wallNow();

    bool participate = true;
    if (use_px4_) {
      comm_->setStatus(armed_, offboard_, now - last_position_ < 0.5);
      participate = participate_ == "always" || (participate_ == "armed" && armed_) ||
        (participate_ == "airborne" && armed_ && !landed_);
    } else {
      comm_->setStatus(false, false, true);
    }
    if (participate != comm_->participating()) {
      comm_->setParticipating(participate, now);
      RCLCPP_INFO(get_logger(), participate ? "開始參與出價" : "停止參與出價，釋放手上的任務");
    }
    comm_->setState(state_);

    for (const auto & bytes : udp_->receiveAll()) {
      comm_->receive(bytes, now);
    }
    for (const auto & bytes : comm_->poll(now)) {
      if (!udp_->send(bytes)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "UDP 送出失敗");
      }
    }

    updateAssigned(now);
    report(now);
  }

  // 目前排第一的任務連續 assign_hold 秒沒變，才交給 BT
  void updateAssigned(double now)
  {
    const auto current = comm_->currentTask();
    const uav_cbba::TaskId candidate = current ? current->id : 0;
    if (candidate != candidate_) {
      candidate_ = candidate;
      candidate_since_ = now;
    }
    if (candidate_ != published_ && now - candidate_since_ >= assign_hold_) {
      publishAssigned(current, now);
    }
  }

  void publishAssigned(const std::optional<uav_cbba::Task> & task, double now)
  {
    TaskMsg msg;
    if (task) {
      msg = toMsg(*task);
      RCLCPP_INFO(get_logger(), "指派給 BT：%s (%.1f, %.1f, %.1f)", uav_cbba::taskName(task->id).c_str(),
        task->position.x, task->position.y, task->position.z);
    } else {
      msg.status = TaskMsg::CANCELLED;   // task_id = 0：沒有任務
      msg.status_stamp = toRosTime(now);
      if (published_ != 0) {
        RCLCPP_INFO(get_logger(), "指派給 BT：無");
      }
    }
    assigned_pub_->publish(msg);
    published_ = task ? task->id : 0;
  }

  void report(double now)
  {
    const auto & path = comm_->agent().path();
    if (path != last_path_) {
      std::string s;
      for (auto id : path) {
        s += (s.empty() ? "" : " → ") + uav_cbba::taskName(id);
      }
      RCLCPP_INFO(get_logger(), "路徑：%s", s.empty() ? "（無）" : s.c_str());
      last_path_ = path;
    }
    // 任務結束（自己收齊確認、或收到別台的完成證明）時印一次
    for (const auto & [id, task] : comm_->agent().tasks()) {
      if (task.status == uav_cbba::TaskStatus::OPEN || !closed_.insert(id).second) {
        continue;
      }
      if (task.status != uav_cbba::TaskStatus::DONE) {
        RCLCPP_INFO(get_logger(), "任務 %s 已取消", uav_cbba::taskName(id).c_str());
      } else if (const auto it = reported_at_.find(id); it != reported_at_.end()) {
        RCLCPP_INFO(get_logger(), "任務 %s 已確認完成（回報後 %.2f s）",
          uav_cbba::taskName(id).c_str(), now - it->second);
      } else {
        RCLCPP_INFO(get_logger(), "任務 %s 已確認完成（收到完成證明）", uav_cbba::taskName(id).c_str());
      }
    }

    const bool converged = comm_->converged(now);
    if (converged && !last_converged_) {
      RCLCPP_INFO(get_logger(), "收斂：%zu 個任務，%zu 個鄰居", comm_->taskCount(),
        comm_->aliveNeighbors(now).size());
    }
    last_converged_ = converged;
    if (comm_->stats().index_conflicts != last_conflicts_) {
      last_conflicts_ = comm_->stats().index_conflicts;
      RCLCPP_ERROR(get_logger(), "任務編號衝突 %d 次：檢查各機的 agent_id 是否重複", last_conflicts_);
    }
  }

  bool use_px4_{true};
  std::string participate_{"airborne"};
  double assign_hold_{0.6};
  double recon_altitude_{5.0};
  uav_cbba::Vec3 spawn_{};
  uav_cbba::AgentState state_;
  std::unique_ptr<uav_cbba::CbbaComm> comm_;
  std::unique_ptr<uav_cbba::UdpLink> udp_;

  bool armed_{false};
  bool offboard_{false};
  bool landed_{true};
  bool have_home_{false};
  double last_position_{0.0};

  uav_cbba::TaskId candidate_{0};
  double candidate_since_{0.0};
  uav_cbba::TaskId published_{0};
  std::vector<uav_cbba::TaskId> last_path_;
  bool last_converged_{false};
  std::map<uav_cbba::TaskId, double> reported_at_;   // 自己回報完成的時刻
  std::set<uav_cbba::TaskId> closed_;                // 已經印過結束的任務
  int last_conflicts_{0};

  rclcpp::Publisher<TaskMsg>::SharedPtr assigned_pub_;
  rclcpp::Subscription<TaskMsg>::SharedPtr new_task_sub_;
  rclcpp::Subscription<TaskMsg>::SharedPtr result_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CbbaNode>());
  rclcpp::shutdown();
  return 0;
}
