#include "uav_cbba/cbba_node_base.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>

namespace uav_cbba
{

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
}  // namespace

// ===========================================================================
// 工具
// ===========================================================================
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

Vec3 toVec(const std::vector<double> & v, const char * name)
{
  if (v.size() != 3) {
    throw std::invalid_argument(std::string(name) + " needs 3 values");
  }
  return {v[0], v[1], v[2]};
}

swarm_interfaces::msg::Task toMsg(const Task & t)
{
  swarm_interfaces::msg::Task m;
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

// ===========================================================================
// 啟動
// ===========================================================================
CbbaNodeBase::CbbaNodeBase(const std::string & name)
: Node(name)
{
}

CbbaNodeBase::CommonParams CbbaNodeBase::declareCommon(AgentState & s, const std::string & seq_name)
{
  s.safety_reserve = declare_parameter<double>("safety_reserve", s.safety_reserve);
  s.energy_per_meter = declare_parameter<double>("energy_per_meter", s.energy_per_meter);
  s.hover_energy_per_sec = declare_parameter<double>("hover_energy_per_sec", s.hover_energy_per_sec);
  s.cruise_speed = declare_parameter<double>("cruise_speed", s.cruise_speed);

  CommonParams p;
  p.scoring.battery_weight = declare_parameter<double>("battery_weight", p.scoring.battery_weight);
  p.scoring.cost_ref = declare_parameter<double>("cost_ref", p.scoring.cost_ref);
  p.scoring.max_bundle = static_cast<std::size_t>(
    declare_parameter<int>("max_bundle", static_cast<int>(p.scoring.max_bundle)));

  p.comm.index_slots = declare_parameter<int>("index_slots", p.comm.index_slots);
  p.comm.lost_timeout = declare_parameter<double>("lost_timeout", p.comm.lost_timeout);

  p.udp.group = declare_parameter<std::string>("udp_group", p.udp.group);
  p.udp.port = static_cast<std::uint16_t>(declare_parameter<int>("udp_port", p.udp.port));
  p.udp.interface_ip = declare_parameter<std::string>("mesh_ip", envOr("MESH_IP", ""));

  p.assign_hold = declare_parameter<double>("assign_hold", p.assign_hold);
  p.tick_ms = declare_parameter<int>("tick_ms", p.tick_ms);
  p.seq_file = declare_parameter<std::string>(
    "seq_file", envOr("HOME", ".") + "/.cbba/" + seq_name);
  return p;
}

void CbbaNodeBase::start(const AgentState & s, const CommonParams & p, const std::string & prefix)
{
  state_ = s;
  assign_hold_ = p.assign_hold;
  comm_ = std::make_unique<CbbaComm>(s, p.scoring, p.comm);
  setupSeqStore(p.seq_file);
  comm_->setParticipating(false, wallNow());   // 子類別在 beforeStep() 決定
  udp_ = std::make_unique<UdpLink>(p.udp);

  assigned_pub_ = create_publisher<TaskMsg>(
    prefix + "/assigned_task", rclcpp::QoS(1).reliable().transient_local());
  publishAssigned(std::nullopt, wallNow());   // 一開始：沒有任務
  timer_ = create_wall_timer(std::chrono::milliseconds(p.tick_ms), [this]() {step();});
}

void CbbaNodeBase::setupSeqStore(const std::string & path)
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

// ===========================================================================
// 子類別用
// ===========================================================================
void CbbaNodeBase::setParticipating(bool participate, double now)
{
  if (participate != comm_->participating()) {
    comm_->setParticipating(participate, now);
    RCLCPP_INFO(get_logger(), participate ? "開始參與出價" : "停止參與出價，釋放手上的任務");
  }
}

void CbbaNodeBase::reportResult(
  TaskId task_id, bool success, const std::string & detail, double now)
{
  const std::string name = taskName(task_id);
  if (!comm_->reportResult(task_id, success, now)) {
    RCLCPP_WARN(get_logger(), "任務 %s 的結果無法處理：不認得、已結束，或正在確認中", name.c_str());
    return;
  }
  if (success) {
    reported_at_[task_id] = now;
    RCLCPP_INFO(get_logger(), "任務 %s 完成，等待全隊確認", name.c_str());
  } else if (detail.empty()) {
    RCLCPP_WARN(get_logger(), "任務 %s 失敗，交回競標池", name.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "任務 %s 失敗（%s），交回競標池", name.c_str(), detail.c_str());
  }
}

// ===========================================================================
// 每個 tick
// ===========================================================================
void CbbaNodeBase::step()
{
  const double now = wallNow();
  beforeStep(now);
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
void CbbaNodeBase::updateAssigned(double now)
{
  const auto current = comm_->currentTask();
  const TaskId candidate = current ? current->id : 0;
  if (candidate != candidate_) {
    candidate_ = candidate;
    candidate_since_ = now;
  }
  if (candidate_ != published_ && now - candidate_since_ >= assign_hold_) {
    publishAssigned(current, now);
  }
}

void CbbaNodeBase::publishAssigned(const std::optional<Task> & task, double now)
{
  TaskMsg msg;
  if (task) {
    msg = toMsg(*task);
    RCLCPP_INFO(get_logger(), "指派給 BT：%s (%.1f, %.1f, %.1f)", taskName(task->id).c_str(),
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

void CbbaNodeBase::report(double now)
{
  const auto & path = comm_->agent().path();
  if (path != last_path_) {
    std::string s;
    for (auto id : path) {
      s += (s.empty() ? "" : " → ") + taskName(id);
    }
    RCLCPP_INFO(get_logger(), "路徑：%s", s.empty() ? "（無）" : s.c_str());
    last_path_ = path;
  }
  // 任務結束（自己收齊確認、或收到別台的完成證明）時印一次
  for (const auto & [id, task] : comm_->agent().tasks()) {
    if (task.status == TaskStatus::OPEN || !closed_.insert(id).second) {
      continue;
    }
    if (task.status != TaskStatus::DONE) {
      RCLCPP_INFO(get_logger(), "任務 %s 已取消", taskName(id).c_str());
    } else if (const auto it = reported_at_.find(id); it != reported_at_.end()) {
      RCLCPP_INFO(get_logger(), "任務 %s 已確認完成（回報後 %.2f s）",
        taskName(id).c_str(), now - it->second);
    } else {
      RCLCPP_INFO(get_logger(), "任務 %s 已確認完成（收到完成證明）", taskName(id).c_str());
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

}  // namespace uav_cbba
