// 通用的 CBBA 節點：每台載具（無人機、機器狗）的機上電腦各跑一個，程式不分載具
//
// 機間（mesh）：UDP multicast，協定版本 2（task_id 識別任務，見 cbba_comm.hpp、doc/protocol.md）
// 機內（DDS）：只吃一套標準介面，話題都在 /<ns> 底下：
//   → CBBA   <ns>/robot_state  (swarm_interfaces/RobotState)   位置（map ENU）、電量、飛行狀態
//            <ns>/task_result  (swarm_interfaces/TaskResult)   任務結束時一次：
//                              success = false 時交回競標池、自己不再接；detail 只記 log
//            <ns>/new_task     (swarm_interfaces/Task)         本機發現的新任務（task_id 建立者要是自己）
//            <ns>/cancel_task  (swarm_interfaces/Task)         取消任務（只看 task_id）：要是建立者或 cancel_authorities
//   CBBA →   <ns>/assigned_task (swarm_interfaces/Task)        目前要執行的任務（正式指派）：
//            同一任務連續 assign_hold 秒排第一才發，帶 assignment_version；沒有任務時 task_id = 0、status = CANCELLED
//
// 載具的差異由外面處理：
//   - 狗的 BT 直接送 RobotState、TaskResult（飛行狀態的欄位填 false）。
//   - 無人機由 px4_waypoint_node/px4_state_bridge 把 PX4 的話題轉成 RobotState（NED → map ENU、加出生點、
//     真實的 armed／offboard／landed），PX4 有在更新就一直送。
//   - 節點名稱用啟動時的 remap 區分（-r __node:=uav1_cbba_node、v60_cbba_node），程式裡固定叫 cbba_node。
//   - vehicle_type 決定 AGENT_STATE 的載具種類（能接哪些任務）、參與出價的條件，以及能量模型、巡檢高度的預設值。
//
// 參與出價（participate）：
//   state  = robot_state 在 state_timeout 內有更新（預設）。無人機另外要飛行狀態有效、已解鎖，
//            而且 require_airborne（預設 true）時要離地。轉接節點或狗的 BT 停掉時停止參與、立刻釋放任務
//   always = 一律參與，不訂閱 robot_state；位置與電量用參數 initial_position、battery（測試協商用）
//
// AGENT_STATE 的旗標用 robot_state 的真實值：armed、offboard、landed 只在 flight_state_valid 時送。
//
// 返航點（只用在出價時估返航的耗電，不會叫載具飛回去）：第一筆 robot_state 的位置。
//
// 正式指派（2026-10-09，流程在 assignment_manager.hpp）：
//   CBBA 得標只是候選，BT（地面端）保有最終的接受與中斷權。require_accept = true（狗）時：
//   <ns>/assignment_request（ACTIVATE、RESERVE、RELEASE）→ BT → <ns>/assignment_response（接受／拒絕＋原因）；
//   接受後才發 assigned_task。<ns>/exec_state（ExecState）是 BT 的執行狀態：能不能接新任務、能不能中斷、
//   預估剩餘時間（排隊用）。require_accept = false（無人機的 BT 還沒好）跳過握手，其他檢查照常。
//   task_result 的 task_id、assignment_version 必須是目前執行中的正式指派，不是就拒絕、任務狀態不變。
//   每個開始執行的指派印一行分段量測（建立 → 排第一 → 請求 → 接受 → 發布 → BT 回報執行）。
//
// 機號（agent_id）1～254：RobotState.agent_id 是 uint8。協定本身是 uint16（1～65534）。
//
// 程序鎖（2026-10-09）：啟動時對 <lock_dir>/agentN.lock 加 flock（不等待），拿不到就拒絕啟動，
// 從啟動持有到程序結束（程序死掉時核心自動釋放），不因任務完成、取消、切換而釋放。
// 只有 cbba_node 用這把鎖；BT、Nav2 等其他節點不碰。lock_dir 要在同一台機器的所有容器共用
// （docker 掛 /run/cbba），否則兩個容器各跑一個同機號的節點時鎖不到。
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <swarm_interfaces/msg/assignment_request.hpp>
#include <swarm_interfaces/msg/assignment_response.hpp>
#include <swarm_interfaces/msg/exec_state.hpp>
#include <swarm_interfaces/msg/robot_state.hpp>
#include <swarm_interfaces/msg/task.hpp>
#include <swarm_interfaces/msg/task_result.hpp>

#include "cbba_core/assignment_manager.hpp"
#include "cbba_core/cbba_comm.hpp"
#include "cbba_core/udp_link.hpp"

namespace cbba_core
{

namespace
{
using RobotStateMsg = swarm_interfaces::msg::RobotState;
using TaskMsg = swarm_interfaces::msg::Task;
using TaskResultMsg = swarm_interfaces::msg::TaskResult;
using ExecStateMsg = swarm_interfaces::msg::ExecState;
using RequestMsg = swarm_interfaces::msg::AssignmentRequest;
using ResponseMsg = swarm_interfaces::msg::AssignmentResponse;

// 判斷 BT 的執行狀態有沒有在更新：本機的單調時間（不受對時影響）
double steadyNow()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

std::string envOr(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr && *v != '\0' ? v : fallback;
}

// 系統時鐘（秒），各機對時
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

// 預設的鎖目錄：CBBA_LOCK_DIR → $XDG_RUNTIME_DIR/cbba → /tmp/cbba（都是重開機就清掉的地方）
std::string defaultLockDir()
{
  const std::string env = envOr("CBBA_LOCK_DIR", "");
  if (!env.empty()) {
    return env;
  }
  const std::string runtime = envOr("XDG_RUNTIME_DIR", "");
  return runtime.empty() ? "/tmp/cbba" : runtime + "/cbba";
}

TaskMsg toMsg(const Task & t)
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
    // ---------- 載具 ----------
    const int agent_id = declare_parameter<int>("agent_id", 0);
    if (agent_id < 1 || agent_id > 254) {
      throw std::invalid_argument("agent_id must be 1~254 (0 is reserved for \"nobody\")");
    }
    const std::string vehicle = declare_parameter<std::string>("vehicle_type", "uav");
    if (vehicle != "uav" && vehicle != "ugv") {
      throw std::invalid_argument("vehicle_type must be uav or ugv");
    }
    const bool uav = vehicle == "uav";
    agent_id_ = static_cast<AgentId>(agent_id);
    acquireLock(declare_parameter<std::string>("lock_dir", defaultLockDir()), agent_id);
    const std::string ns = declare_parameter<std::string>(
      "ns", uav ? "uav" + std::to_string(agent_id) : "v60");
    const std::string prefix = ns.empty() ? "" : "/" + ns;

    participate_ = declare_parameter<std::string>("participate", "state");
    if (participate_ != "state" && participate_ != "always") {
      throw std::invalid_argument("participate must be state or always");
    }
    state_timeout_ = declare_parameter<double>("state_timeout", 1.5);
    require_airborne_ = declare_parameter<bool>("require_airborne", true);
    // 巡檢任務的 z 是巡檢高度；0 = 不調整（狗不接巡檢任務）
    recon_altitude_ = declare_parameter<double>("recon_altitude", uav ? 5.0 : 0.0);
    is_uav_ = uav;

    AgentState s;
    s.id = agent_id_;
    s.type = uav ? AgentType::UAV : AgentType::UGV;
    if (uav) {
      s.cruise_speed = 5.0;   // PX4 多旋翼的 MPC_XY_CRUISE
    } else {
      // 狗的能量模型是佔位值（未校正）：步行約 1 m/s；續航、耗電要用狗的實測資料換掉（見 doc/ugv_tuning.md）
      s.safety_reserve = 20.0;
      s.energy_per_meter = 0.1;
      s.hover_energy_per_sec = 0.05;
      s.cruise_speed = 1.0;
    }
    s.position = toVec(declare_parameter<std::vector<double>>(
        "initial_position", {0.0, 0.0, 0.0}), "initial_position");
    s.home = s.position;
    s.battery = declare_parameter<double>("battery", 100.0);
    s.safety_reserve = declare_parameter<double>("safety_reserve", s.safety_reserve);
    s.energy_per_meter = declare_parameter<double>("energy_per_meter", s.energy_per_meter);
    s.hover_energy_per_sec = declare_parameter<double>("hover_energy_per_sec", s.hover_energy_per_sec);
    s.cruise_speed = declare_parameter<double>("cruise_speed", s.cruise_speed);

    // ---------- 出價、協定、UDP ----------
    ScoringParams scoring;
    scoring.battery_weight = declare_parameter<double>("battery_weight", scoring.battery_weight);
    scoring.cost_ref = declare_parameter<double>("cost_ref", scoring.cost_ref);
    scoring.max_bundle = static_cast<std::size_t>(
      declare_parameter<int>("max_bundle", static_cast<int>(scoring.max_bundle)));

    CommConfig comm;
    comm.lost_timeout = declare_parameter<double>("lost_timeout", comm.lost_timeout);
    comm.max_packet = static_cast<std::size_t>(
      declare_parameter<int>("max_packet", static_cast<int>(comm.max_packet)));
    // 除了建立者，可以取消任務的機號（授權的管理端）。全隊要一樣，收到 TASK_CLOSE 時也用它檢查
    for (const auto id : declare_parameter<std::vector<std::int64_t>>(
        "cancel_authorities", std::vector<std::int64_t>{}))
    {
      if (id < 1 || id > 65534) {
        throw std::invalid_argument("cancel_authorities must be agent ids 1~65534");
      }
      comm.cancel_authorities.push_back(static_cast<AgentId>(id));
    }

    UdpConfig udp;
    udp.group = declare_parameter<std::string>("udp_group", udp.group);
    udp.port = static_cast<std::uint16_t>(declare_parameter<int>("udp_port", udp.port));
    udp.interface_ip = declare_parameter<std::string>("mesh_ip", envOr("MESH_IP", ""));

    AssignmentConfig assign;
    // 狗的 BT 要接受才執行；無人機的 BT 還沒好，先跳過握手（BT 完成後改成 true）
    assign.require_accept = declare_parameter<bool>("require_accept", !uav);
    assign.assign_hold = declare_parameter<double>("assign_hold", assign.assign_hold);
    assign.accept_timeout = declare_parameter<double>("accept_timeout", assign.accept_timeout);
    assign.max_queued_fires = static_cast<std::size_t>(
      declare_parameter<int>("max_queued_fires", static_cast<int>(assign.max_queued_fires)));
    assign.max_queue_wait = declare_parameter<double>("max_queue_wait", assign.max_queue_wait);
    exec_timeout_ = declare_parameter<double>("exec_timeout", 1.5);
    const int tick_ms = declare_parameter<int>("tick_ms", 10);

    state_ = s;
    comm_ = std::make_unique<CbbaComm>(s, scoring, comm);   // 每次啟動新的 session
    comm_->setParticipating(false, wallNow());   // 第一個 tick 依 participate 決定
    udp_ = std::make_unique<UdpLink>(udp);
    assignments_ = std::make_unique<AssignmentManager>(*comm_, assign,
        [this](int level, const std::string & text) {
          if (level >= 2) {
            RCLCPP_ERROR(get_logger(), "%s", text.c_str());
          } else if (level == 1) {
            RCLCPP_WARN(get_logger(), "%s", text.c_str());
          } else {
            RCLCPP_INFO(get_logger(), "%s", text.c_str());
          }
        });

    // ---------- 機內話題 ----------
    assigned_pub_ = create_publisher<TaskMsg>(
      prefix + "/assigned_task", rclcpp::QoS(1).reliable().transient_local());
    request_pub_ = create_publisher<RequestMsg>(prefix + "/assignment_request", rclcpp::QoS(20).reliable());
    response_sub_ = create_subscription<ResponseMsg>(
      prefix + "/assignment_response", rclcpp::QoS(20).reliable(),
      [this](ResponseMsg::ConstSharedPtr msg) {
        assignments_->onResponse(msg->task_id, msg->assignment_version, msg->accepted,
          static_cast<RejectReason>(msg->reason), msg->detail, wallNow());
      });
    exec_sub_ = create_subscription<ExecStateMsg>(
      prefix + "/exec_state", rclcpp::QoS(10).best_effort(),
      [this](ExecStateMsg::ConstSharedPtr msg) {last_exec_ = *msg; exec_rx_ = steadyNow(); have_exec_ = true;});

    if (participate_ == "state") {
      // 用什麼 QoS 發都收得到：best effort 的訂閱可以接 reliable 和 best effort 的發布
      state_sub_ = create_subscription<RobotStateMsg>(
        prefix + "/robot_state", rclcpp::QoS(10).best_effort(),
        [this](RobotStateMsg::ConstSharedPtr msg) {onRobotState(*msg);});
    }
    // 任務結果、新任務只有一則，掉了就沒有了：用 reliable（發的一方也要用 reliable）
    result_sub_ = create_subscription<TaskResultMsg>(
      prefix + "/task_result", rclcpp::QoS(20).reliable(),
      [this](TaskResultMsg::ConstSharedPtr msg) {onTaskResult(*msg);});
    // new_task 可能一次來很多（整區巡檢點）：佇列要夠深，否則 callback 來不及處理時舊的會被蓋掉
    new_task_sub_ = create_subscription<TaskMsg>(
      prefix + "/new_task", rclcpp::QoS(256).reliable(),
      [this](TaskMsg::ConstSharedPtr msg) {onNewTask(*msg);});
    cancel_sub_ = create_subscription<TaskMsg>(
      prefix + "/cancel_task", rclcpp::QoS(20).reliable(),
      [this](TaskMsg::ConstSharedPtr msg) {onCancelTask(*msg);});

    timer_ = create_wall_timer(std::chrono::milliseconds(tick_ms), [this]() {step();});

    RCLCPP_INFO(get_logger(),
      "agent %d（%s）：UDP %s:%u（網卡 %s），session %016llX；話題 %s/{robot_state,exec_state,task_result,"
      "new_task,cancel_task,assigned_task,assignment_request,assignment_response}；BT 要接受：%s；參與出價：%s",
      agent_id, uav ? "UAV" : "UGV", udp.group.c_str(), udp.port,
      udp.interface_ip.empty() ? "系統預設" : udp.interface_ip.c_str(),
      static_cast<unsigned long long>(comm_->sessionId()), prefix.c_str(),
      assign.require_accept ? "是" : "否（跳過握手）",
      participate_ == "always" ? "一律（位置與電量來自參數）" :
      (uav ? (require_airborne_ ? "robot_state 有在更新、已解鎖且離地" : "robot_state 有在更新、已解鎖") :
      "robot_state 有在更新"));
  }

  ~CbbaNode() override
  {
    if (lock_fd_ >= 0) {
      ::close(lock_fd_);   // 釋放程序鎖（程序結束時核心也會釋放）
    }
  }

private:
  // 同一機號只能有一個 cbba_node。檔案裡寫 PID 方便查是誰拿著
  void acquireLock(const std::string & dir, int agent_id)
  {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    const std::string path = dir + "/agent" + std::to_string(agent_id) + ".lock";
    lock_fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0666);
    if (lock_fd_ < 0) {
      throw std::runtime_error("cannot open lock file " + path + ": " + std::strerror(errno));
    }
    if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) {
      char holder[32] = {0};
      const ssize_t n = ::pread(lock_fd_, holder, sizeof(holder) - 1, 0);
      ::close(lock_fd_);
      lock_fd_ = -1;
      std::string pid = n > 0 ? std::string(holder, static_cast<std::size_t>(n)) : std::string("?");
      pid.erase(pid.find_last_not_of("\n ") + 1);
      throw std::runtime_error("agent " + std::to_string(agent_id) +
              " already has a running cbba_node (lock " + path + ", pid " + pid + ")");
    }
    const std::string pid = std::to_string(::getpid()) + "\n";
    if (::ftruncate(lock_fd_, 0) != 0 || ::pwrite(lock_fd_, pid.data(), pid.size(), 0) < 0) {
      RCLCPP_WARN(get_logger(), "程序鎖 %s：寫入 PID 失敗（鎖照樣有效）", path.c_str());
    }
    RCLCPP_INFO(get_logger(), "程序鎖 %s", path.c_str());
  }

  // ===========================================================================
  // 機內輸入
  // ===========================================================================
  void onRobotState(const RobotStateMsg & msg)
  {
    if (msg.agent_id != agent_id_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "robot_state 的 agent_id 是 %u，不是 %u：略過", msg.agent_id, agent_id_);
      return;
    }
    state_.position = {msg.position.x, msg.position.y, msg.position.z};
    if (msg.battery >= 0.0f) {   // 負值 = 不知道，沿用上一筆
      state_.battery = msg.battery;
    }
    if (!have_state_) {
      state_.home = state_.position;   // 第一次收到的位置當返航點
      RCLCPP_INFO(get_logger(), "返航點 map (%.1f, %.1f, %.1f)，電量 %.0f%%",
        state_.home.x, state_.home.y, state_.home.z, state_.battery);
    }
    last_robot_state_ = msg;
    have_state_ = true;
    last_state_ = wallNow();
  }

  void onNewTask(const TaskMsg & msg)
  {
    const double now = wallNow();
    Task t;
    t.id = msg.task_id;
    t.type = static_cast<TaskType>(msg.type);
    t.position = {msg.position.x, msg.position.y, msg.position.z};
    // BT 常給地面的座標（z = 0）。低於巡檢高度時改成巡檢高度，否則每段距離都多算到地面的高度差，
    // 耗電也估錯。要和執行者（task_executor 的 altitude）一致。地面處置任務不改
    if (t.type == TaskType::AIR_RECON && t.position.z < recon_altitude_) {
      t.position.z = recon_altitude_;
    }
    t.created = msg.created.sec == 0 && msg.created.nanosec == 0 ? now : fromRosTime(msg.created);
    // 沒給期限：出價用預設 300 s；排隊的最大等待（max_queue_wait）從 created 算，不會每輪重新起算
    t.explicit_deadline = msg.deadline_sec > 0.0f;
    t.deadline_sec = t.explicit_deadline ? msg.deadline_sec : 300.0;
    t.value = msg.value;
    t.duration_sec = msg.duration_sec;
    t.status_stamp = t.created;
    if (comm_->addLocalTask(t, now)) {
      RCLCPP_INFO(get_logger(), "新任務 %s (%.1f, %.1f, %.1f) value %.0f，期限 %.0f s",
        taskName(t.id).c_str(), t.position.x, t.position.y, t.position.z, t.value, t.deadline_sec);
    } else {
      RCLCPP_WARN(get_logger(),
        "新任務 %s 沒有加入：task_id 要是 %u × 65536 + 流水號，而且不能和已知的任務重複",
        taskName(msg.task_id).c_str(), comm_->id());
    }
  }

  // 只接受目前執行中的正式指派（assigned_task 上的那一個、同一個版本）的回報
  void onTaskResult(const TaskResultMsg & msg)
  {
    const std::string name = taskName(msg.task_id);
    const double now = wallNow();
    const auto & active = assignments_->active();
    const ReportCheck check = assignments_->onResult(msg.task_id, msg.assignment_version, msg.success, now);
    if (check != ReportCheck::ACCEPTED) {
      RCLCPP_WARN(get_logger(), "任務 %s v%llu 的回報拒絕（%s）：目前執行中的是 %s，任務狀態不變",
        name.c_str(), static_cast<unsigned long long>(msg.assignment_version), toString(check),
        active ? (taskName(active->task) + " v" + std::to_string(active->version)).c_str() : "無");
      return;
    }
    if (msg.success) {
      reported_at_[msg.task_id] = now;
      RCLCPP_INFO(get_logger(), "任務 %s 完成，等待全隊確認", name.c_str());
    } else if (msg.detail.empty()) {
      RCLCPP_WARN(get_logger(), "任務 %s 失敗，交回競標池", name.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "任務 %s 失敗（%s），交回競標池", name.c_str(), msg.detail.c_str());
    }
  }

  void onCancelTask(const TaskMsg & msg)
  {
    const ReportCheck check = comm_->cancelTask(msg.task_id, wallNow());
    if (check == ReportCheck::ACCEPTED) {
      RCLCPP_INFO(get_logger(), "取消任務 %s，通知全隊", taskName(msg.task_id).c_str());
    } else {
      RCLCPP_WARN(get_logger(), "取消任務 %s 拒絕：%s", taskName(msg.task_id).c_str(), toString(check));
    }
  }

  // robot_state → 參與出價、AGENT_STATE 的旗標
  void updateStatus(double now)
  {
    bool participate = true;
    VehicleStatus status;
    if (participate_ == "state") {
      const bool fresh = have_state_ && now - last_state_ <= state_timeout_;
      const auto & m = last_robot_state_;
      status.telemetry_ok = fresh;
      status.flight_state_valid = fresh && m.flight_state_valid;
      status.armed = m.armed;
      status.offboard = m.offboard;
      status.landed = m.landed;
      participate = fresh;
      if (is_uav_) {
        participate = status.flight_state_valid && m.armed && (!require_airborne_ || !m.landed);
      }
    } else {
      status.telemetry_ok = true;
    }
    comm_->setStatus(status);
    if (participate != comm_->participating()) {
      comm_->setParticipating(participate, now);
      RCLCPP_INFO(get_logger(), participate ? "開始參與出價" : "停止參與出價，釋放手上的任務");
    }
  }

  // ===========================================================================
  // 每個 tick：收 UDP → 協定層 → 送 UDP
  // ===========================================================================
  void step()
  {
    const double now = wallNow();
    updateStatus(now);
    comm_->setState(state_);

    for (const auto & bytes : udp_->receiveAll()) {
      comm_->receive(bytes, now);
    }
    for (const auto & bytes : comm_->poll(now)) {
      if (!udp_->send(bytes)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "UDP 送出失敗");
      }
    }

    // BT 的執行狀態（逾時用本機單調時間判斷）→ 指派流程
    ExecInput exec;
    exec.fresh = have_exec_ && steadyNow() - exec_rx_ <= exec_timeout_;
    exec.execution_state = last_exec_.execution_state;
    exec.preemptible = last_exec_.preemptible;
    exec.remaining_time = last_exec_.estimated_remaining_time;
    exec.has_active = last_exec_.has_active;
    exec.active_task = last_exec_.active_task_id;
    exec.active_version = last_exec_.active_assignment_version;
    assignments_->setExec(exec, now);
    assignments_->step(now);
    publishOutputs(now);
    report(now);
  }

  // 指派流程要送給 BT 的：請求、assigned_task、量測
  void publishOutputs(double now)
  {
    for (const AssignmentRequest & r : assignments_->takeRequests()) {
      RequestMsg msg;
      msg.kind = static_cast<std::uint8_t>(r.kind);
      msg.preempt = r.preempt;
      msg.task = toMsg(r.task);
      msg.task.assignment_version = r.version;
      request_pub_->publish(msg);
    }
    if (assignments_->takeAssignedChanged()) {
      TaskMsg msg;
      const auto & active = assignments_->active();
      if (active && comm_->agent().tasks().count(active->task) > 0) {
        const Task & t = comm_->agent().tasks().at(active->task);
        msg = toMsg(t);
        msg.assignment_version = active->version;
        RCLCPP_INFO(get_logger(), "指派給 BT：%s v%llu (%.1f, %.1f, %.1f)", taskName(t.id).c_str(),
          static_cast<unsigned long long>(active->version), t.position.x, t.position.y, t.position.z);
      } else {
        msg.status = TaskMsg::CANCELLED;   // task_id = 0：沒有任務
        msg.status_stamp = toRosTime(now);
        RCLCPP_INFO(get_logger(), "指派給 BT：無");
      }
      assigned_pub_->publish(msg);
    }
    for (const AssignmentTiming & t : assignments_->takeTimings()) {
      const auto ms = [](double a, double b) {return a >= 0.0 && b >= 0.0 ? (b - a) * 1e3 : -1.0;};
      const double end = t.running >= 0.0 ? t.running : t.assigned;
      RCLCPP_INFO(get_logger(),
        "量測 %s v%llu%s：建立→排第一 %.0f ms、排第一→請求 %.0f ms（assign_hold）、請求→接受 %.0f ms、"
        "接受→發布 %.0f ms、發布→BT 執行 %.0f ms；建立→%s %.0f ms",
        taskName(t.task).c_str(), static_cast<unsigned long long>(t.version), t.preempt ? "（中斷）" : "",
        ms(t.created, t.head), ms(t.head, t.request), ms(t.request, t.accept), ms(t.accept, t.assigned),
        ms(t.assigned, t.running), t.running >= 0.0 ? "BT 執行" : "發布", ms(t.created, end));
    }
  }

  // log：路徑、任務結束、收斂、task_id 衝突
  void report(double now)
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
    // 任務結束（自己收齊確認、或收到別台的 TASK_CLOSE）時印一次
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
        RCLCPP_INFO(get_logger(), "任務 %s 已確認完成（收到 TASK_CLOSE）", taskName(id).c_str());
      }
    }

    const bool converged = comm_->converged(now);
    if (converged && !last_converged_) {
      RCLCPP_INFO(get_logger(), "收斂：%zu 個任務，%zu 個鄰居", comm_->taskCount(),
        comm_->aliveNeighbors(now).size());
    }
    last_converged_ = converged;
    // 鄰居的 AGENT_STATE 旗標有變時印一次（真實的 armed／offboard／landed）
    for (const auto & [agent, n] : comm_->neighbors()) {
      if (!n.has_status) {
        continue;
      }
      const std::uint8_t flags = wire::packFlags(n.flags);
      const auto last = neighbor_flags_.find(agent);
      if (last == neighbor_flags_.end() || last->second != flags) {
        neighbor_flags_[agent] = flags;
        RCLCPP_INFO(get_logger(), "鄰居 %u：參與 %d、遙測 %d、飛行狀態有效 %d、armed %d、offboard %d、landed %d",
          agent, n.flags.participating, n.flags.telemetry_ok, n.flags.flight_state_valid,
          n.flags.armed, n.flags.offboard, n.flags.landed);
      }
    }
    // 鎖著、但持有者失聯的任務：只標記待確認，不重新指派（避免原載具還在執行）
    const auto lost = comm_->lockedByLostAgents(now);
    if (lost != last_lost_) {
      last_lost_ = lost;
      for (const auto & [task, holder] : lost) {
        RCLCPP_WARN(get_logger(), "任務 %s 的持有者 agent %u 失聯：待確認，暫不重新指派",
          taskName(task).c_str(), holder);
      }
    }
    const auto & stats = comm_->stats();
    if (stats.task_conflicts != last_conflicts_) {
      last_conflicts_ = stats.task_conflicts;
      RCLCPP_ERROR(get_logger(),
        "task_id 衝突 %d 次：同一個 task_id 被宣布兩次，檢查各機的 agent_id 是否重複", last_conflicts_);
    }
    if (stats.rejected_closes != last_rejected_closes_) {
      last_rejected_closes_ = stats.rejected_closes;
      RCLCPP_WARN(get_logger(), "收到不合權限的 TASK_CLOSE 共 %d 次（舊版本的完成、沒有授權的取消），沒有套用",
        last_rejected_closes_);
    }
    if (stats.aborted_completions != last_aborted_) {
      last_aborted_ = stats.aborted_completions;
      RCLCPP_WARN(get_logger(), "完成宣告作廢 %d 次：別台知道更新的正式指派（這次是舊的回報）", last_aborted_);
    }
    if (stats.rejected_local_tasks != last_rejected_) {
      last_rejected_ = stats.rejected_local_tasks;
      RCLCPP_ERROR(get_logger(),
        "本機建立的任務有 %d 個和已知的 task_id 重複，沒有送出：BT 重開機後流水號要接續，不能重用",
        last_rejected_);
    }
  }

  int lock_fd_{-1};
  AgentId agent_id_{1};
  bool is_uav_{true};
  std::string participate_{"state"};
  double state_timeout_{1.5};
  bool require_airborne_{true};
  double recon_altitude_{5.0};
  double exec_timeout_{1.5};

  AgentState state_;
  std::unique_ptr<CbbaComm> comm_;
  std::unique_ptr<UdpLink> udp_;
  std::unique_ptr<AssignmentManager> assignments_;
  ExecStateMsg last_exec_;
  bool have_exec_{false};
  double exec_rx_{0.0};
  std::vector<std::pair<TaskId, AgentId>> last_lost_;
  bool have_state_{false};
  double last_state_{0.0};
  RobotStateMsg last_robot_state_;

  std::vector<TaskId> last_path_;
  bool last_converged_{false};
  std::map<TaskId, double> reported_at_;   // 自己回報完成的時刻
  std::set<TaskId> closed_;                // 已經印過結束的任務
  int last_conflicts_{0};
  int last_rejected_{0};
  int last_rejected_closes_{0};
  int last_aborted_{0};
  std::map<AgentId, std::uint8_t> neighbor_flags_;   // 上次印出的鄰居旗標

  rclcpp::Publisher<TaskMsg>::SharedPtr assigned_pub_;
  rclcpp::Publisher<RequestMsg>::SharedPtr request_pub_;
  rclcpp::Subscription<ResponseMsg>::SharedPtr response_sub_;
  rclcpp::Subscription<ExecStateMsg>::SharedPtr exec_sub_;
  rclcpp::Subscription<RobotStateMsg>::SharedPtr state_sub_;
  rclcpp::Subscription<TaskResultMsg>::SharedPtr result_sub_;
  rclcpp::Subscription<TaskMsg>::SharedPtr new_task_sub_;
  rclcpp::Subscription<TaskMsg>::SharedPtr cancel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cbba_core

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cbba_core::CbbaNode>());
  rclcpp::shutdown();
  return 0;
}
