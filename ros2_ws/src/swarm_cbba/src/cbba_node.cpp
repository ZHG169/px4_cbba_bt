// 統一版 CBBA 節點（swarm_cbba）：無人機和機器狗跑同一支，agent_type 與能量模型由 params yaml 決定
//
// 2026-10-12 起照機器狗端（伯宇，spec v1.1 §2）的套件與介面：launch 給 namespace、話題用相對名稱、
// params yaml（config/cbba_uav.yaml、cbba_v60.yaml），BT 端不用改。核心與機間協定用 cbba_core（協定版本 2）。
//
// 機間（mesh）：UDP multicast，協定版本 4（2026-11-11；task_id 64 位元，見 cbba_core/doc/protocol.md、cbba_m0_contract.md）。
// 機內話題的 task_id 也是 uint64（swarm_interfaces，2026-11-11）
// 機內（DDS），話題在 /<namespace> 底下：
//   BT → CBBA   robot_state  (swarm_interfaces/RobotState)   位置（map ENU）、電量、mode；約 2 Hz
//               task_result  (swarm_interfaces/TaskResult)   任務結束時一次；success = false 交回競標池，
//                                                            reason 決定這台多久不再對它出價（PERMANENT 永久、PREEMPTED
//                                                            preempt_exclude_sec、其他 failure_exclude_sec）；TRANSIENT 另外累計
//                                                            失敗次數，之後對它的出價打折（成功率 1/(1+f)，2026-10-17）
//               new_task     (swarm_interfaces/Task)         本機發現的新任務（task_id 建立者要是自己）
//               cancel_task  (swarm_interfaces/Task)         取消任務（只看 task_id）：要是建立者或 cancel_authorities
//   CBBA → BT   current_task (swarm_interfaces/Task)         目前要執行的任務；task_id = 0 表示沒有任務
//               task_closed  (swarm_interfaces/Task)         任務結束（status DONE／CANCELLED），每個任務一次（2026-10-13，
//                                                            我們加的、狗端沒有）：無人機發布火警後懸停監控，等地面處置完成才走
//               task_announced (swarm_interfaces/Task)       全隊已知的任務，每個任務一次（2026-10-14，預設只有 agent_type = UAV
//                                                            才發，狗端沒有這個話題；參數 publish_announced 可以改，2026-11-09 給
//                                                            整合測試量狗的送達）：無人機的火點去重複要知道別台已經報過的火
//               task_orphaned (swarm_interfaces/Task)        孤兒任務（2026-10-17，我們加的）：任務還開著，但 orphan_warn_sec 內
//                                                            一直沒有人得標、也沒有正式指派。只通知（給地面站），不處理
//   CBBA → 測試、地面站
//               cbba_path    (std_msgs/UInt64MultiArray)     這台目前的路徑（bundle，依執行順序的 task_id），變動時發一次（2026-11-11 起 64 位元）
//                                                            （2026-10-27，我們加的）：current_task 只有路徑第一個，弱網測試要比對
//                                                            「每台分到幾個任務」才看得到後面的；BT 不用訂
//               cbba_tx      (std_msgs/UInt64MultiArray)     送出位元組的累計 [全部, 重傳, 轉送]，每 tx_stats_sec（0.2 s）發一次
//                                                            （2026-11-09，我們加的）：第 5 週的頻寬圖（每台 TX、重傳 TX）；BT 不用訂
//
// 參與出價（spec §2.3 規則 2、4）：robot_state 在 state_timeout 內有更新，而且 mode == AUTO。
//   MANUAL（平板接管）、ESTOP（急停）、沒有 robot_state 時對所有任務出價 0，手上的任務立刻釋放。
//   無人機的 robot_state 由 px4_state_bridge 發；mode 由正式 BT 決定（/uavN/bt_mode：起飛完成、BT 在控制 = AUTO，
//   被接管、PX4 失效保護、返航 = MANUAL，2026-10-17），備用 BT 時 bridge 自己判斷（在空中、起飛完成、offboard）。
//   participate = always（測試協商用）：一律參與，不訂閱 robot_state，位置與電量用參數 initial_position、battery。
//
// 正式指派（流程在 cbba_core/assignment_manager.hpp）：同一任務連續 assign_hold 秒排第一就發 current_task，
// 不和 BT 握手（BT 的哨兵負責中斷舊的）。指派版本只在 CBBA 節點之間用（機間封包）：
// task_result 的 task_id 必須是目前 current_task 上的任務，版本由節點自己補上；不是就拒絕、任務狀態不變。
//
// 和 2026-10-09 版本（cbba_core 的 cbba_node）不同的地方：話題 assigned_task → current_task；
// RobotState 的飛行狀態欄位改成 mode；TaskResult 的 assignment_version 改成 reason；
// 拿掉 exec_state、assignment_request／response（握手）；assign_hold 0.6 → 0.4 s；vehicle_type → agent_type。
//
// 機號（agent_id）1～254：RobotState.agent_id 是 uint8。協定本身是 uint16（1～65534）。
// 返航點（只用在出價時估返航的耗電）：第一筆 robot_state 的位置。
//
// 程序鎖：啟動時對 <lock_dir>/agentN.lock 加 flock（不等待），拿不到就拒絕啟動，從啟動持有到程序結束。
// lock_dir 要在同一台機器的所有容器共用（docker 掛 /run/cbba，CBBA_LOCK_DIR），否則兩個容器各跑一個同機號的節點時鎖不到。
#include <execinfo.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <swarm_interfaces/msg/robot_state.hpp>
#include <swarm_interfaces/msg/task.hpp>
#include <swarm_interfaces/msg/task_result.hpp>

#include "cbba_core/assignment_manager.hpp"
#include "cbba_core/capabilities.hpp"
#include "cbba_core/cbba_comm.hpp"
#include "cbba_core/udp_link.hpp"

namespace cbba_core
{

namespace
{
using RobotStateMsg = swarm_interfaces::msg::RobotState;
using TaskMsg = swarm_interfaces::msg::Task;
using TaskResultMsg = swarm_interfaces::msg::TaskResult;

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
  m.revision = t.revision;
  return m;
}

const char * reasonName(std::uint8_t r)
{
  switch (r) {
    case TaskResultMsg::NONE: return "NONE";
    case TaskResultMsg::PERMANENT: return "PERMANENT";
    case TaskResultMsg::TRANSIENT: return "TRANSIENT";
    case TaskResultMsg::PREEMPTED: return "PREEMPTED";
    case TaskResultMsg::STOPPED: return "STOPPED";
  }
  return "?";
}

const char * modeName(std::uint8_t m)
{
  switch (m) {
    case RobotStateMsg::AUTO: return "AUTO";
    case RobotStateMsg::MANUAL: return "MANUAL";
    case RobotStateMsg::ESTOP: return "ESTOP";
  }
  return "?";
}
}  // namespace

class CbbaNode : public rclcpp::Node
{
public:
  CbbaNode()
  : Node("cbba_node")
  {
    // ---------- 載具 ----------
    const int agent_id = declare_parameter<int>("agent_id", std::stoi(envOr("AGENT_ID", "50")));
    if (agent_id < 1 || agent_id > 254) {
      throw std::invalid_argument("agent_id must be 1~254 (0 is reserved for \"nobody\")");
    }
    const std::string agent_type = declare_parameter<std::string>("agent_type", "UGV");
    if (agent_type != "UAV" && agent_type != "UGV") {
      throw std::invalid_argument("agent_type must be UAV or UGV");
    }
    const bool uav = agent_type == "UAV";
    agent_id_ = static_cast<AgentId>(agent_id);
    const std::string lock_dir = declare_parameter<std::string>("lock_dir", defaultLockDir());
    acquireLock(lock_dir, agent_id);
    // 世代（協定版本 4，2026-11-11）：存在 <state_dir>/epoch_<機號>，每次啟動 + 1 再寫回。程序鎖保證同一台機器只有一個人在改。
    // state_dir 預設 CBBA_STATE_DIR，沒設時和程序鎖同一個目錄（docker：主機的 /tmp/cbba_locks，主機重開機會清掉，
    // 模擬時全隊一起重開所以沒關係）。實機的閘道要設成不會被清掉的目錄，並設 epoch_required = true
    const std::string state_dir = declare_parameter<std::string>("state_dir", envOr("CBBA_STATE_DIR", lock_dir));
    const bool epoch_required = declare_parameter<bool>("epoch_required", false);
    epoch_ = loadEpoch(state_dir, agent_id, epoch_required);
    // 空字串＝相對名稱，由 launch 的 namespace 決定（/v60/current_task …）；給了就是 /<ns>/…
    const std::string ns = declare_parameter<std::string>("ns", "");
    const auto topic = [&ns](const char * name) {
        return ns.empty() ? std::string(name) : "/" + ns + "/" + name;
      };

    participate_ = declare_parameter<std::string>("participate", "state");
    if (participate_ != "state" && participate_ != "always") {
      throw std::invalid_argument("participate must be state or always");
    }
    state_timeout_ = declare_parameter<double>("state_timeout", 1.5);
    // AIR_RECON 的 z 低於巡檢高度時抬到巡檢高度（地面處置任務不改）
    recon_altitude_ = declare_parameter<double>("recon_altitude", 5.0);
    // 失敗（reason 不是 PERMANENT）後多久不再對同一個任務出價（spec §2.3 規則 3）
    failure_exclude_sec_ = declare_parameter<double>("failure_exclude_sec", 60.0);
    // 孤兒任務：開著但這麼久沒有人得標就警告（0 = 不檢查）。全隊能做的都被永久排除（PERMANENT）、不在 AUTO 或失聯時，
    // CBBA 不會有任何訊號，期限又是軟限制，地面站不會知道（2026-10-17，宜臻的實作說明）
    orphan_warn_sec_ = declare_parameter<double>("orphan_warn_sec", 30.0);
    // task_announced 要不要發（2026-11-09）：預設只有無人機發（火點去重複），狗的 cbba_node 照舊沒有這個話題。
    // 整合測試（itest comm／loss --dogs）給狗 true，才量得到「狗知道幾個任務」（送達）；真的狗不用設
    const bool publish_announced = declare_parameter<bool>("publish_announced", uav);
    // PREEMPTED（被接管 PREEMPTED_BY_MANUAL；無人機的備用 BT 路過發現火警 FIRE_FOUND）的排除時間，預設和
    // failure_exclude_sec 一樣。cbba_v60.yaml、cbba_uav.yaml 設 5 s（2026-10-17）：只擋「回報先到、mode = MANUAL 晚到」的
    // 空檔；接管期間靠 MANUAL 不出價，做完回到 AUTO 就能接回（任務還沒人接時）
    preempt_exclude_sec_ = declare_parameter<double>("preempt_exclude_sec", failure_exclude_sec_);

    AgentState s;
    s.id = agent_id_;
    s.type = uav ? AgentType::UAV : AgentType::UGV;
    if (uav) {
      s.cruise_speed = 5.0;   // PX4 多旋翼的 MPC_XY_CRUISE
    } else {
      // 狗的能量模型是佔位值（未校正）：步行約 1 m/s；續航、耗電要用狗的實測資料換掉（見 cbba_core/doc/ugv_tuning.md）
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
    // 能力標籤（2-1b M0 合約第一節，2026-11-11）：之後從 fleet.yaml 讀；空 = 依 agent_type 的預設
    // （UAV：mobility_air、rgb_camera；UGV：mobility_ground、rgb_camera），和原本的相容表結果相同
    const auto cap_names = declare_parameter<std::vector<std::string>>(
        "capabilities", std::vector<std::string>{});
    std::string bad_cap;
    const auto caps = parseCapabilities(cap_names, &bad_cap);
    if (!caps) {
      throw std::invalid_argument("capabilities: unknown capability \"" + bad_cap + "\"");
    }
    s.capabilities = *caps;

    // ---------- 出價、協定、UDP ----------
    ScoringParams scoring;
    scoring.battery_weight = declare_parameter<double>("battery_weight", scoring.battery_weight);
    scoring.cost_ref = declare_parameter<double>("cost_ref", scoring.cost_ref);
    scoring.max_bundle = static_cast<std::size_t>(
      declare_parameter<int>("max_bundle", static_cast<int>(scoring.max_bundle)));
    // 同一個任務 TRANSIENT 失敗到這個次數就永久不出價；0 = 不啟用（只靠成功率打折）
    scoring.max_failures = declare_parameter<int>("max_failures", scoring.max_failures);

    CommConfig comm;
    comm.lost_timeout = declare_parameter<double>("lost_timeout", comm.lost_timeout);
    // 失聯接手：持有者失聯後多久撤銷它已接受的任務、重新競標（火警寧可重複、巡檢寧可等）。全隊要一樣
    comm.takeover_fire = declare_parameter<double>("takeover_fire", comm.takeover_fire);
    comm.takeover_patrol = declare_parameter<double>("takeover_patrol", comm.takeover_patrol);
    comm.max_packet = static_cast<std::size_t>(
      declare_parameter<int>("max_packet", static_cast<int>(comm.max_packet)));
    // 補發（協定版本 3，cbba_core/doc/protocol_v3_sync.md）：摘要比對 → 缺口請求 → 限速的分批回覆
    comm.repair_rate = declare_parameter<double>("repair_rate", comm.repair_rate);
    comm.push_recent_sec = declare_parameter<double>("push_recent_sec", comm.push_recent_sec);
    comm.gap_request_gap = declare_parameter<double>("gap_request_gap", comm.gap_request_gap);
    comm.digest_period = declare_parameter<double>("digest_period", comm.digest_period);
    comm.resend_gap = declare_parameter<double>("resend_gap", comm.resend_gap);
    // 完成確認：沒收齊確認時的重送間隔、次數（2026-11-04 開成參數，第 5 週量弱網的完成確認時間用；全隊應該一樣）
    comm.completion_retry = declare_parameter<double>("completion_retry", comm.completion_retry);
    comm.completion_max = declare_parameter<int>("completion_max", comm.completion_max);
    stats_log_sec_ = declare_parameter<double>("stats_log_sec", stats_log_sec_);
    // 除了建立者，可以取消任務的機號（授權的管理端）。全隊要一樣，收到 TASK_CLOSE 時也用它檢查
    for (const auto id : declare_parameter<std::vector<std::int64_t>>(
        "cancel_authorities", std::vector<std::int64_t>{}))
    {
      if (id < 1 || id > 65534) {
        throw std::invalid_argument("cancel_authorities must be agent ids 1~65534");
      }
      comm.cancel_authorities.push_back(static_cast<AgentId>(id));
    }
    comm.epoch = epoch_;

    UdpConfig udp;
    udp.group = declare_parameter<std::string>("udp_group", udp.group);
    udp.port = static_cast<std::uint16_t>(declare_parameter<int>("udp_port", udp.port));
    udp.interface_ip = declare_parameter<std::string>("mesh_ip", envOr("MESH_IP", ""));

    AssignmentConfig assign;
    assign.require_accept = false;   // 不握手：直接發 current_task，BT 的哨兵負責中斷（2026-10-12）
    assign.assign_hold = declare_parameter<double>("assign_hold", 0.4);   // spec §2.3 規則 1：2 個廣播週期
    assign.max_queued_fires = static_cast<std::size_t>(
      declare_parameter<int>("max_queued_fires", static_cast<int>(assign.max_queued_fires)));
    assign.max_queue_wait = declare_parameter<double>("max_queue_wait", assign.max_queue_wait);
    assign.overrun_margin = declare_parameter<double>("overrun_margin", assign.overrun_margin);
    // WAITING_SPEC（協定版本 4）：排第一但 TaskSpec 還沒收齊超過這麼久 → 向每個鄰居要、告警（task_spec_missing），不算失敗
    assign.spec_wait = declare_parameter<double>("spec_wait", assign.spec_wait);
    spec_wait_ = assign.spec_wait;
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

    // ---------- 機內話題（QoS 和機器狗端的 task_manager 一致）----------
    current_pub_ = create_publisher<TaskMsg>(topic("current_task"), rclcpp::QoS(1).reliable().transient_local());
    // 晚啟動的 BT 也要知道之前結束的任務：transient_local、保留最近 100 個
    closed_pub_ = create_publisher<TaskMsg>(topic("task_closed"), rclcpp::QoS(100).reliable().transient_local());
    // 全隊已知的任務（無人機的火點去重複用，2026-10-14）：預設只有無人機發，狗的 cbba_node 完全不變
    // （publish_announced 參數，測試時狗也可以發）
    if (publish_announced) {
      announced_pub_ = create_publisher<TaskMsg>(topic("task_announced"),
          rclcpp::QoS(256).reliable().transient_local());
    }
    orphan_pub_ = create_publisher<TaskMsg>(topic("task_orphaned"), rclcpp::QoS(20).reliable());
    // 排第一卻等 TaskSpec 超過 spec_wait（協定版本 4，2026-11-12）：每 spec_wait 發一次，給閘道重送、改版或取消
    spec_missing_pub_ = create_publisher<TaskMsg>(topic("task_spec_missing"), rclcpp::QoS(20).reliable());
    // 路徑（2026-10-27）：只要最新的一份，晚訂閱的也拿得到
    // 同機號衝突（協定版本 4，2026-11-11）：衝突的機號，每次新的衝突發一次（給閘道、地面站）
    conflict_pub_ = create_publisher<std_msgs::msg::UInt16>(topic("agent_conflict"), rclcpp::QoS(10).reliable());
    path_pub_ = create_publisher<std_msgs::msg::UInt64MultiArray>(topic("cbba_path"),
        rclcpp::QoS(1).reliable().transient_local());
    path_pub_->publish(std_msgs::msg::UInt64MultiArray());
    // 送出位元組的累計（2026-11-09，第 5 週頻寬圖）：[全部, 重傳, 轉送]，送進 UDP 之前（tc 丟掉的也算），
    // 每 tx_stats_sec 發一次；整合測試用固定的 10 s 區間相減，算每台的 TX、重傳 TX
    tx_stats_sec_ = declare_parameter<double>("tx_stats_sec", tx_stats_sec_);
    tx_pub_ = create_publisher<std_msgs::msg::UInt64MultiArray>(topic("cbba_tx"), rclcpp::QoS(10).reliable());
    if (participate_ == "state") {
      // best effort 的訂閱可以接 reliable 和 best effort 的發布（BT 用 reliable）
      state_sub_ = create_subscription<RobotStateMsg>(
        topic("robot_state"), rclcpp::QoS(10).best_effort(),
        [this](RobotStateMsg::ConstSharedPtr msg) {onRobotState(*msg);});
    }
    // 任務結果、新任務只有一則，掉了就沒有了：用 reliable（發的一方也要用 reliable）
    result_sub_ = create_subscription<TaskResultMsg>(
      topic("task_result"), rclcpp::QoS(20).reliable(),
      [this](TaskResultMsg::ConstSharedPtr msg) {onTaskResult(*msg);});
    // new_task 可能一次來很多（整區巡檢點）：佇列要夠深，否則 callback 來不及處理時舊的會被蓋掉
    new_task_sub_ = create_subscription<TaskMsg>(
      topic("new_task"), rclcpp::QoS(256).reliable(),
      [this](TaskMsg::ConstSharedPtr msg) {onNewTask(*msg);});
    cancel_sub_ = create_subscription<TaskMsg>(
      topic("cancel_task"), rclcpp::QoS(20).reliable(),
      [this](TaskMsg::ConstSharedPtr msg) {onCancelTask(*msg);});

    timer_ = create_wall_timer(std::chrono::milliseconds(tick_ms), [this]() {step();});

    RCLCPP_INFO(get_logger(),
      "agent %d（%s）：UDP %s:%u（網卡 %s），session %016llX；話題 %s %s %s %s；參與出價：%s",
      agent_id, agent_type.c_str(), udp.group.c_str(), udp.port,
      udp.interface_ip.empty() ? "系統預設" : udp.interface_ip.c_str(),
      static_cast<unsigned long long>(comm_->sessionId()), current_pub_->get_topic_name(),
      state_sub_ ? state_sub_->get_topic_name() : "（不訂閱 robot_state）",
      result_sub_->get_topic_name(), new_task_sub_->get_topic_name(),
      participate_ == "always" ? "一律（位置與電量來自參數）" :
      "robot_state 在 state_timeout 內有更新且 mode = AUTO");
    RCLCPP_INFO(get_logger(), "平台 %s，能力 %s%s", platformName(platformOf(s.type)),
      capabilitiesToString(availableCaps(s)).c_str(), s.capabilities == 0 ? "（依 agent_type 的預設）" : "");
  }

  ~CbbaNode() override
  {
    if (lock_fd_ >= 0) {
      ::close(lock_fd_);   // 釋放程序鎖（程序結束時核心也會釋放）
    }
  }

private:
  // 讀世代、+ 1、寫回（先寫暫存檔再 rename，寫到一半斷電也不會留下壞掉的檔案）。寫回成功才用：
  // 寫不回去的話下次啟動會拿到同一個世代，建立的 task_id 會和這次的撞在一起。
  // 讀寫失敗時：required（閘道）拒絕啟動；否則（機器人，很少建任務）用隨機世代並警告
  Epoch loadEpoch(const std::string & dir, int agent_id, bool required)
  {
    const std::string path = dir + "/epoch_" + std::to_string(agent_id);
    std::string error;
    unsigned long previous = 0;
    {
      std::ifstream in(path);
      if (in && !(in >> previous)) {
        error = "內容不是數字";
      }
    }
    if (error.empty() && previous > 0xFFFFu) {
      error = "內容超過 65535";
    }
    Epoch epoch = 0;
    if (error.empty()) {
      epoch = nextEpoch(static_cast<Epoch>(previous));
      std::error_code ec;
      std::filesystem::create_directories(dir, ec);
      const std::string tmp = path + ".tmp";
      {
        std::ofstream out(tmp, std::ios::trunc);
        out << epoch << "\n";
        out.flush();
        if (!out) {
          error = "寫不進 " + tmp;
        }
      }
      if (error.empty()) {
        std::filesystem::rename(tmp, path, ec);
        if (ec) {
          error = "rename 失敗：" + ec.message();
        }
      }
    }
    if (error.empty()) {
      RCLCPP_INFO(get_logger(), "世代 %u（%s，上次 %lu）", static_cast<unsigned>(epoch), path.c_str(), previous);
      return epoch;
    }
    if (required) {
      throw std::runtime_error("epoch file " + path + ": " + error + "（epoch_required = true，拒絕啟動）");
    }
    std::random_device rd;
    epoch = static_cast<Epoch>(1 + rd() % 0xFFFFu);
    RCLCPP_WARN(get_logger(), "世代檔 %s：%s → 改用隨機世代 %u（重開機後可能和之前的撞，機器人很少建任務所以只警告）",
      path.c_str(), error.c_str(), static_cast<unsigned>(epoch));
    return epoch;
  }

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
    if (have_state_ && msg.mode != mode_) {
      // 格式照機器狗端（伯宇的 e2e 腳本 grep「mode 0 → 1」），名稱補在後面
      RCLCPP_INFO(get_logger(), "mode %u → %u（%s → %s）", mode_, msg.mode, modeName(mode_),
        modeName(msg.mode));
    }
    mode_ = msg.mode;
    have_state_ = true;
    last_state_ = wallNow();
  }

  void onNewTask(const TaskMsg & msg)
  {
    const double now = wallNow();
    // 協定版本 3 的 TASK_ANNOUNCE 只帶舊型任務（1～3）；新型任務（type 0，平台／能力／優先級）是協定版本 4、由閘道建立
    if (msg.type < TaskMsg::AIR_RECON || msg.type > TaskMsg::PATROL) {
      RCLCPP_WARN(get_logger(), "新任務 %s 沒有加入：type %u 不是 1～3", taskName(msg.task_id).c_str(), msg.type);
      return;
    }
    Task t;
    t.id = msg.task_id;
    t.type = static_cast<TaskType>(msg.type);
    t.position = {msg.position.x, msg.position.y, msg.position.z};
    // BT 常給地面的座標（z = 0）。低於巡檢高度時改成巡檢高度，否則每段距離都多算到地面的高度差，
    // 耗電也估錯。要和執行者（BT 的巡檢高度）一致。地面處置任務不改
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
    // TaskSpec（協定版本 4）：new_task 帶 spec 的話一起送給全隊（閘道注入在第 4 步；這裡給測試、整合用）
    if (comm_->addLocalTask(t, now, msg.spec)) {
      RCLCPP_INFO(get_logger(), "新任務 %s (%.1f, %.1f, %.1f) value %.0f，期限 %.0f s%s",
        taskName(t.id).c_str(), t.position.x, t.position.y, t.position.z, t.value, t.deadline_sec,
        msg.spec.empty() ? "" : ("，TaskSpec " + std::to_string(msg.spec.size()) + " B").c_str());
    } else {
      RCLCPP_WARN(get_logger(),
        "新任務 %s 沒有加入：task_id 要是 %u × 65536 + 流水號，而且不能和已知的任務重複",
        taskName(msg.task_id).c_str(), comm_->id());
    }
  }

  // 只接受目前 current_task 上的任務的回報；指派版本由節點自己補（BT 不用帶）
  void onTaskResult(const TaskResultMsg & msg)
  {
    const std::string name = taskName(msg.task_id);
    const double now = wallNow();
    // 格式照機器狗端（e2e 腳本 grep「reason 2，detail「…」」），reason 的名稱只能補在最後
    RCLCPP_INFO(get_logger(), "task_result %s：success %d，reason %u，detail「%s」（%s）",
      name.c_str(), msg.success ? 1 : 0, msg.reason, msg.detail.c_str(), reasonName(msg.reason));
    const auto & active = assignments_->active();
    if (!active || active->task != msg.task_id) {
      RCLCPP_WARN(get_logger(), "任務 %s 的回報拒絕：不是目前 current_task 上的任務（目前是 %s），任務狀態不變",
        name.c_str(), active ? taskName(active->task).c_str() : "無");
      return;
    }
    // spec §2.3 規則 3：PERMANENT 這台永久不再對它出價，PREEMPTED preempt_exclude_sec 秒，
    // 其他（TRANSIENT／STOPPED）failure_exclude_sec 秒
    const double exclude = msg.reason == TaskResultMsg::PERMANENT ? kInf :
      msg.reason == TaskResultMsg::PREEMPTED ? preempt_exclude_sec_ : failure_exclude_sec_;
    const ReportCheck check = assignments_->onResult(msg.task_id, active->version, msg.success, now, exclude);
    if (check != ReportCheck::ACCEPTED) {
      RCLCPP_WARN(get_logger(), "任務 %s 的回報拒絕（%s），任務狀態不變", name.c_str(), toString(check));
      return;
    }
    // 括號內照機器狗端放 detail（比 reason 名稱具體）；BT 沒給 detail 時用 reason 名稱
    const char * why = msg.detail.empty() ? reasonName(msg.reason) : msg.detail.c_str();
    if (msg.success) {
      reported_at_[msg.task_id] = now;
      RCLCPP_INFO(get_logger(), "任務 %s 完成，等待全隊確認", name.c_str());
    } else if (std::isinf(exclude)) {
      RCLCPP_WARN(get_logger(), "任務 %s 失敗（%s），交回競標池，這台不再接", name.c_str(), why);
    } else {
      RCLCPP_WARN(get_logger(), "任務 %s 失敗（%s），交回競標池，這台 %.0f s 內不接",
        name.c_str(), why, exclude);
    }
    // TRANSIENT 才算「這台做這個點會失敗」：之後的出價打折，別台優先接手（上面的 log 格式照機器狗端，另外印一行）
    if (!msg.success && msg.reason == TaskResultMsg::TRANSIENT) {
      comm_->countFailure(msg.task_id, now);
      const auto & agent = comm_->agent();
      const int f = agent.failures(msg.task_id);
      const auto it = agent.tasks().find(msg.task_id);
      const double p = it == agent.tasks().end() ? 1.0 : it->second.success_prob;
      const int max_failures = agent.params().max_failures;
      if (max_failures > 0 && f >= max_failures) {
        RCLCPP_WARN(get_logger(), "任務 %s 這台失敗 %d 次（max_failures %d），之後不再接", name.c_str(), f, max_failures);
      } else {
        RCLCPP_INFO(get_logger(), "任務 %s 這台失敗 %d 次，估計成功率 %.2f，之後的出價打折", name.c_str(), f, p);
      }
    }
  }

  // 孤兒任務：開著、沒有人得標、也沒有正式指派，超過 orphan_warn_sec 印警告、發 task_orphaned 一次；之後有人得標時印一行。
  // 每台各自看本機的表（得標者經由 CBBA_STATE 傳到每台），不需要知道別台的排除狀態
  void checkOrphans(double now)
  {
    if (!(orphan_warn_sec_ > 0.0)) {
      return;
    }
    const auto & agent = comm_->agent();
    for (const auto & [id, task] : agent.tasks()) {
      const AgentId winner = agent.winner(id);
      const AgentId assignee = comm_->assignment(id).assignee;
      if (task.status != TaskStatus::OPEN || winner != kNoAgent || assignee != kNoAgent) {
        if (orphan_warned_.erase(id) > 0 && task.status == TaskStatus::OPEN) {
          RCLCPP_INFO(get_logger(), "任務 %s 有人得標了（agent %u），不再是孤兒任務", taskName(id).c_str(),
            winner != kNoAgent ? winner : assignee);
        }
        unassigned_since_.erase(id);
        continue;
      }
      const double since = unassigned_since_.emplace(id, now).first->second;
      if (now - since >= orphan_warn_sec_ && orphan_warned_.insert(id).second) {
        RCLCPP_WARN(get_logger(), "孤兒任務 %s (%.1f, %.1f, %.1f)：開著但 %.0f s 沒有人得標"
          "（能做的載具可能都被永久排除、不在 AUTO 或失聯），請地面站處理", taskName(id).c_str(),
          task.position.x, task.position.y, task.position.z, now - since);
        orphan_pub_->publish(toMsg(task));
      }
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

  // robot_state → 參與出價（spec §2.3 規則 2、4）。AGENT_STATE 的飛行狀態旗標不再有來源，送 false
  void updateStatus(double now)
  {
    VehicleStatus status;
    bool participate = true;
    if (participate_ == "state") {
      const bool fresh = have_state_ && now - last_state_ <= state_timeout_;
      status.telemetry_ok = fresh;
      participate = fresh && mode_ == RobotStateMsg::AUTO;
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
  // 每個 tick：收 UDP → 協定層 → 送 UDP → 指派流程 → current_task
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

    assignments_->setExec(ExecInput{}, now);   // 不握手、沒有 BT 的執行狀態
    assignments_->step(now);
    assignments_->takeRequests();              // 不握手：沒有要送給 BT 的請求
    publishOutputs(now);
    checkSpecWait(now);
    report(now);
    logStats(now);
    publishTx(now);
  }

  void publishTx(double now)
  {
    if (tx_stats_sec_ <= 0.0 || now - tx_last_ < tx_stats_sec_) {
      return;
    }
    tx_last_ = now;
    const auto & st = comm_->stats();
    std_msgs::msg::UInt64MultiArray m;
    m.data = {st.tx_bytes, st.tx_retry_bytes, st.tx_relay_bytes};
    tx_pub_->publish(m);
  }

  // 封包統計（2026-10-28，協定版本 3）：每 stats_log_sec 印一次這段時間送出的流量與各種封包的則數、位元組，
  // 弱網測試比對補發的流量用（版本 2 的補發和已結束的任務數成正比）。0 = 不印
  void logStats(double now)
  {
    if (stats_log_sec_ <= 0.0) {
      return;
    }
    if (stats_last_ < 0.0) {
      stats_last_ = now;
      stats_prev_ = comm_->stats();
      return;
    }
    const double dt = now - stats_last_;
    if (dt < stats_log_sec_) {
      return;
    }
    const CommStats & st = comm_->stats();
    std::string detail;
    std::size_t bytes = 0;
    for (const auto & [type, count] : st.sent) {
      const auto prev = stats_prev_.sent.find(type);
      const auto prev_bytes = stats_prev_.sent_bytes.find(type);
      const int n = count - (prev != stats_prev_.sent.end() ? prev->second : 0);
      const std::size_t b = st.sent_bytes.at(type) -
        (prev_bytes != stats_prev_.sent_bytes.end() ? prev_bytes->second : 0);
      bytes += b;
      if (n > 0) {
        detail += std::string(detail.empty() ? "" : "、") + wire::toString(type) + " " + std::to_string(n) +
          " 則 " + std::to_string(b) + " B";
      }
    }
    RCLCPP_INFO(get_logger(),
      "封包統計 %.0f s：送出 %.2f kB/s（%s）；任務 %zu 個、補發佇列 %zu、缺口請求 %d、補發 %d 筆、收到補發 %d 筆、"
      "額度不足 %d 次", dt, bytes / dt / 1024.0, detail.c_str(), comm_->taskCount(), comm_->repairQueued(),
      st.gap_requests - stats_prev_.gap_requests, st.repair_items_sent - stats_prev_.repair_items_sent,
      st.repair_items_applied - stats_prev_.repair_items_applied, st.repair_deferred - stats_prev_.repair_deferred);
    stats_prev_ = st;
    stats_last_ = now;
  }

  // WAITING_SPEC 超過 spec_wait：告警（log＋task_spec_missing），之後每 spec_wait 再一次。補發由 CbbaComm 加強
  void checkSpecWait(double now)
  {
    const auto & w = assignments_->waitingSpec();
    if (!w || now - w->since < spec_wait_) {
      spec_warned_task_ = 0;
      return;
    }
    if (w->task == spec_warned_task_ && now - spec_warned_at_ < spec_wait_) {
      return;
    }
    spec_warned_task_ = w->task;
    spec_warned_at_ = now;
    const auto it = comm_->agent().tasks().find(w->task);
    if (it == comm_->agent().tasks().end()) {
      return;
    }
    RCLCPP_WARN(get_logger(), "任務 %s 排第一，但 TaskSpec（revision %u）等了 %.1f s 還沒收齊：向每個鄰居要，"
      "不發 current_task（不算失敗）", taskName(w->task).c_str(), static_cast<unsigned>(it->second.revision),
      now - w->since);
    spec_missing_pub_->publish(toMsg(it->second));
  }

  void publishOutputs(double now)
  {
    if (assignments_->takeAssignedChanged()) {
      TaskMsg msg;
      const auto & active = assignments_->active();
      if (active && comm_->agent().tasks().count(active->task) > 0) {
        const Task & t = comm_->agent().tasks().at(active->task);
        msg = toMsg(t);
        // TaskSpec（協定版本 4）：正式指派只在收齊後才建立（WAITING_SPEC），這裡一定有
        msg.spec = comm_->taskSpec(t.id).value_or(std::string());
        RCLCPP_INFO(get_logger(), "指派給 BT：%s (%.1f, %.1f, %.1f)", taskName(t.id).c_str(),
          t.position.x, t.position.y, t.position.z);
      } else {
        msg.status = TaskMsg::CANCELLED;   // task_id = 0：沒有任務
        msg.status_stamp = toRosTime(now);
        RCLCPP_INFO(get_logger(), "指派給 BT：無");
      }
      current_pub_->publish(msg);
    }
    for (const AssignmentTiming & t : assignments_->takeTimings()) {
      const auto ms = [](double a, double b) {return a >= 0.0 && b >= 0.0 ? (b - a) * 1e3 : -1.0;};
      RCLCPP_INFO(get_logger(), "量測 %s：建立→排第一 %.0f ms、排第一→發布 %.0f ms（assign_hold）；建立→發布 %.0f ms",
        taskName(t.task).c_str(), ms(t.created, t.head), ms(t.head, t.assigned), ms(t.created, t.assigned));
    }
  }

  // 出價表（2026-10-20，使用者要每個任務每一台的出價當參考資料）：開著的任務第一次知道、得標者改變、
  // 這台的出價狀態（why）改變時印一行固定格式；docker/itest/bid_table.py 收集各台（無人機、狗）的 log 做成表格。
  // 只在這些時刻印，飛行中出價隨位置變動不印（會洗版）
  void logBids(double now)
  {
    const auto & agent = comm_->agent();
    for (const auto & [id, task] : agent.tasks()) {
      if (task.status != TaskStatus::OPEN) {
        continue;
      }
      const CbbaAgent::BidInfo b = agent.bidFor(id, now);
      const AgentId w = agent.winner(id);
      const auto key = std::make_pair(w, std::string(b.why));
      const auto it = bid_logged_.find(id);
      if (it != bid_logged_.end() && it->second == key) {
        continue;
      }
      bid_logged_[id] = key;
      RCLCPP_INFO(get_logger(), "出價表 task=%s type=%u x=%.1f y=%.1f agent=%u bid=%.3f why=%s winner=%u winner_bid=%.3f",
        taskName(id).c_str(), static_cast<unsigned>(task.type), task.position.x, task.position.y,
        static_cast<unsigned>(comm_->id()), b.bid, b.why,
        static_cast<unsigned>(w), agent.score(id));
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
      std_msgs::msg::UInt64MultiArray msg;
      msg.data.assign(path.begin(), path.end());
      path_pub_->publish(msg);
    }
    // 新知道的任務（自己建立、或收到別台的 TASK_ANNOUNCE）轉給 BT 一次
    if (announced_pub_) {
      for (const auto & [id, task] : comm_->agent().tasks()) {
        if (announced_.insert(id).second) {
          announced_pub_->publish(toMsg(task));
        }
      }
    }
    logBids(now);
    // 任務結束（自己收齊確認、或收到別台的 TASK_CLOSE）時印一次
    for (const auto & [id, task] : comm_->agent().tasks()) {
      if (task.status == TaskStatus::OPEN || !closed_.insert(id).second) {
        continue;
      }
      closed_pub_->publish(toMsg(task));
      if (task.status != TaskStatus::DONE) {
        RCLCPP_INFO(get_logger(), "任務 %s 已取消", taskName(id).c_str());
      } else if (const auto it = reported_at_.find(id); it != reported_at_.end()) {
        RCLCPP_INFO(get_logger(), "任務 %s 已確認完成（回報後 %.2f s）",
          taskName(id).c_str(), now - it->second);
      } else {
        RCLCPP_INFO(get_logger(), "任務 %s 已確認完成（收到完成證明）", taskName(id).c_str());
      }
    }

    checkOrphans(now);

    const bool converged = comm_->converged(now);
    if (converged && !last_converged_) {
      RCLCPP_INFO(get_logger(), "收斂：%zu 個任務，%zu 個鄰居", comm_->taskCount(),
        comm_->aliveNeighbors(now).size());
    }
    last_converged_ = converged;
    // 鄰居開始／停止參與出價時印一次
    for (const auto & [agent, n] : comm_->neighbors()) {
      if (!n.has_status) {
        continue;
      }
      const auto last = neighbor_participating_.find(agent);
      if (last == neighbor_participating_.end() || last->second != n.flags.participating) {
        neighbor_participating_[agent] = n.flags.participating;
        RCLCPP_INFO(get_logger(), "鄰居 %u：%s", agent, n.flags.participating ? "參與出價" : "不參與出價");
      }
    }
    // 鎖著、但持有者失聯的任務：先鎖著（原載具可能還在執行），到失聯接手的時限才撤銷、重新競標
    const auto lost = comm_->lockedByLostAgents(now);
    if (lost != last_lost_) {
      for (const auto & [task, holder] : lost) {
        if (std::find(last_lost_.begin(), last_lost_.end(), std::make_pair(task, holder)) == last_lost_.end()) {
          RCLCPP_WARN(get_logger(), "任務 %s 的持有者 agent %u 失聯：先鎖著，%.0f s 後沒恢復就重新競標",
            taskName(task).c_str(), holder, comm_->takeoverLimit(task));
        }
      }
      last_lost_ = lost;
    }
    for (const auto & [task, holder] : comm_->takeTakeovers()) {
      RCLCPP_WARN(get_logger(), "失聯接手：任務 %s 的持有者 agent %u 失聯超過 %.0f s，撤銷、重新競標",
        taskName(task).c_str(), holder, comm_->takeoverLimit(task));
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
    reportConflicts(now);
  }

  // 同機號衝突（協定版本 4）：log 錯誤、agent_conflict 發衝突的機號（自己的機號被別台用了也發自己的）
  void reportConflicts(double now)
  {
    for (const auto & [agent, c] : comm_->conflicts()) {
      if (conflict_reported_.insert({agent, c.rejected_session}).second) {
        RCLCPP_ERROR(get_logger(),
          "同機號衝突：機號 %u 有兩台在講話（session %016llX、%016llX）→ 留先出現的 %016llX，"
          "較晚那台的封包和它建立的任務都拒收。檢查各機的 agent_id", agent,
          static_cast<unsigned long long>(c.kept_session), static_cast<unsigned long long>(c.rejected_session),
          static_cast<unsigned long long>(c.kept_session));
        conflict_pub_->publish(std_msgs::msg::UInt16().set__data(agent));
      }
    }
    const auto self = comm_->selfConflict(now);
    if (self && conflict_reported_.insert({agent_id_, *self}).second) {
      RCLCPP_ERROR(get_logger(),
        "同機號衝突：另一台也用機號 %u（session %016llX，我是 %016llX）。先出現的那台會被留下，"
        "如果我比較晚，我建立的任務別台會拒收。檢查各機的 agent_id", agent_id_,
        static_cast<unsigned long long>(*self), static_cast<unsigned long long>(comm_->sessionId()));
      conflict_pub_->publish(std_msgs::msg::UInt16().set__data(agent_id_));
    }
  }

  int lock_fd_{-1};
  Epoch epoch_{0};
  std::set<std::pair<AgentId, std::uint64_t>> conflict_reported_;   // 已經報過的（機號, 被拒收的 session）
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr conflict_pub_;
  AgentId agent_id_{50};
  std::string participate_{"state"};
  double state_timeout_{1.5};
  double recon_altitude_{5.0};
  double failure_exclude_sec_{60.0};
  double preempt_exclude_sec_{60.0};

  AgentState state_;
  std::unique_ptr<CbbaComm> comm_;
  std::unique_ptr<UdpLink> udp_;
  std::unique_ptr<AssignmentManager> assignments_;
  std::vector<std::pair<TaskId, AgentId>> last_lost_;
  bool have_state_{false};
  double last_state_{0.0};
  std::uint8_t mode_{RobotStateMsg::AUTO};

  std::vector<TaskId> last_path_;
  double stats_log_sec_{10.0};   // 封包統計的間隔（0 = 不印）
  double tx_stats_sec_{0.2};     // cbba_tx 的間隔（0 = 不發）
  double tx_last_{-1e9};
  double stats_last_{-1.0};
  CommStats stats_prev_;
  std::map<TaskId, std::pair<AgentId, std::string>> bid_logged_;   // 出價表：上次印的（得標者, why）
  bool last_converged_{false};
  std::map<TaskId, double> reported_at_;   // 自己回報完成的時刻
  std::set<TaskId> closed_;                // 已經印過結束的任務
  std::set<TaskId> announced_;             // 已經發過 task_announced 的任務
  double orphan_warn_sec_{30.0};
  std::map<TaskId, double> unassigned_since_;   // 開著、沒有人得標的任務 → 從什麼時候開始
  std::set<TaskId> orphan_warned_;              // 已經警告過的孤兒任務
  int last_conflicts_{0};
  int last_rejected_{0};
  int last_rejected_closes_{0};
  int last_aborted_{0};
  std::map<AgentId, bool> neighbor_participating_;   // 上次印出的鄰居參與狀態

  rclcpp::Publisher<TaskMsg>::SharedPtr current_pub_;
  rclcpp::Publisher<TaskMsg>::SharedPtr closed_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr tx_pub_;
  rclcpp::Publisher<TaskMsg>::SharedPtr announced_pub_;   // 預設只有無人機（publish_announced）
  rclcpp::Publisher<TaskMsg>::SharedPtr orphan_pub_;
  rclcpp::Publisher<TaskMsg>::SharedPtr spec_missing_pub_;
  double spec_wait_{5.0};
  TaskId spec_warned_task_{0};
  double spec_warned_at_{-1e9};
  rclcpp::Publisher<std_msgs::msg::UInt64MultiArray>::SharedPtr path_pub_;
  rclcpp::Subscription<RobotStateMsg>::SharedPtr state_sub_;
  rclcpp::Subscription<TaskResultMsg>::SharedPtr result_sub_;
  rclcpp::Subscription<TaskMsg>::SharedPtr new_task_sub_;
  rclcpp::Subscription<TaskMsg>::SharedPtr cancel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace cbba_core

namespace
{

// 當機時印呼叫堆疊（2026-10-28）：70% 掉包的弱網測試遇過一次 Segmentation fault，之後在 gdb 底下沒有重現
// （gdb 會改變時序）。改成節點自己處理：平常不做任何事、不影響時序；收到 SIGSEGV 等訊號時把堆疊寫到 stderr，
// 再照預設的方式結束（ros2 run 照樣印 Segmentation fault）。位址換成行號：
//   addr2line -f -C -e $(ros2 pkg prefix swarm_cbba)/lib/swarm_cbba/cbba_node <括號裡的 +0x 位移>
// （RelWithDebInfo 編譯才有行號；cbba_core 的函式在 libcbba_core_core 的那幾行，用那個 .so 換）
// 訊號處理裡只用 async-signal-safe 的函式：write、backtrace_symbols_fd（backtrace 在安裝時先呼叫一次，載入 libgcc）
void onCrash(int sig)
{
  static const char kHead[] = "\n===== cbba_node 當掉：收到訊號，呼叫堆疊如下 =====\n";
  if (write(STDERR_FILENO, kHead, sizeof(kHead) - 1) < 0) {}
  void * frames[64];
  const int n = backtrace(frames, 64);
  backtrace_symbols_fd(frames, n, STDERR_FILENO);
  static const char kTail[] = "===== 呼叫堆疊結束 =====\n";
  if (write(STDERR_FILENO, kTail, sizeof(kTail) - 1) < 0) {}
  std::signal(sig, SIG_DFL);
  std::raise(sig);
}

void installCrashHandler()
{
  void * warm[1];
  backtrace(warm, 1);
  // 堆疊溢位時原本的堆疊不能用：處理函式在另外一塊堆疊上跑
  static char alt_stack[64 * 1024];
  stack_t ss{};
  ss.ss_sp = alt_stack;
  ss.ss_size = sizeof(alt_stack);
  sigaltstack(&ss, nullptr);
  struct sigaction sa{};
  sa.sa_handler = onCrash;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_ONSTACK | SA_RESETHAND;
  for (int sig : {SIGSEGV, SIGBUS, SIGFPE, SIGILL, SIGABRT}) {
    sigaction(sig, &sa, nullptr);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  installCrashHandler();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cbba_core::CbbaNode>());
  rclcpp::shutdown();
  return 0;
}
