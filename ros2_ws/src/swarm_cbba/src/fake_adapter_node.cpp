// 假的 adapter（ExecuteStep.action 草案，2026-11-19）：/<ns>/execute_step 的 Action server，用 FakeExecutor 模擬執行
//（fake_executor.hpp）。不接真的載具：給伯宇、羅凡對 Action 欄位、BT 的 ExecuteSteps 在沒有載具時測流程。
//
//   ros2 run swarm_cbba fake_adapter --ros-args -r __ns:=/v60 -p platform:=quadruped -p registry_dir:=<repo>/registry
//   ros2 action send_goal --feedback /v60/execute_step swarm_interfaces/action/ExecuteStep
//     "{action: go_to, location_json: '{\"x\": 5.0, \"y\": 2.0}'}"   （上下兩行是同一個指令）
//
// 參數：
//   registry_dir   註冊表（必要；或環境變數 CBBA_REGISTRY_DIR）：檢查動作、參數、平台支援
//   site_dir       場域設定（或 CBBA_SITE_DIR）：location 的 zone／poi／asset／dock、patrol 的 area、return_to_dock 的 dock；
//                  沒給時 location 只收座標
//   platform       platforms.yaml 的 name（uav、quadruped）
//   speed          m/s；0 = platforms.yaml 的 cruise_speed
//   cruise_altitude、takeoff_altitude   UAV（m）
//   home           起點 [x, y, z]（return_to_dock 沒給 dock 時回這裡）
//   faults         故障注入（可以執行中 ros2 param set）："action=E_CODE[:capability][@進度]"，例如
//                  ["inspect=E_HW:thermal_camera@0.3", "go_to=E_UNREACHABLE"]
//   slowdown       實際速度的倍數（< 1 測逾時；可以執行中改）
//   feedback_hz    回饋頻率（預設 5）
//   supported_actions  宣告能做的動作（空 = 這個平台在 actions.yaml 有列實作的都模擬，not_started 的也算、附警告）
// 執行中又收到新的 goal：舊的 abort（status CANCELED、detail「被新的 goal 取代」），做新的（BT 換任務時本來應該先取消）。
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <swarm_interfaces/action/execute_step.hpp>

#include "swarm_cbba/fake_executor.hpp"
#include "swarm_cbba/registry.hpp"

namespace
{
using ExecuteStep = swarm_interfaces::action::ExecuteStep;
using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteStep>;
using swarm_cbba::FakeExecutor;
using swarm_cbba::StepSnapshot;
using swarm_cbba::StepStatus;

std::string envOr(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return v != nullptr && *v != '\0' ? v : fallback;
}

class FakeAdapter : public rclcpp::Node
{
public:
  FakeAdapter()
  : Node("fake_adapter")
  {
    const std::string registry_dir = declare_parameter<std::string>("registry_dir", envOr("CBBA_REGISTRY_DIR", ""));
    if (registry_dir.empty()) {
      throw std::invalid_argument("fake_adapter needs registry_dir (or CBBA_REGISTRY_DIR)");
    }
    registry_ = swarm_cbba::loadRegistry(registry_dir);
    const std::string site_dir = declare_parameter<std::string>("site_dir", envOr("CBBA_SITE_DIR", ""));
    if (!site_dir.empty()) {
      site_ = swarm_cbba::loadSite(site_dir);
    }
    swarm_cbba::FakeExecutorConfig c;
    c.platform = declare_parameter<std::string>("platform", c.platform);
    const swarm_cbba::PlatformEntry * p = registry_.platform(c.platform);
    if (p == nullptr) {
      throw std::invalid_argument("platform \"" + c.platform + "\" is not in platforms.yaml");
    }
    c.speed = declare_parameter<double>("speed", 0.0);
    if (!(c.speed > 0.0)) {
      c.speed = p->cruise_speed > 0.0 ? p->cruise_speed : 1.0;
    }
    c.cruise_altitude = declare_parameter<double>("cruise_altitude", c.cruise_altitude);
    c.takeoff_altitude = declare_parameter<double>("takeoff_altitude", c.takeoff_altitude);
    const auto home = declare_parameter<std::vector<double>>("home", std::vector<double>{0.0, 0.0, 0.0});
    if (home.size() != 3) {
      throw std::invalid_argument("home needs [x, y, z]");
    }
    c.home = {{home[0], home[1], home[2]}};
    c.slowdown = declare_parameter<double>("slowdown", 1.0);
    c.supported_actions = declare_parameter<std::vector<std::string>>("supported_actions", std::vector<std::string>{});
    c.faults = parseFaults(declare_parameter<std::vector<std::string>>("faults", std::vector<std::string>{}));
    feedback_period_ = 1.0 / std::max(declare_parameter<double>("feedback_hz", 5.0), 0.1);
    executor_ = std::make_unique<FakeExecutor>(registry_, site_ ? &*site_ : nullptr, c);

    params_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & ps) {
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        for (const auto & q : ps) {
          if (q.get_name() == "faults") {
            executor_->setFaults(parseFaults(q.as_string_array()));
          } else if (q.get_name() == "slowdown") {
            executor_->setSlowdown(q.as_double());
          }
        }
        return r;
      });

    server_ = rclcpp_action::create_server<ExecuteStep>(this, "execute_step",
        [](const rclcpp_action::GoalUUID &, std::shared_ptr<const ExecuteStep::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](const std::shared_ptr<GoalHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [this](const std::shared_ptr<GoalHandle> gh) {onAccepted(gh);});
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
    RCLCPP_INFO(get_logger(), "假 adapter：%s/execute_step，平台 %s、速度 %.2f m/s、%s、狀態 %s",
      get_effective_namespace().c_str(), c.platform.c_str(), c.speed,
      site_ ? ("場域 " + site_dir).c_str() : "沒有場域（location 只收座標）",
      swarm_cbba::toString(executor_->platformState()));
  }

private:
  std::vector<swarm_cbba::StepFault> parseFaults(const std::vector<std::string> & texts)
  {
    std::vector<swarm_cbba::StepFault> out;
    for (const auto & t : texts) {
      if (const auto f = FakeExecutor::parseFault(t)) {
        out.push_back(*f);
        RCLCPP_WARN(get_logger(), "故障注入：%s → %s%s 在進度 %.2f", f->action.empty() ? "全部" : f->action.c_str(),
          f->failure_code.c_str(), f->failed_capability.empty() ? "" : (":" + f->failed_capability).c_str(), f->at);
      } else {
        RCLCPP_ERROR(get_logger(), "faults 的「%s」格式不對（action=E_CODE[:capability][@進度]），略過", t.c_str());
      }
    }
    return out;
  }

  double nowSec() {return now().seconds();}

  void onAccepted(const std::shared_ptr<GoalHandle> gh)
  {
    if (current_) {
      finish(current_, executor_->cancel("被新的 goal 取代"));
      current_.reset();
    }
    const auto g = gh->get_goal();
    swarm_cbba::StepGoal goal;
    goal.task_id = g->task_id;
    goal.revision = g->revision;
    goal.step_id = g->step_id;
    goal.action = g->action;
    goal.object = g->object;
    goal.location_json = g->location_json;
    goal.params_json = g->params_json;
    goal.deadline = g->deadline.sec == 0 && g->deadline.nanosec == 0 ? 0.0 :
      rclcpp::Time(g->deadline).seconds();
    RCLCPP_INFO(get_logger(), "執行 task %lu rev %u step %u：%s%s%s%s%s", static_cast<unsigned long>(goal.task_id),
      goal.revision, goal.step_id, goal.action.c_str(), goal.object.empty() ? "" : (" object " + goal.object).c_str(),
      goal.location_json.empty() ? "" : (" location " + goal.location_json).c_str(),
      goal.params_json.empty() ? "" : " params ", goal.params_json.c_str());
    const StepSnapshot s = executor_->start(goal, nowSec());
    if (s.status != StepStatus::RUNNING) {
      finish(gh, s);
      return;
    }
    if (!s.detail.empty()) {
      RCLCPP_WARN(get_logger(), "%s", s.detail.c_str());
    }
    current_ = gh;
    last_feedback_ = -1e9;
  }

  void tick()
  {
    if (!current_) {
      return;
    }
    if (current_->is_canceling()) {
      finish(current_, executor_->cancel("BT 取消"));
      current_.reset();
      return;
    }
    const double t = nowSec();
    const StepSnapshot s = executor_->step(t);
    if (s.status != StepStatus::RUNNING) {
      finish(current_, s);
      current_.reset();
      return;
    }
    if (t - last_feedback_ >= feedback_period_) {
      last_feedback_ = t;
      auto fb = std::make_shared<ExecuteStep::Feedback>();
      fb->state = "running";
      fb->progress = static_cast<float>(s.progress);
      fb->phase = s.phase;
      fb->platform_state = swarm_cbba::toString(s.platform);
      fb->pose = pose(s);
      current_->publish_feedback(fb);
    }
  }

  geometry_msgs::msg::PoseStamped pose(const StepSnapshot & s)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = now();
    p.pose.position.x = s.position[0];
    p.pose.position.y = s.position[1];
    p.pose.position.z = s.position[2];
    p.pose.orientation.w = 1.0;
    return p;
  }

  void finish(const std::shared_ptr<GoalHandle> & gh, const StepSnapshot & s)
  {
    auto r = std::make_shared<ExecuteStep::Result>();
    r->status = static_cast<std::uint8_t>(s.status);
    r->failure_code = s.failure_code;
    r->failed_capability = s.failed_capability;
    r->findings_json = s.findings_json;
    r->detail = s.detail;
    const auto g = gh->get_goal();
    const std::string text = std::string("結果 task ") + std::to_string(g->task_id) + " step " +
      std::to_string(g->step_id) + " " + g->action + "：" + swarm_cbba::toString(s.status) +
      (s.failure_code.empty() ? "" : " " + s.failure_code) + (s.detail.empty() ? "" : "（" + s.detail + "）") +
      "，平台 " + swarm_cbba::toString(s.platform);
    if (s.status == StepStatus::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
      gh->succeed(r);
    } else if (s.status == StepStatus::CANCELED && gh->is_canceling()) {
      RCLCPP_INFO(get_logger(), "%s", text.c_str());
      gh->canceled(r);
    } else {
      RCLCPP_WARN(get_logger(), "%s", text.c_str());
      gh->abort(r);   // 失敗、逾時、被新的 goal 取代（沒有要求取消時只能 abort，status 照實填）
    }
  }

  swarm_cbba::Registry registry_;
  std::optional<swarm_cbba::Site> site_;
  std::unique_ptr<FakeExecutor> executor_;
  rclcpp_action::Server<ExecuteStep>::SharedPtr server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_;
  std::shared_ptr<GoalHandle> current_;
  double feedback_period_{0.2};
  double last_feedback_{-1e9};
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeAdapter>());
  rclcpp::shutdown();
  return 0;
}
