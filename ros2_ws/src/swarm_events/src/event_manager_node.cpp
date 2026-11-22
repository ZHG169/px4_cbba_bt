// 事件管理節點（GCS；2026-11-23，docs/l5_l6_plan.md A、B 批）
//
// A 批：L6 範本展開的服務 expand_incident（GCS 是 /gcs/expand_incident）：**只回傳規劃結果，不注入**。
// B 批：L2 累積、L3 關聯、L4 VLM 介面、L5 事件狀態機（swarm_events::IncidentManager）接上 ROS：
//   訂 detections（閘道收到的偵測）→ 事件 → 決定派遣 → 展開（RuleExpander）→ 對每個角色呼叫閘道的 inject_task
//   （incident_id = 事件編號）；改版（incident_updates、目標移動）只注入 TaskSpec、位置或欄位改了的角色（revision + 1）；
//   換範本（中信心的確認任務升級成完整範本）時取消舊範本多出來的角色（cancel_task）。
//   回饋：task_closed（角色完成、取消）、task_reports（失敗碼；E_TARGET_LOST → 誤報）、assignments（角色已指派、執行中）、
//   fallback_request、task_orphaned（P0 沒人接 → 升級、降級派子事件）、incident_updates（UAV 定位、位置）、vlm_result。
//   輸出：incidents（EventIncident，內容變了才發）、vlm_request、operator_notices（給值班，std_msgs/String）、
//   服務 resolve_incident（值班結案、標誤報）。
//
// 參數：registry_dir、site_dir（必填，和閘道同一份）、rules_file（預設 <site_dir>/event_rules.yaml）；
//   auto_dispatch（true；false = 只判斷、不注入，派遣交給值班）、vlm_enabled（false：沒有 VLM 時直接用 L2 的信心）、
//   on_hours_start／on_hours_end（8／18，time_rule 的上班時段）、retarget_m（3.0）、
//   state_dir（空 = 不存檔；有的話存 event_manager_state.yaml、事件日誌 incidents.log，重開沿用）、id_prefix（空 = ev＋啟動時刻）。
// 註冊表、場域、規則讀不到或不一致就拒絕啟動。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "swarm_cbba/registry.hpp"
#include "swarm_events/event_rules.hpp"
#include "swarm_events/expander.hpp"
#include "swarm_events/incident_manager.hpp"
#include "swarm_interfaces/msg/detection.hpp"
#include "swarm_interfaces/msg/event_incident.hpp"
#include "swarm_interfaces/msg/fallback_request.hpp"
#include "swarm_interfaces/msg/incident_state.hpp"
#include "swarm_interfaces/msg/task.hpp"
#include "swarm_interfaces/msg/task_assignments.hpp"
#include "swarm_interfaces/msg/task_orphaned.hpp"
#include "swarm_interfaces/msg/task_report.hpp"
#include "swarm_interfaces/msg/vlm_request.hpp"
#include "swarm_interfaces/msg/vlm_result.hpp"
#include "swarm_interfaces/srv/expand_incident.hpp"
#include "swarm_interfaces/srv/inject_task.hpp"
#include "swarm_interfaces/srv/resolve_incident.hpp"

namespace
{

using ExpandIncident = swarm_interfaces::srv::ExpandIncident;
using InjectTask = swarm_interfaces::srv::InjectTask;
using ResolveIncident = swarm_interfaces::srv::ResolveIncident;
using swarm_events::Action;
using swarm_events::EventState;
using swarm_events::RoleState;

swarm_interfaces::msg::PlannedRole toMsg(const swarm_events::PlannedRole & r)
{
  swarm_interfaces::msg::PlannedRole m;
  m.role = r.role;
  m.role_index = r.role_index;
  m.role_count = r.role_count;
  m.platform = r.platform;
  m.platform_code = r.platform_code;
  m.required_capabilities = r.required_capabilities;
  m.priority = r.priority;
  m.position.x = r.x;
  m.position.y = r.y;
  m.position.z = r.z;
  m.location_source = r.location_source;
  m.optional = r.optional;
  m.skip_satisfies = r.skip_satisfies;
  m.all_or_nothing = r.all_or_nothing;
  m.precedence = r.precedence;
  m.distinct_agent_set = r.distinct_agent_set;
  m.sync_window_s = r.sync_window_s;
  m.fallback_template = r.fallback_template;
  m.requires_approval = r.requires_approval;
  m.expires_s = static_cast<float>(r.expires_s);
  m.value = static_cast<float>(r.value);
  m.service_time_s = static_cast<float>(r.service_time_s);
  m.retarget = r.retarget;
  m.spec = r.spec;
  return m;
}

class EventManagerNode : public rclcpp::Node
{
public:
  EventManagerNode()
  : Node("event_manager")
  {
    const std::string registry_dir = declare_parameter<std::string>("registry_dir", "");
    const std::string site_dir = declare_parameter<std::string>("site_dir", "");
    std::string rules_file = declare_parameter<std::string>("rules_file", "");
    auto_dispatch_ = declare_parameter<bool>("auto_dispatch", true);
    state_dir_ = declare_parameter<std::string>("state_dir", "");
    swarm_events::ManagerConfig cfg;
    cfg.vlm_enabled = declare_parameter<bool>("vlm_enabled", false);
    cfg.on_hours_start = static_cast<int>(declare_parameter<int64_t>("on_hours_start", 8));
    cfg.on_hours_end = static_cast<int>(declare_parameter<int64_t>("on_hours_end", 18));
    cfg.retarget_m = declare_parameter<double>("retarget_m", 3.0);
    retarget_m_ = cfg.retarget_m;
    cfg.id_prefix = declare_parameter<std::string>("id_prefix", "");
    if (registry_dir.empty() || site_dir.empty()) {
      throw std::invalid_argument("event_manager 要 registry_dir、site_dir");
    }
    if (rules_file.empty()) {
      rules_file = site_dir + "/event_rules.yaml";
    }
    registry_ = swarm_cbba::loadRegistry(registry_dir);
    site_ = swarm_cbba::loadSite(site_dir);
    rules_ = swarm_events::loadEventRules(rules_file);
    std::string bad;
    for (const auto & e : swarm_cbba::checkRegistry(registry_)) {
      bad += "\n  " + e;
    }
    for (const auto & e : swarm_events::checkEventRules(rules_, registry_, &site_)) {
      bad += "\n  " + e;
    }
    if (!bad.empty()) {
      throw std::runtime_error("註冊表或事件範本有錯，拒絕啟動：" + bad);
    }
    if (cfg.id_prefix.empty()) {
      // 沒有存檔時事件編號要和重開前不同（閘道的冪等鍵用 incident_id）；有存檔時編號計數會讀回來
      cfg.id_prefix = state_dir_.empty() ?
        "ev" + std::to_string(static_cast<long>(std::fmod(nowSec(), 1e6))) : std::string("ev");
    }
    manager_ = std::make_unique<swarm_events::IncidentManager>(registry_, site_, rules_, cfg);
    loadState();

    // ---------- A 批：只回傳規劃結果的展開服務 ----------
    expand_srv_ = create_service<ExpandIncident>("expand_incident",
        [this](const std::shared_ptr<ExpandIncident::Request> req, std::shared_ptr<ExpandIncident::Response> res) {
          onExpand(*req, *res);
        });

    // ---------- B 批：事件狀態機 ----------
    const auto latched = rclcpp::QoS(100).reliable().transient_local();
    incidents_pub_ = create_publisher<swarm_interfaces::msg::EventIncident>("incidents", latched);
    vlm_pub_ = create_publisher<swarm_interfaces::msg::VlmRequest>("vlm_request", rclcpp::QoS(20).reliable());
    notice_pub_ = create_publisher<std_msgs::msg::String>("operator_notices", rclcpp::QoS(50).reliable());
    cancel_pub_ = create_publisher<swarm_interfaces::msg::Task>("cancel_task", rclcpp::QoS(20).reliable());
    inject_client_ = create_client<InjectTask>("inject_task");
    resolve_srv_ = create_service<ResolveIncident>("resolve_incident",
        [this](const std::shared_ptr<ResolveIncident::Request> req, std::shared_ptr<ResolveIncident::Response> res) {
          onResolve(*req, *res);
        });
    const auto reliable = rclcpp::QoS(100).reliable();
    det_sub_ = create_subscription<swarm_interfaces::msg::Detection>("detections", reliable,
        [this](const swarm_interfaces::msg::Detection & m) {onDetection(m);});
    // 訂閱一律 volatile：發布端不論 volatile 或 transient_local 都配得上（訂閱要 transient_local 時，volatile 的發布端會收不到）
    update_sub_ = create_subscription<swarm_interfaces::msg::IncidentState>("incident_updates", reliable,
        [this](const swarm_interfaces::msg::IncidentState & m) {
          handle(manager_->onIncidentUpdate(m.incident_id, m.incident_revision, m.position_valid, m.position.x,
          m.position.y, nowSec()));
        });
    closed_sub_ = create_subscription<swarm_interfaces::msg::Task>("task_closed", reliable,
        [this](const swarm_interfaces::msg::Task & m) {onTaskClosed(m);});
    report_sub_ = create_subscription<swarm_interfaces::msg::TaskReport>("task_reports", reliable,
        [this](const swarm_interfaces::msg::TaskReport & m) {onTaskReport(m);});
    assign_sub_ = create_subscription<swarm_interfaces::msg::TaskAssignments>("assignments", reliable,
        [this](const swarm_interfaces::msg::TaskAssignments & m) {onAssignments(m);});
    fallback_sub_ = create_subscription<swarm_interfaces::msg::FallbackRequest>("fallback_request", reliable,
        [this](const swarm_interfaces::msg::FallbackRequest & m) {onFallback(m);});
    orphan_sub_ = create_subscription<swarm_interfaces::msg::TaskOrphaned>("task_orphaned", reliable,
        [this](const swarm_interfaces::msg::TaskOrphaned & m) {onOrphaned(m);});
    vlm_sub_ = create_subscription<swarm_interfaces::msg::VlmResult>("vlm_result", reliable,
        [this](const swarm_interfaces::msg::VlmResult & m) {
          RCLCPP_INFO(get_logger(), "VLM %s：%s %.2f %s", m.event_id.c_str(), m.verdict.c_str(), m.confidence,
          m.evidence.c_str());
          handle(manager_->onVlmResult(m.event_id, m.verdict, m.confidence, nowSec()));
        });
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {handle(manager_->step(nowSec()));});
    RCLCPP_INFO(get_logger(), "事件範本 %s：%zu 條規則；自動派遣 %s、VLM %s、上班時段 %d～%d 時%s",
      rules_file.c_str(), rules_.rules.size(), auto_dispatch_ ? "開" : "關", cfg.vlm_enabled ? "開" : "關",
      cfg.on_hours_start, cfg.on_hours_end, state_dir_.empty() ? "" : ("；存檔 " + state_dir_).c_str());
  }

private:
  // 每個角色注入的紀錄
  struct RoleInj
  {
    std::uint64_t task_id{0};
    std::uint16_t revision{0};
    bool has_fallback{false};
  };
  struct Injected
  {
    std::string rule;
    swarm_events::Plan plan;
    std::map<std::string, RoleInj> roles;
  };

  double nowSec() {return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}

  // ================= A 批 =================
  void onExpand(const ExpandIncident::Request & req, ExpandIncident::Response & res)
  {
    swarm_events::Incident inc;
    inc.event_id = req.event_id;
    inc.event_type = req.event_type;
    inc.revision = req.incident_revision;
    inc.x = req.location.x;
    inc.y = req.location.y;
    inc.z = req.location.z;
    inc.radius_m = req.radius_m;
    inc.zone = req.zone;
    inc.track_id = req.track_id;
    inc.asset = req.asset;
    swarm_events::ExpandOptions opt;
    opt.rule = req.rule;
    for (const auto id : req.available_agents) {
      opt.available.push_back(static_cast<cbba_core::AgentId>(id));
    }
    const swarm_events::Plan plan = swarm_events::expand(inc, rules_, registry_, site_, opt);
    res.ok = plan.ok;
    res.code = plan.code;
    res.reason = plan.reason;
    res.rule = plan.rule;
    res.zone = plan.zone;
    res.warnings = plan.warnings;
    res.plan_json = plan.json;
    for (const auto & r : plan.roles) {
      res.roles.push_back(toMsg(r));
    }
    if (plan.ok) {
      RCLCPP_INFO(get_logger(), "展開 %s（%s rev %u）→ 規則 %s、%zu 個角色、%zu 個警告（只回傳，不注入）",
        req.event_id.c_str(), req.event_type.c_str(), req.incident_revision, plan.rule.c_str(), plan.roles.size(),
        plan.warnings.size());
    } else {
      RCLCPP_WARN(get_logger(), "展開 %s（%s）失敗：%s %s", req.event_id.c_str(), req.event_type.c_str(),
        plan.code.c_str(), plan.reason.c_str());
    }
  }

  // ================= B 批：輸入 =================
  void onDetection(const swarm_interfaces::msg::Detection & m)
  {
    swarm_events::DetectionIn d;
    d.agent_id = m.agent_id;
    d.seq = m.detection_seq;
    d.detector = m.detector;
    d.label = m.label;
    d.source = m.source;
    d.score = m.score;
    d.track_id = m.track_id;
    d.position_valid = m.position_valid;
    d.x = m.position.x;
    d.y = m.position.y;
    d.z = m.position.z;
    handle(manager_->onDetection(d, nowSec()));
  }

  void onTaskClosed(const swarm_interfaces::msg::Task & m)
  {
    const auto it = task_role_.find(m.task_id);
    if (it == task_role_.end() || m.status == swarm_interfaces::msg::Task::OPEN) {
      return;
    }
    const RoleState st = m.status == swarm_interfaces::msg::Task::DONE ? RoleState::DONE : RoleState::CANCELLED;
    handle(manager_->onRoleState(it->second.first, it->second.second, st, "", nowSec()));
  }

  void onTaskReport(const swarm_interfaces::msg::TaskReport & m)
  {
    const auto it = task_role_.find(m.task_id);
    if (it == task_role_.end() || m.success) {
      return;   // 成功看 task_closed（DONE）
    }
    handle(manager_->onRoleState(it->second.first, it->second.second, RoleState::FAILED, m.failure_code, nowSec()));
  }

  void onAssignments(const swarm_interfaces::msg::TaskAssignments & m)
  {
    std::vector<Action> acts;
    for (const auto & t : m.tasks) {
      const auto it = task_role_.find(t.task_id);
      if (it == task_role_.end()) {
        continue;
      }
      RoleState st = RoleState::PLANNED;
      if (t.assign_state == swarm_interfaces::msg::TaskAssignment::ACTIVE) {
        st = RoleState::ACTIVE;
      } else if (t.assign_state != swarm_interfaces::msg::TaskAssignment::NONE) {
        st = RoleState::ASSIGNED;
      }
      auto & last = assign_state_[t.task_id];
      if (st == RoleState::PLANNED || last == st) {
        continue;
      }
      last = st;
      const auto more = manager_->onRoleState(it->second.first, it->second.second, st, "", nowSec());
      acts.insert(acts.end(), more.begin(), more.end());
    }
    handle(acts);
  }

  void onFallback(const swarm_interfaces::msg::FallbackRequest & m)
  {
    // 閘道對同一次失效（failure_version）可能重送：只處理一次
    const std::string key = m.incident_id + "#" + std::to_string(m.failure_version) + "#" + std::to_string(m.task_id);
    if (!fallback_seen_.insert(key).second) {
      return;
    }
    handle(manager_->onEscalate(m.incident_id, "閘道 FallbackRequest：" + m.reason_text, m.fallback_template,
      nowSec()));
  }

  void onOrphaned(const swarm_interfaces::msg::TaskOrphaned & m)
  {
    // P0 沒人接、整組失效：有 fallback_template 的角色由閘道發 FallbackRequest（onFallback），這裡只處理沒有的
    if (m.reason != swarm_interfaces::msg::TaskOrphaned::P0_UNASSIGNED &&
      m.reason != swarm_interfaces::msg::TaskOrphaned::GROUP_BROKEN)
    {
      return;
    }
    const auto it = task_role_.find(m.task.task_id);
    if (it == task_role_.end()) {
      return;
    }
    const auto inj = injected_.find(it->second.first);
    if (inj == injected_.end() || inj->second.roles[it->second.second].has_fallback) {
      return;
    }
    if (!orphan_seen_.insert(it->second.first + "#" + std::to_string(m.reason)).second) {
      return;
    }
    handle(manager_->onEscalate(it->second.first, "task_orphaned：" + m.detail, "", nowSec()));
  }

  void onResolve(const ResolveIncident::Request & req, ResolveIncident::Response & res)
  {
    if (req.operator_name.empty()) {
      res.ok = false;
      res.reason = "要填 operator_name";
      return;
    }
    const auto * rec = manager_->find(req.event_id);
    if (rec == nullptr) {
      res.ok = false;
      res.reason = "沒有事件 " + req.event_id;
      return;
    }
    if (swarm_events::isTerminal(rec->state)) {
      res.ok = false;
      res.reason = std::string("事件已經結案（") + swarm_events::stateName(rec->state) + "）";
      res.state = swarm_events::stateName(rec->state);
      return;
    }
    handle(manager_->resolve(req.event_id, req.false_alarm, req.operator_name + (req.note.empty() ? "" : "：" +
      req.note), nowSec()));
    res.ok = true;
    res.state = swarm_events::stateName(manager_->find(req.event_id)->state);
  }

  // ================= B 批：執行動作 =================
  void handle(const std::vector<Action> & acts)
  {
    for (const auto & a : acts) {
      switch (a.kind) {
        case Action::Kind::NOTIFY: {
            RCLCPP_WARN(get_logger(), "值班：%s", a.text.c_str());
            std_msgs::msg::String s;
            s.data = a.text;
            notice_pub_->publish(s);
            break;
          }
        case Action::Kind::VLM_REQUEST:
          requestVlm(a);
          break;
        case Action::Kind::DISPATCH:
        case Action::Kind::REPLAN:
          if (!auto_dispatch_) {
            const auto more = manager_->onDispatchResult(a.event_id, false, {}, "自動派遣關閉（auto_dispatch false）",
                nowSec());
            flushLog();
            handle(more);
          } else {
            expandAndInject(a);
          }
          break;
      }
    }
    flushLog();
    publishChanged();
    saveState();
  }

  void requestVlm(const Action & a)
  {
    const auto * rec = manager_->find(a.event_id);
    const auto * et = rec ? registry_.eventType(rec->event_type) : nullptr;
    if (rec == nullptr || et == nullptr) {
      return;
    }
    swarm_interfaces::msg::VlmRequest m;
    m.event_id = rec->event_id;
    m.event_type = rec->event_type;
    m.prompt_id = et->vlm_prompt_id;
    m.samples = static_cast<std::uint8_t>(et->vlm_samples);
    m.timeout_s = static_cast<float>(et->vlm_timeout_s);
    m.position.x = rec->x;
    m.position.y = rec->y;
    m.position.z = rec->z;
    m.sources.assign(rec->sources.begin(), rec->sources.end());
    vlm_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "%s 送 VLM 驗證（%s）", rec->event_id.c_str(), m.prompt_id.c_str());
  }

  void expandAndInject(const Action & a)
  {
    const auto * rec = manager_->find(a.event_id);
    if (rec == nullptr) {
      return;
    }
    swarm_events::Incident inc;
    inc.event_id = rec->event_id;
    inc.event_type = rec->event_type;
    inc.revision = rec->incident_revision;
    inc.x = rec->x;
    inc.y = rec->y;
    inc.z = rec->z;
    inc.radius_m = rec->radius_m;
    inc.zone = rec->zone;
    inc.track_id = rec->track_id ? "trk_" + std::to_string(rec->track_id) : std::string();
    swarm_events::ExpandOptions opt;
    opt.rule = a.rule;
    const swarm_events::Plan plan = swarm_events::expand(inc, rules_, registry_, site_, opt);
    const double now = nowSec();
    if (!plan.ok) {
      RCLCPP_WARN(get_logger(), "%s 展開 %s 失敗：%s %s", a.event_id.c_str(), a.rule.c_str(), plan.code.c_str(),
        plan.reason.c_str());
      const auto more = manager_->onDispatchResult(a.event_id, false, {}, plan.code + " " + plan.reason, now);
      flushLog();
      handle(more);
      return;
    }
    for (const auto & w : plan.warnings) {
      RCLCPP_INFO(get_logger(), "%s 展開警告：%s", a.event_id.c_str(), w.c_str());
    }
    if (!inject_client_->service_is_ready()) {
      const auto more = manager_->onDispatchResult(a.event_id, false, {}, "閘道的 inject_task 沒有回應", now);
      flushLog();
      handle(more);
      return;
    }
    Injected & inj = injected_[a.event_id];
    std::vector<std::string> to_inject;
    if (a.kind == Action::Kind::DISPATCH || inj.rule != a.rule) {
      // 新的派遣、或換範本：舊範本多出來的角色取消
      for (const auto & [role, ri] : inj.roles) {
        const bool kept = std::any_of(plan.roles.begin(), plan.roles.end(),
            [&role](const swarm_events::PlannedRole & r) {return r.role == role;});
        if (!kept && ri.task_id != 0) {
          swarm_interfaces::msg::Task t;
          t.task_id = ri.task_id;
          cancel_pub_->publish(t);
          RCLCPP_INFO(get_logger(), "%s 換範本 %s → %s：取消角色 %s", a.event_id.c_str(), inj.rule.c_str(),
            a.rule.c_str(), role.c_str());
        }
      }
      for (const auto & r : plan.roles) {
        to_inject.push_back(r.role);
      }
    } else {
      to_inject = swarm_events::changedRoles(inj.plan, plan, retarget_m_);
    }
    inj.rule = a.rule;
    inj.plan = plan;
    std::vector<std::pair<std::string, bool>> roles;
    for (const auto & r : plan.roles) {
      roles.emplace_back(r.role, r.optional);
    }
    handle(manager_->onDispatchResult(a.event_id, true, roles, "", now));
    RCLCPP_INFO(get_logger(), "%s %s %s（plan %u）：注入 %zu／%zu 個角色", a.event_id.c_str(),
      swarm_events::actionName(a.kind), a.rule.c_str(), a.plan_version, to_inject.size(), plan.roles.size());
    for (const auto & r : plan.roles) {
      if (std::find(to_inject.begin(), to_inject.end(), r.role) == to_inject.end()) {
        continue;
      }
      RoleInj & ri = inj.roles[r.role];
      const std::uint16_t revision = static_cast<std::uint16_t>(ri.revision + 1);
      auto req = std::make_shared<InjectTask::Request>();
      req->incident_id = a.event_id;
      req->role = r.role;
      req->revision = revision;
      req->incident_revision = rec->incident_revision;
      req->platform = r.platform_code;
      req->required_capabilities = r.required_capabilities;
      req->priority = r.priority;
      req->position.x = r.x;
      req->position.y = r.y;
      req->position.z = r.z;
      req->value = static_cast<float>(r.value);
      req->service_time_s = static_cast<float>(r.service_time_s);
      req->expires_sec = static_cast<float>(r.expires_s);
      req->role_index = r.role_index;
      req->role_count = r.role_count;
      req->optional = r.optional;
      req->all_or_nothing = r.all_or_nothing;
      req->skip_satisfies = r.skip_satisfies;
      req->precedence = r.precedence;
      req->sync_window_s = r.sync_window_s;
      req->distinct_agent_set = r.distinct_agent_set;
      req->fallback_template = r.fallback_template;
      req->spec = r.spec;
      req->event_type = rec->event_type;
      req->requires_approval = r.requires_approval;
      req->description = a.rule + " " + r.role;
      ri.has_fallback = !r.fallback_template.empty();
      const std::string event_id = a.event_id;
      const std::string role = r.role;
      inject_client_->async_send_request(req,
        [this, event_id, role, revision](rclcpp::Client<InjectTask>::SharedFuture f) {
          const auto res = f.get();
          onInjected(event_id, role, revision, *res);
        });
    }
  }

  void onInjected(const std::string & event_id, const std::string & role, std::uint16_t revision,
    const InjectTask::Response & res)
  {
    Injected & inj = injected_[event_id];
    RoleInj & ri = inj.roles[role];
    if (!res.accepted) {
      RCLCPP_WARN(get_logger(), "%s 角色 %s 注入被拒絕：%s", event_id.c_str(), role.c_str(), res.reason.c_str());
      handle(manager_->onDispatchResult(event_id, false, {}, "角色 " + role + " 注入被拒絕：" + res.reason, nowSec()));
      return;
    }
    ri.revision = std::max(ri.revision, revision);
    if (res.task_id != 0) {
      ri.task_id = res.task_id;
      task_role_[res.task_id] = {event_id, role};
    }
    RCLCPP_INFO(get_logger(), "%s 角色 %s rev %u → 任務 %llu%s", event_id.c_str(), role.c_str(), revision,
      static_cast<unsigned long long>(res.task_id), res.waiting_approval ? "（等核准）" : "");
    publishChanged();
    saveState();
  }

  // ================= 輸出、存檔 =================
  void flushLog()
  {
    const auto lines = manager_->takeLog();
    if (lines.empty()) {
      return;
    }
    std::ofstream file;
    if (!state_dir_.empty()) {
      file.open(state_dir_ + "/incidents.log", std::ios::app);
    }
    for (const auto & l : lines) {
      RCLCPP_INFO(get_logger(), "事件 %s", l.c_str());
      if (file) {
        file << l << "\n";
      }
    }
  }

  void publishChanged()
  {
    for (const auto & [id, r] : manager_->events()) {
      swarm_interfaces::msg::EventIncident m;
      m.event_id = r.event_id;
      m.event_type = r.event_type;
      m.state = swarm_events::stateName(r.state);
      m.reason = r.reason;
      m.rule = r.rule;
      m.incident_revision = r.incident_revision;
      m.plan_version = r.plan_version;
      m.confidence = static_cast<float>(r.confidence);
      m.evidence_score = static_cast<float>(r.s);
      m.position.x = r.x;
      m.position.y = r.y;
      m.position.z = r.z;
      m.radius_m = static_cast<float>(r.radius_m);
      m.zone = r.zone;
      m.track_id = r.track_id;
      m.sources.assign(r.sources.begin(), r.sources.end());
      m.detections = static_cast<std::uint32_t>(r.detections);
      m.parent = r.parent;
      m.children = r.children;
      const auto inj = injected_.find(id);
      for (const auto & [role, st] : r.roles) {
        m.roles.push_back(role);
        m.role_states.push_back(swarm_events::roleStateName(st.state));
        std::uint64_t tid = 0;
        if (inj != injected_.end()) {
          const auto ri = inj->second.roles.find(role);
          tid = ri != inj->second.roles.end() ? ri->second.task_id : 0;
        }
        m.task_ids.push_back(tid);
      }
      // 內容變了才發（不看時間戳記）
      std::ostringstream sig;
      sig << m.state << "|" << m.reason << "|" << m.rule << "|" << m.incident_revision << "|" << m.plan_version << "|" <<
        std::lround(m.confidence * 100) << "|" << m.detections << "|" << m.children.size();
      for (std::size_t i = 0; i < m.roles.size(); ++i) {
        sig << "|" << m.roles[i] << ":" << m.role_states[i] << ":" << m.task_ids[i];
      }
      auto & last = published_[id];
      if (last == sig.str()) {
        continue;
      }
      last = sig.str();
      m.stamp = now();
      incidents_pub_->publish(m);
    }
  }

  void saveState()
  {
    if (state_dir_.empty()) {
      return;
    }
    YAML::Emitter e;
    e << YAML::BeginMap << YAML::Key << "manager" << YAML::Value << YAML::Literal << manager_->save();
    e << YAML::Key << "tasks" << YAML::Value << YAML::BeginSeq;
    for (const auto & [event_id, inj] : injected_) {
      for (const auto & [role, ri] : inj.roles) {
        e << YAML::Flow << YAML::BeginMap << YAML::Key << "event" << YAML::Value << event_id << YAML::Key << "rule" <<
          YAML::Value << inj.rule << YAML::Key << "role" << YAML::Value << role << YAML::Key << "task_id" <<
          YAML::Value << ri.task_id << YAML::Key << "revision" << YAML::Value << ri.revision << YAML::Key <<
          "has_fallback" << YAML::Value << ri.has_fallback << YAML::EndMap;
      }
    }
    e << YAML::EndSeq << YAML::EndMap;
    const std::string path = state_dir_ + "/event_manager_state.yaml";
    std::ofstream(path + ".tmp") << e.c_str() << "\n";
    std::rename((path + ".tmp").c_str(), path.c_str());
  }

  void loadState()
  {
    if (state_dir_.empty()) {
      return;
    }
    const std::string path = state_dir_ + "/event_manager_state.yaml";
    std::ifstream in(path);
    if (!in) {
      return;
    }
    try {
      const YAML::Node doc = YAML::LoadFile(path);
      if (doc["manager"]) {
        manager_->load(doc["manager"].as<std::string>());
      }
      if (doc["tasks"]) {
        for (const auto & t : doc["tasks"]) {
          const std::string ev = t["event"].as<std::string>();
          Injected & inj = injected_[ev];
          inj.rule = t["rule"].as<std::string>();
          RoleInj & ri = inj.roles[t["role"].as<std::string>()];
          ri.task_id = t["task_id"].as<std::uint64_t>();
          ri.revision = t["revision"].as<std::uint16_t>();
          ri.has_fallback = t["has_fallback"].as<bool>();
          if (ri.task_id != 0) {
            task_role_[ri.task_id] = {ev, t["role"].as<std::string>()};
          }
        }
      }
      RCLCPP_INFO(get_logger(), "讀回 %s：%zu 個開著的事件", path.c_str(), manager_->events().size());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "讀不了 %s（%s）：從空的開始", path.c_str(), e.what());
    }
  }

  swarm_cbba::Registry registry_;
  swarm_cbba::Site site_;
  swarm_events::EventRules rules_;
  std::unique_ptr<swarm_events::IncidentManager> manager_;
  bool auto_dispatch_{true};
  double retarget_m_{3.0};
  std::string state_dir_;
  std::map<std::string, Injected> injected_;
  std::map<std::uint64_t, std::pair<std::string, std::string>> task_role_;   // task_id → (事件, 角色)
  std::map<std::uint64_t, RoleState> assign_state_;
  std::set<std::string> fallback_seen_;
  std::set<std::string> orphan_seen_;
  std::map<std::string, std::string> published_;

  rclcpp::Service<ExpandIncident>::SharedPtr expand_srv_;
  rclcpp::Service<ResolveIncident>::SharedPtr resolve_srv_;
  rclcpp::Client<InjectTask>::SharedPtr inject_client_;
  rclcpp::Publisher<swarm_interfaces::msg::EventIncident>::SharedPtr incidents_pub_;
  rclcpp::Publisher<swarm_interfaces::msg::VlmRequest>::SharedPtr vlm_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr notice_pub_;
  rclcpp::Publisher<swarm_interfaces::msg::Task>::SharedPtr cancel_pub_;
  rclcpp::Subscription<swarm_interfaces::msg::Detection>::SharedPtr det_sub_;
  rclcpp::Subscription<swarm_interfaces::msg::IncidentState>::SharedPtr update_sub_;
  rclcpp::Subscription<swarm_interfaces::msg::Task>::SharedPtr closed_sub_;
  rclcpp::Subscription<swarm_interfaces::msg::TaskReport>::SharedPtr report_sub_;
  rclcpp::Subscription<swarm_interfaces::msg::TaskAssignments>::SharedPtr assign_sub_;
  rclcpp::Subscription<swarm_interfaces::msg::FallbackRequest>::SharedPtr fallback_sub_;
  rclcpp::Subscription<swarm_interfaces::msg::TaskOrphaned>::SharedPtr orphan_sub_;
  rclcpp::Subscription<swarm_interfaces::msg::VlmResult>::SharedPtr vlm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<EventManagerNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("event_manager"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
