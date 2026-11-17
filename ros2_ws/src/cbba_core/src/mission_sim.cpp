#include "cbba_core/mission_sim.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/scoring.hpp"

namespace cbba_core
{

const char * toString(TaskOutcome outcome)
{
  switch (outcome) {
    case TaskOutcome::DONE: return "done";
    case TaskOutcome::LATE: return "late";
    case TaskOutcome::FAILED: return "failed";
    case TaskOutcome::UNASSIGNED: return "unassigned";
    case TaskOutcome::UNFINISHED: return "unfinished";
  }
  return "?";
}

int MissionResult::count(TaskOutcome o) const
{
  return static_cast<int>(std::count_if(tasks.begin(), tasks.end(),
    [o](const TaskRecord & t) {return t.outcome == o;}));
}

namespace
{
// CbbaComm 的時間是系統時鐘（秒），s 用低 32 位元的 ms：用和實機同樣量級的起點
constexpr double kEpoch = 1.7e9;

struct InFlight
{
  double at;
  std::size_t to;
  Bytes bytes;
};

// 一台載具：cbba_node（CbbaComm＋AssignmentManager）＋假的 BT＋飛行與電池
struct Vehicle
{
  MissionAgent spec;
  AgentState state;
  std::unique_ptr<CbbaComm> comm;
  std::unique_ptr<AssignmentManager> manager;
  AgentRecord record;

  bool airborne{true};
  std::optional<HeldAssignment> task;   // BT 正在執行的（assigned_task）
  Vec3 target{};
  double duration{0.0};
  double hovered{-1.0};                 // 到達後停留了多久；< 0 = 還在前往
  double arrived_at{-1.0};              // 這次到達任務點的時刻
  std::set<TaskId> last_path;           // 上次取樣時的路徑（算 bundle_drops）
};

// 每個任務的量測狀態（不輸出）
struct TaskTrack
{
  double agree_since{-1.0};             // 這段一致從什麼時候開始；< 0 = 目前不一致
  AgentId last_executor{kNoAgent};
  bool last_forced{false};              // 上一位執行者是被迫停下的（失敗、返航、搶占）
};

double effectiveHalfLife(const Task & task, const std::array<double, 4> & by_priority)
{
  if (task.ext.half_life_s > 0) {
    return task.ext.half_life_s;
  }
  return by_priority[priorityLevel(task)];
}

// 收斂當下各台的出價：第一、第二名與（record_bids 時）分項。
// 收斂要持續 converge_hold（0.5 s ＞ assign_hold 0.4 s），這時得標者通常已經正式指派、別台看到的是鎖定（bidFor = 0、
// locked），所以第一、第二名用「這台的路徑去掉這個任務、再插入」重算的出價（每台同一個方法，不看鎖定），
// bidFor 的結果只記在 bid、why
void snapshotBids(
  const std::vector<Vehicle> & fleet, const Task & task, AgentId winner, double now,
  const MissionConfig & config, TaskRecord & record, std::vector<BidRecord> & out)
{
  std::vector<double> bids;
  for (const Vehicle & v : fleet) {
    if (!v.airborne) {
      continue;
    }
    const CbbaAgent & agent = v.comm->agent();
    std::vector<Task> others;
    for (TaskId id : agent.path()) {
      const auto it = agent.tasks().find(id);
      if (id != task.id && it != agent.tasks().end()) {
        others.push_back(it->second);
      }
    }
    const AgentState & state = agent.state();
    const Insertion ins = bestInsertion(state, others, task, now, agent.params());
    if (ins.feasible && ins.score > 0.0) {
      bids.push_back(ins.score);
    }
    if (!config.record_bids) {
      continue;
    }
    const CbbaAgent::BidInfo info = agent.bidFor(task.id, now);
    BidRecord b;
    b.task = task.id;
    b.agent = v.state.id;
    b.bid = info.bid;
    b.why = info.why;
    b.winner = v.state.id == winner;
    b.feasible = ins.feasible;
    if (ins.feasible) {
      const PathEval base = evaluatePath(state, others, now, agent.params());
      b.position = ins.position;
      b.gain = ins.gain;
      b.energy_penalty = ins.eval.energy_cost - base.energy_cost;
      for (const LegEval & leg : ins.eval.legs) {
        if (leg.task_id == task.id) {
          b.eta = leg.arrival;
          b.time_reward = leg.reward;
        }
      }
    }
    out.push_back(b);
  }
  std::sort(bids.begin(), bids.end(), std::greater<double>());
  record.bidders = static_cast<int>(bids.size());
  record.top_bid = bids.empty() ? 0.0 : bids[0];
  record.second_bid = bids.size() > 1 ? bids[1] : 0.0;
  record.score_gap = bids.empty() ? -1.0 : (bids.size() == 1 ? 1.0 : (bids[0] - bids[1]) / bids[0]);
}
}  // namespace

MissionResult runMission(const std::vector<MissionAgent> & agents, const std::vector<MissionTask> & tasks,
  const MissionConfig & config, std::uint32_t seed)
{
  if (agents.empty()) {
    throw std::invalid_argument("runMission() needs at least one agent");
  }
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> delay(config.delay_min, config.delay_max);

  std::vector<Vehicle> fleet(agents.size());
  std::map<AgentId, std::size_t> index;
  for (std::size_t i = 0; i < agents.size(); ++i) {
    Vehicle & v = fleet[i];
    v.spec = agents[i];
    v.state = agents[i].state;
    v.state.home = v.state.position;
    // session_id 由 seed 決定：同一個 seed 的結果可以重現
    const std::uint64_t session = (static_cast<std::uint64_t>(seed) << 20) + v.state.id + 1;
    ScoringParams params = config.params;
    if (agents[i].max_bundle > 0) {
      params.max_bundle = agents[i].max_bundle;
    }
    v.comm = std::make_unique<CbbaComm>(v.state, params, config.comm, session);
    v.manager = std::make_unique<AssignmentManager>(*v.comm, config.assign,
        [](int, const std::string &) {});
    v.record.id = v.state.id;
    v.record.battery_start = v.state.battery;
    v.record.min_battery = v.state.battery;
    v.record.platform = platformOf(v.state.type);
    index[v.state.id] = i;
  }

  MissionResult result;
  std::vector<bool> inserted(tasks.size(), false);
  std::map<TaskId, std::size_t> task_index;
  for (std::size_t k = 0; k < tasks.size(); ++k) {
    const AgentId creator = taskCreator(tasks[k].task.id);
    if (index.count(creator) == 0) {
      throw std::invalid_argument("task " + taskName(tasks[k].task.id) + ": creator is not an agent");
    }
    TaskRecord r;
    r.id = tasks[k].task.id;
    r.label = tasks[k].label;
    r.priority = priorityLevel(tasks[k].task);
    r.platform = effectivePlatform(tasks[k].task);
    r.value = tasks[k].task.value;
    r.expected = tasks[k].expected;
    result.tasks.push_back(r);
    task_index[r.id] = k;
  }

  std::vector<TaskTrack> track(tasks.size());

  // 執行中的任務沒完成就被撤銷（核心因電量釋放）
  const auto lost = [&](Vehicle & v) {
      ++result.wrong_accepts;
      ++result.released;
      ++v.record.released;
      const std::size_t k = task_index.at(v.task->task);
      ++result.tasks[k].failures;
      track[k].last_forced = true;
      v.task.reset();
    };

  std::vector<InFlight> flight;
  double next_check = 0.0;
  double t = 0.0;
  for (; t <= config.max_time; t += config.dt) {
    const double now = kEpoch + t;

    // ---------- 插入任務（建立者的 BT 發 new_task）----------
    for (std::size_t k = 0; k < tasks.size(); ++k) {
      if (inserted[k] || t < tasks[k].insert_at) {
        continue;
      }
      Task task = tasks[k].task;
      task.created = now;
      task.status = TaskStatus::OPEN;
      task.status_stamp = now;
      fleet[index.at(taskCreator(task.id))].comm->addLocalTask(task, now);
      result.tasks[k].inserted = t;
      inserted[k] = true;
    }

    // ---------- 飛行、電池、假的 BT ----------
    for (Vehicle & v : fleet) {
      if (!v.airborne) {
        continue;
      }
      // PX4 的模擬電池：依時間線性下降，最低停在 critical_battery
      v.state.battery = std::max(config.critical_battery, v.state.battery - v.spec.drain_rate * config.dt);
      if (v.state.battery <= config.critical_battery) {
        // IsUAVBatteryCritical → ExecuteReturnToLaunch：手上的任務回報失敗（BATTERY_RTL），降落後不再出價
        v.airborne = false;
        v.record.rtl_at = t;
        if (v.task) {
          v.record.rtl_task = v.task->task;
          const std::size_t k = task_index.at(v.task->task);
          if (v.manager->onResult(v.task->task, v.task->version, false, now) == ReportCheck::ACCEPTED) {
            ++result.wrong_accepts;
            ++result.tasks[k].failures;
          }
          track[k].last_forced = true;
          v.task.reset();
        }
        v.state.position = v.state.home;   // 返航的過程不影響 CBBA（已經不參與）
        continue;
      }
      if (!v.task) {
        continue;   // 待命：原地懸停（LandOrLoiter）
      }
      v.record.work_time += config.dt;
      const double speed = v.state.cruise_speed > 0.0 ? v.state.cruise_speed : config.params.cruise_speed;
      const double d = distance(v.state.position, v.target);
      if (v.hovered < 0.0) {
        const double step = std::min(d, speed * config.dt);
        if (d > 1e-9) {
          v.state.position.x += (v.target.x - v.state.position.x) / d * step;
          v.state.position.y += (v.target.y - v.state.position.y) / d * step;
          v.state.position.z += (v.target.z - v.state.position.z) / d * step;
        }
        v.record.distance += step;
        v.state.battery -= v.spec.drain_per_meter * step;
        if (distance(v.state.position, v.target) <= config.reach_tolerance) {
          v.hovered = 0.0;
          v.arrived_at = t;
        }
      } else {
        v.hovered += config.dt;
        if (v.hovered >= v.duration) {
          // HoverAndMonitor 到時間 → ReportTaskResult success
          const TaskId id = v.task->task;
          if (v.manager->onResult(id, v.task->version, true, now) == ReportCheck::ACCEPTED) {
            TaskRecord & r = result.tasks[task_index.at(id)];
            r.done_by = v.state.id;
            r.done_at = t;
            r.arrived_at = v.arrived_at;
            ++v.record.completed;
          }
          v.task.reset();
        }
      }
    }

    // ---------- cbba_node 的 tick：robot_state → 收 → 送 → 指派流程 ----------
    for (Vehicle & v : fleet) {
      VehicleStatus status;
      status.telemetry_ok = true;
      status.flight_state_valid = true;
      status.armed = v.airborne;
      status.offboard = v.airborne;
      status.landed = !v.airborne;
      v.comm->setStatus(status);
      if (v.airborne != v.comm->participating()) {
        v.comm->setParticipating(v.airborne, now);
      }
      v.comm->setState(v.state);
    }
    std::stable_sort(flight.begin(), flight.end(),
      [](const InFlight & a, const InFlight & b) {return a.at < b.at;});
    std::size_t delivered = 0;
    for (; delivered < flight.size() && flight[delivered].at <= now; ++delivered) {
      fleet[flight[delivered].to].comm->receive(flight[delivered].bytes, now);
    }
    flight.erase(flight.begin(), flight.begin() + static_cast<std::ptrdiff_t>(delivered));
    for (std::size_t a = 0; a < fleet.size(); ++a) {
      for (Bytes & bytes : fleet[a].comm->poll(now)) {
        for (std::size_t b = 0; b < fleet.size(); ++b) {
          if (b != a && unit(rng) >= config.loss) {
            flight.push_back(InFlight{now + delay(rng), b, bytes});
          }
        }
      }
    }
    for (Vehicle & v : fleet) {
      ExecInput exec;
      exec.fresh = true;
      exec.preemptible = true;
      if (!v.airborne) {
        exec.execution_state = kExecFault;   // 返航、降落：不接任務
      } else if (v.task) {
        exec.execution_state = v.hovered < 0.0 ? 1 : 2;   // NAVIGATING／EXECUTING
        exec.has_active = true;
        exec.active_task = v.task->task;
        exec.active_version = v.task->version;
        const double speed = v.state.cruise_speed > 0.0 ? v.state.cruise_speed : config.params.cruise_speed;
        exec.remaining_time = v.hovered < 0.0 ?
          distance(v.state.position, v.target) / speed + v.duration : std::max(0.0, v.duration - v.hovered);
      }
      v.manager->setExec(exec, now);
      v.manager->step(now);
      v.manager->takeRequests();   // require_accept = false：不用回覆
      v.manager->takeTimings();
      if (!v.manager->takeAssignedChanged()) {
        continue;
      }
      // IsTaskValidAndUnchanged：assigned_task 換了就換成新的任務（從目前位置飛過去）
      const auto & active = v.manager->active();
      if (!active || !v.airborne || v.comm->agent().tasks().count(active->task) == 0) {
        if (v.task) {
          // 被搶占時，新的任務要排第一 assign_hold 才發，中間可能先撤掉舊的：路徑第一個是較高優先級的就算搶占
          const std::size_t old = task_index.at(v.task->task);
          const auto & path = v.comm->agent().path();
          const auto head = path.empty() ? task_index.end() : task_index.find(path.front());
          if (v.airborne && head != task_index.end() && result.tasks[head->second].priority <
            result.tasks[old].priority)
          {
            ++result.preempted;
            ++result.tasks[old].preemptions;
            track[old].last_forced = true;
            v.task.reset();
          } else {
            lost(v);
          }
        }
        continue;
      }
      if (v.task && v.task->task == active->task && v.task->version == active->version) {
        continue;
      }
      const Task & task = v.comm->agent().tasks().at(active->task);
      const std::size_t k = task_index.at(task.id);
      if (v.task) {
        const std::size_t old = task_index.at(v.task->task);
        if (result.tasks[k].priority < result.tasks[old].priority) {
          // 被較高優先級的任務搶占：舊的交回重新競標，不算接錯
          ++result.preempted;
          ++result.tasks[old].preemptions;
          track[old].last_forced = true;
          v.task.reset();
        } else {
          lost(v);   // 換成同級或較低的任務：舊的沒做完
        }
      }
      v.task = *active;
      v.target = task.position;
      v.duration = task.duration_sec;
      v.hovered = -1.0;
      TaskRecord & r = result.tasks[k];
      if (r.first_assigned < 0.0) {
        r.first_assigned = t;
        r.first_executor = v.state.id;
      }
      if (track[k].last_executor != kNoAgent && track[k].last_executor != v.state.id) {
        ++r.handoffs;
        if (track[k].last_forced) {
          ++r.handoffs_forced;
        }
      }
      track[k].last_executor = v.state.id;
      track[k].last_forced = false;
    }

    for (Vehicle & v : fleet) {
      v.record.min_battery = std::min(v.record.min_battery, v.state.battery);
    }

    // ---------- 量測：收斂、重複認領、路徑變動（每 check_period）----------
    if (t + 1e-9 >= next_check) {
      next_check += config.check_period;
      for (std::size_t k = 0; k < tasks.size(); ++k) {
        TaskRecord & r = result.tasks[k];
        if (!inserted[k] || r.done_at >= 0.0) {
          continue;
        }
        const int executors = static_cast<int>(std::count_if(fleet.begin(), fleet.end(),
          [&r](const Vehicle & v) {return v.task && v.task->task == r.id;}));
        if (executors >= 2) {
          r.duplicate_time += config.check_period;
        }
        if (r.converged_at >= 0.0) {
          continue;
        }
        bool agree = false;
        AgentId winner = kNoAgent;
        bool first = true;
        for (const Vehicle & v : fleet) {
          if (!v.airborne) {
            continue;
          }
          const CbbaAgent & agent = v.comm->agent();
          if (agent.tasks().count(r.id) == 0) {
            agree = false;
            first = false;
            break;
          }
          const AgentId w = agent.winner(r.id);
          if (first) {
            winner = w;
            agree = true;
            first = false;
          } else if (w != winner) {
            agree = false;
            break;
          }
        }
        if (agree && winner != kNoAgent) {
          const auto it = index.find(winner);
          if (it == index.end() || !fleet[it->second].airborne) {
            agree = false;
          } else {
            const auto & path = fleet[it->second].comm->agent().path();
            agree = std::find(path.begin(), path.end(), r.id) != path.end();
          }
        }
        if (!agree) {
          track[k].agree_since = -1.0;
          continue;
        }
        if (track[k].agree_since < 0.0) {
          track[k].agree_since = t;
        }
        if (t - track[k].agree_since + 1e-9 >= config.converge_hold) {
          r.converged_at = track[k].agree_since;
          r.converged_winner = winner;
          const Task & known = fleet[index.at(taskCreator(r.id))].comm->agent().tasks().count(r.id) > 0 ?
            fleet[index.at(taskCreator(r.id))].comm->agent().tasks().at(r.id) : tasks[k].task;
          snapshotBids(fleet, known, winner, now, config, r, result.bids);
        }
      }
      for (Vehicle & v : fleet) {
        const auto & path = v.comm->agent().path();
        const std::set<TaskId> current(path.begin(), path.end());
        for (TaskId id : v.last_path) {
          const auto it = task_index.find(id);
          if (current.count(id) == 0 && it != task_index.end() && result.tasks[it->second].done_at < 0.0) {
            ++v.record.bundle_drops;
          }
        }
        v.last_path = current;
      }
    }

    // ---------- 結束：全部插入後，全部完成或全部返航 ----------
    const bool all_inserted = std::all_of(inserted.begin(), inserted.end(), [](bool b) {return b;});
    const bool all_done = std::all_of(result.tasks.begin(), result.tasks.end(),
        [](const TaskRecord & r) {return r.done_at >= 0.0;});
    const bool all_down = std::none_of(fleet.begin(), fleet.end(), [](const Vehicle & v) {return v.airborne;});
    if (all_inserted && (all_done || all_down)) {
      break;
    }
  }
  result.end_time = std::min(t, config.max_time);

  for (std::size_t k = 0; k < tasks.size(); ++k) {
    TaskRecord & r = result.tasks[k];
    if (r.done_at >= 0.0) {
      r.outcome = r.done_at - r.inserted > tasks[k].task.deadline_sec ? TaskOutcome::LATE : TaskOutcome::DONE;
    } else if (r.failures > 0) {
      r.outcome = TaskOutcome::FAILED;
    } else if (std::any_of(fleet.begin(), fleet.end(),
      [&r](const Vehicle & v) {return v.task && v.task->task == r.id;}))
    {
      r.outcome = TaskOutcome::UNFINISHED;
    } else {
      // 沒人接過，或接了之後在出發前就因電量釋放（核心的 reevaluate）、之後沒人能接
      r.outcome = TaskOutcome::UNASSIGNED;
    }
    const Task & task = tasks[k].task;
    if (r.arrived_at >= 0.0) {
      const double h = effectiveHalfLife(task, config.eval_half_life);
      r.discounted_value = h > 0.0 ? task.value * std::exp2(-r.eta() / h) : task.value;
      if (tasks[k].ideal_eta >= 0.0) {
        r.delay = std::max(0.0, r.eta() - tasks[k].ideal_eta);   // reach_tolerance 會早一點點到
      }
    }
    r.deadline_miss = task.explicit_deadline &&
      (r.arrived_at < 0.0 || r.eta() > task.deadline_sec + 1e-9);
  }
  for (Vehicle & v : fleet) {
    v.record.battery_end = v.state.battery;
    v.record.energy_used = v.record.battery_start - v.record.battery_end;
    v.record.return_margin = v.state.battery -
      distance(v.state.position, v.state.home) * v.state.energy_per_meter - v.state.safety_reserve;
    result.agents.push_back(v.record);
  }
  return result;
}

}  // namespace cbba_core
