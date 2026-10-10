#include "cbba_core/mission_sim.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <stdexcept>

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
};
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
    v.comm = std::make_unique<CbbaComm>(v.state, config.params, config.comm, session);
    v.manager = std::make_unique<AssignmentManager>(*v.comm, config.assign,
        [](int, const std::string &) {});
    v.record.id = v.state.id;
    v.record.battery_start = v.state.battery;
    index[v.state.id] = i;
  }

  MissionResult result;
  std::vector<bool> inserted(tasks.size(), false);
  std::map<TaskId, std::size_t> task_index;
  for (std::size_t k = 0; k < tasks.size(); ++k) {
    const AgentId creator = static_cast<AgentId>(tasks[k].task.id >> 16);
    if (index.count(creator) == 0) {
      throw std::invalid_argument("task " + taskName(tasks[k].task.id) + ": creator is not an agent");
    }
    TaskRecord r;
    r.id = tasks[k].task.id;
    r.label = tasks[k].label;
    result.tasks.push_back(r);
    task_index[r.id] = k;
  }

  // 執行中的任務沒完成就被撤銷（核心因電量釋放）
  const auto lost = [&](Vehicle & v) {
      ++result.wrong_accepts;
      ++result.released;
      ++v.record.released;
      ++result.tasks[task_index.at(v.task->task)].failures;
      v.task.reset();
    };

  std::vector<InFlight> flight;
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
      fleet[index.at(static_cast<AgentId>(task.id >> 16))].comm->addLocalTask(task, now);
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
          if (v.manager->onResult(v.task->task, v.task->version, false, now) == ReportCheck::ACCEPTED) {
            ++result.wrong_accepts;
            ++result.tasks[task_index.at(v.task->task)].failures;
          }
          v.task.reset();
        }
        v.state.position = v.state.home;   // 返航的過程不影響 CBBA（已經不參與）
        continue;
      }
      if (!v.task) {
        continue;   // 待命：原地懸停（LandOrLoiter）
      }
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
        if (distance(v.state.position, v.target) <= config.reach_tolerance) {
          v.hovered = 0.0;
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
          lost(v);
        }
        continue;
      }
      if (v.task && v.task->task == active->task && v.task->version == active->version) {
        continue;
      }
      if (v.task) {
        lost(v);   // 換成別的任務：舊的沒做完（不會是搶占，見檔頭）
      }
      const Task & task = v.comm->agent().tasks().at(active->task);
      v.task = *active;
      v.target = task.position;
      v.duration = task.duration_sec;
      v.hovered = -1.0;
      TaskRecord & r = result.tasks[task_index.at(task.id)];
      if (r.first_assigned < 0.0) {
        r.first_assigned = t;
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
  }
  for (Vehicle & v : fleet) {
    v.record.battery_end = v.state.battery;
    result.agents.push_back(v.record);
  }
  return result;
}

}  // namespace cbba_core
