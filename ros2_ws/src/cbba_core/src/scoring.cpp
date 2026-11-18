#include "cbba_core/scoring.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "cbba_core/capabilities.hpp"
#include "cbba_core/energy_model.hpp"

namespace cbba_core
{

double halfLifeOf(const Task & task, const ScoringParams & params)
{
  if (task.ext.half_life_s > 0) {
    return static_cast<double>(task.ext.half_life_s);
  }
  return std::max(params.half_life_by_priority[priorityLevel(task)], 0.0);
}

double decayedValue(const Task & task, double arrival, const ScoringParams & params)
{
  const double h = halfLifeOf(task, params);
  if (!(h > 0.0)) {
    return task.value;
  }
  return task.value * std::exp2(-std::max(arrival, 0.0) / h);
}

PathEval evaluatePath(
  const AgentState & agent, const std::vector<Task> & ordered_tasks, double now,
  const ScoringParams & params, double head_remaining, TaskId exempt)
{
  const double cruise = agent.cruise_speed > 0.0 ? agent.cruise_speed : params.cruise_speed;
  const double speed = cruise * params.speed_margin;
  if (!(speed > 0.0)) {
    throw std::invalid_argument("planning speed must be positive");
  }

  PathEval eval;
  Vec3 current = agent.position;
  double clock = 0.0;  // 從現在起算的時間
  bool hard_late = false;
  double reserve = agent.safety_reserve;

  for (const Task & task : ordered_tasks) {
    LegEval leg;
    leg.task_id = task.id;
    leg.distance = distance(current, task.position);
    leg.arrival = clock + leg.distance / speed;
    leg.completion = leg.arrival + task.duration_sec;
    const bool head_running = eval.legs.empty() && head_remaining >= 0.0;
    if (head_running) {
      // 已經在執行：完成時間用 BT 的預估
      leg.completion = head_remaining;
      leg.arrival = std::min(leg.arrival, head_remaining);
    } else if (task.success_prob < 1.0) {
      // 期望重試時間：成功前預期失敗 (1 − p) / p 次，每次花「前往＋執行」的時間
      const double p = std::max(task.success_prob, 1e-3);
      const double wasted = (1.0 - p) / p * (leg.distance / speed + task.duration_sec);
      leg.arrival += wasted;
      leg.completion += wasted;
    }
    // 能開始的時刻（執行中、用 BT 剩餘時間的第一個任務不用等）：
    //   release 之前不能開始；依賴的角色等前置角色的里程碑（09）；會合的角色等隊友，最多等到「第一個到的 + sync_window_s」
    //  （逾時照樣開始，和執行端的 evaluateSync 一樣）
    if (!head_running) {
      double start = leg.arrival;
      if (task.ext.release > 0.0) {
        start = std::max(start, task.ext.release - now);
      }
      if (task.coop.ready > 0.0) {
        start = std::max(start, task.coop.ready - now);
      }
      if (task.ext.sync_window_s > 0 && task.coop.sync_last > 0.0) {
        const double first = std::min(start, task.coop.sync_first - now);
        start = std::max(start, std::min(task.coop.sync_last - now, first + task.ext.sync_window_s));
      }
      leg.wait = start - leg.arrival;
      leg.arrival = start;
      leg.completion += leg.wait;
    }
    // 期限只用在硬期限的可行性；軟期限不影響分數（「急」由 half_life 表達）
    if (task.explicit_deadline) {
      leg.overdue = leg.arrival > (task.created + task.deadline_sec) - now;
      if (leg.overdue && task.ext.deadline_hard && task.id != exempt) {
        hard_late = true;   // 硬期限來不及：這條路徑不可行
      }
    }
    // expires（06）：到了任務自動失效、執行中的也撤銷，做不完的白做 → 一律是硬限制（不看 deadline_hard）
    if (task.ext.expires > 0.0 && leg.completion > task.ext.expires - now && task.id != exempt) {
      hard_late = true;
    }
    reserve = std::max(reserve, static_cast<double>(task.ext.min_battery_reserve));
    leg.half_life = halfLifeOf(task, params);
    leg.reward = task.success_prob * decayedValue(task, leg.arrival, params);   // 期望價值

    eval.reward += leg.reward;
    eval.legs.push_back(leg);
    current = task.position;
    clock = leg.completion;
  }

  const PathEnergy energy = pathEnergy(agent, ordered_tasks);
  eval.path_energy = energy.total;
  eval.battery_after = agent.battery - energy.total;
  if (ordered_tasks.empty()) {
    return eval;   // 原地待命：報酬 0、不耗電
  }

  eval.battery_fraction = batteryCost(agent.battery, energy.total, reserve);
  eval.feasible = std::isfinite(eval.battery_fraction) && !hard_late;
  eval.energy_cost = eval.feasible ? params.energy_weight * eval.battery_fraction : kInf;
  eval.score = eval.feasible ? eval.reward - eval.energy_cost : -kInf;
  return eval;
}

Insertion bestInsertion(
  const AgentState & agent, const std::vector<Task> & path, const Task & task, double now,
  const ScoringParams & params, const InsertionLimits & limits)
{
  Insertion best;
  if (fitness(agent, task, params.zones.get()) != Fitness::OK || task.status != TaskStatus::OPEN) {
    return best;
  }

  const PathEval current = evaluatePath(agent, path, now, params, limits.head_remaining, limits.exempt);
  if (!current.feasible) {
    return best;
  }

  for (std::size_t pos = std::min(limits.min_position, path.size()); pos <= path.size(); ++pos) {
    std::vector<Task> candidate = path;
    candidate.insert(candidate.begin() + static_cast<std::ptrdiff_t>(pos), task);
    // 插在最前面時，原本在執行的任務不再是第一個：不用它的剩餘時間
    PathEval eval = evaluatePath(agent, candidate, now, params, pos == 0 ? -1.0 : limits.head_remaining,
        limits.exempt);
    if (!eval.feasible || (limits.allow && !limits.allow(eval, pos, candidate))) {
      continue;
    }
    double gain = eval.score - current.score;
    if (pos == 0 && limits.preempt_drops_head && !path.empty()) {
      // 搶占：被搶占的任務會交回重新競標，比較「拿掉它之後」插入前後的報酬
      std::vector<Task> rest(path.begin() + 1, path.end());
      std::vector<Task> with = rest;
      with.insert(with.begin(), task);
      const PathEval before = evaluatePath(agent, rest, now, params, -1.0, limits.exempt);
      const PathEval after = evaluatePath(agent, with, now, params, -1.0, limits.exempt);
      if (before.feasible && after.feasible) {
        gain = after.score - before.score;
      }
    }
    if (pos < limits.ahead_positions && !(gain > limits.ahead_min_gain)) {
      continue;   // 同級搶占：沒有好到超過遲滯（preempt_margin_pct）就不插到前面
    }
    if (!best.feasible || gain > best.gain) {
      best.feasible = true;
      best.position = pos;
      best.gain = gain;
      best.eval = std::move(eval);
    }
  }

  if (best.feasible) {
    best.score = std::max(best.gain, 0.0);   // 報酬沒有增加就不出價
  }
  return best;
}

}  // namespace cbba_core
