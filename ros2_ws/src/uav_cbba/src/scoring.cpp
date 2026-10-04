#include "uav_cbba/scoring.hpp"

#include <algorithm>
#include <stdexcept>

#include "uav_cbba/energy_model.hpp"

namespace uav_cbba
{

PathEval evaluatePath(
  const AgentState & agent, const std::vector<Task> & ordered_tasks, double now,
  const ScoringParams & params)
{
  const double cruise = agent.cruise_speed > 0.0 ? agent.cruise_speed : params.cruise_speed;
  const double speed = cruise * params.speed_margin;
  if (!(speed > 0.0)) {
    throw std::invalid_argument("planning speed must be positive");
  }

  PathEval eval;
  Vec3 current = agent.position;
  double clock = 0.0;  // 從現在起算的時間

  for (const Task & task : ordered_tasks) {
    LegEval leg;
    leg.task_id = task.id;
    leg.distance = distance(current, task.position);
    leg.arrival = clock + leg.distance / speed;
    leg.completion = leg.arrival + task.duration_sec;
    const double remaining = (task.created + task.deadline_sec) - now;
    leg.remaining_deadline = std::max(remaining, params.min_deadline);
    leg.overdue = leg.arrival > remaining;
    leg.temporal = leg.arrival / leg.remaining_deadline;
    leg.cost = leg.distance * leg.temporal;

    eval.st_cost += leg.cost;
    eval.legs.push_back(leg);
    current = task.position;
    clock = leg.completion;
  }

  const PathEnergy energy = pathEnergy(agent, ordered_tasks);
  eval.path_energy = energy.total;
  eval.battery_after = agent.battery - energy.total;

  if (ordered_tasks.empty()) {
    eval.battery_cost = 0.0;
    eval.total_cost = 0.0;
    return eval;
  }

  eval.battery_cost = batteryCost(agent.battery, energy.total, agent.safety_reserve);
  eval.feasible = std::isfinite(eval.battery_cost);
  eval.total_cost = eval.feasible ?
    eval.st_cost * (1.0 + params.battery_weight * eval.battery_cost) : kInf;
  return eval;
}

double costToScore(double value, double marginal_cost, double cost_ref)
{
  if (!std::isfinite(marginal_cost) || !(value > 0.0)) {
    return 0.0;
  }
  if (!(cost_ref > 0.0)) {
    throw std::invalid_argument("cost_ref must be positive");
  }
  return value / (1.0 + std::max(marginal_cost, 0.0) / cost_ref);
}

Insertion bestInsertion(
  const AgentState & agent, const std::vector<Task> & path, const Task & task, double now,
  const ScoringParams & params)
{
  Insertion best;
  if (!isCompatible(agent.type, task.type) || task.status != TaskStatus::OPEN) {
    return best;
  }

  const PathEval current = evaluatePath(agent, path, now, params);
  if (!current.feasible) {
    return best;
  }

  for (std::size_t pos = 0; pos <= path.size(); ++pos) {
    std::vector<Task> candidate = path;
    candidate.insert(candidate.begin() + static_cast<std::ptrdiff_t>(pos), task);
    PathEval eval = evaluatePath(agent, candidate, now, params);
    if (!eval.feasible) {
      continue;
    }
    const double marginal = eval.total_cost - current.total_cost;
    if (marginal < best.marginal_cost) {
      best.feasible = true;
      best.position = pos;
      best.marginal_cost = marginal;
      best.eval = std::move(eval);
    }
  }

  if (best.feasible) {
    best.score = costToScore(task.value, best.marginal_cost, params.cost_ref);
  }
  return best;
}

}  // namespace uav_cbba
