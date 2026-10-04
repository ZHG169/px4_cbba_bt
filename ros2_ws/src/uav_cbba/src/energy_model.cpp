#include "uav_cbba/energy_model.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace uav_cbba
{

namespace
{
double nonNegative(const char * name, double value)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(name) + " must be a finite non-negative number");
  }
  return value;
}
}  // namespace

double travelEnergy(double distance, double energy_per_meter)
{
  return nonNegative("distance", distance) * nonNegative("energy_per_meter", energy_per_meter);
}

double predictedTaskEnergy(double distance, double energy_per_meter, double execution_energy)
{
  return travelEnergy(distance, energy_per_meter) +
         nonNegative("execution_energy", execution_energy);
}

double remainingEnergy(double battery, double predicted_energy)
{
  return nonNegative("battery", battery) - nonNegative("predicted_energy", predicted_energy);
}

bool batteryFeasible(double battery, double predicted_energy, double safety_reserve)
{
  nonNegative("safety_reserve", safety_reserve);
  return remainingEnergy(battery, predicted_energy) >= safety_reserve;
}

double batteryCost(double battery, double predicted_energy, double safety_reserve)
{
  nonNegative("battery", battery);
  nonNegative("predicted_energy", predicted_energy);
  nonNegative("safety_reserve", safety_reserve);
  const double usable = battery - safety_reserve;
  if (usable <= 0.0 || predicted_energy > usable) {
    return kInf;
  }
  return predicted_energy / usable;
}

PathEnergy pathEnergy(const AgentState & agent, const std::vector<Task> & tasks)
{
  nonNegative("energy_per_meter", agent.energy_per_meter);
  nonNegative("hover_energy_per_sec", agent.hover_energy_per_sec);

  PathEnergy result;
  Vec3 current = agent.position;
  for (const Task & task : tasks) {
    EnergyLeg leg;
    leg.task_id = task.id;
    leg.distance = distance(current, task.position);
    leg.travel_energy = travelEnergy(leg.distance, agent.energy_per_meter);
    leg.execution_energy =
      nonNegative("duration_sec", task.duration_sec) * agent.hover_energy_per_sec;
    result.total += leg.travel_energy + leg.execution_energy;
    leg.cumulative_energy = result.total;
    result.legs.push_back(leg);
    current = task.position;
  }
  if (!tasks.empty()) {
    result.return_energy = travelEnergy(distance(current, agent.home), agent.energy_per_meter);
    result.total += result.return_energy;
  }
  return result;
}

}  // namespace uav_cbba
