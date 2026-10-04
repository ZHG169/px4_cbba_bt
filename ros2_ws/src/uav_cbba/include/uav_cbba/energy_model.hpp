// 電池與路徑能量模型（由 energy_model.py 移植，另加回程能量）
#pragma once

#include <vector>

#include "uav_cbba/types.hpp"

namespace uav_cbba
{

// 以固定的每公尺耗能飛行 distance 所需的能量
double travelEnergy(double distance, double energy_per_meter);

// 飛行能量加上任務的固定執行能量
double predictedTaskEnergy(double distance, double energy_per_meter, double execution_energy);

// 消耗 predicted_energy 之後預估的剩餘電量
double remainingEnergy(double battery, double predicted_energy);

// 預估剩餘電量是否仍高於安全存量
bool batteryFeasible(double battery, double predicted_energy, double safety_reserve);

// 路徑用掉「可用電量」的比例；不可行時回傳無限大
double batteryCost(double battery, double predicted_energy, double safety_reserve);

struct EnergyLeg
{
  TaskId task_id{0};
  double distance{0.0};
  double travel_energy{0.0};
  double execution_energy{0.0};
  double cumulative_energy{0.0};
};

struct PathEnergy
{
  double total{0.0};          // 含回程
  double return_energy{0.0};  // 最後一個任務飛回 home 的能量
  std::vector<EnergyLeg> legs;
};

// 依序執行 tasks 並飛回 home 的總能量。
// 執行能量 = duration_sec * hover_energy_per_sec。
// tasks 為空時能量為 0（原地待命，不計回程）。
PathEnergy pathEnergy(const AgentState & agent, const std::vector<Task> & tasks);

}  // namespace uav_cbba
