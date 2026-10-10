// 多台載具在有丟包、延遲、亂序的網路上協商（對應週計畫第 5 週的預先驗證）
//
// 模擬的網路在 network_sim.hpp：每則訊息獨立丟包，延遲 50 +/- 30 ms（因此會亂序）。
#include <gtest/gtest.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "cbba_core/network_sim.hpp"

using namespace cbba_core;

namespace
{
struct SimConfig
{
  int agents{3};
  int tasks{6};
  double loss{0.0};
  double timeout{5.0};
};

NegotiationResult runTrial(const SimConfig & cfg, std::uint32_t seed)
{
  std::mt19937 rng(seed);
  const Scenario scenario = randomScenario(cfg.agents, cfg.tasks, rng);
  std::vector<CbbaAgent> agents;
  for (const AgentState & state : scenario.agents) {
    agents.emplace_back(state, ScoringParams{});
  }
  NetworkConfig net;
  net.loss = cfg.loss;
  net.timeout = cfg.timeout;
  return negotiate(agents, scenario.tasks, net, rng);
}

struct Summary
{
  double rate{0.0};
  double mean_time{0.0};
  double p95_time{0.0};
  double mean_broadcasts{0.0};
  double mean_kbytes{0.0};
  int duplicates{0};
  int inconsistent{0};
};

Summary runBatch(const SimConfig & cfg, int trials)
{
  Summary s;
  std::vector<double> times;
  for (int i = 0; i < trials; ++i) {
    const NegotiationResult r = runTrial(cfg, 1000u + static_cast<std::uint32_t>(i));
    s.mean_broadcasts += r.broadcasts;
    s.mean_kbytes += r.bytes / 1000.0;
    if (r.converged) {
      times.push_back(r.time);
      s.duplicates += r.duplicates;
      s.inconsistent += r.inconsistent;
    }
  }
  s.rate = static_cast<double>(times.size()) / trials;
  s.mean_broadcasts /= trials;
  s.mean_kbytes /= trials;
  if (!times.empty()) {
    std::sort(times.begin(), times.end());
    double sum = 0.0;
    for (double t : times) {sum += t;}
    s.mean_time = sum / static_cast<double>(times.size());
    s.p95_time = times[static_cast<std::size_t>(0.95 * static_cast<double>(times.size() - 1))];
  }
  return s;
}
}  // namespace

TEST(LossyNetwork, ConvergenceUnderPacketLoss)
{
  const int trials = 200;
  std::cout << "\n  3 UAV, 6 tasks, delay 50+/-30 ms, " << trials << " trials per level\n"
            << "  loss   converged   mean_time   p95_time   broadcasts   kB\n";
  for (double loss : {0.0, 0.1, 0.3, 0.5, 0.7}) {
    SimConfig cfg;
    cfg.loss = loss;
    const Summary s = runBatch(cfg, trials);
    std::cout << std::fixed << std::setprecision(0)
              << "  " << std::setw(3) << loss * 100 << "%   "
              << std::setprecision(1) << std::setw(7) << s.rate * 100 << "%   "
              << std::setprecision(2) << std::setw(7) << s.mean_time << " s   "
              << std::setw(6) << s.p95_time << " s   "
              << std::setprecision(1) << std::setw(8) << s.mean_broadcasts << "   "
              << std::setw(5) << s.mean_kbytes << "\n";

    EXPECT_EQ(s.duplicates, 0) << "loss " << loss;
    EXPECT_EQ(s.inconsistent, 0) << "loss " << loss;
    if (loss <= 0.3) {
      // 週計畫的驗收指標：30% 丟包下，5 秒內收斂率 >= 95%
      EXPECT_GE(s.rate, 0.95) << "loss " << loss;
    }
  }
}

TEST(LossyNetwork, MoreAgentsAndTasksStillConverge)
{
  SimConfig cfg;
  cfg.agents = 5;
  cfg.tasks = 12;
  cfg.loss = 0.3;
  const Summary s = runBatch(cfg, 100);
  std::cout << "  5 UAV, 12 tasks, 30% loss: converged " << s.rate * 100 << "%, mean "
            << s.mean_time << " s, p95 " << s.p95_time << " s\n";
  EXPECT_EQ(s.duplicates, 0);
  EXPECT_GE(s.rate, 0.95);
}

TEST(LossyNetwork, TotalBlackoutFallsBackToLocalGreedy)
{
  // 100% 丟包：每台只依自己的分數行動，不會卡住
  SimConfig cfg;
  cfg.loss = 1.0;
  cfg.timeout = 1.0;
  const NegotiationResult r = runTrial(cfg, 42);
  EXPECT_FALSE(r.converged);       // 彼此聽不到，各自認為自己得標
  EXPECT_GT(r.duplicates, 0);      // 沒有通訊時重複認領是預期的結果
  EXPECT_LT(r.unassigned, cfg.tasks);
}
