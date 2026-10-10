// 在同一個程式裡模擬多台載具經過不可靠的網路協商（測試與離線分析用，不是機上程式）
//
// 每則訊息對每個接收者獨立以 loss 的機率丟掉，延遲在 [delay_min, delay_max] 之間均勻分布，
// 因此也會亂序。收斂的定義和介面規格相同：
//   所有載具對每個任務記錄的得標者相同，並連續 stable 秒不變。
#pragma once

#include <random>
#include <string>
#include <vector>

#include "cbba_core/cbba_agent.hpp"

namespace cbba_core
{

struct NetworkConfig
{
  double loss{0.0};        // 丟包率 0~1
  double delay_min{0.02};  // 延遲 50 +/- 30 ms
  double delay_max{0.08};
  double period{0.2};      // 週期重送
  double min_gap{0.02};    // 立即發送的最短間隔
  double timeout{5.0};     // 收斂的時限
  double stable{0.5};      // 連續多久不變才算收斂
  double dt{0.01};         // 模擬的時間步長
};

struct TimelinePoint
{
  double time{0.0};
  int disagreements{0};  // 有幾個任務各載具記錄的得標者不一致
  int broadcasts{0};     // 累計的廣播次數
};

struct NegotiationResult
{
  bool converged{false};
  double time{0.0};       // 收斂時間
  double elapsed{0.0};    // 模擬跑了多久
  int broadcasts{0};
  double bytes{0.0};      // 估計的流量（含封包標頭）
  int duplicates{0};      // 同一個任務出現在兩台的 bundle 裡
  int inconsistent{0};    // 得標者和實際持有者不一致
  int unassigned{0};      // 沒有任何載具持有的任務
  std::vector<TimelinePoint> timeline;  // record_timeline 為 true 時才有
};

// 所有載具先收到全部任務（/swarm/tasks 是 reliable），再經由模擬的網路交換出價訊息。
NegotiationResult negotiate(
  std::vector<CbbaAgent> & agents, const std::vector<Task> & tasks, const NetworkConfig & cfg,
  std::mt19937 & rng, bool record_timeline = false);

// ---------------------------------------------------------------------------
// 場景（測資）
// ---------------------------------------------------------------------------
struct Scenario
{
  std::vector<AgentState> agents;
  std::vector<Task> tasks;
};

// 讀取場景檔（CSV）。格式見 scenarios/fire_demo.csv。讀不到或格式錯誤時丟出 std::runtime_error。
Scenario loadScenario(const std::string & path);

// 隨機場景：UAV 與 AIR_RECON 任務散布在 100 m x 100 m 的範圍內
Scenario randomScenario(int num_agents, int num_tasks, std::mt19937 & rng);

const char * toString(AgentType type);
const char * toString(TaskType type);

}  // namespace cbba_core
