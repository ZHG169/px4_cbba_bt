// 單一載具的 CBBA 狀態與共識邏輯（不依賴 ROS，介面規格 v0.1）
//
// 每台載具只根據自己收到的訊息更新自己的表：
//   onTask()     收到 /swarm/tasks 的任務
//   onMessage()  收到 /swarm/cbba 的出價訊息（17 條消解規則 + 連鎖退標 + 重新出價）
//   reevaluate() 執行中依目前的位置、時間、電量重新計算手上任務的分數
//   makeMessage() 產生要廣播的訊息
//   setActive()  是否參與出價（例如還沒起飛時不參與）
//   releaseAgent() / releaseStale()  某台停止參與或失聯：把它得標的任務設為無人，重新出價
//   excludeTask() 自己做這個任務失敗：釋放並不再對它出價（交回給其他載具）
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "uav_cbba/scoring.hpp"
#include "uav_cbba/types.hpp"

namespace uav_cbba
{

struct Bid
{
  TaskId task_id{0};
  double score{0.0};          // y
  AgentId winner{kNoAgent};   // z
};

struct CbbaMessage
{
  double stamp{0.0};          // 送出時間
  AgentId sender{kNoAgent};
  AgentType sender_type{AgentType::UAV};
  std::uint32_t seq{0};
  std::vector<Bid> bids;                 // 每個 OPEN 任務一筆
  std::map<AgentId, double> stamps;      // 傳送者最後一次取得各載具資訊的時間 (s)
};

enum class RuleAction { Leave, Update, Reset };

struct RuleInput
{
  AgentId me{kNoAgent};             // 接收者 i
  AgentId sender{kNoAgent};         // 傳送者 k
  AgentId sender_winner{kNoAgent};  // z_kj
  double sender_score{0.0};         // y_kj
  AgentId my_winner{kNoAgent};      // z_ij
  double my_score{0.0};             // y_ij
  const std::map<AgentId, double> * sender_stamps{nullptr};  // s_k
  const std::map<AgentId, double> * my_stamps{nullptr};      // s_i（更新前）
  double epsilon{1e-3};
};

struct RuleResult
{
  RuleAction action{RuleAction::Leave};
  int rule{0};  // 1~17，對應 Choi 2009 Table 1
};

// 消解規則表（純函數，供測試案例檔驗證）
RuleResult resolveConflict(const RuleInput & in);

class CbbaAgent
{
public:
  CbbaAgent(const AgentState & state, const ScoringParams & params);

  void setState(const AgentState & state) {state_ = state;}
  const AgentState & state() const {return state_;}
  const ScoringParams & params() const {return params_;}

  // 收到任務（新任務或狀態改變）。回傳 true 表示自己的表有改變，應立即廣播。
  bool onTask(const Task & task, double now);

  // 收到出價訊息。回傳 true 表示自己的表有改變，應立即廣播。
  bool onMessage(const CbbaMessage & msg, double now);

  // 依目前狀態重新計算手上任務的分數；電量不足時釋放任務。回傳 true 表示有改變。
  bool reevaluate(double now);

  // 自己完成了任務：標記為 DONE 並從 bundle 移除。回傳要在 /swarm/tasks 發布的任務。
  std::optional<Task> completeTask(TaskId id, double now);

  // 產生要廣播的訊息（seq 會遞增）
  CbbaMessage makeMessage(double now);

  // 超過 timeout 秒沒收到訊息的鄰居
  std::vector<AgentId> lostNeighbors(double now, double timeout) const;

  // 是否參與出價。停止時釋放自己所有的任務；恢復時重新出價。回傳 true 表示有改變。
  bool setActive(bool active, double now);
  bool active() const {return active_;}

  // 把 agent 得標的任務設為無人得標，再重新出價。回傳 true 表示有改變。
  bool releaseAgent(AgentId agent, double now);

  // 時間戳 s 超過 timeout 秒沒更新的載具視為失聯，釋放它得標的任務。
  // s 會經由鄰居的訊息傳遞，所以多跳之外、但仍有人聽得到的載具不會被誤判。
  bool releaseStale(double now, double timeout);

  // 自己做這個任務失敗：從 bundle 釋放（連同之後加入的），之後不再對它出價。
  bool excludeTask(TaskId id, double now);
  bool excluded(TaskId id) const {return excluded_.count(id) > 0;}

  const std::map<AgentId, double> & stamps() const {return stamps_;}

  const std::vector<TaskId> & bundle() const {return bundle_;}  // 得標加入的順序
  const std::vector<TaskId> & path() const {return path_;}      // 實際執行的順序
  const std::map<TaskId, Task> & tasks() const {return tasks_;}
  AgentId winner(TaskId id) const;
  double score(TaskId id) const;

private:
  bool isOpen(TaskId id) const;
  std::vector<Task> pathTasks(const std::vector<TaskId> & ids) const;
  bool releaseFrom(std::size_t bundle_index);
  bool releaseLost();
  bool buildBundle(double now);

  AgentState state_;
  ScoringParams params_;

  std::map<TaskId, Task> tasks_;
  std::map<TaskId, double> y_;         // 已知的最高分
  std::map<TaskId, AgentId> z_;        // 已知的得標者
  std::map<AgentId, double> stamps_;   // s：最後一次取得各載具資訊的時間
  std::map<AgentId, double> last_heard_;
  std::map<AgentId, std::uint32_t> last_seq_;
  std::vector<TaskId> bundle_;
  std::vector<TaskId> path_;
  std::set<TaskId> excluded_;
  bool active_{true};
  std::uint32_t seq_{0};
};

}  // namespace uav_cbba
