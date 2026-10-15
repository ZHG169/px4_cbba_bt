// 正式指派的流程（AssignmentManager）：BT 接受、搶占、排隊、拒絕、逾時、鎖定，經過模擬網路跑多台
// 對應 2026-10-09 確認的地面端狀態表：
//   閒置且可行 → 可以；巡檢中可中斷 → 先停止巡檢再切換；處理另一個火警 → 不切換（排隊或交給別台）；
//   不可中斷階段 → 暫時不能，到安全中斷點再評估；故障、電量不足 → 不可以
#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include "cbba_core/assignment_manager.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
// 假的 BT（地面端）：依 policy 回覆請求，執行狀態由測試設定
struct FakeBt
{
  enum class Policy { ACCEPT, REJECT_TEMPORARY, REJECT_PERMANENT, SILENT };
  Policy policy{Policy::ACCEPT};
  double delay{0.05};               // 回覆延遲（停止舊導航、交接）
  bool fresh{true};
  std::uint8_t state{0};
  bool preemptible{true};
  double remaining{20.0};
  std::vector<AssignmentRequest> requests;   // 收到的（含 RELEASE）
  std::vector<std::tuple<double, TaskId, std::uint64_t, bool, RejectReason>> replies;
  int assigned_changes{0};
};

class Team
{
public:
  explicit Team(AssignmentConfig config = AssignmentConfig{})
  : config_(config)
  {
    swarm.onTick([this](double now) {tick(now);});
  }

  void addUav(AgentId id, Vec3 pos) {swarm.add(id, pos);}

  void addDog(AgentId id, Vec3 pos, double battery = 100.0, bool require_accept = true)
  {
    swarm.add(id, pos, battery, AgentType::UGV);
    AssignmentConfig c = config_;
    c.require_accept = require_accept;
    managers[id] = std::make_unique<AssignmentManager>(swarm.node(id), c,
        [](int, const std::string &) {});
    bts[id] = FakeBt{};
  }

  AssignmentManager & mgr(AgentId id) {return *managers.at(id);}
  FakeBt & bt(AgentId id) {return bts.at(id);}

  int requestsOf(AgentId id, RequestKind kind, TaskId task) const
  {
    int n = 0;
    for (const auto & r : bts.at(id).requests) {
      n += r.kind == kind && r.task.id == task ? 1 : 0;
    }
    return n;
  }
  const AssignmentRequest * lastRequest(AgentId id, RequestKind kind) const
  {
    const auto & rs = bts.at(id).requests;
    for (auto it = rs.rbegin(); it != rs.rend(); ++it) {
      if (it->kind == kind) {return &*it;}
    }
    return nullptr;
  }

  Swarm swarm;
  std::map<AgentId, std::unique_ptr<AssignmentManager>> managers;
  std::map<AgentId, FakeBt> bts;
  std::vector<AssignmentTiming> timings;

private:
  void tick(double now)
  {
    for (auto & [id, m] : managers) {
      FakeBt & b = bts.at(id);
      ExecInput e;
      e.fresh = b.fresh;
      e.execution_state = b.state;
      e.preemptible = b.preemptible;
      e.remaining_time = b.remaining;
      if (m->active()) {   // BT 照指派執行
        e.has_active = true;
        e.active_task = m->active()->task;
        e.active_version = m->active()->version;
      }
      m->setExec(e, now);
      m->step(now);
      for (const AssignmentRequest & r : m->takeRequests()) {
        b.requests.push_back(r);
        if (r.kind == RequestKind::RELEASE || b.policy == FakeBt::Policy::SILENT) {
          continue;
        }
        const bool accept = b.policy == FakeBt::Policy::ACCEPT;
        const RejectReason reason = accept ? RejectReason::NONE :
          (b.policy == FakeBt::Policy::REJECT_TEMPORARY ? RejectReason::TEMPORARY : RejectReason::PERMANENT);
        b.replies.emplace_back(now + b.delay, r.task.id, r.version, accept, reason);
      }
      for (auto it = b.replies.begin(); it != b.replies.end(); ) {
        if (std::get<0>(*it) <= now) {
          m->onResponse(std::get<1>(*it), std::get<2>(*it), std::get<3>(*it), std::get<4>(*it), "", now);
          it = b.replies.erase(it);
        } else {
          ++it;
        }
      }
      if (m->takeAssignedChanged()) {
        ++b.assigned_changes;
      }
      for (const auto & t : m->takeTimings()) {
        timings.push_back(t);
      }
    }
  }

  AssignmentConfig config_;
};

bool activeIs(const AssignmentManager & m, TaskId t)
{
  return m.active() && m.active()->task == t;
}
}  // namespace

TEST(Assignment, IdleDogAcceptsFire)
{
  // 閒置、可行：請 BT 接受 → 接受後才成為執行中（版本 = 請求的版本），全隊看得到；量測各段
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {5, 20, 0});
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.bt(50).delay = 0.3;   // BT 0.3 s 才回覆：看得到「還沒接受」的階段
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 6, 15, team.swarm.now()), team.swarm.now());
  team.swarm.run(0.7);   // 排第一 0.6 s 後送出請求
  ASSERT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 1);
  const AssignmentRequest * req = team.lastRequest(50, RequestKind::ACTIVATE);
  EXPECT_FALSE(req->preempt);
  EXPECT_FALSE(team.mgr(50).active());   // 還沒接受：不發 assigned_task
  team.swarm.run(0.5);
  ASSERT_TRUE(activeIs(team.mgr(50), f));
  EXPECT_EQ(team.mgr(50).active()->version, req->version);
  EXPECT_EQ(team.swarm.node(1).assignment(f).assignee, 50);
  EXPECT_EQ(team.swarm.node(1).assignment(f).state, AssignState::ACTIVE);
  ASSERT_EQ(team.timings.size(), 1u);
  const AssignmentTiming & t = team.timings.front();
  EXPECT_GE(t.request - t.head, 0.6);          // assign_hold
  EXPECT_NEAR(t.accept - t.request, 0.3, 0.02);
  EXPECT_GE(t.running, t.assigned);
}

TEST(Assignment, PreemptsPatrolOnlyAfterAccept)
{
  // 巡檢中、可以安全中斷：火警請求帶 preempt；BT 接受前 assigned_task 還是巡檢（黑板不覆寫）；
  // 接受後巡檢只撤銷這次指派、任務保留（不送 TASK_CLOSE）
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.swarm.run(2.0);
  const TaskId p = makeTaskId(1, 1);
  Task patrol = groundTask(1, 1, 30, 20, team.swarm.now());
  patrol.type = TaskType::PATROL;
  team.swarm.node(1).addLocalTask(patrol, team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), p));

  team.bt(50).delay = 0.3;   // 停止巡檢、交接要 0.3 s
  const TaskId f = makeTaskId(1, 2);
  team.swarm.node(1).addLocalTask(groundTask(1, 2, 2, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(0.75);
  ASSERT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 1);
  EXPECT_TRUE(team.lastRequest(50, RequestKind::ACTIVATE)->preempt);
  EXPECT_TRUE(activeIs(team.mgr(50), p));   // 還沒接受
  team.swarm.run(0.5);
  EXPECT_TRUE(activeIs(team.mgr(50), f));
  EXPECT_TRUE(team.swarm.node(1).taskOpen(p));
  EXPECT_EQ(team.swarm.node(1).assignment(p).state, AssignState::NONE);
  EXPECT_EQ(team.swarm.node(50).stats().sent.count(PacketType::TASK_CLOSE), 0u);
}

TEST(Assignment, BusyWithFireQueuesOneAndPromotesWithoutNewRequest)
{
  // 處理火警 1 時來了火警 2（同級）：不中斷，排隊（保留，全隊看得到）；火警 1 完成後保留的直接開始，
  // 版本不變、不再問 BT
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.swarm.run(2.0);
  const TaskId f1 = makeTaskId(1, 1);
  const TaskId f2 = makeTaskId(1, 2);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 10, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), f1));
  team.swarm.node(1).addLocalTask(groundTask(1, 2, 2, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  EXPECT_TRUE(activeIs(team.mgr(50), f1));
  EXPECT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f2), 0);
  ASSERT_EQ(team.requestsOf(50, RequestKind::RESERVE, f2), 1);
  ASSERT_TRUE(team.mgr(50).reserved());
  const std::uint64_t reserved_version = team.mgr(50).reserved()->version;
  EXPECT_EQ(team.swarm.node(1).assignment(f2).state, AssignState::RESERVED);
  EXPECT_EQ(team.swarm.node(50).agentStateBody(team.swarm.now()).queued_task, f2);

  ASSERT_EQ(team.mgr(50).onResult(f1, team.mgr(50).active()->version, true, team.swarm.now()),
    ReportCheck::ACCEPTED);
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), f2));
  EXPECT_EQ(team.mgr(50).active()->version, reserved_version);
  EXPECT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f2), 0);
}

TEST(Assignment, SecondFireGoesToIdleDog)
{
  // 狗 50 在處理火警 1（不可中斷）；火警 2 交給閒置的狗 51
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.addDog(51, {0, -20, 0});
  team.swarm.run(2.0);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), makeTaskId(1, 1)));
  team.bt(50).preemptible = false;
  team.bt(50).state = 3;
  team.swarm.node(1).addLocalTask(groundTask(1, 2, 2, -18, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  EXPECT_TRUE(activeIs(team.mgr(51), makeTaskId(1, 2)));
  EXPECT_TRUE(activeIs(team.mgr(50), makeTaskId(1, 1)));
}

TEST(Assignment, NonInterruptibleWaitsThenSwitchesAtSafePoint)
{
  // 巡檢中但在不可中斷階段：火警只排隊；BT 走到可以中斷的地方後，火警請求（preempt）→ 接受後切換
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.swarm.run(2.0);
  const TaskId p = makeTaskId(1, 1);
  Task patrol = groundTask(1, 1, 30, 20, team.swarm.now());
  patrol.type = TaskType::PATROL;
  team.swarm.node(1).addLocalTask(patrol, team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), p));
  team.bt(50).state = 3;   // 不可中斷
  team.bt(50).preemptible = false;
  team.swarm.run(0.2);

  const TaskId f = makeTaskId(1, 2);
  team.swarm.node(1).addLocalTask(groundTask(1, 2, 2, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  EXPECT_TRUE(activeIs(team.mgr(50), p));
  EXPECT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 0);

  team.bt(50).state = 2;   // 走到安全中斷點
  team.bt(50).preemptible = true;
  team.swarm.run(1.5);
  ASSERT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 1);
  EXPECT_TRUE(team.lastRequest(50, RequestKind::ACTIVATE)->preempt);
  EXPECT_TRUE(activeIs(team.mgr(50), f));
  EXPECT_TRUE(team.swarm.node(1).taskOpen(p));
}

TEST(Assignment, FaultOrLowBatteryIsNeverAssigned)
{
  // 故障、電量不足：只有這隻狗也不指派
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.addDog(51, {0, -20, 0}, 20.5);   // 安全存量 20%：做不完
  team.bt(50).state = kExecFault;
  team.swarm.run(2.0);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 0, team.swarm.now()), team.swarm.now());
  team.swarm.run(2.0);
  EXPECT_TRUE(team.bt(50).requests.empty());
  EXPECT_TRUE(team.bt(51).requests.empty());
  EXPECT_FALSE(team.mgr(50).active());
  EXPECT_FALSE(team.mgr(51).active());
}

TEST(Assignment, TemporaryRejectWithdrawsUntilBtStateChanges)
{
  // 暫時拒絕：撤回出價，不會一直重新請求；BT 狀態改變後重新評估，用新的版本（不重用）
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.bt(50).policy = FakeBt::Policy::REJECT_TEMPORARY;
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(5.0);
  ASSERT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 1);
  const std::uint64_t first = team.lastRequest(50, RequestKind::ACTIVATE)->version;
  EXPECT_FALSE(team.mgr(50).active());
  EXPECT_TRUE(team.swarm.node(50).agent().suspended(f));

  team.bt(50).policy = FakeBt::Policy::ACCEPT;
  team.bt(50).state = 1;   // 狀態改變
  team.swarm.run(1.5);
  ASSERT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 2);
  EXPECT_GT(team.lastRequest(50, RequestKind::ACTIVATE)->version, first + 1);   // 中間有一個作廢的版本
  EXPECT_TRUE(activeIs(team.mgr(50), f));
}

TEST(Assignment, PermanentRejectStopsBidding)
{
  // 永久拒絕：不再出價，BT 狀態改變也不會再請求
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.bt(50).policy = FakeBt::Policy::REJECT_PERMANENT;
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(2.0);
  team.bt(50).state = 1;
  team.swarm.run(2.0);
  EXPECT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 1);
  EXPECT_TRUE(team.swarm.node(50).agent().excluded(f, team.swarm.now() + 1e6));
}

TEST(Assignment, TimeoutReleasesAndLateAcceptCannotStart)
{
  // 3 s 沒回覆：當成暫時拒絕，送 RELEASE；之後才到的接受回覆不能啟動任務
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.bt(50).policy = FakeBt::Policy::SILENT;
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.0);
  ASSERT_TRUE(team.mgr(50).pending());
  const std::uint64_t version = team.mgr(50).pending()->version;
  team.swarm.run(3.2);
  EXPECT_FALSE(team.mgr(50).pending());
  ASSERT_EQ(team.requestsOf(50, RequestKind::RELEASE, f), 1);
  EXPECT_EQ(team.lastRequest(50, RequestKind::RELEASE)->version, version);

  team.mgr(50).onResponse(f, version, true, RejectReason::NONE, "", team.swarm.now());
  team.swarm.run(0.3);
  EXPECT_FALSE(team.mgr(50).active());
  EXPECT_NE(team.swarm.node(1).assignment(f).assignee, 50);
}

TEST(Assignment, AcceptedTaskIsLockedUntilRevokedOrClosed)
{
  // 狗 50 已接受的火警：後來出現、比較近的狗 51 不出價；狗 50 執行失敗（撤銷）→ 重新開放，狗 51 接手；
  // 完成的任務永久結束，不會再被競標
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 30, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), f));

  team.addDog(51, {29, 20, 0});   // 就在火點旁邊
  team.swarm.run(3.0);
  EXPECT_EQ(team.requestsOf(51, RequestKind::ACTIVATE, f), 0);
  EXPECT_TRUE(team.swarm.node(51).agent().locked(f));
  EXPECT_TRUE(activeIs(team.mgr(50), f));

  ASSERT_EQ(team.mgr(50).onResult(f, team.mgr(50).active()->version, false, team.swarm.now()),
    ReportCheck::ACCEPTED);   // 執行失敗：任務保留
  team.swarm.run(1.5);
  EXPECT_TRUE(activeIs(team.mgr(51), f));

  ASSERT_EQ(team.mgr(51).onResult(f, team.mgr(51).active()->version, true, team.swarm.now()),
    ReportCheck::ACCEPTED);
  team.swarm.run(2.0);
  EXPECT_FALSE(team.swarm.node(1).taskOpen(f));
  EXPECT_FALSE(team.mgr(50).active());
  EXPECT_FALSE(team.mgr(51).active());
  EXPECT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 1);   // 50 沒有再請求（自己失敗過、任務也結束了）
}

TEST(Assignment, LostHolderStaysLockedPendingConfirmation)
{
  // 持有者失聯：失聯接手的時限（火警 10 s）以內標記待確認、照樣鎖著，不重新指派（避免原載具還在執行）
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.addDog(51, {0, -20, 0});
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 18, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), f));
  team.swarm.setOnline(50, false);
  team.swarm.run(3.0);
  const auto lost = team.swarm.node(1).lockedByLostAgents(team.swarm.now());
  ASSERT_EQ(lost.size(), 1u);
  EXPECT_EQ(lost.front(), std::make_pair(f, AgentId{50}));
  EXPECT_EQ(team.requestsOf(51, RequestKind::ACTIVATE, f), 0);
  EXPECT_TRUE(team.swarm.node(1).taskOpen(f));
}

TEST(Assignment, LostHolderFireTakenOverAfterTimeout)
{
  // 失聯接手（2026-10-11 定案）：火警的持有者失聯（1.5 s）後再 10 s 還沒恢復 → 撤銷鎖定（新版本）、別台接手
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.addDog(51, {0, -20, 0});
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 18, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), f));
  const std::uint64_t v = team.mgr(50).active()->version;

  team.swarm.setOnline(50, false);
  team.swarm.run(1.5 + 8.0);   // 失聯後 8 s：還鎖著
  EXPECT_TRUE(team.swarm.node(51).agent().locked(f));
  EXPECT_EQ(team.requestsOf(51, RequestKind::ACTIVATE, f), 0);
  EXPECT_EQ(team.swarm.node(1).stats().takeovers, 0);

  team.swarm.run(3.0);         // 超過 10 s
  EXPECT_TRUE(activeIs(team.mgr(51), f));
  EXPECT_GT(team.swarm.node(1).assignment(f).version, v);
  EXPECT_EQ(team.swarm.node(1).assignment(f).assignee, 51);
  EXPECT_EQ(team.swarm.node(1).stats().takeovers, 1);
  EXPECT_EQ(team.swarm.node(51).stats().takeovers, 1);
  EXPECT_TRUE(team.swarm.node(1).lockedByLostAgents(team.swarm.now()).empty());
  EXPECT_TRUE(team.mgr(50).active());   // 失聯的那台照樣做手上的（它不知道被接手）
}

TEST(Assignment, LostHolderPatrolWaitsLonger)
{
  // 巡檢寧可等：持有者失聯後 takeover_patrol（30 s）才接手，火警的 10 s 不適用
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.addDog(51, {0, -20, 0});
  team.swarm.run(2.0);
  const TaskId p = makeTaskId(1, 1);
  Task patrol = groundTask(1, 1, 2, 18, team.swarm.now());
  patrol.type = TaskType::PATROL;
  team.swarm.node(1).addLocalTask(patrol, team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), p));

  team.swarm.setOnline(50, false);
  team.swarm.run(1.5 + 25.0);
  EXPECT_TRUE(team.swarm.node(51).agent().locked(p));
  EXPECT_EQ(team.requestsOf(51, RequestKind::ACTIVATE, p), 0);
  team.swarm.run(7.0);
  EXPECT_TRUE(activeIs(team.mgr(51), p));
}

TEST(Assignment, LostHolderBackBeforeTimeoutKeepsTask)
{
  // 時限內恢復連線（短暫斷線）：不接手，任務還是原持有者的
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.addDog(51, {0, -20, 0});
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 18, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), f));
  const std::uint64_t v = team.mgr(50).active()->version;

  team.swarm.setOnline(50, false);
  team.swarm.run(6.0);
  team.swarm.setOnline(50, true);
  team.swarm.run(12.0);
  EXPECT_TRUE(activeIs(team.mgr(50), f));
  EXPECT_EQ(team.mgr(50).active()->version, v);
  EXPECT_EQ(team.requestsOf(51, RequestKind::ACTIVATE, f), 0);
  EXPECT_EQ(team.swarm.node(1).stats().takeovers, 0);
  EXPECT_TRUE(team.swarm.consistent());
}

TEST(Assignment, LostHolderBackAfterTakeoverIsSuperseded)
{
  // 接手之後舊持有者恢復連線：它的指派被新版本取代（撤下），它的舊版本回報被拒，不會把別台在做的任務關掉
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.addDog(51, {0, -20, 0});
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 18, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), f));
  const std::uint64_t old = team.mgr(50).active()->version;

  team.swarm.setOnline(50, false);
  team.swarm.run(13.0);
  ASSERT_TRUE(activeIs(team.mgr(51), f));

  team.swarm.setOnline(50, true);
  team.swarm.run(1.0);
  EXPECT_FALSE(team.mgr(50).active());
  EXPECT_EQ(team.swarm.node(50).reportResult(f, true, old, team.swarm.now()), ReportCheck::STALE);
  team.swarm.run(2.0);
  EXPECT_TRUE(activeIs(team.mgr(51), f));
  EXPECT_TRUE(team.swarm.node(1).taskOpen(f));
  EXPECT_TRUE(team.swarm.consistent());
}

TEST(Assignment, PatrollingDogNearFireBeatsIdleFarDog)
{
  // 2026-10-12：巡檢中的 V60 離火點 6 m、另一隻閒著但在 45 m 外：近的那隻要得標、中斷巡檢前往火點
  // （巡檢交回重新競標）。修正前近的那隻的出價被 bundle 上限壓在巡檢分數以下、評分又算了「延後巡檢」的損失，
  // 輸給遠的那隻
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 0, 0}, 100.0, false);
  team.addDog(51, {-40, 0, 0}, 100.0, false);
  team.swarm.run(2.0);
  const TaskId patrol = makeTaskId(50, 1);
  Task p = groundTask(50, 1, 30, 0, team.swarm.now());
  p.type = TaskType::PATROL;
  p.value = 80.0;
  team.swarm.node(50).addLocalTask(p, team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), patrol));
  const std::uint64_t v = team.mgr(50).active()->version;

  const TaskId fire = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 5, 4, team.swarm.now()), team.swarm.now());
  team.swarm.run(2.0);
  EXPECT_TRUE(activeIs(team.mgr(50), fire));                          // 近的那隻中斷巡檢、前往火點
  EXPECT_FALSE(team.mgr(51).active() && team.mgr(51).active()->task == fire);
  EXPECT_GT(team.swarm.node(1).assignment(patrol).version, v);        // 巡檢這次的指派撤銷
  EXPECT_TRUE(team.swarm.node(1).taskOpen(patrol));                   // 巡檢還在（交回重新競標）
  EXPECT_TRUE(team.swarm.consistent());
}

TEST(Assignment, HolderThatStopsParticipatingReleasesItsLock)
{
  // 執行中的持有者停止參與（PX4 斷線、降落）：它自己撤銷，撤銷要傳到別台 → 解除鎖定、別台接手。
  // 2026-10-11 SITL 測試發現：停止參與的節點的 CBBA_STATE 不帶紀錄，撤銷傳不出去，任務一直鎖著沒人接
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.addDog(51, {0, -20, 0});
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 18, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.5);
  ASSERT_TRUE(activeIs(team.mgr(50), f));
  const std::uint64_t v = team.mgr(50).active()->version;

  team.swarm.node(50).setParticipating(false, team.swarm.now());
  team.swarm.run(2.0);
  EXPECT_FALSE(team.mgr(50).active());
  EXPECT_GT(team.swarm.node(1).assignment(f).version, v);          // 收到撤銷（新版本）
  EXPECT_FALSE(team.swarm.node(51).agent().locked(f));
  EXPECT_TRUE(team.swarm.node(1).lockedByLostAgents(team.swarm.now()).empty());
  EXPECT_TRUE(activeIs(team.mgr(51), f));                           // 別台接手
  EXPECT_TRUE(team.swarm.consistent());
}

TEST(Assignment, TemporaryFailureRetriesAfterExclusion)
{
  // TaskResult.reason 不是 PERMANENT（2026-10-12）：失敗後 60 s 內不出價，之後同一隻狗重新接回；
  // PERMANENT 則一直不接
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0}, 100.0, false);
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  const TaskId g = makeTaskId(1, 2);
  const auto longTask = [&team](std::uint16_t seq, double x) {
      Task t = groundTask(1, seq, x, 20, team.swarm.now());
      t.deadline_sec = 1000.0;   // 測試要跑 100 多秒：期限拉長，不讓逾期影響出價
      return t;
    };
  team.swarm.node(1).addLocalTask(longTask(1, 2), team.swarm.now());
  team.swarm.run(1.0);
  ASSERT_TRUE(activeIs(team.mgr(50), f));
  ASSERT_EQ(team.mgr(50).onResult(f, team.mgr(50).active()->version, false, team.swarm.now(), 60.0),
    ReportCheck::ACCEPTED);
  team.swarm.run(30.0);
  EXPECT_FALSE(team.mgr(50).active());          // 排除中
  team.swarm.run(32.0);
  EXPECT_TRUE(activeIs(team.mgr(50), f));       // 60 s 後重新接回

  ASSERT_EQ(team.mgr(50).onResult(f, team.mgr(50).active()->version, false, team.swarm.now()),
    ReportCheck::ACCEPTED);                     // 永久
  team.swarm.node(1).addLocalTask(longTask(2, 3), team.swarm.now());
  team.swarm.run(70.0);
  EXPECT_TRUE(activeIs(team.mgr(50), g));       // 別的任務照接
  EXPECT_TRUE(team.swarm.node(50).agent().excluded(f, team.swarm.now()));
}

TEST(Assignment, WithoutAcceptStillVersionedAndChecked)
{
  // require_accept = false：跳過 BT 握手，當下就是執行中；版本與回報檢查照常
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0}, 100.0, false);
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(1.0);
  ASSERT_TRUE(activeIs(team.mgr(50), f));
  EXPECT_TRUE(team.bt(50).requests.empty());
  const std::uint64_t v = team.mgr(50).active()->version;
  EXPECT_EQ(team.mgr(50).onResult(f, v + 1, true, team.swarm.now()), ReportCheck::STALE);
  EXPECT_EQ(team.mgr(50).onResult(f, v, true, team.swarm.now()), ReportCheck::ACCEPTED);
}

TEST(Assignment, FaultAfterTimeoutDoesNotResumeIntoRequest)
{
  // 逾時被撤回的任務：BT 接著變成故障（狀態改變）時，不能因為「狀態改變 → 恢復出價」而又請求
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 20, 0});
  team.bt(50).policy = FakeBt::Policy::SILENT;
  team.swarm.run(2.0);
  const TaskId f = makeTaskId(1, 1);
  team.swarm.node(1).addLocalTask(groundTask(1, 1, 2, 20, team.swarm.now()), team.swarm.now());
  team.swarm.run(4.0);   // 請求、逾時
  ASSERT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 1);
  team.bt(50).policy = FakeBt::Policy::ACCEPT;
  team.bt(50).state = kExecFault;
  team.swarm.run(2.0);
  EXPECT_EQ(team.requestsOf(50, RequestKind::ACTIVATE, f), 1);
  EXPECT_FALSE(team.mgr(50).active());
  EXPECT_TRUE(team.swarm.node(50).agent().path().empty());
}

TEST(Assignment, OverrunningActiveStopsQueueingButStillPreemptable)
{
  // 執行中的巡檢超過預估（前往＋停留＋overrun_margin）還沒完成（例如無人機懸停監控火點、等地面處置）：
  // 排在它後面的任務撤回、交給別台；更高優先級的火警照樣可以搶占（2026-10-13 SITL 的 U6）
  Team team;
  team.addUav(1, {0, 0, 5});
  team.addDog(50, {0, 0, 0}, 100.0, false);
  team.addDog(51, {30, 0, 0}, 100.0, false);
  team.bt(50).fresh = false;   // 節點 2026-10-12 起沒有 BT 的執行狀態
  team.bt(51).fresh = false;
  team.swarm.run(2.0);
  const auto patrol = [&](std::uint16_t seq, double x) {
      Task t = newTask(1, seq, x, 0.0, team.swarm.now(), 80.0, 300.0);
      t.type = TaskType::PATROL;
      t.position.z = 0.0;
      team.swarm.node(1).addLocalTask(t, team.swarm.now());
    };
  patrol(1, 2.0);
  team.swarm.run(1.0);
  ASSERT_TRUE(activeIs(team.mgr(50), makeTaskId(1, 1)));
  patrol(2, 4.0);
  team.swarm.run(2.0);
  const auto & path50 = team.swarm.node(50).agent().path();
  ASSERT_EQ(path50.size(), 2u);   // 還沒超過預估：排在執行中的後面
  EXPECT_EQ(path50[1], makeTaskId(1, 2));
  team.swarm.run(18.0);           // 2 m／0.8 m/s＋5 s＋10 s 之後
  EXPECT_EQ(team.swarm.node(50).agent().path().size(), 1u);
  EXPECT_TRUE(activeIs(team.mgr(51), makeTaskId(1, 2)));
  // 火警：照樣搶占超過預估的巡檢
  team.swarm.node(1).addLocalTask(groundTask(1, 3, 1.0, 0.0, team.swarm.now()), team.swarm.now());
  team.swarm.run(2.0);
  EXPECT_TRUE(activeIs(team.mgr(50), makeTaskId(1, 3)));
}
