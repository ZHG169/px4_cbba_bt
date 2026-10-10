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
  EXPECT_TRUE(team.swarm.node(50).agent().excluded(f));
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
  // 持有者失聯：標記待確認、照樣鎖著，不重新指派（避免原載具還在執行）
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
