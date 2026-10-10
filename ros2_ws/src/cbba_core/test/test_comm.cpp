// CbbaComm（協定版本 2）：經過模擬的廣播網路（誰聽得到誰、掉包、延遲）跑完整的機間協定
#include <gtest/gtest.h>

#include <algorithm>
#include <set>

#include "swarm_harness.hpp"

using namespace cbba_test;


TEST(TaskName, Hex)
{
  EXPECT_EQ(taskName(makeTaskId(3, 5)), "00030005");
  EXPECT_EQ(taskName(makeTaskId(50, 1)), "00320001");
}

TEST(StampMs, NearNow)
{
  const double now = 1.7e9 + 123.456;
  EXPECT_NEAR(fromStampMs(toStampMs(now - 2.5), now), now - 2.5, 1e-3);
  EXPECT_NEAR(fromStampMs(toStampMs(now + 0.01), now), now, 1e-3);   // 對方時鐘稍快
}

TEST(CbbaComm, TasksSpreadByTaskId)
{
  // 各台建立的任務以 task_id 傳遍全隊，不需要切任務編號
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.run(2.0);   // 過了 join_wait

  const double now = swarm.now();
  ASSERT_TRUE(swarm.node(1).addLocalTask(newTask(1, 1, 1, 1, now), now));
  ASSERT_TRUE(swarm.node(1).addLocalTask(newTask(1, 2, 2, 2, now), now));
  ASSERT_TRUE(swarm.node(3).addLocalTask(newTask(3, 1, 19, 1, now), now));
  EXPECT_FALSE(swarm.node(3).addLocalTask(newTask(3, 1, 19, 1, now), now));   // 重複
  EXPECT_FALSE(swarm.node(2).addLocalTask(newTask(1, 9, 0, 0, now), now));    // 建立者不是自己
  swarm.run(2.0);

  EXPECT_TRUE(swarm.consistent());
  for (AgentId a : {1, 2, 3}) {
    EXPECT_EQ(swarm.node(a).taskCount(), 3u);
    EXPECT_TRUE(swarm.node(a).converged(swarm.now())) << a;
    EXPECT_EQ(swarm.node(a).roundKey(), swarm.node(1).roundKey());
    EXPECT_EQ(swarm.node(a).stats().task_conflicts, 0);
    EXPECT_EQ(swarm.node(a).stats().malformed, 0);
  }
  // 截止時刻在各台相同（created 是各自收到的時間）
  const auto deadline = [&](AgentId a) {
      const Task & t = swarm.node(a).agent().tasks().at(makeTaskId(3, 1));
      return t.created + t.deadline_sec;
    };
  EXPECT_NEAR(deadline(1), deadline(3), 1e-3);
  EXPECT_NEAR(deadline(2), deadline(3), 1e-3);
  EXPECT_EQ(swarm.node(3).currentTask()->id, makeTaskId(3, 1));   // 離 uav3 最近
}

TEST(CbbaComm, CbbaStateFields)
{
  // 紀錄只放自己認得、進行中的任務；s 帶機號
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {30, 0, 5});
  swarm.run(2.0);
  swarm.node(2).addLocalTask(newTask(2, 1, 28, 0, swarm.now()), swarm.now());
  swarm.run(1.0);

  const auto b = swarm.node(1).cbbaStateBody(swarm.now());
  EXPECT_TRUE(b.participating);
  ASSERT_EQ(b.records.size(), 1u);
  EXPECT_EQ(b.records[0].task_id, makeTaskId(2, 1));
  EXPECT_EQ(b.records[0].winner_id, 2);
  EXPECT_GT(b.records[0].winning_bid, 0.0f);
  std::set<std::uint16_t> stamped;
  for (const auto & s : b.stamps) {
    stamped.insert(s.agent_id);
    EXPECT_NE(s.stamp_ms, 0u);
  }
  EXPECT_EQ(stamped, (std::set<std::uint16_t>{1, 2}));
}

TEST(CbbaComm, AgentStateFields)
{
  // AGENT_STATE 帶真實的 armed／offboard／landed（飛行狀態有效時），以及鄰居、路徑、目前的任務
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {30, 0, 5});
  VehicleStatus st;
  st.telemetry_ok = true;
  st.flight_state_valid = true;
  st.armed = true;
  st.offboard = true;
  swarm.node(2).setStatus(st);
  swarm.run(2.0);
  swarm.node(2).addLocalTask(newTask(2, 1, 28, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  const std::uint64_t version = activate(swarm.node(2), makeTaskId(2, 1));
  swarm.run(1.0);

  const auto b = swarm.node(2).agentStateBody(swarm.now());
  EXPECT_TRUE(b.has_active);
  EXPECT_EQ(b.active_version, version);
  EXPECT_TRUE(b.flags.armed);
  EXPECT_TRUE(b.flags.offboard);
  EXPECT_FALSE(b.flags.landed);
  EXPECT_EQ(b.path, std::vector<std::uint32_t>{makeTaskId(2, 1)});
  EXPECT_EQ(b.active_task, makeTaskId(2, 1));
  EXPECT_FALSE(b.has_queued);
  EXPECT_EQ(b.neighbors, std::vector<std::uint16_t>{1});
  // 鄰居收到的
  const NeighborInfo & n = swarm.node(1).neighbors().at(2);
  EXPECT_TRUE(n.has_status);
  EXPECT_TRUE(n.flags.armed);
  EXPECT_TRUE(n.flags.offboard);
  ASSERT_TRUE(n.active);
  EXPECT_EQ(n.active->first, makeTaskId(2, 1));
  EXPECT_EQ(n.active->second, version);
  EXPECT_EQ(n.neighbors, std::set<AgentId>{1});

  // 離開 offboard、降落：跟著變
  st.offboard = false;
  st.landed = true;
  swarm.node(2).setStatus(st);
  swarm.run(1.0);
  EXPECT_FALSE(swarm.node(1).neighbors().at(2).flags.offboard);
  EXPECT_TRUE(swarm.node(1).neighbors().at(2).flags.landed);
  // 飛行狀態逾時：flight_state_valid = 0，armed 等不送
  st.flight_state_valid = false;
  swarm.node(2).setStatus(st);
  swarm.run(1.0);
  EXPECT_FALSE(swarm.node(1).neighbors().at(2).flags.flight_state_valid);
  EXPECT_FALSE(swarm.node(1).neighbors().at(2).flags.armed);
}

TEST(CbbaComm, UnknownTaskDoesNotEraseWinner)
{
  // 紀錄裡沒有的任務（對方不認得）不影響；「認得但沒人得標」依規則 16 會清掉
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  swarm.node(2).addLocalTask(newTask(2, 1, 11, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  const TaskId t = makeTaskId(2, 1);
  ASSERT_EQ(swarm.node(1).agent().winner(t), 2);
  swarm.setOnline(2, false);   // 0.3 s 沒聽到 uav2（還不到失聯），讓 uav3 對 uav2 的資訊比較新
  swarm.run(0.3);

  std::uint32_t seq = 0;
  const auto inject = [&](std::vector<wire::Record> records) {
      wire::CbbaStateBody b;
      b.participating = true;
      b.snapshot_id = static_cast<std::uint16_t>(seq);
      b.stamps = {{2, toStampMs(swarm.now())}, {3, toStampMs(swarm.now())}};
      b.records = std::move(records);
      wire::Header h;
      h.sender_id = 3;
      h.session_id = 0x33;
      h.sequence = ++seq;
      swarm.node(1).receive(wire::encodeCbbaState(h, b), swarm.now());
    };
  inject({});                       // 不認得：沒有紀錄
  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);
  swarm.run(0.05);
  inject({{t, 0, 0.0f}});           // 對照：認得但沒人得標
  EXPECT_NE(swarm.node(1).agent().winner(t), 2);
}

TEST(CbbaComm, MultiHopRelayAndCompletion)
{
  // 1 — 2 — 3：1 和 3 互相聽不到
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {30, 0, 5});
  swarm.add(3, {60, 0, 5});
  swarm.cut(1, 3);
  swarm.run(2.0);
  swarm.node(3).addLocalTask(newTask(3, 1, 0, 2, swarm.now()), swarm.now());    // 在 uav1 旁邊
  swarm.node(1).addLocalTask(newTask(1, 1, 61, 0, swarm.now()), swarm.now());   // 在 uav3 旁邊
  swarm.run(3.0);
  EXPECT_TRUE(swarm.consistent());
  EXPECT_EQ(swarm.node(3).agent().winner(makeTaskId(3, 1)), 1);
  EXPECT_EQ(swarm.node(1).agent().winner(makeTaskId(1, 1)), 3);
  EXPECT_GT(swarm.node(2).stats().relayed, 0);

  // uav3 完成：uav1 的確認要經 uav2 轉送；第一次轉送故意掉，靠重送（新序號）補回
  swarm.dropNext(2, 3, PacketType::COMPLETION_ACK);
  ASSERT_EQ(assignAndReport(swarm.node(3), makeTaskId(1, 1), true, swarm.now()),
    ReportCheck::ACCEPTED);
  EXPECT_FALSE(swarm.node(3).currentTask());   // 已回報完成：不再交給 BT
  swarm.run(0.5);
  EXPECT_TRUE(swarm.node(3).completionPending(makeTaskId(1, 1)));   // 還缺 uav1 的確認
  swarm.run(1.5);
  EXPECT_FALSE(swarm.node(3).completionPending(makeTaskId(1, 1)));
  for (AgentId a : {1, 2, 3}) {
    EXPECT_EQ(swarm.node(a).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE) << a;
  }
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, CompletionSkipsAgentLostAfterAnnounce)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.run(1.0);

  swarm.setOnline(3, false);   // 剛斷線，還在存活名單裡
  ASSERT_EQ(assignAndReport(swarm.node(1), makeTaskId(1, 1), true, swarm.now()),
    ReportCheck::ACCEPTED);
  swarm.run(1.0);
  EXPECT_TRUE(swarm.node(1).completionPending(makeTaskId(1, 1)));
  swarm.run(1.0);              // 判定 uav3 失聯後就不再等它，不用等 10 次重送
  EXPECT_FALSE(swarm.node(1).completionPending(makeTaskId(1, 1)));
  EXPECT_EQ(swarm.node(2).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);
}

TEST(CbbaComm, FailedTaskGoesBackToPool)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(2).agent().winner(t), 1);

  const std::uint64_t v1 = swarm.node(1).assignment(t).version;
  ASSERT_EQ(assignAndReport(swarm.node(1), t, false, swarm.now()),
    ReportCheck::ACCEPTED);   // uav1 做失敗
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);    // 交給 uav2
  EXPECT_EQ(swarm.node(2).currentTask()->id, t);
  EXPECT_FALSE(swarm.node(1).currentTask());
  EXPECT_EQ(swarm.node(1).agent().tasks().at(t).status, TaskStatus::OPEN);   // 失敗不關閉任務
  EXPECT_EQ(swarm.node(1).stats().sent.count(PacketType::TASK_CLOSE), 0u);
  // 失敗撤銷了 uav1 的指派（新版本、執行者 0），傳到 uav2；uav2 正式指派時版本更大
  EXPECT_EQ(swarm.node(2).assignment(t).assignee, kNoAgent);
  EXPECT_EQ(swarm.node(2).assignment(t).version, v1 + 2);
  const std::uint64_t v2 = activate(swarm.node(2), t);
  EXPECT_EQ(v2, v1 + 3);
  swarm.run(0.5);
  EXPECT_EQ(swarm.node(1).assignment(t).assignee, 2);
  EXPECT_EQ(swarm.node(2).reportResult(t, true, v2, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(1).agent().tasks().at(t).status, TaskStatus::DONE);
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, NonParticipantDoesNotBid)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.node(3).setParticipating(false, swarm.now());   // 還沒起飛
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 21, 0, swarm.now()), swarm.now());   // 最靠近 uav3
  swarm.run(1.0);

  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);     // uav3 不出價，由次近的 uav2 接
  EXPECT_EQ(swarm.node(3).taskCount(), 1u);          // 但它有收到任務
  EXPECT_FALSE(swarm.node(3).currentTask());
  const auto b = swarm.node(3).cbbaStateBody(swarm.now());
  EXPECT_FALSE(b.participating);
  EXPECT_EQ(wire::encodeCbbaState(wire::Header{PacketType::CBBA_STATE, 3, 1, 1, 0}, b).size(),
    36u);                                            // 沒有 s、沒有紀錄
  EXPECT_TRUE(swarm.node(1).converged(swarm.now()));   // 不參與的鄰居不影響收斂判定

  swarm.node(3).setParticipating(true, swarm.now());  // 起飛
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 3);

  swarm.node(3).setParticipating(false, swarm.now()); // 降落：任務立即交回
  swarm.run(0.5);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);
  EXPECT_EQ(swarm.node(2).currentTask()->id, t);
}

TEST(CbbaComm, LostAgentTasksAreReassigned)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 21, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(1).agent().winner(t), 3);

  swarm.setOnline(3, false);   // 墜毀
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 3);   // 1.5 s 內還不算失聯
  swarm.run(1.5);
  EXPECT_EQ(swarm.node(1).agent().winner(t), 2);
  EXPECT_EQ(swarm.node(2).currentTask()->id, t);
}

TEST(CbbaComm, MultiHopAgentIsNotLost)
{
  // 1 — 2 — 3：uav1 聽不到 uav3，但 uav3 的時間戳經 uav2 傳來
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {30, 0, 5});
  swarm.add(3, {60, 0, 5});
  swarm.cut(1, 3);
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 61, 0, swarm.now()), swarm.now());
  swarm.run(5.0);
  EXPECT_EQ(swarm.node(1).agent().winner(makeTaskId(1, 1)), 3);
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, MissedAnnounceIsResent)
{
  // uav3 漏收 uav2 的第一個任務、收到第二個：比對紀錄後補發第一個
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.run(2.0);
  swarm.setOnline(3, false);
  swarm.node(2).addLocalTask(newTask(2, 1, 11, 0, swarm.now()), swarm.now());
  swarm.run(0.5);
  swarm.setOnline(3, true);
  swarm.node(2).addLocalTask(newTask(2, 2, 12, 0, swarm.now()), swarm.now());
  swarm.run(2.0);
  EXPECT_TRUE(swarm.node(3).knows(makeTaskId(2, 1)));
  EXPECT_TRUE(swarm.node(3).knows(makeTaskId(2, 2)));
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, LateJoinerGetsTasksAndCloses)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.setOnline(3, false);
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.node(2).addLocalTask(newTask(2, 1, 11, 0, swarm.now()), swarm.now());
  swarm.node(2).addLocalTask(newTask(2, 2, 12, 3, swarm.now()), swarm.now());
  swarm.run(1.0);
  assignAndReport(swarm.node(1), makeTaskId(1, 1), true, swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(2).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);

  swarm.setOnline(3, true);   // 晚起飛：靠補發拿到任務與 TASK_CLOSE
  swarm.run(3.0);
  EXPECT_EQ(swarm.node(3).taskCount(), 3u);
  EXPECT_EQ(swarm.node(3).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);
  EXPECT_EQ(swarm.node(3).roundKey(), swarm.node(1).roundKey());
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, NonParticipantCatchesUpQuickly)
{
  // 還在地面（不出價）的晚加入者沒有附紀錄：鄰居看到摘要不同就全部補，不用等每 5 s 的保底重播
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.setOnline(3, false);
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.node(1).addLocalTask(newTask(1, 2, 3, 0, swarm.now()), swarm.now());
  swarm.node(2).addLocalTask(newTask(2, 1, 11, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  assignAndReport(swarm.node(1), makeTaskId(1, 1), true, swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(2).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);

  swarm.node(3).setParticipating(false, swarm.now());
  swarm.setOnline(3, true);
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(3).taskCount(), 3u);
  EXPECT_EQ(swarm.node(3).agent().tasks().at(makeTaskId(1, 1)).status, TaskStatus::DONE);
  EXPECT_EQ(swarm.node(3).roundKey(), swarm.node(1).roundKey());
}

TEST(CbbaComm, RestartedNodeLearnsItsOwnClosedTask)
{
  // 自己完成的任務，重開機後靠鄰居補發的 TASK_CLOSE 知道已結束（執行機是自己也要套用）
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  swarm.node(2).addLocalTask(newTask(2, 1, 9, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(assignAndReport(swarm.node(2), makeTaskId(2, 1), true, swarm.now()),
    ReportCheck::ACCEPTED);
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(1).agent().tasks().at(makeTaskId(2, 1)).status, TaskStatus::DONE);

  swarm.setOnline(2, false);
  swarm.run(2.0);
  swarm.restart(2);
  swarm.setOnline(2, true);
  swarm.run(2.0);
  ASSERT_TRUE(swarm.node(2).knows(makeTaskId(2, 1)));
  EXPECT_EQ(swarm.node(2).agent().tasks().at(makeTaskId(2, 1)).status, TaskStatus::DONE);
  EXPECT_EQ(swarm.node(2).roundKey(), swarm.node(1).roundKey());
}

TEST(CbbaComm, ReusedTaskIdAfterRestartIsRejected)
{
  // 重開機後 BT 重用開機前的 task_id：先和鄰居同步，同步後查出重複、不送出，鄰居不受影響
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  swarm.node(2).addLocalTask(newTask(2, 1, 9, 0, swarm.now()), swarm.now());
  swarm.run(2.0);
  swarm.setOnline(2, false);
  swarm.run(4.0);
  swarm.restart(2).setParticipating(false, swarm.now());
  swarm.setOnline(2, true);
  ASSERT_TRUE(swarm.node(2).addLocalTask(newTask(2, 1, 30, 30, swarm.now()), swarm.now()));   // 排隊
  ASSERT_TRUE(swarm.node(2).addLocalTask(newTask(2, 2, 12, 0, swarm.now()), swarm.now()));
  swarm.run(3.0);

  EXPECT_EQ(swarm.node(2).stats().rejected_local_tasks, 1);
  EXPECT_EQ(swarm.node(2).stats().join_timeouts, 0);
  EXPECT_EQ(swarm.node(1).stats().task_conflicts, 0);
  EXPECT_TRUE(swarm.node(1).knows(makeTaskId(2, 2)));
  // 保留的是開機前的那一個（位置 (9, 0)）
  EXPECT_NEAR(swarm.node(1).agent().tasks().at(makeTaskId(2, 1)).position.x, 9.0, 1e-6);
  EXPECT_NEAR(swarm.node(2).agent().tasks().at(makeTaskId(2, 1)).position.x, 9.0, 1e-6);
  EXPECT_EQ(swarm.node(2).roundKey(), swarm.node(1).roundKey());
}

TEST(CbbaComm, JoinTimeoutStartsAnyway)
{
  // 鄰居的摘要一直對不上（它知道的任務一直補不過來）：等滿 join_timeout 才送出自己的任務
  CommConfig cc;
  CbbaComm node(AgentState{}, ScoringParams{}, cc, 0x77);
  const double start = 1.7e9;
  wire::CbbaStateBody b;
  b.round_key = 0x12345678u;   // 和 node 的摘要永遠不同
  b.participating = true;
  std::uint32_t seq = 0;
  int tick = 0;
  auto step = [&](double t) {
    if (tick++ % 20 == 0) {    // 每 0.2 s 聽到一次
      wire::Header h;
      h.sender_id = 2;
      h.session_id = 0x22;
      h.sequence = ++seq;
      b.snapshot_id = static_cast<std::uint16_t>(seq);
      b.stamps = {{2, toStampMs(t)}};
      node.receive(wire::encodeCbbaState(h, b), t);
    }
    node.poll(t);
  };

  double t = start;
  step(t);
  ASSERT_TRUE(node.addLocalTask(newTask(1, 1, 5, 0, t), t));
  for (; t < start + 4.9; t += 0.01) {
    step(t);
  }
  EXPECT_FALSE(node.joined());
  EXPECT_FALSE(node.knows(makeTaskId(1, 1)));
  for (; t < start + 5.2; t += 0.01) {
    step(t);
  }
  EXPECT_TRUE(node.joined());
  EXPECT_EQ(node.stats().join_timeouts, 1);
  EXPECT_TRUE(node.knows(makeTaskId(1, 1)));
}

TEST(CbbaComm, AloneNodeStartsAfterJoinWait)
{
  // 附近沒有別台：join_wait 後就送出，不用等 join_timeout
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  ASSERT_TRUE(swarm.node(1).addLocalTask(newTask(1, 1, 5, 0, swarm.now()), swarm.now()));
  swarm.run(1.0);
  EXPECT_FALSE(swarm.node(1).knows(makeTaskId(1, 1)));
  swarm.run(1.0);
  EXPECT_TRUE(swarm.node(1).knows(makeTaskId(1, 1)));
  EXPECT_EQ(swarm.node(1).stats().join_timeouts, 0);
}

TEST(CbbaComm, NewSessionEveryStart)
{
  // 每次建立都換新的 session，序號從 1 開始；不需要存檔
  const AgentState s;
  CbbaComm a(s, ScoringParams{}, CommConfig{});
  CbbaComm b(s, ScoringParams{}, CommConfig{});
  EXPECT_NE(a.sessionId(), 0u);
  EXPECT_NE(a.sessionId(), b.sessionId());
  const auto out = a.poll(1.7e9);
  ASSERT_FALSE(out.empty());
  const auto h = wire::parseHeader(out.front());
  EXPECT_EQ(h->session_id, a.sessionId());
  EXPECT_EQ(h->sequence, 1u);
}

TEST(CbbaComm, DelayedPacketFromBeforeRestartIsDropped)
{
  // 重開機前送出、在網路上延遲的封包，重開機後才到：鄰居丟掉，不會把舊的任務狀態套回來
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.record(2);
  swarm.run(1.0);
  ASSERT_FALSE(swarm.recorded().empty());
  const Bytes old_packet = swarm.recorded().back();

  swarm.restart(2);
  swarm.run(1.0);
  const int stale = swarm.node(1).stats().stale;
  swarm.node(1).receive(old_packet, swarm.now());
  EXPECT_EQ(swarm.node(1).stats().stale, stale + 1);
}

TEST(CbbaComm, Agent50AndUav2CreateTasksAtTheSameTime)
{
  // 機號 50（狗）也能建立任務，和 uav2 同時建立也不會撞：兩個都被接受
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(50, {5, 20, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  const double now = swarm.now();
  ASSERT_TRUE(swarm.node(2).addLocalTask(newTask(2, 1, 12, 0, now), now));
  ASSERT_TRUE(swarm.node(50).addLocalTask(groundTask(50, 1, 6, 15, now), now));
  ASSERT_TRUE(swarm.node(50).addLocalTask(newTask(50, 2, 1, 1, now), now));   // 空中任務，給無人機
  swarm.run(2.0);

  for (AgentId a : {1, 2, 50}) {
    EXPECT_EQ(swarm.node(a).taskCount(), 3u) << a;
    EXPECT_EQ(swarm.node(a).stats().task_conflicts, 0) << a;
    EXPECT_EQ(swarm.node(a).agent().winner(makeTaskId(50, 1)), 50) << a;
    EXPECT_EQ(swarm.node(a).agent().winner(makeTaskId(2, 1)), 2) << a;
    EXPECT_EQ(swarm.node(a).agent().winner(makeTaskId(50, 2)), 1) << a;
  }
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, UgvAgent50TakesGroundTask)
{
  // 狗（機號 50）：地面處置任務只有狗出價；無人機完成任務時也要等狗的確認（版本 1 的鄰居位元做不到）
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(50, {5, 20, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  ASSERT_TRUE(swarm.node(1).addLocalTask(groundTask(1, 1, 6, 15, swarm.now()), swarm.now()));
  swarm.node(2).addLocalTask(newTask(2, 1, 12, 0, swarm.now()), swarm.now());   // 空中任務
  swarm.run(2.0);

  const TaskId fire_id = makeTaskId(1, 1);
  for (AgentId a : {1, 2, 50}) {
    EXPECT_EQ(swarm.node(a).agent().winner(fire_id), 50) << a;
    EXPECT_EQ(swarm.node(a).agent().winner(makeTaskId(2, 1)), 2) << a;
  }
  ASSERT_TRUE(swarm.consistent());
  // s 只放認識的機號：3 筆（版本 1 是長度 50 的陣列）
  EXPECT_EQ(swarm.node(1).cbbaStateBody(swarm.now()).stamps.size(), 3u);

  // 狗完成：照常等無人機的確認，全隊標成 DONE
  ASSERT_EQ(assignAndReport(swarm.node(50), fire_id, true, swarm.now()),
    ReportCheck::ACCEPTED);
  swarm.run(2.0);
  for (AgentId a : {1, 2, 50}) {
    EXPECT_EQ(swarm.node(a).agent().tasks().at(fire_id).status, TaskStatus::DONE) << a;
  }
  // 無人機完成：也要等狗的確認。狗的第一份確認掉了，等宣告重送後補回
  swarm.dropNext(50, 2, PacketType::COMPLETION_ACK);
  ASSERT_EQ(assignAndReport(swarm.node(2), makeTaskId(2, 1), true, swarm.now()),
    ReportCheck::ACCEPTED);
  swarm.run(0.5);
  EXPECT_TRUE(swarm.node(2).completionPending(makeTaskId(2, 1)));   // 狗的確認掉了：等重送
  swarm.run(1.5);
  EXPECT_FALSE(swarm.node(2).completionPending(makeTaskId(2, 1)));
  EXPECT_EQ(swarm.node(50).agent().tasks().at(makeTaskId(2, 1)).status, TaskStatus::DONE);
}

TEST(CbbaComm, LostUgvReleasesGroundTask)
{
  // 狗失聯：它的地面任務釋放，但沒有別的地面載具，任務留在競標池（無人機不接）
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(50, {5, 20, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  ASSERT_TRUE(swarm.node(1).addLocalTask(groundTask(1, 1, 6, 15, swarm.now()), swarm.now()));
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(1).agent().winner(makeTaskId(1, 1)), 50);
  swarm.setOnline(50, false);
  swarm.run(3.0);
  EXPECT_EQ(swarm.node(1).agent().winner(makeTaskId(1, 1)), kNoAgent);
  EXPECT_TRUE(swarm.node(1).agent().path().empty());
}

TEST(CbbaComm, RestartedNodeRejoins)
{
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.node(2).addLocalTask(newTask(2, 1, 9, 0, swarm.now()), swarm.now());
  swarm.run(2.0);

  swarm.setOnline(2, false);
  swarm.run(4.0);
  swarm.restart(2);            // 重開機：新的 session、任務清單是空的
  swarm.setOnline(2, true);
  ASSERT_TRUE(swarm.node(2).addLocalTask(newTask(2, 2, 12, 0, swarm.now()), swarm.now()));
  swarm.run(4.0);

  EXPECT_EQ(swarm.node(1).taskCount(), 3u);
  EXPECT_EQ(swarm.node(1).agent().winner(makeTaskId(2, 1)), 2);
  EXPECT_EQ(swarm.node(1).stats().task_conflicts, 0);
  EXPECT_TRUE(swarm.consistent());
}

TEST(CbbaComm, ManyTasksSplitIntoParts)
{
  // 一台建立 150 個任務（版本 1 每台只能約 32 個）：CBBA_STATE 分批，每批 ≤ 1200 B，全隊一致
  Swarm swarm;
  swarm.add(1, {0, 0, 5}, 100.0);
  swarm.add(2, {50, 0, 5}, 100.0);
  swarm.add(3, {0, 50, 5}, 100.0);
  swarm.run(2.0);
  for (std::uint16_t k = 1; k <= 150; ++k) {
    swarm.node(1).addLocalTask(
      newTask(1, k, (k % 15) * 7.0, (k / 15) * 7.0, swarm.now(), 80.0, 600.0), swarm.now());
  }
  swarm.run(6.0);
  EXPECT_EQ(swarm.node(3).taskCount(), 150u);
  EXPECT_TRUE(swarm.consistent());
  const auto full = swarm.node(2).cbbaStateBody(swarm.now());
  EXPECT_EQ(full.records.size(), 150u);
  EXPECT_EQ(wire::splitCbbaState(full).size(), 3u);   // 54 筆一批
  const auto & st = swarm.node(2).stats();
  EXPECT_GT(st.cbba_parts, 0);
  EXPECT_EQ(st.malformed, 0);
}

TEST(CbbaComm, LostPartIsNotTreatedAsDeletion)
{
  // 分批時掉了一批：收斂判定、補發只看收齊的那一次，下一次收齊後一致
  Swarm swarm(0.0, 1, [] {CommConfig c; c.max_packet = 200; return c;}());   // 每批 7 筆
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {50, 0, 5});
  swarm.run(2.0);
  for (std::uint16_t k = 1; k <= 40; ++k) {
    swarm.node(1).addLocalTask(newTask(1, k, k * 2.0, 0, swarm.now(), 80.0, 600.0), swarm.now());
  }
  swarm.run(1.0);
  swarm.dropNext(2, 1, PacketType::CBBA_STATE);
  swarm.run(3.0);
  EXPECT_EQ(swarm.node(1).neighbors().at(2).view.size(), 40u);
  EXPECT_GE(swarm.node(1).stats().incomplete_snapshots, 1);
  EXPECT_TRUE(swarm.consistent());
  EXPECT_TRUE(swarm.node(1).converged(swarm.now()));
}

TEST(CbbaComm, ConvergesUnderLoss)
{
  for (double loss : {0.1, 0.3}) {
    int ok = 0;
    const int trials = 10;
    for (int trial = 0; trial < trials; ++trial) {
      Swarm swarm(loss, 100 + trial);
      for (AgentId a = 1; a <= 5; ++a) {
        swarm.add(a, {10.0 * a, (a % 2) * 15.0, 5.0}, 40.0 + 12.0 * a);
      }
      swarm.run(2.0);
      for (AgentId a = 1; a <= 5; ++a) {
        swarm.node(a).addLocalTask(newTask(a, 1, 12.0 * a, 8.0, swarm.now()), swarm.now());
      }
      swarm.node(3).addLocalTask(newTask(3, 2, 25, 25, swarm.now(), 160.0), swarm.now());
      swarm.run(5.0);
      if (swarm.consistent() && swarm.node(1).taskCount() == 6) {
        ++ok;
      }
    }
    EXPECT_EQ(ok, trials) << "loss " << loss;
  }
}

TEST(CbbaComm, QuietWhenConverged)
{
  Swarm swarm;
  for (AgentId a = 1; a <= 5; ++a) {
    swarm.add(a, {10.0 * a, 0, 5});
  }
  swarm.run(2.0);
  for (AgentId a = 1; a <= 5; ++a) {
    swarm.node(a).addLocalTask(newTask(a, 1, 10.0 * a + 1, 0, swarm.now()), swarm.now());
  }
  swarm.run(3.0);
  ASSERT_TRUE(swarm.consistent());

  // 收斂後每架每秒：CBBA_STATE 5 則（5 架 5 任務：36 + 6×5 + 21×5 = 171 B）、
  // AGENT_STATE 2 則（路徑 1、鄰居 4：74 + 4 + 8 = 86 B），加上每 5 s 一則保底重播（65 B）
  const std::size_t before = swarm.bytes();
  swarm.run(10.0);
  const double per_agent_per_sec = (swarm.bytes() - before) / 10.0 / 5.0;
  EXPECT_NEAR(per_agent_per_sec, 5 * 171.0 + 2 * 86.0 + 65.0 / 5.0, 30.0);
  for (AgentId a = 1; a <= 5; ++a) {
    EXPECT_TRUE(swarm.node(a).converged(swarm.now()));
  }
}

// ===========================================================================
// 正式指派版本、回報與 TASK_CLOSE 的權限（2026-10-09）
// ===========================================================================
namespace
{
// 經過轉送者 9（不存在的載具）送來、origin 是 from 的 TASK_CLOSE（模擬舊的、偽造的轉送封包）。
// 不直接用 from 當發送者：不同的 session 會被當成 from 重開機，把真的 from 的 session 退役
constexpr AgentId kRelayer = 9;

Bytes forgedClose(AgentId from, AgentId actor, TaskId t, std::uint8_t reason, std::uint64_t version,
  std::uint32_t seq)
{
  wire::Header h;
  h.sender_id = kRelayer;
  h.session_id = 0x9999;
  h.sequence = seq;
  wire::TaskCloseBody b;
  b.relay = wire::RelayHeader{from, 0xF00Du + from, seq};
  b.task_id = t;
  b.reason = reason;
  b.actor_id = actor;
  b.assign_version = version;
  return wire::encodeTaskClose(h, b);
}

// 把 out 裡第一則 COMPLETION_ACK 解出來
std::optional<wire::CompletionAckBody> firstAck(const std::vector<Bytes> & out)
{
  for (const Bytes & bytes : out) {
    if (wire::parseHeader(bytes)->type == PacketType::COMPLETION_ACK) {
      return wire::decodeCompletionAck(bytes);
    }
  }
  return std::nullopt;
}
}  // namespace

TEST(Assignment, VersionSpreadsMultiHopAndNormalCompletion)
{
  // 1 — 2 — 3：uav3 的正式指派經 CBBA_STATE 傳到聽不到它的 uav1；帶目前版本回報，全隊完成
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {30, 0, 5});
  swarm.add(3, {60, 0, 5});
  swarm.cut(1, 3);
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 61, 0, swarm.now()), swarm.now());
  swarm.run(2.0);
  ASSERT_EQ(swarm.node(3).currentTask()->id, t);
  const std::uint64_t v = activate(swarm.node(3), t);
  swarm.run(0.5);
  EXPECT_EQ(swarm.node(1).assignment(t).version, v);
  EXPECT_EQ(swarm.node(1).assignment(t).assignee, 3);
  EXPECT_EQ(swarm.node(3).agentStateBody(swarm.now()).active_task, t);

  EXPECT_EQ(swarm.node(3).reportResult(t, true, v, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(2.0);
  for (AgentId a : {1, 2, 3}) {
    EXPECT_EQ(swarm.node(a).agent().tasks().at(t).status, TaskStatus::DONE) << a;
    EXPECT_EQ(swarm.node(a).stats().rejected_closes, 0) << a;
  }
}

TEST(Assignment, WrongReporterIsRejected)
{
  // 指派給 uav1 的任務：uav2 回報 → 拒絕；以 uav2 名義送的 TASK_CLOSE（舊版本）、冒名的 TASK_CLOSE → 不套用
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  const std::uint64_t v = activate(swarm.node(1), t);
  swarm.run(0.5);

  EXPECT_EQ(swarm.node(2).reportResult(t, true, v, swarm.now()), ReportCheck::STALE);
  EXPECT_EQ(swarm.node(2).reportResult(t, false, v, swarm.now()), ReportCheck::STALE);

  // uav2 自稱完成，但版本比 uav1 的正式指派舊（uav2 從來沒被指派）
  swarm.node(1).receive(forgedClose(2, 2, t, wire::kCloseDone, v - 1, 1), swarm.now());
  // 冒名：origin 是 uav2，actor 寫 uav1
  swarm.node(1).receive(forgedClose(2, 1, t, wire::kCloseDone, v, 2), swarm.now());
  swarm.run(2.0);   // 轉送者 9 不再算存活的鄰居（否則完成要等它的確認）
  EXPECT_EQ(swarm.node(1).stats().rejected_closes, 2);
  EXPECT_EQ(swarm.node(1).agent().tasks().at(t).status, TaskStatus::OPEN);
  EXPECT_TRUE(swarm.node(1).taskOpen(t));

  // 正確的執行者、目前的版本 → 完成
  EXPECT_EQ(swarm.node(1).reportResult(t, true, v, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(2).agent().tasks().at(t).status, TaskStatus::DONE);
}

TEST(Assignment, StaleReportAfterXYXIsRejected)
{
  // X→Y→X：uav1 第一次的正式指派 v1 → 撤銷 → uav2 → 撤銷 → uav1 第二次 v5。
  // v1 的回報：本機拒絕；以 v1 送出的完成宣告（例如延遲的封包）：別台回 status 2
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.run(1.0);

  const std::uint64_t v1 = activate(swarm.node(1), t);
  swarm.run(0.3);
  swarm.node(1).endAssignment(t);
  swarm.run(0.3);
  const std::uint64_t v3 = activate(swarm.node(2), t);
  swarm.run(0.3);
  swarm.node(2).endAssignment(t);
  swarm.run(0.3);
  const std::uint64_t v5 = activate(swarm.node(1), t);
  swarm.run(0.3);
  EXPECT_EQ(v3, v1 + 2);
  EXPECT_EQ(v5, v1 + 4);
  EXPECT_EQ(swarm.node(2).assignment(t).version, v5);

  EXPECT_EQ(swarm.node(1).reportResult(t, true, v1, swarm.now()), ReportCheck::STALE);

  wire::Header h;   // 經轉送者送來的、uav1 以 v1 發出的宣告
  h.sender_id = kRelayer;
  h.session_id = 0x9999;
  h.sequence = 1;
  wire::CompletionBody old;
  old.relay = wire::RelayHeader{1, 0xBEEF, 1};
  old.task_id = t;
  old.executor_id = 1;
  old.assign_version = v1;
  old.view = {1, 2};
  swarm.node(2).receive(wire::encodeCompletion(h, old), swarm.now());
  const auto ack = firstAck(swarm.node(2).poll(swarm.now()));
  ASSERT_TRUE(ack);
  EXPECT_EQ(ack->status, wire::kAckStaleAssignment);
  EXPECT_TRUE(swarm.node(2).taskOpen(t));

  EXPECT_EQ(swarm.node(1).reportResult(t, true, v5, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(2).agent().tasks().at(t).status, TaskStatus::DONE);
}

TEST(Assignment, ConcurrentAssignmentStaleCompletionIsAborted)
{
  // CBBA 暫時重複指派：uav1、uav2 同時正式指派同一個任務（都是 v1，uav2 的機號大，比較新）。
  // uav1 立刻回報完成：uav2 回 status 2，uav1 的完成宣告作廢，任務保留
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 1, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  const std::uint64_t v = activate(swarm.node(1), t);
  EXPECT_EQ(activate(swarm.node(2), t), v);
  EXPECT_EQ(swarm.node(1).reportResult(t, true, v, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(1.0);
  EXPECT_EQ(swarm.node(1).stats().aborted_completions, 1);
  EXPECT_FALSE(swarm.node(1).completionPending(t));
  EXPECT_TRUE(swarm.node(1).taskOpen(t));
  EXPECT_TRUE(swarm.node(2).taskOpen(t));
  EXPECT_EQ(swarm.node(1).assignment(t).assignee, 2);
}

TEST(Assignment, CancelByCreatorOrAuthorityOnly)
{
  // 取消：建立者、cancel_authorities 裡的機號可以；別台不行，冒名的 TASK_CLOSE 不套用
  CommConfig cc;
  cc.cancel_authorities = {50};   // 狗當授權的管理端（測試用）
  Swarm swarm(0.0, 1, cc);
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(50, {5, 20, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  const TaskId a = makeTaskId(1, 1);
  const TaskId b = makeTaskId(1, 2);
  const TaskId c = makeTaskId(1, 3);
  for (std::uint16_t k = 1; k <= 3; ++k) {
    swarm.node(1).addLocalTask(newTask(1, k, 2.0 * k, 0, swarm.now()), swarm.now());
  }
  swarm.run(1.0);

  EXPECT_EQ(swarm.node(2).cancelTask(a, swarm.now()), ReportCheck::NOT_AUTHORIZED);
  EXPECT_EQ(swarm.node(1).cancelTask(a, swarm.now()), ReportCheck::ACCEPTED);    // 建立者
  EXPECT_EQ(swarm.node(50).cancelTask(b, swarm.now()), ReportCheck::ACCEPTED);   // 授權的管理端
  EXPECT_EQ(swarm.node(1).cancelTask(a, swarm.now()), ReportCheck::NOT_OPEN);    // 已經取消
  // uav2 自稱取消（沒有權限）
  swarm.node(1).receive(forgedClose(2, 2, c, wire::kCloseCancelled, 0, 1), swarm.now());
  swarm.run(1.0);

  for (AgentId n : {1, 2, 50}) {
    EXPECT_EQ(swarm.node(n).agent().tasks().at(a).status, TaskStatus::CANCELLED) << n;
    EXPECT_EQ(swarm.node(n).agent().tasks().at(b).status, TaskStatus::CANCELLED) << n;
    EXPECT_EQ(swarm.node(n).agent().tasks().at(c).status, TaskStatus::OPEN) << n;
  }
  EXPECT_EQ(swarm.node(1).stats().rejected_closes, 1);
  EXPECT_TRUE(swarm.consistent());
}

TEST(Assignment, CancelStopsPendingCompletion)
{
  // 執行者正在確認完成時被建立者取消：結果是取消，不會再送完成
  Swarm swarm;
  swarm.add(1, {0, 0, 5});
  swarm.add(2, {10, 0, 5});
  swarm.add(3, {20, 0, 5});
  swarm.run(2.0);
  const TaskId t = makeTaskId(1, 1);
  swarm.node(1).addLocalTask(newTask(1, 1, 21, 0, swarm.now()), swarm.now());
  swarm.run(1.0);
  ASSERT_EQ(swarm.node(3).currentTask()->id, t);
  swarm.setOnline(2, false);   // uav3 的完成要等 uav2 確認，暫時收不齊
  ASSERT_EQ(assignAndReport(swarm.node(3), t, true, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(0.2);
  ASSERT_TRUE(swarm.node(3).completionPending(t));
  ASSERT_EQ(swarm.node(1).cancelTask(t, swarm.now()), ReportCheck::ACCEPTED);
  swarm.run(1.0);
  EXPECT_FALSE(swarm.node(3).completionPending(t));
  EXPECT_EQ(swarm.node(3).agent().tasks().at(t).status, TaskStatus::CANCELLED);
  EXPECT_EQ(swarm.node(1).agent().tasks().at(t).status, TaskStatus::CANCELLED);
}
