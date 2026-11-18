// 任務的執行結果送閘道（任務欄位補齊 11，2026-11-18）：outbox 的重送、限速、滿了丟最舊的；閘道確認、去重複；
// 閘道斷線時暫存、恢復後送出（結果 outbox）
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "cbba_core/task_report.hpp"
#include "swarm_harness.hpp"

using namespace cbba_test;

namespace
{
constexpr AgentId kGateway = 200;

TaskReport report(std::uint32_t seq, bool success = true)
{
  TaskReport r;
  r.task = makeTaskId(kGateway, 3, seq);
  r.revision = 1;
  r.assign_version = 2;
  r.t = 1.7e9;
  r.success = success;
  r.completed_steps = 2;
  r.position_valid = true;
  r.final_position = {10.0, 25.0, 0.0};
  r.failure_code = success ? "" : "E_TIMEOUT";
  r.findings_uri = "evidence://ev_" + std::to_string(seq) + "/findings.json";
  r.attachments = {"evidence://ev_" + std::to_string(seq) + "/snap.jpg"};
  return r;
}

Swarm makeSwarm()
{
  CommConfig cc;
  cc.epoch = 3;
  Swarm swarm(0.0, 1, cc);
  swarm.addGateway(kGateway);
  swarm.add(50, {0, 0, 0}, 100.0, AgentType::UGV);
  swarm.run(2.0);
  return swarm;
}
}  // namespace

TEST(ReportOutbox, RetryRateAndCapacity)
{
  ReportOutboxConfig c;
  c.capacity = 3;
  c.retry = 1.0;
  c.rate = 1.0;
  c.burst = 2.0;
  ReportOutbox box(c);
  EXPECT_FALSE(box.add(report(1), true));
  EXPECT_FALSE(box.add(report(2), true));
  EXPECT_FALSE(box.add(report(3), false));
  EXPECT_TRUE(box.add(report(4), true));    // 滿了：丟最舊的
  EXPECT_EQ(box.size(), 3u);
  EXPECT_EQ(box.items().front().report.task, makeTaskId(kGateway, 3, 2));
  EXPECT_EQ(box.bufferedCount(), 1u);
  EXPECT_EQ(box.due(0.0).size(), 2u);       // burst 2
  EXPECT_EQ(box.due(0.5).size(), 0u);       // 只補了 0.5 個
  EXPECT_EQ(box.due(1.0).size(), 1u);       // 第三個
  EXPECT_TRUE(box.ack(box.items().front().seq));
  EXPECT_FALSE(box.ack(999));
  EXPECT_EQ(box.size(), 2u);
}

TEST(TaskReport, ReachesGatewayOnceAndIsAcked)
{
  // 11 驗收 3：狗的結果（findings、附件、最後位置、做完幾步）送到閘道；確認後佇列清空；掉了確認時重送、閘道去重複
  Swarm swarm = makeSwarm();
  ASSERT_TRUE(swarm.node(50).addReport(report(1), swarm.now()));
  swarm.block(kGateway, 50, PacketType::TASK_REPORT_ACK);   // 第一次的確認掉了 → 重送
  swarm.run(1.5);
  swarm.block(kGateway, 50, PacketType::TASK_REPORT_ACK, false);
  swarm.run(2.0);
  EXPECT_EQ(swarm.node(50).reportsPending(), 0u);
  EXPECT_GE(swarm.node(50).stats().report_retries, 1);
  EXPECT_GE(swarm.node(kGateway).stats().report_duplicates, 1);
  const auto got = swarm.node(kGateway).takeReports();
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].agent, 50);
  EXPECT_FALSE(got[0].buffered);
  const TaskReport & r = got[0].report;
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.task, makeTaskId(kGateway, 3, 1));
  EXPECT_EQ(r.completed_steps, 2);
  EXPECT_NEAR(r.final_position.y, 25.0, 1e-3);
  EXPECT_EQ(r.findings_uri, "evidence://ev_1/findings.json");
  EXPECT_EQ(r.attachments, std::vector<std::string>{"evidence://ev_1/snap.jpg"});
  EXPECT_NEAR(r.t, 1.7e9, 1e-3);
}

TEST(TaskReport, BufferedWhileGatewayOfflineThenDelivered)
{
  // 結果 outbox：閘道斷線時產生的結果只暫存（不送），恢復後由舊到新送到，標「暫存過」
  Swarm swarm = makeSwarm();
  swarm.setOnline(kGateway, false);
  swarm.run(4.0);
  ASSERT_FALSE(swarm.node(50).gatewayOnline(swarm.now()));
  swarm.node(50).addReport(report(1, false), swarm.now());
  swarm.node(50).addReport(report(2), swarm.now());
  swarm.run(2.0);
  EXPECT_EQ(swarm.node(50).reportsBuffered(), 2u);
  EXPECT_EQ(swarm.node(50).stats().reports_sent, 0);
  swarm.setOnline(kGateway, true);
  swarm.run(4.0);
  EXPECT_EQ(swarm.node(50).reportsPending(), 0u);
  const auto got = swarm.node(kGateway).takeReports();
  ASSERT_EQ(got.size(), 2u);
  EXPECT_TRUE(got[0].buffered);
  EXPECT_EQ(got[0].report.failure_code, "E_TIMEOUT");
  EXPECT_FALSE(got[0].report.success);
  EXPECT_EQ(got[1].report.task, makeTaskId(kGateway, 3, 2));
}

TEST(TaskReport, InvalidReportIsRefused)
{
  Swarm swarm = makeSwarm();
  TaskReport r = report(1);
  r.attachments.assign(9, "evidence://x");
  EXPECT_FALSE(swarm.node(50).addReport(r, swarm.now()));
  r = report(1);
  r.task = 0;
  EXPECT_FALSE(swarm.node(50).addReport(r, swarm.now()));
}
