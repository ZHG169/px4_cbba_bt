// TASK_REPORT、TASK_REPORT_ACK（協定版本 5，任務欄位補齊 11，2026-11-18）：大小、來回編解碼、轉送、拒絕不合法的
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "cbba_core/types.hpp"
#include "cbba_core/wire_task_report.hpp"

using namespace cbba_core::wire;

namespace
{
Header makeHeader()
{
  Header h;
  h.sender_id = 50;
  h.session_id = 0xAB;
  h.sequence = 7;
  return h;
}

TaskReportBody sample()
{
  TaskReportBody b;
  b.relay = RelayHeader{50, 0xAB, 7};
  b.report_seq = 3;
  b.task_id = cbba_core::makeTaskId(200, 3, 17);
  b.revision = 2;
  b.assign_version = 9;
  b.t_ms = 1794597973974ULL;
  b.flags = kReportSuccess | kReportPositionValid;
  b.completed_steps = 3;
  b.x = 12.0F;
  b.y = -4.0F;
  b.z = 0.5F;
  b.findings_uri = "evidence://ev_0042/findings/dog1.json";
  b.attachments = {"evidence://ev_0042/snapshot/dog1_0001.jpg", "evidence://ev_0042/snapshot/dog1_0002.jpg"};
  return b;
}
}  // namespace

TEST(WireTaskReport, RoundTripAndSize)
{
  const TaskReportBody b = sample();
  const auto bytes = encodeTaskReport(makeHeader(), b);
  EXPECT_EQ(bytes.size(), taskReportSize(b));
  EXPECT_EQ(bytes.size(), kTaskReportBaseSize + b.findings_uri.size() + 2 * 2 + b.attachments[0].size() +
    b.attachments[1].size());
  EXPECT_EQ(parseHeader(bytes)->type, PacketType::TASK_REPORT);
  EXPECT_TRUE(isRelayed(PacketType::TASK_REPORT));
  const auto d = decodeTaskReport(bytes);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->relay, b.relay);
  EXPECT_EQ(d->report_seq, 3u);
  EXPECT_EQ(d->task_id, b.task_id);
  EXPECT_EQ(d->revision, 2);
  EXPECT_EQ(d->assign_version, 9u);
  EXPECT_EQ(d->t_ms, b.t_ms);
  EXPECT_EQ(d->flags, b.flags);
  EXPECT_EQ(d->completed_steps, 3);
  EXPECT_FLOAT_EQ(d->y, -4.0F);
  EXPECT_EQ(d->findings_uri, b.findings_uri);
  EXPECT_EQ(d->attachments, b.attachments);
  EXPECT_TRUE(d->failure_code.empty());

  TaskReportBody failed = sample();
  failed.flags = 0;
  failed.failure_code = "E_TIMEOUT";
  failed.detail = "TASK_TIMEOUT";
  failed.attachments.clear();
  const auto f = decodeTaskReport(encodeTaskReport(makeHeader(), failed));
  ASSERT_TRUE(f);
  EXPECT_EQ(f->failure_code, "E_TIMEOUT");
  EXPECT_EQ(f->detail, "TASK_TIMEOUT");
  EXPECT_TRUE(f->attachments.empty());
}

TEST(WireTaskReport, RejectsInvalid)
{
  // 流水號 0、task_id 不合法、字串太長、附件太多、整個封包超過 1200 B；截斷、多出位元組
  TaskReportBody b = sample();
  b.report_seq = 0;
  EXPECT_THROW(encodeTaskReport(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.task_id = 0;
  EXPECT_THROW(encodeTaskReport(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.detail = std::string(201, 'd');
  EXPECT_THROW(encodeTaskReport(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.attachments.assign(9, "evidence://x");
  EXPECT_THROW(encodeTaskReport(makeHeader(), b), std::invalid_argument);
  b = sample();
  b.findings_uri = "evidence://" + std::string(500, 'u');
  b.attachments.assign(2, "evidence://" + std::string(500, 'a'));   // 每個合法，加起來超過 1200 B
  EXPECT_GT(taskReportSize(b), kMaxReportPacket);
  EXPECT_THROW(encodeTaskReport(makeHeader(), b), std::invalid_argument);

  auto bytes = encodeTaskReport(makeHeader(), sample());
  auto cut = bytes;
  cut.pop_back();
  EXPECT_FALSE(decodeTaskReport(cut));
  auto extra = bytes;
  extra.push_back(0);
  EXPECT_FALSE(decodeTaskReport(extra));
}

TEST(WireTaskReport, Ack)
{
  TaskReportAckBody a;
  a.relay = RelayHeader{200, 0xCD, 1};
  a.target_id = 50;
  a.target_session = 0xAB;
  a.seqs = {3, 4, 5};
  const auto bytes = encodeTaskReportAck(makeHeader(), a);
  EXPECT_EQ(bytes.size(), kTaskReportAckBaseSize + 12);
  EXPECT_TRUE(isRelayed(PacketType::TASK_REPORT_ACK));
  const auto d = decodeTaskReportAck(bytes);
  ASSERT_TRUE(d);
  EXPECT_EQ(d->target_id, 50);
  EXPECT_EQ(d->seqs, a.seqs);
  a.seqs.clear();
  EXPECT_THROW(encodeTaskReportAck(makeHeader(), a), std::invalid_argument);
}
