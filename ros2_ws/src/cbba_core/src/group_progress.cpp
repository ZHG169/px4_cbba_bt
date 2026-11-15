#include "cbba_core/group_progress.hpp"

namespace cbba_core
{

bool mergeProgress(ProgressRecord & rec, bool has, const wire::TaskProgressBody & b, double now)
{
  const MilestoneMask exec = static_cast<MilestoneMask>(b.milestones & kExecutorMilestones);
  const MilestoneMask event = static_cast<MilestoneMask>(b.milestones & kEventMilestones);
  const ProgressRecord before = rec;
  bool changed = false;

  // 執行者相關：指派版本較新的取代；同一個版本 progress_seq 較大的更新、里程碑 OR
  if (!has || b.assign_version > rec.assign_version) {
    rec.origin = b.relay;
    rec.assign_version = b.assign_version;
    rec.executor = b.executor;
    rec.progress_seq = b.progress_seq;
    rec.phase = static_cast<ProgressPhase>(b.phase);
    rec.step_index = b.step_index;
    rec.executor_bits = exec;
    changed = true;
  } else if (b.assign_version == rec.assign_version && b.progress_seq > rec.progress_seq) {
    rec.origin = b.relay;
    rec.progress_seq = b.progress_seq;
    rec.phase = static_cast<ProgressPhase>(b.phase);
    rec.step_index = b.step_index;
    rec.executor_bits = static_cast<MilestoneMask>(rec.executor_bits | exec);
    changed = rec.phase != before.phase || rec.step_index != before.step_index ||
      rec.executor_bits != before.executor_bits;
  }

  // 事件結果：跟著 revision（不跟指派版本，舊的執行者晚到的回報也算）
  if (!has || b.revision > rec.revision) {
    rec.revision = b.revision;
    rec.event_bits = event;
    changed = changed || !has || event != before.event_bits || b.revision != before.revision;
  } else if (b.revision == rec.revision && (rec.event_bits | event) != rec.event_bits) {
    rec.event_bits = static_cast<MilestoneMask>(rec.event_bits | event);
    changed = true;
  }
  if (changed) {
    rec.updated = now;
  }
  return changed;
}

wire::TaskProgressBody progressBody(TaskId task, const ProgressRecord & rec)
{
  wire::TaskProgressBody b;
  b.relay = rec.origin;
  b.task_id = task;
  b.revision = rec.revision;
  b.assign_version = rec.assign_version;
  b.executor = rec.executor;
  b.progress_seq = rec.progress_seq;
  b.phase = static_cast<std::uint8_t>(rec.phase);
  b.step_index = rec.step_index;
  b.milestones = static_cast<MilestoneMask>(rec.executor_bits | rec.event_bits);
  return b;
}

MilestoneMask effectiveMilestones(const ProgressRecord * rec, std::uint64_t current_version,
  std::uint16_t current_revision, bool accepted, bool expired, bool completed)
{
  if (completed) {
    return kAllMilestones;
  }
  MilestoneMask out = accepted ? milestoneBit(Milestone::ACCEPTED) : 0;
  if (rec != nullptr) {
    if (rec->assign_version >= current_version) {
      out = static_cast<MilestoneMask>(out | rec->executor_bits);
    }
    if (!expired && rec->revision == current_revision) {
      out = static_cast<MilestoneMask>(out | rec->event_bits);
    }
  }
  return out;
}

const char * toString(GateState s)
{
  switch (s) {
    case GateState::OPEN: return "OPEN";
    case GateState::WAITING: return "WAITING";
    case GateState::UNSATISFIABLE: return "UNSATISFIABLE";
  }
  return "?";
}

}  // namespace cbba_core
