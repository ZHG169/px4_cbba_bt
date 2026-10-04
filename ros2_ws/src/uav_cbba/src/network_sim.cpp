#include "uav_cbba/network_sim.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace uav_cbba
{

namespace
{
struct Pending
{
  double deliver_at;
  std::size_t target;
  CbbaMessage msg;
};

int countDisagreements(const std::vector<CbbaAgent> & agents, const std::vector<Task> & tasks)
{
  int count = 0;
  for (const Task & task : tasks) {
    const AgentId w = agents.front().winner(task.id);
    for (const CbbaAgent & agent : agents) {
      if (agent.winner(task.id) != w) {
        ++count;
        break;
      }
    }
  }
  return count;
}

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto begin = item.find_first_not_of(" \t");
    const auto end = item.find_last_not_of(" \t");
    out.push_back(begin == std::string::npos ? "" : item.substr(begin, end - begin + 1));
  }
  return out;
}
}  // namespace

const char * toString(AgentType type)
{
  return type == AgentType::UAV ? "UAV" : "UGV";
}

const char * toString(TaskType type)
{
  switch (type) {
    case TaskType::AIR_RECON: return "AIR_RECON";
    case TaskType::GROUND_INTERVENTION: return "GROUND_INTERVENTION";
    case TaskType::PATROL: return "PATROL";
  }
  return "?";
}

NegotiationResult negotiate(
  std::vector<CbbaAgent> & agents, const std::vector<Task> & tasks, const NetworkConfig & cfg,
  std::mt19937 & rng, bool record_timeline)
{
  if (agents.empty()) {
    throw std::invalid_argument("negotiate() needs at least one agent");
  }
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> delay(cfg.delay_min, cfg.delay_max);

  std::vector<bool> dirty(agents.size(), false);
  for (std::size_t a = 0; a < agents.size(); ++a) {
    for (const Task & task : tasks) {
      dirty[a] = agents[a].onTask(task, 0.0) || dirty[a];
    }
  }

  std::vector<double> next_periodic(agents.size());
  std::vector<double> last_send(agents.size(), -1.0);
  for (double & p : next_periodic) {
    p = unit(rng) * cfg.period;
  }

  std::vector<Pending> network;
  NegotiationResult result;
  double stable_since = -1.0;

  const int steps = static_cast<int>((cfg.timeout + cfg.stable) / cfg.dt) + 1;
  for (int step = 0; step <= steps; ++step) {
    const double now = step * cfg.dt;
    result.elapsed = now;

    // 送達到期的訊息（依到達時間，不是送出順序）
    std::stable_sort(
      network.begin(), network.end(),
      [](const Pending & a, const Pending & b) {return a.deliver_at < b.deliver_at;});
    std::size_t delivered = 0;
    while (delivered < network.size() && network[delivered].deliver_at <= now) {
      const Pending & p = network[delivered];
      dirty[p.target] = agents[p.target].onMessage(p.msg, now) || dirty[p.target];
      ++delivered;
    }
    network.erase(network.begin(), network.begin() + static_cast<std::ptrdiff_t>(delivered));

    // 發送：狀態改變時立即送，另外週期重送
    for (std::size_t a = 0; a < agents.size(); ++a) {
      const bool periodic = now >= next_periodic[a];
      const bool immediate = dirty[a] && (now - last_send[a]) >= cfg.min_gap;
      if (!periodic && !immediate) {
        continue;
      }
      const CbbaMessage msg = agents[a].makeMessage(now);
      ++result.broadcasts;
      // 約略的封包大小：標頭 28 + 每筆出價 12 + 每筆時間戳 12 + RTPS/UDP/IP 約 100 bytes
      result.bytes += 128.0 + 12.0 * static_cast<double>(msg.bids.size()) +
        12.0 * static_cast<double>(msg.stamps.size());
      for (std::size_t b = 0; b < agents.size(); ++b) {
        if (b == a || unit(rng) < cfg.loss) {
          continue;
        }
        network.push_back(Pending{now + delay(rng), b, msg});
      }
      dirty[a] = false;
      last_send[a] = now;
      if (periodic) {
        next_periodic[a] += cfg.period;
      }
    }

    const int disagreements = countDisagreements(agents, tasks);
    if (record_timeline) {
      result.timeline.push_back(TimelinePoint{now, disagreements, result.broadcasts});
    }
    if (disagreements > 0) {
      stable_since = -1.0;
    } else if (stable_since < 0.0) {
      stable_since = now;
    }
    if (disagreements == 0 && now - stable_since >= cfg.stable && stable_since <= cfg.timeout) {
      result.converged = true;
      result.time = stable_since;
      break;
    }
  }

  for (const Task & task : tasks) {
    int holders = 0;
    AgentId holder = kNoAgent;
    for (const CbbaAgent & agent : agents) {
      const auto & b = agent.bundle();
      if (std::find(b.begin(), b.end(), task.id) != b.end()) {
        ++holders;
        holder = agent.state().id;
      }
    }
    if (holders > 1) {++result.duplicates;}
    if (holders == 0) {++result.unassigned;}
    if (result.converged && holders == 1 && agents.front().winner(task.id) != holder) {
      ++result.inconsistent;
    }
  }
  return result;
}

Scenario loadScenario(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("cannot open scenario file: " + path);
  }

  Scenario scenario;
  std::string line;
  int line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    if (line.empty() || line[0] == '#') {continue;}
    const auto f = splitCsv(line);
    const auto fail = [&](const std::string & why) {
        throw std::runtime_error(path + ":" + std::to_string(line_no) + ": " + why);
      };
    try {
      if (f.at(0) == "agent") {
        if (f.size() < 10) {fail("agent needs 10 columns");}
        AgentState a;
        a.id = static_cast<AgentId>(std::stoi(f[1]));
        if (f[2] == "UAV") {a.type = AgentType::UAV;} else if (f[2] == "UGV") {
          a.type = AgentType::UGV;
        } else {fail("unknown agent type " + f[2]);}
        a.position = {std::stod(f[3]), std::stod(f[4]), std::stod(f[5])};
        a.home = a.position;
        a.battery = std::stod(f[6]);
        a.safety_reserve = std::stod(f[7]);
        a.energy_per_meter = std::stod(f[8]);
        a.hover_energy_per_sec = std::stod(f[9]);
        a.cruise_speed = f.size() > 10 && !f[10].empty() ? std::stod(f[10]) : 0.0;
        scenario.agents.push_back(a);
      } else if (f.at(0) == "task") {
        if (f.size() < 9) {fail("task needs 9 columns");}
        Task t;
        t.id = static_cast<TaskId>(std::stoul(f[1]));
        if (f[2] == "AIR_RECON") {t.type = TaskType::AIR_RECON;} else if (
          f[2] == "GROUND_INTERVENTION")
        {
          t.type = TaskType::GROUND_INTERVENTION;
        } else if (f[2] == "PATROL") {t.type = TaskType::PATROL;} else {
          fail("unknown task type " + f[2]);
        }
        t.position = {std::stod(f[3]), std::stod(f[4]), std::stod(f[5])};
        t.deadline_sec = std::stod(f[6]);
        t.value = std::stod(f[7]);
        t.duration_sec = std::stod(f[8]);
        scenario.tasks.push_back(t);
      } else {
        fail("first column must be 'agent' or 'task'");
      }
    } catch (const std::invalid_argument &) {
      fail("not a number");
    } catch (const std::out_of_range &) {
      fail("missing or out-of-range value");
    }
  }
  if (scenario.agents.empty()) {
    throw std::runtime_error(path + ": no agents");
  }
  return scenario;
}

Scenario randomScenario(int num_agents, int num_tasks, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> pos(-50.0, 50.0);
  std::uniform_real_distribution<double> battery(80.0, 200.0);
  std::uniform_real_distribution<double> deadline(60.0, 300.0);

  Scenario scenario;
  for (int a = 0; a < num_agents; ++a) {
    AgentState s;
    s.id = static_cast<AgentId>(a + 1);
    s.type = AgentType::UAV;
    s.position = {pos(rng), pos(rng), 0.0};
    s.home = s.position;
    s.battery = battery(rng);
    scenario.agents.push_back(s);
  }
  for (int t = 0; t < num_tasks; ++t) {
    Task task;
    task.id = makeTaskId(100, static_cast<std::uint16_t>(t + 1));
    task.type = TaskType::AIR_RECON;
    task.position = {pos(rng), pos(rng), 0.0};
    task.deadline_sec = deadline(rng);
    task.value = 80.0;
    task.duration_sec = 5.0;
    scenario.tasks.push_back(task);
  }
  return scenario;
}

}  // namespace uav_cbba
