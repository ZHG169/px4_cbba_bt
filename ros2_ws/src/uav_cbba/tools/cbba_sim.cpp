// 離線模擬工具：讀取場景檔，執行 CBBA 協商，把結果寫成 CSV（再用 scripts/plot_results.py 畫圖）
// run 另外輸出 bids.csv：協商結束後每台載具對每個任務算出的成本與出價，用來說明為什麼是誰得標
//
//   cbba_sim run     <場景檔>          [--loss 0.3] [--seed 1] [--battery-weight 1.0] [--out 目錄]
//   cbba_sim sweep   <場景檔|random>   [--trials 200] [--agents 3] [--tasks 6] [--out 目錄]
//   cbba_sim weights <場景檔>          [--out 目錄]
#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>

#include "uav_cbba/network_sim.hpp"

using namespace uav_cbba;

namespace
{
struct Options
{
  std::string command;
  std::string scenario;
  std::string out{"cbba_out"};
  double loss{0.0};
  std::uint32_t seed{1};
  int trials{200};
  int agents{3};
  int tasks{6};
  ScoringParams params;
};

void usage()
{
  std::cout <<
    "用法：\n"
    "  cbba_sim run     <場景檔>         [選項]   單次協商，輸出分配結果與協商過程\n"
    "  cbba_sim sweep   <場景檔|random>  [選項]   各丟包率下重複多次，輸出收斂統計\n"
    "  cbba_sim weights <場景檔>         [選項]   比較不同電池權重的分配結果\n"
    "選項：\n"
    "  --out DIR             輸出目錄（預設 cbba_out）\n"
    "  --loss P              丟包率 0~1（run 用，預設 0）\n"
    "  --seed N              亂數種子（預設 1）\n"
    "  --trials N            sweep 每個丟包率的次數（預設 200）\n"
    "  --agents N --tasks M  sweep random 時的規模（預設 3、6）\n"
    "  --battery-weight W    電池權重（預設 1.0）\n"
    "  --speed V             預設速度 m/s（場景檔有指定時以場景檔為準）\n"
    "  --cost-ref C          成本轉分數的基準（預設 50）\n"
    "  --max-bundle N        一次最多接幾個任務（預設 5）\n";
}

bool parse(int argc, char ** argv, Options & opt)
{
  if (argc < 3) {return false;}
  opt.command = argv[1];
  opt.scenario = argv[2];
  for (int i = 3; i < argc; ++i) {
    const std::string key = argv[i];
    if (i + 1 >= argc) {
      std::cerr << "選項 " << key << " 缺少數值\n";
      return false;
    }
    const std::string value = argv[++i];
    try {
      if (key == "--out") {opt.out = value;} else if (key == "--loss") {
        opt.loss = std::stod(value);
      } else if (key == "--seed") {
        opt.seed = static_cast<std::uint32_t>(std::stoul(value));
      } else if (key == "--trials") {
        opt.trials = std::stoi(value);
      } else if (key == "--agents") {
        opt.agents = std::stoi(value);
      } else if (key == "--tasks") {
        opt.tasks = std::stoi(value);
      } else if (key == "--battery-weight") {
        opt.params.battery_weight = std::stod(value);
      } else if (key == "--speed") {
        opt.params.cruise_speed = std::stod(value);
      } else if (key == "--cost-ref") {
        opt.params.cost_ref = std::stod(value);
      } else if (key == "--max-bundle") {
        opt.params.max_bundle = static_cast<std::size_t>(std::stoul(value));
      } else {
        std::cerr << "不認得的選項：" << key << "\n";
        return false;
      }
    } catch (const std::exception &) {
      std::cerr << "選項 " << key << " 的數值無法解析：" << value << "\n";
      return false;
    }
  }
  if (opt.loss < 0.0 || opt.loss > 1.0) {
    std::cerr << "--loss 必須在 0 到 1 之間\n";
    return false;
  }
  return true;
}

std::vector<CbbaAgent> makeAgents(const Scenario & scenario, const ScoringParams & params)
{
  std::vector<CbbaAgent> agents;
  for (const AgentState & state : scenario.agents) {
    agents.emplace_back(state, params);
  }
  return agents;
}

std::ofstream openOut(const std::string & dir, const std::string & name)
{
  std::filesystem::create_directories(dir);
  std::ofstream file(std::filesystem::path(dir) / name);
  if (!file.is_open()) {
    throw std::runtime_error("cannot write " + dir + "/" + name);
  }
  file << std::fixed << std::setprecision(4);
  return file;
}

std::vector<Task> pathOf(const CbbaAgent & agent)
{
  std::vector<Task> tasks;
  for (TaskId id : agent.path()) {
    tasks.push_back(agent.tasks().at(id));
  }
  return tasks;
}

// 把協商完的分配結果寫成 CSV
void writeAssignment(
  const std::string & dir, const Scenario & scenario, const std::vector<CbbaAgent> & agents)
{
  auto agents_csv = openOut(dir, "agents.csv");
  agents_csv << "agent_id,type,x,y,battery,safety_reserve,battery_after,path_energy,"
    "num_tasks,st_cost\n";
  auto assign_csv = openOut(dir, "assignment.csv");
  assign_csv << "agent_id,order,task_id,x,y,arrival,completion,remaining_deadline,overdue,bid\n";

  std::map<TaskId, AgentId> holder;
  for (const CbbaAgent & agent : agents) {
    const AgentState & s = agent.state();
    const PathEval eval = evaluatePath(s, pathOf(agent), 0.0, agent.params());
    agents_csv << int(s.id) << "," << toString(s.type) << "," << s.position.x << ","
               << s.position.y << "," << s.battery << "," << s.safety_reserve << ","
               << (agent.path().empty() ? s.battery : eval.battery_after) << ","
               << eval.path_energy << "," << agent.path().size() << "," << eval.st_cost << "\n";
    for (std::size_t k = 0; k < eval.legs.size(); ++k) {
      const LegEval & leg = eval.legs[k];
      const Task & task = agent.tasks().at(leg.task_id);
      holder[leg.task_id] = s.id;
      assign_csv << int(s.id) << "," << k + 1 << "," << leg.task_id << "," << task.position.x
                 << "," << task.position.y << "," << leg.arrival << "," << leg.completion << ","
                 << leg.remaining_deadline << "," << (leg.overdue ? 1 : 0) << ","
                 << agent.score(leg.task_id) << "\n";
    }
  }

  // 每台 x 每個任務的成本與出價（協商結束時的狀態）
  //   marginal_*：把任務插入「自己最終路徑（去掉這個任務）」的最佳位置，等同 CBBA 的出價方式
  //   alone_*   ：只做這一個任務（從起點直飛再返航）
  auto bids_csv = openOut(dir, "bids.csv");
  bids_csv << "task_id,agent_id,compatible,feasible,marginal_cost,score,alone_feasible,"
    "alone_cost,alone_score,alone_arrival,alone_battery_after,winner\n";
  for (const Task & task : scenario.tasks) {
    for (const CbbaAgent & agent : agents) {
      const AgentState & s = agent.state();
      std::vector<Task> others;
      for (const Task & t : pathOf(agent)) {
        if (t.id != task.id) {others.push_back(t);}
      }
      const Insertion with_path = bestInsertion(s, others, task, 0.0, agent.params());
      const Insertion alone = bestInsertion(s, {}, task, 0.0, agent.params());
      const double arrival = alone.feasible ? alone.eval.legs.front().arrival : 0.0;
      const auto it = holder.find(task.id);
      bids_csv << task.id << "," << int(s.id) << "," << (isCompatible(s.type, task.type) ? 1 : 0)
               << "," << (with_path.feasible ? 1 : 0) << ","
               << (with_path.feasible ? with_path.marginal_cost : -1.0) << "," << with_path.score
               << "," << (alone.feasible ? 1 : 0) << ","
               << (alone.feasible ? alone.marginal_cost : -1.0) << "," << alone.score << ","
               << arrival << "," << (alone.feasible ? alone.eval.battery_after : s.battery) << ","
               << (it != holder.end() && it->second == s.id ? 1 : 0) << "\n";
    }
  }

  auto tasks_csv = openOut(dir, "tasks.csv");
  tasks_csv << "task_id,type,x,y,deadline_sec,value,duration_sec,winner\n";
  for (const Task & task : scenario.tasks) {
    const auto it = holder.find(task.id);
    tasks_csv << task.id << "," << toString(task.type) << "," << task.position.x << ","
              << task.position.y << "," << task.deadline_sec << "," << task.value << ","
              << task.duration_sec << "," << (it == holder.end() ? 0 : int(it->second)) << "\n";
  }
}

int cmdRun(const Options & opt)
{
  const Scenario scenario = loadScenario(opt.scenario);
  std::vector<CbbaAgent> agents = makeAgents(scenario, opt.params);
  NetworkConfig net;
  net.loss = opt.loss;
  std::mt19937 rng(opt.seed);
  const NegotiationResult r = negotiate(agents, scenario.tasks, net, rng, true);

  writeAssignment(opt.out, scenario, agents);

  auto timeline = openOut(opt.out, "timeline.csv");
  timeline << "time,disagreements,broadcasts\n";
  for (const TimelinePoint & p : r.timeline) {
    timeline << p.time << "," << p.disagreements << "," << p.broadcasts << "\n";
  }
  auto summary = openOut(opt.out, "summary.csv");
  summary << "key,value\n"
          << "loss," << opt.loss << "\nseed," << opt.seed << "\nbattery_weight,"
          << opt.params.battery_weight << "\nconverged," << (r.converged ? 1 : 0)
          << "\nconvergence_time," << r.time << "\nbroadcasts," << r.broadcasts
          << "\nbytes," << r.bytes << "\nduplicates," << r.duplicates
          << "\nunassigned," << r.unassigned << "\n";

  std::cout << std::fixed << std::setprecision(2)
            << "場景：" << scenario.agents.size() << " 台載具、" << scenario.tasks.size()
            << " 個任務，丟包率 " << opt.loss * 100 << "%\n"
            << (r.converged ? "已收斂" : "未在時限內收斂") << "，收斂時間 " << r.time
            << " 秒，廣播 " << r.broadcasts << " 次，重複認領 " << r.duplicates
            << "，無人認領 " << r.unassigned << "\n\n";
  for (const CbbaAgent & agent : agents) {
    const AgentState & s = agent.state();
    const PathEval eval = evaluatePath(s, pathOf(agent), 0.0, agent.params());
    std::cout << "  " << toString(s.type) << " " << int(s.id) << "  電量 " << s.battery << " -> "
              << (agent.path().empty() ? s.battery : eval.battery_after) << "   路徑:";
    if (agent.path().empty()) {std::cout << " (無)";}
    for (const LegEval & leg : eval.legs) {
      std::cout << " T" << leg.task_id << "@" << leg.arrival << "s" << (leg.overdue ? "(逾期)" : "");
    }
    std::cout << "\n";
  }
  std::cout << "\n結果已寫入 " << opt.out << "/\n";
  return 0;
}

int cmdSweep(const Options & opt)
{
  const bool random = opt.scenario == "random";
  Scenario fixed;
  if (!random) {
    fixed = loadScenario(opt.scenario);
  }

  auto csv = openOut(opt.out, "sweep.csv");
  csv << "loss,trial,converged,time,broadcasts,bytes,elapsed,duplicates\n";
  std::cout << "  丟包率   收斂率   平均收斂時間   重複認領\n";
  for (double loss : {0.0, 0.1, 0.3, 0.5, 0.7}) {
    int converged = 0;
    int duplicates = 0;
    double time_sum = 0.0;
    for (int trial = 0; trial < opt.trials; ++trial) {
      std::mt19937 rng(opt.seed * 100003u + static_cast<std::uint32_t>(trial));
      const Scenario scenario = random ? randomScenario(opt.agents, opt.tasks, rng) : fixed;
      std::vector<CbbaAgent> agents = makeAgents(scenario, opt.params);
      NetworkConfig net;
      net.loss = loss;
      const NegotiationResult r = negotiate(agents, scenario.tasks, net, rng);
      csv << loss << "," << trial << "," << (r.converged ? 1 : 0) << "," << r.time << ","
          << r.broadcasts << "," << r.bytes << "," << r.elapsed << "," << r.duplicates << "\n";
      if (r.converged) {
        ++converged;
        time_sum += r.time;
        duplicates += r.duplicates;
      }
    }
    std::cout << std::fixed << std::setprecision(1) << "  " << std::setw(5) << loss * 100 << "%  "
              << std::setw(6) << 100.0 * converged / opt.trials << "%  " << std::setprecision(2)
              << std::setw(10) << (converged > 0 ? time_sum / converged : 0.0) << " 秒  "
              << std::setw(6) << duplicates << "\n";
  }
  std::cout << "\n結果已寫入 " << opt.out << "/sweep.csv\n";
  return 0;
}

int cmdWeights(const Options & opt)
{
  const Scenario scenario = loadScenario(opt.scenario);
  auto csv = openOut(opt.out, "weights.csv");
  csv << "weight,agent_id,type,num_tasks,battery,battery_after,safety_reserve,mean_arrival,"
    "max_arrival,overdue\n";
  std::cout << "  權重   最低剩餘電量   平均到達時間   逾期任務   已分配任務\n";
  for (double weight : {0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0}) {
    ScoringParams params = opt.params;
    params.battery_weight = weight;
    std::vector<CbbaAgent> agents = makeAgents(scenario, params);
    NetworkConfig net;   // 不丟包：只看分配結果
    std::mt19937 rng(opt.seed);
    negotiate(agents, scenario.tasks, net, rng);

    double min_after = kInf;
    double arrival_sum = 0.0;
    int assigned = 0;
    int overdue_total = 0;
    for (const CbbaAgent & agent : agents) {
      const AgentState & s = agent.state();
      const PathEval eval = evaluatePath(s, pathOf(agent), 0.0, params);
      const double after = agent.path().empty() ? s.battery : eval.battery_after;
      double sum = 0.0;
      double max_arrival = 0.0;
      int overdue = 0;
      for (const LegEval & leg : eval.legs) {
        sum += leg.arrival;
        max_arrival = std::max(max_arrival, leg.arrival);
        overdue += leg.overdue ? 1 : 0;
      }
      const double mean = eval.legs.empty() ? 0.0 : sum / static_cast<double>(eval.legs.size());
      csv << weight << "," << int(s.id) << "," << toString(s.type) << "," << eval.legs.size()
          << "," << s.battery << "," << after << "," << s.safety_reserve << "," << mean << ","
          << max_arrival << "," << overdue << "\n";
      if (s.type == AgentType::UAV) {
        min_after = std::min(min_after, after);
      }
      arrival_sum += sum;
      assigned += static_cast<int>(eval.legs.size());
      overdue_total += overdue;
    }
    std::cout << std::fixed << std::setprecision(1) << "  " << std::setw(5) << weight << "  "
              << std::setw(10) << min_after << "   " << std::setw(10)
              << (assigned > 0 ? arrival_sum / assigned : 0.0) << " 秒   " << std::setw(6)
              << overdue_total << "   " << std::setw(8) << assigned << "\n";
  }
  std::cout << "\n結果已寫入 " << opt.out << "/weights.csv\n";
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  if (!parse(argc, argv, opt)) {
    usage();
    return 2;
  }
  try {
    if (opt.command == "run") {return cmdRun(opt);}
    if (opt.command == "sweep") {return cmdSweep(opt);}
    if (opt.command == "weights") {return cmdWeights(opt);}
    std::cerr << "不認得的指令：" << opt.command << "\n";
    usage();
    return 2;
  } catch (const std::exception & e) {
    std::cerr << "錯誤：" << e.what() << "\n";
    return 1;
  }
}
