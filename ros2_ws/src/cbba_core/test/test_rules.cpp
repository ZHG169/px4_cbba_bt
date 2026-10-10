// 用共用的測試案例檔驗證 17 條消解規則
#include <gtest/gtest.h>

#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "cbba_core/cbba_agent.hpp"

using namespace cbba_core;

namespace
{
std::vector<std::string> split(const std::string & line, char sep)
{
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, sep)) {
    out.push_back(item);
  }
  return out;
}

std::map<AgentId, double> parseStamps(const std::string & field)
{
  std::map<AgentId, double> stamps;
  if (field == "-" || field.empty()) {
    return stamps;
  }
  for (const std::string & pair : split(field, ';')) {
    const auto kv = split(pair, ':');
    stamps[static_cast<AgentId>(std::stoi(kv.at(0)))] = std::stod(kv.at(1));
  }
  return stamps;
}

const char * toString(RuleAction a)
{
  switch (a) {
    case RuleAction::Update: return "UPDATE";
    case RuleAction::Reset: return "RESET";
    case RuleAction::Leave: return "LEAVE";
  }
  return "?";
}
}  // namespace

TEST(Rules, SharedCaseFile)
{
  std::ifstream file(RULE_CASES_PATH);
  ASSERT_TRUE(file.is_open()) << "cannot open " << RULE_CASES_PATH;

  std::set<int> rules_covered;
  int cases = 0;
  std::string line;
  bool header_seen = false;
  while (std::getline(file, line)) {
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    if (line.empty() || line[0] == '#') {continue;}
    if (!header_seen) {header_seen = true; continue;}

    const auto f = split(line, ',');
    ASSERT_EQ(f.size(), 11u) << line;
    const std::string & name = f[0];
    const int expected_rule = std::stoi(f[1]);
    const auto sender_stamps = parseStamps(f[8]);
    const auto my_stamps = parseStamps(f[9]);

    RuleInput in;
    in.me = static_cast<AgentId>(std::stoi(f[2]));
    in.sender = static_cast<AgentId>(std::stoi(f[3]));
    in.sender_winner = static_cast<AgentId>(std::stoi(f[4]));
    in.sender_score = std::stod(f[5]);
    in.my_winner = static_cast<AgentId>(std::stoi(f[6]));
    in.my_score = std::stod(f[7]);
    in.sender_stamps = &sender_stamps;
    in.my_stamps = &my_stamps;
    in.epsilon = 1e-3;

    const RuleResult result = resolveConflict(in);
    EXPECT_EQ(result.rule, expected_rule) << name;
    EXPECT_STREQ(toString(result.action), f[10].c_str()) << name;
    rules_covered.insert(result.rule);
    ++cases;
  }

  EXPECT_GE(cases, 34);
  for (int rule = 1; rule <= 17; ++rule) {
    EXPECT_TRUE(rules_covered.count(rule)) << "rule " << rule << " has no test case";
  }
}
