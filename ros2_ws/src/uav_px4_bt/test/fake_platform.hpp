// 測試用的假平台（2026-10-13 從 test_tree.cpp 抽出，test_tree、test_skeleton 共用）：
// 速度一階追隨命令（時間常數 0.3 s），要求 offboard＋解鎖後下一個 tick 就成立。
#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include "uav_px4_bt/uav_platform.hpp"

namespace uav_px4_bt::test
{
struct Result
{
  std::uint32_t task_id;
  std::uint64_t version;
  bool success;
  std::string detail;
};

struct Created
{
  std::uint8_t type;
  Vec3 position;
  double duration;
  std::uint32_t task_id;
};

class FakePlatform : public UavPlatform
{
public:
  double timeNow() const override {return t;}
  VehicleState vehicle() const override {return v;}
  std::vector<Neighbor> neighbors() const override {return nbrs;}
  std::vector<FireObservation> fireObservations() const override {return fires;}
  void setVelocity(const Vec3 & cmd) override {velocity_cmd = cmd;}
  // 位置＋速度前饋：模擬 PX4 的位置控制器（速度 = 前饋 + 1.0 × 位置誤差）
  void setSetpoint(const Vec3 & p, const Vec3 & vel) override {velocity_cmd = vel + (p - v.position) * 1.0;}
  void requestOffboardAndArm() override {++arm_requests; v.armed = true; v.offboard = true;}
  void requestReturnToLaunch() override {++rtl_requests; v.offboard = false;}
  void stopOffboard() override {offboard = false;}
  void reportResult(const AssignedTask & task, bool success, const std::string & detail) override
  {
    results.push_back({task.task_id, task.version, success, detail});
  }
  std::uint32_t createTask(std::uint8_t type, const Vec3 & p, double, double duration, double) override
  {
    const std::uint32_t id = 0x00018001u + static_cast<std::uint32_t>(created.size());
    created.push_back({type, p, duration, id});
    return id;
  }
  void setExecStatus(const ExecStatus & s) override {exec = s;}
  bool taskClosed(std::uint32_t id) const override {return closed.count(id) > 0;}
  std::vector<KnownGroundTask> groundTasks() const override {return ground;}
  void log(const std::string &) override {}

  // 一個 tick 之後的動態
  void advance(double dt)
  {
    if (v.armed && v.offboard && !stuck) {
      v.velocity = v.velocity + (velocity_cmd - v.velocity) * (dt / 0.3);
      v.position = v.position + v.velocity * dt;
      v.landed = v.position.z < 0.1;
    }
    t += dt;
  }

  double t{100.0};
  VehicleState v;
  Vec3 velocity_cmd;
  bool stuck{false};
  bool offboard{true};
  int arm_requests{0};
  int rtl_requests{0};
  std::vector<FireObservation> fires;
  std::vector<Neighbor> nbrs;
  std::vector<Result> results;
  std::vector<Created> created;
  ExecStatus exec;
  std::set<std::uint32_t> closed;   // cbba_node 的 task_closed
  std::vector<KnownGroundTask> ground;   // cbba_node 的 task_announced（地面處置任務）
};

}  // namespace uav_px4_bt::test
