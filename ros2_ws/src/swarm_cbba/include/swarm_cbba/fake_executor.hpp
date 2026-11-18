// 假的動作執行端（ExecuteStep.action 的草案用，2026-11-19；registry/api.md 第 5 節）：不接真的 PX4、Nav2、雲台，
// 照 registry/actions.yaml 檢查一步的內容，用簡單的運動模型模擬執行、進度、取消、逾時、失敗碼。給：
//   - 伯宇、羅凡對 Action 欄位（不用等真的 adapter）
//   - BT 的 ExecuteSteps 在沒有載具時測流程
//   - uav_adapter 的參考行為（平台狀態機、自動起飛、逾時、取消）
// 不依賴 ROS（節點層 fake_adapter_node 包成 Action server）；JSON 用 yaml-cpp 解析（JSON 是 YAML 的子集）。
//
// 檢查：和閘道注入前的 TaskSpec 檢查是同一套（validateSpec，把這一步包成只有一步的 spec），錯誤碼一樣；
//   另外收系統動作 emergency_stop、emergency_land、report_state（不是 plannable，validateSpec 不收）。
//   實作標 not_started 的（patrol、track、speak…）預設也模擬，附警告（allow_not_started）。
// 模擬（速度 speed m/s，UAV 的爬升、下降也用它）：
//   go_to           移到 location（standoff_m：停在前面多遠）；UAV 用 altitude_m（沒給用區域的 cruise_altitude_m，再沒有用
//                   cruise_altitude）
//   patrol          waypoints（[[x, y, z], …]）或 area（區域的多邊形頂點），繞 loops 圈
//   inspect         不移動：瞄準 1 s → 拍 duration_s；findings 一張截圖（mode）
//   track           跟 duration_s；findings 追蹤結果
//   wait            duration_s；speak 2 s；alert_operator 0.5 s；report_state、emergency_stop 立即
//   return_to_dock  移到 dock（docks.csv；沒給用起點），UAV 降落 → LANDED
//   emergency_land  （UAV）EMERGENCY → 原地降落 → LANDED
// 平台狀態機（UAV）：LANDED → 需要在空中的動作先自動起飛（takeoff 到 1.5 m → climbing 到巡航高度）→ AIRBORNE；
//   狗：READY（emergency_stop 之後 EMERGENCY，下一個 goal 恢復）。
// 逾時：goal 的 deadline；沒有時 actions.yaml 的 timeout_s；0 的動作用「估計時間 × 1.5 + 10 s」。
// 故障注入（測試用）：Fault{action, failure_code, failed_capability, at}：做到進度 at 時失敗；slowdown < 1 讓實際速度變慢（測逾時）。
#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "swarm_cbba/registry.hpp"

namespace swarm_cbba
{

enum class StepStatus : std::uint8_t { RUNNING = 255, SUCCEEDED = 0, FAILED = 1, CANCELED = 2, TIMEOUT = 3 };
const char * toString(StepStatus s);

enum class PlatformState : std::uint8_t { LANDED, AIRBORNE, READY, EMERGENCY };
const char * toString(PlatformState s);

struct StepGoal
{
  std::uint64_t task_id{0};
  std::uint16_t revision{0};
  std::uint32_t step_id{0};
  std::string action;
  std::string object;
  std::string location_json;
  std::string params_json;
  double deadline{0.0};          // 絕對時刻（秒）；0 = 用 actions.yaml 的 timeout_s
};

struct StepFault
{
  std::string action;            // 哪個動作（空 = 全部）
  std::string failure_code;      // E_*
  std::string failed_capability; // E_HW 時
  double at{0.5};                // 做到這個進度時失敗（0～1）
};

struct StepSnapshot
{
  StepStatus status{StepStatus::RUNNING};
  double progress{0.0};
  std::string phase;
  PlatformState platform{PlatformState::READY};
  std::array<double, 3> position{{0.0, 0.0, 0.0}};
  // 結束時
  std::string failure_code;
  std::string failed_capability;
  std::string findings_json{"[]"};
  std::string detail;
};

struct FakeExecutorConfig
{
  std::string platform{"quadruped"};   // platforms.yaml 的 name
  double speed{1.0};                    // m/s
  double cruise_altitude{3.0};          // UAV 沒給高度時（2-1b：室內 3 m）
  double takeoff_altitude{1.5};         // UAV 的 takeoff（Node-RED 的 /api/takeoff）
  std::array<double, 3> home{{0.0, 0.0, 0.0}};   // 起點（return_to_dock 沒給 dock 時回這裡）
  double slowdown{1.0};                 // 實際速度 = speed × slowdown（< 1 測逾時）
  // actions.yaml 裡這個平台的實作是 not_started 的也模擬（閘道的檢查會擋，假 adapter 是要先測介面）：只附警告
  bool allow_not_started{true};
  std::vector<StepFault> faults;
};

class FakeExecutor
{
public:
  // site 可以是 nullptr（location 只收座標）。registry、site 要活得比這個物件久
  FakeExecutor(const Registry & registry, const Site * site, const FakeExecutorConfig & config);

  // 開始一步。內容不合法時回傳結束的快照（FAILED、E_PRECONDITION、detail 寫第一個錯誤）；否則 RUNNING。
  // 正在執行時呼叫：舊的當成被取代（呼叫端先拿 cancel() 的結果回給舊的 goal）
  StepSnapshot start(const StepGoal & goal, double now);
  // 往前推到 now，回傳目前的快照（結束後一直回傳結束的那一份）
  StepSnapshot step(double now);
  // 取消：停在原地，回傳 CANCELED 的快照（沒在執行時回傳目前的）
  StepSnapshot cancel(const std::string & detail = "canceled");
  bool busy() const {return active_;}

  PlatformState platformState() const {return platform_;}
  const std::array<double, 3> & position() const {return pos_;}
  void setFaults(const std::vector<StepFault> & faults) {config_.faults = faults;}
  void setSlowdown(double slowdown) {config_.slowdown = slowdown;}

  // 參數字串 "action=E_CODE[:capability][@at]"（例如 "inspect=E_HW:thermal_camera@0.3"）；格式錯回 nullopt
  static std::optional<StepFault> parseFault(const std::string & text);

private:
  struct Segment
  {
    std::string phase;
    bool move{false};
    std::array<double, 3> target{{0.0, 0.0, 0.0}};
    double duration{0.0};          // 不移動的段：秒
  };

  StepSnapshot finish(StepStatus status, const std::string & code, const std::string & detail,
    const std::string & capability = std::string());
  std::optional<std::string> plan(const StepGoal & goal);   // 排出 segments_；回傳錯誤
  std::optional<std::array<double, 3>> resolveLocation(const std::string & json, std::string * why) const;
  std::optional<double> zoneAltitude(const std::string & json) const;   // location 是區域、區域有 cruise_altitude_m
  void addTakeoffIfNeeded();
  void addMove(const std::string & phase, std::array<double, 3> target);
  void addHold(const std::string & phase, double seconds);
  double estimate() const;           // 估計的總時間（s）
  StepSnapshot snapshot() const;

  const Registry & registry_;
  const Site * site_;
  FakeExecutorConfig config_;
  bool uav_{false};

  PlatformState platform_{PlatformState::READY};
  std::array<double, 3> pos_{{0.0, 0.0, 0.0}};
  bool active_{false};
  StepGoal goal_;
  std::vector<Segment> segments_;
  std::size_t seg_{0};
  double seg_elapsed_{0.0};
  double started_{0.0};
  double last_{0.0};
  double deadline_{0.0};
  double total_{0.0};                // 估計的總時間（進度用）
  double done_{0.0};                 // 已完成的段的估計時間
  bool lands_{false};                // 做完變 LANDED
  std::string findings_{"[]"};
  std::optional<StepFault> fault_;
  std::string warnings_;             // 參數檢查的警告（不認得的參數）：附在結果的 detail
  std::array<double, 3> cursor_{{0.0, 0.0, 0.0}};   // 排段時的位置
  bool plan_airborne_{false};        // 排段時是否在空中
  StepSnapshot last_snapshot_;
};

}  // namespace swarm_cbba
