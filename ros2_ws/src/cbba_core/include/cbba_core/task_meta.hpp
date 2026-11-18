// 任務的識別、語意與空間資訊（任務欄位補齊第一批：01 識別與追溯、02 任務語意、04 空間；2026-11-17，
// docs/planner_task_definition_plan.md「TaskCore 最終定義與逐列規劃」）
//
// 這些欄位**不進 TASK_ANNOUNCE**（TaskCore 只放出價、可行性、協同要的；使用者 11/17 定），由閘道寫進每個角色的
// TaskSpec（B 層，current_task.spec）最上層，給 BT、GCS、紀錄用：
//   schema_version  角色 TaskSpec 的版本（kRoleSpecSchema；BT 看主版本，不認得的主版本不執行）
//   incident_id     事件識別（規劃器給；和閘道的冪等鍵相同）
//   source          事件來源（固定列舉，只增不改）：0 UNKNOWN、1 ALARM、2 UAV、3 QUADRUPED、4 VMS、5 OPERATOR、6 SENSOR
//   event_type／task_type／role  名稱＋編號（registry 的 event_types.yaml、task_types.yaml；編號只增不改，0 = 不知道）
//   description     人看的說明（GCS、log）
//   observe_pose    這個角色真正要到的位置（＝ TASK_ANNOUNCE 的 x、y、z，出價、TDR 都算到這裡）
//   target、target_kind（point／area／moving）、track_id、uncertainty_m、standoff_m  事件（目標）本身的位置與形狀
//   evidence        {"uri", "confidence"}（任務欄位補齊 03，2026-11-18）：只放證據的參照，影像、VLM 結果本身留在控制中心
//                  （閘道另外記整筆事件證據，發 incident_evidence）；CBBA 封包不帶
//
// 規劃器負責把觀察點放在離目標 standoff_m 以外（安全距離）；閘道只檢查距離夠不夠，不幫忙推。
// 這裡不依賴 JSON 函式庫：metaJsonMembers() 只產生最上層的成員字串，由節點層插進規劃器給的 spec。
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "cbba_core/types.hpp"

namespace cbba_core
{

constexpr const char * kRoleSpecSchema = "1.1";   // 1.0 = 只有 steps（11/13～11/16）
constexpr std::uint16_t kRoleSpecSchemaMajor = 1;

enum class TaskSource : std::uint8_t
{
  UNKNOWN = 0,
  ALARM = 1,       // 固定的告警（門禁、SCADA 告警點）
  UAV = 2,         // 無人機的偵測
  QUADRUPED = 3,   // 機器狗的偵測
  VMS = 4,         // 影像管理系統的分析
  OPERATOR = 5,    // 值班人員下的
  SENSOR = 6,      // 其他感測器（氣體、溫度）
};
constexpr std::uint8_t kMaxTaskSource = 6;
const char * toString(TaskSource source);

enum class TargetKind : std::uint8_t
{
  NONE = 0,     // 沒有另外的目標：觀察點就是目標（舊的做法、巡檢點）
  POINT = 1,    // 固定的點（設備、遺留物）
  AREA = 2,     // 一塊區域（煙霧範圍）：uncertainty_m 是半徑
  MOVING = 3,   // 會動的（人員）：要有 track_id，位置是最後看到的
};
const char * toString(TargetKind kind);

struct TaskMeta
{
  std::string incident_id;
  TaskSource source{TaskSource::UNKNOWN};
  std::string event_type;        // 空 = 不指定
  std::uint16_t event_type_id{0};
  std::string task_type;
  std::uint16_t task_type_id{0};
  std::string role;              // 空 = 單獨任務
  std::uint16_t role_id{0};
  std::string description;
  TargetKind target_kind{TargetKind::NONE};
  Vec3 target{};                 // map ENU；target_kind = NONE 時不用
  std::string track_id;          // MOVING 必填
  double uncertainty_m{0.0};     // 目標位置的不確定半徑（AREA：區域半徑）
  double standoff_m{0.0};        // 觀察點至少離目標多遠；0 = 不檢查
  std::string evidence_uri;      // 證據的參照（evidence:// 、https:// …）；空 = 沒有
  double evidence_confidence{0.0};   // 0～1
};

constexpr std::size_t kMaxDescription = 200;   // B
constexpr std::size_t kMaxTrackId = 64;
constexpr double kStandoffTolerance = 0.05;    // m：規劃器算出的觀察點剛好在 standoff_m 上時的數值誤差
constexpr std::size_t kMaxUri = 512;

// 證據的 URI（任務欄位補齊 03）：scheme 是 evidence（控制中心的證據庫，evidence://<incident_id>/<種類>/<檔名>）、
// https、http、s3、file 之一，≤ kMaxUri B，不能有空白。空字串 = 沒有（合法）
bool validEvidenceUri(const std::string & uri);

// 欄位檢查（閘道注入前）；observe 是 TASK_ANNOUNCE 的位置（觀察點）。空字串 = 合法
std::string checkMeta(const TaskMeta & meta, const Vec3 & observe);

// 觀察點到目標的水平距離（AREA 從區域邊緣算：減掉半徑）；NONE 時 0
double standoffDistance(const TaskMeta & meta, const Vec3 & observe);

// 角色 TaskSpec 最上層的成員（不含大括號，逗號分隔，例：`"schema_version":"1.1","incident_id":"ev_1",...`），
// 空的欄位不寫。observe 是觀察點、floor 是 A 層的樓層。節點層插在規劃器的 spec 物件最前面
std::string metaJsonMembers(const TaskMeta & meta, const Vec3 & observe, std::uint8_t floor);

// 由閘道寫入的最上層鍵：規劃器的 spec 不能自己帶（role、schema_version 例外：一樣的話照收、不重寫）
const std::vector<std::string> & reservedSpecKeys();

// JSON 字串跳脫（含引號）
std::string jsonString(const std::string & text);

// schema_version 的主版本（"1.1" → 1）；格式不對回 nullopt
std::optional<std::uint16_t> specSchemaMajor(const std::string & version);

}  // namespace cbba_core
