#include "cbba_core/task_meta.hpp"

#include <cmath>
#include <cstdio>

namespace cbba_core
{

const char * toString(TaskSource source)
{
  switch (source) {
    case TaskSource::UNKNOWN: return "UNKNOWN";
    case TaskSource::ALARM: return "ALARM";
    case TaskSource::UAV: return "UAV";
    case TaskSource::QUADRUPED: return "QUADRUPED";
    case TaskSource::VMS: return "VMS";
    case TaskSource::OPERATOR: return "OPERATOR";
    case TaskSource::SENSOR: return "SENSOR";
  }
  return "UNKNOWN";
}

const char * toString(TargetKind kind)
{
  switch (kind) {
    case TargetKind::NONE: return "none";
    case TargetKind::POINT: return "point";
    case TargetKind::AREA: return "area";
    case TargetKind::MOVING: return "moving";
  }
  return "none";
}

namespace
{
bool finite(const Vec3 & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

std::string number(double v)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.3f", v);
  return buf;
}

std::string pose(const Vec3 & p, std::uint8_t floor)
{
  return "{\"frame\":\"map\",\"floor_id\":" + std::to_string(floor) + ",\"x\":" + number(p.x) + ",\"y\":" +
         number(p.y) + ",\"z\":" + number(p.z) + "}";
}
}  // namespace

double standoffDistance(const TaskMeta & meta, const Vec3 & observe)
{
  if (meta.target_kind == TargetKind::NONE) {
    return 0.0;
  }
  const double d = std::hypot(observe.x - meta.target.x, observe.y - meta.target.y);
  return meta.target_kind == TargetKind::AREA ? d - meta.uncertainty_m : d;
}

std::string checkMeta(const TaskMeta & meta, const Vec3 & observe)
{
  if (static_cast<std::uint8_t>(meta.source) > kMaxTaskSource) {
    return "source 要是 0～6（UNKNOWN、ALARM、UAV、QUADRUPED、VMS、OPERATOR、SENSOR）";
  }
  if (meta.description.size() > kMaxDescription) {
    return "description 最多 " + std::to_string(kMaxDescription) + " B";
  }
  if (static_cast<std::uint8_t>(meta.target_kind) > static_cast<std::uint8_t>(TargetKind::MOVING)) {
    return "target_kind 要是 0（沒有）、1（point）、2（area）、3（moving）";
  }
  if (!std::isfinite(meta.uncertainty_m) || meta.uncertainty_m < 0.0 || !std::isfinite(meta.standoff_m) ||
    meta.standoff_m < 0.0)
  {
    return "uncertainty_m、standoff_m 要 ≥ 0";
  }
  if (!validEvidenceUri(meta.evidence_uri)) {
    return "evidence_uri 要是 evidence://、https://、http://、s3://、file:// 開頭、≤ 512 B、沒有空白";
  }
  if (!std::isfinite(meta.evidence_confidence) || meta.evidence_confidence < 0.0 || meta.evidence_confidence > 1.0) {
    return "evidence_confidence 要在 0～1";
  }
  if (meta.track_id.size() > kMaxTrackId) {
    return "track_id 最多 " + std::to_string(kMaxTrackId) + " B";
  }
  if (meta.target_kind == TargetKind::NONE) {
    // 沒有另外的目標：觀察點就是目標，安全距離、不確定半徑、追蹤都沒有意義
    if (meta.standoff_m > 0.0 || meta.uncertainty_m > 0.0 || !meta.track_id.empty()) {
      return "target_kind 是 0（沒有目標）時不能給 standoff_m、uncertainty_m、track_id";
    }
    return {};
  }
  if (!finite(meta.target)) {
    return "target 不是有限值";
  }
  if (meta.target_kind == TargetKind::MOVING && meta.track_id.empty()) {
    return "target_kind moving 要給 track_id";
  }
  if (meta.target_kind == TargetKind::AREA && meta.uncertainty_m <= 0.0) {
    return "target_kind area 要給 uncertainty_m（區域半徑）";
  }
  const double d = standoffDistance(meta, observe);
  if (meta.standoff_m > 0.0 && d + kStandoffTolerance < meta.standoff_m) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "觀察點（position）離目標%s只有 %.2f m，小於 standoff_m %.2f m（規劃器要把觀察點往外推）",
      meta.target_kind == TargetKind::AREA ? "區域邊緣" : "", d, meta.standoff_m);
    return buf;
  }
  return {};
}

std::string jsonString(const std::string & text)
{
  std::string out = "\"";
  for (const char ch : text) {
    const auto c = static_cast<unsigned char>(ch);
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += ch;   // UTF-8 照原樣
        }
    }
  }
  return out + "\"";
}

std::string metaJsonMembers(const TaskMeta & meta, const Vec3 & observe, std::uint8_t floor)
{
  std::string out = "\"schema_version\":" + jsonString(kRoleSpecSchema);
  const auto add = [&out](const std::string & key, const std::string & value) {
      out += "," + jsonString(key) + ":" + value;
    };
  add("incident_id", jsonString(meta.incident_id));
  add("source", jsonString(toString(meta.source)));
  add("source_id", std::to_string(static_cast<unsigned>(meta.source)));
  if (!meta.event_type.empty()) {
    add("event_type", jsonString(meta.event_type));
    add("event_type_id", std::to_string(meta.event_type_id));
  }
  if (!meta.task_type.empty()) {
    add("task_type", jsonString(meta.task_type));
    add("task_type_id", std::to_string(meta.task_type_id));
  }
  if (!meta.role.empty()) {
    add("role", jsonString(meta.role));
    add("role_id", std::to_string(meta.role_id));
  }
  if (!meta.description.empty()) {
    add("description", jsonString(meta.description));
  }
  if (!meta.evidence_uri.empty()) {
    add("evidence", "{\"uri\":" + jsonString(meta.evidence_uri) + ",\"confidence\":" + number(meta.evidence_confidence) + "}");
  }
  add("observe_pose", pose(observe, floor));
  if (meta.target_kind != TargetKind::NONE) {
    add("target_kind", jsonString(toString(meta.target_kind)));
    add("target", pose(meta.target, floor));
    if (!meta.track_id.empty()) {
      add("track_id", jsonString(meta.track_id));
    }
    if (meta.uncertainty_m > 0.0) {
      add("uncertainty_m", number(meta.uncertainty_m));
    }
    if (meta.standoff_m > 0.0) {
      add("standoff_m", number(meta.standoff_m));
    }
  }
  return out;
}

const std::vector<std::string> & reservedSpecKeys()
{
  static const std::vector<std::string> keys = {
    "schema_version", "incident_id", "source", "source_id", "event_type", "event_type_id", "task_type",
    "task_type_id", "role", "role_id", "description", "observe_pose", "target_kind", "target", "track_id",
    "uncertainty_m", "standoff_m", "evidence"};
  return keys;
}

bool validEvidenceUri(const std::string & uri)
{
  if (uri.empty()) {
    return true;
  }
  if (uri.size() > kMaxUri || uri.find_first_of(" \t\r\n") != std::string::npos) {
    return false;
  }
  for (const char * scheme : {"evidence://", "https://", "http://", "s3://", "file://"}) {
    const std::string s(scheme);
    if (uri.size() > s.size() && uri.compare(0, s.size(), s) == 0) {
      return true;
    }
  }
  return false;
}

std::optional<std::uint16_t> specSchemaMajor(const std::string & version)
{
  const auto dot = version.find('.');
  const std::string major = version.substr(0, dot);
  if (major.empty() || major.size() > 4 || major.find_first_not_of("0123456789") != std::string::npos) {
    return std::nullopt;
  }
  if (dot != std::string::npos) {
    const std::string minor = version.substr(dot + 1);
    if (minor.empty() || minor.find_first_not_of("0123456789") != std::string::npos) {
      return std::nullopt;
    }
  }
  return static_cast<std::uint16_t>(std::stoul(major));
}

}  // namespace cbba_core
