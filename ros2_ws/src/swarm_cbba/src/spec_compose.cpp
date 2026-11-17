#include "swarm_cbba/spec_compose.hpp"

#include <yaml-cpp/yaml.h>

namespace swarm_cbba
{

namespace
{
std::optional<std::string> failWith(std::string * error, const std::string & why)
{
  if (error != nullptr) {
    *error = why;
  }
  return std::nullopt;
}
}  // namespace

std::optional<std::string> composeSpec(const std::string & planner_spec, const cbba_core::TaskMeta & meta,
  const cbba_core::Vec3 & observe, std::uint8_t floor, std::string * error)
{
  if (planner_spec.empty()) {
    return std::string();
  }
  const auto open = planner_spec.find_first_not_of(" \t\r\n");
  if (open == std::string::npos || planner_spec[open] != '{') {
    return failWith(error, "spec 要是 JSON 物件（{…}）");
  }
  YAML::Node root;
  try {
    root = YAML::Load(planner_spec);
  } catch (const YAML::Exception & e) {
    return failWith(error, std::string("spec 不是合法的 JSON：") + e.what());
  }
  if (!root.IsMap()) {
    return failWith(error, "spec 要是 JSON 物件（{…}）");
  }
  for (const std::string & key : cbba_core::reservedSpecKeys()) {
    const YAML::Node v = root[key];
    if (!v) {
      continue;
    }
    if (key == "role" && v.IsScalar() && v.as<std::string>() == meta.role) {
      continue;   // 和注入的 role 一樣：照收（組好之後有兩個一樣的 role，見下面）
    }
    if (key == "schema_version" && v.IsScalar() && v.as<std::string>() == cbba_core::kRoleSpecSchema) {
      continue;
    }
    return failWith(error, "spec 的 " + key + " 由閘道填（注入欄位），規劃器不要自己帶" +
             (key == "role" || key == "schema_version" ? std::string("（或要和注入的一樣）") : std::string()));
  }
  std::string members = cbba_core::metaJsonMembers(meta, observe, floor);
  // 規劃器已經寫了一樣的 role、schema_version：閘道這邊就不寫，避免重複的鍵
  const auto drop = [&members](const std::string & key, const std::string & value_json) {
      const std::string item = cbba_core::jsonString(key) + ":" + value_json;
      const auto at = members.find(item);
      if (at == std::string::npos) {
        return;
      }
      // 第一個成員（schema_version）後面接逗號，其他前面有逗號
      if (at == 0) {
        members.erase(0, item.size() + (members.size() > item.size() ? 1 : 0));
      } else {
        members.erase(at - 1, item.size() + 1);
      }
    };
  if (root["schema_version"]) {
    drop("schema_version", cbba_core::jsonString(cbba_core::kRoleSpecSchema));
  }
  if (root["role"]) {
    drop("role", cbba_core::jsonString(meta.role));
  }
  const auto next = planner_spec.find_first_not_of(" \t\r\n", open + 1);
  const bool empty_object = next != std::string::npos && planner_spec[next] == '}';
  return planner_spec.substr(0, open + 1) + members + (empty_object ? "" : ",") + planner_spec.substr(open + 1);
}

std::string specSchemaVersion(const std::string & spec)
{
  try {
    const YAML::Node root = YAML::Load(spec);
    if (root.IsMap() && root["schema_version"] && root["schema_version"].IsScalar()) {
      return root["schema_version"].as<std::string>();
    }
  } catch (const YAML::Exception &) {
  }
  return {};
}

}  // namespace swarm_cbba
