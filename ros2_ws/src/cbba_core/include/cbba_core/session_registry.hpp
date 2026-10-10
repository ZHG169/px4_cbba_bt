// 成員與 session 層的介面（2026-10-09：只定義，還沒接上）
//
// 目前 wire::SessionFilter 看到新的 session_id 就當成對方重開機、直接切換，任何資料封包都能觸發切換：
// 偽造或延遲的封包、同機號的第二個程序都會把真的那台的 session 退役。要改成：
//
//   收到資料封包：
//     沒通過認證                         → 丟掉
//     session 不是這個機號已接受的 session → 丟掉，請它註冊（資料封包不能觸發切換）
//     否則                               → 序號檢查（滑動視窗）→ 交給協定層
//
//   新 session 只能經由註冊接受，註冊要驗證來源與新鮮性（不能只是一個沒驗證的 HELLO）：
//     簽章＋時間戳（各機本來就要對時；在有效期內被重放的註冊沒有傷害，因為是同一個 session）
//   舊 session 還活著（lost_timeout 內有聽到）→ 拒絕新的，回報身分衝突，保留現有連線
//   舊 session 逾時，或收到經驗證的交接／重新註冊 → 才切換
//
// 這樣不需要跨重啟保存的 boot_epoch：只有舊的已經沒聲音才接受新的，不用判斷哪個 session 比較新。
// 同一台機器上的重複啟動由 cbba_node 的程序鎖（flock）擋掉。
//
// 待確認：實機 mesh 的形式（802.11s、batman-adv、IBSS…：決定轉送放在哪一層）、
// 認證方式（連結層 SAE／WPA3；要防隊內被入侵的節點時再加應用層 Ed25519 簽章）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "cbba_core/types.hpp"

namespace cbba_core
{

enum class PacketAuth : std::uint8_t
{
  NOT_CONFIGURED,   // 還沒有認證（目前的狀態）
  VERIFIED,
  FAILED,
};

// 封包的認證（例如 Ed25519 簽章）。只用成熟的程式庫（libsodium 等），不自己設計加密
class PacketAuthenticator
{
public:
  virtual ~PacketAuthenticator() = default;
  // 在封包後面附上認證資料
  virtual void sign(std::vector<std::uint8_t> & packet) const = 0;
  // 驗證封包（含附加的認證資料）確實來自 claimed
  virtual PacketAuth verify(const std::uint8_t * data, std::size_t size, AgentId claimed) const = 0;
};

struct SessionRegistration
{
  AgentId agent{kNoAgent};
  std::uint64_t session_id{0};
  std::uint64_t timestamp_ms{0};    // 新鮮性：和本機時鐘差太多就拒絕
  bool handover{false};             // 經驗證的交接（舊 session 主動交出）
};

enum class RegistrationResult : std::uint8_t
{
  ACCEPTED,
  ALREADY_ACCEPTED,       // 同一個 session 重送註冊
  CONFLICT_OLD_ALIVE,     // 舊 session 還活著：拒絕新的，保留現有連線，回報身分衝突
  STALE,                  // 時間戳太舊或太新
  AUTH_FAILED,
};

struct IdentityConflict
{
  AgentId agent{kNoAgent};
  std::uint64_t accepted_session{0};
  std::uint64_t rejected_session{0};
  double first_seen{0.0};
  double last_seen{0.0};
};

// 每個機號已接受的 session、存活判斷、註冊與衝突
class SessionRegistry
{
public:
  virtual ~SessionRegistry() = default;

  // 資料封包：只有這個機號已接受的 session 才交給協定層
  virtual bool acceptsData(AgentId agent, std::uint64_t session_id) const = 0;
  // 已接受的 session 有聽到（存活判斷用）
  virtual void onHeard(AgentId agent, std::uint64_t session_id, double now) = 0;
  // 收到註冊（已通過認證）
  virtual RegistrationResult onRegistration(const SessionRegistration & reg, double now) = 0;
  virtual std::optional<std::uint64_t> acceptedSession(AgentId agent) const = 0;
  virtual std::vector<IdentityConflict> conflicts() const = 0;
};

}  // namespace cbba_core
