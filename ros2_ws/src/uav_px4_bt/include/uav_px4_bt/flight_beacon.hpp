// 飛行層的位置廣播（避碰用，2026-10-09）：每台無人機 10 Hz 送自己的位置與速度，收到的鄰機給 CPF 用。
//
// 和 CBBA 的協定版本 2 分開（不同的 UDP 埠、不同的 magic），CBBA 的封包格式、機器狗都不受影響：
//   - AGENT_STATE 每 0.5 s 一次、沒有速度，5 m/s 時位置差到 2.5 m，不夠避碰用
//   - 避碰只需要「直接聽得到」的鄰機（會撞到的一定很近），所以不轉送、不去重複、不確認
//
// 格式（大端序，42 B）：
//   magic 0x4346（'CF'）2 | version 1 | sender_id 2 | session_id 8 | sequence 4 | flags 1 |
//   x、y、z 4×3 | vx、vy、vz 4×3（map ENU，float32）
//   flags bit 0 在空中（armed 且沒有 landed）、bit 1 位置有效、bit 2 執行任務中（前往或停留；待命的會讓路）
// 收端只保留每台最新的一筆：同一個 session 的 sequence 比較舊就丟掉；換 session（重開機）直接接受。
// 資料的年齡用收到的時間算（不用對時）。
#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <vector>

#include "uav_px4_bt/cpf.hpp"

namespace uav_px4_bt
{

constexpr std::uint16_t kBeaconMagic = 0x4346;
constexpr std::uint8_t kBeaconVersion = 1;
constexpr std::size_t kBeaconSize = 42;

struct Beacon
{
  std::uint16_t sender{0};
  std::uint64_t session{0};
  std::uint32_t sequence{0};
  bool airborne{false};
  bool position_valid{false};
  bool busy{false};
  Vec3 position;
  Vec3 velocity;
};

std::vector<std::uint8_t> encodeBeacon(const Beacon & b);
// 長度、magic、version 不對，或 sender = 0 時回傳空
std::optional<Beacon> decodeBeacon(const std::uint8_t * data, std::size_t size);

// 每台鄰機最新的一筆
class NeighborTable
{
public:
  explicit NeighborTable(std::uint16_t self_id)
  : self_id_(self_id) {}

  // 收到的封包；自己的、舊的、格式不對的回傳 false
  bool receive(const std::uint8_t * data, std::size_t size, double now);
  // 在空中、位置有效、timeout 內有更新的鄰機（age = now − 收到的時間）
  std::vector<Neighbor> neighbors(double now, double timeout) const;

private:
  struct Entry
  {
    Beacon beacon;
    double received{0.0};
  };
  std::uint16_t self_id_;
  std::map<std::uint16_t, Entry> table_;
};

}  // namespace uav_px4_bt
