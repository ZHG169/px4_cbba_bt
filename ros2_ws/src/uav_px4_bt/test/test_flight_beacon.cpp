// 飛行層的位置廣播：格式、鄰機表（自己的、舊的、逾時、地面上的不算）
#include <gtest/gtest.h>

#include <vector>

#include "uav_px4_bt/flight_beacon.hpp"

using namespace uav_px4_bt;

namespace
{
Beacon make(std::uint16_t sender, std::uint64_t session, std::uint32_t seq, double x)
{
  Beacon b;
  b.sender = sender;
  b.session = session;
  b.sequence = seq;
  b.airborne = true;
  b.position_valid = true;
  b.position = {x, 2.0, 5.0};
  b.velocity = {1.0, -1.0, 0.5};
  return b;
}
}  // namespace

TEST(FlightBeacon, RoundTrip)
{
  // 編碼後長度固定 42 B、大端序，解回來相同
  Beacon b = make(3, 0x0123456789ABCDEFull, 77, 12.5);
  b.busy = true;
  const auto bytes = encodeBeacon(b);
  ASSERT_EQ(bytes.size(), kBeaconSize);
  EXPECT_EQ(bytes[0], 0x43);
  EXPECT_EQ(bytes[1], 0x46);
  const auto d = decodeBeacon(bytes.data(), bytes.size());
  ASSERT_TRUE(d);
  EXPECT_EQ(d->sender, 3);
  EXPECT_EQ(d->session, 0x0123456789ABCDEFull);
  EXPECT_EQ(d->sequence, 77u);
  EXPECT_TRUE(d->airborne);
  EXPECT_TRUE(d->busy);
  EXPECT_FLOAT_EQ(static_cast<float>(d->position.x), 12.5f);
  EXPECT_FLOAT_EQ(static_cast<float>(d->velocity.z), 0.5f);
}

TEST(FlightBeacon, RejectsMalformed)
{
  // 長度、magic、版本不對、sender = 0：丟掉（CBBA 的封包送到這個埠也不會被誤認）
  auto bytes = encodeBeacon(make(3, 1, 1, 0));
  EXPECT_FALSE(decodeBeacon(bytes.data(), bytes.size() - 1));
  auto bad = bytes;
  bad[0] = 0x42;
  EXPECT_FALSE(decodeBeacon(bad.data(), bad.size()));
  bad = bytes;
  bad[2] = 2;
  EXPECT_FALSE(decodeBeacon(bad.data(), bad.size()));
  const auto zero = encodeBeacon(make(0, 1, 1, 0));
  EXPECT_FALSE(decodeBeacon(zero.data(), zero.size()));
}

TEST(FlightBeacon, TableKeepsLatestPerSender)
{
  // 自己的不收；同一個 session 舊的序號丟掉；換 session（重開機）直接接受
  NeighborTable table(1);
  const auto own = encodeBeacon(make(1, 5, 1, 0));
  EXPECT_FALSE(table.receive(own.data(), own.size(), 0.0));

  const auto a = encodeBeacon(make(2, 5, 10, 1.0));
  const auto older = encodeBeacon(make(2, 5, 9, 2.0));
  const auto rebooted = encodeBeacon(make(2, 6, 0, 3.0));
  EXPECT_TRUE(table.receive(a.data(), a.size(), 0.0));
  EXPECT_FALSE(table.receive(older.data(), older.size(), 0.05));
  auto n = table.neighbors(0.1, 1.0);
  ASSERT_EQ(n.size(), 1u);
  EXPECT_DOUBLE_EQ(n[0].position.x, 1.0);
  EXPECT_NEAR(n[0].age, 0.1, 1e-9);
  EXPECT_TRUE(table.receive(rebooted.data(), rebooted.size(), 0.2));
  n = table.neighbors(0.2, 1.0);
  ASSERT_EQ(n.size(), 1u);
  EXPECT_DOUBLE_EQ(n[0].position.x, 3.0);
}

TEST(FlightBeacon, TableDropsStaleAndGrounded)
{
  // 逾時的、在地面上的、位置無效的都不給 CPF
  NeighborTable table(1);
  Beacon ground = make(3, 1, 1, 0);
  ground.airborne = false;
  Beacon invalid = make(4, 1, 1, 0);
  invalid.position_valid = false;
  for (const Beacon & b : {make(2, 1, 1, 0), ground, invalid}) {
    const auto bytes = encodeBeacon(b);
    table.receive(bytes.data(), bytes.size(), 0.0);
  }
  EXPECT_EQ(table.neighbors(0.5, 1.0).size(), 1u);
  EXPECT_TRUE(table.neighbors(1.5, 1.0).empty());
}
