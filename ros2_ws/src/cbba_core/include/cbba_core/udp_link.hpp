// 機間通訊的 UDP multicast（Linux，不依賴 ROS）
//
// 規格：每架以無線廣播送出，一次送出、範圍內都收得到（模擬中為 UDP multicast）。
// 指定 interface_ip（mesh 網卡）時，送出與加入群組都只用這張網卡，
// 不會跑到每台無人機和 PX4 之間的線材網路。
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace cbba_core
{

struct UdpConfig
{
  std::string group{"239.255.42.99"};
  std::uint16_t port{14600};
  std::string interface_ip;   // mesh 網卡的 IP；空字串表示由系統決定
  int ttl{1};                 // 只送到同一個網段
  bool loopback{true};        // 同一台主機跑多個節點時需要；自己的封包由收端依機號略過
};

class UdpLink
{
public:
  explicit UdpLink(const UdpConfig & config);   // 失敗時丟出 std::runtime_error
  ~UdpLink();
  UdpLink(const UdpLink &) = delete;
  UdpLink & operator=(const UdpLink &) = delete;

  bool send(const std::vector<std::uint8_t> & bytes);
  // 非阻塞：取出目前收到的所有封包
  std::vector<std::vector<std::uint8_t>> receiveAll();

private:
  int fd_{-1};
  std::uint32_t group_addr_{0};   // network byte order
  std::uint16_t port_{0};
};

}  // namespace cbba_core
