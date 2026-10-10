#include "cbba_core/udp_link.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace cbba_core
{

namespace
{
[[noreturn]] void fail(const std::string & what)
{
  throw std::runtime_error("UdpLink: " + what + ": " + std::strerror(errno));
}

in_addr parseIp(const std::string & ip, const char * what)
{
  in_addr addr{};
  if (inet_pton(AF_INET, ip.c_str(), &addr) != 1) {
    throw std::runtime_error(std::string("UdpLink: invalid ") + what + " '" + ip + "'");
  }
  return addr;
}
}  // namespace

UdpLink::UdpLink(const UdpConfig & config)
{
  const in_addr group = parseIp(config.group, "multicast group");
  in_addr iface{};
  iface.s_addr = htonl(INADDR_ANY);
  if (!config.interface_ip.empty()) {
    iface = parseIp(config.interface_ip, "interface ip");
  }
  group_addr_ = group.s_addr;
  port_ = htons(config.port);

  fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    fail("socket");
  }
  const int one = 1;
  setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  setsockopt(fd_, SOL_SOCKET, SO_REUSEPORT, &one, sizeof(one));   // 同一台主機多個節點

  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_port = port_;
  local.sin_addr = group;   // 綁群組位址：只收這個群組的封包
  if (bind(fd_, reinterpret_cast<sockaddr *>(&local), sizeof(local)) < 0) {
    fail("bind");
  }

  ip_mreq mreq{};
  mreq.imr_multiaddr = group;
  mreq.imr_interface = iface;
  if (setsockopt(fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
    fail("IP_ADD_MEMBERSHIP");
  }
  if (!config.interface_ip.empty() &&
    setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_IF, &iface, sizeof(iface)) < 0)
  {
    fail("IP_MULTICAST_IF");
  }
  const unsigned char ttl = static_cast<unsigned char>(config.ttl);
  setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
  const unsigned char loop = config.loopback ? 1 : 0;
  setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

  const int flags = fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    fail("O_NONBLOCK");
  }
}

UdpLink::~UdpLink()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool UdpLink::send(const std::vector<std::uint8_t> & bytes)
{
  sockaddr_in dest{};
  dest.sin_family = AF_INET;
  dest.sin_port = port_;
  dest.sin_addr.s_addr = group_addr_;
  const ssize_t n = sendto(fd_, bytes.data(), bytes.size(), 0,
    reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
  return n == static_cast<ssize_t>(bytes.size());
}

std::vector<std::vector<std::uint8_t>> UdpLink::receiveAll()
{
  std::vector<std::vector<std::uint8_t>> out;
  std::uint8_t buf[1500];
  for (;;) {
    const ssize_t n = recv(fd_, buf, sizeof(buf), 0);
    if (n < 0) {
      break;   // EAGAIN：目前沒有了
    }
    out.emplace_back(buf, buf + n);
  }
  return out;
}

}  // namespace cbba_core
