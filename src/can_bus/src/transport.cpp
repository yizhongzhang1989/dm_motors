#include "can_bus/transport.hpp"
#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>
namespace can_bus
{
double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
void validate_frame(const Frame & f, bool allow_fd)
{
  if (f.id > (f.extended ? CAN_EFF_MASK : CAN_SFF_MASK) || f.channel != 0 ||
    f.length > (f.fd ? 64 : 8) || (f.fd && (!allow_fd || f.rtr)) ||
    (f.brs && !f.fd) || f.error)
  {throw std::invalid_argument("unsupported CAN ID, channel, flags or payload length");}
  if (f.fd && f.length > 8 && f.length != 12 && f.length != 16 && f.length != 20 &&
    f.length != 24 && f.length != 32 && f.length != 48 && f.length != 64)
  {throw std::invalid_argument("CAN-FD length must match a wire DLC");}
}
std::array<uint8_t, 30> serial_packet(const Frame & f)
{
  validate_frame(f, false);
  if (f.length != 8 || f.extended || f.rtr) {
    throw std::invalid_argument("legacy USB2CAN requires standard classical eight-byte frames");
  }
  std::array<uint8_t, 30> p{};
  p[0] = 0x55; p[1] = 0xaa; p[2] = 30; p[3] = 3;
  p[4] = 1; p[8] = 10; p[13] = f.id & 255; p[14] = f.id >> 8; p[18] = 8;
  std::copy_n(f.data.begin(), 8, p.begin() + 21);
  return p;
}
Frame legacy_serial_frame(const Frame & f)
{
  Frame result = f;
  validate_frame(result, false);
  if (result.length == 4) {
    std::fill(result.data.begin() + 4, result.data.end(), 0);
    result.length = 8;
  }
  serial_packet(result);
  return result;
}
std::vector<Frame> SerialParser::feed(const uint8_t * bytes, size_t size)
{
  if (size > 4096 || buffer_.size() + size > 8192) {
    buffer_.clear(); throw std::runtime_error("USB2CAN receive buffer overflow");
  }
  if (size) {buffer_.insert(buffer_.end(), bytes, bytes + size);}
  std::vector<Frame> result;
  while (buffer_.size() >= 16) {
    if (buffer_[0] != 0xaa || buffer_[15] != 0x55) {
      buffer_.erase(buffer_.begin()); continue;
    }
    const auto cmd = buffer_[1];
    if (cmd == 1 || cmd == 2 || cmd == 3 || cmd == 0xee) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + 16);
      throw std::runtime_error("USB2CAN adapter reported error " + std::to_string(cmd));
    }
    if (cmd == 0x11 && buffer_[2] == 8) {
      Frame f;
      for (unsigned i = 0; i < 4; ++i) {f.id |= uint32_t(buffer_[3 + i]) << (8 * i);}
      if (f.id <= CAN_SFF_MASK) {
        std::copy_n(buffer_.begin() + 7, 8, f.data.begin());
        f.received_steady = static_cast<uint64_t>(steady_seconds() * 1e9);
        result.push_back(f);
      }
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + 16);
  }
  return result;
}
namespace
{
std::runtime_error io_error(const char * what)
{return std::runtime_error(std::string(what) + ": " + std::strerror(errno));}
class Serial final : public Transport
{
  int fd_{-1};
  SerialParser parser_;
public:
  explicit Serial(const std::string & port)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {throw io_error("open serial");}
    try {
      if (flock(fd_, LOCK_EX | LOCK_NB) < 0 || ioctl(fd_, TIOCEXCL) < 0) {
        throw io_error("exclusive serial access");
      }
      termios config{};
      if (tcgetattr(fd_, &config) < 0) {throw io_error("tcgetattr");}
      cfmakeraw(&config);
      if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0) {
        throw io_error("serial baudrate");
      }
      config.c_cflag |= CLOCAL | CREAD; config.c_cflag &= ~CRTSCTS;
      if (tcsetattr(fd_, TCSANOW, &config) < 0 || tcflush(fd_, TCIOFLUSH) < 0) {
        throw io_error("configure serial");
      }
    } catch (...) {::close(fd_); fd_ = -1; throw;}
  }
  ~Serial() override {if (fd_ >= 0) {ioctl(fd_, TIOCNXCL); ::close(fd_);}}
  void send(const Frame & f) override
  {
    const auto p = serial_packet(legacy_serial_frame(f));
    const auto n = ::write(fd_, p.data(), p.size());
    if (n < 0) {throw io_error("serial write");}
    if (size_t(n) != p.size()) {throw std::runtime_error("partial serial write; bus latched");}
  }
  std::vector<Frame> receive() override
  {
    std::array<uint8_t, 4096> bytes{};
    const auto n = ::read(fd_, bytes.data(), bytes.size());
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {return {};}
      throw io_error("serial read");
    }
    if (!n) {
      pollfd p{fd_, POLLIN, 0};
      if (::poll(&p, 1, 0) < 0) {throw io_error("serial poll");}
      if (p.revents & (POLLHUP | POLLERR | POLLNVAL)) {
        throw std::runtime_error("serial disconnected");
      }
      return {};
    }
    return parser_.feed(bytes.data(), size_t(n));
  }
};
class SocketCan final : public Transport
{
  int fd_{-1}, lock_{-1};
  bool allow_fd_;
public:
  SocketCan(const std::string & interface, bool fd) : allow_fd_(fd)
  {
    if (interface.empty() || interface.size() >= IFNAMSIZ ||
      interface.find('/') != std::string::npos) {throw std::invalid_argument("invalid CAN interface");}
    // Abstract UNIX socket provides process lifetime exclusivity without lock files.
    lock_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (lock_ < 0) {throw io_error("CAN owner lock");}
    try {
      struct {sa_family_t family; char path[108];} owner{};
      owner.family = AF_UNIX;
      const std::string key = "can_bus." + interface;
      std::copy(key.begin(), key.end(), owner.path + 1);
      if (::bind(lock_, reinterpret_cast<sockaddr *>(&owner),
        sizeof(sa_family_t) + key.size() + 1) < 0) {throw io_error("CAN interface already owned");}
      fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
      if (fd_ < 0) {throw io_error("SocketCAN socket");}
      if (fd) {
        int one = 1;
        if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &one, sizeof(one)) < 0) {
          throw io_error("enable CAN-FD");
        }
      }
      can_err_mask_t errors = CAN_ERR_MASK;
      if (setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0) {
        throw io_error("CAN error filter");
      }
      sockaddr_can address{};
      address.can_family = AF_CAN;
      address.can_ifindex = static_cast<int>(if_nametoindex(interface.c_str()));
      if (!address.can_ifindex || ::bind(fd_, reinterpret_cast<sockaddr *>(&address),
        sizeof(address)) < 0) {throw io_error("SocketCAN bind");}
    } catch (...) {
      if (fd_ >= 0) {::close(fd_);}
      ::close(lock_); throw;
    }
  }
  ~SocketCan() override {::close(fd_); ::close(lock_);}
  void send(const Frame & f) override
  {
    validate_frame(f, allow_fd_);
    canfd_frame wire{};
    wire.can_id = f.id | (f.extended ? CAN_EFF_FLAG : 0) | (f.rtr ? CAN_RTR_FLAG : 0);
    wire.len = f.length; wire.flags = f.brs ? CANFD_BRS : 0;
    std::copy_n(f.data.begin(), f.length, wire.data);
    const size_t size = f.fd ? CANFD_MTU : CAN_MTU;
    const auto n = ::write(fd_, &wire, size);
    if (n < 0) {throw io_error("SocketCAN write");}
    if (size_t(n) != size) {throw std::runtime_error("partial SocketCAN write");}
  }
  std::vector<Frame> receive() override
  {
    std::vector<Frame> result;
    for (unsigned i = 0; i < 256; ++i) {
      canfd_frame wire{};
      const auto n = ::read(fd_, &wire, sizeof(wire));
      if (n < 0) {
        if (errno == EAGAIN || errno == EINTR) {break;}
        throw io_error("SocketCAN read");
      }
      if (n != CAN_MTU && n != CANFD_MTU) {throw std::runtime_error("invalid SocketCAN frame");}
      if (wire.can_id & CAN_ERR_FLAG) {throw std::runtime_error("SocketCAN bus error");}
      Frame f;
      f.extended = wire.can_id & CAN_EFF_FLAG;
      f.rtr = wire.can_id & CAN_RTR_FLAG;
      f.id = wire.can_id & (f.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
      f.fd = n == CANFD_MTU; f.brs = f.fd && (wire.flags & CANFD_BRS);
      f.length = wire.len;
      validate_frame(f, allow_fd_);
      std::copy_n(wire.data, f.length, f.data.begin());
      f.received_steady = static_cast<uint64_t>(steady_seconds() * 1e9);
      result.push_back(f);
    }
    return result;
  }
};
}
std::unique_ptr<Transport> make_serial_transport(const std::string & port)
{return std::make_unique<Serial>(port);}
std::unique_ptr<Transport> make_socketcan_transport(const std::string & interface, bool fd)
{return std::make_unique<SocketCan>(interface, fd);}
}
