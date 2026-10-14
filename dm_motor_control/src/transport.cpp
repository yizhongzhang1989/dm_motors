#include "dm_motor_control/core.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace dm_motor
{
double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
namespace
{
std::runtime_error io_error(const char * operation)
{
  return std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}
uint32_t read_register(Transport & transport, uint8_t id, uint16_t master, uint8_t rid,
  std::optional<uint32_t> value = std::nullopt)
{
  Frame f{0x7ff, {id, 0, static_cast<uint8_t>(value ? 0x55 : 0x33), rid, 0, 0, 0, 0}};
  if (value) {
    for (unsigned i = 0; i < 4; ++i) {f.data[4 + i] = (*value >> (8 * i)) & 255;}
  }
  transport.send(f);
  const double deadline = steady_seconds() + 0.5;
  while (steady_seconds() < deadline) {
    for (const auto & reply : transport.receive()) {
      if (reply.id == master && std::equal(f.data.begin(), f.data.begin() + 4, reply.data.begin())) {
        uint32_t result = 0;
        for (unsigned i = 0; i < 4; ++i) {result |= uint32_t(reply.data[4 + i]) << (8 * i);}
        if (value && result != *value) {throw std::runtime_error("watchdog write rejected");}
        return result;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  throw std::runtime_error("register " + std::to_string(rid) +
    " timed out; verify motor/master IDs, adapter, CAN bitrate and wiring");
}
float float_register(Transport & transport, uint8_t id, uint16_t master, uint8_t rid)
{
  uint32_t bits = read_register(transport, id, master, rid);
  float value;
  static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}
SerialTransport::SerialTransport(const std::string & port)
{
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {throw io_error("open serial");}
  try {
    if (flock(fd_, LOCK_EX | LOCK_NB) < 0) {throw io_error("serial already in use");}
    if (ioctl(fd_, TIOCEXCL) < 0) {throw io_error("exclusive serial access");}
    termios config{};
    if (tcgetattr(fd_, &config) < 0) {throw io_error("tcgetattr");}
    cfmakeraw(&config);
    if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0) {
      throw io_error("set baudrate");
    }
    config.c_cflag |= CLOCAL | CREAD;
    config.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd_, TCSANOW, &config) < 0 || tcflush(fd_, TCIOFLUSH) < 0) {
      throw io_error("configure serial");
    }
  } catch (...) {
    ioctl(fd_, TIOCNXCL);
    ::close(fd_);
    fd_ = -1;
    throw;
  }
}
SerialTransport::~SerialTransport()
{
  if (fd_ >= 0) {ioctl(fd_, TIOCNXCL); ::close(fd_);}
}
void SerialTransport::send(const Frame & frame)
{
  auto packet = serial_packet(frame);
  size_t offset = 0;
  const auto deadline = steady_seconds() + 0.005;
  while (offset < packet.size()) {
    auto count = ::write(fd_, packet.data() + offset, packet.size() - offset);
    if (count > 0) {offset += static_cast<size_t>(count); continue;}
    if (count < 0 && errno != EAGAIN && errno != EINTR) {throw io_error("serial write");}
    if (steady_seconds() >= deadline) {throw std::runtime_error("serial write exceeded 5 ms");}
    pollfd descriptor{fd_, POLLOUT, 0};
    if (::poll(&descriptor, 1, 1) < 0 && errno != EINTR) {throw io_error("serial poll");}
  }
}
std::vector<Frame> SerialTransport::receive()
{
  std::array<uint8_t, 4096> bytes{};
  auto count = ::read(fd_, bytes.data(), bytes.size());
  if (count < 0) {
    if (errno == EAGAIN || errno == EINTR) {return {};}
    throw io_error("serial read");
  }
  if (count == 0) {
    pollfd descriptor{fd_, POLLIN, 0};
    if (::poll(&descriptor, 1, 0) < 0) {throw io_error("serial poll");}
    if (descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)) {
      throw std::runtime_error("serial disconnected");
    }
    return {};
  }
  return parser_.feed(bytes.data(), static_cast<size_t>(count));
}
Mapping initialize_serial(Transport & transport, uint8_t id, uint16_t master)
{
  if (read_register(transport, id, master, 7) != master) {
    throw std::runtime_error("MST_ID does not match master_id");
  }
  if (read_register(transport, id, master, 10) != 1) {
    throw std::runtime_error("motor is not in MIT mode; configure it disabled before launch");
  }
  Mapping mapping{float_register(transport, id, master, 21),
    float_register(transport, id, master, 22), float_register(transport, id, master, 23)};
  validate_mapping(mapping);
  transport.send(query_frame(id));
  bool disabled = false;
  const double deadline = steady_seconds() + 0.5;
  while (steady_seconds() < deadline && !disabled) {
    for (const auto & f : transport.receive()) {
      auto fb = decode_feedback(f, id, master, mapping);
      if (fb) {
        if (fb->status != 0) {
          transport.send(special_frame(id, 0xfd));
          throw std::runtime_error("motor not disabled at startup; disable and inspect before retry");
        }
        disabled = true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!disabled) {throw std::runtime_error("initial disabled feedback not confirmed");}
  read_register(transport, id, master, 9, 4000);
  if (read_register(transport, id, master, 9) != 4000) {
    throw std::runtime_error("200 ms hardware watchdog readback failed");
  }
  return mapping;
}
SimTransport::SimTransport(uint8_t id, uint16_t master, Mapping mapping)
: id_(id), master_(master), mapping_(mapping)
{
  feedback_.mos = 28;
  feedback_.rotor = 29;
  last_time_ = steady_seconds();
}
void SimTransport::send(const Frame & f)
{
  const double now = steady_seconds();
  const double dt = std::clamp(now - last_time_, 0.0, 0.02);
  last_time_ = now;
  if (f.id == 0x7ff && f.data[0] == id_ && f.data[2] == 0xcc) {
    pending_.push_back(feedback_frame(id_, master_, feedback_, mapping_));
    return;
  }
  if (f.id != id_) {return;}
  if (std::all_of(f.data.begin(), f.data.begin() + 7, [](uint8_t b) {return b == 255;})) {
    if (f.data[7] == 0xfc) {feedback_.status = 1;}
    else if (f.data[7] == 0xfd) {feedback_.status = 0; feedback_.v = 0; feedback_.t = 0;}
    else {throw std::invalid_argument("unsupported simulated special command");}
  } else if (feedback_.status == 1) {
    const auto c = decode_mit(f, mapping_);
    feedback_.t = std::clamp(c.kp * (c.p - feedback_.p) +
      c.kd * (c.v - feedback_.v) + c.t, -mapping_.t, mapping_.t);
    feedback_.v = std::clamp(feedback_.v + (feedback_.t - 0.2 * feedback_.v) * dt,
      -mapping_.v, mapping_.v);
    feedback_.p = std::clamp(feedback_.p + feedback_.v * dt, -mapping_.p, mapping_.p);
  }
  pending_.push_back(feedback_frame(id_, master_, feedback_, mapping_));
}
std::vector<Frame> SimTransport::receive()
{
  auto result = std::move(pending_);
  pending_.clear();
  return result;
}
}  // namespace dm_motor
