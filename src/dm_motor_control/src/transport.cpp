#include "dm_motor_control/core.hpp"
#include "dm_motor_control/runtime.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
namespace dm_motor
{
double steady_seconds() {return can_bus::steady_seconds();}
namespace
{
can_bus::Frame generic(const Frame & f)
{
  can_bus::Frame result;
  result.id = f.id; result.length = f.length; result.channel = f.channel;
  result.fd = f.fd; result.brs = f.brs;
  std::copy(f.data.begin(), f.data.end(), result.data.begin());
  return result;
}
std::vector<Frame> motor_frames(const std::vector<can_bus::Frame> & frames)
{
  std::vector<Frame> result;
  for (const auto & f : frames) {
    if (f.length > 8 || f.extended || f.rtr || f.error || f.id > 0x7ff) {continue;}
    Frame out;
    out.id = f.id; out.length = f.length; out.channel = f.channel; out.fd = f.fd; out.brs = f.brs;
    out.received_steady = f.received_steady;
    std::copy_n(f.data.begin(), f.length, out.data.begin()); result.push_back(out);
  }
  return result;
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
  float value; static_assert(sizeof(value) == sizeof(bits));
  std::memcpy(&value, &bits, sizeof(value)); return value;
}
class SocketCanWrapper final : public Transport
{
  std::unique_ptr<can_bus::Transport> transport_;
public:
  SocketCanWrapper(const std::string & interface, bool fd)
  : transport_(can_bus::make_socketcan_transport(interface, fd)) {}
  void send(const Frame & f) override {transport_->send(generic(f));}
  std::vector<Frame> receive() override {return motor_frames(transport_->receive());}
};
}
SerialTransport::SerialTransport(const std::string & port)
: transport_(can_bus::make_serial_transport(port)) {}
SerialTransport::~SerialTransport() = default;
void SerialTransport::send(const Frame & f) {transport_->send(generic(f));}
std::vector<Frame> SerialTransport::receive() {return motor_frames(transport_->receive());}
std::unique_ptr<Transport> make_socketcan_transport(const std::string & interface, bool fd)
{return std::make_unique<SocketCanWrapper>(interface, fd);}
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
  feedback_.mos = 28; feedback_.rotor = 29; last_time_ = steady_seconds();
}
void SimTransport::send(const Frame & f)
{
  const double now = steady_seconds();
  const double dt = std::clamp(now - last_time_, 0.0, 0.02); last_time_ = now;
  if (f.id == 0x7ff && f.data[0] == id_ && f.data[2] == 0xcc) {
    pending_.push_back(feedback_frame(id_, master_, feedback_, mapping_)); return;
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
  auto result = std::move(pending_); pending_.clear(); return result;
}
}
