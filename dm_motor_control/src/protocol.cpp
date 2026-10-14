#include "dm_motor_control/core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dm_motor
{
namespace
{
void range(double value, double low, double high, const char * name)
{
  if (!std::isfinite(value) || value < low || value > high) {
    throw std::invalid_argument(std::string(name) + " outside configured range");
  }
}
uint32_t quantize(double value, double low, double high, unsigned bits)
{
  range(value, low, high, "MIT value");
  return static_cast<uint32_t>((value - low) / (high - low) * ((1u << bits) - 1));
}
double unquantize(uint32_t value, double low, double high, unsigned bits)
{
  return low + value * (high - low) / ((1u << bits) - 1);
}
}
void validate_mapping(const Mapping & m)
{
  range(m.p, 1e-6, 1e6, "PMAX");
  range(m.v, 1e-6, 1e6, "VMAX");
  range(m.t, 1e-6, 1e6, "TMAX");
}
Limits resolve_limits(const Mapping & m, Limits l)
{
  validate_mapping(m);
  if (l.p == 0) {l.p = m.p;}
  if (l.v == 0) {l.v = m.v;}
  if (l.t == 0) {l.t = m.t;}
  range(l.p, 0, m.p, "position_limit");
  range(l.v, 0, m.v, "velocity_limit");
  range(l.t, 0, m.t, "torque_limit");
  range(l.kp, 0, 500, "kp_limit");
  range(l.kd, 0, 5, "kd_limit");
  return l;
}
void validate_command(const Command & c, const Mapping & m, const Limits & requested)
{
  const auto l = resolve_limits(m, requested);
  range(c.p, -l.p, l.p, "position");
  range(c.v, -l.v, l.v, "velocity");
  range(c.t, -l.t, l.t, "torque");
  range(c.kp, 0, l.kp, "kp");
  range(c.kd, 0, l.kd, "kd");
  mit_frame(1, c, m);
}
Frame mit_frame(uint8_t id, const Command & c, const Mapping & m)
{
  const auto p = quantize(c.p, -m.p, m.p, 16);
  const auto v = quantize(c.v, -m.v, m.v, 12);
  const auto kp = quantize(c.kp, 0, 500, 12);
  const auto kd = quantize(c.kd, 0, 5, 12);
  const auto t = quantize(c.t, -m.t, m.t, 12);
  Frame f;
  f.id = id;
  f.data = {static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
    static_cast<uint8_t>(v >> 4), static_cast<uint8_t>((v << 4) | (kp >> 8)),
    static_cast<uint8_t>(kp), static_cast<uint8_t>(kd >> 4),
    static_cast<uint8_t>((kd << 4) | (t >> 8)), static_cast<uint8_t>(t)};
  if (std::all_of(f.data.begin(), f.data.begin() + 7, [](uint8_t b) {return b == 0xff;})) {
    throw std::invalid_argument("MIT target collides with special-command prefix; change a target field");
  }
  return f;
}
Command decode_mit(const Frame & f, const Mapping & m)
{
  const auto & d = f.data;
  return {unquantize((d[0] << 8) | d[1], -m.p, m.p, 16),
    unquantize((d[2] << 4) | (d[3] >> 4), -m.v, m.v, 12),
    unquantize(((d[3] & 15) << 8) | d[4], 0, 500, 12),
    unquantize((d[5] << 4) | (d[6] >> 4), 0, 5, 12),
    unquantize(((d[6] & 15) << 8) | d[7], -m.t, m.t, 12)};
}
Frame special_frame(uint8_t id, uint8_t opcode)
{
  Frame f;
  f.id = id;
  f.data.fill(0xff);
  f.data[7] = opcode;
  return f;
}
Frame query_frame(uint8_t id)
{
  return {0x7ff, {id, 0, 0xcc, 0, 0, 0, 0, 0}};
}
Frame feedback_frame(uint8_t id, uint16_t master, const Feedback & fb, const Mapping & m)
{
  auto p = quantize(fb.p, -m.p, m.p, 16);
  auto v = quantize(fb.v, -m.v, m.v, 12);
  auto t = quantize(fb.t, -m.t, m.t, 12);
  return {master, {static_cast<uint8_t>(id | (fb.status << 4)),
    static_cast<uint8_t>(p >> 8), static_cast<uint8_t>(p),
    static_cast<uint8_t>(v >> 4), static_cast<uint8_t>((v << 4) | (t >> 8)),
    static_cast<uint8_t>(t), fb.mos, fb.rotor}};
}
std::optional<Feedback> decode_feedback(
  const Frame & f, uint8_t id, uint16_t master, const Mapping & m)
{
  const auto & d = f.data;
  if (f.id != master || (d[0] & 15) != id) {return std::nullopt;}
  return Feedback{static_cast<uint8_t>(d[0] >> 4),
    unquantize((d[1] << 8) | d[2], -m.p, m.p, 16),
    unquantize((d[3] << 4) | (d[4] >> 4), -m.v, m.v, 12),
    unquantize(((d[4] & 15) << 8) | d[5], -m.t, m.t, 12), d[6], d[7]};
}
std::array<uint8_t, 30> serial_packet(const Frame & f)
{
  if (f.id > 0x7ff) {throw std::invalid_argument("invalid standard CAN ID");}
  std::array<uint8_t, 30> packet{};
  packet[0] = 0x55; packet[1] = 0xaa; packet[2] = 30; packet[3] = 3;
  packet[4] = 1; packet[8] = 10;
  packet[13] = f.id & 255; packet[14] = f.id >> 8;
  packet[18] = 8;
  std::copy(f.data.begin(), f.data.end(), packet.begin() + 21);
  return packet;
}
std::vector<Frame> SerialParser::feed(const uint8_t * data, size_t length)
{
  if (length > 4096 || buffer_.size() + length > 8192) {
    buffer_.clear();
    throw std::runtime_error("USB2CAN receive buffer overflow");
  }
  buffer_.insert(buffer_.end(), data, data + length);
  std::vector<Frame> frames;
  while (buffer_.size() >= 16) {
    if (buffer_[0] != 0xaa || buffer_[15] != 0x55) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const auto cmd = buffer_[1];
    if (cmd == 0x01 || cmd == 0x02 || cmd == 0x03 || cmd == 0xee) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + 16);
      throw std::runtime_error("USB2CAN adapter reported error " + std::to_string(cmd));
    }
    if (cmd == 0x11 && buffer_[2] == 8) {
      uint32_t id = 0;
      for (unsigned i = 0; i < 4; ++i) {id |= uint32_t(buffer_[3 + i]) << (8 * i);}
      if (id <= 0x7ff) {
        Frame f;
        f.id = static_cast<uint16_t>(id);
        std::copy_n(buffer_.begin() + 7, 8, f.data.begin());
        frames.push_back(f);
      }
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + 16);
  }
  return frames;
}
}  // namespace dm_motor
