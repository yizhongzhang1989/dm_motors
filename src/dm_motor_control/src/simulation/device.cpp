#include "dm_motor_control/simulation/device.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace dm_motor::simulation
{
namespace
{
uint32_t bits(double value)
{
  float input = static_cast<float>(value);
  uint32_t raw;
  std::memcpy(&raw, &input, sizeof(raw));
  return raw;
}
double number(uint32_t raw)
{
  float result;
  std::memcpy(&result, &raw, sizeof(result));
  return result;
}
uint32_t raw_at(const dm_motor::Frame & frame, unsigned offset)
{
  uint32_t raw = 0;
  for (unsigned i = 0; i < 4; ++i) {raw |= uint32_t(frame.data[offset + i]) << (8 * i);}
  return raw;
}
void put_raw(dm_motor::Frame & frame, uint32_t raw)
{
  for (unsigned i = 0; i < 4; ++i) {frame.data[4 + i] = (raw >> (8 * i)) & 255;}
}
}

VirtualMotor::VirtualMotor(DeviceParameters parameters, double time)
: parameters_(parameters), physics_(parameters.physics), time_(time), last_command_(time),
  voltage_(parameters.bus_voltage)
{
  dm_motor::validate_mapping(parameters.motor.mapping);
  dm_motor::control_id(parameters.motor.id, parameters.motor.mode);
  if (parameters.motor.master > 0x7fe || !std::isfinite(time) ||
    !std::isfinite(parameters.integration_step) || parameters.integration_step < 1e-5 ||
    parameters.integration_step > 0.001 ||
    !std::isfinite(voltage_) || voltage_ < 0 || voltage_ > 1000)
  {
    throw std::invalid_argument("Invalid device ID, integration step or voltage");
  }
  for (double gain : {parameters.velocity_kp, parameters.velocity_ki,
      parameters.position_kp, parameters.position_ki})
  {
    if (!std::isfinite(gain) || gain < 0) {throw std::invalid_argument("Invalid simulated servo gains");}
  }
  if (!std::isfinite(parameters.anchor_stiffness) ||
    parameters.anchor_stiffness <= 0 || parameters.anchor_stiffness > 100)
  {
    throw std::invalid_argument("Anchor stiffness must be finite and in (0,100]");
  }
  for (const auto & info : dm_motor::register_catalog()) {registers_[info.rid] = 0;}
  registers_[0] = bits(12); registers_[1] = bits(parameters.physics.torque_constant);
  registers_[2] = bits(100); registers_[3] = bits(0.5);
  registers_[4] = bits(100); registers_[5] = bits(-100);
  registers_[6] = bits(parameters.physics.no_load_speed);
  registers_[7] = parameters.motor.master; registers_[8] = parameters.motor.id;
  registers_[9] = 0; registers_[10] = static_cast<uint32_t>(parameters.motor.mode);
  registers_[11] = bits(parameters.physics.viscous_friction);
  registers_[12] = bits(parameters.physics.inertia);
  registers_[13] = 0x53494d31; registers_[14] = 0x00010000; registers_[15] = parameters.motor.id;
  registers_[16] = 14; registers_[17] = bits(parameters.physics.winding_resistance);
  registers_[18] = bits(0.001); registers_[19] = bits(0.01); registers_[20] = bits(1);
  registers_[21] = bits(parameters.motor.mapping.p);
  registers_[22] = bits(parameters.motor.mapping.v);
  registers_[23] = bits(parameters.motor.mapping.t);
  registers_[24] = bits(1000); registers_[25] = bits(parameters.velocity_kp);
  registers_[26] = bits(parameters.velocity_ki); registers_[27] = bits(parameters.position_kp);
  registers_[28] = bits(parameters.position_ki); registers_[29] = bits(60);
  registers_[30] = bits(1); registers_[31] = bits(1); registers_[32] = bits(100);
  registers_[33] = bits(1000); registers_[34] = bits(1000);
  registers_[35] = 4; registers_[36] = 1; registers_[37] = 1;
  registers_[55] = bits(1);
  registers_[59] = bits(parameters.physics.max_torque / parameters.physics.torque_constant);
  saved_ = registers_;
  if (!parameters.persistence_file.empty() && std::filesystem::exists(parameters.persistence_file)) {
    const auto image = YAML::LoadFile(parameters.persistence_file);
    if (image["version"].as<int>() != 1 || !image["registers"].IsMap()) {
      throw std::runtime_error("Invalid simulation flash image");
    }
    for (const auto & info : dm_motor::register_catalog()) {
      const auto raw = image["registers"][std::to_string(info.rid)].as<uint32_t>();
      if (info.writable) {dm_motor::validate_register_value(info.rid, raw);}
      saved_.at(info.rid) = raw;
    }
    if (saved_.at(7) != parameters.motor.master || saved_.at(8) != parameters.motor.id) {
      throw std::runtime_error("Simulation flash image motor identity does not match launch configuration");
    }
    saved_offset_ = image["zero_offset"].as<double>();
    if (!std::isfinite(saved_offset_)) {throw std::runtime_error("Invalid saved zero offset");}
    registers_ = saved_;
    zero_offset_ = saved_offset_;
    restore_registers();
  }
  target_.mode = parameters_.motor.mode;
}

void VirtualMotor::restore_registers()
{
  const auto mode = registers_.at(10);
  if (mode < 1 || mode > 4) {throw std::runtime_error("Invalid saved control mode");}
  parameters_.motor.mode = static_cast<dm_motor::Mode>(mode);
  parameters_.motor.master = registers_.at(7);
  parameters_.motor.id = registers_.at(8);
  parameters_.motor.mapping = {
    number(registers_.at(21)), number(registers_.at(22)), number(registers_.at(23))};
  dm_motor::validate_mapping(parameters_.motor.mapping);
}

uint8_t VirtualMotor::physical_fault() const
{
  if (injected_fault_) {return injected_fault_;}
  if (voltage_ < number(registers_.at(0))) {return 9;}
  if (voltage_ > number(registers_.at(29))) {return 8;}
  if (state().mos_temperature >= number(registers_.at(2))) {return 11;}
  if (state().rotor_temperature >= number(registers_.at(2))) {return 12;}
  return 0;
}

double VirtualMotor::commanded_torque(double dt)
{
  using dm_motor::Mode;
  if (target_.mode == Mode::Mit) {
    return target_.kp * (target_.position - encoder_position()) +
           target_.kd * (target_.velocity - state().velocity) + target_.torque;
  }
  double desired_velocity = target_.velocity;
  if (target_.mode != Mode::Velocity) {
    const double error = target_.position - encoder_position();
    position_integral_ = std::clamp(position_integral_ + error * dt, -100.0, 100.0);
    desired_velocity = std::clamp(
      number(registers_.at(27)) * error + number(registers_.at(28)) * position_integral_,
      -target_.velocity_limit, target_.velocity_limit);
  }
  const double error = desired_velocity - state().velocity;
  const double previous = velocity_integral_;
  velocity_integral_ = std::clamp(velocity_integral_ + error * dt, -100.0, 100.0);
  const double torque = number(registers_.at(25)) * error + number(registers_.at(26)) * velocity_integral_;
  double limit = parameters_.physics.max_torque;
  if (target_.mode == Mode::PositionForce) {limit *= target_.current_ratio;}
  if (std::abs(torque) > limit && error * torque > 0) {velocity_integral_ = previous;}
  return std::clamp(torque, -limit, limit);
}

void VirtualMotor::advance(double time)
{
  if (!std::isfinite(time) || time < time_) {throw std::invalid_argument("Simulation time must be finite and monotonic");}
  if (time - time_ > 2) {
    time_ = time; status_ = 13; external_ = 0; external_deadline_ = time; spring_ = false;
    clear_anchor();
    throw std::runtime_error("Simulation stalled over two seconds; dynamics not extrapolated, device fault latched");
  }
  while (time_ + 1e-12 < time) {
    const double dt = std::min(parameters_.integration_step, time - time_);
    const auto fault = physical_fault();
    if (fault) {status_ = fault;}
    const double watchdog = registers_.at(9) * 50e-6;
    if (status_ == 1 && watchdog > 0 && time_ - last_command_ >= watchdog) {status_ = 13;}
    if (time_ >= external_deadline_) {external_ = 0; spring_ = false;}
    try {
      if (spring_) {
        external_ = spring_stiffness_ * (spring_position_ - state().position) -
          spring_damping_ * state().velocity;
      }
      physics_.step(dt, status_ == 1 ? commanded_torque(dt) : 0, external_ + anchor_torque(), status_ == 1);
    } catch (const std::exception &) {
      time_ = time; status_ = 13; external_ = 0; external_deadline_ = time; spring_ = false;
      clear_anchor();
      throw;
    }
    time_ += dt;
  }
  time_ = time;
  if (time_ >= external_deadline_) {external_ = 0; spring_ = false;}
}

void VirtualMotor::external_torque(double torque, double valid_for, double time)
{
  if (!std::isfinite(torque) ||
    !std::isfinite(valid_for) || valid_for <= 0 || valid_for > 5)
  {
    throw std::invalid_argument("External torque must be finite and valid_for must be in (0,5]");
  }
  advance(time);
  external_ = torque; external_deadline_ = time + valid_for; spring_ = false;
}

void VirtualMotor::external_spring(
  double position, double stiffness, double damping, double valid_for, double time)
{
  if (!std::isfinite(position) || !std::isfinite(stiffness) || stiffness <= 0 || stiffness > 100 ||
    !std::isfinite(damping) || damping < 0 || damping > 20 ||
    !std::isfinite(valid_for) || valid_for <= 0 || valid_for > 5)
  {
    throw std::invalid_argument("Invalid external spring position, gains or validity");
  }
  advance(time);
  spring_ = true; spring_position_ = position; spring_stiffness_ = stiffness;
  spring_damping_ = damping; external_deadline_ = time + valid_for;
}

void VirtualMotor::set_anchor(double bearing)
{
  if (!std::isfinite(bearing)) {throw std::invalid_argument("Anchor bearing must be finite");}
  if (anchor_.active) {throw std::runtime_error("Remove the existing anchor before creating another");}
  anchor_.active = true;
  anchor_.position = state().position;
  anchor_.bearing = std::remainder(bearing, 2 * std::acos(-1.0));
  ++anchor_.revision;
}

void VirtualMotor::clear_anchor()
{
  anchor_.active = false;
  ++anchor_.revision;
}

double VirtualMotor::anchor_torque() const
{
  return anchor_.active ? parameters_.anchor_stiffness * (anchor_.position - state().position) : 0;
}

void VirtualMotor::inject_fault(uint8_t status)
{
  if (status == 1 || status == 2 || status > 14 || (status > 5 && status < 8)) {
    throw std::invalid_argument("fault_status must be 0, 3..5 or 8..14");
  }
  injected_fault_ = status;
  if (status) {status_ = status;}
}

void VirtualMotor::set_voltage(double voltage)
{
  if (!std::isfinite(voltage) || voltage < 0 || voltage > 1000) {
    throw std::invalid_argument("Supply voltage must be finite in 0..1000 V");
  }
  voltage_ = voltage;
}

void VirtualMotor::power_cycle()
{
  if (status_ == 1) {throw std::runtime_error("Power cycle requires a disabled simulated motor");}
  registers_ = saved_; zero_offset_ = saved_offset_;
  restore_registers();
  status_ = injected_fault_ = 0;
  target_ = {}; target_.mode = parameters_.motor.mode;
  velocity_integral_ = position_integral_ = 0;
  external_ = 0; external_deadline_ = time_; spring_ = false;
  last_command_ = time_;
}

void VirtualMotor::save()
{
  if (!parameters_.persistence_file.empty()) {
    YAML::Node image;
    image["version"] = 1;
    image["zero_offset"] = zero_offset_;
    for (const auto & item : registers_) {image["registers"][std::to_string(item.first)] = item.second;}
    const auto temporary = parameters_.persistence_file + ".tmp";
    std::ofstream file(temporary, std::ios::trunc);
    file << image;
    file.close();
    if (!file) {throw std::runtime_error("Cannot write simulated flash image: " + temporary);}
    std::filesystem::rename(temporary, parameters_.persistence_file);
  }
  saved_ = registers_; saved_offset_ = zero_offset_;
}

uint32_t VirtualMotor::register_value(uint8_t rid) const
{
  if (!registers_.count(rid)) {throw std::invalid_argument("Unknown device register");}
  if (rid == 56) {return bits(zero_offset_);}
  if (rid == 60) {return bits(voltage_);}
  if (rid == 61) {return bits(state().mos_temperature);}
  if (rid == 62) {return bits(state().rotor_temperature);}
  if (rid == 80 || rid == 81) {return bits(encoder_position());}
  return registers_.at(rid);
}

dm_motor::Feedback VirtualMotor::feedback() const
{
  const auto & mapping = parameters_.motor.mapping;
  const double period = 2 * mapping.p * 65536 / 65535;
  double wrapped = std::fmod(encoder_position() + mapping.p, period);
  if (wrapped < 0) {wrapped += period;}
  return {status_, std::clamp(wrapped - mapping.p, -mapping.p, mapping.p),
    std::clamp(state().velocity, -mapping.v, mapping.v),
    std::clamp(state().motor_torque, -mapping.t, mapping.t),
    static_cast<uint8_t>(std::clamp(std::round(state().mos_temperature), 0.0, 255.0)),
    static_cast<uint8_t>(std::clamp(std::round(state().rotor_temperature), 0.0, 255.0))};
}

dm_motor::Frame VirtualMotor::feedback_packet() const
{
  const auto & motor = parameters_.motor;
  return dm_motor::feedback_frame(motor.id, motor.master, feedback(), motor.mapping);
}

std::vector<dm_motor::Frame> VirtualMotor::receive(const dm_motor::Frame & frame, double time)
{
  using dm_motor::Mode;
  advance(time);
  if (frame.channel != 0 || frame.id > 0x7ff || frame.length > 8 || (frame.brs && !frame.fd)) {
    throw std::invalid_argument("Unsupported virtual motor CAN frame");
  }
  auto & motor = parameters_.motor;
  std::vector<dm_motor::Frame> replies;
  if (frame.id == 0x7ff && (frame.length == 4 || frame.length == 8) &&
    frame.data[0] == motor.id && frame.data[1] == 0)
  {
    const auto opcode = frame.data[2], rid = frame.data[3];
    if (opcode == 0xcc) {replies.push_back(feedback_packet());}
    else if (opcode == 0xaa && rid == 1) {
      if (status_ != 0) {throw std::runtime_error("Save requires disabled device");}
      save();
      auto reply = frame; reply.id = motor.master; replies.push_back(reply);
    } else if (opcode == 0x33 || opcode == 0x55) {
      if (!registers_.count(rid)) {throw std::invalid_argument("Unknown register");}
      if (opcode == 0x55) {
        if (status_ != 0 || frame.length != 8) {throw std::runtime_error("Register write requires disabled device");}
        const auto raw = raw_at(frame, 4);
        dm_motor::validate_register_value(rid, raw);
        if ((rid == 8 && (raw < 1 || raw > 15)) || (rid == 7 && raw > 0x7fe) ||
          (rid == 10 && (raw < 1 || raw > 4)))
        {
          throw std::invalid_argument("Unsupported motor identity or mode");
        }
        registers_.at(rid) = raw;
        restore_registers();
        if (rid == 10) {
          target_ = {}; target_.mode = motor.mode;
          velocity_integral_ = position_integral_ = 0;
        }
      }
      auto reply = frame; reply.id = motor.master; reply.length = 8;
      put_raw(reply, register_value(rid)); replies.push_back(reply);
    } else {throw std::invalid_argument("Unsupported parameter opcode");}
  } else if (frame.id == dm_motor::control_id(motor.id, motor.mode)) {
    if (frame.length == 8 && std::all_of(frame.data.begin(), frame.data.begin() + 7,
        [](uint8_t value) {return value == 255;}))
    {
      const auto opcode = frame.data[7];
      if (opcode == 0xfc) {
        if (status_ == 0 && !physical_fault()) {status_ = 1; last_command_ = time_;}
      } else if (opcode == 0xfd) {
        if (status_ <= 1) {status_ = 0;}
        target_ = {}; target_.mode = motor.mode;
        velocity_integral_ = position_integral_ = 0;
      } else if (opcode == 0xfb) {
        if (status_ != 1 && !physical_fault()) {status_ = 0;}
      } else if (opcode == 0xfe) {
        if (status_ != 0 || std::abs(state().velocity) > 0.1) {
          throw std::runtime_error("Device zero requires disabled stationary device");
        }
        zero_offset_ = state().position;
      } else {throw std::invalid_argument("Unsupported special command");}
    } else {
      dm_motor::Target target; target.mode = motor.mode;
      if (motor.mode == Mode::Mit && frame.length == 8) {
        const auto decoded = dm_motor::decode_mit(frame, motor.mapping);
        target.position = decoded.p; target.velocity = decoded.v;
        target.kp = decoded.kp; target.kd = decoded.kd; target.torque = decoded.t;
      } else if (motor.mode == Mode::Velocity && (frame.length == 4 || frame.length == 8)) {
        target.velocity = number(raw_at(frame, 0));
      } else if (motor.mode != Mode::Mit && frame.length == 8) {
        target.position = number(raw_at(frame, 0));
        if (motor.mode == Mode::PositionVelocity) {target.velocity_limit = number(raw_at(frame, 4));}
        else {
          target.velocity_limit = (frame.data[4] | (frame.data[5] << 8)) / 100.0;
          target.current_ratio = (frame.data[6] | (frame.data[7] << 8)) / 10000.0;
        }
      } else {throw std::invalid_argument("Target payload length does not match motor mode");}
      // MIT bit fields already decode within the firmware ranges; float modes need validation.
      if (motor.mode != Mode::Mit) {dm_motor::target_frame(motor.id, target, motor.mapping);}
      target_ = target; last_command_ = time_; ++commands_;
    }
    replies.push_back(feedback_packet());
  }
  for (auto & reply : replies) {reply.fd = frame.fd; reply.brs = frame.brs;}
  return replies;
}
}  // namespace dm_motor::simulation
