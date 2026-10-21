#pragma once

#include "dm_motor_control/simulation/physics.hpp"
#include <dm_motor_control/runtime.hpp>

#include <map>
#include <string>
#include <vector>

namespace dm_motor::simulation
{
struct DeviceParameters
{
  dm_motor::MotorConfig motor;
  PhysicsParameters physics;
  double integration_step{0.0005}, bus_voltage{48};
  double velocity_kp{1}, velocity_ki{0}, position_kp{8}, position_ki{0};
  double anchor_stiffness{0.5};
  std::string persistence_file;
};

struct SpringAnchor
{
  bool active{false};
  double position{0}, bearing{0};
  uint64_t revision{0};
};

class VirtualMotor
{
public:
  explicit VirtualMotor(DeviceParameters parameters, double time = 0);
  void advance(double time);
  std::vector<dm_motor::Frame> receive(const dm_motor::Frame & frame, double time);
  void external_torque(double torque, double valid_for, double time);
  void external_spring(double position, double stiffness, double damping,
    double valid_for, double time);
  void set_anchor(double bearing);
  void clear_anchor();
  const SpringAnchor & anchor() const {return anchor_;}
  double anchor_torque() const;
  double transient_torque() const {return external_remaining() > 0 ? external_ : 0;}
  void inject_fault(uint8_t status);
  void set_voltage(double voltage);
  void power_cycle();
  uint32_t register_value(uint8_t rid) const;
  dm_motor::Feedback feedback() const;
  const PhysicsState & state() const {return physics_.state();}
  const DeviceParameters & parameters() const {return parameters_;}
  double encoder_position() const {return state().position - zero_offset_;}
  double external_remaining() const {return std::max(0.0, external_deadline_ - time_);}
  double voltage() const {return voltage_;}
  double time() const {return time_;}
  uint8_t status() const {return status_;}
  uint64_t commands() const {return commands_;}
private:
  double commanded_torque(double dt);
  uint8_t physical_fault() const;
  dm_motor::Frame feedback_packet() const;
  void save();
  void restore_registers();
  DeviceParameters parameters_;
  JointPhysics physics_;
  dm_motor::Target target_;
  std::map<uint8_t, uint32_t> registers_, saved_;
  double time_, last_command_, zero_offset_{0}, saved_offset_{0};
  double external_{0}, external_deadline_{0}, voltage_;
  double spring_position_{0}, spring_stiffness_{0}, spring_damping_{0};
  bool spring_{false};
  SpringAnchor anchor_;
  double velocity_integral_{0}, position_integral_{0};
  uint8_t status_{0}, injected_fault_{0};
  uint64_t commands_{0};
};
}  // namespace dm_motor::simulation
