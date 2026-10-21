#pragma once

#include <cstdint>

namespace dm_motor::simulation
{
struct PhysicsParameters
{
  double inertia{0.01};
  double viscous_friction{0.02}, coulomb_friction{0.03}, static_friction{0.05};
  double gravity_torque{0}, gravity_offset{0};
  double max_torque{10}, no_load_speed{50}, torque_time_constant{0.002};
  double lower_limit{-1e6}, upper_limit{1e6};
  double contact_stiffness{200}, contact_damping{2};
  double ambient_temperature{25}, heat_capacity{100}, thermal_resistance{1};
  double winding_resistance{0.8}, torque_constant{1};
  double initial_position{0}, initial_velocity{0};
  void validate() const;
};

struct PhysicsState
{
  double position{0}, velocity{0}, motor_torque{0}, external_torque{0};
  double gravity_torque{0}, friction_torque{0}, contact_torque{0}, net_torque{0};
  double rotor_temperature{25}, mos_temperature{25}, mechanical_energy{0};
  uint64_t integration_steps{0};
};

class JointPhysics
{
public:
  explicit JointPhysics(PhysicsParameters parameters);
  void step(double dt, double requested_torque, double external_torque, bool enabled);
  const PhysicsState & state() const {return state_;}
  const PhysicsParameters & parameters() const {return parameters_;}
private:
  PhysicsParameters parameters_;
  PhysicsState state_;
};
}  // namespace dm_motor::simulation
