#include "dm_motor_control/simulation/physics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dm_motor::simulation
{
void PhysicsParameters::validate() const
{
  for (double value : {inertia, viscous_friction, coulomb_friction, static_friction,
      gravity_torque, gravity_offset, max_torque, no_load_speed, torque_time_constant,
      lower_limit, upper_limit, contact_stiffness, contact_damping, ambient_temperature,
      heat_capacity, thermal_resistance, winding_resistance, torque_constant,
      initial_position, initial_velocity})
  {
    if (!std::isfinite(value)) {throw std::invalid_argument("Physics parameters must be finite");}
  }
  if (inertia < 1e-5 || inertia > 1e4 || max_torque <= 0 || max_torque > 1000 ||
    no_load_speed <= 0 || torque_time_constant < 0 ||
    viscous_friction < 0 || coulomb_friction < 0 || static_friction < coulomb_friction ||
    gravity_torque < 0 || lower_limit >= upper_limit || contact_stiffness < 0 || contact_damping < 0 ||
    heat_capacity <= 0 || thermal_resistance <= 0 || winding_resistance < 0 || torque_constant <= 0 ||
    ambient_temperature < -40 || ambient_temperature > 150 ||
    initial_position < lower_limit || initial_position > upper_limit)
  {
    throw std::invalid_argument("Invalid inertia, friction, actuator, thermal or joint-limit parameters");
  }
}

JointPhysics::JointPhysics(PhysicsParameters parameters) : parameters_(parameters)
{
  parameters_.validate();
  state_.position = parameters.initial_position;
  state_.velocity = parameters.initial_velocity;
  state_.rotor_temperature = state_.mos_temperature = parameters.ambient_temperature;
  state_.mechanical_energy = parameters.inertia * state_.velocity * state_.velocity / 2 +
    parameters.gravity_torque * (1 - std::cos(state_.position + parameters.gravity_offset));
  if (!std::isfinite(state_.mechanical_energy)) {throw std::invalid_argument("Initial mechanical energy is not finite");}
}

void JointPhysics::step(double dt, double requested, double external, bool enabled)
{
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.001 ||
    !std::isfinite(requested) || !std::isfinite(external))
  {
    throw std::invalid_argument("Physics step requires finite inputs and dt in (0, 0.001]");
  }
  const auto & p = parameters_;
  auto next = state_;
  auto & s = next;
  double limit = p.max_torque;
  if (requested * s.velocity > 0) {
    limit *= std::max(0.0, 1.0 - std::abs(s.velocity) / p.no_load_speed);
  }
  const double command = enabled ? std::clamp(requested, -limit, limit) : 0;
  if (!enabled) {s.motor_torque = 0;}
  else {
    const double response = p.torque_time_constant == 0 ? 1 : -std::expm1(-dt / p.torque_time_constant);
    s.motor_torque += response * (command - s.motor_torque);
  }
  s.external_torque = external;
  s.gravity_torque = -p.gravity_torque * std::sin(s.position + p.gravity_offset);
  const double penetration = s.position < p.lower_limit ? s.position - p.lower_limit :
    s.position > p.upper_limit ? s.position - p.upper_limit : 0;
  s.contact_torque = penetration == 0 ? 0 :
    -p.contact_stiffness * penetration - p.contact_damping * s.velocity;
  if (penetration > 0) {s.contact_torque = std::min(0.0, s.contact_torque);}
  if (penetration < 0) {s.contact_torque = std::max(0.0, s.contact_torque);}
  const double drive = s.motor_torque + external + s.gravity_torque + s.contact_torque;
  const double old_velocity = s.velocity;
  if (std::abs(old_velocity) < 1e-8 && std::abs(drive) <= p.static_friction) {
    s.velocity = 0;
    s.friction_torque = -drive;
  } else {
    const double sign = std::copysign(1.0, std::abs(old_velocity) > 1e-8 ? old_velocity : drive);
    const double coulomb = -p.coulomb_friction * sign;
    s.velocity = (old_velocity + dt * (drive + coulomb) / p.inertia) /
      (1 + dt * p.viscous_friction / p.inertia);
    if (old_velocity * s.velocity < 0 && std::abs(drive) <= p.static_friction) {s.velocity = 0;}
    s.friction_torque = p.inertia * (s.velocity - old_velocity) / dt - drive;
  }
  s.net_torque = drive + s.friction_torque;
  s.position += (old_velocity + s.velocity) * dt / 2;
  const double current = s.motor_torque / p.torque_constant;
  const double heating = current * current * p.winding_resistance;
  const double thermal_response = -std::expm1(-dt / (p.heat_capacity * p.thermal_resistance));
  s.rotor_temperature += thermal_response *
    (p.ambient_temperature + heating * p.thermal_resistance - s.rotor_temperature);
  s.mos_temperature = p.ambient_temperature + 0.3 * (s.rotor_temperature - p.ambient_temperature);
  const double contact = s.position < p.lower_limit ? s.position - p.lower_limit :
    s.position > p.upper_limit ? s.position - p.upper_limit : 0;
  s.mechanical_energy = p.inertia * s.velocity * s.velocity / 2 +
    p.gravity_torque * (1 - std::cos(s.position + p.gravity_offset)) +
    p.contact_stiffness * contact * contact / 2;
  ++s.integration_steps;
  if (!std::isfinite(s.position) || !std::isfinite(s.velocity) ||
    !std::isfinite(s.rotor_temperature) || !std::isfinite(s.mechanical_energy))
  {
    throw std::runtime_error("Physics integration diverged; inspect parameters and step size");
  }
  state_ = next;
}
}  // namespace dm_motor::simulation
