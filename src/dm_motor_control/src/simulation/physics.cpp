#include "dm_motor_control/simulation/physics.hpp"
#include "dm_motor_control/simulation/lugre.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace dm_motor::simulation
{
namespace
{
struct MotionSegment
{
  double velocity, travel;
};

// Exact constant-drive/dry-friction segment with linear viscous friction.
// Small-exponent forms avoid cancellation for nearly zero fitted viscosity.
MotionSegment integrate_segment(double velocity, double duration, double inertia,
  double force, double damping)
{
  const double exponent = damping * duration / inertia;
  const double phi1 = exponent == 0 ? 1 : -std::expm1(-exponent) / exponent;
  if (exponent >= 1e-4) {
    // A very large damping exponent can have a finite equilibrium velocity
    // even when force/inertia or the exponent itself would overflow.
    const double equilibrium = force / damping;
    return {velocity * std::exp(-exponent) + equilibrium * -std::expm1(-exponent),
      velocity * duration * phi1 + equilibrium * duration * (1 - phi1)};
  }
  const double phi2 = .5 + exponent *
    (-1. / 6 + exponent * (1. / 24 + exponent * (-1. / 120 + exponent / 720)));
  const double impulse = force * duration / inertia;
  return {velocity * std::exp(-exponent) + impulse * phi1,
    velocity * duration * phi1 + impulse * duration * phi2};
}

double time_to_zero(double velocity, double inertia, double force, double damping)
{
  if (!((velocity > 0 && force < 0) || (velocity < 0 && force > 0))) {
    return std::numeric_limits<double>::infinity();
  }
  const double log_linear_time = std::log(inertia) +
    std::log(std::abs(velocity)) - std::log(std::abs(force));
  if (damping == 0) {return std::exp(log_linear_time);}
  // Form ratios in log space: velocity/force can overflow even when
  // damping*velocity/force is small (e.g. subnormal damping and force).
  const double log_ratio = std::log(damping) +
    std::log(std::abs(velocity)) - std::log(std::abs(force));
  if (log_ratio <= 0) {
    const double ratio = std::exp(log_ratio);
    const double correction = ratio == 0 ? 1 : std::log1p(ratio) / ratio;
    return std::exp(log_linear_time + std::log(correction));
  }
  const double logarithm = log_ratio + std::log1p(std::exp(-log_ratio));
  return std::exp(std::log(inertia) - std::log(damping) + std::log(logarithm));
}
}  // namespace

void PhysicsParameters::validate() const
{
  for (double value : {inertia, viscous_friction, coulomb_friction, static_friction,
      stribeck_velocity, friction_negative_scale, friction_displacement_scale,
      initial_bristle_displacement, joint_stiffness, load_torque,
      gravity_torque, gravity_offset, max_torque, no_load_speed, torque_time_constant,
      constant_torque_speed,
      lower_limit, upper_limit, contact_stiffness, contact_damping, ambient_temperature,
      heat_capacity, thermal_resistance, winding_resistance, torque_constant,
      initial_position, initial_velocity})
  {
    if (!std::isfinite(value)) {throw std::invalid_argument("Physics parameters must be finite");}
  }
  if (inertia < 1e-5 || inertia > 1e4 || max_torque <= 0 || max_torque > 1000 ||
    no_load_speed <= 0 || torque_time_constant < 0 ||
    constant_torque_speed < 0 || constant_torque_speed >= no_load_speed ||
    viscous_friction < 0 || coulomb_friction < 0 || static_friction < coulomb_friction ||
    stribeck_velocity <= 0 || friction_negative_scale <= 0 || joint_stiffness < 0 ||
    gravity_torque < 0 || lower_limit >= upper_limit || contact_stiffness < 0 || contact_damping < 0 ||
    heat_capacity <= 0 || thermal_resistance <= 0 || winding_resistance < 0 || torque_constant <= 0 ||
    ambient_temperature < -40 || ambient_temperature > 150 ||
    initial_position < lower_limit || initial_position > upper_limit)
  {
    throw std::invalid_argument("Invalid inertia, friction, actuator, thermal or joint-limit parameters");
  }
  if (friction_model != "viscous" && friction_model != "coulomb_viscous" &&
    friction_model != "stribeck" && friction_model != "lugre")
  {
    throw std::invalid_argument("friction_model must be viscous, coulomb_viscous, stribeck or lugre");
  }
  if (friction_model == "lugre") {
    const double stiffness = static_friction / friction_displacement_scale;
    if (!std::isfinite(friction_displacement_scale) || friction_displacement_scale <= 0 ||
      !std::isfinite(initial_bristle_displacement) ||
      std::abs(initial_bristle_displacement) > friction_displacement_scale ||
      coulomb_friction <= 0 || friction_negative_scale != 1 ||
      gravity_torque != 0 || joint_stiffness != 0 ||
      initial_position <= lower_limit || initial_position >= upper_limit ||
      !std::isfinite(stiffness) || stiffness <= 0 ||
      !std::isfinite(static_friction * friction_displacement_scale))
    {
      throw std::invalid_argument(
              "LuGre requires Fs>=Fc>0, finite delta>0 and |z0|<=delta, symmetric friction, "
              "no gravity/joint potential, and an initial position strictly inside limits");
    }
  }
}

JointPhysics::JointPhysics(PhysicsParameters parameters) : parameters_(parameters)
{
  parameters_.validate();
  state_.position = parameters.initial_position;
  state_.velocity = parameters.initial_velocity;
  state_.rotor_temperature = state_.mos_temperature = parameters.ambient_temperature;
  state_.mechanical_energy = parameters.inertia * state_.velocity * state_.velocity / 2 +
    parameters.gravity_torque * (1 - std::cos(state_.position + parameters.gravity_offset)) +
    parameters.joint_stiffness * state_.position * state_.position / 2 +
    parameters.load_torque * state_.position;
  if (!std::isfinite(state_.mechanical_energy)) {throw std::invalid_argument("Initial mechanical energy is not finite");}
  if (parameters.friction_model == "lugre") {
    state_.bristle_displacement = parameters.initial_bristle_displacement;
    state_.bristle_torque = parameters.static_friction *
      (state_.bristle_displacement / parameters.friction_displacement_scale);
    state_.bristle_energy = .5 * parameters.static_friction * parameters.friction_displacement_scale *
      std::pow(state_.bristle_displacement / parameters.friction_displacement_scale, 2);
  }
  state_.total_storage = state_.mechanical_energy + state_.bristle_energy;
  if (!std::isfinite(state_.total_storage)) {throw std::invalid_argument("Initial total storage is not finite");}
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
    const double speed = std::abs(s.velocity);
    if (p.constant_torque_speed == 0) {
      // Preserve the original expression and rounding for existing scenarios.
      limit *= std::max(0.0, 1.0 - speed / p.no_load_speed);
    } else if (speed > p.constant_torque_speed) {
      limit *= std::max(0.0, 1.0 -
        (speed - p.constant_torque_speed) / (p.no_load_speed - p.constant_torque_speed));
    }
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
  s.load_torque = -p.joint_stiffness * s.position - p.load_torque;
  const double drive = s.motor_torque + external + s.gravity_torque + s.contact_torque + s.load_torque;
  const double old_velocity = s.velocity;
  double travel = 0;
  if (p.friction_model == "lugre") {
    if (penetration != 0) {throw std::invalid_argument("LuGre does not support contact");}
    travel = lugre::integrate(p, s, drive, dt).travel;
  } else {
    const double static_limit = p.friction_model == "viscous" ? 0 :
      (drive < 0 ? p.friction_negative_scale : 1) * p.static_friction;
    const auto coefficients = [&](double velocity, double direction) {
      const double scale = direction < 0 ? p.friction_negative_scale : 1;
      double dry = p.friction_model == "viscous" ? 0 : p.coulomb_friction;
      if (p.friction_model == "stribeck") {
        dry += (p.static_friction - p.coulomb_friction) *
          std::exp(-std::pow(velocity / p.stribeck_velocity, 2));
      }
      return std::pair<double, double>{drive - scale * dry * std::copysign(1.0, direction),
        scale * p.viscous_friction};
    };
    double remaining = dt;
    if (old_velocity != 0) {
      const auto [force, damping] = coefficients(old_velocity, old_velocity);
      const double stop = time_to_zero(old_velocity, p.inertia, force, damping);
      if (stop <= dt) {
        travel = integrate_segment(old_velocity, stop, p.inertia, force, damping).travel;
        s.velocity = 0;
        remaining = dt - stop;
      } else {
        const auto motion = integrate_segment(old_velocity, dt, p.inertia, force, damping);
        s.velocity = motion.velocity;
        travel = motion.travel;
        remaining = 0;
      }
    }
    if (remaining > 0 && std::abs(drive) > static_limit) {
      // At zero, decide stick/slip using the load direction, then reevaluate
      // asymmetric friction and the Stribeck dry level for the new segment.
      const auto [force, damping] = coefficients(0, drive);
      const auto motion = integrate_segment(0, remaining, p.inertia, force, damping);
      s.velocity = motion.velocity;
      travel += motion.travel;
    }
  }
  // These are impulse averages over this step, including any stop/reversal.
  s.friction_torque = p.inertia * (s.velocity - old_velocity) / dt - drive;
  s.net_torque = drive + s.friction_torque;
  s.position += travel;
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
    p.joint_stiffness * s.position * s.position / 2 + p.load_torque * s.position +
    p.contact_stiffness * contact * contact / 2;
  s.total_storage = s.mechanical_energy + s.bristle_energy;
  ++s.integration_steps;
  if (!std::isfinite(s.position) || !std::isfinite(s.velocity) ||
    !std::isfinite(s.rotor_temperature) || !std::isfinite(s.mechanical_energy) ||
    !std::isfinite(s.total_storage) || !std::isfinite(s.bristle_displacement) ||
    !std::isfinite(s.bristle_torque) || !std::isfinite(s.bristle_energy) ||
    !std::isfinite(s.bristle_drive_work) || !std::isfinite(s.viscous_dissipation) ||
    !std::isfinite(s.bristle_dissipation))
  {
    throw std::runtime_error("Physics integration diverged; inspect parameters and step size");
  }
  state_ = next;
}
}  // namespace dm_motor::simulation
