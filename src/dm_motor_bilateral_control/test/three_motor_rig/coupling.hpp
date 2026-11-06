#pragma once
#include <cmath>
#include <stdexcept>

namespace three_motor_rig
{
// Passive coaxial link with opposite encoder directions: q1 + q3 = offset.
// This models an effective compliant/backlash path, not identified shaft rigidity.
struct Coupling
{
  double stiffness{40}, damping{0.02}, half_gap{0}, offset{0};
  struct Result {double torque1, torque3, energy, dissipation, extension;};
  void validate() const
  {
    if (!std::isfinite(stiffness) || stiffness <= 0 || !std::isfinite(damping) ||
      damping < 0 || !std::isfinite(half_gap) || half_gap < 0 || !std::isfinite(offset))
    {throw std::invalid_argument("Invalid passive coupling");}
  }
  Result evaluate(double q1, double v1, double q3, double v3) const
  {
    validate();
    if (!std::isfinite(q1) || !std::isfinite(v1) || !std::isfinite(q3) || !std::isfinite(v3))
    {throw std::invalid_argument("Nonfinite coupling state");}
    const double delta = q1 + q3 - offset, speed = v1 + v3;
    const double x = std::copysign(std::fmax(0., std::abs(delta) - half_gap), delta);
    const double d = std::abs(delta) > half_gap ? damping : 0.;
    const double torque = -stiffness * x - d * speed;
    return {torque, torque, .5 * stiffness * x * x, d * speed * speed, x};
  }
};
}  // namespace three_motor_rig
