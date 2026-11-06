#pragma once

#include "dm_motor_control/simulation/physics.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dm_motor::simulation::lugre
{
// sigma1=0. One mechanical coordinate; the bristle state carries ALL friction.
// Every accepted midpoint substep obeys uniqueness, endpoint convexity and an
// elastic-resolution condition. Neither state nor energy loss is clamped.
struct Step
{
  double travel{0}, maximum_a{0}, maximum_uniqueness_ratio{0}, maximum_energy_residual{0};
  unsigned substeps{0};
};
inline long double level(const PhysicsParameters & p, long double velocity)
{
  const long double ratio = velocity / p.stribeck_velocity;
  return p.coulomb_friction +
    (static_cast<long double>(p.static_friction) - p.coulomb_friction) * std::exp(-ratio * ratio);
}
inline Step integrate(const PhysicsParameters & p, PhysicsState & state, double drive, double dt)
{
  if (!std::isfinite(dt) || dt <= 0 || !std::isfinite(drive) ||
    !std::isfinite(state.velocity) || !std::isfinite(state.bristle_displacement) ||
    std::abs(state.bristle_displacement) > p.friction_displacement_scale)
  {
    throw std::invalid_argument("Invalid LuGre step or bristle state");
  }
  const long double J = p.inertia, Fs = p.static_friction, Fc = p.coulomb_friction;
  const long double delta = p.friction_displacement_scale, k = Fs / delta, b = p.viscous_friction;
  const long double ratio = Fs / Fc;
  const long double C = 1 + ratio + (2 / std::exp(1.L)) * ratio * (ratio - 1);
  if (!(J > 0 && Fc > 0 && Fs >= Fc && delta > 0 && b >= 0) ||
    !std::isfinite(k) || !std::isfinite(C))
  {
    throw std::invalid_argument("Invalid LuGre coefficient/bound");
  }
  auto next = state;
  auto & s = next;
  Step result;
  double remaining = dt;
  const double minimum_step = dt / 4096;
  long double total_travel = 0;
  while (remaining > 0) {
    if (result.substeps >= 4096) {
      throw std::invalid_argument("LuGre step underresolved: 4096 substep budget");
    }
    const long double v0 = s.velocity, z0 = s.bristle_displacement;
    double duration = remaining;
    long double low = 0, high = 0, uniqueness = 0;
    for (;;) {
      const long double h = duration;
      uniqueness = h * h * k * C / (4 * J);
      const long double denominator = 2 * J + h * b;
      const long double center = (2 * J * v0 + h * drive) / denominator;
      const long double radius = h * Fs / denominator;
      const long double padding = 32 * std::numeric_limits<long double>::epsilon() *
        (std::abs(center) + radius + 1e-300L);
      low = center - radius - padding;
      high = center + radius + padding;
      const long double a_bound = h * k * std::max(std::abs(low), std::abs(high)) / (2 * Fc);
      // .5 is an explicit margin inside the proved uniqueness limit <1.
      if (std::isfinite(low) && std::isfinite(high) && std::isfinite(a_bound) &&
        uniqueness <= .5L && a_bound <= 1 && h * std::sqrt(k / J) <= .25L)
      {
        break;
      }
      if (duration / 2 < minimum_step || duration / 2 == duration || duration / 2 == 0) {
        throw std::invalid_argument("LuGre step underresolved: uniqueness/endpoint/elastic bound");
      }
      duration /= 2;
    }
    const long double h = duration;
    const auto midpoint_z = [&](long double w) {
        const long double a = h * std::abs(w) * k / (2 * level(p, w));
        return (z0 + h * w / 2) / (1 + a);
      };
    const auto residual = [&](long double w) {
        return 2 * J * (w - v0) - h * drive + h * k * midpoint_z(w) + h * b * w;
      };
    if (residual(low) > 0 || residual(high) < 0) {
      throw std::runtime_error("LuGre monotone root is not bracketed");
    }
    for (unsigned iteration = 0; iteration < 96; ++iteration) {
      const long double middle = low + (high - low) / 2;
      if (middle == low || middle == high) {break;}
      const long double value = residual(middle);
      if (value == 0) {low = high = middle; break;}
      if (value > 0) {high = middle;} else {low = middle;}
    }
    const long double root = low + (high - low) / 2;
    const double v1 = static_cast<double>(2 * root - v0);
    const long double w = (v0 + static_cast<long double>(v1)) / 2;
    const long double g = level(p, w), a = h * std::abs(w) * k / (2 * g);
    if (!std::isfinite(v1) || !std::isfinite(a) || a > 1) {
      throw std::runtime_error("LuGre accepted root violates endpoint condition");
    }
    // Convex update evaluated as a lerp; no clamp even for roundoff.
    const long double target = w == 0 ? z0 : std::copysign(g / k, w);
    const long double fraction = 2 * a / (1 + a);
    const double z1 = static_cast<double>(z0 + fraction * (target - z0));
    if (!std::isfinite(z1) || std::abs(z1) > p.friction_displacement_scale) {
      throw std::runtime_error("LuGre endpoint escaped bristle invariant");
    }
    const long double zm = (z0 + static_cast<long double>(z1)) / 2;
    const long double force = k * zm, travel = h * w;
    const long double work = drive * travel;
    const long double viscous_loss = h * b * w * w;
    const long double memory_loss = h * std::abs(w) * force * force / g;
    const long double old_energy = (J * v0 * v0 + k * z0 * z0) / 2;
    const long double new_energy = (J * v1 * v1 + k * z1 * z1) / 2;
    const long double balance = new_energy - old_energy - work + viscous_loss + memory_loss;
    const long double tolerance = 512 * std::numeric_limits<double>::epsilon() *
      (old_energy + new_energy + std::abs(work) + viscous_loss + memory_loss + 1e-300L);
    if (!std::isfinite(new_energy) || !std::isfinite(memory_loss) ||
      viscous_loss < 0 || memory_loss < 0 || std::abs(balance) > tolerance)
    {
      throw std::runtime_error("LuGre midpoint work identity failed");
    }
    total_travel += travel;
    if (!std::isfinite(total_travel) || s.position + total_travel <= p.lower_limit ||
      s.position + total_travel >= p.upper_limit)
    {
      throw std::invalid_argument("LuGre trajectory would enter unsupported contact");
    }
    s.velocity = v1;
    s.bristle_displacement = z1;
    s.bristle_torque = static_cast<double>(k * z1);
    s.bristle_energy = static_cast<double>(k * z1 * z1 / 2);
    s.bristle_drive_work += static_cast<double>(
      (static_cast<long double>(drive) + p.load_torque) * travel);
    s.viscous_dissipation += static_cast<double>(viscous_loss);
    s.bristle_dissipation += static_cast<double>(memory_loss);
    result.maximum_a = std::max(result.maximum_a, static_cast<double>(a));
    result.maximum_uniqueness_ratio = std::max(result.maximum_uniqueness_ratio, static_cast<double>(uniqueness));
    result.maximum_energy_residual = std::max(result.maximum_energy_residual, static_cast<double>(std::abs(balance)));
    ++result.substeps;
    remaining -= duration;
  }
  result.travel = static_cast<double>(total_travel);
  state = next;
  return result;
}
}  // namespace dm_motor::simulation::lugre
