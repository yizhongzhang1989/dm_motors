#pragma once

#include <dm_motor_bilateral_control/bilateral.hpp>

namespace bench_control
{
using namespace dm_motor_bilateral_control;

// Sensor-only boundary: physical truth and external forces never enter this type.
struct Input
{
  std::array<Sample, 2> latest;
  std::array<double, 2> origins, filtered_velocity, friction;
  std::array<AxisLimits, 2> limits;
  Settings settings;
  double recovery{1};
  double angle_offset{0};
};

struct Result
{
  Output output;
  std::array<double, 2> coupling{};
  bool headroom_clipped{false};
};

inline dm_motor::Frame wire(uint8_t id, const dm_motor::Command & command, const AxisLimits & axis)
{
  return dm_motor::mit_frame(id,
    {axis.direction * (command.p - axis.offset), axis.direction * command.v,
      command.kp, command.kd, axis.direction * command.t}, axis.mapping);
}

inline Result compute(const Input & input)
{
  auto reference = input.latest;
  for (size_t axis = 0; axis < 2; ++axis) {
    reference[axis].velocity = input.filtered_velocity[axis];
  }
  auto desired = shared_reference_targets(reference, input.origins, input.settings);
  if (input.angle_offset != 0) {
    desired = apply_angle_offset(desired, input.angle_offset, input.settings);
  }
  for (size_t axis = 0; axis < 2; ++axis) {
    desired[axis].t = input.friction[axis];
  }
  Result result;
  result.output = symmetric_targets(
    desired, input.latest, input.limits, input.settings, input.recovery);
  result.coupling = result.output.coupling_torque;
  result.headroom_clipped = result.output.coupling_scale + 1e-14 < result.output.gain_scale;
  return result;
}
}  // namespace bench_control
