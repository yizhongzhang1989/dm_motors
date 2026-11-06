#pragma once

#include "dm_motor_bilateral_control/bilateral.hpp"
#include <algorithm>
#include <stdexcept>

namespace dm_motor_bilateral_control
{
// With P=D=feedforward=0 the position is unused, but the driver must still
// validate and encode it. A multi-turn observation is not an MIT setpoint.
// Joint offset maps to native zero for either direction; respect both joint
// bounds and a possibly tighter command position limit when zero is excluded.
inline dm_motor::Command neutral_mit_target(const AxisLimits & axis)
{
  axis.validate();
  const double p = dm_motor::resolve_limits(axis.mapping, axis.command).p;
  const double lower = std::max(axis.position_min, axis.offset - p);
  const double upper = std::min(axis.position_max, axis.offset + p);
  if (lower > upper) {
    throw std::invalid_argument("No encodable neutral MIT position inside joint bounds");
  }
  return {std::clamp(axis.offset, lower, upper), 0, 0, 0, 0};
}
}  // namespace dm_motor_bilateral_control
