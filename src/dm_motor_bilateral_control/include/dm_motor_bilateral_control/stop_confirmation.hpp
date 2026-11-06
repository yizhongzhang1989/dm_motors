#pragma once

#include "dm_motor_control/motor_session.hpp"
#include <cmath>

namespace dm_motor_bilateral_control
{
// Driver feedback_count is local to a process and may restart at zero. Confirm
// STOP from a new hardware acquisition instead of comparing process counters.
inline bool motor_stopped_after(
  const dm_motor::MotorSession & axis, const rclcpp::Time & ros_now,
  double steady_now, double stop_started, double timeout)
{
  if (!dm_motor::motor_state_fresh(axis, ros_now, timeout)) {return false;}
  const auto & state = *axis.state;
  const double acquired = state.feedback_receive_time;
  return std::isfinite(acquired) && std::isfinite(stop_started) && std::isfinite(steady_now) &&
         acquired > stop_started && acquired <= steady_now && steady_now - acquired <= timeout &&
         state.status == 0 &&
         (state.phase == dm_motor::MotorSession::State::DISABLED ||
         state.phase == dm_motor::MotorSession::State::FAULT) &&
         axis.token.empty() && !axis.pending;
}
}  // namespace dm_motor_bilateral_control
