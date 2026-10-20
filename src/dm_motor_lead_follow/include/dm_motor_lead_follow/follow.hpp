#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dm_motor_lead_follow
{
struct Settings
{
  int direction{1};
  double kp{8.0}, kd{0.5};
  bool motion_limits_enabled{false};
  double target_speed{0.5}, max_error{0.5}, max_travel{2.0};
  double max_speed{2.0}, max_torque{3.0}, max_temperature{55.0};

  void validate() const
  {
    if (direction != 1 && direction != -1) {
      throw std::invalid_argument("direction must be 1 or -1");
    }
    if (!std::isfinite(kp) || kp < 0.1 || kp > 500 ||
      !std::isfinite(kd) || kd < 0 || kd > 5)
    {
      throw std::invalid_argument("kp must be 0.1..500 and kd must be 0..5");
    }
    for (double value : {target_speed, max_error, max_travel, max_speed, max_torque, max_temperature}) {
      if (!std::isfinite(value) || value <= 0) {
        throw std::invalid_argument("motion limits must be finite and positive");
      }
    }
    if (motion_limits_enabled && (target_speed > max_speed || max_error > max_travel)) {
      throw std::invalid_argument("target_speed <= max_speed and max_error <= max_travel required");
    }
  }
};

inline double desired_position(double leader, double leader_origin, double follower_origin, int direction)
{
  return follower_origin + direction * (leader - leader_origin);
}

inline double limited_target(double ideal, double previous, double dt, const Settings & settings)
{
  if (!std::isfinite(ideal) || !std::isfinite(previous) || !std::isfinite(dt) || dt < 0) {
    throw std::runtime_error("Invalid follow target or time interval");
  }
  if (!settings.motion_limits_enabled) {return ideal;}
  const double step = settings.target_speed * dt;
  return std::clamp(ideal, previous - step, previous + step);
}

inline void check_motion(
  double position, double origin, double velocity, double torque, double temperature,
  const Settings & settings)
{
  if (!std::isfinite(position) || !std::isfinite(origin) ||
    !std::isfinite(velocity) || !std::isfinite(torque) || !std::isfinite(temperature))
  {
    throw std::runtime_error("Non-finite motor feedback");
  }
  if (settings.motion_limits_enabled &&
    (std::abs(position - origin) > settings.max_travel ||
    std::abs(velocity) > settings.max_speed || std::abs(torque) > settings.max_torque ||
    temperature >= settings.max_temperature))
  {
    throw std::runtime_error("Motion safety limit exceeded");
  }
}
}  // namespace dm_motor_lead_follow
