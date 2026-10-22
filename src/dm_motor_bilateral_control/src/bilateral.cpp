#include "dm_motor_bilateral_control/bilateral.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace dm_motor_bilateral_control
{
namespace
{
void finite_sample(const Sample & sample)
{
  if (!std::isfinite(sample.time) || !std::isfinite(sample.position) ||
    !std::isfinite(sample.velocity) || !std::isfinite(sample.torque) || sample.time < 0)
  {
    throw std::invalid_argument("Bilateral feedback must have finite position, velocity, torque and time");
  }
}

dm_motor::Command encode_decode(
  const dm_motor::Command & target, const AxisLimits & axis)
{
  if (!std::isfinite(target.p) || target.p < axis.position_min || target.p > axis.position_max) {
    throw std::invalid_argument("Bilateral target outside joint bounds");
  }
  const dm_motor::Command native{
    axis.direction * (target.p - axis.offset), axis.direction * target.v,
    target.kp, target.kd, axis.direction * target.t};
  dm_motor::validate_command(native, axis.mapping, axis.command);
  return dm_motor::decode_mit(dm_motor::mit_frame(1, native, axis.mapping), axis.mapping);
}
}  // namespace

void Settings::validate() const
{
  if (direction != 1 && direction != -1) {throw std::invalid_argument("direction must be 1 or -1");}
  for (double value : {stiffness, coupling_damping, local_damping, max_torque, max_feedback_torque,
      max_travel, warning_temperature, max_temperature, feedback_age, pair_skew, feedback_stop, recovery_time})
  {
    if (!std::isfinite(value)) {throw std::invalid_argument("Bilateral settings must be finite");}
  }
  if (stiffness < 0 || stiffness > 100 || coupling_damping < 0 || local_damping < 0 ||
    coupling_damping + local_damping > dm_motor::max_kd || max_torque <= 0 || max_feedback_torque <= 0 ||
    max_travel <= 0 || warning_temperature < 0 || warning_temperature >= max_temperature ||
    max_temperature > 120)
  {
    throw std::invalid_argument("Invalid gains or motion limits; Dc+B must respect the driver's Kd limit");
  }
  if (feedback_age < 0.002 || feedback_age > 0.1 || pair_skew <= 0 || pair_skew > feedback_age ||
    feedback_stop <= feedback_age || feedback_stop > 0.5 || recovery_time < 0.01 || recovery_time > 1)
  {
    throw std::invalid_argument("Require 0 < pair_skew <= feedback_age < feedback_stop, and recovery_time in 0.01..1");
  }
}

void AxisLimits::validate() const
{
  dm_motor::resolve_limits(mapping, command);
  if ((direction != 1 && direction != -1) || !std::isfinite(offset) ||
    !std::isfinite(position_min) || !std::isfinite(position_max) || position_min >= position_max)
  {
    throw std::invalid_argument("Invalid motor coordinates or joint bounds");
  }
}

std::array<dm_motor::Command, 2> relative_targets(
  const std::array<Sample, 2> & pair, const std::array<double, 2> & origins, const Settings & settings)
{
  settings.validate();
  const double damping = settings.coupling_damping + settings.local_damping;
  const double ratio = damping == 0 ? 0 : settings.coupling_damping / damping;
  std::array<dm_motor::Command, 2> result;
  for (size_t i = 0; i < 2; ++i) {
    finite_sample(pair[i]);
    if (!std::isfinite(origins[i])) {throw std::invalid_argument("Captured origins must be finite");}
    result[i] = {origins[i] + settings.direction * (pair[1 - i].position - origins[1 - i]),
      settings.direction * ratio * pair[1 - i].velocity, settings.stiffness, damping, 0};
  }
  return result;
}

Output symmetric_targets(
  const std::array<dm_motor::Command, 2> & desired, const std::array<Sample, 2> & latest,
  const std::array<AxisLimits, 2> & limits, const Settings & settings, double recovery_scale)
{
  settings.validate();
  if (!std::isfinite(recovery_scale) || recovery_scale < 0 || recovery_scale > 1) {
    throw std::invalid_argument("Recovery gain scale must be in 0..1");
  }
  const double kp = settings.stiffness * recovery_scale;
  const double kd = (settings.coupling_damping + settings.local_damping) * recovery_scale;
  std::array<double, 2> pe{}, ve{}, bias{};
  double common_scale = 1;
  for (size_t i = 0; i < 2; ++i) {
    limits[i].validate();
    finite_sample(latest[i]);
    const auto bounds = dm_motor::resolve_limits(limits[i].mapping, limits[i].command);
    if (settings.stiffness > bounds.kp || settings.coupling_damping + settings.local_damping > bounds.kd ||
      settings.max_torque > bounds.t ||
      (settings.motion_limits_enabled && settings.max_feedback_torque > limits[i].mapping.t))
    {
      throw std::invalid_argument("Configured bilateral gains or torque budget exceed a motor's capabilities");
    }
    auto target = desired[i];
    target.kp = 0; target.kd = 0; target.t = 0;
    const auto decoded = encode_decode(target, limits[i]);
    pe[i] = decoded.p - limits[i].direction * (latest[i].position - limits[i].offset);
    ve[i] = decoded.v - limits[i].direction * latest[i].velocity;
    bias[i] = decoded.t;
    const double available = settings.max_torque - std::abs(bias[i]);
    if (available < 0) {throw std::invalid_argument("PD budget is below neutral torque quantization bias");}
    const double magnitude = kp * std::abs(pe[i]) + kd * std::abs(ve[i]);
    if (!std::isfinite(magnitude)) {throw std::runtime_error("Bilateral PD estimate overflow");}
    if (magnitude > 0) {common_scale = std::min(common_scale, available / magnitude);}
  }
  Output result;
  result.gain_scale = common_scale * recovery_scale;
  for (size_t i = 0; i < 2; ++i) {
    result.targets[i] = desired[i];
    result.targets[i].kp = kp * common_scale;
    result.targets[i].kd = kd * common_scale;
    result.targets[i].t = 0;
    const auto decoded = encode_decode(result.targets[i], limits[i]);
    result.decoded_kp = decoded.kp; result.decoded_kd = decoded.kd;
    result.estimated_torque[i] = limits[i].direction * (decoded.kp * pe[i] + decoded.kd * ve[i] + bias[i]);
    result.raw_torque[i] = settings.stiffness * (desired[i].p - latest[i].position) +
      (settings.coupling_damping + settings.local_damping) * (desired[i].v - latest[i].velocity);
    if (!std::isfinite(result.estimated_torque[i]) ||
      std::abs(result.estimated_torque[i]) > settings.max_torque + 1e-9)
    {
      throw std::runtime_error("Decoded bilateral PD estimate exceeds the common budget");
    }
  }
  return result;
}

void check_motion(const Sample & sample, double origin, double temperature, const Settings & settings)
{
  finite_sample(sample);
  if (!std::isfinite(origin) || !std::isfinite(temperature)) {
    throw std::runtime_error("Non-finite bilateral feedback");
  }
  if (settings.motion_limits_enabled) {
    if (std::abs(sample.position - origin) > settings.max_travel) {
      throw std::runtime_error("Bilateral travel limit exceeded");
    }
    if (std::abs(sample.torque) > settings.max_feedback_torque) {
      throw std::runtime_error("Bilateral measured torque limit exceeded");
    }
    if (temperature >= settings.max_temperature) {
      throw std::runtime_error("Bilateral temperature limit exceeded");
    }
  }
}

void FeedbackPairs::push(size_t axis, const Sample & sample)
{
  if (axis >= 2) {throw std::invalid_argument("Invalid bilateral axis");}
  finite_sample(sample);
  auto & history = history_[axis];
  if (sample.sequence == 0 || (!history.empty() &&
    (sample.time <= history.back().time || sample.sequence <= history.back().sequence)))
  {
    throw std::runtime_error("Feedback sequence or acquisition time restarted/replayed");
  }
  history.push_back(sample);
  if (history.size() > 256) {history.pop_front();}
}

void FeedbackPairs::reset(double after)
{
  for (auto & history : history_) {history.clear();}
  used_.fill(after);
  pair_.reset();
}

bool FeedbackPairs::select(double now, double maximum_age, double maximum_skew)
{
  if (!std::isfinite(now) || !std::isfinite(maximum_age) || !std::isfinite(maximum_skew) ||
    maximum_age <= 0 || maximum_skew <= 0 || maximum_skew > maximum_age)
  {
    throw std::invalid_argument("Invalid pairing time bounds");
  }
  auto left = history_[0].rbegin(), right = history_[1].rbegin();
  while (left != history_[0].rend() && right != history_[1].rend()) {
    if (left->time <= used_[0] || right->time <= used_[1] ||
      now - left->time > maximum_age || now - right->time > maximum_age) {break;}
    if (left->time > now || right->time > now) {
      throw std::runtime_error("Feedback acquisition timestamps are not in the controller's monotonic clock");
    }
    const double delta = left->time - right->time;
    if (std::abs(delta) <= maximum_skew) {
      pair_ = std::array<Sample, 2>{*left, *right};
      used_ = {left->time, right->time};
      return true;
    }
    if (delta > 0) {++left;} else {++right;}
  }
  return false;
}

double FeedbackPairs::age(double now) const
{
  return pair_ ? now - std::min((*pair_)[0].time, (*pair_)[1].time) : std::numeric_limits<double>::infinity();
}

double FeedbackPairs::skew() const
{
  return pair_ ? std::abs((*pair_)[0].time - (*pair_)[1].time) : 0;
}

void Recovery::reset()
{
  quality = Quality::Normal; scale = 1; fresh_pairs = 0; degradation_count = 0;
}

void Recovery::update(double age, bool updated, double dt, double period, const Settings & settings)
{
  if (!std::isfinite(age) || age < 0 || !std::isfinite(dt) || dt < 0 ||
    !std::isfinite(period) || period <= 0)
  {
    throw std::runtime_error("Invalid paired-feedback or control time");
  }
  if (age > settings.feedback_stop) {throw std::runtime_error("Valid bilateral feedback pair expired");}
  if (age > settings.feedback_age) {
    if (quality != Quality::Degraded) {++degradation_count;}
    quality = Quality::Degraded; scale = 0; fresh_pairs = 0;
  } else if (updated) {
    if (quality == Quality::Degraded) {quality = Quality::Recovering; fresh_pairs = 1;}
    else if (quality == Quality::Recovering) {++fresh_pairs;}
  }
  if (quality == Quality::Recovering && fresh_pairs >= 3) {
    scale = std::min(1.0, scale + std::min(dt, 2 * period) / settings.recovery_time);
    if (scale >= 1) {quality = Quality::Normal;}
  }
}
}  // namespace dm_motor_bilateral_control
