#include "dm_motor_bilateral_control/bilateral.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace dm_motor_bilateral_control
{
namespace
{
NativeGains mapped_gains(const Settings & settings)
{
  switch (settings.control_mode) {
    case ControlMode::Native:
      return {settings.stiffness, settings.coupling_damping + settings.local_damping};
    case ControlMode::SharedReference:
      return {2 * settings.stiffness, 2 * settings.coupling_damping + settings.local_damping};
    case ControlMode::DifferentialTorque:
      return {0, 2 * settings.coupling_damping + settings.local_damping};
  }
  throw std::invalid_argument("Unknown bilateral control mode");
}

void validate_offset(double offset, int direction)
{
  if (!std::isfinite(offset) || std::abs(offset) > std::acos(-1.0) / 6 ||
    (direction != 1 && direction != -1))
  {
    throw std::invalid_argument("Angle offset must be finite and within +/-pi/6, with direction +/-1");
  }
}

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

Output differential_output(
  const std::array<dm_motor::Command, 2> & desired, const std::array<Sample, 2> & latest,
  const std::array<AxisLimits, 2> & limits, const Settings & settings, double recovery_scale,
  const std::array<double, 2> & disturbance,
  const std::optional<CommonTimePositions> & spring_positions)
{
  const auto gains = native_gains(settings);
  if (!std::isfinite(recovery_scale) || recovery_scale < 0 || recovery_scale > 1) {
    throw std::invalid_argument("Recovery gain scale must be in 0..1");
  }
  Output out;
  std::array<double, 2> cap{}, torque_bin{}, feedforward{}, ve{};
  for (size_t i = 0; i < 2; ++i) {
    limits[i].validate(); finite_sample(latest[i]);
    const auto bounds = dm_motor::resolve_limits(limits[i].mapping, limits[i].command);
    if (gains.kd > bounds.kd || (settings.max_torque > 0 && settings.max_torque > bounds.t) ||
      (settings.motion_limits_enabled && settings.max_feedback_torque > limits[i].mapping.t))
    {
      throw std::invalid_argument("Configured differential damping or torque budget exceeds capabilities");
    }
    if (!std::isfinite(desired[i].p) || !std::isfinite(desired[i].v) || !std::isfinite(desired[i].t) ||
      !std::isfinite(disturbance[i]) || std::abs(disturbance[i]) > 0.3)
    {
      throw std::invalid_argument("Differential references/feedforward must be finite; pulse within +/-0.3 Nm");
    }
    cap[i] = bounds.t;
    torque_bin[i] = 2 * limits[i].mapping.t / 4095.;
    out.friction_torque[i] = desired[i].t * recovery_scale;
    out.disturbance_torque[i] = disturbance[i] * recovery_scale;
    feedforward[i] = out.friction_torque[i] + out.disturbance_torque[i];
    // P=0: a large relative separation must saturate the spring, not reject
    // the unused position field. Choose an encodable neutral position.
    const double lower = std::max(limits[i].position_min, limits[i].offset - limits[i].mapping.p);
    const double upper = std::min(limits[i].position_max, limits[i].offset + limits[i].mapping.p);
    if (lower > upper) {throw std::invalid_argument("No encodable neutral position inside joint bounds");}
    out.targets[i] = {std::clamp(limits[i].offset, lower, upper), desired[i].v,
      0, gains.kd * recovery_scale, feedforward[i]};
    const auto decoded = encode_decode(out.targets[i], limits[i]);
    ve[i] = decoded.v - limits[i].direction * latest[i].velocity;
  }
  // The virtual center cancels algebraically. Align only the host spring's
  // positions; local damping estimates and feedforward keep latest velocities.
  std::array<double, 2> spring_position{latest[0].position, latest[1].position};
  if (spring_positions) {
    const auto & aligned = *spring_positions;
    if (!std::isfinite(aligned.time) || aligned.time < 0 ||
      aligned.time > std::min(latest[0].time, latest[1].time))
    {
      throw std::invalid_argument("Common spring time must not exceed either latest feedback time");
    }
    for (size_t i = 0; i < spring_position.size(); ++i) {
      if (!std::isfinite(aligned.position[i]) || !std::isfinite(aligned.bracket_gap[i]) ||
        aligned.bracket_gap[i] < 0 || !std::isfinite(aligned.latest_age[i]) || aligned.latest_age[i] < 0)
      {
        throw std::invalid_argument("Invalid common-time spring position evidence");
      }
    }
    spring_position = aligned.position;
  }
  const double error = (spring_position[0] - desired[0].p) -
    settings.direction * (spring_position[1] - desired[1].p);
  const double spring = -settings.stiffness * error;
  if (!std::isfinite(spring)) {throw std::runtime_error("Differential spring overflow");}
  std::array<double, 2> raw_spring{spring, -settings.direction * spring};
  if (settings.coupling_mode == CouplingMode::ALeadsB) {raw_spring[0] = 0;}
  double budget_scale = 1;
  if (settings.max_torque > 0) {
    for (size_t i = 0; i < 2; ++i) {
      const double available = settings.max_torque - std::abs(feedforward[i]) - torque_bin[i];
      if (available < 0) {throw std::invalid_argument("Feedforward/encoding reserve exceeds torque budget");}
      const double magnitude = recovery_scale * (std::abs(raw_spring[i]) + gains.kd * std::abs(ve[i]));
      if (!std::isfinite(magnitude)) {throw std::runtime_error("Differential demand overflow");}
      if (magnitude > 0) {budget_scale = std::min(budget_scale, available / magnitude);}
    }
  }
  out.gain_scale = recovery_scale * budget_scale;
  double protocol_scale = 1;
  for (size_t i = 0; i < 2; ++i) {
    out.targets[i].kd = gains.kd * out.gain_scale;
    const auto decoded = encode_decode(out.targets[i], limits[i]);
    if (settings.max_torque == 0) {
      const double encoded_velocity = limits[i].direction * decoded.v;
      out.quantization_torque[i] = decoded.kd * (desired[i].v - encoded_velocity);
    }
    const double base = feedforward[i] + out.quantization_torque[i];
    const double available = cap[i] - std::abs(base) - torque_bin[i];
    if (available < 0) {throw std::invalid_argument("Feedforward/encoding reserve exceeds protocol headroom");}
    const double magnitude = std::abs(raw_spring[i] * out.gain_scale);
    if (magnitude > 0) {protocol_scale = std::min(protocol_scale, available / magnitude);}
  }
  out.coupling_scale = out.gain_scale * protocol_scale;
  for (size_t i = 0; i < 2; ++i) {
    out.coupling_torque[i] = raw_spring[i] * out.coupling_scale;
    out.targets[i].t = feedforward[i] + out.quantization_torque[i] + out.coupling_torque[i];
    const auto decoded = encode_decode(out.targets[i], limits[i]);
    out.decoded_kp = decoded.kp; out.decoded_kd = decoded.kd;
    out.estimated_torque[i] = limits[i].direction * (decoded.kd * ve[i] + decoded.t);
    out.raw_torque[i] = raw_spring[i] + gains.kd * (desired[i].v - latest[i].velocity) +
      desired[i].t + disturbance[i];
    if (!std::isfinite(out.estimated_torque[i]) || !std::isfinite(out.raw_torque[i])) {
      throw std::runtime_error("Differential torque estimate overflow");
    }
    out.protocol_saturated[i] = protocol_scale < 1 || std::abs(out.estimated_torque[i]) > limits[i].mapping.t;
    if (settings.coupling_mode == CouplingMode::ALeadsB && i == 0) {
      // Only B has a spring in this mode; its headroom cannot change A's
      // command or make A appear spring-saturated. Native D stays local.
      out.protocol_saturated[i] = std::abs(out.estimated_torque[i]) > limits[i].mapping.t;
    }
    if (settings.max_torque > 0 && std::abs(out.estimated_torque[i]) > settings.max_torque + 1e-9) {
      throw std::runtime_error("Encoded differential demand exceeds common budget");
    }
  }
  return out;
}
}  // namespace

void Settings::validate() const
{
  if (direction != 1 && direction != -1) {throw std::invalid_argument("direction must be 1 or -1");}
  for (double value : {stiffness, coupling_damping, local_damping, max_torque, max_feedback_torque,
      max_travel, warning_temperature, max_temperature, feedback_age, pair_skew, feedback_stop, recovery_time,
      friction_compensation_fraction, friction_torque_limit,
      extra_viscous[0], extra_viscous[1], extra_viscous_velocity_limit})
  {
    if (!std::isfinite(value)) {throw std::invalid_argument("Bilateral settings must be finite");}
  }
  const auto gains = mapped_gains(*this);
  switch (coupling_mode) {
    case CouplingMode::Bilateral:
      break;
    case CouplingMode::ALeadsB:
      if (control_mode != ControlMode::DifferentialTorque || max_torque != 0) {
        throw std::invalid_argument("A-leads-B coupling requires differential torque mode and max_torque=0");
      }
      break;
    default:
      throw std::invalid_argument("Unknown bilateral coupling mode");
  }
  if (stiffness < 0 || gains.kp > 500 ||
    (control_mode == ControlMode::DifferentialTorque && stiffness > 500) ||
    coupling_damping < 0 || local_damping < 0 ||
    gains.kd > dm_motor::max_kd || max_torque < 0 || max_feedback_torque <= 0 ||
    max_travel <= 0 || warning_temperature < 0 || warning_temperature >= max_temperature ||
    max_temperature > 120)
  {
    throw std::invalid_argument("Invalid gains or motion limits; mapped MIT Kp/Kd must respect driver limits");
  }
  if (friction_compensation_fraction < 0 || friction_compensation_fraction > 1 ||
    friction_torque_limit <= 0 || friction_torque_limit > 0.5)
  {
    throw std::invalid_argument("Friction fraction must be in 0..1 and torque limit in (0,0.5] Nm");
  }
  if (extra_viscous[0] < 0 || extra_viscous[0] > 0.05 ||
    extra_viscous[1] < 0 || extra_viscous[1] > 0.05 ||
    extra_viscous_velocity_limit < 0 || extra_viscous_velocity_limit > 50 ||
    ((extra_viscous[0] > 0 || extra_viscous[1] > 0) && extra_viscous_velocity_limit == 0))
  {
    throw std::invalid_argument("Extra viscosity must be in [0,0.05] Nm s/rad with an explicit speed ceiling in (0,50] rad/s");
  }
  if (feedback_age < 0.002 || feedback_age > 0.1 || pair_skew <= 0 || pair_skew > feedback_age ||
    feedback_stop <= feedback_age || feedback_stop > 0.5 || recovery_time < 0.01 || recovery_time > 1)
  {
    throw std::invalid_argument("Require 0 < pair_skew <= feedback_age < feedback_stop, and recovery_time in 0.01..1");
  }
}

NativeGains native_gains(const Settings & settings)
{
  settings.validate();
  return mapped_gains(settings);
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

double PeerVelocityFilter::update(double velocity, double acquisition_time)
{
  if (!std::isfinite(velocity) || !std::isfinite(acquisition_time) || acquisition_time < 0 ||
    (acquisition_time_ && acquisition_time < *acquisition_time_))
  {
    throw std::invalid_argument("Peer velocity filter requires finite, nondecreasing acquisition time and velocity");
  }
  if (!acquisition_time_) {
    acquisition_time_ = acquisition_time;
    return filtered_;
  }
  const double elapsed = acquisition_time - *acquisition_time_;
  if (elapsed == 0) {return filtered_;}
  constexpr double time_constant = 0.008;
  const double alpha = -std::expm1(-elapsed / time_constant);
  const double filtered = (1 - alpha) * filtered_ + alpha * velocity;
  if (!std::isfinite(filtered)) {throw std::runtime_error("Peer velocity filter overflow");}
  acquisition_time_ = acquisition_time;
  filtered_ = filtered;
  return filtered_;
}

void PeerVelocityFilter::reset()
{
  acquisition_time_.reset();
  filtered_ = 0;
}

uint64_t DisturbancePulse::start(uint8_t axis, double torque, double duration, double now)
{
  if (axis >= 2 || !std::isfinite(torque) || torque == 0 || std::abs(torque) > 0.3 ||
    !std::isfinite(duration) || duration <= 0 || duration > 0.2 ||
    !std::isfinite(now) || now < 0 || (last_time_ && now < *last_time_) ||
    !std::isfinite(now + duration) || now + duration <= now)
  {
    throw std::invalid_argument("Disturbance requires axis 0/1, nonzero torque within +/-0.3 Nm, duration in (0,0.2] s and monotonic time");
  }
  if (active_ && now < deadline_) {throw std::runtime_error("A bilateral disturbance is already active; requests are not queued");}
  if (id_ == std::numeric_limits<uint64_t>::max()) {throw std::runtime_error("Disturbance id exhausted");}
  axis_ = axis; torque_ = torque; deadline_ = now + duration; last_time_ = now;
  active_ = true;
  return ++id_;
}

std::array<double, 2> DisturbancePulse::update(double now, bool permitted)
{
  if (!permitted) {cancel();}
  if (!std::isfinite(now) || now < 0 || (last_time_ && now < *last_time_)) {
    cancel();
    throw std::invalid_argument("Disturbance clock must be finite and monotonic");
  }
  last_time_ = now;
  if (now >= deadline_) {cancel();}
  std::array<double, 2> result{};
  if (active_) {result[axis_] = torque_;}
  return result;
}

void DisturbancePulse::cancel()
{
  active_ = false;
}

double DisturbancePulse::remaining(double now) const
{
  if (!std::isfinite(now) || now < 0 || (last_time_ && now < *last_time_)) {
    throw std::invalid_argument("Disturbance clock must be finite and monotonic");
  }
  return active_ ? std::max(0.0, deadline_ - now) : 0;
}

void AngleOffset::check_time(double now) const
{
  if (!std::isfinite(now) || now < 0 || (last_time_ && now < *last_time_)) {
    throw std::invalid_argument("Angle-offset clock must be finite and monotonic; stop before clearing the offset");
  }
}

uint64_t AngleOffset::start(double offset, double transition_time, double hold_duration, double now)
{
  check_time(now);
  if (!std::isfinite(offset) || offset == 0 || std::abs(offset) > std::acos(-1.0) / 6 ||
    !std::isfinite(transition_time) || transition_time < 0 || transition_time > 5 ||
    !std::isfinite(hold_duration) || hold_duration <= 0 || hold_duration > 10 ||
    !std::isfinite(now + transition_time + hold_duration) || now + transition_time + hold_duration <= now)
  {
    throw std::invalid_argument("Angle offset requires nonzero abs(offset)<=pi/6, transition in [0,5] s and hold in (0,10] s");
  }
  if (active_) {throw std::runtime_error("An angle offset is already active; stop it before requesting another");}
  if (id_ == std::numeric_limits<uint64_t>::max()) {throw std::runtime_error("Angle-offset id exhausted");}
  target_ = offset; applied_ = 0; started_ = now; transition_ = transition_time;
  deadline_ = now + transition_time + hold_duration; last_time_ = now;
  active_ = true; ramping_ = transition_time > 0;
  return ++id_;
}

double AngleOffset::update(double now)
{
  check_time(now);
  if (expired(now)) {
    throw std::runtime_error("Angle-offset hold expired; stop both motors before clearing the offset");
  }
  last_time_ = now;
  if (!active_) {return 0;}
  const double u = transition_ == 0 ? 1 : std::clamp((now - started_) / transition_, 0.0, 1.0);
  ramping_ = u < 1;
  applied_ = ramping_ ? target_ * u * u * u * (10 + u * (-15 + 6 * u)) : target_;
  return applied_;
}

bool AngleOffset::expired(double now) const
{
  check_time(now);
  return active_ && now >= deadline_;
}

double AngleOffset::remaining(double now) const
{
  check_time(now);
  return active_ ? std::max(0.0, deadline_ - now) : 0;
}

void AngleOffset::cancel()
{
  active_ = ramping_ = false;
  applied_ = 0;
}

std::array<dm_motor::Command, 2> relative_targets(
  const std::array<Sample, 2> & pair, const std::array<double, 2> & origins, const Settings & settings)
{
  settings.validate();
  if (settings.control_mode != ControlMode::Native) {
    throw std::invalid_argument("Relative peer targets require native control mode");
  }
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

std::array<dm_motor::Command, 2> relative_targets(
  const std::array<Sample, 2> & pair, const std::array<Sample, 2> & latest,
  const std::array<double, 2> & origins, const Settings & settings)
{
  auto result = relative_targets(pair, origins, settings);
  for (size_t i = 0; i < 2; ++i) {
    finite_sample(latest[i]);
    const double elapsed = latest[i].time - pair[1 - i].time;
    if (std::abs(elapsed) > settings.feedback_age) {
      throw std::invalid_argument("Bilateral peer time alignment exceeds the feedback age limit");
    }
    constexpr double maximum_prediction = 0.002;
    const double prediction = std::clamp(elapsed, -maximum_prediction, maximum_prediction);
    result[i].p += settings.direction * pair[1 - i].velocity * prediction;
    if (!std::isfinite(result[i].p)) {
      throw std::invalid_argument("Bilateral peer time alignment overflow");
    }
  }
  return result;
}

std::array<dm_motor::Command, 2> shared_reference_targets(
  const std::array<Sample, 2> & latest_reference,
  const std::array<double, 2> & origins, const Settings & settings)
{
  const auto gains = native_gains(settings);
  if (settings.control_mode != ControlMode::SharedReference &&
    settings.control_mode != ControlMode::DifferentialTorque) {
    throw std::invalid_argument("Shared reference targets require a common-velocity control mode");
  }
  for (size_t i = 0; i < 2; ++i) {
    finite_sample(latest_reference[i]);
    if (!std::isfinite(origins[i])) {throw std::invalid_argument("Captured origins must be finite");}
  }
  const double center = 0.5 * ((latest_reference[0].position - origins[0]) +
    settings.direction * (latest_reference[1].position - origins[1]));
  const double common_velocity = 0.5 * (latest_reference[0].velocity +
    settings.direction * latest_reference[1].velocity);
  const double ratio = gains.kd == 0 ? 0 : 2 * settings.coupling_damping / gains.kd;
  std::array<dm_motor::Command, 2> result{{
      {origins[0] + center, ratio * common_velocity, gains.kp, gains.kd, 0},
      {origins[1] + settings.direction * center,
        settings.direction * ratio * common_velocity, gains.kp, gains.kd, 0}}};
  if (settings.coupling_mode == CouplingMode::ALeadsB) {
    result[0].v = ratio * latest_reference[0].velocity;
  }
  for (const auto & target : result) {
    if (!std::isfinite(target.p) || !std::isfinite(target.v)) {
      throw std::invalid_argument("Bilateral shared reference overflow");
    }
  }
  return result;
}

std::array<dm_motor::Command, 2> apply_angle_offset(
  const std::array<dm_motor::Command, 2> & targets, double offset, int direction)
{
  validate_offset(offset, direction);
  auto result = targets;
  result[0].p -= offset;
  result[1].p += direction * offset;
  for (const auto & target : result) {
    if (!std::isfinite(target.p)) {throw std::invalid_argument("Angle-offset target position overflow");}
  }
  return result;
}

std::array<dm_motor::Command, 2> apply_angle_offset(
  const std::array<dm_motor::Command, 2> & targets, double offset, const Settings & settings)
{
  settings.validate();
  // Validate the full virtual offset before halving the per-axis reference shift.
  validate_offset(offset, settings.direction);
  return apply_angle_offset(targets,
    settings.control_mode == ControlMode::Native ? offset : 0.5 * offset, settings.direction);
}

void validate_angle_offset_path(
  const std::array<dm_motor::Command, 2> & targets, const std::array<Sample, 2> & latest,
  const std::array<AxisLimits, 2> & limits, const Settings & settings, double offset)
{
  symmetric_targets(targets, latest, limits, settings, 1);
  symmetric_targets(apply_angle_offset(targets, offset, settings), latest, limits, settings, 1);
}

Output symmetric_targets(
  const std::array<dm_motor::Command, 2> & desired, const std::array<Sample, 2> & latest,
  const std::array<AxisLimits, 2> & limits, const Settings & settings, double recovery_scale,
  const std::array<double, 2> & disturbance,
  const std::optional<CommonTimePositions> & spring_positions)
{
  if (settings.control_mode == ControlMode::DifferentialTorque) {
    return differential_output(desired, latest, limits, settings, recovery_scale, disturbance, spring_positions);
  }
  const auto gains = native_gains(settings);
  if (!std::isfinite(recovery_scale) || recovery_scale < 0 || recovery_scale > 1) {
    throw std::invalid_argument("Recovery gain scale must be in 0..1");
  }
  const double kp = gains.kp * recovery_scale;
  const double kd = gains.kd * recovery_scale;
  const bool budget_enabled = settings.max_torque > 0;
  std::array<double, 2> pe{}, ve{};
  double common_scale = 1;
  for (size_t i = 0; i < 2; ++i) {
    if (!std::isfinite(disturbance[i]) || std::abs(disturbance[i]) > 0.3) {
      throw std::invalid_argument("Disturbance feedforward must be finite and within +/-0.3 Nm");
    }
    limits[i].validate();
    finite_sample(latest[i]);
    const auto bounds = dm_motor::resolve_limits(limits[i].mapping, limits[i].command);
    if (gains.kp > bounds.kp || gains.kd > bounds.kd ||
      (budget_enabled && settings.max_torque > bounds.t) ||
      (settings.motion_limits_enabled && settings.max_feedback_torque > limits[i].mapping.t))
    {
      throw std::invalid_argument("Configured bilateral gains or torque budget exceed a motor's capabilities");
    }
    auto target = desired[i];
    target.kp = 0; target.kd = 0;
    target.t = target.t * recovery_scale + disturbance[i] * recovery_scale;
    const auto decoded = encode_decode(target, limits[i]);
    pe[i] = decoded.p - limits[i].direction * (latest[i].position - limits[i].offset);
    ve[i] = decoded.v - limits[i].direction * latest[i].velocity;
    const double magnitude = kp * std::abs(pe[i]) + kd * std::abs(ve[i]);
    if (!std::isfinite(magnitude)) {throw std::runtime_error("Bilateral PD estimate overflow");}
    if (budget_enabled) {
      // Reserve the actual encoded feedforward, including its nonzero neutral bias.
      const double available = settings.max_torque - std::abs(decoded.t);
      if (available < 0) {throw std::invalid_argument("Quantized feedforward exceeds the bilateral torque budget");}
      if (magnitude > 0) {common_scale = std::min(common_scale, available / magnitude);}
    }
  }
  Output result;
  result.gain_scale = common_scale * recovery_scale;
  for (size_t i = 0; i < 2; ++i) {
    result.targets[i] = desired[i];
    result.targets[i].kp = kp * common_scale;
    result.targets[i].kd = kd * common_scale;
    result.friction_torque[i] = desired[i].t * recovery_scale;
    result.disturbance_torque[i] = disturbance[i] * recovery_scale;
    const double feedforward = result.friction_torque[i] + result.disturbance_torque[i];
    result.targets[i].t = feedforward;
    auto decoded = encode_decode(result.targets[i], limits[i]);
    if (!budget_enabled) {
      const double joint_position = limits[i].direction * decoded.p + limits[i].offset;
      const double joint_velocity = limits[i].direction * decoded.v;
      const double correction = decoded.kp * (result.targets[i].p - joint_position) +
        decoded.kd * (result.targets[i].v - joint_velocity);
      if (!std::isfinite(correction)) {throw std::runtime_error("Bilateral PD quantization correction overflow");}
      const double cap = dm_motor::resolve_limits(limits[i].mapping, limits[i].command).t;
      // Preserve friction and the explicit pulse first; the correction only uses remaining
      // feedforward-field headroom. It does not impose a total PD torque budget.
      result.targets[i].t = std::clamp(feedforward + correction, -cap, cap);
      result.quantization_torque[i] = result.targets[i].t - feedforward;
      decoded = encode_decode(result.targets[i], limits[i]);
    }
    result.decoded_kp = decoded.kp; result.decoded_kd = decoded.kd;
    result.estimated_torque[i] = limits[i].direction * (decoded.kp * pe[i] + decoded.kd * ve[i] + decoded.t);
    result.raw_torque[i] = gains.kp * (desired[i].p - latest[i].position) +
      gains.kd * (desired[i].v - latest[i].velocity) +
      desired[i].t + disturbance[i];
    if (!std::isfinite(result.estimated_torque[i]) || !std::isfinite(result.raw_torque[i])) {
      throw std::runtime_error("Bilateral PD and feedforward estimate overflow");
    }
    result.protocol_saturated[i] = std::abs(result.estimated_torque[i]) > limits[i].mapping.t;
    if (budget_enabled && std::abs(result.estimated_torque[i]) > settings.max_torque + 1e-9)
    {
      throw std::runtime_error("Decoded bilateral PD and feedforward estimate exceeds the common budget");
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
    (sample.time < history.back().time || sample.sequence <= history.back().sequence)))
  {
    throw std::runtime_error("Feedback sequence or acquisition time restarted/replayed");
  }
  if (!history.empty() && sample.time == history.back().time) {
    // Batched feedback can share an acquisition time. Keep its newest value without
    // advancing the consumed time watermark or rewriting an already selected pair.
    history.back() = sample;
    return;
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

std::optional<CommonTimePositions> FeedbackPairs::common_time_positions(
  double now, double maximum_age, double maximum_bracket_gap, std::string * rejection_reason) const
{
  if (rejection_reason) {rejection_reason->clear();}
  const auto reject = [&](const char * reason, size_t axis, const char * quantity,
      double value, double limit) -> std::optional<CommonTimePositions> {
      if (rejection_reason) {
        std::ostringstream text;
        text.precision(17);
        text << reason;
        if (axis < history_.size()) {text << " axis=" << axis + 1;}
        text << ' ' << quantity << '=' << value << " limit=" << limit;
        *rejection_reason = text.str();
      }
      return std::nullopt;
    };
  if (!std::isfinite(now) || !std::isfinite(maximum_age) || maximum_age <= 0 ||
    !std::isfinite(maximum_bracket_gap) || maximum_bracket_gap <= 0 || maximum_bracket_gap > maximum_age)
  {
    throw std::invalid_argument("Invalid common-time position bounds");
  }
  for (size_t i = 0; i < history_.size(); ++i) {
    if (history_[i].empty()) {return reject("missing_history", i, "samples", 0, 1);}
  }
  CommonTimePositions result;
  result.time = std::min(history_[0].back().time, history_[1].back().time);
  for (size_t i = 0; i < history_.size(); ++i) {
    const auto & history = history_[i];
    if (history.back().time > now) {
      throw std::runtime_error("Feedback acquisition timestamps are not in the controller's monotonic clock");
    }
    result.latest_age[i] = now - history.back().time;
  }
  if (now - result.time > maximum_age) {
    return reject("stale_common", history_.size(), "age_s", now - result.time, maximum_age);
  }
  const auto at_time = [&](double time, bool explain) -> std::optional<CommonTimePositions> {
      auto aligned = result;
      aligned.time = time;
      const auto failed = [&](const char * reason, size_t axis, const char * quantity,
          double value, double limit) -> std::optional<CommonTimePositions> {
          return explain ? reject(reason, axis, quantity, value, limit) : std::nullopt;
        };
      for (size_t i = 0; i < history_.size(); ++i) {
        const auto & history = history_[i];
        const auto upper = std::lower_bound(history.begin(), history.end(), aligned.time,
          [](const Sample & sample, double stamp) {return sample.time < stamp;});
        if (upper == history.end()) {
          return failed("missing_bracket", i, "common_time", aligned.time, history.back().time);
        }
        if (upper->time == aligned.time) {
          aligned.position[i] = upper->position;
          continue;
        }
        if (upper == history.begin()) {
          return failed("missing_bracket", i, "common_time", aligned.time, history.front().time);
        }
        const auto lower = std::prev(upper);
        const double gap = upper->time - lower->time;
        if (gap <= 0 || gap > maximum_bracket_gap) {
          return failed("bracket_gap", i, "gap_s", gap, maximum_bracket_gap);
        }
        if (now - lower->time > maximum_age) {
          return failed("stale_bracket", i, "age_s", now - lower->time, maximum_age);
        }
        const double fraction = (aligned.time - lower->time) / gap;
        aligned.position[i] = lower->position + fraction * (upper->position - lower->position);
        if (!std::isfinite(aligned.position[i])) {throw std::runtime_error("Common-time position interpolation overflow");}
        aligned.bracket_gap[i] = gap;
      }
      return aligned;
    };
  // Preserve the usual min(latest) path, including its interpolation arithmetic.
  if (const auto preferred = at_time(result.time, true)) {return preferred;}

  // Each axis' usable times are fresh exact samples plus closed intervals whose
  // actual endpoints meet both bounds. The latest point in their intersection
  // must be the preferred time or one of these received endpoints. Merge the
  // endpoint sets in descending order without allocation (<=512 stored samples).
  // Latest timestamps were validated above; older evidence cannot mask a clock
  // failure. Neither pairing consumption nor a new synthetic sample is involved.
  auto left = history_[0].rbegin(), right = history_[1].rbegin();
  while (left != history_[0].rend() || right != history_[1].rend()) {
    const double candidate = right == history_[1].rend() ? left->time :
      left == history_[0].rend() ? right->time : std::max(left->time, right->time);
    if (left != history_[0].rend() && left->time == candidate) {++left;}
    if (right != history_[1].rend() && right->time == candidate) {++right;}
    if (candidate >= result.time) {continue;}
    if (now - candidate > maximum_age) {break;}
    if (const auto earlier = at_time(candidate, false)) {
      if (rejection_reason) {rejection_reason->clear();}
      return earlier;
    }
  }
  // Retain the preferred time's first rejection, rather than a less useful
  // failure from the oldest candidate searched.
  return std::nullopt;
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
