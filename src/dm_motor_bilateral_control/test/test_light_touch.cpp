#include "dm_motor_bilateral_control/bilateral.hpp"
#include "dm_motor_control/simulation/device.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using dm_motor_bilateral_control::AxisLimits;
using dm_motor_bilateral_control::PeerVelocityFilter;
using dm_motor_bilateral_control::Sample;
using dm_motor_bilateral_control::Settings;
using dm_motor::simulation::DeviceParameters;
using dm_motor::simulation::VirtualMotor;

DeviceParameters parameters(size_t axis)
{
  // Frozen from config/system_parameters.yaml: qualified hardware fits
  // 20261030T055853-8c3a2b81 (/motor_1), 20261030T055934-dfaba92f (/motor_2).
  constexpr std::array<double, 2> inertia{0.0038742723463633427, 0.0014637501871318296};
  constexpr std::array<double, 2> coulomb{0.1416881637628295, 0.10164556437387359};
  DeviceParameters value;
  value.motor.id = static_cast<uint8_t>(axis + 1);
  value.motor.master = 0;
  value.motor.mode = dm_motor::Mode::Mit;
  value.motor.mapping = {12.5, 50, 10};
  value.physics.inertia = inertia[axis];
  value.physics.viscous_friction = 0;
  value.physics.coulomb_friction = coulomb[axis];
  value.physics.static_friction = 1.2 * coulomb[axis];
  // These two lags and static friction are robustness assumptions, not identified hardware values.
  value.physics.torque_time_constant = 0.002;
  value.velocity_time_constant = 0.002;
  value.integration_step = 0.0005;
  return value;
}

struct Metrics
{
  double rest_peak_velocity{0}, peak_velocity{0}, maximum_position{0};
  std::array<double, 2> tail_square{}, tail_rms{}, tail_min{}, tail_max{};
  std::array<unsigned, 2> tail_crossings{};
  double tail_cross_product{0}, tail_correlation{0};
  double maximum_sample_skew{0}, tail_opposed_fraction{0};
  unsigned tail_samples{0}, tail_moving_pairs{0}, tail_opposed_pairs{0};
};

Metrics light_touch(bool filter_peer_velocity, bool stagger_feedback)
{
  std::array<VirtualMotor, 2> motors{VirtualMotor(parameters(0)), VirtualMotor(parameters(1))};
  Settings settings;
  // Isolate the delayed bilateral loop: the same physical friction remains in both trials.
  settings.friction_compensation_enabled = false;
  std::array<AxisLimits, 2> limits;
  for (size_t i = 0; i < 2; ++i) {
    limits[i].mapping = motors[i].parameters().motor.mapping;
    motors[i].receive(dm_motor::special_frame(static_cast<uint16_t>(i + 1), 0xfc), 0);
  }
  std::array<PeerVelocityFilter, 2> filters;
  std::array<double, 2> origins{}, previous_tail_velocity{};
  std::array<Sample, 2> latest;
  struct Pending {unsigned delivery_step; std::array<dm_motor::Frame, 2> frames;};
  std::deque<Pending> commands;
  Metrics result;
  result.tail_min.fill(std::numeric_limits<double>::infinity());
  result.tail_max.fill(-std::numeric_limits<double>::infinity());
  constexpr double physical_step = 0.0005;
  constexpr unsigned feedback_steps = 4;  // 500 Hz controller and feedback.
  constexpr unsigned command_delay_steps = 8;  // 4 ms deterministic transport delay.
  constexpr unsigned push_step = 1000;  // Initially at rest for 0.5 s.
  constexpr unsigned tail_step = 8000, final_step = 10000;  // Tail is 4..5 s.
  for (unsigned step = 0; step <= final_step; ++step) {
    const double time = step * physical_step;
    for (auto & motor : motors) {motor.advance(time);}
    if (step == push_step) {motors[0].external_torque(0.3, 0.25, time);}
    for (size_t i = 0; i < 2; ++i) {
      const unsigned acquisition_phase = stagger_feedback && i == 1 ? feedback_steps / 2 : 0;
      if (step == 0 || step % feedback_steps == acquisition_phase) {
        // Query at the actual physical instant; never relabel newer data as an older sample.
        const auto replies = motors[i].receive(dm_motor::query_frame(static_cast<uint8_t>(i + 1)), time);
        if (replies.size() != 1) {throw std::runtime_error("Expected one real encoded device feedback packet");}
        const auto feedback = dm_motor::decode_feedback(replies.front(), static_cast<uint8_t>(i + 1), 0, limits[i].mapping);
        if (!feedback || feedback->status != 1) {throw std::runtime_error("Virtual device lost enable confirmation");}
        latest[i] = {time, feedback->p, feedback->v, feedback->t, step + 1};
        if (step == 0) {origins[i] = latest[i].position;}
      }
    }
    if (step % feedback_steps == 0) {
      result.maximum_sample_skew = std::max(result.maximum_sample_skew, std::abs(latest[0].time - latest[1].time));
      auto pair = latest;
      for (size_t i = 0; i < 2; ++i) {
        // Match the node: midpoint correction is only in the peer channel;
        // latest retains raw feedback, and VirtualMotor owns its internal Kd feedback.
        const double midpoint = std::clamp(pair[i].velocity + limits[i].mapping.v / 4095,
          -limits[i].mapping.v, limits[i].mapping.v);
        pair[i].velocity = filter_peer_velocity ? filters[i].update(midpoint, pair[i].time) : midpoint;
      }
      const auto desired = dm_motor_bilateral_control::relative_targets(pair, latest, origins, settings);
      const auto output = dm_motor_bilateral_control::symmetric_targets(desired, latest, limits, settings, 1);
      Pending pending{step + command_delay_steps, {}};
      for (size_t i = 0; i < 2; ++i) {
        pending.frames[i] = dm_motor::mit_frame(static_cast<uint8_t>(i + 1), output.targets[i], limits[i].mapping);
      }
      commands.push_back(pending);
    }
    while (!commands.empty() && commands.front().delivery_step <= step) {
      for (size_t i = 0; i < 2; ++i) {motors[i].receive(commands.front().frames[i], time);}
      commands.pop_front();
    }
    for (size_t i = 0; i < 2; ++i) {
      const auto & state = motors[i].state();
      result.peak_velocity = std::max(result.peak_velocity, std::abs(state.velocity));
      result.maximum_position = std::max(result.maximum_position, std::abs(state.position));
      if (step < push_step) {result.rest_peak_velocity = std::max(result.rest_peak_velocity, std::abs(state.velocity));}
      if (step >= tail_step) {
        result.tail_square[i] += state.velocity * state.velocity;
        result.tail_min[i] = std::min(result.tail_min[i], state.position);
        result.tail_max[i] = std::max(result.tail_max[i], state.position);
        if (std::abs(state.velocity) > 0.05) {
          if (state.velocity * previous_tail_velocity[i] < 0) {++result.tail_crossings[i];}
          previous_tail_velocity[i] = state.velocity;
        }
      }
    }
    if (step >= tail_step) {
      ++result.tail_samples;
      const double a = motors[0].state().velocity, b = motors[1].state().velocity;
      result.tail_cross_product += a * b;
      if (std::abs(a) > 0.05 && std::abs(b) > 0.05) {
        ++result.tail_moving_pairs;
        if (a * b < 0) {++result.tail_opposed_pairs;}
      }
    }
  }
  for (size_t i = 0; i < 2; ++i) {result.tail_rms[i] = std::sqrt(result.tail_square[i] / result.tail_samples);}
  const double norm = std::sqrt(result.tail_square[0] * result.tail_square[1]);
  result.tail_correlation = norm > 0 ? result.tail_cross_product / norm : 0;
  result.tail_opposed_fraction = result.tail_moving_pairs > 0 ?
    static_cast<double>(result.tail_opposed_pairs) / result.tail_moving_pairs : 0;
  return result;
}

void report(const char * name, const Metrics & result)
{
  std::cout << name << ": rest_peak_v=" << result.rest_peak_velocity
            << ", peak_v=" << result.peak_velocity << ", max_abs_q=" << result.maximum_position
            << ", tail_rms=" << result.tail_rms[0] << ',' << result.tail_rms[1]
            << ", tail_crossings=" << result.tail_crossings[0] << ',' << result.tail_crossings[1]
            << ", tail_correlation=" << result.tail_correlation
            << ", tail_opposed_fraction=" << result.tail_opposed_fraction
            << ", maximum_sample_skew=" << result.maximum_sample_skew
            << ", tail_q_span=" << result.tail_max[0] - result.tail_min[0]
            << ',' << result.tail_max[1] - result.tail_min[1] << '\n';
}

void expect_released_touch_to_settle(bool stagger_feedback)
{
  const auto baseline = light_touch(false, stagger_feedback);
  const auto candidate = light_touch(true, stagger_feedback);
  report(stagger_feedback ? "unfiltered_staggered" : "unfiltered_synchronous", baseline);
  report(stagger_feedback ? "filtered_staggered" : "filtered_synchronous", candidate);
  EXPECT_DOUBLE_EQ(baseline.rest_peak_velocity, 0);
  EXPECT_DOUBLE_EQ(candidate.rest_peak_velocity, 0);
  EXPECT_NEAR(baseline.maximum_sample_skew, stagger_feedback ? 0.001 : 0, 1e-12);
  EXPECT_NEAR(candidate.maximum_sample_skew, stagger_feedback ? 0.001 : 0, 1e-12);
  // A finite 0.3 Nm / 250 ms touch triggers the old loop from actual rest;
  // it keeps oscillating in opposition over three seconds after the force expires.
  // Unequal inertias and estimator lags shift the two oscillation phases;
  // require clearly opposing motion rather than perfect 180-degree opposition.
  EXPECT_LT(baseline.tail_correlation, -0.25);
  EXPECT_GT(baseline.tail_opposed_fraction, 0.6);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_GT(baseline.tail_rms[i], 0.5);
    EXPECT_GT(baseline.tail_crossings[i], 20u);
    EXPECT_GT(baseline.tail_max[i] - baseline.tail_min[i], 0.01);
    EXPECT_LT(candidate.tail_rms[i], 0.01);
    EXPECT_EQ(candidate.tail_crossings[i], 0u);
    EXPECT_LT(candidate.tail_max[i] - candidate.tail_min[i], 0.001);
  }
  EXPECT_GT(candidate.maximum_position, 0.001);  // The weak push actually moved the system.
  EXPECT_LT(candidate.maximum_position, 0.15);
  EXPECT_LT(candidate.peak_velocity, 2);
}

TEST(BilateralLightTouch, FiltersPeerVelocitySoAReleasedSmallPushCannotSustainTheBaselineOscillation)
{
  expect_released_touch_to_settle(false);
}

TEST(BilateralLightTouch, FilteredPredictionAlsoSettlesWithActualOneMillisecondAcquisitionSkew)
{
  expect_released_touch_to_settle(true);
}
}  // namespace
