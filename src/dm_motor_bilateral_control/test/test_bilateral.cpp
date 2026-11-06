#include "dm_motor_bilateral_control/bilateral.hpp"
#include "dm_motor_bilateral_control/neutral_target.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace dm_motor_bilateral_control;

TEST(BilateralNeutral, MultiTurnObservationsNeverEnterTheNeutralMitPositionField)
{
  for (const double direction : {-1., 1.}) {
    for (const double offset : {-25., 0., 25.}) {
      AxisLimits axis;
      axis.direction = direction; axis.offset = offset;
      axis.position_min = -350; axis.position_max = 350;
      const auto target = neutral_mit_target(axis);
      EXPECT_DOUBLE_EQ(target.p, offset);
      const dm_motor::Command native{direction * (target.p - offset),
        direction * target.v, target.kp, target.kd, direction * target.t};
      ASSERT_NO_THROW(dm_motor::validate_command(native, axis.mapping, axis.command));
      const auto decoded = dm_motor::decode_mit(dm_motor::mit_frame(1, native, axis.mapping), axis.mapping);
      EXPECT_DOUBLE_EQ(decoded.kp, 0); EXPECT_DOUBLE_EQ(decoded.kd, 0);
      EXPECT_NEAR(decoded.t, 0, axis.mapping.t / 4095. + 1e-12);
      for (const double position : {-80., -15.9959, 15.9959, 80.}) {
        // Even after wire quantization, neither multi-turn position nor
        // motion can generate PD effort from a neutral command.
        const double torque = decoded.kp * (decoded.p - direction * (position - offset)) +
          decoded.kd * (decoded.v - direction * 13.2) + decoded.t;
        EXPECT_DOUBLE_EQ(torque, decoded.t);
      }
    }
  }
}

TEST(BilateralNeutral, IntersectsJointBoundsWithTheResolvedNativePositionLimit)
{
  for (const double direction : {-1., 1.}) {
    for (const double offset : {-20., 20.}) {
      for (const double side : {-1., 1.}) {
        AxisLimits axis;
        axis.direction = direction; axis.offset = offset; axis.command.p = 2.;
        axis.position_min = offset + (side < 0 ? -3. : 1.);
        axis.position_max = offset + (side < 0 ? -1. : 3.);
        const auto target = neutral_mit_target(axis);
        EXPECT_DOUBLE_EQ(target.p, offset + side);
        EXPECT_GE(target.p, axis.position_min); EXPECT_LE(target.p, axis.position_max);
        EXPECT_NO_THROW(dm_motor::validate_command(
          {direction * (target.p - offset), target.v, target.kp, target.kd, target.t},
          axis.mapping, axis.command));
        axis.position_min = offset + 3.; axis.position_max = offset + 4.;
        EXPECT_THROW(neutral_mit_target(axis), std::invalid_argument);
      }
    }
  }
}

TEST(BilateralNeutral, DegradedRecoveryIsEncodableAtMultiTurnPositionsInEveryControlMode)
{
  for (const auto mode : {ControlMode::Native, ControlMode::SharedReference, ControlMode::DifferentialTorque}) {
    Settings settings; settings.control_mode = mode; settings.coupling_damping = .4975;
    std::array<AxisLimits, 2> limits;
    limits[0].direction = -1; limits[0].offset = -20;
    limits[1].direction = 1; limits[1].offset = 20;
    for (auto & axis : limits) {axis.position_min = -350; axis.position_max = 350;}
    const std::array<Sample, 2> latest{{{1, -80, 13.2, 0, 1}, {1, 80, -13.2, 0, 1}}};
    const std::array<dm_motor::Command, 2> neutral{
      neutral_mit_target(limits[0]), neutral_mit_target(limits[1])};
    const auto output = symmetric_targets(neutral, latest, limits, settings, 0);
    for (size_t i = 0; i < 2; ++i) {
      const auto & target = output.targets[i]; const auto & axis = limits[i];
      EXPECT_DOUBLE_EQ(target.kp, 0); EXPECT_DOUBLE_EQ(target.kd, 0); EXPECT_DOUBLE_EQ(target.t, 0);
      EXPECT_NO_THROW(dm_motor::validate_command(
        {axis.direction * (target.p - axis.offset), axis.direction * target.v,
          target.kp, target.kd, axis.direction * target.t}, axis.mapping, axis.command));
    }
    if (mode != ControlMode::DifferentialTorque) {
      auto active = neutral; active[0].p = latest[0].position;
      EXPECT_THROW(symmetric_targets(active, latest, limits, settings, 1), std::invalid_argument);
    }
  }
}

TEST(Bilateral, StandardRelativeTargetsPreserveOriginsAndSplitDamping)
{
  Settings s;
  s.coupling_damping = 0.5;
  s.local_damping = 0.25;
  const std::array<double, 2> origins{1.2, -0.7};
  std::array<Sample, 2> pair{{{1, 1.5, 2, 0, 1}, {1, -0.9, -1, 0, 1}}};
  auto targets = relative_targets(pair, origins, s);
  EXPECT_NEAR(targets[0].p, 1.0, 1e-12);
  EXPECT_NEAR(targets[1].p, -0.4, 1e-12);
  EXPECT_NEAR(targets[0].v, -2.0 / 3, 1e-12);
  EXPECT_NEAR(targets[1].v, 4.0 / 3, 1e-12);
  EXPECT_DOUBLE_EQ(targets[0].kd, 0.75);
  s.direction = -1;
  targets = relative_targets(pair, origins, s);
  EXPECT_NEAR(targets[0].p, 1.4, 1e-12);
  EXPECT_NEAR(targets[1].p, -1, 1e-12);
  EXPECT_NEAR(targets[0].v, 2.0 / 3, 1e-12);
  s.coupling_damping = s.local_damping = 0;
  targets = relative_targets(pair, origins, s);
  EXPECT_DOUBLE_EQ(targets[0].v, 0);
  EXPECT_DOUBLE_EQ(targets[1].kd, 0);
}

TEST(Bilateral, DefaultDampingDissipatesCommonMotionWithoutHoldingItsPosition)
{
  for (const int direction : {1, -1}) {
    Settings settings;
    settings.direction = direction;
    ASSERT_GT(settings.local_damping, 0);
    const std::array<double, 2> origins{0.2, -0.1};
    const std::array<Sample, 2> samples{{
        {1, origins[0] + 1.0, 0.4, 0, 1},
        {1, origins[1] + direction, direction * 0.4, 0, 1}}};
    const auto targets = relative_targets(samples, origins, settings);
    double power = 0;
    for (size_t i = 0; i < 2; ++i) {
      EXPECT_NEAR(targets[i].p, samples[i].position, 1e-12);
      const double torque = targets[i].kp * (targets[i].p - samples[i].position) +
        targets[i].kd * (targets[i].v - samples[i].velocity);
      EXPECT_NEAR(torque, -settings.local_damping * samples[i].velocity, 1e-12);
      power += torque * samples[i].velocity;
    }
    EXPECT_LT(power, 0);
    settings.local_damping = 0;
    const auto undamped = relative_targets(samples, origins, settings);
    for (size_t i = 0; i < 2; ++i) {
      EXPECT_NEAR(undamped[i].v, samples[i].velocity, 1e-12);
    }
  }
}

TEST(Bilateral, AlignedTargetsRemoveConstantCommonMotionDelayWithoutRemovingLocalDamping)
{
  for (const int direction : {1, -1}) {
    Settings settings;
    settings.direction = direction; settings.stiffness = 500;
    settings.coupling_damping = 0.995; settings.local_damping = 0.005;
    const std::array<double, 2> origins{0.2, -0.4};
    const auto sample = [&](size_t axis, double time) {
        const double velocity = (axis == 0 ? 1 : direction) * 0.4;
        return Sample{time, origins[axis] + velocity * (time - 1), velocity, 0, 1};
      };
    const std::array<Sample, 2> pair{{sample(0, 1), sample(1, 1.001)}};
    const std::array<Sample, 2> latest{{sample(0, 1.002), sample(1, 1.002)}};
    const auto previous = relative_targets(pair, origins, settings);
    const auto aligned = relative_targets(pair, latest, origins, settings);
    for (size_t i = 0; i < 2; ++i) {
      EXPECT_NE(previous[i].p, latest[i].position);
      EXPECT_NEAR(aligned[i].p, latest[i].position, 1e-12);
      EXPECT_DOUBLE_EQ(aligned[i].v, previous[i].v);
      EXPECT_DOUBLE_EQ(aligned[i].kp, 500);
      EXPECT_DOUBLE_EQ(aligned[i].kd, 1);
      const double torque = aligned[i].kp * (aligned[i].p - latest[i].position) +
        aligned[i].kd * (aligned[i].v - latest[i].velocity);
      EXPECT_NEAR(torque, -settings.local_damping * latest[i].velocity, 1e-12);
    }
  }
}

TEST(Bilateral, AlignedTargetsKeepDifferentialRestoringSignsAndLatestLocalVelocityFeedback)
{
  for (const int direction : {1, -1}) {
    Settings settings; settings.direction = direction;
    const std::array<double, 2> origins{1.2, -0.7};
    const auto position = [&](size_t axis, double time) {
        return origins[axis] + (axis == 0 ? 0.2 + 0.3 * (time - 1) :
               direction * (-0.1 - 0.2 * (time - 1)));
      };
    const std::array<Sample, 2> pair{{
        {1, position(0, 1), 0.3, 0, 1}, {1.001, position(1, 1.001), -direction * 0.2, 0, 1}}};
    const std::array<Sample, 2> latest{{
        {1.0015, position(0, 1.0015), 0.35, 0, 2},
        {1.002, position(1, 1.002), -direction * 0.23, 0, 2}}};
    const auto aligned = relative_targets(pair, latest, origins, settings);
    std::array<double, 2> torque;
    for (size_t i = 0; i < 2; ++i) {
      const double peer = position(1 - i, latest[i].time);
      const double error = origins[i] + direction * (peer - origins[1 - i]) - latest[i].position;
      const double expected = settings.stiffness * error + settings.coupling_damping *
        (direction * pair[1 - i].velocity - latest[i].velocity) -
        settings.local_damping * latest[i].velocity;
      torque[i] = aligned[i].kp * (aligned[i].p - latest[i].position) +
        aligned[i].kd * (aligned[i].v - latest[i].velocity);
      EXPECT_NEAR(torque[i], expected, 1e-12);
    }
    EXPECT_LT(torque[0], 0);
    EXPECT_GT(direction * torque[1], 0);
  }
}

TEST(Bilateral, PeerTimeAlignmentAllowsBoundedForwardAndBackwardProjection)
{
  Settings settings;
  const std::array<Sample, 2> pair{{{1, 0.1, 0.3, 0, 1}, {1.01, 0.2, -0.4, 0, 1}}};
  std::array<Sample, 2> latest{{{1.005, 0.1, 0.3, 0, 2}, {1.012, 0.2, -0.4, 0, 2}}};
  const auto aligned = relative_targets(pair, latest, {0, 0}, settings);
  EXPECT_NEAR(aligned[0].p, 0.2008, 1e-12);
  EXPECT_NEAR(aligned[1].p, 0.1006, 1e-12);
  EXPECT_DOUBLE_EQ(pair[0].time, 1);
  EXPECT_DOUBLE_EQ(pair[1].time, 1.01);
  EXPECT_DOUBLE_EQ(latest[0].time, 1.005);
  EXPECT_DOUBLE_EQ(latest[1].time, 1.012);
  latest[0].time = 1.031;
  EXPECT_THROW(relative_targets(pair, latest, {0, 0}, settings), std::invalid_argument);
  latest[0].time = 0.989;
  EXPECT_THROW(relative_targets(pair, latest, {0, 0}, settings), std::invalid_argument);
  latest[0].time = NAN;
  EXPECT_THROW(relative_targets(pair, latest, {0, 0}, settings), std::invalid_argument);
  latest[0].time = 1.005; latest[0].velocity = NAN;
  EXPECT_THROW(relative_targets(pair, latest, {0, 0}, settings), std::invalid_argument);

  settings.feedback_age = 0.015625;
  const std::array<Sample, 2> boundary_pair{{{1, 0, 0.3, 0, 1}, {1.015625, 0, -0.4, 0, 1}}};
  const std::array<Sample, 2> boundary_latest{{{1, 0, 0.3, 0, 1}, {1.015625, 0, -0.4, 0, 1}}};
  EXPECT_NO_THROW(relative_targets(boundary_pair, boundary_latest, {0, 0}, settings));
}

TEST(Bilateral, PeerPredictionStopsGrowingAfterTwoMillisecondsWithoutRefreshingSampleAge)
{
  Settings settings;
  for (const int direction : {-1, 1}) {
    settings.direction = direction;
    const std::array<Sample, 2> pair{{{1, 0.1, 0.3, 0, 1}, {1, 0.2, -0.4, 0, 1}}};
    auto latest = pair;
    for (auto & sample : latest) {sample.time = 1.002;}
    const auto at_cap = relative_targets(pair, latest, {0, 0}, settings);
    for (auto & sample : latest) {sample.time = 1.019;}
    const auto older = relative_targets(pair, latest, {0, 0}, settings);
    for (size_t i = 0; i < 2; ++i) {
      EXPECT_DOUBLE_EQ(older[i].p, at_cap[i].p);
      EXPECT_DOUBLE_EQ(older[i].v, at_cap[i].v);
      EXPECT_DOUBLE_EQ(pair[i].time, 1);
    }
    latest[0].time = 1.021;
    EXPECT_THROW(relative_targets(pair, latest, {0, 0}, settings), std::invalid_argument);
  }
}

TEST(BilateralPeerVelocity, UpdatesOnlyByAcquisitionTimeIncludingCoalescedFrames)
{
  PeerVelocityFilter filter;
  EXPECT_DOUBLE_EQ(filter.update(1, 10), 0);
  const double first = filter.update(1, 10.002);
  EXPECT_NEAR(first, 1 - std::exp(-0.002 / 0.008), 1e-12);
  for (unsigned tick = 0; tick < 100; ++tick) {
    EXPECT_DOUBLE_EQ(filter.update(-1, 10.002), first);
  }
  const double coalesced = filter.update(1, 10.00200005);
  EXPECT_GT(coalesced, first);
  EXPECT_LT(coalesced - first, 1e-5);
  EXPECT_NEAR(filter.update(1, 10.008), 1 - std::exp(-1), 1e-12);
}

TEST(BilateralPeerVelocity, InvalidSamplesLeaveHistoryIntactAndResetStartsAnew)
{
  PeerVelocityFilter filter;
  EXPECT_THROW(filter.update(0, -1), std::invalid_argument);
  EXPECT_DOUBLE_EQ(filter.update(0.5, 10), 0);
  const double previous = filter.update(0.5, 10.008);
  EXPECT_THROW(filter.update(NAN, 10.010), std::invalid_argument);
  EXPECT_THROW(filter.update(0, NAN), std::invalid_argument);
  EXPECT_THROW(filter.update(0, std::numeric_limits<double>::infinity()), std::invalid_argument);
  EXPECT_THROW(filter.update(std::numeric_limits<double>::infinity(), 10.010), std::invalid_argument);
  EXPECT_THROW(filter.update(0, 10.007), std::invalid_argument);
  EXPECT_DOUBLE_EQ(filter.update(-0.5, 10.008), previous);
  filter.reset();
  EXPECT_DOUBLE_EQ(filter.update(-0.5, 1), 0);
  EXPECT_NEAR(filter.update(-0.5, 1.008), -0.5 * (1 - std::exp(-1)), 1e-12);
  const double settled = filter.update(-0.5, 2);
  EXPECT_GE(settled, -0.5);
  EXPECT_LE(settled, 0);
}

TEST(BilateralPeerVelocity, FilteredPeerFeedforwardRetainsLocalMitGainsAndRawVelocityFeedback)
{
  Settings settings;
  std::array<PeerVelocityFilter, 2> filters;
  std::array<Sample, 2> pair{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  for (size_t i = 0; i < 2; ++i) {EXPECT_DOUBLE_EQ(filters[i].update(0, pair[i].time), 0);}
  const std::array<Sample, 2> latest{{{1.002, 0.001, 0.2, 0, 2}, {1.002, -0.001, -0.2, 0, 2}}};
  pair = latest;
  for (size_t i = 0; i < 2; ++i) {pair[i].velocity = filters[i].update(pair[i].velocity, pair[i].time);}
  const auto desired = relative_targets(pair, latest, {0, 0}, settings);
  const auto output = symmetric_targets(desired, latest, {}, settings, 1);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(output.targets[i].kp, 50);
    EXPECT_DOUBLE_EQ(output.targets[i].kd, 1);
    EXPECT_NEAR(output.targets[i].v, 0.995 * pair[1 - i].velocity, 1e-12);
    EXPECT_LT(std::abs(output.targets[i].v), 0.995 * std::abs(latest[1 - i].velocity));
    const double ideal = 50 * (desired[i].p - latest[i].position) + desired[i].v - latest[i].velocity;
    EXPECT_NEAR(output.raw_torque[i], ideal, 1e-12);
  }
}

TEST(Bilateral, InitialOffsetDoesNotBecomeAnAbsolutePositionSwap)
{
  Settings s;
  const std::array<double, 2> origins{0.8, -0.5};
  std::array<Sample, 2> pair{{{1, 0.8, 0, 0, 1}, {1, -0.5, 0, 0, 1}}};
  const auto targets = relative_targets(pair, origins, s);
  EXPECT_DOUBLE_EQ(targets[0].p, 0.8);
  EXPECT_DOUBLE_EQ(targets[1].p, -0.5);
  EXPECT_DOUBLE_EQ(targets[0].t, 0);
  EXPECT_DOUBLE_EQ(targets[1].t, 0);
}

TEST(Bilateral, IdealCouplingHasCorrectForceSignsAndDissipation)
{
  for (const int direction : {1, -1}) {
    Settings s; s.direction = direction; s.stiffness = 16;
    s.max_torque = 8; s.coupling_damping = 0.35; s.local_damping = 0.2;
    std::array<Sample, 2> pair{{{1, 0.2, 0.3, 0, 1}, {1, -0.1, -0.2, 0, 1}}};
    const auto targets = relative_targets(pair, {0, 0}, s);
    const double e = pair[0].position - direction * pair[1].position;
    const double v = pair[0].velocity - direction * pair[1].velocity;
    const double a = targets[0].kp * (targets[0].p - pair[0].position) +
      targets[0].kd * (targets[0].v - pair[0].velocity);
    const double b = targets[1].kp * (targets[1].p - pair[1].position) +
      targets[1].kd * (targets[1].v - pair[1].velocity);
    EXPECT_NEAR(a, -s.stiffness * e - s.coupling_damping * v - s.local_damping * pair[0].velocity, 1e-12);
    EXPECT_NEAR(b, direction * (s.stiffness * e + s.coupling_damping * v) - s.local_damping * pair[1].velocity, 1e-12);
    const double power = a * pair[0].velocity + b * pair[1].velocity;
    EXPECT_NEAR(power + s.stiffness * e * v,
      -s.coupling_damping * v * v - s.local_damping *
      (pair[0].velocity * pair[0].velocity + pair[1].velocity * pair[1].velocity), 1e-12);
  }
}

TEST(Bilateral, CommonBudgetUsesProductionQuantizationAndMotorCoordinates)
{
  Settings s; s.stiffness = 100; s.max_torque = 0.3;
  s.coupling_damping = s.local_damping = 0.35;
  std::array<AxisLimits, 2> limits;
  limits[0].mapping = {10, 20, 5};
  limits[1].mapping = {12.5, 50, 10};
  limits[1].direction = -1; limits[1].offset = 0.4;
  std::array<Sample, 2> samples{{{1, -2, -3, 0, 1}, {1, 2.4, 4, 0, 1}}};
  const auto desired = relative_targets(samples, {0, 0.4}, s);
  const auto output = symmetric_targets(desired, samples, limits, s, 1);
  EXPECT_LT(output.gain_scale, 1);
  EXPECT_EQ(output.targets[0].kp, output.targets[1].kp);
  EXPECT_EQ(output.targets[0].kd, output.targets[1].kd);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(output.targets[i].t, 0);
    const auto & t = output.targets[i];
    const dm_motor::Command native{limits[i].direction * (t.p - limits[i].offset),
      limits[i].direction * t.v, t.kp, t.kd, 0};
    const auto decoded = dm_motor::decode_mit(dm_motor::mit_frame(1, native, limits[i].mapping), limits[i].mapping);
    const double estimate = limits[i].direction * (decoded.kp *
      (decoded.p - limits[i].direction * (samples[i].position - limits[i].offset)) +
      decoded.kd * (decoded.v - limits[i].direction * samples[i].velocity) + decoded.t);
    EXPECT_NEAR(estimate, output.estimated_torque[i], 1e-12);
    EXPECT_LE(std::abs(estimate), s.max_torque + 1e-12);
  }
}

TEST(Bilateral, ZeroRecoveryHasZeroGainsButReportsNeutralBias)
{
  Settings s;
  std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  const auto desired = relative_targets(samples, {0, 0}, s);
  const auto output = symmetric_targets(desired, samples, {}, s, 0);
  EXPECT_DOUBLE_EQ(output.targets[0].kp, 0);
  EXPECT_DOUBLE_EQ(output.targets[1].kd, 0);
  EXPECT_NEAR(output.estimated_torque[0], -10.0 / 4095, 1e-12);
  EXPECT_DOUBLE_EQ(output.gain_scale, 0);
}

TEST(Bilateral, UnlimitedBudgetPreservesGainsAndReportsUnclippedProtocolDemand)
{
  Settings settings;
  settings.max_torque = 0; settings.stiffness = 500;
  settings.coupling_damping = 0.995; settings.local_damping = 0.005;
  std::array<AxisLimits, 2> limits;
  limits[0].mapping = {10, 20, 5};
  limits[1].mapping = {12.5, 50, 10};
  limits[1].direction = -1; limits[1].offset = 0.4;
  for (const int direction : {1, -1}) {
    settings.direction = direction;
    const std::array<Sample, 2> samples{{
        {1, -3, -0.3, 0, 1}, {1, 0.4 + direction * 3, direction * 0.4, 0, 1}}};
    auto desired = relative_targets(samples, {0, 0.4}, settings);
    desired[0].t = 0.12; desired[1].t = -0.08;
    for (const double recovery : {0.0, 0.25, 1.0}) {
      const auto output = symmetric_targets(desired, samples, limits, settings, recovery);
      EXPECT_DOUBLE_EQ(output.gain_scale, recovery);
      for (size_t i = 0; i < 2; ++i) {
        const auto & target = output.targets[i];
        EXPECT_DOUBLE_EQ(target.kp, settings.stiffness * recovery);
        EXPECT_DOUBLE_EQ(target.kd, recovery);
        EXPECT_DOUBLE_EQ(output.friction_torque[i], desired[i].t * recovery);
        EXPECT_DOUBLE_EQ(target.t, output.friction_torque[i] + output.quantization_torque[i]);
        const dm_motor::Command native{
          limits[i].direction * (target.p - limits[i].offset), limits[i].direction * target.v,
          target.kp, target.kd, limits[i].direction * target.t};
        const auto decoded = dm_motor::decode_mit(dm_motor::mit_frame(1, native, limits[i].mapping), limits[i].mapping);
        const double expected = limits[i].direction * (decoded.kp *
          (decoded.p - limits[i].direction * (samples[i].position - limits[i].offset)) +
          decoded.kd * (decoded.v - limits[i].direction * samples[i].velocity) + decoded.t);
        EXPECT_NEAR(output.estimated_torque[i], expected, 1e-12);
        EXPECT_EQ(output.protocol_saturated[i], recovery > 0);
        if (recovery > 0) {
          EXPECT_GT(std::abs(output.estimated_torque[i]), limits[i].mapping.t);
          EXPECT_GT(decoded.kp, 0);
          EXPECT_GT(decoded.kd, 0);
        } else {
          EXPECT_DOUBLE_EQ(output.quantization_torque[i], 0);
          EXPECT_NEAR(output.estimated_torque[i], -limits[i].direction * limits[i].mapping.t / 4095, 1e-12);
        }
      }
      if (recovery > 0) {
        EXPECT_GT(output.estimated_torque[0], 0);
        EXPECT_LT(direction * output.estimated_torque[1], 0);
      }
    }
  }
}

TEST(Bilateral, PositiveBudgetStillScalesGainsAndCapsDecodedDemand)
{
  Settings settings; settings.stiffness = 100; settings.max_torque = 0.5;
  settings.coupling_damping = 0.995; settings.local_damping = 0.005;
  const std::array<Sample, 2> samples{{{1, -3, 0, 0, 1}, {1, 3, 0, 0, 1}}};
  const auto desired = relative_targets(samples, {0, 0}, settings);
  const auto limited = symmetric_targets(desired, samples, {}, settings, 1);
  EXPECT_GT(limited.gain_scale, 0);
  EXPECT_LT(limited.gain_scale, 1);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_LE(std::abs(limited.estimated_torque[i]), settings.max_torque);
    EXPECT_FALSE(limited.protocol_saturated[i]);
    EXPECT_DOUBLE_EQ(limited.quantization_torque[i], 0);
  }
  settings.max_torque = 0;
  const auto unlimited = symmetric_targets(desired, samples, {}, settings, 1);
  EXPECT_DOUBLE_EQ(unlimited.gain_scale, 1);
  EXPECT_DOUBLE_EQ(unlimited.targets[0].kp, 100);
  EXPECT_DOUBLE_EQ(unlimited.targets[0].kd, 1);
  EXPECT_TRUE(unlimited.protocol_saturated[0]);
  EXPECT_TRUE(unlimited.protocol_saturated[1]);
}

TEST(Bilateral, UnlimitedBudgetCorrectsPdRoundingAcrossCoordinatesVelocitiesAndRecovery)
{
  Settings settings; settings.max_torque = 0; settings.stiffness = 50;
  settings.coupling_damping = 0.995; settings.local_damping = 0.005;
  std::array<AxisLimits, 2> limits;
  limits[0].mapping = {10, 20, 5};
  limits[1].mapping = {12.5, 50, 10}; limits[1].direction = -1; limits[1].offset = 0.4;
  for (const int direction : {1, -1}) {
    settings.direction = direction;
    for (const double velocity : {-0.4, 0.0, 0.4}) {
      std::array<Sample, 2> samples;
      std::array<double, 2> origins;
      for (size_t i = 0; i < 2; ++i) {
        const double joint_velocity = (i == 0 ? 1 : direction) * velocity;
        const dm_motor::Command native{0.123, limits[i].direction * joint_velocity, 0, 0, 0};
        const auto feedback = dm_motor::decode_mit(dm_motor::mit_frame(1, native, limits[i].mapping), limits[i].mapping);
        origins[i] = limits[i].direction * feedback.p + limits[i].offset;
        samples[i] = {1, origins[i], limits[i].direction * feedback.v, 0, 1};
      }
      auto desired = relative_targets(samples, origins, settings);
      const double friction_sign = velocity < 0 ? -1 : 1;
      desired[0].t = friction_sign * 0.08; desired[1].t = -friction_sign * 0.05;
      for (const double recovery : {0.0, 0.25, 1.0}) {
        const auto output = symmetric_targets(desired, samples, limits, settings, recovery);
        for (size_t i = 0; i < 2; ++i) {
          const auto & target = output.targets[i];
          const dm_motor::Command native{
            limits[i].direction * (target.p - limits[i].offset), limits[i].direction * target.v,
            target.kp, target.kd, limits[i].direction * target.t};
          const auto decoded = dm_motor::decode_mit(dm_motor::mit_frame(1, native, limits[i].mapping), limits[i].mapping);
          const double correction = decoded.kp * (target.p - (limits[i].direction * decoded.p + limits[i].offset)) +
            decoded.kd * (target.v - limits[i].direction * decoded.v);
          EXPECT_NEAR(output.quantization_torque[i], correction, 1e-12);
          EXPECT_DOUBLE_EQ(output.friction_torque[i], desired[i].t * recovery);
          EXPECT_DOUBLE_EQ(target.t, output.friction_torque[i] + output.quantization_torque[i]);
          const double ideal = decoded.kp * (desired[i].p - samples[i].position) +
            decoded.kd * (desired[i].v - samples[i].velocity) + desired[i].t * recovery;
          const double encoded = limits[i].direction * (decoded.kp *
            (decoded.p - limits[i].direction * (samples[i].position - limits[i].offset)) +
            decoded.kd * (decoded.v - limits[i].direction * samples[i].velocity) + decoded.t);
          EXPECT_NEAR(output.estimated_torque[i], encoded, 1e-12);
          // Position/velocity rounding is compensated; the torque field still has its own LSB.
          EXPECT_LE(std::abs(output.estimated_torque[i] - ideal), 2 * limits[i].mapping.t / 4095 + 1e-12);
          const double raw = settings.stiffness * (desired[i].p - samples[i].position) +
            (settings.coupling_damping + settings.local_damping) * (desired[i].v - samples[i].velocity) + desired[i].t;
          EXPECT_NEAR(output.raw_torque[i], raw, 1e-12);
          if (recovery == 0) {
            EXPECT_DOUBLE_EQ(output.friction_torque[i], 0);
            EXPECT_DOUBLE_EQ(output.quantization_torque[i], 0);
            EXPECT_DOUBLE_EQ(target.t, 0);
          }
        }
      }
    }
  }
}

TEST(Bilateral, QuantizationCorrectionRemovesTheExtraVelocityCodeOfCommonMotionDrag)
{
  Settings settings; settings.max_torque = 0; settings.stiffness = 50;
  settings.coupling_damping = 0.995; settings.local_damping = 0.005;
  std::array<AxisLimits, 2> limits;
  for (auto & axis : limits) {axis.mapping = {12.5, 50, 10};}
  for (const double direction : {-1.0, 1.0}) {
    for (auto & axis : limits) {axis.direction = direction; axis.offset = 0.4;}
    for (const double velocity : {-0.04, 0.04}) {
      const auto native = dm_motor::decode_mit(
        dm_motor::mit_frame(1, {0, direction * velocity, 0, 0, 0}, limits[0].mapping), limits[0].mapping);
      const double position = direction * native.p + 0.4;
      const double measured_velocity = direction * native.v;
      const std::array<Sample, 2> samples{{
          {1, position, measured_velocity, 0, 1}, {1, position, measured_velocity, 0, 1}}};
      const auto desired = relative_targets(samples, {position, position}, settings);
      const auto output = symmetric_targets(desired, samples, limits, settings, 1);
      for (size_t i = 0; i < 2; ++i) {
        const double intended_drag = -settings.local_damping * measured_velocity;
        EXPECT_LE(std::abs(output.estimated_torque[i] - intended_drag), 20.0 / 4095 + 1e-12);
        EXPECT_DOUBLE_EQ(output.friction_torque[i], 0);
        EXPECT_NEAR(output.raw_torque[i], intended_drag, 1e-12);
      }
      if (direction * measured_velocity > 0) {
        // A slightly lower positive velocity target falls down a complete 12-bit code.
        EXPECT_GT(direction * output.quantization_torque[0], 0.02);
      }
    }
  }
}

TEST(Bilateral, QuantizationCorrectionRemovesPdBiasAtRestAndClearsAtZeroRecovery)
{
  Settings settings;
  std::array<AxisLimits, 2> limits;
  for (auto & axis : limits) {axis.mapping = {12.5, 50, 10};}
  limits[1].direction = -1; limits[1].offset = 0.4;
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0.4, 0, 0, 1}}};
  const auto desired = relative_targets(samples, {0, 0.4}, settings);
  const auto output = symmetric_targets(desired, samples, limits, settings, 1);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(output.friction_torque[i], 0);
    EXPECT_DOUBLE_EQ(output.raw_torque[i], 0);
    EXPECT_GT(limits[i].direction * output.quantization_torque[i], 0.02);
    EXPECT_LT(std::abs(output.estimated_torque[i]), 20.0 / 4095);
    EXPECT_FALSE(output.protocol_saturated[i]);
  }
  const auto neutral = symmetric_targets(desired, samples, limits, settings, 0);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(neutral.targets[i].t, 0);
    EXPECT_DOUBLE_EQ(neutral.friction_torque[i], 0);
    EXPECT_DOUBLE_EQ(neutral.quantization_torque[i], 0);
    EXPECT_NEAR(neutral.estimated_torque[i], -limits[i].direction * 10 / 4095, 1e-12);
  }
}

TEST(Bilateral, QuantizationCorrectionOnlyUsesFeedforwardHeadroomAndNeverReplacesFriction)
{
  Settings settings; settings.max_torque = 0; settings.stiffness = 500;
  settings.coupling_damping = 0.995; settings.local_damping = 0.005;
  std::array<AxisLimits, 2> limits;
  for (auto & axis : limits) {axis.command.t = 0.05; axis.mapping.v = 50;}
  limits[1].direction = -1; limits[1].offset = 0.4;
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0.4, 0, 0, 1}}};
  std::array<dm_motor::Command, 2> desired{{
      {0.0003, 0, 500, 1, 0.049}, {0.4003, 0, 500, 1, -0.049}}};
  const auto output = symmetric_targets(desired, samples, limits, settings, 1);
  EXPECT_DOUBLE_EQ(output.friction_torque[0], 0.049);
  EXPECT_DOUBLE_EQ(output.friction_torque[1], -0.049);
  EXPECT_DOUBLE_EQ(output.targets[0].t, 0.05);
  EXPECT_DOUBLE_EQ(output.targets[1].t, -0.05);
  EXPECT_NEAR(output.quantization_torque[0], 0.001, 1e-12);
  EXPECT_NEAR(output.quantization_torque[1], -0.001, 1e-12);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_LE(std::abs(output.targets[i].t), limits[i].command.t);
    const auto & target = output.targets[i];
    const dm_motor::Command native{
      limits[i].direction * (target.p - limits[i].offset), limits[i].direction * target.v,
      target.kp, target.kd, limits[i].direction * target.t};
    const auto decoded = dm_motor::decode_mit(dm_motor::mit_frame(1, native, limits[i].mapping), limits[i].mapping);
    const double actual = limits[i].direction * (decoded.kp *
      (decoded.p - limits[i].direction * (samples[i].position - limits[i].offset)) +
      decoded.kd * (decoded.v - limits[i].direction * samples[i].velocity) + decoded.t);
    EXPECT_NEAR(output.estimated_torque[i], actual, 1e-12);
    EXPECT_EQ(output.protocol_saturated[i], std::abs(actual) > limits[i].mapping.t);
  }
  desired[0].t = 0.05; desired[1].t = -0.05;
  const auto no_headroom = symmetric_targets(desired, samples, limits, settings, 1);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(no_headroom.targets[i].t, desired[i].t);
    EXPECT_DOUBLE_EQ(no_headroom.quantization_torque[i], 0);
  }
}

TEST(Bilateral, UnlimitedBudgetStillEnforcesProtocolAndConfiguredCommandBounds)
{
  Settings settings; settings.max_torque = 0; settings.stiffness = 500;
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  const auto desired = relative_targets(samples, {0, 0}, settings);
  std::array<AxisLimits, 2> limits;
  EXPECT_NO_THROW(symmetric_targets(desired, samples, limits, settings, 1));
  limits[0].command.kp = 250;
  EXPECT_THROW(symmetric_targets(desired, samples, limits, settings, 1), std::invalid_argument);
  limits = {}; limits[1].command.kd = 0.5;
  EXPECT_THROW(symmetric_targets(desired, samples, limits, settings, 1), std::invalid_argument);
  limits = {}; limits[0].command.t = 0.1;
  auto invalid = desired; invalid[0].t = 0.11;
  EXPECT_THROW(symmetric_targets(invalid, samples, limits, settings, 1), std::invalid_argument);
  limits = {}; invalid = desired; invalid[0].p = 12.6;
  EXPECT_THROW(symmetric_targets(invalid, samples, limits, settings, 1), std::invalid_argument);
  invalid = desired; invalid[0].v = 31;
  EXPECT_THROW(symmetric_targets(invalid, samples, limits, settings, 1), std::invalid_argument);
  limits[0].position_min = -0.2; limits[0].position_max = 0.2;
  invalid = desired; invalid[0].p = 0.3;
  EXPECT_THROW(symmetric_targets(invalid, samples, limits, settings, 1), std::invalid_argument);
  invalid = desired; invalid[0].t = NAN;
  EXPECT_THROW(symmetric_targets(invalid, samples, {}, settings, 1), std::invalid_argument);
}

TEST(Bilateral, TimeAlignmentDoesNotWrapOrClampAnUnencodablePosition)
{
  Settings settings; settings.max_torque = 0;
  const std::array<Sample, 2> pair{{{1, 0, 0, 0, 1}, {1, 12.499, 5, 0, 1}}};
  const std::array<Sample, 2> latest{{{1.01, 0, 0, 0, 2}, {1.01, 12.499, 5, 0, 2}}};
  const auto aligned = relative_targets(pair, latest, {0, 0}, settings);
  EXPECT_GT(aligned[0].p, 12.5);
  EXPECT_THROW(symmetric_targets(aligned, latest, {}, settings, 1), std::invalid_argument);
}

TEST(Bilateral, ZeroBudgetAndProtocolMaximumStiffnessAreExplicitValidSettings)
{
  Settings settings;
  EXPECT_DOUBLE_EQ(settings.stiffness, 50);
  EXPECT_DOUBLE_EQ(settings.coupling_damping, 0.995);
  EXPECT_DOUBLE_EQ(settings.local_damping, 0.005);
  EXPECT_DOUBLE_EQ(settings.max_torque, 0);
  EXPECT_DOUBLE_EQ(settings.friction_compensation_fraction, 0.6);
  EXPECT_DOUBLE_EQ(settings.friction_torque_limit, 0.2);
  settings.stiffness = 500;
  EXPECT_NO_THROW(settings.validate());
  settings.stiffness = 500.001;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings.stiffness = 500;
  for (const double budget : {-0.001, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    settings.max_torque = budget;
    EXPECT_THROW(settings.validate(), std::invalid_argument);
  }
}

TEST(Bilateral, FrictionAndPdShareTheQuantizedBudgetAcrossCoordinatesAndRecovery)
{
  Settings settings;
  settings.friction_compensation_enabled = true;
  settings.max_torque = 0.3;
  settings.stiffness = 16;
  settings.coupling_damping = settings.local_damping = 0.35;
  std::array<AxisLimits, 2> limits;
  limits[0].mapping = {10, 20, 5};
  limits[1].mapping = {12.5, 50, 10};
  limits[1].direction = -1; limits[1].offset = 0.4;
  const std::array<Sample, 2> samples{{{1, -0.2, -0.3, 0, 1}, {1, 0.5, 0.4, 0, 1}}};
  for (const int direction : {1, -1}) {
    settings.direction = direction;
    const auto unassisted = relative_targets(samples, {0, 0.4}, settings);
    const auto baseline = symmetric_targets(unassisted, samples, limits, settings, 1);
    for (const double sign : {-1.0, 1.0}) {
      auto desired = unassisted;
      desired[0].t = sign * 0.12; desired[1].t = -sign * 0.08;
      for (const double recovery : {0.0, 0.25, 1.0}) {
        const auto output = symmetric_targets(desired, samples, limits, settings, recovery);
        EXPECT_DOUBLE_EQ(output.targets[0].kp, output.targets[1].kp);
        EXPECT_DOUBLE_EQ(output.targets[0].kd, output.targets[1].kd);
        if (recovery == 1) {EXPECT_LT(output.gain_scale, baseline.gain_scale);}
        for (size_t i = 0; i < 2; ++i) {
          const auto & target = output.targets[i];
          EXPECT_DOUBLE_EQ(target.t, desired[i].t * recovery);
          EXPECT_DOUBLE_EQ(output.friction_torque[i], desired[i].t * recovery);
          EXPECT_DOUBLE_EQ(output.quantization_torque[i], 0);
          const dm_motor::Command native{
            limits[i].direction * (target.p - limits[i].offset), limits[i].direction * target.v,
            target.kp, target.kd, limits[i].direction * target.t};
          const auto decoded = dm_motor::decode_mit(dm_motor::mit_frame(1, native, limits[i].mapping), limits[i].mapping);
          const double actual = limits[i].direction * (decoded.kp *
            (decoded.p - limits[i].direction * (samples[i].position - limits[i].offset)) +
            decoded.kd * (decoded.v - limits[i].direction * samples[i].velocity) + decoded.t);
          EXPECT_NEAR(output.estimated_torque[i], actual, 1e-12);
          EXPECT_LE(std::abs(actual), settings.max_torque + 1e-12);
          EXPECT_NEAR(output.raw_torque[i], baseline.raw_torque[i] + desired[i].t, 1e-12);
          if (recovery == 0) {
            EXPECT_DOUBLE_EQ(target.kp, 0);
            EXPECT_DOUBLE_EQ(target.kd, 0);
            EXPECT_DOUBLE_EQ(target.t, 0);
            EXPECT_NEAR(actual, -limits[i].direction * limits[i].mapping.t / 4095, 1e-12);
          }
        }
      }
    }
  }
}

TEST(Bilateral, FeedforwardBudgetUsesDecodedTorqueRatherThanTheRequest)
{
  Settings settings;
  settings.stiffness = settings.coupling_damping = settings.local_damping = 0;
  settings.max_torque = 0.1;
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  auto desired = relative_targets(samples, {0, 0}, settings);
  desired[0].t = 0.1;
  const auto positive = symmetric_targets(desired, samples, {}, settings, 1);
  EXPECT_LE(std::abs(positive.estimated_torque[0]), settings.max_torque);
  desired[0].t = -0.1;
  // Floor encoding makes this negative request exceed 0.1 Nm in magnitude.
  EXPECT_THROW(symmetric_targets(desired, samples, {}, settings, 1), std::invalid_argument);
  desired[0].t = NAN;
  EXPECT_THROW(symmetric_targets(desired, samples, {}, settings, 1), std::invalid_argument);
  EXPECT_THROW(symmetric_targets(desired, samples, {}, settings, 0), std::invalid_argument);
  desired[0].t = std::numeric_limits<double>::infinity();
  EXPECT_THROW(symmetric_targets(desired, samples, {}, settings, 1), std::invalid_argument);
}

TEST(Bilateral, FrictionSettingsValidateWithoutRequiringTheCapBelowThePdBudget)
{
  Settings settings;
  EXPECT_FALSE(settings.friction_compensation_enabled);
  EXPECT_DOUBLE_EQ(settings.friction_compensation_fraction, 0.6);
  EXPECT_DOUBLE_EQ(settings.friction_torque_limit, 0.2);
  settings.max_torque = 0.05;
  EXPECT_NO_THROW(settings.validate());
  for (const double fraction : {0.0, 1.0}) {
    settings.friction_compensation_fraction = fraction;
    EXPECT_NO_THROW(settings.validate());
  }
  for (const double fraction : {-0.01, 1.01, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    settings.friction_compensation_fraction = fraction;
    EXPECT_THROW(settings.validate(), std::invalid_argument);
  }
  settings.friction_compensation_fraction = 0.8;
  settings.friction_torque_limit = 0.5;
  EXPECT_NO_THROW(settings.validate());
  for (const double cap : {0.0, -0.1, 0.50001, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    settings.friction_torque_limit = cap;
    EXPECT_THROW(settings.validate(), std::invalid_argument);
  }
}

TEST(BilateralPairs, EightMsBracketPolicyCoversObservedBatchingButStillRejectsWiderHoles)
{
  for (const double gap : {.006652, .0081}) {
    FeedbackPairs pairs;
    // Both positions describe exactly the same common motion. The later A
    // sample brackets B's time; compact velocity is irrelevant to interpolation.
    pairs.push(0, {1., 30., 0., 0., 1});
    pairs.push(0, {1. + gap, 30. + 13.2 * gap, 0., 0., 2});
    pairs.push(1, {1.003, -20. + 13.2 * .003, 0., 0., 1});
    std::string reason;
    const auto aligned = pairs.common_time_positions(1. + gap, .02, .008, &reason);
    if (gap < .008) {
      ASSERT_TRUE(aligned);
      EXPECT_TRUE(reason.empty());
      EXPECT_NEAR(aligned->position[0] - aligned->position[1], 50., 1e-12);
    } else {
      EXPECT_FALSE(aligned);
      EXPECT_NE(reason.find("bracket_gap"), std::string::npos);
    }
  }
}

TEST(Bilateral, ExplicitViscousAssistanceRequiresFiniteCoefficientsAndSpeedCeiling)
{
  Settings settings;
  EXPECT_EQ(settings.extra_viscous, (std::array<double, 2>{0, 0}));
  EXPECT_DOUBLE_EQ(settings.extra_viscous_velocity_limit, 0);
  settings.extra_viscous[1] = .02067790017359263;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings.extra_viscous_velocity_limit = 13.2;
  EXPECT_NO_THROW(settings.validate());
  for (const double value : {-0.001, 0.050001, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    auto invalid = settings; invalid.extra_viscous[0] = value;
    EXPECT_THROW(invalid.validate(), std::invalid_argument);
  }
  for (const double value : {-0.001, 50.0001, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    auto invalid = settings; invalid.extra_viscous_velocity_limit = value;
    EXPECT_THROW(invalid.validate(), std::invalid_argument);
  }
}

TEST(Bilateral, InvalidInputsAndUnencodableTargetsAreNotSilentlyClamped)
{
  Settings s;
  std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  auto desired = relative_targets(samples, {0, 0}, s);
  s.max_torque = 0.0001;
  EXPECT_THROW(symmetric_targets(desired, samples, {}, s, 1), std::invalid_argument);
  s.max_torque = 0.5;
  desired[0].p = 100;
  EXPECT_THROW(symmetric_targets(desired, samples, {}, s, 1), std::invalid_argument);
  s.local_damping = 1;
  EXPECT_THROW(s.validate(), std::invalid_argument);
  s.local_damping = 0;
  s.feedback_stop = s.feedback_age;
  EXPECT_THROW(s.validate(), std::invalid_argument);
  s.feedback_stop = 0.08;
  samples[0].position = NAN;
  EXPECT_THROW(relative_targets(samples, {0, 0}, s), std::invalid_argument);
}

TEST(Bilateral, OptionalMotionLimitsDoNotCreateAnAngleDifferenceTrip)
{
  Settings s;
  EXPECT_FALSE(s.motion_limits_enabled);
  EXPECT_NO_THROW(check_motion({1, 10, 50, 10, 1}, 0, 90, s));
  EXPECT_THROW(check_motion({1, NAN, 0, 0, 1}, 0, 25, s), std::invalid_argument);
  s.motion_limits_enabled = true;
  EXPECT_THROW(check_motion({1, 1, 0, 0, 1}, 0, 25, s), std::runtime_error);
  EXPECT_THROW(check_motion({1, 0, 0, 2, 1}, 0, 25, s), std::runtime_error);
  EXPECT_THROW(check_motion({1, 0, 0, 0, 1}, 0, 60, s), std::runtime_error);
  EXPECT_NO_THROW(check_motion({1, 0, 100, 0, 1}, 0, 25, s));
}

TEST(BilateralPairs, UsesNewestEligibleHistoricalPairWithoutReuseOrInterpolation)
{
  FeedbackPairs pairs;
  pairs.push(0, {1.001, 1, 0, 0, 1});
  pairs.push(0, {1.010, 2, 0, 0, 2});
  pairs.push(1, {1.000, 3, 0, 0, 1});
  ASSERT_TRUE(pairs.select(1.012, 0.02, 0.003));
  EXPECT_DOUBLE_EQ((*pairs.pair())[0].position, 1);
  EXPECT_DOUBLE_EQ((*pairs.pair())[1].position, 3);
  EXPECT_FALSE(pairs.select(1.013, 0.02, 0.003));
  pairs.push(0, {1.014, 4, 0, 0, 3});
  pairs.push(1, {1.015, 5, 0, 0, 2});
  ASSERT_TRUE(pairs.select(1.016, 0.02, 0.003));
  EXPECT_DOUBLE_EQ((*pairs.pair())[0].position, 4);
  EXPECT_NEAR(pairs.age(1.03), 0.016, 1e-12);
  pairs.push(1, {1.02, 6, 0, 0, 3});
  EXPECT_FALSE(pairs.select(1.03, 0.02, 0.003));
  EXPECT_NEAR(pairs.age(1.03), 0.016, 1e-12);
  EXPECT_THROW(pairs.push(1, {1.02, 6, 0, 0, 3}), std::runtime_error);
}

TEST(BilateralPairs, FutureAndNonFiniteSamplesAreRejected)
{
  FeedbackPairs pairs;
  EXPECT_THROW(pairs.push(0, {NAN, 0, 0, 0, 1}), std::invalid_argument);
  pairs.push(0, {2, 0, 0, 0, 1});
  pairs.push(1, {2, 0, 0, 0, 1});
  EXPECT_THROW(pairs.select(1, 0.02, 0.005), std::runtime_error);
  pairs.reset(2);
  EXPECT_FALSE(pairs.select(2, 0.02, 0.005));
}

TEST(BilateralPairs, CoalescesUnconsumedFeedbackAtTheSameTimeUsingTheNewestSequence)
{
  FeedbackPairs pairs;
  pairs.push(0, {1.001, 1, 0, 0, 1});
  EXPECT_NO_THROW(pairs.push(0, {1.001, 2, 0.2, 0.1, 2}));
  pairs.push(1, {1.000, 3, 0, 0, 1});
  EXPECT_NO_THROW(pairs.push(1, {1.000, 4, -0.3, -0.1, 2}));
  ASSERT_TRUE(pairs.select(1.002, 0.02, 0.003));
  const auto & pair = *pairs.pair();
  EXPECT_DOUBLE_EQ(pair[0].time, 1.001);
  EXPECT_DOUBLE_EQ(pair[0].position, 2);
  EXPECT_DOUBLE_EQ(pair[0].velocity, 0.2);
  EXPECT_DOUBLE_EQ(pair[0].torque, 0.1);
  EXPECT_EQ(pair[0].sequence, 2u);
  EXPECT_DOUBLE_EQ(pair[1].time, 1.000);
  EXPECT_DOUBLE_EQ(pair[1].position, 4);
  EXPECT_DOUBLE_EQ(pair[1].velocity, -0.3);
  EXPECT_DOUBLE_EQ(pair[1].torque, -0.1);
  EXPECT_EQ(pair[1].sequence, 2u);
}

TEST(BilateralPairs, CoalescedConsumedFeedbackDoesNotRewriteOrReuseTheSelectedPair)
{
  FeedbackPairs pairs;
  pairs.push(0, {1, 1, 0, 0, 1});
  pairs.push(1, {1, 2, 0, 0, 1});
  ASSERT_TRUE(pairs.select(1.001, 0.02, 0.003));
  pairs.push(0, {1, 3, 0, 0, 2});
  pairs.push(1, {1, 4, 0, 0, 2});
  EXPECT_FALSE(pairs.select(1.002, 0.02, 0.003));
  EXPECT_DOUBLE_EQ((*pairs.pair())[0].position, 1);
  EXPECT_DOUBLE_EQ((*pairs.pair())[1].position, 2);
  EXPECT_EQ((*pairs.pair())[0].sequence, 1u);
  EXPECT_EQ((*pairs.pair())[1].sequence, 1u);
  EXPECT_NEAR(pairs.age(1.002), 0.002, 1e-12);
  pairs.push(0, {1.003, 5, 0, 0, 3});
  EXPECT_FALSE(pairs.select(1.004, 0.02, 0.005));
  pairs.push(1, {1.003, 6, 0, 0, 3});
  ASSERT_TRUE(pairs.select(1.004, 0.02, 0.005));
  EXPECT_DOUBLE_EQ((*pairs.pair())[0].position, 5);
  EXPECT_DOUBLE_EQ((*pairs.pair())[1].position, 6);

  pairs.reset(2);
  for (size_t axis = 0; axis < 2; ++axis) {
    pairs.push(axis, {2, 7, 0, 0, 4});
    pairs.push(axis, {2, 8, 0, 0, 5});
  }
  EXPECT_FALSE(pairs.select(2.001, 0.02, 0.005));
  EXPECT_FALSE(pairs.pair().has_value());
  for (size_t axis = 0; axis < 2; ++axis) {pairs.push(axis, {2.001, 9, 0, 0, 6});}
  ASSERT_TRUE(pairs.select(2.002, 0.02, 0.005));
}

TEST(BilateralPairs, CoalescingStillRejectsTimeRegressionAndNonIncreasingSequences)
{
  FeedbackPairs pairs;
  pairs.push(0, {1, 1, 0, 0, 4});
  EXPECT_THROW(pairs.push(0, {1, 2, 0, 0, 4}), std::runtime_error);
  EXPECT_THROW(pairs.push(0, {1.001, 2, 0, 0, 4}), std::runtime_error);
  EXPECT_THROW(pairs.push(0, {1.001, 2, 0, 0, 3}), std::runtime_error);
  EXPECT_THROW(pairs.push(0, {0.999, 2, 0, 0, 5}), std::runtime_error);
  EXPECT_THROW(pairs.push(0, {1.001, 2, 0, 0, 0}), std::runtime_error);
  EXPECT_NO_THROW(pairs.push(0, {1, 3, 0, 0, 5}));
  pairs.push(1, {1, 4, 0, 0, 1});
  ASSERT_TRUE(pairs.select(1.001, 0.02, 0.003));
  EXPECT_EQ((*pairs.pair())[0].sequence, 5u);
  EXPECT_DOUBLE_EQ((*pairs.pair())[0].position, 3);
}

TEST(BilateralRecovery, RequiresThreeNewPairsAndRampsAfterDegradation)
{
  Settings s;
  Recovery recovery;
  recovery.update(0.03, false, 0.005, 0.005, s);
  EXPECT_EQ(recovery.quality, Quality::Degraded);
  EXPECT_DOUBLE_EQ(recovery.scale, 0);
  EXPECT_EQ(recovery.degradation_count, 1u);
  recovery.update(0.005, true, 0.005, 0.005, s);
  EXPECT_EQ(recovery.fresh_pairs, 1u);
  recovery.update(0.01, false, 0.005, 0.005, s);
  EXPECT_EQ(recovery.fresh_pairs, 1u);
  EXPECT_DOUBLE_EQ(recovery.scale, 0);
  recovery.update(0.005, true, 0.005, 0.005, s);
  EXPECT_DOUBLE_EQ(recovery.scale, 0);
  recovery.update(0.005, true, 0.005, 0.005, s);
  EXPECT_NEAR(recovery.scale, 0.05, 1e-12);
  for (int i = 0; i < 20; ++i) {recovery.update(0.005, true, 0.005, 0.005, s);}
  EXPECT_EQ(recovery.quality, Quality::Normal);
  EXPECT_DOUBLE_EQ(recovery.scale, 1);
  recovery.update(0.03, false, 0.005, 0.005, s);
  EXPECT_DOUBLE_EQ(recovery.scale, 0);
  EXPECT_EQ(recovery.fresh_pairs, 0u);
  EXPECT_THROW(recovery.update(0.09, false, 0.005, 0.005, s), std::runtime_error);
}

TEST(BilateralDisturbance, BusyRequestsCannotExtendAnAbsoluteDeadlineOrQueueAnotherPulse)
{
  DisturbancePulse pulse;
  EXPECT_EQ(pulse.start(0, 0.3, 0.2, 10), 1u);
  EXPECT_TRUE(pulse.active());
  EXPECT_DOUBLE_EQ(pulse.update(10, true)[0], 0.3);
  EXPECT_DOUBLE_EQ(pulse.update(10.1, true)[1], 0);
  EXPECT_NEAR(pulse.remaining(10.1), 0.1, 1e-12);
  EXPECT_THROW(pulse.start(1, -0.2, 0.2, 10.19), std::runtime_error);
  EXPECT_EQ(pulse.id(), 1u);
  EXPECT_EQ(pulse.axis(), 0u);
  EXPECT_NEAR(pulse.remaining(10.19), 0.01, 1e-12);
  const auto ended = pulse.update(10.2, true);
  EXPECT_FALSE(pulse.active());
  EXPECT_DOUBLE_EQ(ended[0], 0);
  EXPECT_DOUBLE_EQ(ended[1], 0);
  EXPECT_DOUBLE_EQ(pulse.torque(), 0);
  EXPECT_DOUBLE_EQ(pulse.remaining(10.2), 0);
  EXPECT_EQ(pulse.start(1, -0.2, 0.1, 11), 2u);
  EXPECT_DOUBLE_EQ(pulse.update(11, true)[1], -0.2);
  // A delayed control tick cannot stretch a pulse to compensate for missed ticks.
  EXPECT_DOUBLE_EQ(pulse.update(12, true)[1], 0);
  EXPECT_FALSE(pulse.active());
}

TEST(BilateralDisturbance, LossOfPermissionAndExplicitStopCancelWithoutRecoveryReplay)
{
  DisturbancePulse pulse;
  pulse.start(1, -0.2, 0.2, 1);
  EXPECT_DOUBLE_EQ(pulse.update(1.02, false)[1], 0);
  EXPECT_FALSE(pulse.active());
  EXPECT_DOUBLE_EQ(pulse.remaining(1.03), 0);
  EXPECT_DOUBLE_EQ(pulse.update(1.04, true)[1], 0);
  EXPECT_EQ(pulse.id(), 1u);
  EXPECT_EQ(pulse.start(0, 0.1, 0.2, 2), 2u);
  pulse.cancel();
  EXPECT_DOUBLE_EQ(pulse.update(2.01, true)[0], 0);
  EXPECT_DOUBLE_EQ(pulse.remaining(2.01), 0);
  EXPECT_EQ(pulse.start(0, -0.1, 0.1, 3), 3u);
  EXPECT_DOUBLE_EQ(pulse.update(3.01, true)[0], -0.1);
}

TEST(BilateralDisturbance, InvalidRequestsDoNotMutateAPulseAndInvalidClocksCancelIt)
{
  DisturbancePulse pulse;
  for (const double torque : {0.0, -0.30001, 0.30001,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(pulse.start(0, torque, 0.1, 1), std::invalid_argument);
  }
  for (const double duration : {0.0, -0.1, 0.20001,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(pulse.start(0, 0.2, duration, 1), std::invalid_argument);
  }
  EXPECT_THROW(pulse.start(2, 0.2, 0.1, 1), std::invalid_argument);
  EXPECT_THROW(pulse.start(0, 0.2, 0.1, -1), std::invalid_argument);
  EXPECT_THROW(pulse.start(0, 0.2, 0.1, NAN), std::invalid_argument);
  EXPECT_EQ(pulse.id(), 0u);
  EXPECT_FALSE(pulse.active());
  pulse.start(0, 0.2, 0.2, 1);
  EXPECT_THROW(pulse.start(2, 0.2, 0.1, 1.01), std::invalid_argument);
  EXPECT_EQ(pulse.id(), 1u);
  EXPECT_NEAR(pulse.remaining(1.01), 0.19, 1e-12);
  pulse.update(1.02, true);
  EXPECT_THROW(pulse.update(1.01, true), std::invalid_argument);
  EXPECT_FALSE(pulse.active());
  EXPECT_DOUBLE_EQ(pulse.update(1.03, true)[0], 0);
  pulse.start(0, 0.2, 0.2, 2);
  EXPECT_THROW(pulse.update(NAN, true), std::invalid_argument);
  EXPECT_FALSE(pulse.active());
  EXPECT_DOUBLE_EQ(pulse.update(2.01, true)[0], 0);
}

TEST(BilateralDisturbance, PulseAndFrictionShareThePositiveDecodedBudgetWithoutChangingComponents)
{
  Settings settings;
  settings.max_torque = 0.3; settings.stiffness = 16;
  settings.coupling_damping = settings.local_damping = 0.35;
  std::array<AxisLimits, 2> limits;
  limits[0].mapping = {10, 20, 5};
  limits[1].mapping = {12.5, 50, 10}; limits[1].direction = -1; limits[1].offset = 0.4;
  const std::array<Sample, 2> samples{{{1, -0.2, -0.3, 0, 1}, {1, 0.6, 0.3, 0, 1}}};
  for (const int direction : {-1, 1}) {
    settings.direction = direction;
    auto desired = relative_targets(samples, {0, 0.4}, settings);
    desired[0].t = 0.06; desired[1].t = -0.04;
    const auto baseline = symmetric_targets(desired, samples, limits, settings, 1);
    for (const double sign : {-1.0, 1.0}) {
      for (size_t disturbed_axis = 0; disturbed_axis < 2; ++disturbed_axis) {
        std::array<double, 2> disturbance{};
        disturbance[disturbed_axis] = sign * 0.2;
        for (const double recovery : {0.0, 0.25, 1.0}) {
          const auto output = symmetric_targets(desired, samples, limits, settings, recovery, disturbance);
          EXPECT_DOUBLE_EQ(output.targets[0].kp, output.targets[1].kp);
          EXPECT_DOUBLE_EQ(output.targets[0].kd, output.targets[1].kd);
          for (size_t i = 0; i < 2; ++i) {
            const auto & target = output.targets[i];
            EXPECT_DOUBLE_EQ(output.friction_torque[i], desired[i].t * recovery);
            EXPECT_DOUBLE_EQ(output.disturbance_torque[i], disturbance[i] * recovery);
            EXPECT_DOUBLE_EQ(output.quantization_torque[i], 0);
            EXPECT_DOUBLE_EQ(target.t, output.friction_torque[i] + output.disturbance_torque[i]);
            EXPECT_NEAR(output.raw_torque[i], baseline.raw_torque[i] + disturbance[i], 1e-12);
            const dm_motor::Command native{limits[i].direction * (target.p - limits[i].offset),
              limits[i].direction * target.v, target.kp, target.kd, limits[i].direction * target.t};
            const auto decoded = dm_motor::decode_mit(dm_motor::mit_frame(1, native, limits[i].mapping), limits[i].mapping);
            const double actual = limits[i].direction * (decoded.kp *
              (decoded.p - limits[i].direction * (samples[i].position - limits[i].offset)) +
              decoded.kd * (decoded.v - limits[i].direction * samples[i].velocity) + decoded.t);
            EXPECT_NEAR(output.estimated_torque[i], actual, 1e-12);
            EXPECT_LE(std::abs(actual), settings.max_torque + 1e-12);
            if (recovery == 0) {EXPECT_DOUBLE_EQ(target.t, 0);}
          }
        }
      }
    }
  }
  settings.max_torque = 0.1;
  const auto desired = relative_targets(samples, {0, 0.4}, settings);
  EXPECT_THROW(symmetric_targets(desired, samples, limits, settings, 1, {0.2, 0}), std::invalid_argument);
}

TEST(BilateralDisturbance, UnlimitedPulseKeepsFrictionAndEncodingCorrectionSeparate)
{
  Settings settings;
  std::array<AxisLimits, 2> limits;
  limits[0].mapping = {10, 20, 5};
  limits[1].mapping = {12.5, 50, 10}; limits[1].direction = -1; limits[1].offset = 0.4;
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0.4, 0, 0, 1}}};
  auto desired = relative_targets(samples, {0, 0.4}, settings);
  desired[0].t = 0.07; desired[1].t = -0.06;
  const auto baseline = symmetric_targets(desired, samples, limits, settings, 1);
  for (const std::array<double, 2> disturbance : {std::array<double, 2>{0.2, 0}, {0, -0.2}}) {
    for (const double recovery : {0.0, 0.25, 1.0}) {
      const auto output = symmetric_targets(desired, samples, limits, settings, recovery, disturbance);
      for (size_t i = 0; i < 2; ++i) {
        const auto & target = output.targets[i];
        EXPECT_DOUBLE_EQ(target.kp, settings.stiffness * recovery);
        EXPECT_DOUBLE_EQ(target.kd, recovery);
        EXPECT_DOUBLE_EQ(output.friction_torque[i], desired[i].t * recovery);
        EXPECT_DOUBLE_EQ(output.disturbance_torque[i], disturbance[i] * recovery);
        EXPECT_NEAR(target.t, output.friction_torque[i] + output.quantization_torque[i] +
          output.disturbance_torque[i], 1e-12);
        EXPECT_NEAR(output.raw_torque[i], baseline.raw_torque[i] + disturbance[i], 1e-12);
        const dm_motor::Command native{limits[i].direction * (target.p - limits[i].offset),
          limits[i].direction * target.v, target.kp, target.kd, limits[i].direction * target.t};
        const auto decoded = dm_motor::decode_mit(dm_motor::mit_frame(1, native, limits[i].mapping), limits[i].mapping);
        const double actual = limits[i].direction * (decoded.kp *
          (decoded.p - limits[i].direction * (samples[i].position - limits[i].offset)) +
          decoded.kd * (decoded.v - limits[i].direction * samples[i].velocity) + decoded.t);
        EXPECT_NEAR(output.estimated_torque[i], actual, 1e-12);
        EXPECT_EQ(output.protocol_saturated[i], std::abs(actual) > limits[i].mapping.t);
        if (recovery == 0) {
          EXPECT_DOUBLE_EQ(target.t, 0);
          EXPECT_DOUBLE_EQ(output.quantization_torque[i], 0);
        }
      }
    }
  }
  // No pulse leaves precisely the existing path, without persistent output state.
  const auto ended = symmetric_targets(desired, samples, limits, settings, 1, {});
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(ended.targets[i].t, baseline.targets[i].t);
    EXPECT_DOUBLE_EQ(ended.estimated_torque[i], baseline.estimated_torque[i]);
    EXPECT_DOUBLE_EQ(ended.disturbance_torque[i], 0);
  }
}

TEST(BilateralDisturbance, PulseMustFitDriverFeedforwardAndOnlyResidualCorrectionCanBeClamped)
{
  Settings settings; settings.stiffness = 500;
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  auto desired = relative_targets(samples, {0, 0}, settings);
  desired[0].t = 0.049;
  std::array<AxisLimits, 2> limits;
  for (auto & axis : limits) {axis.command.t = 0.15; axis.mapping.v = 50;}
  const auto output = symmetric_targets(desired, samples, limits, settings, 1, {0.1, 0});
  EXPECT_DOUBLE_EQ(output.targets[0].t, 0.15);
  EXPECT_DOUBLE_EQ(output.friction_torque[0], 0.049);
  EXPECT_DOUBLE_EQ(output.disturbance_torque[0], 0.1);
  EXPECT_NEAR(output.quantization_torque[0], 0.001, 1e-12);
  EXPECT_THROW(symmetric_targets(desired, samples, limits, settings, 1, {0.11, 0}), std::invalid_argument);
  for (const double torque : {0.30001, -0.30001, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(symmetric_targets(desired, samples, limits, settings, 0, {torque, 0}), std::invalid_argument);
  }
}

TEST(BilateralAngleOffset, QuinticTransitionHoldsItsEndpointUntilTheCallerStops)
{
  AngleOffset offset;
  const double target = std::acos(-1.0) / 6;
  EXPECT_EQ(offset.start(target, 2, 5, 10), 1u);
  EXPECT_TRUE(offset.active());
  EXPECT_TRUE(offset.ramping());
  EXPECT_DOUBLE_EQ(offset.applied(), 0);
  EXPECT_DOUBLE_EQ(offset.remaining(10), 7);
  double previous = 0;
  for (unsigned step = 0; step <= 100; ++step) {
    const double u = step / 100.0;
    const double now = 10 + 2 * u;
    const double value = offset.update(now);
    EXPECT_NEAR(value, target * (10 * u * u * u - 15 * std::pow(u, 4) + 6 * std::pow(u, 5)), 1e-14);
    EXPECT_GE(value, previous);
    EXPECT_LE(value, target);
    EXPECT_DOUBLE_EQ(offset.update(now), value);  // Repeated ticks do not advance a transition.
    previous = value;
  }
  EXPECT_FALSE(offset.ramping());
  EXPECT_DOUBLE_EQ(offset.applied(), target);
  EXPECT_DOUBLE_EQ(offset.update(16.9), target);
  EXPECT_NEAR(offset.remaining(16.9), 0.1, 1e-12);
  EXPECT_TRUE(offset.expired(17));
  EXPECT_DOUBLE_EQ(offset.remaining(17), 0);
  EXPECT_THROW(offset.update(17), std::runtime_error);
  // Expiry must not create an enabled reverse step: the node must begin STOP first.
  EXPECT_TRUE(offset.active());
  EXPECT_DOUBLE_EQ(offset.applied(), target);
  EXPECT_THROW(offset.start(-target, 2, 5, 17), std::runtime_error);
  offset.cancel();
  EXPECT_FALSE(offset.active());
  EXPECT_FALSE(offset.ramping());
  EXPECT_DOUBLE_EQ(offset.applied(), 0);
  EXPECT_DOUBLE_EQ(offset.update(18), 0);
  EXPECT_EQ(offset.id(), 1u);
  EXPECT_EQ(offset.start(-target, 2, 5, 19), 2u);
  EXPECT_DOUBLE_EQ(offset.update(19), 0);
  EXPECT_NEAR(offset.update(20), -target / 2, 1e-12);
}

TEST(BilateralAngleOffset, InvalidRequestsAndBusyRetriesNeverRewriteTheReference)
{
  AngleOffset offset;
  for (const double value : {0.0, -0.524, 0.524, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(offset.start(value, 2, 5, 1), std::invalid_argument);
  }
  for (const double value : {-0.01, 5.01, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(offset.start(0.1, value, 5, 1), std::invalid_argument);
  }
  for (const double value : {0.0, -0.01, 10.01, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(offset.start(0.1, 2, value, 1), std::invalid_argument);
  }
  EXPECT_THROW(offset.start(0.1, 2, 5, -1), std::invalid_argument);
  EXPECT_THROW(offset.start(0.1, 2, 5, NAN), std::invalid_argument);
  EXPECT_FALSE(offset.active());
  EXPECT_EQ(offset.id(), 0u);
  offset.start(0.2, 2, 5, 1);
  EXPECT_NEAR(offset.update(2), 0.1, 1e-12);
  EXPECT_THROW(offset.start(-0.2, 5, 10, 2), std::runtime_error);
  EXPECT_DOUBLE_EQ(offset.target(), 0.2);
  EXPECT_DOUBLE_EQ(offset.remaining(2), 6);
  EXPECT_EQ(offset.id(), 1u);
  EXPECT_THROW(offset.update(1.9), std::invalid_argument);
  EXPECT_THROW(offset.update(NAN), std::invalid_argument);
  EXPECT_NEAR(offset.applied(), 0.1, 1e-12);
  EXPECT_TRUE(offset.active());  // The caller handles the exception through dual STOP.
  offset.cancel();
  EXPECT_DOUBLE_EQ(offset.update(2.1), 0);
}

TEST(BilateralAngleOffset, SimulationStepReportsDemandBeyondTmaxWithoutRemovingFixedGains)
{
  Settings settings;
  const double angle = std::acos(-1.0) / 6;
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  AngleOffset offset;
  offset.start(angle, 0, 5, 1);
  EXPECT_DOUBLE_EQ(offset.applied(), 0);  // Applied telemetry changes at the controller update.
  EXPECT_DOUBLE_EQ(offset.update(1), angle);
  EXPECT_FALSE(offset.ramping());
  const auto base = relative_targets(samples, {0, 0}, settings);
  const auto desired = apply_angle_offset(base, offset.applied(), settings.direction);
  EXPECT_NO_THROW(validate_angle_offset_path(base, samples, {}, settings, angle));
  const auto output = symmetric_targets(desired, samples, {}, settings, 1);
  EXPECT_NEAR(output.raw_torque[0], -50 * angle, 1e-12);
  EXPECT_NEAR(output.raw_torque[1], 50 * angle, 1e-12);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(output.targets[i].kp, 50);
    EXPECT_DOUBLE_EQ(output.targets[i].kd, 1);
    EXPECT_TRUE(output.protocol_saturated[i]);
    EXPECT_GT(std::abs(output.estimated_torque[i]), 10);
    EXPECT_DOUBLE_EQ(output.disturbance_torque[i], 0);
    EXPECT_DOUBLE_EQ(output.friction_torque[i], 0);
  }
  settings.max_torque = 0.5;
  const auto budgeted = symmetric_targets(desired, samples, {}, settings, 1);
  EXPECT_LT(budgeted.gain_scale, 1);
  EXPECT_LE(std::abs(budgeted.estimated_torque[0]), 0.5);
  EXPECT_LE(std::abs(budgeted.estimated_torque[1]), 0.5);
}

TEST(BilateralAngleOffset, PositiveAndNegativeOffsetsCloseTheVirtualGapWithoutRewritingRawCoordinates)
{
  Settings settings;
  const std::array<double, 2> origins{0.6, -0.4};
  std::array<AxisLimits, 2> limits;
  limits[0].mapping = {10, 20, 5};
  limits[1].mapping = {12.5, 50, 10}; limits[1].direction = -1; limits[1].offset = 0.3;
  for (const int direction : {-1, 1}) {
    settings.direction = direction;
    for (const double angle : {-0.4, 0.4}) {
      const std::array<Sample, 2> samples{{
          {1, origins[0] + 0.03, 0, 0, 1}, {1, origins[1] - direction * 0.02, 0, 0, 1}}};
      const auto base = relative_targets(samples, origins, settings);
      const auto desired = apply_angle_offset(base, angle, direction);
      const double raw_error = (samples[0].position - origins[0]) -
        direction * (samples[1].position - origins[1]);
      const auto output = symmetric_targets(desired, samples, limits, settings, 1);
      EXPECT_NEAR(raw_error, 0.05, 1e-12);
      EXPECT_NEAR(output.raw_torque[0], -settings.stiffness * (raw_error + angle), 1e-12);
      EXPECT_NEAR(output.raw_torque[1], direction * settings.stiffness * (raw_error + angle), 1e-12);
      EXPECT_NEAR(desired[0].p, base[0].p - angle, 1e-12);
      EXPECT_NEAR(desired[1].p, base[1].p + direction * angle, 1e-12);
      for (size_t i = 0; i < 2; ++i) {
        EXPECT_DOUBLE_EQ(desired[i].v, base[i].v);
        EXPECT_DOUBLE_EQ(desired[i].kp, base[i].kp);
        EXPECT_DOUBLE_EQ(desired[i].kd, base[i].kd);
        EXPECT_DOUBLE_EQ(desired[i].t, base[i].t);
      }
      // One possible equilibrium; no common-mode position term forces this midpoint.
      const std::array<Sample, 2> settled{{
          {2, origins[0] - angle / 2, 0, 0, 2},
          {2, origins[1] + direction * angle / 2, 0, 0, 2}}};
      const auto balanced = symmetric_targets(apply_angle_offset(
          relative_targets(settled, origins, settings), angle, direction), settled, limits, settings, 1);
      EXPECT_NEAR(balanced.raw_torque[0], 0, 1e-12);
      EXPECT_NEAR(balanced.raw_torque[1], 0, 1e-12);
      EXPECT_NEAR((settled[0].position - origins[0]) - direction *
        (settled[1].position - origins[1]), -angle, 1e-12);
    }
  }
}

TEST(BilateralAngleOffset, OffsetDoesNotBypassFilteredPeerVelocityOrBoundedPrediction)
{
  Settings settings;
  std::array<PeerVelocityFilter, 2> filters;
  std::array<Sample, 2> pair{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  for (size_t i = 0; i < 2; ++i) {filters[i].update(0, pair[i].time);}
  const std::array<Sample, 2> latest{{{1.004, 0.001, 0.2, 0, 2}, {1.004, -0.001, -0.2, 0, 2}}};
  for (size_t i = 0; i < 2; ++i) {
    pair[i] = latest[i]; pair[i].time = 1.002;
    pair[i].velocity = filters[i].update(pair[i].velocity, pair[i].time);
  }
  const auto base = relative_targets(pair, latest, {0, 0}, settings);
  const auto shifted = apply_angle_offset(base, 0.2, settings.direction);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(shifted[i].v, base[i].v);
    EXPECT_NEAR(shifted[i].v, 0.995 * pair[1 - i].velocity, 1e-12);
    EXPECT_LT(std::abs(shifted[i].v), 0.995 * std::abs(latest[1 - i].velocity));
    EXPECT_NEAR(shifted[i].p - base[i].p, i == 0 ? -0.2 : 0.2, 1e-12);
  }
}

TEST(BilateralAngleOffset, PathPreflightChecksBothEndpointsInJointAndNativeCoordinates)
{
  Settings settings;
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0.4, 0, 0, 1}}};
  const auto base = relative_targets(samples, {0, 0.4}, settings);
  std::array<AxisLimits, 2> limits;
  limits[1].direction = -1; limits[1].offset = 0.4;
  EXPECT_NO_THROW(validate_angle_offset_path(base, samples, limits, settings, 0.5));
  limits[0].position_min = -0.1;
  EXPECT_THROW(validate_angle_offset_path(base, samples, limits, settings, 0.2), std::invalid_argument);
  limits[0].position_min = -12.5;
  limits[1].mapping.p = 0.1;
  EXPECT_THROW(validate_angle_offset_path(base, samples, limits, settings, 0.2), std::invalid_argument);
  limits[1].mapping.p = 12.5; limits[1].command.p = 0.1;
  EXPECT_THROW(validate_angle_offset_path(base, samples, limits, settings, 0.2), std::invalid_argument);
  limits[1].command.p = 0;
  limits[0].position_max = 0.1;
  auto invalid_start = base; invalid_start[0].p = 0.2;
  EXPECT_THROW(validate_angle_offset_path(invalid_start, samples, limits, settings, 0.2), std::invalid_argument);
  for (const double angle : {0.524, -0.524, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(apply_angle_offset(base, angle, 1), std::invalid_argument);
  }
  EXPECT_THROW(apply_angle_offset(base, 0.1, 0), std::invalid_argument);
  const auto unchanged = apply_angle_offset(base, 0, 1);
  EXPECT_DOUBLE_EQ(unchanged[0].p, base[0].p);
  EXPECT_DOUBLE_EQ(unchanged[1].p, base[1].p);
}

namespace
{
Settings shared_settings()
{
  Settings settings;
  settings.control_mode = ControlMode::SharedReference;
  settings.coupling_damping = 0.4975;
  return settings;
}

void expect_same_command(const dm_motor::Command & a, const dm_motor::Command & b)
{
  EXPECT_DOUBLE_EQ(a.p, b.p); EXPECT_DOUBLE_EQ(a.v, b.v);
  EXPECT_DOUBLE_EQ(a.kp, b.kp); EXPECT_DOUBLE_EQ(a.kd, b.kd); EXPECT_DOUBLE_EQ(a.t, b.t);
}
}  // namespace

TEST(BilateralSharedReference, NativeDefaultAndModeSpecificGainBoundariesAreExplicit)
{
  Settings native;
  EXPECT_EQ(native.control_mode, ControlMode::Native);
  EXPECT_DOUBLE_EQ(native_gains(native).kp, 50);
  EXPECT_DOUBLE_EQ(native_gains(native).kd, 1);
  auto shared = shared_settings();
  EXPECT_DOUBLE_EQ(native_gains(shared).kp, 100);
  EXPECT_DOUBLE_EQ(native_gains(shared).kd, 1);
  shared.stiffness = 250;
  EXPECT_DOUBLE_EQ(native_gains(shared).kp, 500);
  shared.stiffness = 250.001;
  EXPECT_THROW(native_gains(shared), std::invalid_argument);
  shared = shared_settings(); shared.coupling_damping = 0.497501;
  EXPECT_THROW(native_gains(shared), std::invalid_argument);
  shared = native; shared.control_mode = ControlMode::SharedReference;
  EXPECT_THROW(shared.validate(), std::invalid_argument);  // No silent halving of old Dc0.995.
  shared = shared_settings(); shared.control_mode = static_cast<ControlMode>(99);
  EXPECT_THROW(shared.validate(), std::invalid_argument);
  EXPECT_THROW(native_gains(shared), std::invalid_argument);
  for (const double value : {-0.001, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    shared = shared_settings(); shared.stiffness = value;
    EXPECT_THROW(native_gains(shared), std::invalid_argument);
    shared = shared_settings(); shared.coupling_damping = value;
    EXPECT_THROW(native_gains(shared), std::invalid_argument);
    shared = shared_settings(); shared.local_damping = value;
    EXPECT_THROW(native_gains(shared), std::invalid_argument);
  }
}

TEST(BilateralSharedReference, NativeOffsetOverloadPreservesAllCommandFieldsExactly)
{
  Settings settings;
  const std::array<double, 2> origins{0.6, -0.3};
  const std::array<Sample, 2> pair{{{1, 0.8, 0.2, 0, 1}, {1.001, -0.2, -0.1, 0, 1}}};
  for (const int direction : {-1, 1}) {
    settings.direction = direction;
    for (const double angle : {-0.4, 0., 0.4}) {
      auto desired = relative_targets(pair, pair, origins, settings);
      desired[0].t = 0.08; desired[1].t = -0.06;
      const auto legacy = apply_angle_offset(desired, angle, direction);
      const auto with_settings = apply_angle_offset(desired, angle, settings);
      for (size_t i = 0; i < 2; ++i) {expect_same_command(with_settings[i], legacy[i]);}
    }
  }
}

TEST(BilateralSharedReference, EffectiveSpringAndDampingLawRetainsBothJointDirections)
{
  const std::array<double, 2> origins{1.2, -0.7};
  for (const int direction : {-1, 1}) {
    auto settings = shared_settings(); settings.direction = direction;
    const std::array<Sample, 2> samples{{
        {1, origins[0] + 0.1, 0.3, 0, 1},
        {1, origins[1] - direction * 0.05, -direction * 0.2, 0, 1}}};
    for (const double angle : {-std::acos(-1.0) / 6, 0., std::acos(-1.0) / 6}) {
      const auto base = shared_reference_targets(samples, origins, settings);
      const auto desired = apply_angle_offset(base, angle, settings);
      const double e = 0.15 + angle, dv = 0.5;
      const auto output = symmetric_targets(desired, samples, {}, settings, 1);
      EXPECT_NEAR(output.raw_torque[0], -50 * e - 0.4975 * dv - 0.005 * 0.3, 1e-12);
      EXPECT_NEAR(output.raw_torque[1], direction * (50 * e + 0.4975 * dv + 0.005 * 0.2), 1e-12);
      for (size_t i = 0; i < 2; ++i) {
        EXPECT_DOUBLE_EQ(desired[i].kp, 100);
        EXPECT_DOUBLE_EQ(desired[i].kd, 1);
        EXPECT_DOUBLE_EQ(desired[i].v, base[i].v);  // No offset-velocity feedforward.
      }
      EXPECT_NEAR(desired[0].p - base[0].p, -angle / 2, 1e-12);
      EXPECT_NEAR(desired[1].p - base[1].p, direction * angle / 2, 1e-12);
    }
  }
}

TEST(BilateralSharedReference, CommonTranslationRemainsFreeAndDoesNotHoldTheCapturedMidpoint)
{
  for (const int direction : {-1, 1}) {
    auto settings = shared_settings(); settings.direction = direction;
    const std::array<double, 2> origins{2., -1.};
    for (const double translation : {-1.0, 0.0, 1.0}) {
      const std::array<Sample, 2> samples{{
          {1, origins[0] + translation, 0.2, 0, 1},
          {1, origins[1] + direction * translation, direction * 0.2, 0, 1}}};
      const auto desired = shared_reference_targets(samples, origins, settings);
      const auto output = symmetric_targets(desired, samples, {}, settings, 1);
      for (size_t i = 0; i < 2; ++i) {
        EXPECT_DOUBLE_EQ(desired[i].p, samples[i].position);
        EXPECT_NEAR(output.raw_torque[i], -settings.local_damping * samples[i].velocity, 1e-12);
      }
    }
  }
}

TEST(BilateralSharedReference, LatestAcquisitionFiltersDoNotPredictPositionOrFilterLocalFeedback)
{
  auto settings = shared_settings();
  std::array<PeerVelocityFilter, 2> filters;
  std::array<Sample, 2> latest{{{1, 0, 0, 0, 1}, {1.001, 0, 0, 0, 1}}};
  for (size_t i = 0; i < 2; ++i) {EXPECT_DOUBLE_EQ(filters[i].update(0, latest[i].time), 0);}
  latest[0] = {1.002, 0.04, 0.3, 0, 2};
  auto reference = latest;
  const auto filtered_reference = [&]() {
      for (size_t i = 0; i < 2; ++i) {
        // The node supplies bin-centered samples here; local estimates keep raw latest.
        reference[i] = latest[i];
        reference[i].velocity = filters[i].update(latest[i].velocity, latest[i].time);
      }
      return shared_reference_targets(reference, {0, 0}, settings);
    };
  const auto first = filtered_reference();
  EXPECT_NEAR(first[0].p, 0.02, 1e-12);
  EXPECT_DOUBLE_EQ(first[1].p, first[0].p);
  EXPECT_NEAR(first[0].v, 0.995 * 0.5 * 0.3 * -std::expm1(-0.002 / 0.008), 1e-12);
  for (unsigned repeat = 0; repeat < 20; ++repeat) {
    const auto unchanged = filtered_reference();
    for (size_t i = 0; i < 2; ++i) {expect_same_command(unchanged[i], first[i]);}
  }
  // B genuinely acquires a later sample, rather than retimestamping a future position.
  latest[1] = {1.003, -0.01, -0.1, 0, 2};
  const auto next = filtered_reference();
  EXPECT_NEAR(next[0].p, 0.015, 1e-12);
  EXPECT_DOUBLE_EQ(next[1].p, next[0].p);
  const auto output = symmetric_targets(next, latest, {}, settings, 1);
  EXPECT_NEAR(output.raw_torque[0], 100 * (0.015 - 0.04) + next[0].v - 0.3, 1e-12);
  EXPECT_NEAR(output.raw_torque[1], 100 * (0.015 + 0.01) + next[1].v + 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(latest[0].time, 1.002); EXPECT_DOUBLE_EQ(latest[1].time, 1.003);
  for (auto & filter : filters) {filter.reset();}
  const auto reset = filtered_reference();
  EXPECT_DOUBLE_EQ(reset[0].v, 0); EXPECT_DOUBLE_EQ(reset[1].v, 0);
  EXPECT_DOUBLE_EQ(reset[0].kp, 100); EXPECT_DOUBLE_EQ(reset[0].kd, 1);
}

TEST(BilateralSharedReference, ActualMitCapabilitiesAreCheckedBeforeRecoveryScaling)
{
  auto settings = shared_settings();
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  const auto desired = shared_reference_targets(samples, {0, 0}, settings);
  for (size_t axis = 0; axis < 2; ++axis) {
    for (const double recovery : {0., 0.1, 1.}) {
      std::array<AxisLimits, 2> limits;
      limits[axis].command.kp = 99;
      EXPECT_THROW(symmetric_targets(desired, samples, limits, settings, recovery), std::invalid_argument);
      limits[axis].command.kp = 100; limits[axis].command.kd = 0.99;
      EXPECT_THROW(symmetric_targets(desired, samples, limits, settings, recovery), std::invalid_argument);
      limits[axis].command.kd = 1;
      EXPECT_NO_THROW(symmetric_targets(desired, samples, limits, settings, recovery));
    }
  }
}

TEST(BilateralSharedReference, OutputMatchesIdenticalNativeMitDemandsAcrossBudgetsAndCoordinates)
{
  const std::array<double, 2> origins{0.1, -0.3};
  const std::array<Sample, 2> samples{{{1, 0.2, 0.3, 0, 1}, {1.001, -0.4, -0.2, 0, 1}}};
  for (const int direction : {-1, 1}) {
    for (const double budget : {0., 0.3}) {
      auto settings = shared_settings(); settings.direction = direction; settings.max_torque = budget;
      auto native = settings; native.control_mode = ControlMode::Native;
      native.stiffness = 100; native.coupling_damping = 0; native.local_damping = 1;
      std::array<AxisLimits, 2> limits;
      limits[0].mapping = {10, 20, 5}; limits[0].direction = -1; limits[0].offset = -0.2;
      limits[1].mapping = {12.5, 50, 10}; limits[1].offset = 0.4;
      auto desired = apply_angle_offset(shared_reference_targets(samples, origins, settings), 0.2, settings);
      desired[0].t = 0.03; desired[1].t = -0.04;
      const std::array<double, 2> disturbance{0.1, -0.1};
      for (const double recovery : {0., 0.25, 1.}) {
        const auto actual = symmetric_targets(desired, samples, limits, settings, recovery, disturbance);
        const auto expected = symmetric_targets(desired, samples, limits, native, recovery, disturbance);
        EXPECT_DOUBLE_EQ(actual.gain_scale, expected.gain_scale);
        EXPECT_DOUBLE_EQ(actual.decoded_kp, expected.decoded_kp);
        EXPECT_DOUBLE_EQ(actual.decoded_kd, expected.decoded_kd);
        for (size_t i = 0; i < 2; ++i) {
          expect_same_command(actual.targets[i], expected.targets[i]);
          EXPECT_DOUBLE_EQ(actual.raw_torque[i], expected.raw_torque[i]);
          EXPECT_DOUBLE_EQ(actual.estimated_torque[i], expected.estimated_torque[i]);
          EXPECT_DOUBLE_EQ(actual.friction_torque[i], expected.friction_torque[i]);
          EXPECT_DOUBLE_EQ(actual.quantization_torque[i], expected.quantization_torque[i]);
          EXPECT_DOUBLE_EQ(actual.disturbance_torque[i], expected.disturbance_torque[i]);
          EXPECT_EQ(actual.protocol_saturated[i], expected.protocol_saturated[i]);
          if (budget > 0) {
            EXPECT_LE(std::abs(actual.estimated_torque[i]), budget + 1e-9);
            EXPECT_DOUBLE_EQ(actual.quantization_torque[i], 0);
          }
          if (recovery == 0) {
            EXPECT_DOUBLE_EQ(actual.targets[i].kp, 0); EXPECT_DOUBLE_EQ(actual.targets[i].kd, 0);
            EXPECT_DOUBLE_EQ(actual.targets[i].t, 0);
          }
        }
      }
    }
  }
}

TEST(BilateralSharedReference, QuantizationCorrectionUsesDecodedDoubledGainsAndFinalNativeEstimate)
{
  auto settings = shared_settings();
  const std::array<Sample, 2> samples{{{1, 0.012, 0.21, 0, 1}, {1.001, -0.009, -0.13, 0, 1}}};
  auto desired = shared_reference_targets(samples, {0, 0}, settings);
  desired[0].t = 0.03; desired[1].t = -0.04;
  std::array<AxisLimits, 2> limits;
  limits[0].mapping = {10, 20, 5}; limits[0].direction = -1; limits[0].offset = 0.2;
  limits[1].mapping = {12.5, 50, 10}; limits[1].offset = -0.3;
  const std::array<double, 2> disturbance{0.1, -0.1};
  const auto output = symmetric_targets(desired, samples, limits, settings, 1, disturbance);
  for (size_t i = 0; i < 2; ++i) {
    const auto & axis = limits[i];
    const auto & target = output.targets[i];
    const auto decode = [&](double feedforward) {
        const dm_motor::Command native{axis.direction * (target.p - axis.offset), axis.direction * target.v,
          target.kp, target.kd, axis.direction * feedforward};
        return dm_motor::decode_mit(dm_motor::mit_frame(1, native, axis.mapping), axis.mapping);
      };
    const double base_ff = desired[i].t + disturbance[i];
    const auto before = decode(base_ff);
    EXPECT_DOUBLE_EQ(before.kp, 100); EXPECT_DOUBLE_EQ(before.kd, 1);
    const double correction = before.kp * (target.p - (axis.direction * before.p + axis.offset)) +
      before.kd * (target.v - axis.direction * before.v);
    EXPECT_NEAR(output.quantization_torque[i], correction, 1e-12);
    EXPECT_NEAR(target.t, output.friction_torque[i] + output.disturbance_torque[i] +
      output.quantization_torque[i], 1e-12);
    const auto actual = decode(target.t);
    const double native_q = axis.direction * (samples[i].position - axis.offset);
    const double native_v = axis.direction * samples[i].velocity;
    EXPECT_NEAR(output.estimated_torque[i], axis.direction *
      (actual.kp * (actual.p - native_q) + actual.kd * (actual.v - native_v) + actual.t), 1e-12);
    EXPECT_EQ(output.protocol_saturated[i], std::abs(output.estimated_torque[i]) > axis.mapping.t);
  }
}

TEST(BilateralSharedReference, HalfOffsetPreservesFullVirtualDemandAndReportsPhysicalSaturation)
{
  auto settings = shared_settings();
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  const double angle = std::acos(-1.0) / 6;
  const auto base = shared_reference_targets(samples, {0, 0}, settings);
  const auto desired = apply_angle_offset(base, angle, settings);
  EXPECT_NO_THROW(validate_angle_offset_path(base, samples, {}, settings, angle));
  auto output = symmetric_targets(desired, samples, {}, settings, 1);
  EXPECT_NEAR(output.raw_torque[0], -50 * angle, 1e-12);
  EXPECT_NEAR(output.raw_torque[1], 50 * angle, 1e-12);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(output.targets[i].kp, 100); EXPECT_DOUBLE_EQ(output.targets[i].kd, 1);
    EXPECT_TRUE(output.protocol_saturated[i]);
  }
  settings.max_torque = 0.5;
  output = symmetric_targets(desired, samples, {}, settings, 1);
  EXPECT_LT(output.gain_scale, 1);
  for (size_t i = 0; i < 2; ++i) {EXPECT_LE(std::abs(output.estimated_torque[i]), 0.5);}
}

TEST(BilateralSharedReference, HalfOffsetPathStillChecksJointNativeAndVelocityBounds)
{
  auto settings = shared_settings();
  std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0.4, 0, 0, 1}}};
  auto base = shared_reference_targets(samples, {0, 0.4}, settings);
  std::array<AxisLimits, 2> limits;
  limits[0].position_min = -0.15;
  limits[1].mapping.p = 0.15; limits[1].direction = -1; limits[1].offset = 0.4;
  EXPECT_NO_THROW(validate_angle_offset_path(base, samples, limits, settings, 0.2));
  EXPECT_THROW(validate_angle_offset_path(base, samples, limits, settings, 0.4), std::invalid_argument);
  limits[0].position_min = -12.5;
  EXPECT_THROW(validate_angle_offset_path(base, samples, limits, settings, 0.4), std::invalid_argument);
  samples[0].velocity = samples[1].velocity = 0.2;
  base = shared_reference_targets(samples, {0, 0.4}, settings);
  limits[0].command.v = 0.01;
  EXPECT_THROW(symmetric_targets(base, samples, limits, settings, 1), std::invalid_argument);
  for (const double invalid : {-0.6, 0.6, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(apply_angle_offset(base, invalid, settings), std::invalid_argument);
  }
}

TEST(BilateralSharedReference, ZeroGainsInvalidSamplesAndWrongReferenceApiCannotHideErrors)
{
  auto settings = shared_settings();
  std::array<Sample, 2> samples{{{1, 0.1, 0.2, 0, 1}, {1.001, -0.1, -0.2, 0, 1}}};
  EXPECT_THROW(relative_targets(samples, {0, 0}, settings), std::invalid_argument);
  EXPECT_THROW(relative_targets(samples, samples, {0, 0}, settings), std::invalid_argument);
  EXPECT_THROW(shared_reference_targets(samples, {0, 0}, Settings{}), std::invalid_argument);
  settings.stiffness = settings.coupling_damping = settings.local_damping = 0;
  const auto desired = shared_reference_targets(samples, {0, 0}, settings);
  const auto output = symmetric_targets(desired, samples, {}, settings, 1);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(desired[i].v, 0); EXPECT_DOUBLE_EQ(desired[i].kp, 0);
    EXPECT_DOUBLE_EQ(desired[i].kd, 0); EXPECT_DOUBLE_EQ(output.raw_torque[i], 0);
  }
  for (size_t i = 0; i < 2; ++i) {
    auto invalid = samples; invalid[i].position = NAN;
    EXPECT_THROW(shared_reference_targets(invalid, {0, 0}, settings), std::invalid_argument);
    invalid = samples; invalid[i].velocity = INFINITY;
    EXPECT_THROW(shared_reference_targets(invalid, {0, 0}, settings), std::invalid_argument);
    invalid = samples; invalid[i].time = -0.1;
    EXPECT_THROW(shared_reference_targets(invalid, {0, 0}, settings), std::invalid_argument);
  }
  EXPECT_THROW(shared_reference_targets(samples, {NAN, 0}, settings), std::invalid_argument);
  samples[0].position = samples[1].position = std::numeric_limits<double>::max();
  EXPECT_THROW(shared_reference_targets(samples, {0, 0}, settings), std::invalid_argument);
}

TEST(BilateralSharedReference, ExpiredOffsetRetainsItsAppliedValueUntilExplicitStopCancellation)
{
  auto settings = shared_settings();
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  const auto base = shared_reference_targets(samples, {0, 0}, settings);
  AngleOffset offset;
  offset.start(0.2, 0, 0.1, 1);
  EXPECT_DOUBLE_EQ(offset.update(1), 0.2);
  EXPECT_TRUE(offset.expired(1.11));
  EXPECT_DOUBLE_EQ(offset.applied(), 0.2);  // Caller must dual STOP, never apply a reverse zero step.
  const auto last = apply_angle_offset(base, offset.applied(), settings);
  const auto neutral = symmetric_targets(last, samples, {}, settings, 0);
  for (const auto & target : neutral.targets) {
    EXPECT_DOUBLE_EQ(target.kp, 0); EXPECT_DOUBLE_EQ(target.kd, 0); EXPECT_DOUBLE_EQ(target.t, 0);
  }
  offset.cancel();
  const auto reset = apply_angle_offset(base, offset.update(1.2), settings);
  for (size_t i = 0; i < 2; ++i) {expect_same_command(reset[i], base[i]);}
}

TEST(BilateralCommonTime, InterpolatesOnlyPositionsAtTheOlderLatestTime)
{
  FeedbackPairs pairs;
  pairs.push(0, {1.008, -37 + 13.2 * .008, 101, 2, 1});
  pairs.push(1, {1.007, 25 + 13.2 * .007, -101, -2, 1});
  pairs.push(1, {1.009, 25 + 13.2 * .009, -102, -3, 2});
  pairs.push(1, {1.017, 25 + 13.2 * .017, -103, -4, 3});
  const auto common = pairs.common_time_positions(1.018, .02, .005);
  ASSERT_TRUE(common);
  EXPECT_DOUBLE_EQ(common->time, 1.008);
  EXPECT_NEAR(common->position[0], -37 + 13.2 * .008, 1e-12);
  EXPECT_NEAR(common->position[1], 25 + 13.2 * .008, 1e-12);
  EXPECT_DOUBLE_EQ(common->bracket_gap[0], 0);
  EXPECT_NEAR(common->bracket_gap[1], .002, 1e-12);
  EXPECT_NEAR(common->latest_age[0], .010, 1e-12);
  EXPECT_NEAR(common->latest_age[1], .001, 1e-12);
  // Selecting a display/health pair must neither consume nor change interpolation.
  pairs.select(1.018, .02, .005);
  const auto again = pairs.common_time_positions(1.018, .02, .005);
  ASSERT_TRUE(again);
  EXPECT_DOUBLE_EQ(again->position[1], common->position[1]);
}

TEST(BilateralCommonTime, MissingOldStaleWideAndFutureEvidenceNeverFallsBackToLatest)
{
  FeedbackPairs pairs;
  EXPECT_FALSE(pairs.common_time_positions(1, .02, .005));
  pairs.push(0, {1.010, 1, 0, 0, 1});
  pairs.push(1, {1.011, 2, 0, 0, 1});
  EXPECT_FALSE(pairs.common_time_positions(1.012, .02, .005));  // Would extrapolate B backward.
  pairs.reset();
  pairs.push(0, {1.010, 1, 0, 0, 1});
  pairs.push(1, {1.007, 2, 0, 0, 1});
  pairs.push(1, {1.013, 3, 0, 0, 2});
  EXPECT_FALSE(pairs.common_time_positions(1.014, .02, .005));  // Six-millisecond bracket.
  EXPECT_TRUE(pairs.common_time_positions(1.014, .02, .007));
  EXPECT_FALSE(pairs.common_time_positions(1.029, .02, .007));  // Old bracket endpoint, even though t* is fresh.
  EXPECT_FALSE(pairs.common_time_positions(1.031, .02, .007));
  EXPECT_THROW(pairs.common_time_positions(1.012, .02, .007), std::runtime_error);
  EXPECT_THROW(pairs.common_time_positions(NAN, .02, .005), std::invalid_argument);
  EXPECT_THROW(pairs.common_time_positions(1.014, 0, .005), std::invalid_argument);
  EXPECT_THROW(pairs.common_time_positions(1.014, .02, 0), std::invalid_argument);
  EXPECT_THROW(pairs.common_time_positions(1.014, .02, .03), std::invalid_argument);
  pairs.reset(1.020);
  EXPECT_FALSE(pairs.common_time_positions(1.021, .02, .005));
}

TEST(BilateralCommonTime, EqualTimestampReplacementHasNoZeroDivisionAndHoldExpires)
{
  FeedbackPairs pairs;
  pairs.push(0, {1, 2, 0, 0, 1});
  pairs.push(1, {1, 3, 0, 0, 1});
  pairs.push(1, {1, 4, 0, 0, 2});
  auto aligned = pairs.common_time_positions(1.001, .02, .005);
  ASSERT_TRUE(aligned);
  EXPECT_DOUBLE_EQ(aligned->position[1], 4);
  EXPECT_DOUBLE_EQ(aligned->bracket_gap[1], 0);
  // B continues while A delivery pauses. Keep the same bounded common time;
  // never turn B's new displacement into a false spring error.
  for (int i = 1; i <= 8; ++i) {
    pairs.push(1, {1 + i * .002, 4 + 13.2 * i * .002, 13.2, 0, static_cast<uint64_t>(i + 2)});
  }
  aligned = pairs.common_time_positions(1.017, .02, .005);
  ASSERT_TRUE(aligned);
  EXPECT_DOUBLE_EQ(aligned->time, 1);
  EXPECT_DOUBLE_EQ(aligned->position[1], 4);
  EXPECT_FALSE(pairs.common_time_positions(1.021, .02, .005));
}

TEST(BilateralCommonTime, WideNewestGapUsesLatestValidEndpointWithBoundedPeerInterpolation)
{
  for (const double gap : {.0081213531084358692, .010}) {
    for (const size_t wide_axis : {size_t{0}, size_t{1}}) {
      FeedbackPairs pairs;
      const size_t regular_axis = 1 - wide_axis;
      const std::array<double, 2> origin{-37, 25};
      for (uint64_t i = 0; i < 5; ++i) {
        const double t = 1.001 + .002 * i;
        pairs.push(regular_axis, {t, origin[regular_axis] + 13.2 * (t - 1), -99, 2, i + 1});
      }
      pairs.push(wide_axis, {1.002, origin[wide_axis] + 13.2 * .002, 88, -3, 1});
      pairs.push(wide_axis, {1.002 + gap, origin[wide_axis] + 13.2 * (.002 + gap), 77, -4, 2});
      std::string reason = "previous failure";
      const double now = 1.002 + gap + .0005;
      const auto aligned = pairs.common_time_positions(now, .02, .008, &reason);
      ASSERT_TRUE(aligned);
      EXPECT_TRUE(reason.empty());
      EXPECT_EQ(aligned->time, 1.002);
      EXPECT_EQ(aligned->bracket_gap[wide_axis], 0);
      EXPECT_NEAR(aligned->bracket_gap[regular_axis], .002, 1e-12);
      EXPECT_NEAR(aligned->position[0] - origin[0], 13.2 * .002, 1e-12);
      EXPECT_NEAR(aligned->position[1] - origin[1], 13.2 * .002, 1e-12);
      // These remain ages of the actual latest samples, not the held endpoint.
      EXPECT_NEAR(aligned->latest_age[wide_axis], .0005, 1e-12);
      EXPECT_NEAR(aligned->latest_age[regular_axis], now - 1.009, 1e-12);
    }
  }
}

TEST(BilateralCommonTime, SearchesPastAlternatingWideGapsAndAllowsExactCommonEndpoints)
{
  FeedbackPairs pairs;
  pairs.push(0, {1.000, 0, 0, 0, 1});
  pairs.push(0, {1.002, .002, 0, 0, 2});
  pairs.push(0, {1.012, .012, 0, 0, 3});
  pairs.push(1, {1.001, .001, 0, 0, 1});
  pairs.push(1, {1.003, .003, 0, 0, 2});
  pairs.push(1, {1.014, .014, 0, 0, 3});
  // t=1.012 is in B's wide gap; t=1.003 is in A's wide gap.
  // The next candidate is an A endpoint inside B's real two-ms bracket.
  const auto aligned = pairs.common_time_positions(1.015, .02, .008);
  ASSERT_TRUE(aligned);
  EXPECT_EQ(aligned->time, 1.002);
  EXPECT_EQ(aligned->bracket_gap[0], 0);
  EXPECT_NEAR(aligned->bracket_gap[1], .002, 1e-12);
  EXPECT_NEAR(aligned->position[0], aligned->position[1], 1e-12);

  pairs.reset();
  pairs.push(0, {1.002, 2, 0, 0, 1});
  pairs.push(0, {1.012, 3, 0, 0, 2});
  pairs.push(1, {1.002, -2, 0, 0, 1});
  pairs.push(1, {1.014, -3, 0, 0, 2});
  const auto exact = pairs.common_time_positions(1.015, .02, .008);
  ASSERT_TRUE(exact);
  EXPECT_EQ(exact->time, 1.002);
  EXPECT_EQ(exact->position, (std::array<double, 2>{2, -2}));
  EXPECT_EQ(exact->bracket_gap, (std::array<double, 2>{0, 0}));
}

TEST(BilateralCommonTime, NoValidIntersectionAndExpiredContributingEndpointsStillReject)
{
  FeedbackPairs pairs;
  pairs.push(0, {1.000, 0, 0, 0, 1});
  pairs.push(0, {1.012, 0, 0, 0, 2});
  pairs.push(1, {1.006, 0, 0, 0, 1});
  pairs.push(1, {1.018, 0, 0, 0, 2});
  std::string reason;
  EXPECT_FALSE(pairs.common_time_positions(1.019, .02, .008, &reason));
  EXPECT_NE(reason.find("bracket_gap axis=2"), std::string::npos);  // First failure is retained.

  pairs.reset();
  pairs.push(0, {1.009, 0, 0, 0, 1});
  pairs.push(0, {1.011, 0, 0, 0, 2});
  pairs.push(1, {1.0045, 0, 0, 0, 1});
  pairs.push(1, {1.012, 0, 0, 0, 2});
  // The common time is only14ms old, but the contributing B endpoint is20.5ms old.
  EXPECT_FALSE(pairs.common_time_positions(1.025, .02, .008, &reason));
  EXPECT_NE(reason.find("stale_bracket axis=2"), std::string::npos);

  pairs.reset();
  pairs.push(0, {1.001, 0, 0, 0, 1});
  pairs.push(0, {1.009, 0, 0, 0, 2});
  pairs.push(1, {1.002, 0, 0, 0, 1});
  pairs.push(1, {1.010121353, 0, 0, 0, 2});
  // Both latest samples are fresh, but the only usable earlier endpoint expired.
  EXPECT_FALSE(pairs.common_time_positions(1.0221, .02, .008));
}

TEST(BilateralCommonTime, LatestClockFailureCannotBeHiddenByAnOlderCommonPoint)
{
  FeedbackPairs pairs;
  pairs.push(0, {1.0, 0, 0, 0, 1});
  pairs.push(1, {1.0, 0, 0, 0, 1});
  pairs.push(0, {1.009, 0, 0, 0, 2});
  pairs.push(1, {1.011, 0, 0, 0, 2});
  EXPECT_THROW(pairs.common_time_positions(1.010, .02, .008), std::runtime_error);
  EXPECT_THROW(pairs.common_time_positions(NAN, .02, .008), std::invalid_argument);
  EXPECT_THROW(pairs.push(1, {NAN, 0, 0, 0, 3}), std::invalid_argument);
}

TEST(BilateralCommonTime, ChosenTimeDoesNotRetreatWhileEarlierValidEvidenceRemainsFresh)
{
  FeedbackPairs pairs;
  pairs.push(0, {1., 0, 0, 0, 1});
  pairs.push(1, {1., 0, 0, 0, 1});
  std::array<uint64_t, 2> sequence{1, 1};
  std::array<double, 2> latest{1, 1};
  double selected = 1;
  const auto add = [&](size_t axis, double t) {
      latest[axis] = t;
      pairs.push(axis, {t, 13.2 * (t - 1), 0, 0, ++sequence[axis]});
      const auto aligned = pairs.common_time_positions(std::max(latest[0], latest[1]) + .0001, .02, .008);
      ASSERT_TRUE(aligned);
      EXPECT_GE(aligned->time, selected);
      EXPECT_NEAR(aligned->position[0], aligned->position[1], 1e-12);
      selected = aligned->time;
    };
  add(0, 1.001); add(0, 1.003); add(1, 1.002);
  add(0, 1.005); add(0, 1.007); add(0, 1.009);
  EXPECT_EQ(selected, 1.002);
  add(1, 1.010121353);  // The wide arrival gap holds the last valid common time.
  EXPECT_EQ(selected, 1.002);
  add(0, 1.011);       // New real brackets permit forward progress again.
  EXPECT_EQ(selected, 1.010121353);
  add(1, 1.012121353);
  EXPECT_EQ(selected, 1.011);
}

TEST(BilateralCommonTime, ValidPreferredPathKeepsOriginalArithmeticDespiteOlderCandidates)
{
  FeedbackPairs pairs;
  pairs.push(0, {1., -40, 0, 0, 1});
  pairs.push(0, {1.008, -37.123456789, 0, 0, 2});
  pairs.push(1, {1., 24., 0, 0, 1});
  pairs.push(1, {1.007, 24.987654321, 0, 0, 2});
  pairs.push(1, {1.009, 25.135791113, 0, 0, 3});
  pairs.push(1, {1.017, 26., 0, 0, 4});
  const auto aligned = pairs.common_time_positions(1.018, .02, .008);
  ASSERT_TRUE(aligned);
  const double gap = 1.009 - 1.007;
  const double fraction = (1.008 - 1.007) / gap;
  EXPECT_EQ(aligned->time, 1.008);
  EXPECT_EQ(aligned->position[0], -37.123456789);
  EXPECT_EQ(aligned->position[1], 24.987654321 + fraction * (25.135791113 - 24.987654321));
  EXPECT_EQ(aligned->bracket_gap, (std::array<double, 2>{0, gap}));
  EXPECT_EQ(aligned->latest_age, (std::array<double, 2>{1.018 - 1.008, 1.018 - 1.017}));
}

TEST(BilateralCommonTime, RejectionDiagnosticsDistinguishHistoryGapAndAgesWithoutChangingResults)
{
  FeedbackPairs pairs;
  std::string reason = "previous rejection";
  EXPECT_FALSE(pairs.common_time_positions(1.014, .02, .005, &reason));
  EXPECT_NE(reason.find("missing_history axis=1"), std::string::npos);
  pairs.push(0, {1.010, 1, 0, 0, 1});
  pairs.push(1, {1.011, 2, 0, 0, 1});
  EXPECT_FALSE(pairs.common_time_positions(1.014, .02, .005, &reason));
  EXPECT_NE(reason.find("missing_bracket axis=2"), std::string::npos);
  pairs.reset();
  pairs.push(0, {1.010, 1, 0, 0, 1});
  pairs.push(1, {1.007, 2, 0, 0, 1});
  pairs.push(1, {1.013, 3, 0, 0, 2});
  EXPECT_FALSE(pairs.common_time_positions(1.014, .02, .005, &reason));
  EXPECT_NE(reason.find("bracket_gap axis=2 gap_s="), std::string::npos);
  EXPECT_NE(reason.find("limit="), std::string::npos);
  EXPECT_FALSE(pairs.common_time_positions(1.029, .02, .007, &reason));
  EXPECT_NE(reason.find("stale_bracket axis=2 age_s="), std::string::npos);
  EXPECT_FALSE(pairs.common_time_positions(1.031, .02, .007, &reason));
  EXPECT_NE(reason.find("stale_common age_s="), std::string::npos);
  const auto old_call = pairs.common_time_positions(1.014, .02, .007);
  const auto detailed = pairs.common_time_positions(1.014, .02, .007, &reason);
  ASSERT_TRUE(old_call); ASSERT_TRUE(detailed);
  EXPECT_TRUE(reason.empty());
  EXPECT_EQ(old_call->position, detailed->position);
  EXPECT_EQ(old_call->bracket_gap, detailed->bracket_gap);
}

TEST(BilateralCommonTime, DifferentialSpringRemovesEightMillisecondPhantomErrorForBothDirections)
{
  for (const int direction : {-1, 1}) {
    Settings settings;
    settings.control_mode = ControlMode::DifferentialTorque;
    settings.direction = direction;
    settings.coupling_damping = .4975;
    settings.local_damping = .005;
    const std::array<double, 2> origins{-37, 25};
    FeedbackPairs pairs;
    pairs.push(0, {1.008, origins[0] + 13.2 * .008, 0, 0, 1});
    pairs.push(1, {1.007, origins[1] + direction * 13.2 * .007, 0, 0, 1});
    pairs.push(1, {1.009, origins[1] + direction * 13.2 * .009, 0, 0, 2});
    const std::array<Sample, 2> latest{{
      {1.008, origins[0] + 13.2 * .008, 0, 0, 1},
      {1.016, origins[1] + direction * 13.2 * .016, 0, 0, 3}}};
    pairs.push(1, latest[1]);
    const auto aligned = pairs.common_time_positions(1.017, .02, .005);
    ASSERT_TRUE(aligned);
    auto desired = shared_reference_targets(latest, origins, settings);
    const auto old = symmetric_targets(desired, latest, {}, settings, 1);
    const auto output = symmetric_targets(desired, latest, {}, settings, 1, {}, aligned);
    EXPECT_NEAR(std::abs(old.coupling_torque[0]), 5.28, 1e-10);
    EXPECT_NEAR(output.coupling_torque[0], 0, 1e-10);
    EXPECT_NEAR(output.coupling_torque[1], 0, 1e-10);
    // Offset steps remain a real antisymmetric spring demand after alignment.
    desired = apply_angle_offset(desired, .03, settings);
    const auto offset = symmetric_targets(desired, latest, {}, settings, 1, {}, aligned);
    EXPECT_NEAR(offset.coupling_torque[0], -1.5, 1e-10);
    EXPECT_NEAR(offset.coupling_torque[0] + direction * offset.coupling_torque[1], 0, 1e-12);
  }
}

TEST(BilateralCommonTime, OnlyTheSpringUsesAlignedPositionsLatestDampingAndFeedforwardRemain)
{
  Settings settings;
  settings.control_mode = ControlMode::DifferentialTorque;
  settings.coupling_damping = .4975; settings.local_damping = .005;
  const std::array<Sample, 2> latest{{{1.008, .1056, 8, 1, 1}, {1.016, .2112, 12, 2, 1}}};
  auto desired = shared_reference_targets(latest, {0, 0}, settings);
  desired[0].t = .1; desired[1].t = -.07;
  const std::array<double, 2> disturbance{.02, -.03};
  const CommonTimePositions aligned{1.008, {.1056, .1056}, {0, .002}, {.009, .001}};
  const auto old = symmetric_targets(desired, latest, {}, settings, 1, disturbance);
  const auto output = symmetric_targets(desired, latest, {}, settings, 1, disturbance, aligned);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_DOUBLE_EQ(output.targets[i].v, old.targets[i].v);
    EXPECT_DOUBLE_EQ(output.targets[i].kd, old.targets[i].kd);
    EXPECT_DOUBLE_EQ(output.friction_torque[i], old.friction_torque[i]);
    EXPECT_DOUBLE_EQ(output.disturbance_torque[i], old.disturbance_torque[i]);
    EXPECT_DOUBLE_EQ(output.quantization_torque[i], old.quantization_torque[i]);
    EXPECT_NEAR(output.raw_torque[i] - old.raw_torque[i],
      output.coupling_torque[i] - old.coupling_torque[i], 1e-12);
    const auto decoded = dm_motor::decode_mit(dm_motor::mit_frame(1, output.targets[i], {}), {});
    EXPECT_NEAR(output.estimated_torque[i], decoded.kd * (decoded.v - latest[i].velocity) + decoded.t, 1e-12);
  }
  auto invalid = aligned; invalid.time = 1.009;
  EXPECT_THROW(symmetric_targets(desired, latest, {}, settings, 1, {}, invalid), std::invalid_argument);
  invalid = aligned; invalid.time = -1;
  EXPECT_THROW(symmetric_targets(desired, latest, {}, settings, 1, {}, invalid), std::invalid_argument);
  invalid = aligned; invalid.position[0] = NAN;
  EXPECT_THROW(symmetric_targets(desired, latest, {}, settings, 1, {}, invalid), std::invalid_argument);
  invalid = aligned; invalid.bracket_gap[1] = -1;
  EXPECT_THROW(symmetric_targets(desired, latest, {}, settings, 1, {}, invalid), std::invalid_argument);
}
