#include "dm_motor_bilateral_control/bilateral.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace dm_motor_bilateral_control;

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
    Settings s; s.direction = direction; s.max_torque = 8; s.local_damping = 0.2;
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
