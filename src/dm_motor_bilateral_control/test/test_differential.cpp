#include "dm_motor_bilateral_control/bilateral.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

using namespace dm_motor_bilateral_control;

namespace
{
Settings differential()
{
  Settings settings;
  settings.control_mode = ControlMode::DifferentialTorque;
  settings.coupling_damping = .4975;
  return settings;
}

dm_motor::Command decode(const dm_motor::Command & target, const AxisLimits & axis)
{
  const dm_motor::Command native{axis.direction * (target.p - axis.offset),
    axis.direction * target.v, target.kp, target.kd, axis.direction * target.t};
  dm_motor::validate_command(native, axis.mapping, axis.command);
  return dm_motor::decode_mit(dm_motor::mit_frame(1, native, axis.mapping), axis.mapping);
}

void expect_same_command(
  const dm_motor::Command & actual, const dm_motor::Command & expected, const AxisLimits & axis)
{
  EXPECT_EQ(actual.p, expected.p);
  EXPECT_EQ(actual.v, expected.v);
  EXPECT_EQ(actual.kp, expected.kp);
  EXPECT_EQ(actual.kd, expected.kd);
  EXPECT_EQ(actual.t, expected.t);
  const auto wire = [&axis](const dm_motor::Command & target) {
      return dm_motor::mit_frame(1, {axis.direction * (target.p - axis.offset),
        axis.direction * target.v, target.kp, target.kd, axis.direction * target.t}, axis.mapping).data;
    };
  EXPECT_EQ(wire(actual), wire(expected));
}
}  // namespace

TEST(BilateralDifferential, EffectiveGainsAndCapabilitiesAreCheckedBeforeRecovery)
{
  auto settings = differential();
  EXPECT_DOUBLE_EQ(native_gains(settings).kp, 0);
  EXPECT_DOUBLE_EQ(native_gains(settings).kd, 1);
  settings.stiffness = 500;
  EXPECT_NO_THROW(settings.validate());
  settings.stiffness = 500.001;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings = differential();
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  const auto desired = shared_reference_targets(samples, {0, 0}, settings);
  std::array<AxisLimits, 2> limits;
  limits[1].command.kd = .8;
  EXPECT_THROW(symmetric_targets(desired, samples, limits, settings, 0), std::invalid_argument);
}

TEST(BilateralDifferential, RestoringForcesAreOppositeWithOriginsDirectionsAndAngleOffset)
{
  for (const int direction : {-1, 1}) {
    auto settings = differential(); settings.direction = direction;
    const std::array<double, 2> origins{1.2, -.7};
    const std::array<Sample, 2> samples{{
        {1, origins[0] + .02, .1, 0, 1},
        {1.001, origins[1] - direction * .01, direction * .1, 0, 1}}};
    for (const double offset : {-.04, 0., .04}) {
      auto desired = apply_angle_offset(shared_reference_targets(samples, origins, settings), offset, settings);
      desired[0].t = .03; desired[1].t = -.04;
      const auto output = symmetric_targets(desired, samples, {}, settings, 1);
      EXPECT_NEAR(output.coupling_torque[0], -50 * (.03 + offset), 1e-12);
      EXPECT_NEAR(output.coupling_torque[0] + direction * output.coupling_torque[1], 0, 1e-12);
      EXPECT_DOUBLE_EQ(output.targets[0].kp, 0);
      EXPECT_DOUBLE_EQ(output.targets[0].kd, 1);
      EXPECT_DOUBLE_EQ(output.gain_scale, 1);
      EXPECT_DOUBLE_EQ(output.coupling_scale, 1);
      EXPECT_FALSE(output.protocol_saturated[0]);
    }
  }
}

TEST(BilateralDifferential, OldCommonPositionReferenceCannotCreateCommonSpringResistance)
{
  for (const int direction : {-1, 1}) {
    auto settings = differential(); settings.direction = direction;
    const std::array<Sample, 2> old{{{1, .2, 1.3, 0, 1}, {1, direction * .2, direction * 1.3, 0, 1}}};
    const auto desired = shared_reference_targets(old, {0, 0}, settings);
    auto current = old;
    current[0].position += .006; current[1].position += direction * .005;
    const auto output = symmetric_targets(desired, current, {}, settings, 1);
    EXPECT_NEAR(output.coupling_torque[0], -.05, 1e-12);
    EXPECT_NEAR(output.coupling_torque[0] + direction * output.coupling_torque[1], 0, 1e-12);
    // Only the configured local damping dissipates ideal steady common motion.
    EXPECT_NEAR(output.raw_torque[0] + direction * output.raw_torque[1], -2 * .005 * 1.3, 1e-12);
    current[0].position += 8; current[1].position += direction * 8;
    const auto translated = symmetric_targets(desired, current, {}, settings, 1);
    EXPECT_NEAR(translated.coupling_torque[0], output.coupling_torque[0], 1e-11);
  }
}

TEST(BilateralDifferential, LargeSeparationLimitsSpringButRetainsLocalNativeDamping)
{
  auto settings = differential();
  const std::array<Sample, 2> samples{{{1, 12, 0, 0, 1}, {1.001, -12, 0, 0, 1}}};
  auto desired = shared_reference_targets(samples, {0, 0}, settings);
  desired[0].p += 100; desired[1].p += 100;  // P=0: unused reference cannot cause loss of control.
  desired[0].t = .17; desired[1].t = -.13;
  std::array<AxisLimits, 2> limits;
  limits[0].mapping = {10, 20, 5}; limits[0].direction = -1; limits[0].offset = -.2;
  limits[1].mapping = {12.5, 50, 10}; limits[1].offset = .4;
  const auto output = symmetric_targets(desired, samples, limits, settings, 1);
  EXPECT_DOUBLE_EQ(output.gain_scale, 1);
  EXPECT_GT(output.coupling_scale, 0);
  EXPECT_LT(output.coupling_scale, .01);
  EXPECT_DOUBLE_EQ(output.coupling_torque[0], -output.coupling_torque[1]);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_TRUE(output.protocol_saturated[i]);
    EXPECT_DOUBLE_EQ(output.targets[i].kp, 0);
    EXPECT_DOUBLE_EQ(output.targets[i].kd, 1);
    EXPECT_LE(std::abs(output.targets[i].t), limits[i].mapping.t);
    EXPECT_NO_THROW(decode(output.targets[i], limits[i]));
  }
}

TEST(BilateralDifferential, FinalWireTorqueMatchesEstimateAcrossBudgetsCoordinatesAndRecovery)
{
  for (const int direction : {-1, 1}) {
    for (const double budget : {0., .3, 1.}) {
      for (const double recovery : {0., .25, 1.}) {
        auto settings = differential(); settings.direction = direction; settings.max_torque = budget;
        const std::array<Sample, 2> samples{{{1, .12, .21, 0, 1}, {1.001, -.09, -.13, 0, 1}}};
        auto desired = shared_reference_targets(samples, {0, 0}, settings);
        desired[0].t = .03; desired[1].t = -.04;
        std::array<AxisLimits, 2> limits;
        limits[0].mapping = {10, 20, 5}; limits[0].direction = -1; limits[0].offset = .2;
        limits[1].mapping = {12.5, 50, 10}; limits[1].offset = -.3;
        const std::array<double, 2> pulse{.1, -.1};
        const auto output = symmetric_targets(desired, samples, limits, settings, recovery, pulse);
        for (size_t i = 0; i < 2; ++i) {
          const auto & axis = limits[i];
          const auto actual = decode(output.targets[i], axis);
          const double native_v = axis.direction * samples[i].velocity;
          EXPECT_NEAR(output.estimated_torque[i], axis.direction *
            (actual.kd * (actual.v - native_v) + actual.t), 1e-12);
          EXPECT_NEAR(output.targets[i].t, output.friction_torque[i] + output.disturbance_torque[i] +
            output.quantization_torque[i] + output.coupling_torque[i], 1e-12);
          EXPECT_DOUBLE_EQ(output.friction_torque[i], desired[i].t * recovery);
          EXPECT_DOUBLE_EQ(output.disturbance_torque[i], pulse[i] * recovery);
          EXPECT_DOUBLE_EQ(actual.kp, 0);
          if (budget > 0) {EXPECT_LE(std::abs(output.estimated_torque[i]), budget + 1e-9);}
          if (recovery == 0) {
            EXPECT_DOUBLE_EQ(output.targets[i].t, 0);
            EXPECT_DOUBLE_EQ(output.targets[i].kd, 0);
          }
        }
      }
    }
  }
}

TEST(BilateralDifferential, NonfiniteReferencesAndInvalidRecoveryAreRejected)
{
  const auto settings = differential();
  const std::array<Sample, 2> samples{{{1, 0, 0, 0, 1}, {1, 0, 0, 0, 1}}};
  auto desired = shared_reference_targets(samples, {0, 0}, settings);
  for (const double recovery : {-.1, 1.1, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(symmetric_targets(desired, samples, {}, settings, recovery), std::invalid_argument);
  }
  EXPECT_THROW(symmetric_targets(desired, samples, {}, settings, 1, {.31, 0}), std::invalid_argument);
  desired[0].p = NAN;
  EXPECT_THROW(symmetric_targets(desired, samples, {}, settings, 1), std::invalid_argument);
}

TEST(BilateralDifferential, OneWayRequiresDifferentialModeWithoutSharedBudget)
{
  auto settings = differential();
  EXPECT_EQ(settings.coupling_mode, CouplingMode::Bilateral);
  settings.coupling_mode = CouplingMode::ALeadsB;
  EXPECT_NO_THROW(settings.validate());
  for (const auto mode : {ControlMode::Native, ControlMode::SharedReference}) {
    settings.control_mode = mode;
    EXPECT_THROW(settings.validate(), std::invalid_argument);
  }
  settings.control_mode = ControlMode::DifferentialTorque;
  for (const double budget : {1e-12, .3, -1., std::numeric_limits<double>::quiet_NaN()}) {
    settings.max_torque = budget;
    EXPECT_THROW(settings.validate(), std::invalid_argument);
  }
  settings.max_torque = 0;
  settings.coupling_mode = static_cast<CouplingMode>(999);
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings.coupling_mode = CouplingMode::ALeadsB;
  settings.control_mode = static_cast<ControlMode>(999);
  EXPECT_THROW(settings.validate(), std::invalid_argument);

  settings = differential(); settings.coupling_mode = CouplingMode::ALeadsB;
  settings.coupling_damping = settings.local_damping = 0;
  const std::array<Sample, 2> samples{{{1, .2, .3, 0, 1}, {1, -.1, -.2, 0, 1}}};
  const auto targets = shared_reference_targets(samples, {0, 0}, settings);
  EXPECT_DOUBLE_EQ(targets[0].v, 0);
  EXPECT_DOUBLE_EQ(targets[1].v, 0);
  EXPECT_DOUBLE_EQ(targets[0].kd, 0);
}

TEST(BilateralDifferential, OneWayACommandIsIndependentOfBIncludingProtocolSaturation)
{
  for (const int direction : {-1, 1}) {
    for (const double motor_direction : {-1., 1.}) {
      for (const double recovery : {0., .25, 1.}) {
        auto settings = differential(); settings.direction = direction;
        settings.coupling_mode = CouplingMode::ALeadsB;
        const std::array<double, 2> origins{.2, -.1};
        std::array<AxisLimits, 2> limits;
        limits[0].mapping = {10, 20, 5}; limits[0].direction = motor_direction; limits[0].offset = .2;
        limits[1].mapping = {12.5, 50, 1}; limits[1].direction = -motor_direction;
        limits[1].offset = -.1; limits[1].command.t = .25;
        const auto run = [&](double position, double velocity, double torque, double friction) {
            const std::array<Sample, 2> latest{{{1, .35, .7, .2, 1}, {1.001, position, velocity, torque, 1}}};
            auto reference = latest;
            reference[0].velocity = .52;  // A's fixed, independently filtered input.
            auto desired = apply_angle_offset(shared_reference_targets(reference, origins, settings), .01, settings);
            desired[0].t = .03; desired[1].t = friction;
            const CommonTimePositions aligned{.999, {.349, position - .001}, {.002, .002}, {.003, .003}};
            return symmetric_targets(desired, latest, limits, settings, recovery, {.02, -.05}, aligned);
          };
        const auto baseline = run(.04, .1, 0, .02);
        for (const double position : {-1e8, -.1, .04, 1e8}) {
          for (const double velocity : {-13.2, 0., 13.2}) {
            for (const double torque : {-1000., 0., 1000.}) {
              const auto output = run(position, velocity, torque, -.04);
              expect_same_command(output.targets[0], baseline.targets[0], limits[0]);
              EXPECT_EQ(output.estimated_torque[0], baseline.estimated_torque[0]);
              EXPECT_EQ(output.raw_torque[0], baseline.raw_torque[0]);
              EXPECT_EQ(output.quantization_torque[0], baseline.quantization_torque[0]);
              EXPECT_EQ(output.protocol_saturated[0], baseline.protocol_saturated[0]);
              EXPECT_DOUBLE_EQ(output.coupling_torque[0], 0);
              EXPECT_DOUBLE_EQ(output.targets[0].kd, recovery);
              EXPECT_NO_THROW(decode(output.targets[0], limits[0]));
              EXPECT_NO_THROW(decode(output.targets[1], limits[1]));
              if (recovery == 0) {
                EXPECT_DOUBLE_EQ(output.targets[0].t, 0);
                EXPECT_DOUBLE_EQ(output.targets[1].t, 0);
                EXPECT_DOUBLE_EQ(output.targets[1].kd, 0);
              }
            }
          }
        }
        const auto saturated = run(1e8, .1, 0, .02);
        EXPECT_EQ(saturated.protocol_saturated[1], recovery > 0);
        EXPECT_FALSE(saturated.protocol_saturated[0]);
        if (recovery > 0) {
          EXPECT_LT(saturated.coupling_scale, 1e-8);
          EXPECT_DOUBLE_EQ(saturated.targets[1].kd, recovery);
        }
      }
    }
  }
}

TEST(BilateralDifferential, OneWayBMatchesBilateralWithoutSaturation)
{
  for (const int direction : {-1, 1}) {
    for (const double recovery : {0., .25, 1.}) {
      auto bilateral = differential(); bilateral.direction = direction;
      auto one_way = bilateral; one_way.coupling_mode = CouplingMode::ALeadsB;
      const std::array<double, 2> origins{1.2, -.7};
      const std::array<Sample, 2> latest{{
          {1, origins[0] + .12, .4, 0, 1},
          {1.002, origins[1] + direction * .118, -.23, 0, 1}}};
      auto reference = latest; reference[0].velocity = .3; reference[1].velocity = -.2;
      std::array<AxisLimits, 2> limits;
      limits[0].mapping = {10, 20, 5}; limits[0].direction = -1; limits[0].offset = .2;
      limits[1].mapping = {12.5, 50, 10}; limits[1].offset = -.3;
      const CommonTimePositions aligned{.999,
        {origins[0] + .119, origins[1] + direction * .117}, {.002, .004}, {.003, .003}};
      for (const double offset : {-.001, 0., .001}) {
        auto ordinary = apply_angle_offset(shared_reference_targets(reference, origins, bilateral), offset, bilateral);
        auto leading = apply_angle_offset(shared_reference_targets(reference, origins, one_way), offset, one_way);
        ordinary[0].t = leading[0].t = .03;
        ordinary[1].t = leading[1].t = -.04;
        EXPECT_EQ(ordinary[1].v, leading[1].v);
        const auto expected = symmetric_targets(ordinary, latest, limits, bilateral, recovery, {.02, -.03}, aligned);
        const auto actual = symmetric_targets(leading, latest, limits, one_way, recovery, {.02, -.03}, aligned);
        EXPECT_FALSE(expected.protocol_saturated[0]);
        EXPECT_FALSE(expected.protocol_saturated[1]);
        expect_same_command(actual.targets[1], expected.targets[1], limits[1]);
        EXPECT_EQ(actual.estimated_torque[1], expected.estimated_torque[1]);
        EXPECT_EQ(actual.raw_torque[1], expected.raw_torque[1]);
        EXPECT_EQ(actual.friction_torque[1], expected.friction_torque[1]);
        EXPECT_EQ(actual.quantization_torque[1], expected.quantization_torque[1]);
        EXPECT_EQ(actual.coupling_torque[1], expected.coupling_torque[1]);
        EXPECT_DOUBLE_EQ(actual.coupling_torque[0], 0);
      }
    }
  }
}

TEST(BilateralDifferential, OneWayKeepsCommonTimeSpringEvidenceForB)
{
  for (const int direction : {-1, 1}) {
    auto settings = differential(); settings.direction = direction;
    settings.coupling_mode = CouplingMode::ALeadsB;
    FeedbackPairs pairs;
    pairs.push(0, {1, .2, 0, 0, 1});
    pairs.push(0, {1.004, .24, 0, 0, 2});
    pairs.push(1, {1.002, direction * .19, 0, 0, 1});
    pairs.push(1, {1.006, direction * .23, 0, 0, 2});
    const auto aligned = pairs.common_time_positions(1.006, .02, .008);
    ASSERT_TRUE(aligned);
    const std::array<Sample, 2> latest{{
        {1.004, .24, 0, 0, 2}, {1.006, direction * .23, 0, 0, 2}}};
    const auto desired = shared_reference_targets(latest, {0, 0}, settings);
    const auto output = symmetric_targets(desired, latest, {}, settings, 1, {}, aligned);
    EXPECT_DOUBLE_EQ(output.coupling_torque[0], 0);
    EXPECT_NEAR(output.coupling_torque[1], direction * 1.5, 1e-11);
    const auto old_time = symmetric_targets(desired, latest, {}, settings, 1);
    EXPECT_NEAR(old_time.coupling_torque[1], direction * .5, 1e-11);
    expect_same_command(output.targets[0], old_time.targets[0], {});
    auto invalid = *aligned; invalid.time = 1.005;
    EXPECT_THROW(symmetric_targets(desired, latest, {}, settings, 1, {}, invalid), std::invalid_argument);
  }
}

TEST(BilateralDifferential, OneWayAResidualIsLocalDampingPlusFilterLagAndEncoding)
{
  for (const double velocity : {-.3, .3, -6., 6., -13.2, 13.2}) {
    for (const double motor_direction : {-1., 1.}) {
      auto settings = differential(); settings.coupling_mode = CouplingMode::ALeadsB;
      std::array<AxisLimits, 2> limits;
      limits[0].mapping = {10, 20, 5}; limits[0].direction = motor_direction;
      PeerVelocityFilter filter;
      for (int tick = 0; tick <= 160; ++tick) {
        const double time = 1 + .002 * tick;
        const double filtered = filter.update(velocity, time);
        if (tick != 0 && tick != 1 && tick != 4 && tick != 20 && tick != 160) {continue;}
        const std::array<Sample, 2> latest{{{time, .2, velocity, 0, 1}, {time, -.1, -velocity, 0, 1}}};
        auto reference = latest; reference[0].velocity = filtered;
        const auto desired = shared_reference_targets(reference, {0, 0}, settings);
        const double residual = 2 * settings.coupling_damping * (filtered - velocity) -
          settings.local_damping * velocity;
        for (const double recovery : {0., .25, 1.}) {
          const auto output = symmetric_targets(desired, latest, limits, settings, recovery);
          EXPECT_NEAR(output.raw_torque[0], residual, 1e-12);
          EXPECT_DOUBLE_EQ(output.coupling_torque[0], 0);
          const auto actual = decode(output.targets[0], limits[0]);
          const double encoded_damping = actual.kd * (desired[0].v - velocity);
          // Velocity-field quantization is explicitly cancelled in the torque
          // field; only its final torque bin and decoded native D remain.
          EXPECT_NEAR(output.estimated_torque[0], encoded_damping, 2 * limits[0].mapping.t / 4095. + 1e-12);
          EXPECT_NEAR(output.estimated_torque[0], limits[0].direction *
            (actual.kd * (actual.v - limits[0].direction * velocity) + actual.t), 1e-12);
          if (tick == 160) {EXPECT_NEAR(output.raw_torque[0], -settings.local_damping * velocity, 1e-11);}
        }
      }
    }
  }
}
