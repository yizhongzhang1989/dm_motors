#include "dm_motor_control/simulation/device.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <vector>
#include <unistd.h>

using namespace dm_motor::simulation;

TEST(FeedbackDynamics, PreservesInitialStateAndQueriesDoNotAdvanceTheMeasurementChain)
{
  FeedbackParameters p;
  p.velocity_time_constant = .02;
  p.feedback_velocity_source = "physical";
  p.feedback_velocity_time_constant = .01;
  p.feedback_velocity_delay = .0015;
  p.feedback_torque_time_constant = .02;
  p.feedback_torque_delay = .0015;
  FeedbackDynamics chain(p, .2, 10.0);
  EXPECT_DOUBLE_EQ(chain.velocity(10), .2);
  EXPECT_DOUBLE_EQ(chain.torque(10), 0);
  chain.advance(.001, 10.001, 1.2, 1);
  chain.advance(.001, 10.002, 1.2, 1);
  const double v = .2 + .5 * -std::expm1(-.001 / .01);
  const double torque = .5 * -std::expm1(-.001 / .02);
  for (int i = 0; i < 10; ++i) {
    EXPECT_NEAR(chain.velocity(10.002), v, 1e-12);
    EXPECT_NEAR(chain.torque(10.002), torque, 1e-12);
    EXPECT_NEAR(chain.internal_velocity(), .2 + -std::expm1(-.002 / .02), 1e-14);
  }
}

TEST(Device, LuGreMemorySurvivesStopEnableAndVirtualPowerCycle)
{
  DeviceParameters p;
  p.physics.friction_model = "lugre";
  p.physics.inertia = .01; p.physics.coulomb_friction = .12; p.physics.static_friction = .2;
  p.physics.friction_displacement_scale = .005; p.physics.stribeck_velocity = .04;
  p.physics.torque_time_constant = 0; p.integration_step = .0001;
  VirtualMotor switched(p), continuous(p);
  const auto values = [](const PhysicsState & s) {
      return std::vector<double>{s.position, s.velocity, s.motor_torque, s.mechanical_energy,
        s.bristle_displacement, s.bristle_torque, s.bristle_energy, s.total_storage,
        s.bristle_drive_work, s.bristle_dissipation, s.viscous_dissipation};
    };
  for (auto * motor : {&switched, &continuous}) {
    motor->receive(dm_motor::special_frame(1, 0xfc), 0);
    motor->external_torque(.05, .01, 0);
    motor->advance(.01);
    motor->external_torque(0, .1, .01);
  }
  const auto before = values(switched.state());
  const double before_z = switched.state().bristle_displacement;
  ASSERT_NE(before_z, 0);
  switched.receive(dm_motor::special_frame(1, 0xfd), .01);
  EXPECT_EQ(values(switched.state()), before);
  switched.power_cycle();
  EXPECT_EQ(values(switched.state()), before);
  switched.advance(.02); continuous.advance(.02);
  EXPECT_EQ(values(switched.state()), values(continuous.state()));
  EXPECT_NE(switched.state().bristle_displacement, before_z);
  const auto after_release = values(switched.state());
  switched.receive(dm_motor::special_frame(1, 0xfc), .02);
  EXPECT_EQ(values(switched.state()), after_release);
  switched.advance(.03); continuous.advance(.03);
  EXPECT_EQ(values(switched.state()), values(continuous.state()));
  // A new physical instance, unlike a virtual firmware power cycle, initializes z.
  VirtualMotor reconstructed(p);
  EXPECT_DOUBLE_EQ(reconstructed.state().bristle_displacement, p.physics.initial_bristle_displacement);
}

dm_motor::Frame reg(uint8_t opcode, uint8_t rid, uint32_t raw = 0)
{
  dm_motor::Frame frame{0x7ff, {1, 0, opcode, rid, 0, 0, 0, 0}};
  for (unsigned i = 0; i < 4; ++i) {frame.data[4 + i] = (raw >> (8 * i)) & 255;}
  return frame;
}

uint32_t bits(float value)
{
  uint32_t raw; std::memcpy(&raw, &value, sizeof(raw)); return raw;
}

TEST(Device, QueryTrafficDoesNotAcceleratePhysicsAndForceExpires)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  VirtualMotor slow(p), fast(p);
  slow.external_torque(2, 0.5, 0); fast.external_torque(2, 0.5, 0);
  for (int i = 1; i <= 100; ++i) {slow.receive(dm_motor::query_frame(1), i * 0.01);}
  for (int i = 1; i <= 1000; ++i) {fast.receive(dm_motor::query_frame(1), i * 0.001);}
  EXPECT_NEAR(slow.state().position, fast.state().position, 0.002);
  EXPECT_NEAR(slow.state().position, 0.75, 0.002);
  EXPECT_NEAR(slow.state().velocity, 1, 0.002);
  EXPECT_DOUBLE_EQ(slow.external_remaining(), 0);
}

TEST(Device, IdealVelocityEstimatePreservesPhysicalFeedback)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.initial_velocity = 0.2;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  ASSERT_DOUBLE_EQ(p.velocity_time_constant, 0);
  VirtualMotor motor(p);
  EXPECT_DOUBLE_EQ(motor.feedback().v, p.physics.initial_velocity);
  motor.external_torque(1, 0.1, 0);
  for (int i = 1; i <= 20; ++i) {
    motor.advance(i * p.integration_step);
    EXPECT_DOUBLE_EQ(motor.feedback().v, motor.state().velocity);
  }
  EXPECT_NEAR(motor.state().velocity, 0.21, 1e-12);
}

TEST(Device, VelocityEstimatorLagsAtEachPhysicsSubstepAndFeedsWireFeedback)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.initial_velocity = 0.2;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  p.velocity_time_constant = 0.01;
  VirtualMotor motor(p);
  EXPECT_DOUBLE_EQ(motor.feedback().v, p.physics.initial_velocity);
  motor.external_torque(1, 0.1, 0);
  motor.advance(2 * p.integration_step);
  const double response = -std::expm1(-p.integration_step / p.velocity_time_constant);
  double expected = p.physics.initial_velocity;
  for (int i = 1; i <= 2; ++i) {
    expected += response * (p.physics.initial_velocity + i * p.integration_step - expected);
  }
  EXPECT_NEAR(motor.feedback().v, expected, 1e-14);
  EXPECT_NEAR(motor.state().velocity, 0.201, 1e-14);
  EXPECT_LT(motor.feedback().v, motor.state().velocity);
  const auto replies = motor.receive(dm_motor::query_frame(1), motor.time());
  ASSERT_EQ(replies.size(), 1u);
  const auto decoded = dm_motor::decode_feedback(replies.front(), 1, 0, p.motor.mapping);
  ASSERT_TRUE(decoded);
  EXPECT_NEAR(decoded->v, expected, 2 * p.motor.mapping.v / 4095.0);
}

TEST(Device, MitDampingUsesTheSameLaggedVelocityAsFeedback)
{
  DeviceParameters p;
  p.physics.inertia = 0.1;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  p.physics.torque_time_constant = 0;
  p.velocity_time_constant = 0.01;
  VirtualMotor motor(p);
  motor.external_torque(1, 0.1, 0);
  motor.advance(p.integration_step);
  const double measured = motor.feedback().v, physical = motor.state().velocity;
  const auto frame = dm_motor::mit_frame(1, {0, 0, 0, 1, 0}, p.motor.mapping);
  const auto command = dm_motor::decode_mit(frame, p.motor.mapping);
  motor.receive(dm_motor::special_frame(1, 0xfc), motor.time());
  motor.receive(frame, motor.time());
  motor.advance(2 * p.integration_step);
  EXPECT_NEAR(motor.state().motor_torque, command.kd * (command.v - measured) + command.t, 1e-12);
  EXPECT_GT(std::abs(motor.state().motor_torque -
    (command.kd * (command.v - physical) + command.t)), 0.004);
}

TEST(Device, VelocityEstimateStaysContinuousAcrossDisableZeroAndPowerCycle)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  p.velocity_time_constant = 0.01;
  VirtualMotor motor(p);
  motor.external_torque(1, 0.1, 0);
  motor.advance(0.01);
  const double measured = motor.feedback().v, physical = motor.state().velocity;
  ASSERT_LT(measured, physical);
  motor.receive(dm_motor::special_frame(1, 0xfd), motor.time());
  EXPECT_DOUBLE_EQ(motor.feedback().v, measured);
  motor.receive(dm_motor::special_frame(1, 0xfe), motor.time());
  EXPECT_DOUBLE_EQ(motor.feedback().v, measured);
  motor.power_cycle();
  EXPECT_DOUBLE_EQ(motor.feedback().v, measured);
  EXPECT_DOUBLE_EQ(motor.state().velocity, physical);
  motor.advance(0.02);
  EXPECT_GT(motor.feedback().v, measured);
  EXPECT_LT(motor.feedback().v, physical);
}

TEST(Device, VelocityEstimatorRejectsInvalidTimeConstants)
{
  DeviceParameters p;
  for (double value : {-0.001, 0.100001, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()})
  {
    p.velocity_time_constant = value;
    EXPECT_THROW(VirtualMotor{p}, std::invalid_argument);
  }
  for (double value : {0.0, 0.1}) {
    p.velocity_time_constant = value;
    EXPECT_NO_THROW(VirtualMotor{p});
  }
}

TEST(Device, PhysicalFeedbackVelocityCanBypassTheInternalEstimator)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.initial_velocity = 0.2;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  p.velocity_time_constant = 0.01;
  VirtualMotor legacy(p);
  p.feedback_velocity_source = "physical";
  VirtualMotor physical(p);
  legacy.external_torque(1, 0.1, 0); physical.external_torque(1, 0.1, 0);
  legacy.advance(0.01); physical.advance(0.01);
  EXPECT_DOUBLE_EQ(physical.feedback().v, physical.state().velocity);
  EXPECT_LT(legacy.feedback().v, physical.feedback().v);
  EXPECT_DOUBLE_EQ(legacy.state().velocity, physical.state().velocity);
}

TEST(Device, ReportingDynamicsDoNotChangeMitInternalDampingOrPhysicalState)
{
  DeviceParameters p;
  p.physics.inertia = 0.01;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  p.velocity_time_constant = 0.01;
  VirtualMotor legacy(p);
  p.feedback_velocity_source = "physical";
  p.feedback_velocity_time_constant = 0.02;
  p.feedback_velocity_delay = 0.0015;
  p.feedback_torque_time_constant = 0.012;
  p.feedback_torque_delay = 0.003;
  VirtualMotor reported(p);
  const auto command = dm_motor::mit_frame(1, {0.01, 0.2, 20, 1, 0.1}, p.motor.mapping);
  for (auto * motor : {&legacy, &reported}) {
    motor->receive(dm_motor::special_frame(1, 0xfc), 0);
    motor->receive(command, 0);
  }
  double velocity_difference = 0, torque_difference = 0;
  for (int i = 1; i <= 100; ++i) {
    legacy.advance(i * p.integration_step); reported.advance(i * p.integration_step);
    EXPECT_DOUBLE_EQ(legacy.state().position, reported.state().position);
    EXPECT_DOUBLE_EQ(legacy.state().velocity, reported.state().velocity);
    EXPECT_DOUBLE_EQ(legacy.state().motor_torque, reported.state().motor_torque);
    EXPECT_DOUBLE_EQ(legacy.feedback().t, legacy.state().motor_torque);
    velocity_difference = std::max(velocity_difference, std::abs(legacy.feedback().v - reported.feedback().v));
    torque_difference = std::max(torque_difference, std::abs(legacy.feedback().t - reported.feedback().t));
  }
  EXPECT_GT(velocity_difference, 0.01);
  EXPECT_GT(torque_difference, 0.01);
}

TEST(Device, FeedbackVelocityFilterUsesSubstepsAndInterpolatesFractionalDelay)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.initial_velocity = 0.2;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  p.velocity_time_constant = 0.01;
  p.feedback_velocity_source = "physical";
  p.feedback_velocity_time_constant = 0.0045;
  p.feedback_velocity_delay = 0.00075;
  VirtualMotor motor(p);
  motor.external_torque(1, 0.1, 0);
  motor.advance(0.0005);
  EXPECT_DOUBLE_EQ(motor.feedback().v, p.physics.initial_velocity);
  motor.advance(0.005);
  std::vector<double> samples{p.physics.initial_velocity};
  const double response = -std::expm1(-p.integration_step / p.feedback_velocity_time_constant);
  for (int i = 1; i <= 10; ++i) {
    const double velocity = p.physics.initial_velocity + i * p.integration_step;
    samples.push_back((1 - response) * samples.back() + response * velocity);
  }
  // 5 ms - 0.75 ms falls halfway between the 4 and 4.5 ms samples.
  const double expected = (samples[8] + samples[9]) / 2;
  EXPECT_NEAR(motor.feedback().v, expected, 1e-13);
  EXPECT_NEAR(motor.state().velocity, 0.205, 1e-13);
  const auto wire = motor.receive(dm_motor::query_frame(1), motor.time()).front();
  const auto decoded = dm_motor::decode_feedback(wire, 1, 0, p.motor.mapping);
  ASSERT_TRUE(decoded);
  EXPECT_NEAR(decoded->v, expected, 2 * p.motor.mapping.v / 4095.0);
}

TEST(Device, FeedbackTorqueDelayDoesNotDelayTheActualActuator)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  p.physics.torque_time_constant = 0;
  p.feedback_torque_time_constant = 0.012;
  p.feedback_torque_delay = 0.00075;
  VirtualMotor motor(p);
  const auto frame = dm_motor::mit_frame(1, {0, 0, 0, 0, 1}, p.motor.mapping);
  const double torque = dm_motor::decode_mit(frame, p.motor.mapping).t;
  motor.receive(dm_motor::special_frame(1, 0xfc), 0);
  motor.receive(frame, 0);
  motor.advance(p.integration_step);
  EXPECT_DOUBLE_EQ(motor.state().motor_torque, torque);
  EXPECT_DOUBLE_EQ(motor.feedback().t, 0);
  motor.advance(2 * p.integration_step);
  EXPECT_DOUBLE_EQ(motor.state().motor_torque, torque);
  const double response = -std::expm1(-p.integration_step / p.feedback_torque_time_constant);
  EXPECT_NEAR(motor.feedback().t, torque * response / 2, 1e-13);
  EXPECT_GT(motor.state().velocity, 0);
  const auto wire = motor.receive(dm_motor::query_frame(1), motor.time()).front();
  const auto decoded = dm_motor::decode_feedback(wire, 1, 0, p.motor.mapping);
  ASSERT_TRUE(decoded);
  EXPECT_NEAR(decoded->t, motor.feedback().t, 2 * p.motor.mapping.t / 4095.0);
}

TEST(Device, ReportingHistoryRemainsContinuousAcrossDisableZeroAndPowerCycle)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  p.feedback_velocity_time_constant = p.feedback_torque_time_constant = 0.01;
  p.feedback_velocity_delay = p.feedback_torque_delay = 0.003;
  VirtualMotor motor(p);
  motor.receive(dm_motor::special_frame(1, 0xfc), 0);
  motor.receive(dm_motor::mit_frame(1, {0, 0, 0, 0, 0.5}, p.motor.mapping), 0);
  motor.advance(0.01);
  const auto feedback = motor.feedback();
  const double physical_velocity = motor.state().velocity;
  ASSERT_GT(feedback.t, 0);
  for (const auto opcode : {0xfd, 0xfe}) {
    motor.receive(dm_motor::special_frame(1, opcode), motor.time());
    EXPECT_DOUBLE_EQ(motor.feedback().v, feedback.v);
    EXPECT_DOUBLE_EQ(motor.feedback().t, feedback.t);
  }
  motor.power_cycle();
  EXPECT_DOUBLE_EQ(motor.feedback().v, feedback.v);
  EXPECT_DOUBLE_EQ(motor.feedback().t, feedback.t);
  EXPECT_DOUBLE_EQ(motor.state().velocity, physical_velocity);
  motor.advance(0.02);
  EXPECT_DOUBLE_EQ(motor.state().motor_torque, 0);
  EXPECT_GT(motor.feedback().t, 0);
  EXPECT_LT(motor.feedback().t, feedback.t);
}

TEST(Device, FeedbackHistorySupportsTheFullDelayAtMinimumIntegrationStep)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  p.integration_step = 1e-5;
  p.feedback_velocity_delay = p.feedback_torque_delay = 0.1;
  VirtualMotor motor(p);
  motor.external_torque(1, 1.5, 0);
  motor.advance(1);
  EXPECT_NEAR(motor.feedback().v, 0.9, 1e-8);
  EXPECT_NEAR(motor.state().velocity, 1, 1e-8);
}

TEST(Device, ReportingChannelsRejectInvalidSourcesAndLatencies)
{
  DeviceParameters p;
  p.feedback_velocity_source = "unknown";
  EXPECT_THROW(VirtualMotor{p}, std::invalid_argument);
  p.feedback_velocity_source = "internal";
  for (auto member : {&DeviceParameters::feedback_velocity_time_constant, &DeviceParameters::feedback_velocity_delay,
      &DeviceParameters::feedback_torque_time_constant, &DeviceParameters::feedback_torque_delay})
  {
    for (double value : {-0.001, 0.100001, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    {
      p.*member = value;
      EXPECT_THROW(VirtualMotor{p}, std::invalid_argument);
    }
    p.*member = 0.1;
    EXPECT_NO_THROW(VirtualMotor{p});
    p.*member = 0;
    EXPECT_NO_THROW(VirtualMotor{p});
  }
}

TEST(Device, FirmwareWatchdogExpiresWithoutCanCommands)
{
  VirtualMotor motor(DeviceParameters{});
  motor.receive(reg(0x55, 9, 4000), 0);
  motor.receive(dm_motor::special_frame(1, 0xfc), 0);
  EXPECT_EQ(motor.status(), 1);
  motor.advance(0.199);
  EXPECT_EQ(motor.status(), 1);
  motor.advance(0.202);
  EXPECT_EQ(motor.status(), 13);
  motor.receive(dm_motor::special_frame(1, 0xfd), 0.203);
  EXPECT_EQ(motor.status(), 13);
  motor.receive(dm_motor::special_frame(1, 0xfb), 0.204);
  EXPECT_EQ(motor.status(), 0);
}

TEST(Device, ExternalSpringMovesDisabledShaftWithoutTeleportingAndExpires)
{
  DeviceParameters parameters;
  parameters.physics.viscous_friction = parameters.physics.coulomb_friction =
    parameters.physics.static_friction = 0;
  VirtualMotor motor(parameters);
  motor.external_spring(0.5, 1, 0.2, 1, 0);
  EXPECT_DOUBLE_EQ(motor.state().position, 0);
  motor.advance(0.1);
  EXPECT_GT(motor.state().position, 0.02);
  EXPECT_LT(motor.state().position, 0.5);
  motor.advance(0.9);
  EXPECT_NEAR(motor.state().position, 0.5, 0.005);
  EXPECT_EQ(motor.status(), 0);
  motor.advance(1.01);
  EXPECT_DOUBLE_EQ(motor.external_remaining(), 0);
  motor.advance(1.02);
  EXPECT_DOUBLE_EQ(motor.state().external_torque, 0);
}

TEST(Device, DirectForceCancelsSpringAndInvalidSpringsAreRejected)
{
  VirtualMotor motor(DeviceParameters{});
  EXPECT_THROW(motor.external_spring(NAN, 1, 0.1, 0.3, 0), std::invalid_argument);
  EXPECT_THROW(motor.external_spring(1, 101, 0.1, 0.3, 0), std::invalid_argument);
  EXPECT_THROW(motor.external_spring(1, 1, -0.1, 0.3, 0), std::invalid_argument);
  EXPECT_THROW(motor.external_spring(1, 1, 0.1, 0, 0), std::invalid_argument);
  motor.external_spring(5, 1, 0.1, 1, 0);
  motor.advance(0.05);
  EXPECT_GT(motor.state().external_torque, 1);
  motor.external_torque(0, 0.1, 0.05);
  motor.advance(0.1);
  EXPECT_DOUBLE_EQ(motor.state().external_torque, 0);
}

TEST(Device, ExternalForceMagnitudeIsNotClamped)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  for (double torque : {100.0, -100.0, 1e6, -1e6}) {
    VirtualMotor motor(p);
    motor.external_torque(torque, 0.1, 0);
    motor.advance(0.001);
    EXPECT_DOUBLE_EQ(motor.state().external_torque, torque);
    EXPECT_NEAR(motor.state().velocity, torque * 0.001, 1e-8);
    EXPECT_NEAR(motor.state().position, torque * 0.0000005, 1e-8);
    EXPECT_DOUBLE_EQ(motor.state().motor_torque, 0);
    EXPECT_EQ(motor.status(), 0);
  }
  for (double goal : {-50.0, 50.0}) {
    VirtualMotor motor(p);
    motor.external_spring(goal, 1, 0, 0.1, 0);
    EXPECT_DOUBLE_EQ(motor.state().position, 0);
    motor.advance(0.0005);
    EXPECT_DOUBLE_EQ(motor.state().external_torque, goal);
    EXPECT_NEAR(motor.state().velocity, goal * 0.0005, 1e-10);
  }
}

TEST(Device, NumericalOverflowFaultsExplicitlyAndDoesNotPoisonTheLastState)
{
  VirtualMotor motor(DeviceParameters{});
  motor.set_anchor(0);
  motor.external_torque(std::numeric_limits<double>::max(), 0.1, 0);
  EXPECT_THROW(motor.advance(0.001), std::runtime_error);
  EXPECT_DOUBLE_EQ(motor.state().position, 0);
  EXPECT_DOUBLE_EQ(motor.state().velocity, 0);
  EXPECT_DOUBLE_EQ(motor.external_remaining(), 0);
  EXPECT_EQ(motor.status(), 13);
  EXPECT_FALSE(motor.anchor().active);
  motor.external_torque(0, 0.1, 0.002);
  motor.receive(dm_motor::special_frame(1, 0xfb), 0.003);
  EXPECT_EQ(motor.status(), 0);
  EXPECT_DOUBLE_EQ(motor.state().external_torque, 0);
}

TEST(Device, AnchorHasNoPreloadAndRestoresAfterTemporaryForceExpires)
{
  DeviceParameters p;
  p.physics.initial_position = 0.7;
  p.physics.viscous_friction = 0.2;
  p.physics.coulomb_friction = p.physics.static_friction = 0;
  VirtualMotor motor(p);
  EXPECT_THROW(motor.set_anchor(NAN), std::invalid_argument);
  motor.set_anchor(0.4);
  EXPECT_TRUE(motor.anchor().active);
  EXPECT_EQ(motor.anchor().revision, 1u);
  EXPECT_DOUBLE_EQ(motor.state().position, 0.7);
  EXPECT_DOUBLE_EQ(motor.anchor().position, 0.7);
  EXPECT_DOUBLE_EQ(motor.anchor_torque(), 0);
  EXPECT_THROW(motor.set_anchor(1), std::runtime_error);
  motor.external_torque(1, 0.15, 0);
  motor.advance(0.3);
  const double displaced = motor.state().position;
  EXPECT_GT(displaced, 0.8);
  EXPECT_EQ(motor.status(), 0);
  EXPECT_DOUBLE_EQ(motor.external_remaining(), 0);
  EXPECT_DOUBLE_EQ(motor.transient_torque(), 0);
  EXPECT_LT(motor.anchor_torque(), -0.05);
  motor.external_torque(0, 0.1, 0.3);
  EXPECT_TRUE(motor.anchor().active);
  motor.advance(1.5);
  EXPECT_LT(std::abs(motor.state().position - 0.7), std::abs(displaced - 0.7));
  motor.advance(3.0);
  EXPECT_NEAR(motor.state().position, 0.7, 0.005);
  EXPECT_TRUE(motor.anchor().active);
}

TEST(Device, AnchorIsAdditiveAndSurvivesDisableZeroAndPowerCycle)
{
  DeviceParameters p;
  p.physics.inertia = 1;
  p.physics.initial_position = 1;
  p.physics.viscous_friction = p.physics.coulomb_friction = p.physics.static_friction = 0;
  VirtualMotor motor(p);
  motor.clear_anchor();
  EXPECT_EQ(motor.anchor().revision, 1u);
  motor.set_anchor(0.2);
  motor.receive(dm_motor::special_frame(1, 0xfe), 0);
  EXPECT_TRUE(motor.anchor().active);
  EXPECT_DOUBLE_EQ(motor.anchor().position, 1);
  EXPECT_DOUBLE_EQ(motor.encoder_position(), 0);
  motor.external_spring(2, 1, 0.2, 0.1, 0);
  motor.advance(0.05);
  EXPECT_LT(motor.anchor_torque(), 0);
  EXPECT_GT(motor.transient_torque(), 0);
  EXPECT_NEAR(motor.state().external_torque, motor.transient_torque() + motor.anchor_torque(), 0.001);
  motor.receive(dm_motor::special_frame(1, 0xfd), 0.05);
  motor.power_cycle();
  EXPECT_TRUE(motor.anchor().active);
  EXPECT_EQ(motor.anchor().revision, 2u);
  EXPECT_DOUBLE_EQ(motor.transient_torque(), 0);
  motor.advance(0.06);
  EXPECT_LT(motor.state().external_torque, 0);
  motor.clear_anchor();
  EXPECT_EQ(motor.anchor().revision, 3u);
  motor.advance(0.07);
  EXPECT_DOUBLE_EQ(motor.state().external_torque, 0);
}

TEST(Device, FourModesUseTorqueDynamicsAndReturnFeedback)
{
  for (int mode = 1; mode <= 4; ++mode) {
    DeviceParameters p; p.motor.mode = static_cast<dm_motor::Mode>(mode);
    VirtualMotor motor(p);
    motor.receive(dm_motor::special_frame(dm_motor::control_id(1, p.motor.mode), 0xfc), 0);
    dm_motor::Target target; target.mode = p.motor.mode;
    if (mode == 1) {target.position = 0.1; target.kp = 4; target.kd = 0.5;}
    else if (mode == 3) {target.velocity = 0.2;}
    else {target.position = 0.1; target.velocity_limit = 0.5; target.current_ratio = mode == 4 ? 0.2 : 0;}
    const auto frame = dm_motor::target_frame(1, target, p.motor.mapping);
    for (int i = 0; i < 100; ++i) {
      const auto replies = motor.receive(frame, i * 0.005);
      ASSERT_EQ(replies.size(), 1u);
      EXPECT_TRUE(dm_motor::decode_feedback(replies.front(), 1, 0, p.motor.mapping));
    }
    EXPECT_GT(motor.state().position, 0.02);
    EXPECT_LE(std::abs(motor.state().motor_torque), p.physics.max_torque);
  }
}

TEST(Device, DeviceZeroChangesEncoderNotPhysicalPosition)
{
  DeviceParameters p; p.physics.initial_position = 1.0;
  VirtualMotor motor(p);
  const auto before = motor.state().mechanical_energy;
  motor.receive(dm_motor::special_frame(1, 0xfe), 0);
  EXPECT_DOUBLE_EQ(motor.state().position, 1);
  EXPECT_DOUBLE_EQ(motor.encoder_position(), 0);
  EXPECT_DOUBLE_EQ(motor.state().mechanical_energy, before);
  EXPECT_EQ(motor.register_value(80), bits(0));
}

TEST(Device, EveryPublishedRegisterHasARealReplyAndWriteRulesApply)
{
  VirtualMotor motor(DeviceParameters{});
  for (const auto & item : dm_motor::register_catalog()) {
    const auto replies = motor.receive(reg(0x33, item.rid), 0);
    ASSERT_EQ(replies.size(), 1u);
    EXPECT_EQ(replies.front().data[3], item.rid);
  }
  motor.receive(reg(0x55, 25, bits(2.5)), 0);
  EXPECT_EQ(motor.register_value(25), bits(2.5));
  EXPECT_THROW(motor.receive(reg(0x55, 60, bits(0)), 0), std::invalid_argument);
  EXPECT_THROW(motor.receive(reg(0x55, 25, bits(NAN)), 0), std::invalid_argument);
  EXPECT_THROW(motor.external_torque(NAN, 0.2, 0), std::invalid_argument);
  EXPECT_THROW(motor.external_torque(INFINITY, 0.2, 0), std::invalid_argument);
}

TEST(Device, SavePersistsAcrossVirtualPowerCycleAndNewDevice)
{
  char path[] = "/tmp/virtual_motor_flash_XXXXXX";
  const auto fd = mkstemp(path); ASSERT_GE(fd, 0); close(fd); unlink(path);
  DeviceParameters p; p.persistence_file = path;
  {
    VirtualMotor motor(p);
    motor.receive(reg(0x55, 25, bits(2.5)), 0);
    motor.receive(reg(0xaa, 1), 0);
    motor.receive(reg(0x55, 25, bits(1.0)), 0);
    motor.power_cycle();
    EXPECT_EQ(motor.register_value(25), bits(2.5));
  }
  {
    VirtualMotor motor(p);
    EXPECT_EQ(motor.register_value(25), bits(2.5));
  }
  std::filesystem::remove(path);
}

TEST(Device, FaultSourceMustBeRemovedBeforeExplicitClear)
{
  VirtualMotor motor(DeviceParameters{});
  motor.inject_fault(11);
  motor.receive(dm_motor::special_frame(1, 0xfb), 0);
  EXPECT_EQ(motor.status(), 11);
  motor.inject_fault(0);
  EXPECT_EQ(motor.status(), 11);
  motor.receive(dm_motor::special_frame(1, 0xfb), 0);
  EXPECT_EQ(motor.status(), 0);
  motor.set_voltage(5);
  motor.advance(0.01);
  EXPECT_EQ(motor.status(), 9);
}
