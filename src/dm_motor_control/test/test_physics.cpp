#include "dm_motor_control/simulation/physics.hpp"
#include "dm_motor_control/simulation/lugre.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using namespace dm_motor::simulation;

PhysicsParameters frictionless()
{
  PhysicsParameters p;
  p.inertia = 1; p.viscous_friction = p.coulomb_friction = p.static_friction = 0;
  p.torque_time_constant = 0; p.no_load_speed = 1e9;
  return p;
}

TEST(Physics, DisabledMotorRespondsToExternalTorqueAnalytically)
{
  auto p = frictionless();
  JointPhysics model(p);
  for (int i = 0; i < 2000; ++i) {model.step(0.0005, 10, 2, false);}
  EXPECT_NEAR(model.state().velocity, 2.0, 1e-10);
  EXPECT_NEAR(model.state().position, 1.0, 1e-10);
  EXPECT_DOUBLE_EQ(model.state().motor_torque, 0);
  EXPECT_NEAR(model.state().mechanical_energy, 2.0, 1e-9);
}

TEST(Physics, DisabledRotorCoastsInsteadOfTeleportingToRest)
{
  auto p = frictionless(); p.initial_velocity = 1;
  JointPhysics model(p);
  for (int i = 0; i < 1000; ++i) {model.step(0.001, 0, 0, false);}
  EXPECT_NEAR(model.state().position, 1.0, 1e-10);
  EXPECT_DOUBLE_EQ(model.state().velocity, 1);
}

TEST(Physics, FrictionDissipatesEnergyAndStaticFrictionHoldsSmallLoads)
{
  auto p = frictionless(); p.initial_velocity = 1; p.viscous_friction = 0.2;
  JointPhysics model(p);
  for (int i = 0; i < 1000; ++i) {model.step(0.001, 0, 0, false);}
  EXPECT_NEAR(model.state().velocity, std::exp(-0.2), 3e-5);
  EXPECT_LT(model.state().mechanical_energy, 0.5);
  p.initial_velocity = 0; p.coulomb_friction = 0.1; p.static_friction = 0.2;
  JointPhysics static_model(p);
  for (int i = 0; i < 2000; ++i) {static_model.step(0.0005, 0, 0.15, false);}
  EXPECT_DOUBLE_EQ(static_model.state().position, 0);
  for (int i = 0; i < 2000; ++i) {static_model.step(0.0005, 0, 0.4, false);}
  EXPECT_GT(static_model.state().position, 0.1);
}

TEST(Physics, GravityAndContactProducePhysicalRestoringTorques)
{
  auto p = frictionless(); p.initial_position = 0.5; p.gravity_torque = 2;
  JointPhysics pendulum(p);
  pendulum.step(0.0005, 0, 0, false);
  EXPECT_NEAR(pendulum.state().gravity_torque, -2 * std::sin(0.5), 1e-12);
  EXPECT_LT(pendulum.state().velocity, 0);
  p.initial_position = 0; p.gravity_torque = 0; p.upper_limit = 0.1;
  p.contact_stiffness = 200; p.contact_damping = 20;
  JointPhysics contact(p);
  for (int i = 0; i < 10000; ++i) {contact.step(0.0005, 0, 1, false);}
  EXPECT_NEAR(contact.state().position, 0.105, 0.001);
  EXPECT_LT(contact.state().contact_torque, -0.9);
}

TEST(Physics, TorqueBandwidthSaturationAndHeatingAreNotInstantaneous)
{
  auto p = frictionless(); p.max_torque = 1; p.torque_time_constant = 0.01; p.heat_capacity = 1;
  JointPhysics model(p);
  model.step(0.001, 20, 0, true);
  EXPECT_NEAR(model.state().motor_torque, 1 - std::exp(-0.1), 1e-9);
  for (int i = 0; i < 1000; ++i) {model.step(0.001, 20, 0, true);}
  EXPECT_LE(model.state().motor_torque, 1);
  EXPECT_GT(model.state().rotor_temperature, p.ambient_temperature);
}

TEST(Physics, ConstantTorqueRegionPreservesBrakingAndBothMotoringDirections)
{
  for (double direction : {-1., 1.}) {
    for (const auto & point : std::vector<std::pair<double, double>>{
        {0, 8}, {5, 8}, {10, 8}, {30, 4}, {50, 0}, {60, 0}})
    {
      auto p = frictionless();
      p.max_torque = 8; p.no_load_speed = 50; p.constant_torque_speed = 10;
      p.initial_velocity = direction * point.first;
      JointPhysics motoring(p), braking(p), disabled(p), unsaturated(p);
      motoring.step(.001, direction * 20, 0, true);
      braking.step(.001, -direction * 20, 0, true);
      disabled.step(.001, direction * 20, 0, false);
      unsaturated.step(.001, direction, 0, true);
      EXPECT_DOUBLE_EQ(motoring.state().motor_torque, direction * point.second);
      EXPECT_DOUBLE_EQ(braking.state().motor_torque, -direction * 8);
      EXPECT_DOUBLE_EQ(disabled.state().motor_torque, 0);
      EXPECT_DOUBLE_EQ(unsaturated.state().motor_torque,
        direction * std::min(1., point.second));
    }
  }
}

TEST(Physics, ConstantTorqueEnvelopeIsContinuousAtKneeAndNoLoadSpeed)
{
  constexpr double epsilon = 1e-6;
  const auto torque = [](double speed) {
      auto p = frictionless();
      p.max_torque = 8; p.no_load_speed = 50; p.constant_torque_speed = 10;
      p.initial_velocity = speed;
      JointPhysics model(p);
      model.step(.001, 20, 0, true);
      return model.state().motor_torque;
    };
  EXPECT_DOUBLE_EQ(torque(10 - epsilon), 8);
  EXPECT_DOUBLE_EQ(torque(10), 8);
  EXPECT_NEAR(torque(10 + epsilon), 8 - .2 * epsilon, 1e-14);
  EXPECT_NEAR(torque(50 - epsilon), .2 * epsilon, 1e-14);
  EXPECT_DOUBLE_EQ(torque(50), 0);
  EXPECT_DOUBLE_EQ(torque(50 + epsilon), 0);
}

TEST(Physics, ZeroConstantTorqueSpeedRetainsTheLegacyEnvelope)
{
  for (double velocity : {-60., -50., -30., -10., -1., 0., 1., 10., 30., 50., 60.}) {
    for (double request : {-20., -1., 0., 1., 20.}) {
      auto p = frictionless();
      ASSERT_DOUBLE_EQ(p.constant_torque_speed, 0);
      p.max_torque = 8; p.no_load_speed = 50; p.initial_velocity = velocity;
      JointPhysics model(p);
      model.step(.001, request, 0, true);
      const double legacy_limit = request * velocity > 0 ?
        8 * std::max(0., 1 - std::abs(velocity) / 50) : 8;
      EXPECT_DOUBLE_EQ(model.state().motor_torque,
        std::clamp(request, -legacy_limit, legacy_limit));
    }
  }
}

TEST(Physics, ConstantTorqueEnvelopePrecedesActuatorLag)
{
  auto p = frictionless();
  p.max_torque = 8; p.no_load_speed = 50; p.constant_torque_speed = 10;
  p.initial_velocity = 5; p.torque_time_constant = .01;
  JointPhysics model(p);
  model.step(.001, 20, 0, true);
  EXPECT_NEAR(model.state().motor_torque, 8 * (1 - std::exp(-.1)), 1e-14);
  model.step(.001, 20, 0, false);
  EXPECT_DOUBLE_EQ(model.state().motor_torque, 0);
}

TEST(Physics, ConstantTorqueSpeedRequiresAFiniteKneeBelowNoLoadSpeed)
{
  for (double knee : {-1., 50., 51., std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
  {
    auto p = frictionless(); p.no_load_speed = 50; p.constant_torque_speed = knee;
    EXPECT_THROW(JointPhysics{p}, std::invalid_argument);
  }
  auto p = frictionless(); p.no_load_speed = 50;
  p.constant_torque_speed = std::nextafter(p.no_load_speed, 0.);
  EXPECT_NO_THROW(JointPhysics{p});
}

TEST(Physics, InvalidParametersAndInputsAreRejected)
{
  auto p = frictionless(); p.inertia = 0;
  EXPECT_THROW(JointPhysics{p}, std::invalid_argument);
  JointPhysics model(frictionless());
  EXPECT_THROW(model.step(0, 0, 0, false), std::invalid_argument);
  EXPECT_THROW(model.step(0.1, 0, 0, false), std::invalid_argument);
  EXPECT_THROW(model.step(0.001, NAN, 0, false), std::invalid_argument);
  EXPECT_THROW(model.step(0.001, 0, 1e308, false), std::runtime_error);
  EXPECT_DOUBLE_EQ(model.state().position, 0);
  EXPECT_DOUBLE_EQ(model.state().velocity, 0);
}

TEST(Physics, SelectableStribeckAndDirectionalFrictionRemainDissipative)
{
  for (const auto & name : {"viscous", "coulomb_viscous", "stribeck"}) {
    for (double velocity : {-0.1, 0.1}) {
      auto p = frictionless();
      p.friction_model = name; p.initial_velocity = velocity;
      p.coulomb_friction = 0.1; p.static_friction = 0.2; p.viscous_friction = 0.03;
      p.stribeck_velocity = 0.2; p.friction_negative_scale = 1.5;
      JointPhysics model(p);
      model.step(0.00025, 0, 0, false);
      EXPECT_LE(model.state().friction_torque * velocity, 0);
      EXPECT_LT(std::abs(model.state().velocity), std::abs(velocity));
      if (p.friction_model == "stribeck") {
        const double lower = (0.1 + 0.1 * std::exp(-0.25) +
          0.03 * std::abs(model.state().velocity)) * (velocity < 0 ? 1.5 : 1);
        const double upper = (0.1 + 0.1 * std::exp(-0.25) +
          0.03 * std::abs(velocity)) * (velocity < 0 ? 1.5 : 1);
        // Dry level is held within the segment; viscous impulse integrates
        // the changing velocity rather than using only the final velocity.
        EXPECT_GE(std::abs(model.state().friction_torque), lower - 1e-12);
        EXPECT_LE(std::abs(model.state().friction_torque), upper + 1e-12);
      }
    }
  }
  auto p = frictionless(); p.friction_model = "unsupported";
  EXPECT_THROW(JointPhysics{p}, std::invalid_argument);
  p.friction_model = "stribeck"; p.stribeck_velocity = 0;
  EXPECT_THROW(JointPhysics{p}, std::invalid_argument);
  p.stribeck_velocity = 0.1; p.friction_negative_scale = -1;
  EXPECT_THROW(JointPhysics{p}, std::invalid_argument);
}

TEST(Physics, JointSpringAndConstantLoadAreDistinctFromFriction)
{
  auto p = frictionless(); p.initial_position = 0.5; p.joint_stiffness = 0.4; p.load_torque = 0.1;
  JointPhysics model(p);
  model.step(0.0005, 0, 0, false);
  EXPECT_NEAR(model.state().load_torque, -0.3, 1e-12);
  EXPECT_NEAR(model.state().velocity, -0.00015, 1e-12);
  EXPECT_NEAR(model.state().friction_torque, 0, 1e-12);
}

TEST(Physics, ReversalUsesStaticThresholdInTheLoadDirection)
{
  auto p = frictionless(); p.initial_velocity = 1e-5;
  p.coulomb_friction = 0.05; p.static_friction = 0.1; p.friction_negative_scale = 2;
  JointPhysics model(p);
  model.step(0.001, 0, -0.15, false);
  EXPECT_DOUBLE_EQ(model.state().velocity, 0);
  EXPECT_NEAR(model.state().position, 2.5e-10, 1e-20);
  EXPECT_LE(model.state().mechanical_energy - .5 * p.initial_velocity * p.initial_velocity +
    .15 * model.state().position, 0);
  model.step(0.001, 0, -0.15, false);
  EXPECT_DOUBLE_EQ(model.state().velocity, 0);
  EXPECT_NEAR(model.state().position, 2.5e-10, 1e-20);
}

TEST(Physics, ForcedReversalSplitsDryFrictionAtZeroWithoutCreatingEnergy)
{
  for (double direction : {-1., 1.}) {
    auto p = frictionless();
    p.inertia = .0038742723463633427;
    p.coulomb_friction = .14; p.static_friction = .19;
    p.initial_velocity = direction * .0001;
    JointPhysics model(p);
    const double dt = .0001, drive = -direction * .3;
    const double stop = p.inertia * .0001 / (.3 + .14);
    const double first_travel = p.initial_velocity * stop / 2;
    const double remaining = dt - stop;
    const double acceleration = -direction * (.3 - .14) / p.inertia;
    const double expected_velocity = acceleration * remaining;
    const double second_travel = acceleration * remaining * remaining / 2;
    model.step(dt, 0, drive, false);
    const auto & s = model.state();
    EXPECT_NEAR(s.velocity, expected_velocity, 1e-14);
    EXPECT_NEAR(s.position, first_travel + second_travel, 1e-16);
    const double change = .5 * p.inertia *
      (s.velocity * s.velocity - p.initial_velocity * p.initial_velocity);
    const double friction_work = change - drive * s.position;
    EXPECT_LT(friction_work, 0);
    EXPECT_NEAR(friction_work, -.14 * (std::abs(first_travel) + std::abs(second_travel)), 1e-18);
  }
}

TEST(Physics, AStoppedRotorDoesNotTravelForTheRemainderOfTheStep)
{
  auto p = frictionless(); p.inertia = .01; p.initial_velocity = .0001;
  p.coulomb_friction = .1; p.static_friction = .2;
  JointPhysics model(p);
  model.step(.001, 0, .05, false);
  const double stop = p.inertia * p.initial_velocity / (.1 - .05);
  EXPECT_DOUBLE_EQ(model.state().velocity, 0);
  EXPECT_NEAR(model.state().position, p.initial_velocity * stop / 2, 1e-18);
  const double stopped_position = model.state().position;
  model.step(.001, 0, .05, false);
  EXPECT_DOUBLE_EQ(model.state().position, stopped_position);
}

TEST(Physics, ReversalReevaluatesTheNewDirectionFrictionScale)
{
  auto p = frictionless(); p.inertia = .01; p.initial_velocity = .0001;
  p.coulomb_friction = .1; p.static_friction = .15; p.friction_negative_scale = 2;
  JointPhysics model(p);
  const double dt = .001, stop = p.inertia * p.initial_velocity / (.4 + .1);
  const double remaining = dt - stop;
  model.step(dt, 0, -.4, false);
  EXPECT_NEAR(model.state().velocity, -(.4 - .2) / p.inertia * remaining, 1e-14);
  EXPECT_NEAR(model.state().position,
    .5 * p.initial_velocity * stop - .5 * (.4 - .2) / p.inertia * remaining * remaining, 1e-16);
}

TEST(Physics, ViscousReversalMatchesThePiecewiseAnalyticalSolution)
{
  auto p = frictionless(); p.inertia = .01; p.initial_velocity = .001;
  p.coulomb_friction = .1; p.static_friction = .2; p.viscous_friction = .5;
  JointPhysics model(p);
  const double dt = .001, scale = p.inertia / p.viscous_friction;
  const double first_equilibrium = (-.3 - .1) / p.viscous_friction;
  const double stop = scale * std::log1p(-p.initial_velocity / first_equilibrium);
  const double first_travel = first_equilibrium * stop +
    (p.initial_velocity - first_equilibrium) * scale * -std::expm1(-stop / scale);
  const double remaining = dt - stop, second_equilibrium = (-.3 + .1) / p.viscous_friction;
  const double expected_velocity = second_equilibrium * -std::expm1(-remaining / scale);
  const double second_travel = second_equilibrium *
    (remaining + scale * std::expm1(-remaining / scale));
  model.step(dt, 0, -.3, false);
  EXPECT_NEAR(model.state().velocity, expected_velocity, 1e-14);
  EXPECT_NEAR(model.state().position, first_travel + second_travel, 1e-16);
  EXPECT_LE(model.state().mechanical_energy - .5 * p.inertia * p.initial_velocity * p.initial_velocity -
    (-.3) * model.state().position, 0);
}

TEST(Physics, StribeckRestartUsesTheZeroSpeedDryLevel)
{
  auto p = frictionless(); p.inertia = .01; p.initial_velocity = .01;
  p.friction_model = "stribeck"; p.coulomb_friction = .1; p.static_friction = .2;
  p.stribeck_velocity = .05;
  const double first_dry = .1 + .1 * std::exp(-std::pow(.01 / .05, 2));
  const double dt = .001, stop = p.inertia * p.initial_velocity / (.5 + first_dry);
  JointPhysics model(p);
  model.step(dt, 0, -.5, false);
  // The dry level is held within each integration segment, then reevaluated at zero.
  EXPECT_NEAR(model.state().velocity, -(.5 - .2) / p.inertia * (dt - stop), 1e-14);
  EXPECT_NEAR(model.state().position, .5 * p.initial_velocity * stop -
    .5 * (.5 - .2) / p.inertia * (dt - stop) * (dt - stop), 1e-16);
}

TEST(Physics, ViscousTravelIsAnalyticalAndContinuousAtZeroViscosity)
{
  for (double damping : {0., 1e-23, .2}) {
    auto p = frictionless(); p.inertia = .01; p.initial_velocity = .1;
    p.viscous_friction = damping;
    JointPhysics model(p);
    model.step(.001, 0, 0, false);
    const double exponent = damping * .001 / p.inertia;
    EXPECT_NEAR(model.state().velocity, p.initial_velocity * std::exp(-exponent), 1e-15);
    const double expected_travel = damping == 0 ? .0001 :
      p.initial_velocity * .001 * (-std::expm1(-exponent) / exponent);
    EXPECT_NEAR(model.state().position, expected_travel, 1e-16);
  }
}

TEST(Physics, SubnormalViscousLoadDoesNotInstantlyStopAFreeRotor)
{
  auto p = frictionless(); p.inertia = .01; p.initial_velocity = 1;
  p.friction_model = "viscous"; p.viscous_friction = 1e-310;
  JointPhysics model(p);
  model.step(.001, 0, -1e-310, false);
  // Its analytical zero crossing is about 6.93e307 seconds away, not t=0.
  EXPECT_DOUBLE_EQ(model.state().velocity, 1);
  EXPECT_DOUBLE_EQ(model.state().position, .001);
}

TEST(Physics, LargeViscosityKeepsItsFiniteForceResponse)
{
  for (double damping : {1e304, 1e308}) {
    auto p = frictionless(); p.inertia = 1e-5;
    p.friction_model = "viscous"; p.viscous_friction = damping;
    JointPhysics model(p);
    model.step(.001, 0, damping, false);
    EXPECT_DOUBLE_EQ(model.state().velocity, 1);
    EXPECT_DOUBLE_EQ(model.state().position, .001);
    EXPECT_DOUBLE_EQ(model.state().mechanical_energy, .5 * p.inertia);
  }
}

namespace
{
PhysicsParameters lugre_parameters()
{
  PhysicsParameters p;
  p.friction_model = "lugre";
  p.inertia = .01; p.coulomb_friction = .12; p.static_friction = .2;
  p.stribeck_velocity = .04; p.friction_displacement_scale = .005;
  p.viscous_friction = .02; p.torque_time_constant = 0;
  p.lower_limit = -100; p.upper_limit = 100;
  return p;
}

std::vector<double> physics_values(const PhysicsState & s)
{
  return {s.position, s.velocity, s.motor_torque, s.external_torque, s.gravity_torque,
    s.friction_torque, s.contact_torque, s.net_torque, s.load_torque,
    s.rotor_temperature, s.mos_temperature, s.mechanical_energy,
    static_cast<double>(s.integration_steps), s.bristle_displacement, s.bristle_torque,
    s.bristle_energy, s.bristle_drive_work, s.viscous_dissipation,
    s.bristle_dissipation, s.total_storage};
}

void expect_lugre_work(const PhysicsParameters & p, const PhysicsState & s, double initial)
{
  EXPECT_LE(std::abs(s.bristle_displacement), p.friction_displacement_scale);
  EXPECT_GE(s.bristle_dissipation, 0);
  EXPECT_GE(s.viscous_dissipation, 0);
  EXPECT_NEAR(s.total_storage - initial - s.bristle_drive_work +
    s.bristle_dissipation + s.viscous_dissipation, 0, 2e-11);
  EXPECT_DOUBLE_EQ(s.total_storage, s.mechanical_energy + s.bristle_energy);
}

PhysicsState lugre_drive(double dt, bool constant_level)
{
  auto p = lugre_parameters();
  if (constant_level) {p.coulomb_friction = p.static_friction;}
  JointPhysics model(p);
  const unsigned n = static_cast<unsigned>(std::llround(.24 / dt));
  for (unsigned i = 0; i < n; ++i) {
    const double force = i < n / 4 ? .24 : i < n / 2 ? 0 : i < 2 * n / 3 ? -.20 : 0;
    model.step(dt, 0, force, false);
  }
  expect_lugre_work(p, model.state(), 0);
  return model.state();
}
}  // namespace

TEST(Physics, LuGreSubthresholdForceMovesAndStoresElasticEnergy)
{
  const auto p = lugre_parameters();
  JointPhysics model(p);
  model.step(.0001, 0, .05, false);
  EXPECT_GT(model.state().position, 0);
  EXPECT_GT(model.state().bristle_displacement, 0);
  EXPECT_NEAR(model.state().position, .5 * .05 / p.inertia * 1e-8, 3e-12);
  for (int i = 1; i < 1000; ++i) {model.step(.0001, 0, .05, false);}
  EXPECT_GT(model.state().bristle_energy, 0);
  expect_lugre_work(p, model.state(), 0);
}

TEST(Physics, LuGreSymmetryReversalAndFreeReleaseObeyWorkIdentity)
{
  const auto p = lugre_parameters();
  JointPhysics positive(p), negative(p);
  unsigned reversals = 0;
  int last_direction = 0;
  double previous_storage = 0;
  for (unsigned i = 0; i < 6000; ++i) {
    const double force = i < 1000 ? .24 : i < 2000 ? 0 : i < 3000 ? -.24 : 0;
    positive.step(.0001, 0, force, false);
    negative.step(.0001, 0, -force, false);
    const auto & s = positive.state();
    EXPECT_NEAR(s.position, -negative.state().position, 1e-12);
    EXPECT_NEAR(s.velocity, -negative.state().velocity, 1e-12);
    EXPECT_NEAR(s.bristle_displacement, -negative.state().bristle_displacement, 1e-12);
    expect_lugre_work(p, s, 0);
    if (i >= 3000) {
      if (i > 3000) {EXPECT_LE(s.total_storage, previous_storage + 1e-13);}
      previous_storage = s.total_storage;
      const int direction = s.velocity > 0 ? 1 : s.velocity < 0 ? -1 : 0;
      if (last_direction && direction && direction != last_direction) {++reversals;}
      if (direction) {last_direction = direction;}
    }
  }
  EXPECT_GT(reversals, 0u);
  EXPECT_GT(positive.state().bristle_dissipation, 0);
}

TEST(Physics, LuGreConstantLevelHasNoStribeckVelocityDependenceAndConvergesToDahlLaw)
{
  auto p = lugre_parameters(); p.coulomb_friction = p.static_friction;
  p.stribeck_velocity = .001;
  JointPhysics first(p);
  p.stribeck_velocity = .2;
  JointPhysics second(p);
  for (unsigned i = 0; i < 1000; ++i) {
    first.step(.0001, 0, .24, false); second.step(.0001, 0, .24, false);
  }
  EXPECT_EQ(physics_values(first.state()), physics_values(second.state()));
  // Independent monotone-displacement Dahl law when Fs=Fc and z0=0.
  const auto error = [&](double dt) {
      JointPhysics model(p);
      for (unsigned i = 0; i < static_cast<unsigned>(std::llround(.1 / dt)); ++i) {
        model.step(dt, 0, .24, false);
      }
      const auto & s = model.state();
      const double exact_z = -p.friction_displacement_scale *
        std::expm1(-s.position / p.friction_displacement_scale);
      return std::abs(s.bristle_displacement - exact_z);
    };
  const double coarse = error(.0004), medium = error(.0002), fine = error(.0001);
  EXPECT_GT(coarse, medium); EXPECT_GT(medium, fine); EXPECT_LT(fine, 2e-8);
}

TEST(Physics, LuGreTimeRefinementConvergesAcrossStribeckReversals)
{
  for (bool constant : {false, true}) {
    const auto reference = lugre_drive(.0000125, constant);
    const auto distance = [&](double dt) {
        const auto s = lugre_drive(dt, constant);
        return std::abs(s.position - reference.position) +
               .01 * std::abs(s.velocity - reference.velocity) +
               std::abs(s.bristle_displacement - reference.bristle_displacement);
      };
    const double coarse = distance(.0004), medium = distance(.0002), fine = distance(.0001);
    EXPECT_GT(coarse, medium); EXPECT_GT(medium, fine); EXPECT_LT(fine, 2e-6);
  }
}

TEST(Physics, LuGrePreloadReleasesNaturallyAndConstantLoadKeepsTheEnergyBudget)
{
  auto p = lugre_parameters();
  p.initial_bristle_displacement = p.friction_displacement_scale;
  JointPhysics preload(p);
  const double initial = preload.state().total_storage;
  preload.step(.0001, 0, 0, false);
  EXPECT_LT(preload.state().velocity, 0);
  EXPECT_LT(preload.state().position, 0);
  EXPECT_LE(preload.state().total_storage, initial);
  expect_lugre_work(p, preload.state(), initial);

  p.initial_bristle_displacement = .0002; p.initial_position = .5; p.load_torque = .03;
  JointPhysics loaded(p);
  const double loaded_initial = loaded.state().total_storage;
  for (unsigned i = 0; i < 2000; ++i) {
    loaded.step(.0001, 0, .08, false);
    expect_lugre_work(p, loaded.state(), loaded_initial);
  }
  EXPECT_NEAR(loaded.state().bristle_drive_work,
    .08 * (loaded.state().position - p.initial_position), 2e-14);
  EXPECT_NEAR(loaded.state().mechanical_energy,
    .5 * p.inertia * std::pow(loaded.state().velocity, 2) +
    p.load_torque * loaded.state().position, 1e-16);
}

TEST(Physics, LuGreSubdivisionEnforcesEndpointAndUniquenessBounds)
{
  auto p = lugre_parameters(); p.friction_displacement_scale = .0005; p.initial_velocity = 2;
  JointPhysics model(p);
  auto state = model.state();
  const auto result = lugre::integrate(p, state, 0, .001);
  EXPECT_GT(result.substeps, 1u);
  EXPECT_LE(result.maximum_a, 1);
  EXPECT_LE(result.maximum_uniqueness_ratio, .5);
  EXPECT_LE(std::abs(state.bristle_displacement), p.friction_displacement_scale);
  EXPECT_LT(result.maximum_energy_residual, 1e-14);
}

TEST(Physics, LuGreRejectsInvalidParametersWithoutChangingOtherFrictionDefaults)
{
  for (auto modify : std::vector<void (*)(PhysicsParameters &)>{
      [](auto & p) {p.coulomb_friction = 0;},
      [](auto & p) {p.static_friction = .1;},
      [](auto & p) {p.friction_negative_scale = 1.1;},
      [](auto & p) {p.friction_displacement_scale = 0;},
      [](auto & p) {p.friction_displacement_scale = std::numeric_limits<double>::quiet_NaN();},
      [](auto & p) {p.initial_bristle_displacement = .006;},
      [](auto & p) {p.initial_bristle_displacement = std::numeric_limits<double>::infinity();},
      [](auto & p) {p.stribeck_velocity = 0;},
      [](auto & p) {p.gravity_torque = .1;},
      [](auto & p) {p.joint_stiffness = .1;},
      [](auto & p) {p.initial_position = p.upper_limit;}})
  {
    auto p = lugre_parameters(); modify(p);
    EXPECT_THROW(JointPhysics{p}, std::invalid_argument);
  }
  EXPECT_EQ(PhysicsParameters{}.friction_model, "coulomb_viscous");
  EXPECT_DOUBLE_EQ(PhysicsParameters{}.initial_bristle_displacement, 0);
}

TEST(Physics, LuGreInvalidInputUnderresolutionAndContactFailureAreAtomic)
{
  auto p = lugre_parameters(); JointPhysics normal(p);
  const auto initial = physics_values(normal.state());
  for (double dt : {0., -.001, .001001, std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(normal.step(dt, .1, .2, true), std::invalid_argument);
    EXPECT_EQ(physics_values(normal.state()), initial);
  }
  EXPECT_THROW(normal.step(.001, std::numeric_limits<double>::infinity(), 0, true),
    std::invalid_argument);
  EXPECT_EQ(physics_values(normal.state()), initial);

  p.friction_displacement_scale = 1e-15;
  JointPhysics underresolved(p);
  const auto before = physics_values(underresolved.state());
  EXPECT_THROW(underresolved.step(.001, .1, .05, true), std::invalid_argument);
  EXPECT_EQ(physics_values(underresolved.state()), before);

  p = lugre_parameters(); p.friction_displacement_scale = .0005;
  p.initial_velocity = 2; p.upper_limit = .001;
  JointPhysics contact(p);
  const auto before_contact = physics_values(contact.state());
  // Several midpoint substeps would be accepted before crossing the limit.
  EXPECT_THROW(contact.step(.001, .1, 0, true), std::invalid_argument);
  EXPECT_EQ(physics_values(contact.state()), before_contact);
}
