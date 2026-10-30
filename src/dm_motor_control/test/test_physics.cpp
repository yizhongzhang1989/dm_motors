#include "dm_motor_control/simulation/physics.hpp"
#include <gtest/gtest.h>
#include <cmath>

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
        const double expected = (0.1 + 0.1 * std::exp(-0.25) +
          0.03 * std::abs(model.state().velocity)) * (velocity < 0 ? 1.5 : 1);
        EXPECT_NEAR(std::abs(model.state().friction_torque), expected, 1e-10);
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
  model.step(0.001, 0, -0.15, false);
  EXPECT_DOUBLE_EQ(model.state().velocity, 0);
}
