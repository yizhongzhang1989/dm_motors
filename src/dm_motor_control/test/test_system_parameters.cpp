#include "dm_motor_control/system_parameters.hpp"
#include <gtest/gtest.h>
#include <fstream>
#include <limits>
#include <unistd.h>

namespace
{
YAML::Node fit()
{
  return YAML::Load(R"(
kind: mechanical
model: coulomb_viscous
parameters: {inertia: 0.02, bias: 0.0, coulomb: 0.08, viscous: 0.03}
units: {inertia: kg_m2, bias: Nm, coulomb: Nm, viscous: Nm_s_per_rad}
diagnostics: {samples: 100, rmse: 0.001, validation_rmse: 0.002, condition: 2,
              rmse_limit: 0.02, condition_limit: 100}
domain: {velocity_min: -2.0, velocity_max: 2.0}
)");
}
}

TEST(SystemParameters, BoundedFrictionUsesMeasuredVelocityAndResetsOnReversal)
{
  dm_motor::FrictionCompensator compensation(fit());
  EXPECT_DOUBLE_EQ(compensation.step(0, 0.01, 1), 0);
  EXPECT_DOUBLE_EQ(compensation.step(0.01, 0.01, 1), 0);
  EXPECT_NEAR(compensation.step(1, 0.01, 1), 0.05, 1e-12);
  for (int i = 0; i < 100; ++i) {compensation.step(1, 0.01, 1);}
  EXPECT_NEAR(compensation.step(1, 0.01, 1), 0.8 * (0.08 + 0.03), 1e-12);
  EXPECT_DOUBLE_EQ(compensation.step(-1, 0.01, 1), 0);
  EXPECT_NEAR(compensation.step(-1, 0.01, 1), -0.05, 1e-12);
  compensation.reset();
  for (int i = 0; i < 100; ++i) {compensation.step(100, 0.01, 1);}
  EXPECT_NEAR(compensation.step(100, 0.01, 1), 0.8 * (0.08 + 0.03 * 2), 1e-12);
  EXPECT_DOUBLE_EQ(compensation.step(100, 0.01, 0.01), 0.01);
  EXPECT_THROW(compensation.step(NAN, 0.01, 1), std::runtime_error);
  EXPECT_THROW(compensation.step(1, 0.11, 1), std::runtime_error);
}

TEST(SystemParameters, OptionalFractionPreservesDefaultsAndExposesMeasuredDomain)
{
  dm_motor::FrictionCompensator original(fit()), explicit_default(fit()), half(fit());
  EXPECT_EQ(original.model(), "coulomb_viscous");
  EXPECT_DOUBLE_EQ(original.velocity_min(), -2.0);
  EXPECT_DOUBLE_EQ(original.velocity_max(), 2.0);
  for (const double velocity : {0.0, 0.1, 1.0, 20.0, -1.0, -20.0, 0.0}) {
    EXPECT_DOUBLE_EQ(original.step(velocity, 0.01, 0.2), explicit_default.step(velocity, 0.01, 0.2, 0.8));
  }
  for (int i = 0; i < 100; ++i) {half.step(1, 0.01, 1, 0.5);}
  EXPECT_NEAR(half.step(1, 0.01, 1, 0.5), 0.5 * (0.08 + 0.03), 1e-12);
  for (int i = 0; i < 100; ++i) {half.step(20, 0.01, 1, 1);}
  EXPECT_NEAR(half.step(20, 0.01, 1, 1), 0.08 + 0.03 * 2, 1e-12);
  EXPECT_DOUBLE_EQ(half.step(20, 0.01, 0.01, 1), 0.01);
}

TEST(SystemParameters, ZeroFractionClearsHistoryAndInvalidFractionsAreRejected)
{
  dm_motor::FrictionCompensator compensation(fit());
  for (int i = 0; i < 100; ++i) {compensation.step(1, 0.01, 1);}
  EXPECT_GT(compensation.step(1, 0.01, 1), 0);
  EXPECT_DOUBLE_EQ(compensation.step(1, 0.01, 1, 0), 0);
  // Both filtered velocity and prior output must be gone when assistance resumes.
  dm_motor::FrictionCompensator fresh(fit());
  EXPECT_DOUBLE_EQ(compensation.step(-1, 0.01, 1, 1), fresh.step(-1, 0.01, 1, 1));
  EXPECT_DOUBLE_EQ(compensation.step(-1, 0.01, 1, 0), 0);
  for (const double fraction : {-0.01, 1.01, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(compensation.step(1, 0.01, 1, fraction), std::runtime_error);
  }
  EXPECT_THROW(compensation.step(NAN, 0.01, 1, 0), std::runtime_error);
}

TEST(SystemParameters, ExtraViscosityKeepsCalibrationDomainAndHasItsOwnSpeedCeiling)
{
  dm_motor::FrictionCompensator c(fit());
  // The original model stays clipped at 2 rad/s. Only the explicit additional
  // controller term can use the separately specified 13.2 rad/s ceiling.
  for (int i = 0; i < 300; ++i) {c.step(10, .002, .4, .6, .02, 13.2);}
  EXPECT_NEAR(c.step(10, .002, .4, .6, .02, 13.2), .6 * (.08 + .03 * 2) + .02 * 10, 1e-12);
  for (int i = 0; i < 300; ++i) {c.step(100, .002, .4, .6, .02, 13.2);}
  EXPECT_NEAR(c.step(100, .002, .4, .6, .02, 13.2), .6 * (.08 + .03 * 2) + .02 * 13.2, 1e-12);
  EXPECT_DOUBLE_EQ(c.velocity_min(), -2.);
  EXPECT_DOUBLE_EQ(c.velocity_max(), 2.);
  EXPECT_DOUBLE_EQ(c.step(-10, .002, .4, .6, .02, 13.2), 0);
  for (int i = 0; i < 300; ++i) {c.step(-10, .002, .4, .6, .02, 13.2);}
  EXPECT_NEAR(c.step(-10, .002, .4, .6, .02, 13.2), -(.6 * (.08 + .03 * 2) + .02 * 10), 1e-12);
}

TEST(SystemParameters, ExtraViscositySharesCapSlewDeadZoneAndCompleteReset)
{
  dm_motor::FrictionCompensator c(fit());
  double previous = 0;
  for (int i = 0; i < 300; ++i) {
    const double current = c.step(13.2, .002, .2, .6, .03, 13.2);
    EXPECT_LE(std::abs(current - previous), .010000000001);
    EXPECT_LE(std::abs(current), .2);
    previous = current;
  }
  EXPECT_DOUBLE_EQ(previous, .2);
  EXPECT_DOUBLE_EQ(c.step(.02, .002, .2, .6, .03, 13.2), 0);
  dm_motor::FrictionCompensator fresh(fit());
  EXPECT_DOUBLE_EQ(c.step(-13.2, .002, .2, .6, .03, 13.2),
    fresh.step(-13.2, .002, .2, .6, .03, 13.2));
  EXPECT_DOUBLE_EQ(c.step(-13.2, .002, .2, 0, .03, 13.2), 0);
  fresh.reset();
  EXPECT_DOUBLE_EQ(c.step(13.2, .002, .2, .6, .03, 13.2),
    fresh.step(13.2, .002, .2, .6, .03, 13.2));
}

TEST(SystemParameters, ZeroExtraViscosityIsExactlyTheOriginalLaw)
{
  dm_motor::FrictionCompensator original(fit()), zero(fit());
  for (int i = 0; i < 2000; ++i) {
    const double v = i % 17 == 0 ? 0 : 20 * std::sin(i * .037);
    const double dt = .001 + .001 * (i % 3);
    EXPECT_DOUBLE_EQ(original.step(v, dt, .2, .6), zero.step(v, dt, .2, .6, 0, 0));
  }
  for (const double value : {-1., std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(zero.step(1, .002, .2, .6, value, 13.2), std::runtime_error);
    EXPECT_THROW(zero.step(1, .002, .2, .6, .02, value), std::runtime_error);
  }
  EXPECT_THROW(zero.step(1, .002, .2, .6, .02, 0), std::runtime_error);
}

TEST(SystemParameters, FractionStillCompensatesOnlyFriction)
{
  auto full_fit = fit();
  full_fit["parameters"]["inertia"] = 2.0;
  full_fit["parameters"]["bias"] = 0.3;
  full_fit["parameters"]["gravity_sin"] = 0.4;
  full_fit["parameters"]["gravity_cos"] = 0.5;
  full_fit["parameters"]["stiffness"] = 1.0;
  for (const auto & key : {"gravity_sin", "gravity_cos", "stiffness"}) {full_fit["units"][key] = "SI";}
  dm_motor::FrictionCompensator original(fit()), complete(full_fit);
  for (const double velocity : {0.0, 0.1, 1.0, 2.0, -0.1, -1.0, -2.0, 0.0}) {
    EXPECT_DOUBLE_EQ(original.step(velocity, 0.01, 0.2, 0.5), complete.step(velocity, 0.01, 0.2, 0.5));
  }
}

TEST(SystemParameters, SupportsAllAnalyticalModelsAndRejectsBadCoefficients)
{
  for (const auto & model : {"viscous", "coulomb_viscous", "stribeck",
      "asymmetric_coulomb_viscous", "asymmetric_stribeck"})
  {
    auto f = fit();
    f["model"] = model;
    const std::string name(model);
    auto p = f["parameters"];
    if (name.find("asymmetric") == 0) {
      for (const auto & suffix : {"_positive", "_negative"}) {
        p[std::string("coulomb") + suffix] = 0.08;
        p[std::string("viscous") + suffix] = 0.03;
        p[std::string("static") + suffix] = 0.12;
        p[std::string("stribeck_velocity") + suffix] = 0.2;
      }
    } else {
      p["static"] = 0.12;
      p["stribeck_velocity"] = 0.2;
    }
    for (const auto & item : p) {f["units"][item.first.as<std::string>()] = "SI";}
    dm_motor::FrictionCompensator compensation(f);
    for (int i = 0; i < 100; ++i) {compensation.step(1, 0.01, 1);}
    EXPECT_NEAR(compensation.step(1, 0.01, 1), 0.8 * (name == "viscous" ? 0.03 : 0.11), 1e-10);
  }
  auto invalid = fit();
  invalid["parameters"]["coulomb"] = -0.1;
  EXPECT_THROW(dm_motor::FrictionCompensator rejected(invalid), std::runtime_error);
  invalid = fit();
  invalid["diagnostics"]["validation_rmse"] = 1;
  EXPECT_THROW(dm_motor::FrictionCompensator rejected(invalid), std::runtime_error);
}

TEST(SystemParameters, TableInterpolationAndLowSpeedGuard)
{
  auto f = fit();
  f["model"] = "tabulated";
  f["friction_table"] = YAML::Load("[[0.05, 0.1, 0.05, 0.2], [1, 0.2, 1, 0.3], [6.283185307179586, 0.3, 6.283185307179586, 0.4]]");
  f["parameters"]["static_positive"] = 0.3;
  f["parameters"]["static_negative"] = 0.4;
  f["units"]["static_positive"] = "Nm";
  f["units"]["static_negative"] = "Nm";
  f["domain"]["velocity_min"] = -2 * std::acos(-1.0);
  f["domain"]["velocity_max"] = 2 * std::acos(-1.0);
  auto d = f["diagnostics"];
  for (const auto & metric : {"max_bin_error", "repeatability_max", "ci95_max",
      "static_repeatability_max", "static_ci95_max"}) {d[metric] = 0.001;}
  for (const auto & bound : {"max_bin_error_limit", "repeatability_limit", "ci95_limit"}) {d[bound] = 0.01;}
  d["coverage_complete"] = d["static_complete"] = 1;
  d["required_max_speed"] = d["validated_max_speed"] = 2 * std::acos(-1.0);
  dm_motor::FrictionCompensator compensation(f);
  EXPECT_DOUBLE_EQ(compensation.step(0.049, 0.01, 1), 0);
  for (int i = 0; i < 100; ++i) {compensation.step(0.525, 0.01, 1);}
  EXPECT_NEAR(compensation.step(0.525, 0.01, 1), 0.8 * 0.15, 1e-12);
  compensation.reset();
  for (int i = 0; i < 100; ++i) {compensation.step(-0.525, 0.01, 1);}
  EXPECT_NEAR(compensation.step(-0.525, 0.01, 1), -0.8 * 0.25, 1e-12);
}

TEST(SystemParameters, RegistryChecksIdentityAndIsolatesSimulation)
{
  const auto path = "/tmp/dm_system_parameters_" + std::to_string(getpid()) + ".yaml";
  YAML::Node root;
  root["version"] = 1;
  root["motors"]["/motor_1"]["description"] = "test";
  auto calibration = root["motors"]["/motor_1"]["hardware"]["calibrations"]["mechanical"];
  calibration["schema"] = 2; calibration["qualified"] = true; calibration["fit"] = fit();
  calibration["identity"] = YAML::Load(
    "{namespace: /motor_1, motor_id: 1, simulated: false, mode: 1, direction: 1.0, "
    "software_offset: 0.0, pmax: 12.5, vmax: 50.0, tmax: 10.0}");
  {std::ofstream stream(path); stream << root;}
  dm_motor::SystemParameters registry(path);
  std::filesystem::remove(path);
  dm_motor_interfaces::msg::MotorState state;
  state.motor_id = 1; state.mode = 1; state.direction = 1;
  state.pmax = 12.5; state.vmax = 50; state.tmax = 10;
  EXPECT_TRUE(registry.compensator("/motor_1", state).has_value());
  EXPECT_FALSE(registry.compensator("/unknown", state).has_value());
  state.simulated = true;
  EXPECT_FALSE(registry.compensator("/motor_1", state).has_value());
  state.simulated = false; state.direction = -1;
  EXPECT_THROW(registry.compensator("/motor_1", state), std::runtime_error);
  state.direction = 1;
  calibration["qualified"] = false;
  calibration["fit"]["diagnostics"]["validation_rmse"] = 0.1;
  {std::ofstream stream(path); stream << root;}
  EXPECT_THROW(dm_motor::SystemParameters(path).compensator("/motor_1", state), std::runtime_error);
  auto approval = calibration["approval"];
  approval["user_accepted"] = true;
  approval["method"] = "manual_after_drag";
  for (const auto & key : {"calibration_run", "drag_run", "calibration_sha256", "drag_report_sha256", "saved_at"}) {
    approval[key] = "test-evidence";
  }
  {std::ofstream stream(path); stream << root;}
  auto accepted = dm_motor::SystemParameters(path).compensator("/motor_1", state);
  ASSERT_TRUE(accepted);
  EXPECT_DOUBLE_EQ(accepted->step(0, 0.01, 1), 0);
  EXPECT_GT(accepted->step(1, 0.01, 1), 0);
  EXPECT_LE(accepted->step(1, 0.01, 0.01), 0.01);
  calibration["schema"] = 3;
  calibration["identity"].remove("namespace");
  calibration["identity"].remove("simulated");
  calibration["fit"].remove("kind");
  calibration["fit"].remove("diagnostics");
  calibration["approval"] = YAML::Load("{user_accepted: true, drag_report: /missing/report.json, saved_at: test}");
  {std::ofstream stream(path); stream << root;}
  auto compact = dm_motor::SystemParameters(path).compensator("/motor_1", state);
  ASSERT_TRUE(compact);
  accepted->reset();
  for (const double speed : {0.0, 0.01, 0.2, 1.0, 30.0, -0.5, -30.0, 0.0}) {
    EXPECT_DOUBLE_EQ(accepted->step(speed, 0.01, 0.5), compact->step(speed, 0.01, 0.5));
  }
  state.software_offset = 0.2;
  EXPECT_THROW(dm_motor::SystemParameters(path).compensator("/motor_1", state), std::runtime_error);
  state.software_offset = 0;
  state.simulated = true;
  EXPECT_FALSE(dm_motor::SystemParameters(path).compensator("/motor_1", state));
  state.simulated = false;
  calibration["approval"]["user_accepted"] = false;
  {std::ofstream stream(path); stream << root;}
  EXPECT_THROW(dm_motor::SystemParameters(path).compensator("/motor_1", state), std::runtime_error);
  calibration["approval"]["user_accepted"] = true;
  calibration["fit"]["parameters"]["coulomb"] = -0.1;
  {std::ofstream stream(path); stream << root;}
  EXPECT_THROW(dm_motor::SystemParameters(path).compensator("/motor_1", state), std::runtime_error);
  std::filesystem::remove(path);
}
