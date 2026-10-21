#include "dm_motor_control/simulation/device.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <unistd.h>

using namespace dm_motor::simulation;

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
