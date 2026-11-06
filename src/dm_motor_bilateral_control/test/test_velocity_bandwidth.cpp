#include "dm_motor_bilateral_control/velocity_bandwidth.hpp"
#include <gtest/gtest.h>
#include <limits>

using dm_motor_bilateral_control::VelocityBandwidthCheck;

namespace
{
uint32_t raw(float value)
{
  uint32_t result;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}
}

TEST(VelocityBandwidth, RequiresOneOperationAndIndependentConfirmedRead)
{
  VelocityBandwidthCheck check;
  check.reset(100);
  EXPECT_THROW(check.queued({}), std::runtime_error);
  EXPECT_THROW(check.queued({0}), std::runtime_error);
  EXPECT_THROW(check.queued({12, 13}), std::runtime_error);
  check.queued({12});
  EXPECT_FALSE(check.verified());
  EXPECT_EQ(check.operation(), 12u);
  EXPECT_THROW(check.queued({13}), std::runtime_error);
  EXPECT_FALSE(check.observe(12, 0, 0, 0));
  EXPECT_FALSE(check.verified());
  EXPECT_TRUE(check.observe(12, 1, 0x20, raw(100)));
  EXPECT_TRUE(check.verified());
  EXPECT_EQ(check.value(), 100);
}

TEST(VelocityBandwidth, WrongIdentityStateAddressAndValuesNeverVerify)
{
  for (const auto state : {uint8_t(2), uint8_t(3), uint8_t(255)}) {
    VelocityBandwidthCheck check; check.reset(100); check.queued({12});
    EXPECT_THROW(check.observe(12, state, 0x20, raw(100)), std::runtime_error);
    EXPECT_FALSE(check.verified());
  }
  VelocityBandwidthCheck check; check.reset(100); check.queued({12});
  EXPECT_THROW(check.observe(13, 1, 0x20, raw(100)), std::runtime_error);
  EXPECT_THROW(check.observe(12, 1, 0x21, raw(100)), std::runtime_error);
  for (const auto value : {40.f, 0.f, 200.f, std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity(), std::nextafter(100.f, 101.f)})
  {
    EXPECT_THROW(check.observe(12, 1, 0x20, raw(value)), std::runtime_error);
    EXPECT_FALSE(check.verified());
  }
}

TEST(VelocityBandwidth, StopInvalidatesLateReplyAndRestartRequiresNewOperation)
{
  VelocityBandwidthCheck check; check.reset(100); check.queued({12});
  check.cancel();
  EXPECT_THROW(check.observe(12, 1, 0x20, raw(100)), std::runtime_error);
  EXPECT_THROW(check.queued({13}), std::runtime_error);
  EXPECT_FALSE(check.verified());
  check.reset(100);
  EXPECT_FALSE(check.verified());
  EXPECT_EQ(check.value(), 0);
  EXPECT_THROW(check.observe(12, 1, 0x20, raw(100)), std::runtime_error);
  check.queued({14});
  EXPECT_THROW(check.observe(12, 1, 0x20, raw(100)), std::runtime_error);
  EXPECT_TRUE(check.observe(14, 1, 0x20, raw(100)));
  check.reset(100);
  EXPECT_FALSE(check.verified());
}

TEST(VelocityBandwidth, DisabledRequirementAndFloatRegisterRange)
{
  VelocityBandwidthCheck check; check.reset(0);
  EXPECT_FALSE(check.verified());
  EXPECT_THROW(check.queued({12}), std::runtime_error);
  for (const auto value : {-1., 500., 499.999999999, 1e-300,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    EXPECT_THROW(check.reset(value), std::invalid_argument);
  }
  check.reset(100.123);
  check.queued({15});
  EXPECT_TRUE(check.observe(15, 1, 0x20, raw(100.123f)));
}
