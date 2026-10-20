#include "dm_motor_lead_follow/follow.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace dm_motor_lead_follow;

TEST(Follow, PreservesOffsetAndSupportsReverse)
{
  EXPECT_DOUBLE_EQ(desired_position(1.2, 1.2, -0.7, 1), -0.7);
  EXPECT_NEAR(desired_position(1.5, 1.2, -0.7, 1), -0.4, 1e-12);
  EXPECT_NEAR(desired_position(1.5, 1.2, -0.7, -1), -1.0, 1e-12);
  EXPECT_NEAR(desired_position(13.0, 12.4, 0.2, 1), 0.8, 1e-12);
}

TEST(Follow, LimitsAreOptInButNonFiniteFeedbackAlwaysFails)
{
  Settings settings;
  EXPECT_NO_THROW(check_motion(10, 0, 10, 10, 90, settings));
  EXPECT_DOUBLE_EQ(limited_target(2, 0, 0.01, settings), 2);
  EXPECT_THROW(check_motion(NAN, 0, 0, 0, 25, settings), std::runtime_error);
  settings.motion_limits_enabled = true;
  EXPECT_NEAR(limited_target(2, 0, 0.01, settings), 0.005, 1e-12);
  EXPECT_NEAR(limited_target(-2, 0, 0.01, settings), -0.005, 1e-12);
  EXPECT_THROW(check_motion(2.01, 0, 0, 0, 25, settings), std::runtime_error);
  EXPECT_THROW(check_motion(0, 0, 2.01, 0, 25, settings), std::runtime_error);
  EXPECT_THROW(check_motion(0, 0, 0, 3.01, 25, settings), std::runtime_error);
  EXPECT_THROW(check_motion(0, 0, 0, 0, 55, settings), std::runtime_error);
  EXPECT_NO_THROW(check_motion(2, 0, 2, 3, 54, settings));
}

TEST(Follow, ValidatesSettingsAndTime)
{
  Settings settings;
  EXPECT_NO_THROW(settings.validate());
  settings.direction = 0;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings.direction = -1;
  settings.kp = std::numeric_limits<double>::infinity();
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings.kp = 8;
  settings.kd = -0.01;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  settings.kd = 0.5;
  settings.max_error = 3;
  settings.motion_limits_enabled = true;
  EXPECT_THROW(settings.validate(), std::invalid_argument);
  EXPECT_THROW(limited_target(0, 0, -1, settings), std::runtime_error);
  EXPECT_THROW(limited_target(NAN, 0, 0.01, settings), std::runtime_error);
}
