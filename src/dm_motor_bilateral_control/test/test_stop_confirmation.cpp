#include "dm_motor_bilateral_control/stop_confirmation.hpp"
#include <gtest/gtest.h>
#include <limits>

TEST(BilateralStop, DriverCounterResetRequiresNewAcquisitionAndAllExistingStopGates)
{
  using Axis = dm_motor::MotorSession;
  using State = Axis::State;
  using dm_motor_bilateral_control::motor_stopped_after;
  const double time = dm_motor::steady_seconds();
  const double stop_started = time - 0.7;
  const rclcpp::Time ros_now(100, 0, RCL_ROS_TIME);
  Axis axis;
  axis.received = time;
  axis.stop_count = 1000000;  // Previous driver process; deliberately not reset.
  State state;
  state.header.stamp = ros_now;
  state.feedback_fresh = true;
  state.feedback_age = 0.001;
  state.feedback_count = 3;  // A restarted driver has a lower local counter.
  state.feedback_receive_time = time - 0.001;
  state.phase = State::DISABLED;
  state.status = 0;
  axis.state = state;
  const auto confirmed = [&] {return motor_stopped_after(axis, ros_now, time, stop_started, 0.05);};
  ASSERT_LT(state.feedback_count, axis.stop_count);
  EXPECT_TRUE(confirmed());

  // A newly delivered state/header must not turn old acquisition into new data.
  axis.state->feedback_receive_time = stop_started;
  EXPECT_FALSE(confirmed());
  axis.state->feedback_receive_time = stop_started - 0.001;
  EXPECT_FALSE(confirmed());
  axis.state->feedback_receive_time = time - 0.051;
  EXPECT_FALSE(confirmed());
  axis.state->feedback_receive_time = time + 0.001;
  EXPECT_FALSE(confirmed());
  axis.state->feedback_receive_time = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(confirmed());

  axis.state = state;
  axis.state->feedback_fresh = false;
  EXPECT_FALSE(confirmed());
  axis.state = state;
  axis.state->feedback_age = 0.051;
  EXPECT_FALSE(confirmed());
  axis.state = state;
  axis.state->header.stamp = rclcpp::Time(99, 0, RCL_ROS_TIME);
  EXPECT_FALSE(confirmed());
  axis.state = state;
  axis.received = time - 0.051;
  EXPECT_FALSE(confirmed());

  axis.received = time;
  axis.state = state;
  axis.state->status = 1;
  EXPECT_FALSE(confirmed());
  axis.state = state;
  axis.state->phase = State::ENABLED;
  EXPECT_FALSE(confirmed());
  axis.state = state;
  axis.token = "unreleased";
  EXPECT_FALSE(confirmed());
  axis.token.clear();
  axis.pending = Axis::Request{};
  EXPECT_FALSE(confirmed());
  axis.pending.reset();
  EXPECT_TRUE(confirmed());
  axis.state.reset();
  EXPECT_FALSE(confirmed());
}
