#pragma once

#include <dm_motor_control/core.hpp>
#include <array>
#include <deque>
#include <optional>

namespace dm_motor_bilateral_control
{
struct Settings
{
  int direction{1};
  double stiffness{16}, coupling_damping{0.35}, local_damping{0.35};
  double max_torque{0.5};
  bool motion_limits_enabled{false};
  double max_feedback_torque{1}, max_travel{0.3}, warning_temperature{45}, max_temperature{55};
  double feedback_age{0.02}, pair_skew{0.005}, feedback_stop{0.08}, recovery_time{0.1};
  void validate() const;
};

struct Sample
{
  double time{0}, position{0}, velocity{0}, torque{0};
  uint64_t sequence{0};
};

struct AxisLimits
{
  dm_motor::Mapping mapping;
  dm_motor::Limits command;
  double direction{1}, offset{0}, position_min{-12.5}, position_max{12.5};
  void validate() const;
};

struct Output
{
  std::array<dm_motor::Command, 2> targets;
  std::array<double, 2> estimated_torque{}, raw_torque{};
  double gain_scale{0}, decoded_kp{0}, decoded_kd{0};
};

std::array<dm_motor::Command, 2> relative_targets(
  const std::array<Sample, 2> & pair, const std::array<double, 2> & origins, const Settings & settings);
Output symmetric_targets(
  const std::array<dm_motor::Command, 2> & desired, const std::array<Sample, 2> & latest,
  const std::array<AxisLimits, 2> & limits, const Settings & settings, double recovery_scale);
void check_motion(const Sample & sample, double origin, double temperature, const Settings & settings);

class FeedbackPairs
{
public:
  void push(size_t axis, const Sample & sample);
  void reset(double after = 0);
  bool select(double now, double age, double skew);
  const std::optional<std::array<Sample, 2>> & pair() const {return pair_;}
  double age(double now) const;
  double skew() const;
private:
  std::array<std::deque<Sample>, 2> history_;
  std::array<double, 2> used_{};
  std::optional<std::array<Sample, 2>> pair_;
};

enum class Quality : uint8_t {Normal = 0, Degraded = 1, Recovering = 2};
struct Recovery
{
  Quality quality{Quality::Normal};
  double scale{1};
  uint32_t fresh_pairs{0};
  uint64_t degradation_count{0};
  void reset();
  void update(double pair_age, bool new_pair, double dt, double period, const Settings & settings);
};
}  // namespace dm_motor_bilateral_control
