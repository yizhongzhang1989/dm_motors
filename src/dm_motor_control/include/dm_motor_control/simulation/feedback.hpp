#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <stdexcept>
#include <string>

namespace dm_motor::simulation
{
struct FeedbackParameters
{
  double velocity_time_constant{0};
  std::string feedback_velocity_source{"internal"};
  double feedback_velocity_time_constant{0}, feedback_velocity_delay{0};
  double feedback_torque_time_constant{0}, feedback_torque_delay{0};

  void validate() const
  {
    if (!std::isfinite(velocity_time_constant) || velocity_time_constant < 0 || velocity_time_constant > 0.1) {
      throw std::invalid_argument("Velocity estimator time constant must be finite and in [0,0.1] seconds");
    }
    if (feedback_velocity_source != "internal" && feedback_velocity_source != "physical") {
      throw std::invalid_argument("Feedback velocity source must be internal or physical");
    }
    for (double value : {feedback_velocity_time_constant, feedback_velocity_delay,
        feedback_torque_time_constant, feedback_torque_delay})
    {
      if (!std::isfinite(value) || value < 0 || value > 0.1) {
        throw std::invalid_argument("Feedback time constants and delays must be finite and in [0,0.1] seconds");
      }
    }
  }
};

// Device measurement dynamics, independent of command quantization and transport.
// Advance exactly once AFTER each physics substep. Querying never advances state;
// report timestamps remain acquisition times, not backdated filter input times.
class FeedbackDynamics
{
public:
  FeedbackDynamics(FeedbackParameters parameters, double initial_velocity, double time = 0)
  : parameters_(parameters), estimated_velocity_(initial_velocity), reported_velocity_(initial_velocity)
  {
    parameters_.validate();
    history_.push_back({time, reported_velocity_, reported_torque_});
  }

  double internal_velocity() const {return estimated_velocity_;}

  void advance(double dt, double sample_time, double physical_velocity, double motor_torque)
  {
    const auto filtered = [dt](double previous, double input, double tau) {
        if (tau == 0) {return input;}
        const double response = -std::expm1(-dt / tau);
        return (1 - response) * previous + response * input;
      };
    estimated_velocity_ = filtered(estimated_velocity_, physical_velocity, parameters_.velocity_time_constant);
    const double velocity = parameters_.feedback_velocity_source == "physical" ? physical_velocity : estimated_velocity_;
    reported_velocity_ = filtered(reported_velocity_, velocity, parameters_.feedback_velocity_time_constant);
    reported_torque_ = filtered(reported_torque_, motor_torque, parameters_.feedback_torque_time_constant);
    const double maximum_delay = std::max(parameters_.feedback_velocity_delay, parameters_.feedback_torque_delay);
    if (maximum_delay == 0) {return;}
    // Keep the sample immediately before the oldest query, for fractional-step interpolation.
    while (history_.size() > 1 && history_[1].time <= sample_time - maximum_delay) {history_.pop_front();}
    if (history_.size() >= 20002) {
      throw std::runtime_error("Feedback delay history overflow; simulation sampling is too dense");
    }
    history_.push_back({sample_time, reported_velocity_, reported_torque_});
  }

  double velocity(double time) const {return delayed(time, parameters_.feedback_velocity_delay, true);}
  double torque(double time) const {return delayed(time, parameters_.feedback_torque_delay, false);}

private:
  struct Sample {double time, velocity, torque;};
  double delayed(double time, double delay, bool velocity) const
  {
    if (delay == 0) {return velocity ? reported_velocity_ : reported_torque_;}
    const auto value = [velocity](const Sample & sample) {return velocity ? sample.velocity : sample.torque;};
    const double query = time - delay;
    const auto after = std::upper_bound(history_.begin(), history_.end(), query,
      [](double stamp, const Sample & sample) {return stamp < sample.time;});
    if (after == history_.begin()) {return value(history_.front());}
    if (after == history_.end()) {return value(history_.back());}
    const auto before = std::prev(after);
    const double fraction = (query - before->time) / (after->time - before->time);
    return value(*before) + fraction * (value(*after) - value(*before));
  }
  FeedbackParameters parameters_;
  double estimated_velocity_, reported_velocity_, reported_torque_{0};
  std::deque<Sample> history_;
};
}  // namespace dm_motor::simulation
