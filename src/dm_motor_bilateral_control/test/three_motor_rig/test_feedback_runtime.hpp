#pragma once

#include "feedback_runtime.hpp"
#include <dm_motor_bilateral_control/bilateral.hpp>
#include <algorithm>
#include <iostream>

namespace three_motor_rig
{
inline int check_feedback_batches()
{
  using namespace dm_motor_bilateral_control;
  using dm_motor::simulation::DeviceParameters;
  using dm_motor::simulation::VirtualMotor;
  for (const double gap : {.008121353, .010}) {
    std::array<DeviceParameters, 3> pars;
    for (size_t i = 0; i < 3; ++i) {pars[i].motor.id = i + 1; pars[i].motor.mapping.v = 50;}
    std::array<VirtualMotor, 3> motors{{VirtualMotor(pars[0]), VirtualMotor(pars[1]), VirtualMotor(pars[2])}};
    FeedbackRuntime runtime(motors);
    FeedbackPairs pairs;
    struct Reply {double nominal, received; size_t axis;};
    std::vector<Reply> replies;
    for (unsigned tick = 1; tick <= 31; ++tick) {
      const size_t axis = tick % 2 ? 1 : 0;
      const double nominal = tick * .001;
      double received = nominal;
      if (axis == 1 && tick == 21) {received = .031 - gap;}
      else if (axis == 1 && tick > 21) {received = .031;}
      replies.push_back({nominal, received, axis});
    }
    std::stable_sort(replies.begin(), replies.end(), [](const Reply & a, const Reply & b) {
      return a.received < b.received;
    });
    std::array<uint64_t, 2> count{}, previous_bus_count{};
    double previous_b_position = 0;
    for (const auto & reply : replies) {
      // The sensor's encoded payload is sampled before transport delay. Never
      // replace it by the position at the later collection timestamp.
      const auto frame = dm_motor::feedback_frame(reply.axis + 1, 0,
        {1, reply.nominal - .001, 1, 0, 25, 25}, pars[reply.axis].motor.mapping);
      const auto state = runtime.receive(reply.axis, frame, reply.received);
      if (previous_bus_count[reply.axis] && state.feedback_count != previous_bus_count[reply.axis] + 1) {
        throw std::runtime_error("Batched production Bus lost a physical reply");
      }
      previous_bus_count[reply.axis] = state.feedback_count;
      pairs.push(reply.axis, {reply.received, state.absolute_position, state.feedback.v, 0, ++count[reply.axis]});
      if (reply.axis == 1 && reply.nominal == .021) {previous_b_position = state.absolute_position;}
    }
    const auto common = pairs.common_time_positions(.031, .020, .008);
    if (!common || std::abs(common->time - (.031 - gap)) > 1e-12 ||
      common->position[1] != previous_b_position || count[0] != 15 || count[1] != 16 ||
      common->bracket_gap[0] > .008 || common->bracket_gap[1] != 0 ||
      pairs.common_time_positions(.060, .020, .008))
    {throw std::runtime_error("Production RX-batch endpoint/age/no-fallback regression failed");}
  }
  std::cout << "Production Bus receive-batch endpoint checks=2; original payloads/counts retained\n";
  return 0;
}

inline int check_feedback_wraps()
{
  using namespace dm_motor_bilateral_control;
  using dm_motor::simulation::DeviceParameters;
  using dm_motor::simulation::VirtualMotor;
  std::array<DeviceParameters, 3> pars;
  for (size_t i = 0; i < 3; ++i) {pars[i].motor.id = i + 1; pars[i].motor.mapping.v = 50;}
  std::array<VirtualMotor, 3> motors{{VirtualMotor(pars[0]), VirtualMotor(pars[1]), VirtualMotor(pars[2])}};
  FeedbackRuntime runtime(motors);
  FeedbackPairs pairs;
  std::array<bool, 2> positive_wrap{}, negative_wrap{};
  std::array<Sample, 2> latest{};
  double maximum_error = 0, maximum_common_spring = 0, maximum_latest_spring = 0;
  auto position = [](double t) {
      if (t <= 1.4) {return 10 * t;}
      if (t <= 4.2) {return 14 - 10 * (t - 1.4);}
      return -14 + 10 * (t - 4.2);
    };
  unsigned common_checks = 0;
  for (unsigned tick = 1; tick <= 5600; ++tick) {
    const double time = tick * .001, q = position(time);
    const size_t axis = tick % 2 ? 1 : 0;
    const auto & mapping = pars[axis].motor.mapping;
    const double period = 2 * mapping.p * 65536 / 65535;
    double native = std::fmod(q + mapping.p, period);
    if (native < 0) {native += period;}
    const auto frame = dm_motor::feedback_frame(axis + 1, 0,
      {1, std::clamp(native - mapping.p, -mapping.p, mapping.p), 0, 0, 25, 25}, mapping);
    const auto state = runtime.receive(axis, frame, time);
    maximum_error = std::max(maximum_error, std::abs(state.absolute_position - q));
    positive_wrap[axis] = positive_wrap[axis] || state.position_wraps > 0;
    negative_wrap[axis] = negative_wrap[axis] || state.position_wraps < 0;
    latest[axis] = {time, state.absolute_position, state.feedback.v, 0, tick};
    pairs.push(axis, latest[axis]);
    if (tick >= 4) {
      const auto common = pairs.common_time_positions(time, .020, .005);
      if (!common) {throw std::runtime_error("Continuous position wrap regression lost its bracket");}
      if (std::abs(common->time - 1.4) > .003 && std::abs(common->time - 4.2) > .003) {
        for (const auto value : common->position) {
          if (std::abs(value - position(common->time)) > .0008) {
            throw std::runtime_error("Common-time interpolation crossed a raw encoder discontinuity");
          }
        }
        maximum_common_spring = std::max(maximum_common_spring,
          50 * std::abs(common->position[0] - common->position[1]));
        maximum_latest_spring = std::max(maximum_latest_spring,
          50 * std::abs(latest[0].position - latest[1].position));
        ++common_checks;
      }
    }
  }
  if (!positive_wrap[0] || !positive_wrap[1] || !negative_wrap[0] || !negative_wrap[1] ||
    maximum_error > .0008 || maximum_common_spring > .03 || maximum_latest_spring < .45)
  {throw std::runtime_error("Production continuous-position bidirectional wrap regression failed");}
  std::cout << "Production Bus bidirectional wrap checks=" << common_checks
            << ", max_position_error=" << maximum_error
            << ", common_spring_peak=" << maximum_common_spring
            << ", latest_spring_peak=" << maximum_latest_spring << '\n';
  return 0;
}
}  // namespace three_motor_rig
