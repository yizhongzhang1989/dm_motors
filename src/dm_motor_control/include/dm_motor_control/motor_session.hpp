#pragma once

#include "dm_motor_control/runtime.hpp"
#include <dm_motor_interfaces/msg/motor_reply.hpp>
#include <dm_motor_interfaces/msg/motor_request.hpp>
#include <dm_motor_interfaces/msg/motor_setpoint.hpp>
#include <dm_motor_interfaces/msg/motor_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace dm_motor
{
struct MotorSession
{
  using Request = dm_motor_interfaces::msg::MotorRequest;
  using Reply = dm_motor_interfaces::msg::MotorReply;
  using State = dm_motor_interfaces::msg::MotorState;
  using Setpoint = dm_motor_interfaces::msg::MotorSetpoint;
  std::string name, token;
  std::optional<State> state;
  double received{0}, origin{0}, renew_at{0};
  uint64_t sequence{0}, rejected{0}, stop_count{0}, stop_request{0};
  std::optional<Request> pending;
  double pending_since{0};
  bool configuration_checked{false};
  rclcpp::Publisher<Request>::SharedPtr control;
  rclcpp::Publisher<Setpoint>::SharedPtr setpoint;
  rclcpp::Subscription<State>::SharedPtr states;
  rclcpp::Subscription<Reply>::SharedPtr replies;
};

inline void attach_motor_session(
  rclcpp::Node & node, MotorSession & axis,
  std::function<void(const MotorSession::Reply &)> on_reply)
{
  if (axis.name.empty() || axis.name.back() == '/') {
    throw std::invalid_argument("Motor namespaces must be nonempty without trailing slash");
  }
  axis.name = node.get_node_topics_interface()->resolve_topic_name(axis.name);
  axis.control = node.create_publisher<MotorSession::Request>(axis.name + "/control", rclcpp::QoS(64).reliable());
  axis.setpoint = node.create_publisher<MotorSession::Setpoint>(axis.name + "/setpoint", rclcpp::QoS(1).best_effort());
  axis.states = node.create_subscription<MotorSession::State>(
    axis.name + "/state", rclcpp::QoS(1).reliable(),
    [&axis](MotorSession::State::ConstSharedPtr state) {axis.state = *state; axis.received = steady_seconds();});
  axis.replies = node.create_subscription<MotorSession::Reply>(
    axis.name + "/control_result", rclcpp::QoS(256).reliable(), std::move(on_reply));
}

inline bool motor_state_fresh(const MotorSession & axis, const rclcpp::Time & now, double timeout)
{
  if (!axis.state) {return false;}
  const auto & state = *axis.state;
  const double elapsed = steady_seconds() - axis.received;
  const double age = (now - rclcpp::Time(state.header.stamp)).seconds();
  return state.feedback_fresh && std::isfinite(state.feedback_age) && state.feedback_age >= 0 &&
         elapsed >= 0 && state.feedback_age + elapsed <= timeout &&
         age >= -0.05 && age + state.feedback_age <= timeout;
}

inline void send_motor_request(
  MotorSession & axis, uint8_t kind, uint8_t operation, const std::string & client,
  uint64_t & sequence, const rclcpp::Time & stamp)
{
  MotorSession::Request request;
  request.header.stamp = stamp; request.client_id = client; request.request_id = ++sequence;
  request.valid_for = 0.5; request.kind = kind; request.operation = operation;
  request.token = axis.token; request.owner = client; request.duration = 2.0;
  axis.pending = request; axis.pending_since = steady_seconds();
  axis.control->publish(request);
}

inline void send_motor_stop(
  MotorSession & axis, const std::string & client, uint64_t & sequence, const rclcpp::Time & stamp)
{
  MotorSession::Request request;
  request.header.stamp = stamp; request.client_id = client; request.request_id = ++sequence;
  request.kind = MotorSession::Request::OPERATION; request.operation = MotorSession::Request::DISABLE;
  axis.stop_request = request.request_id;
  axis.control->publish(request);
}

inline void send_motor_target(
  MotorSession & axis, const dm_motor_interfaces::msg::MotorTarget & target,
  const std::string & client, double validity, const rclcpp::Time & stamp)
{
  MotorSession::Setpoint message;
  message.header.stamp = stamp; message.token = axis.token; message.sequence = ++axis.sequence;
  message.client_id = client; message.valid_for = validity; message.hold_until_lease_expiry = false;
  message.target = target;
  axis.setpoint->publish(message);
}
}  // namespace dm_motor
