#include "dm_motor_lead_follow/follow.hpp"
#include "common/config.hpp"

#include <array>
#include <chrono>
#include <csignal>
#include <memory>
#include <optional>
#include <random>
#include <sstream>
#include <string>

#include <dm_motor_interfaces/msg/lead_follow_state.hpp>
#include <dm_motor_interfaces/msg/motor_reply.hpp>
#include <dm_motor_interfaces/msg/motor_request.hpp>
#include <dm_motor_interfaces/msg/motor_setpoint.hpp>
#include <dm_motor_interfaces/msg/motor_state.hpp>
#include <dm_motor_interfaces/srv/lead_follow_control.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace dm_motor_lead_follow
{
using Request = dm_motor_interfaces::msg::MotorRequest;
using Reply = dm_motor_interfaces::msg::MotorReply;
using Motor = dm_motor_interfaces::msg::MotorState;
using Setpoint = dm_motor_interfaces::msg::MotorSetpoint;
using State = dm_motor_interfaces::msg::LeadFollowState;
using Trigger = std_srvs::srv::Trigger;
using OperatorControl = dm_motor_interfaces::srv::LeadFollowControl;

double steady_time()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Axis
{
  std::string name, token;
  std::optional<Motor> state;
  double received{0}, origin{0}, renew_at{0};
  uint64_t sequence{0}, rejected{0}, stop_count{0}, stop_request{0};
  std::optional<Request> pending;
  double pending_since{0};
  bool configuration_checked{false};
  rclcpp::Publisher<Request>::SharedPtr control;
  rclcpp::Publisher<Setpoint>::SharedPtr setpoint;
  rclcpp::Subscription<Motor>::SharedPtr states;
  rclcpp::Subscription<Reply>::SharedPtr replies;
};

class LeadFollowNode final : public rclcpp::Node
{
public:
  LeadFollowNode() : Node("lead_follow")
  {
    const common::Config defaults(
      declare_parameter<std::string>("config_file", common::default_config_path()));
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    axes_[0].name = declare_parameter<std::string>("leader", defaults.get<std::string>("packages.dm_motor_lead_follow.nodes.lead_follow_node.leader"), fixed);
    axes_[1].name = declare_parameter<std::string>("follower", defaults.get<std::string>("packages.dm_motor_lead_follow.nodes.lead_follow_node.follower"), fixed);
    rate_ = declare_parameter<double>("rate", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.rate"), fixed);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.feedback_timeout"), fixed);
    validity_ = declare_parameter<double>("target_validity", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.target_validity"), fixed);
    operator_timeout_ = declare_parameter<double>("operator_timeout", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.operator_timeout"), fixed);
    if (!std::isfinite(operator_timeout_) || operator_timeout_ < 0.3 || operator_timeout_ > 5) {
      throw std::invalid_argument("operator_timeout must be 0.3..5 seconds");
    }
    if (!std::isfinite(rate_) || rate_ < 20 || rate_ > 500 ||
      !std::isfinite(feedback_timeout_) || feedback_timeout_ < 2 / rate_ || feedback_timeout_ > 0.5 ||
      !std::isfinite(validity_) || validity_ < 2 / rate_ || validity_ > 0.25)
    {
      throw std::invalid_argument("rate: 20..500; feedback_timeout: 2/rate..0.5; target_validity: 2/rate..0.25");
    }
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("lead_follow requires the system ROS clock, not use_sim_time");
    }
    const auto direction = declare_parameter<int64_t>("direction", defaults.get<int64_t>("packages.dm_motor_lead_follow.nodes.lead_follow_node.direction"));
    if (direction != 1 && direction != -1) {
      throw std::invalid_argument("direction must be 1 or -1");
    }
    settings_.direction = static_cast<int>(direction);
    swap_roles_ = declare_parameter<bool>("swap_roles", defaults.get<bool>("packages.dm_motor_lead_follow.nodes.lead_follow_node.swap_roles"));
    settings_.kp = declare_parameter<double>("kp", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.kp"));
    settings_.kd = declare_parameter<double>("kd", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.kd"));
    settings_.motion_limits_enabled = declare_parameter<bool>("motion_limits_enabled", defaults.get<bool>("packages.dm_motor_lead_follow.nodes.lead_follow_node.motion_limits_enabled"));
    settings_.target_speed = declare_parameter<double>("target_speed", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.target_speed"));
    settings_.max_error = declare_parameter<double>("max_error", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.max_error"));
    settings_.max_travel = declare_parameter<double>("max_travel", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.max_travel"));
    settings_.max_speed = declare_parameter<double>("max_speed", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.max_speed"));
    settings_.max_torque = declare_parameter<double>("max_torque", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.max_torque"));
    settings_.max_temperature = declare_parameter<double>("max_temperature", defaults.get<double>("packages.dm_motor_lead_follow.nodes.lead_follow_node.max_temperature"));
    settings_.validate();
    std::ostringstream identity;
    std::random_device random;
    identity << "lead_follow_" << std::hex << random() << random();
    client_ = identity.str();
    for (auto & axis : axes_) {
      if (axis.name.empty() || axis.name.back() == '/') {
        throw std::invalid_argument("Motor namespaces must be nonempty without trailing slash");
      }
      axis.name = get_node_topics_interface()->resolve_topic_name(axis.name);
      axis.control = create_publisher<Request>(axis.name + "/control", rclcpp::QoS(64).reliable());
      axis.setpoint = create_publisher<Setpoint>(
        axis.name + "/setpoint", rclcpp::QoS(1).best_effort());
      axis.states = create_subscription<Motor>(
        axis.name + "/state", rclcpp::QoS(1).reliable(),
        [&axis](Motor::ConstSharedPtr state) {axis.state = *state; axis.received = steady_time();});
      axis.replies = create_subscription<Reply>(
        axis.name + "/control_result", rclcpp::QoS(256).reliable(),
        [this, &axis](const Reply & reply) {receive_reply(axis, reply);});
    }
    if (axes_[0].name == axes_[1].name) {
      throw std::invalid_argument("leader and follower must resolve to different motor namespaces");
    }
    parameters_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & values) {
        rcl_interfaces::msg::SetParametersResult result;
        try {
          if (phase_ != State::IDLE && phase_ != State::FAULT) {
            throw std::runtime_error("Stop and confirm disabled feedback before changing settings");
          }
          for (const auto & axis : axes_) {
            if (!disabled(axis)) {
              throw std::runtime_error("Settings require fresh disabled motors");
            }
          }
          auto candidate = settings_;
          bool swap_roles = swap_roles_;
          for (const auto & value : values) {
            const auto & name = value.get_name();
            if (name == "swap_roles") {swap_roles = value.as_bool();}
            else if (name == "direction") {
              const auto direction = value.as_int();
              if (direction != 1 && direction != -1) {throw std::invalid_argument("direction must be 1 or -1");}
              candidate.direction = static_cast<int>(direction);
            } else if (name == "motion_limits_enabled") {candidate.motion_limits_enabled = value.as_bool();}
            else if (name == "kp") {candidate.kp = value.as_double();}
            else if (name == "kd") {candidate.kd = value.as_double();}
            else if (name == "target_speed") {candidate.target_speed = value.as_double();}
            else if (name == "max_error") {candidate.max_error = value.as_double();}
            else if (name == "max_travel") {candidate.max_travel = value.as_double();}
            else if (name == "max_speed") {candidate.max_speed = value.as_double();}
            else if (name == "max_torque") {candidate.max_torque = value.as_double();}
            else if (name == "max_temperature") {candidate.max_temperature = value.as_double();}
            else {throw std::invalid_argument("Parameter is not runtime configurable: " + name);}
          }
          candidate.validate();
          settings_ = candidate;
          if (swap_roles_ != swap_roles) {
            swap_roles_ = swap_roles;
            for (auto & axis : axes_) {axis.origin = axis.state->position;}
            ideal_ = target_ = follower().origin;
          }
          result.successful = true;
        } catch (const std::exception & error) {
          result.reason = error.what();
          RCLCPP_WARN(get_logger(), "Configuration rejected: %s", error.what());
        }
        return result;
      });
    publisher_ = create_publisher<State>("~/state", rclcpp::QoS(1).reliable());
    start_service_ = create_service<Trigger>("~/start",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        try {
          start();
          response->success = true;
          response->message = "Start accepted, not yet enabled; wait for state.phase=FOLLOWING.";
        } catch (const std::exception & error) {
          response->message = error.what();
          RCLCPP_WARN(get_logger(), "Start rejected: %s", error.what());
        }
      });
    stop_service_ = create_service<Trigger>("~/stop",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        begin_stop("Explicit stop requested", false);
        response->success = true;
        response->message = "Stop requested; wait for state.stop_confirmed=true.";
      });
    operator_service_ = create_service<OperatorControl>("~/operator_control",
      [this](OperatorControl::Request::SharedPtr request, OperatorControl::Response::SharedPtr response) {
        try {
          if (request->operation == OperatorControl::Request::STOP) {
            begin_stop("Operator STOP requested", false);
          } else if (request->operation == OperatorControl::Request::START) {
            if (request->session.size() < 16 || request->session.size() > 128) {
              throw std::invalid_argument("Operator session must be 16..128 characters");
            }
            start();
            operator_session_ = request->session;
            operator_heartbeat_ = steady_time();
          } else if (request->operation == OperatorControl::Request::HEARTBEAT) {
            if (operator_session_.empty() || request->session != operator_session_) {
              throw std::runtime_error("No matching active operator session");
            }
            operator_heartbeat_ = steady_time();
          } else {throw std::invalid_argument("Unknown operator command");}
          response->success = true;
          response->message = "Accepted; check follow state for transition confirmation.";
        } catch (const std::exception & error) {
          response->message = error.what();
          RCLCPP_WARN(get_logger(), "Operator command rejected: %s", error.what());
        }
      });
    last_tick_ = steady_time();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1 / rate_), [this] {tick();});
    state_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {publish_state();});
    RCLCPP_INFO(get_logger(), "Leader %s -> follower %s; explicit start required",
      axes_[0].name.c_str(), axes_[1].name.c_str());
  }

  void shutdown_stop()
  {
    if (phase_ != State::IDLE || !axes_[0].token.empty() || !axes_[1].token.empty()) {
      begin_stop("Controller shutting down", false);
    }
  }

  bool stopping() const {return phase_ == State::STOPPING;}

private:
  Axis & leader() {return axes_[swap_roles_ ? 1 : 0];}
  Axis & follower() {return axes_[swap_roles_ ? 0 : 1];}

  bool fresh(const Axis & axis) const
  {
    if (!axis.state) {return false;}
    const auto & state = *axis.state;
    const double elapsed = steady_time() - axis.received;
    const double age = (now() - rclcpp::Time(state.header.stamp)).seconds();
    return state.feedback_fresh && std::isfinite(state.feedback_age) && state.feedback_age >= 0 &&
           elapsed >= 0 && state.feedback_age + elapsed <= feedback_timeout_ &&
           age >= -0.05 && age + state.feedback_age <= feedback_timeout_;
  }

  bool disabled(const Axis & axis) const
  {
    return fresh(axis) && axis.state->status == 0 &&
           (axis.state->phase == Motor::DISABLED || axis.state->phase == Motor::FAULT);
  }

  void check(const Axis & axis) const
  {
    if (!fresh(axis)) {throw std::runtime_error(axis.name + ": feedback/state stale or missing");}
    const auto & state = *axis.state;
    if (!state.ready || !state.absolute_position_valid || state.host_fault ||
      state.device_fault || state.phase == Motor::FAULT || !state.error.empty() ||
      state.mode != 1 || state.task_active)
    {
      throw std::runtime_error(axis.name + ": requires ready, fault-free MIT mode without an active task");
    }
    check_motion(state.position, axis.origin, state.velocity, state.torque,
      std::max(state.mos_temperature, state.rotor_temperature), settings_);
    if (!std::isfinite(state.position_min) || !std::isfinite(state.position_max) ||
      state.position_min >= state.position_max || state.position < state.position_min ||
      state.position > state.position_max)
    {
      throw std::runtime_error(axis.name + ": feedback outside joint bounds");
    }
  }

  void start()
  {
    if (phase_ != State::IDLE && phase_ != State::FAULT) {
      throw std::runtime_error("Controller busy; stop before starting again");
    }
    for (auto & axis : axes_) {
      if (!disabled(axis) || axis.pending || !axis.token.empty())
      {
        throw std::runtime_error(axis.name + ": start requires fresh disabled motors");
      }
      axis.origin = axis.state->position;
      check(axis);
      if (std::abs(axis.state->velocity) > 0.05) {
        throw std::runtime_error("Hold both motors stationary before starting");
      }
      if (axis.control->get_subscription_count() == 0 || axis.setpoint->get_subscription_count() == 0) {
        throw std::runtime_error(axis.name + ": motor command topics unavailable");
      }
      axis.rejected = axis.state->rejected_commands;
      axis.configuration_checked = false;
    }
    const auto & follower_state = *follower().state;
    if (!std::isfinite(follower_state.kp_limit) || !std::isfinite(follower_state.kd_limit) ||
      settings_.kp > follower_state.kp_limit || settings_.kd > follower_state.kd_limit)
    {
      throw std::runtime_error("Follower gains exceed motor limits");
    }
    ideal_ = target_ = follower().origin;
    fault_stop_ = false;
    phase_ = State::ACQUIRING; phase_since_ = steady_time();
    message_ = "Opening non-exclusive motor keepalive sessions";
    try {
      for (auto & axis : axes_) {send(axis, Request::LEASE, Request::ACQUIRE);}
    } catch (const std::exception & error) {
      begin_stop(error.what(), true);
      throw;
    }
  }

  void send(Axis & axis, uint8_t kind, uint8_t operation)
  {
    Request request;
    request.header.stamp = now(); request.client_id = client_; request.request_id = ++request_id_;
    request.valid_for = 0.5; request.kind = kind; request.operation = operation;
    request.token = axis.token; request.owner = client_; request.duration = 2.0;
    axis.pending = request; axis.pending_since = steady_time();
    axis.control->publish(request);
  }

  void send_stop(Axis & axis)
  {
    Request request;
    request.header.stamp = now(); request.client_id = client_; request.request_id = ++request_id_;
    request.kind = Request::OPERATION; request.operation = Request::DISABLE;
    axis.stop_request = request.request_id;
    axis.control->publish(request);
  }

  void receive_reply(Axis & axis, const Reply & reply)
  {
    if (reply.client_id != client_) {return;}
    if (reply.request_id == axis.stop_request && !reply.success) {
      RCLCPP_ERROR(get_logger(), "STOP rejected for %s: %s", axis.name.c_str(), reply.message.c_str());
    }
    if (!axis.pending || reply.request_id != axis.pending->request_id ||
      reply.kind != axis.pending->kind) {return;}
    const auto request = *axis.pending;
    axis.pending.reset();
    if (!reply.success) {
      if (phase_ == State::STOPPING) {
        RCLCPP_ERROR(get_logger(), "Stopping %s: %s", axis.name.c_str(), reply.message.c_str());
        if (request.kind == Request::LEASE && request.operation == Request::RELEASE) {
          axis.token.clear();
        }
      } else {begin_stop(axis.name + ": " + reply.message, true);}
      return;
    }
    if (request.kind == Request::LEASE) {
      if (request.operation == Request::RELEASE) {axis.token.clear();}
      else {
        axis.token = reply.token;
        axis.renew_at = steady_time() + 0.5;
        if (axis.token.empty()) {begin_stop(axis.name + ": empty lease token", true);}
      }
    } else if (request.kind == Request::CAPABILITIES && phase_ != State::STOPPING) {
      const auto & configuration = reply.configuration;
      if (!reply.ready || !std::isfinite(configuration.command_timeout) ||
        configuration.command_timeout < validity_ ||
        !std::isfinite(configuration.watchdog_timeout) || configuration.watchdog_timeout <= 0 ||
        !std::isfinite(configuration.state_rate) || configuration.state_rate < 2 / feedback_timeout_)
      {
        begin_stop(axis.name + ": incompatible command timeout, watchdog or state rate", true);
      } else {axis.configuration_checked = true;}
    }
  }

  void begin_stop(const std::string & reason, bool fault)
  {
    operator_session_.clear();
    if (phase_ == State::STOPPING) {return;}
    fault_stop_ = fault;
    phase_ = State::STOPPING; phase_since_ = steady_time(); last_stop_ = 0;
    message_ = reason; stop_timeout_logged_ = false;
    for (auto & axis : axes_) {
      axis.stop_count = axis.state ? axis.state->feedback_count : 0;
      send_stop(axis);
    }
    if (fault) {RCLCPP_ERROR(get_logger(), "%s; stopping both motors", reason.c_str());}
    else {RCLCPP_INFO(get_logger(), "%s", reason.c_str());}
  }

  void stopping_tick(double time)
  {
    for (auto & axis : axes_) {
      if (time - last_stop_ >= 0.2) {send_stop(axis);}
      if (!axis.pending && !axis.token.empty()) {send(axis, Request::LEASE, Request::RELEASE);}
    }
    if (time - last_stop_ >= 0.2) {last_stop_ = time;}
    bool confirmed = true;
    for (const auto & axis : axes_) {
      confirmed = confirmed && disabled(axis) && axis.state->feedback_count > axis.stop_count &&
        axis.token.empty() && !axis.pending;
    }
    // An ACQUIRE/ENABLE already in flight remains valid for 0.5 seconds.
    if (confirmed && time - phase_since_ > 0.6) {
      phase_ = fault_stop_ ? State::FAULT : State::IDLE;
      message_ += "; both motors confirmed disabled, keepalive sessions released";
      RCLCPP_INFO(get_logger(), "%s", message_.c_str());
    } else if (!stop_timeout_logged_ && time - phase_since_ > 3.0) {
      stop_timeout_logged_ = true;
      message_ += "; STOP UNCONFIRMED: inspect devices/physical stop; retrying";
      RCLCPP_ERROR(get_logger(), "%s", message_.c_str());
    }
  }

  void publish_target(Axis & axis, double position, double kp, double kd)
  {
    Setpoint message;
    message.header.stamp = now(); message.token = axis.token; message.sequence = ++axis.sequence;
    message.client_id = client_;
    message.valid_for = validity_; message.hold_until_lease_expiry = false;
    message.target.mode = 1; message.target.position = position;
    message.target.kp = kp; message.target.kd = kd;
    axis.setpoint->publish(message);
  }

  void tick()
  {
    const double time = steady_time(), dt = time - last_tick_;
    last_tick_ = time;
    try {
      for (auto & axis : axes_) {
        if (axis.pending && time - axis.pending_since > 0.8) {
          axis.pending.reset();
          if (phase_ != State::STOPPING) {throw std::runtime_error(axis.name + ": motor request timed out");}
          RCLCPP_ERROR(get_logger(), "%s: request timed out during STOP", axis.name.c_str());
          axis.token.clear();
        }
      }
      if (phase_ == State::STOPPING) {stopping_tick(time); return;}
      if (phase_ == State::IDLE || phase_ == State::FAULT) {return;}
      if (!operator_session_.empty() && time - operator_heartbeat_ > operator_timeout_) {
        throw std::runtime_error("Operator heartbeat lost");
      }
      if (dt > std::min(validity_, feedback_timeout_)) {
        throw std::runtime_error("Follow control timer stalled");
      }
      for (auto & axis : axes_) {
        check(axis);
        if (phase_ != State::FOLLOWING && std::abs(axis.state->position - axis.origin) > 0.02) {
          throw std::runtime_error("Motor moved while arming; stop and retry stationary");
        }
        if (!axis.token.empty() && !axis.pending && time >= axis.renew_at) {
          send(axis, Request::LEASE, Request::RENEW);
        }
      }
      if (phase_ != State::FOLLOWING && time - phase_since_ > 3.0) {
        throw std::runtime_error("Arming timed out");
      }
      if (phase_ == State::ACQUIRING) {
        for (auto & axis : axes_) {
          if (!axis.token.empty() && !axis.pending && !axis.configuration_checked) {
            send(axis, Request::CAPABILITIES, 0);
          }
        }
        if (std::all_of(axes_.begin(), axes_.end(), [](const Axis & axis) {
            return !axis.token.empty() && !axis.pending && axis.configuration_checked;
          }))
        {
          phase_ = State::ENABLING; phase_since_ = time; message_ = "Enabling neutral MIT targets";
          for (auto & axis : axes_) {send(axis, Request::OPERATION, Request::ENABLE);}
        }
        return;
      }
      if (phase_ == State::ENABLING) {
        for (auto & axis : axes_) {
          if (axis.state->phase == Motor::ENABLED) {publish_target(axis, axis.origin, 0, 0);}
        }
        if (std::all_of(axes_.begin(), axes_.end(), [](const Axis & axis) {
            return !axis.pending && axis.state->phase == Motor::ENABLED &&
                   axis.state->status == 1;
          }))
        {
          phase_ = State::FOLLOWING; message_ = "Following";
          RCLCPP_INFO(get_logger(), "Following with captured relative origins");
        }
        return;
      }
      for (const auto & axis : axes_) {
        if (axis.state->phase != Motor::ENABLED || axis.state->status != 1)
        {
          throw std::runtime_error(axis.name + ": lost enable confirmation");
        }
      }
      ideal_ = desired_position(leader().state->position, leader().origin, follower().origin, settings_.direction);
      const auto & follower_state = *follower().state;
      if (!std::isfinite(ideal_) || ideal_ < follower_state.position_min || ideal_ > follower_state.position_max) {
        throw std::runtime_error("Ideal follower target outside joint bounds");
      }
      if (settings_.motion_limits_enabled && std::abs(ideal_ - follower_state.position) > settings_.max_error) {
        throw std::runtime_error("Following error exceeds limit");
      }
      target_ = limited_target(ideal_, target_, dt, settings_);
      publish_target(leader(), leader().state->position, 0, 0);
      publish_target(follower(), target_, settings_.kp, settings_.kd);
      ++command_count_;
    } catch (const std::exception & error) {
      if (phase_ == State::STOPPING) {
        RCLCPP_ERROR(get_logger(), "STOP retry failed: %s", error.what());
      } else {begin_stop(error.what(), true);}
    }
  }

  void publish_state()
  {
    State state;
    state.header.stamp = now(); state.phase = phase_; state.message = message_;
    state.leader = leader().name; state.follower = follower().name;
    state.direction = settings_.direction; state.motion_limits_enabled = settings_.motion_limits_enabled;
    state.operator_guarded = !operator_session_.empty();
    state.stop_confirmed = (phase_ == State::IDLE || phase_ == State::FAULT) &&
      disabled(axes_[0]) && disabled(axes_[1]);
    state.leader_origin = leader().origin; state.follower_origin = follower().origin;
    state.ideal_position = ideal_; state.target_position = target_;
    state.following_error = follower().state ? ideal_ - follower().state->position : 0;
    state.command_count = command_count_;
    publisher_->publish(state);
  }

  std::array<Axis, 2> axes_;
  Settings settings_;
  std::string client_, message_{"Idle; explicit start required"};
  uint8_t phase_{State::IDLE};
  uint64_t request_id_{0}, command_count_{0};
  double rate_, feedback_timeout_, validity_, last_tick_, phase_since_{0}, last_stop_{0};
  double ideal_{0}, target_{0};
  bool fault_stop_{false}, stop_timeout_logged_{false};
  bool swap_roles_{false};
  std::string operator_session_;
  double operator_timeout_, operator_heartbeat_{0};
  rclcpp::Publisher<State>::SharedPtr publisher_;
  rclcpp::Service<Trigger>::SharedPtr start_service_, stop_service_;
  rclcpp::Service<OperatorControl>::SharedPtr operator_service_;
  rclcpp::TimerBase::SharedPtr timer_, state_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameters_;
};
}  // namespace dm_motor_lead_follow

namespace
{
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) {stop_requested = 1;}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  int result = 0;
  try {
    auto node = std::make_shared<dm_motor_lead_follow::LeadFollowNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested) {executor.spin_once(std::chrono::milliseconds(10));}
    if (rclcpp::ok()) {
      node->shutdown_stop();
      const double deadline = dm_motor_lead_follow::steady_time() + 3.0;
      while (rclcpp::ok() && node->stopping() && dm_motor_lead_follow::steady_time() < deadline) {
        executor.spin_once(std::chrono::milliseconds(10));
      }
      if (node->stopping()) {
        RCLCPP_ERROR(node->get_logger(), "Shutdown STOP unconfirmed; inspect devices/physical stop");
        result = 1;
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lead_follow"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
