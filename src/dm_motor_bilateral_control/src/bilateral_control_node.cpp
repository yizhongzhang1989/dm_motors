#include "dm_motor_bilateral_control/bilateral.hpp"
#include "dm_motor_control/motor_session.hpp"
#include "common/config.hpp"

#include <dm_motor_interfaces/msg/bilateral_state.hpp>
#include <dm_motor_interfaces/msg/motor_feedback.hpp>
#include <dm_motor_interfaces/srv/bilateral_control.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <csignal>
#include <map>
#include <random>
#include <sstream>

namespace dm_motor_bilateral_control
{
using Request = dm_motor_interfaces::msg::MotorRequest;
using Reply = dm_motor_interfaces::msg::MotorReply;
using Motor = dm_motor_interfaces::msg::MotorState;
using Feedback = dm_motor_interfaces::msg::MotorFeedback;
using Target = dm_motor_interfaces::msg::MotorTarget;
using State = dm_motor_interfaces::msg::BilateralState;
using Control = dm_motor_interfaces::srv::BilateralControl;
using Trigger = std_srvs::srv::Trigger;
using Axis = dm_motor::MotorSession;

std::map<std::string, double *> settings_fields(Settings & settings)
{
  return {{"stiffness", &settings.stiffness}, {"coupling_damping", &settings.coupling_damping},
    {"local_damping", &settings.local_damping}, {"max_torque", &settings.max_torque},
    {"max_feedback_torque", &settings.max_feedback_torque}, {"max_travel", &settings.max_travel},
    {"warning_temperature", &settings.warning_temperature}, {"max_temperature", &settings.max_temperature},
    {"feedback_age", &settings.feedback_age}, {"pair_skew", &settings.pair_skew},
    {"feedback_stop", &settings.feedback_stop}, {"recovery_time", &settings.recovery_time}};
}

class BilateralNode final : public rclcpp::Node
{
public:
  BilateralNode() : Node("bilateral_control")
  {
    const common::Config config(declare_parameter<std::string>("config_file", common::default_config_path()));
    const std::string prefix = "packages.dm_motor_bilateral_control.nodes.bilateral_control_node.";
    rcl_interfaces::msg::ParameterDescriptor fixed; fixed.read_only = true;
    axes_[0].name = declare_parameter<std::string>("motor_a", config.get<std::string>(prefix + "motor_a"), fixed);
    axes_[1].name = declare_parameter<std::string>("motor_b", config.get<std::string>(prefix + "motor_b"), fixed);
    rate_ = declare_parameter<double>("rate", config.get<double>(prefix + "rate"), fixed);
    validity_ = declare_parameter<double>("target_validity", config.get<double>(prefix + "target_validity"), fixed);
    operator_timeout_ = declare_parameter<double>("operator_timeout", config.get<double>(prefix + "operator_timeout"), fixed);
    if (get_parameter("use_sim_time").as_bool()) {throw std::invalid_argument("Bilateral control requires system ROS time");}
    if (!std::isfinite(rate_) || rate_ < 20 || rate_ > 500 ||
      !std::isfinite(validity_) || validity_ < 2 / rate_ || validity_ > 0.25 ||
      !std::isfinite(operator_timeout_) || operator_timeout_ < 0.3 || operator_timeout_ > 5)
    {
      throw std::invalid_argument("Invalid bilateral rate, target validity or operator timeout");
    }
    const auto direction = declare_parameter<int64_t>("direction", config.get<int64_t>(prefix + "direction"));
    if (direction != 1 && direction != -1) {throw std::invalid_argument("direction must be 1 or -1");}
    settings_.direction = static_cast<int>(direction);
    settings_.motion_limits_enabled = declare_parameter<bool>(
      "motion_limits_enabled", config.get<bool>(prefix + "motion_limits_enabled"));
    for (const auto & item : settings_fields(settings_)) {
      *item.second = declare_parameter<double>(item.first, config.get<double>(prefix + item.first));
    }
    validate_settings(settings_);
    std::ostringstream client;
    std::random_device random;
    client << "bilateral_" << std::hex << random() << random();
    client_ = client.str();
    for (size_t i = 0; i < 2; ++i) {
      dm_motor::attach_motor_session(*this, axes_[i], [this, i](const Reply & reply) {receive_reply(i, reply);});
      feedback_[i] = create_subscription<Feedback>(axes_[i].name + "/feedback", rclcpp::QoS(256).reliable(),
        [this, i](const Feedback & message) {receive_feedback(i, message);});
    }
    if (axes_[0].name == axes_[1].name) {throw std::invalid_argument("Bilateral motor namespaces must differ");}
    parameters_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & values) {
        rcl_interfaces::msg::SetParametersResult result;
        try {
          if ((phase_ != State::IDLE && phase_ != State::FAULT) || !disabled(axes_[0]) || !disabled(axes_[1])) {
            throw std::runtime_error("Settings require fresh disabled motors and a stopped controller");
          }
          auto candidate = settings_;
          auto fields = settings_fields(candidate);
          for (const auto & value : values) {
            const auto & name = value.get_name();
            if (name == "direction") {
              const auto direction = value.as_int();
              if (direction != 1 && direction != -1) {throw std::invalid_argument("direction must be 1 or -1");}
              candidate.direction = static_cast<int>(direction);
            } else if (name == "motion_limits_enabled") {candidate.motion_limits_enabled = value.as_bool();}
            else if (fields.count(name)) {*fields.at(name) = value.as_double();}
            else {throw std::invalid_argument("Parameter is not runtime configurable: " + name);}
          }
          validate_settings(candidate);
          settings_ = candidate;
          result.successful = true;
        } catch (const std::exception & error) {
          result.reason = error.what();
          RCLCPP_WARN(get_logger(), "Bilateral settings rejected: %s", error.what());
        }
        return result;
      });
    publisher_ = create_publisher<State>("~/state", rclcpp::QoS(1).reliable());
    start_ = create_service<Trigger>("~/start", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        try {start(); response->success = true; response->message = "Start accepted; wait for ACTIVE feedback";}
        catch (const std::exception & error) {response->message = error.what(); RCLCPP_WARN(get_logger(), "%s", error.what());}
      });
    stop_ = create_service<Trigger>("~/stop", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        begin_stop("Explicit bilateral STOP", false);
        response->success = true; response->message = "STOP requested; wait for stop_confirmed";
      });
    operator_ = create_service<Control>("~/operator_control",
      [this](Control::Request::SharedPtr request, Control::Response::SharedPtr response) {
        try {
          if (request->operation != Control::Request::STOP) {
            const double age = (now() - rclcpp::Time(request->header.stamp)).seconds();
            if (!std::isfinite(request->valid_for) || request->valid_for <= 0 ||
              request->valid_for > 0.5 || age < -0.05 || age > request->valid_for)
            {
              throw std::runtime_error("Bilateral operator request expired or has invalid timing");
            }
          }
          if (request->operation == Control::Request::STOP) {begin_stop("Operator bilateral STOP", false);}
          else if (request->operation == Control::Request::START) {
            if (request->session.size() < 16 || request->session.size() > 128) {
              throw std::invalid_argument("Operator session must be 16..128 characters");
            }
            start(); operator_session_ = request->session; heartbeat_ = dm_motor::steady_seconds();
          } else if (request->operation == Control::Request::HEARTBEAT) {
            if (operator_session_.empty() || request->session != operator_session_) {
              throw std::runtime_error("No matching active bilateral operator session");
            }
            heartbeat_ = dm_motor::steady_seconds();
          } else {throw std::invalid_argument("Unknown bilateral operator command");}
          response->success = true; response->message = "Accepted; check bilateral state";
        } catch (const std::exception & error) {
          response->message = error.what();
          RCLCPP_WARN(get_logger(), "Bilateral operator request rejected: %s", error.what());
        }
      });
    last_tick_ = dm_motor::steady_seconds();
    timer_ = create_wall_timer(std::chrono::duration<double>(1 / rate_), [this] {tick();});
    state_timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {publish_state();});
    RCLCPP_INFO(get_logger(), "Standard bilateral coupling %s <-> %s; explicit start required",
      axes_[0].name.c_str(), axes_[1].name.c_str());
  }

  bool stopping() const {return phase_ == State::STOPPING;}
  void shutdown_stop()
  {
    if (phase_ != State::IDLE || !axes_[0].token.empty() || !axes_[1].token.empty()) {
      begin_stop("Bilateral controller shutting down", false);
    }
  }

private:
  bool active() const {return phase_ == State::ACQUIRING || phase_ == State::ENABLING || phase_ == State::ACTIVE;}
  void validate_settings(const Settings & value) const
  {
    value.validate();
    if (value.feedback_age < 2 / rate_ || value.feedback_stop < validity_) {
      throw std::invalid_argument("feedback_age must cover two controller periods; feedback_stop must cover target validity");
    }
  }

  bool disabled(const Axis & axis) const
  {
    return dm_motor::motor_state_fresh(axis, now(), settings_.feedback_stop) && axis.state->status == 0 &&
           (axis.state->phase == Motor::DISABLED || axis.state->phase == Motor::FAULT);
  }

  AxisLimits limits(const Motor & state) const
  {
    return {{state.pmax, state.vmax, state.tmax},
      {state.position_limit, state.velocity_limit, state.torque_limit, state.kp_limit, state.kd_limit},
      state.direction, state.software_offset, state.position_min, state.position_max};
  }

  void check_axis(size_t i) const
  {
    const auto & axis = axes_[i];
    if (!dm_motor::motor_state_fresh(axis, now(), settings_.feedback_stop) || !latest_[i] ||
      dm_motor::steady_seconds() - latest_[i]->time > settings_.feedback_stop)
    {
      throw std::runtime_error(axis.name + ": feedback/state missing or expired");
    }
    const auto & state = *axis.state;
    if (!state.ready || !state.absolute_position_valid || state.host_fault || state.device_fault ||
      state.phase == Motor::FAULT || !state.error.empty() || state.mode != 1 || state.task_active)
    {
      throw std::runtime_error(axis.name + ": " + (state.error.empty() ?
        "requires ready fault-free MIT feedback without an active task" : state.error));
    }
    check_motion(*latest_[i], axis.origin, std::max(state.mos_temperature, state.rotor_temperature), settings_);
    if (state.position < state.position_min || state.position > state.position_max) {
      throw std::runtime_error(axis.name + ": position outside motor joint bounds");
    }
    if (phase_ == State::ENABLING || phase_ == State::ACTIVE) {
      const auto & previous = limits_[i];
      if (state.direction != previous.direction || state.software_offset != previous.offset ||
        state.pmax != previous.mapping.p || state.vmax != previous.mapping.v || state.tmax != previous.mapping.t ||
        state.position_min != previous.position_min || state.position_max != previous.position_max)
      {
        throw std::runtime_error(axis.name + ": motor coordinates or ranges changed; explicit restart required");
      }
    }
  }

  void receive_feedback(size_t i, const Feedback & message)
  {
    try {
      const double time = dm_motor::steady_seconds();
      if (!message.absolute_position_valid || message.receive_time > time + 1e-6 ||
        time - message.receive_time > settings_.feedback_stop ||
        (axes_[i].state && message.motor_id != axes_[i].state->motor_id))
      {
        throw std::runtime_error("Invalid/stale bilateral feedback identity, coordinate anchor or monotonic timestamp");
      }
      Sample sample{message.receive_time, message.position, message.velocity, message.torque, message.feedback_count};
      if (!active() && latest_[i] && (sample.sequence <= latest_[i]->sequence || sample.time <= latest_[i]->time)) {
        pairs_.reset(); latest_ = {};
      }
      pairs_.push(i, sample);
      latest_[i] = sample;
    } catch (const std::exception & error) {
      if (active()) {begin_stop(axes_[i].name + ": " + error.what(), true);}
      else {RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());}
    }
  }

  void start()
  {
    if (phase_ != State::IDLE && phase_ != State::FAULT) {throw std::runtime_error("Bilateral controller busy");}
    validate_settings(settings_);
    const double time = dm_motor::steady_seconds();
    for (size_t i = 0; i < 2; ++i) {
      auto & axis = axes_[i];
      if (!disabled(axis) || axis.pending || !axis.token.empty()) {
        throw std::runtime_error(axis.name + ": start requires fresh disabled feedback");
      }
      axis.origin = axis.state->position;
      check_axis(i);
      if (std::abs(axis.state->velocity) > 0.05 || std::abs(latest_[i]->velocity) > 0.05) {
        throw std::runtime_error("Hold both disabled motors stationary before starting");
      }
      if (!axis.control->get_subscription_count() || !axis.setpoint->get_subscription_count()) {
        throw std::runtime_error(axis.name + ": motor command topics unavailable");
      }
      limits_[i] = limits(*axis.state);
      limits_[i].validate();
      axis.configuration_checked = false;
    }
    if (axes_[0].state->simulated != axes_[1].state->simulated) {
      throw std::runtime_error("Do not couple simulated and physical motors");
    }
    if (!pairs_.select(time, settings_.feedback_age, settings_.pair_skew)) {
      if (!pairs_.pair() || pairs_.age(time) > settings_.feedback_age || pairs_.skew() > settings_.pair_skew) {
        throw std::runtime_error("No fresh disabled feedback pair; no motor was enabled");
      }
    }
    for (size_t i = 0; i < 2; ++i) {axes_[i].origin = (*pairs_.pair())[i].position;}
    const auto pair = *pairs_.pair();
    const auto desired = relative_targets(pair, origins(), settings_);
    symmetric_targets(desired, pair, limits_, settings_, 1);
    output_ = {}; recovery_.reset(); pair_count_ = 0; pair_waiting_ = false;
    phase_ = State::ACQUIRING; phase_since_ = time; fault_stop_ = false; enabled_pair_after_ = 0;
    message_ = "Acquiring non-exclusive motor sessions and verifying driver configuration";
    try {
      for (auto & axis : axes_) {request(axis, Request::LEASE, Request::ACQUIRE);}
    } catch (const std::exception & error) {begin_stop(error.what(), true); throw;}
  }

  std::array<double, 2> origins() const {return {axes_[0].origin, axes_[1].origin};}
  std::array<Sample, 2> latest() const {return {*latest_[0], *latest_[1]};}
  void request(Axis & axis, uint8_t kind, uint8_t operation)
  {
    dm_motor::send_motor_request(axis, kind, operation, client_, request_id_, now());
  }

  void receive_reply(size_t i, const Reply & reply)
  {
    auto & axis = axes_[i];
    if (reply.client_id != client_) {return;}
    if (reply.request_id == axis.stop_request && !reply.success) {
      RCLCPP_ERROR(get_logger(), "%s: STOP rejected: %s", axis.name.c_str(), reply.message.c_str());
    }
    if (reply.kind == Request::TARGET && !reply.success && active()) {
      begin_stop(axis.name + ": target rejected: " + reply.message, true);
    }
    if (!axis.pending || reply.request_id != axis.pending->request_id || reply.kind != axis.pending->kind) {return;}
    const auto pending = *axis.pending;
    axis.pending.reset();
    if (!reply.success) {
      if (phase_ == State::STOPPING) {
        RCLCPP_ERROR(get_logger(), "%s: %s", axis.name.c_str(), reply.message.c_str());
        if (pending.kind == Request::LEASE && pending.operation == Request::RELEASE) {axis.token.clear();}
      } else {begin_stop(axis.name + ": " + reply.message, true);}
      return;
    }
    if (pending.kind == Request::LEASE) {
      if (pending.operation == Request::RELEASE) {axis.token.clear();}
      else {
        axis.token = reply.token; axis.renew_at = dm_motor::steady_seconds() + 0.5;
        if (axis.token.empty()) {begin_stop(axis.name + ": empty motor session", true);}
      }
    } else if (pending.kind == Request::CAPABILITIES && phase_ != State::STOPPING) {
      const auto & value = reply.configuration;
      if (!reply.ready || !std::isfinite(value.command_timeout) || value.command_timeout < validity_ ||
        !std::isfinite(value.watchdog_timeout) || value.watchdog_timeout <= std::max(settings_.feedback_stop, 2 / rate_) ||
        value.watchdog_timeout > 0.2 || !std::isfinite(value.state_rate) || value.state_rate < 2 / settings_.feedback_stop)
      {
        begin_stop(axis.name + ": incompatible driver timeout, watchdog or state rate", true);
      } else {axis.configuration_checked = true;}
    }
  }

  void begin_stop(const std::string & reason, bool fault)
  {
    operator_session_.clear();
    if (phase_ == State::STOPPING) {return;}
    fault_stop_ = fault; phase_ = State::STOPPING; phase_since_ = dm_motor::steady_seconds(); last_stop_ = 0;
    message_ = reason; stop_warned_ = false;
    for (auto & axis : axes_) {
      axis.stop_count = axis.state ? axis.state->feedback_count : 0;
      dm_motor::send_motor_stop(axis, client_, request_id_, now());
    }
    if (fault) {RCLCPP_ERROR(get_logger(), "%s; stopping both motors", reason.c_str());}
    else {RCLCPP_INFO(get_logger(), "%s", reason.c_str());}
  }

  void stop_tick(double time)
  {
    bool confirmed = true;
    for (auto & axis : axes_) {
      if (time - last_stop_ >= 0.2) {dm_motor::send_motor_stop(axis, client_, request_id_, now());}
      if (!axis.pending && !axis.token.empty()) {request(axis, Request::LEASE, Request::RELEASE);}
      confirmed = confirmed && disabled(axis) && axis.state->feedback_count > axis.stop_count &&
        axis.token.empty() && !axis.pending;
    }
    if (time - last_stop_ >= 0.2) {last_stop_ = time;}
    if (confirmed && time - phase_since_ > 0.6) {
      phase_ = fault_stop_ ? State::FAULT : State::IDLE;
      message_ += "; both motors confirmed disabled";
    } else if (!stop_warned_ && time - phase_since_ > 3) {
      stop_warned_ = true; message_ += "; STOP UNCONFIRMED, inspect devices; retrying";
      RCLCPP_ERROR(get_logger(), "%s", message_.c_str());
    }
  }

  void send_targets(const std::array<dm_motor::Command, 2> & commands)
  {
    for (size_t i = 0; i < 2; ++i) {
      Target target;
      target.mode = 1; target.position = commands[i].p; target.velocity = commands[i].v;
      target.kp = commands[i].kp; target.kd = commands[i].kd; target.torque = 0;
      dm_motor::send_motor_target(axes_[i], target, client_, validity_, now());
    }
    ++command_count_;
  }

  void tick()
  {
    const double time = dm_motor::steady_seconds(), dt = time - last_tick_;
    last_tick_ = time;
    try {
      for (auto & axis : axes_) {
        if (axis.pending && time - axis.pending_since > 0.8) {
          axis.pending.reset();
          if (phase_ != State::STOPPING) {throw std::runtime_error(axis.name + ": motor request timed out");}
          RCLCPP_ERROR(get_logger(), "%s: pending request expired during STOP", axis.name.c_str());
          axis.token.clear();
        }
      }
      if (phase_ == State::STOPPING) {stop_tick(time); return;}
      if (!active()) {return;}
      if (!operator_session_.empty() && time - heartbeat_ > operator_timeout_) {
        throw std::runtime_error("Bilateral operator heartbeat lost");
      }
      if (dt > std::min(validity_, settings_.feedback_stop)) {
        throw std::runtime_error("Bilateral control timer stalled");
      }
      for (size_t i = 0; i < 2; ++i) {
        auto & axis = axes_[i];
        check_axis(i);
        if (phase_ != State::ACTIVE && std::abs(axis.state->position - axis.origin) > 0.02) {
          throw std::runtime_error("Motor moved while arming; stop and retry stationary");
        }
        if (!axis.token.empty() && !axis.pending && time >= axis.renew_at) {request(axis, Request::LEASE, Request::RENEW);}
      }
      if (axes_[0].state->simulated != axes_[1].state->simulated) {throw std::runtime_error("Motor environment changed");}
      if (phase_ != State::ACTIVE && time - phase_since_ > 3) {throw std::runtime_error("Bilateral arming timed out");}
      if (phase_ == State::ACQUIRING) {
        for (auto & axis : axes_) {
          if (!axis.token.empty() && !axis.pending && !axis.configuration_checked) {request(axis, Request::CAPABILITIES, 0);}
        }
        if (std::all_of(axes_.begin(), axes_.end(), [](const Axis & axis) {
            return !axis.token.empty() && !axis.pending && axis.configuration_checked;
          }))
        {
          phase_ = State::ENABLING; phase_since_ = time; message_ = "Enabling both motors with neutral MIT targets";
          for (auto & axis : axes_) {request(axis, Request::OPERATION, Request::ENABLE);}
        }
        return;
      }
      if (phase_ == State::ENABLING) {
        for (size_t i = 0; i < 2; ++i) {
          if (axes_[i].state->phase == Motor::ENABLED) {
            Target neutral; neutral.mode = 1; neutral.position = axes_[i].state->position;
            dm_motor::send_motor_target(axes_[i], neutral, client_, validity_, now());
          }
        }
        if (std::all_of(axes_.begin(), axes_.end(), [](const Axis & axis) {
            return !axis.pending && axis.state->phase == Motor::ENABLED && axis.state->status == 1;
          }))
        {
          if (!enabled_pair_after_) {enabled_pair_after_ = time; pairs_.reset(time);}
          if (pairs_.select(time, settings_.feedback_age, settings_.pair_skew)) {
            phase_ = State::ACTIVE; recovery_.reset(); message_ = "Standard relative-coordinate bilateral coupling";
            desired_ = relative_targets(*pairs_.pair(), origins(), settings_);
            ++pair_count_;
          } else if (time - enabled_pair_after_ > settings_.feedback_stop) {
            throw std::runtime_error("No fresh paired feedback after neutral enabling");
          }
        }
        return;
      }
      for (const auto & axis : axes_) {
        if (axis.state->phase != Motor::ENABLED || axis.state->status != 1) {
          throw std::runtime_error(axis.name + ": lost enable confirmation");
        }
      }
      const bool updated = pairs_.select(time, settings_.feedback_age, settings_.pair_skew);
      if (!pairs_.pair()) {throw std::runtime_error("No valid bilateral feedback pair");}
      pair_waiting_ = !updated;
      const auto previous_quality = recovery_.quality;
      recovery_.update(pairs_.age(time), updated, dt, 1 / rate_, settings_);
      if (updated) {desired_ = relative_targets(*pairs_.pair(), origins(), settings_); ++pair_count_;}
      if (recovery_.quality != previous_quality) {
        RCLCPP_INFO(get_logger(), "Bilateral feedback quality %u; recovery scale %.3f",
          static_cast<unsigned>(recovery_.quality), recovery_.scale);
      }
      if (recovery_.scale == 0) {
        auto neutral = desired_;
        for (size_t i = 0; i < 2; ++i) {neutral[i] = {latest_[i]->position, 0, 0, 0, 0};}
        output_ = symmetric_targets(neutral, latest(), limits_, settings_, 0);
      } else {output_ = symmetric_targets(desired_, latest(), limits_, settings_, recovery_.scale);}
      send_targets(output_.targets);
    } catch (const std::exception & error) {
      if (phase_ == State::STOPPING) {RCLCPP_ERROR(get_logger(), "Bilateral STOP retry: %s", error.what());}
      else {begin_stop(error.what(), true);}
    }
  }

  void publish_state()
  {
    const double time = dm_motor::steady_seconds();
    if ((phase_ == State::IDLE || phase_ == State::FAULT) && latest_[0] && latest_[1] &&
      disabled(axes_[0]) && disabled(axes_[1]))
    {
      pairs_.select(time, settings_.feedback_age, settings_.pair_skew);
    }
    State state;
    state.header.stamp = now(); state.phase = phase_; state.quality = static_cast<uint8_t>(recovery_.quality);
    state.message = message_; state.motor_a = axes_[0].name; state.motor_b = axes_[1].name;
    state.direction = settings_.direction; state.motion_limits_enabled = settings_.motion_limits_enabled;
    state.stop_confirmed = (phase_ == State::IDLE || phase_ == State::FAULT) && disabled(axes_[0]) && disabled(axes_[1]);
    state.operator_guarded = !operator_session_.empty(); state.pair_waiting = pair_waiting_;
    state.commands_ready = std::all_of(axes_.begin(), axes_.end(), [](const Axis & axis) {
        return axis.control->get_subscription_count() > 0 && axis.setpoint->get_subscription_count() > 0 &&
               axis.replies->get_publisher_count() > 0;
      });
    state.origin_a = axes_[0].origin; state.origin_b = axes_[1].origin; state.paired = pairs_.pair().has_value();
    if (state.paired) {
      const auto & pair = *pairs_.pair();
      state.position_error = (pair[0].position - axes_[0].origin) -
        settings_.direction * (pair[1].position - axes_[1].origin);
      state.relative_velocity = pair[0].velocity - settings_.direction * pair[1].velocity;
      state.pair_age = std::max(0.0, pairs_.age(dm_motor::steady_seconds())); state.pair_skew = pairs_.skew();
    }
    state.pair_count = pair_count_; state.degradation_count = recovery_.degradation_count;
    state.recovery_pairs = recovery_.fresh_pairs; state.recovery_scale = recovery_.scale;
    state.gain_scale = output_.gain_scale; state.decoded_kp = output_.decoded_kp; state.decoded_kd = output_.decoded_kd;
    state.estimated_torque_a = output_.estimated_torque[0]; state.estimated_torque_b = output_.estimated_torque[1];
    state.raw_torque_a = output_.raw_torque[0]; state.raw_torque_b = output_.raw_torque[1];
    for (size_t i = 0; i < 2; ++i) {
      auto & target = i == 0 ? state.target_a : state.target_b;
      const auto & command = output_.targets[i];
      target.mode = 1; target.position = command.p; target.velocity = command.v;
      target.kp = command.kp; target.kd = command.kd; target.torque = 0;
    }
    state.command_count = command_count_;
    publisher_->publish(state);
  }

  std::array<Axis, 2> axes_;
  std::array<std::optional<Sample>, 2> latest_;
  std::array<AxisLimits, 2> limits_;
  std::array<rclcpp::Subscription<Feedback>::SharedPtr, 2> feedback_;
  FeedbackPairs pairs_;
  Recovery recovery_;
  Settings settings_;
  Output output_;
  std::array<dm_motor::Command, 2> desired_;
  std::string client_, message_{"Idle; explicit start required"}, operator_session_;
  uint8_t phase_{State::IDLE};
  uint64_t request_id_{0}, command_count_{0}, pair_count_{0};
  double rate_{0}, validity_{0}, operator_timeout_{0}, heartbeat_{0}, last_tick_{0}, phase_since_{0};
  double enabled_pair_after_{0}, last_stop_{0};
  bool fault_stop_{false}, stop_warned_{false}, pair_waiting_{false};
  rclcpp::Publisher<State>::SharedPtr publisher_;
  rclcpp::Service<Trigger>::SharedPtr start_, stop_;
  rclcpp::Service<Control>::SharedPtr operator_;
  rclcpp::TimerBase::SharedPtr timer_, state_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameters_;
};
}  // namespace dm_motor_bilateral_control

namespace
{
volatile std::sig_atomic_t stopping = 0;
void request_stop(int) {stopping = 1;}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, request_stop); std::signal(SIGTERM, request_stop);
  int result = 0;
  try {
    auto node = std::make_shared<dm_motor_bilateral_control::BilateralNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stopping) {executor.spin_once(std::chrono::milliseconds(5));}
    if (rclcpp::ok()) {
      node->shutdown_stop();
      const double deadline = dm_motor::steady_seconds() + 3;
      while (rclcpp::ok() && node->stopping() && dm_motor::steady_seconds() < deadline) {
        executor.spin_once(std::chrono::milliseconds(5));
      }
      if (node->stopping()) {
        RCLCPP_ERROR(node->get_logger(), "Bilateral shutdown STOP unconfirmed; inspect devices");
        result = 1;
      }
    }
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("bilateral_control"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
