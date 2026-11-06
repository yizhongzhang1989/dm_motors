#include "dm_motor_bilateral_control/bilateral.hpp"
#include "dm_motor_bilateral_control/neutral_target.hpp"
#include "dm_motor_bilateral_control/velocity_bandwidth.hpp"
#include "dm_motor_bilateral_control/stop_confirmation.hpp"
#include "dm_motor_control/motor_session.hpp"
#include "dm_motor_control/system_parameters.hpp"
#include "common/config.hpp"

#include <dm_motor_interfaces/msg/bilateral_state.hpp>
#include <dm_motor_interfaces/msg/motor_feedback.hpp>
#include <dm_motor_interfaces/srv/bilateral_control.hpp>
#include <dm_motor_interfaces/srv/bilateral_disturbance.hpp>
#include <dm_motor_interfaces/srv/bilateral_angle_offset.hpp>
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
using Disturbance = dm_motor_interfaces::srv::BilateralDisturbance;
using AngleOffsetService = dm_motor_interfaces::srv::BilateralAngleOffset;
using Configuration = dm_motor_interfaces::msg::MotorConfiguration;
using Trigger = std_srvs::srv::Trigger;
using Axis = dm_motor::MotorSession;

CouplingMode parse_coupling_mode(const std::string & value)
{
  if (value == "bilateral") {return CouplingMode::Bilateral;}
  if (value == "a_leads_b") {return CouplingMode::ALeadsB;}
  throw std::invalid_argument("coupling_mode must be bilateral or a_leads_b");
}

std::map<std::string, double *> settings_fields(Settings & settings)
{
  return {{"stiffness", &settings.stiffness}, {"coupling_damping", &settings.coupling_damping},
    {"local_damping", &settings.local_damping}, {"max_torque", &settings.max_torque},
    {"friction_compensation_fraction", &settings.friction_compensation_fraction},
    {"friction_torque_limit", &settings.friction_torque_limit},
    {"extra_viscous_a", &settings.extra_viscous[0]},
    {"extra_viscous_b", &settings.extra_viscous[1]},
    {"extra_viscous_velocity_limit", &settings.extra_viscous_velocity_limit},
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
    control_mode_ = declare_parameter<std::string>("control_mode",
      config.has(prefix + "control_mode") ? config.get<std::string>(prefix + "control_mode") : "native", fixed);
    if (control_mode_ == "native") {settings_.control_mode = ControlMode::Native;}
    else if (control_mode_ == "shared_reference") {settings_.control_mode = ControlMode::SharedReference;}
    else if (control_mode_ == "differential_torque") {settings_.control_mode = ControlMode::DifferentialTorque;}
    else {throw std::invalid_argument("control_mode must be native, shared_reference or differential_torque");}
    settings_.coupling_mode = parse_coupling_mode(declare_parameter<std::string>("coupling_mode",
      config.has(prefix + "coupling_mode") ? config.get<std::string>(prefix + "coupling_mode") : "bilateral"));
    allow_differential_hardware_ = declare_parameter<bool>("allow_differential_hardware", false, fixed);
    if (allow_differential_hardware_ && settings_.control_mode != ControlMode::DifferentialTorque) {
      throw std::invalid_argument("allow_differential_hardware requires differential_torque control mode");
    }
    required_velocity_bandwidth_ = declare_parameter<double>("required_velocity_bandwidth",
      config.has(prefix + "required_velocity_bandwidth") ?
      config.get<double>(prefix + "required_velocity_bandwidth") : 0.0, fixed);
    VelocityBandwidthCheck::validate(required_velocity_bandwidth_);
    system_parameters_file_ = declare_parameter<std::string>(
      "system_parameters_file", dm_motor::system_parameters_path(config), fixed);
    axes_[0].name = declare_parameter<std::string>("motor_a", config.get<std::string>(prefix + "motor_a"), fixed);
    axes_[1].name = declare_parameter<std::string>("motor_b", config.get<std::string>(prefix + "motor_b"), fixed);
    rate_ = declare_parameter<double>("rate", config.get<double>(prefix + "rate"), fixed);
    validity_ = declare_parameter<double>("target_validity", config.get<double>(prefix + "target_validity"), fixed);
    operator_timeout_ = declare_parameter<double>("operator_timeout", config.get<double>(prefix + "operator_timeout"), fixed);
    if (get_parameter("use_sim_time").as_bool()) {throw std::invalid_argument("Bilateral control requires system ROS time");}
    if (!std::isfinite(rate_) || rate_ < 20 || rate_ > 1000 ||
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
    settings_.friction_compensation_enabled = declare_parameter<bool>(
      "friction_compensation_enabled", config.get<bool>(prefix + "friction_compensation_enabled"));
    for (const auto & item : settings_fields(settings_)) {
      const bool optional_assistance = item.first.rfind("extra_viscous_", 0) == 0;
      const double fallback = optional_assistance && !config.has(prefix + item.first) ?
        *item.second : config.get<double>(prefix + item.first);
      *item.second = declare_parameter<double>(item.first, fallback);
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
            else if (name == "friction_compensation_enabled") {candidate.friction_compensation_enabled = value.as_bool();}
            else if (name == "coupling_mode") {candidate.coupling_mode = parse_coupling_mode(value.as_string());}
            else if (fields.count(name)) {*fields.at(name) = value.as_double();}
            else {throw std::invalid_argument("Parameter is not runtime configurable: " + name);}
          }
          validate_settings(candidate);
          settings_ = candidate;
          reset_compensation(); models_checked_ = false; output_ = {};
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
    disturbance_service_ = create_service<Disturbance>("~/disturbance",
      [this](Disturbance::Request::SharedPtr request, Disturbance::Response::SharedPtr response) {
        try {
          const auto stamp = rclcpp::Time(request->header.stamp);
          const double age = (now() - stamp).seconds();
          if (!std::isfinite(request->valid_for) || request->valid_for <= 0 ||
            request->valid_for > 0.5 || age < -0.05 || age > request->valid_for)
          {
            throw std::invalid_argument("Bilateral disturbance request expired or has invalid timing");
          }
          if (operator_session_.empty() || request->session != operator_session_) {
            throw std::runtime_error("Disturbance requires the current guarded bilateral operator session");
          }
          const double time = dm_motor::steady_seconds();
          if (!disturbance_permitted(time)) {
            throw std::runtime_error("Disturbance requires ACTIVE, a fresh operator heartbeat and NORMAL fully recovered feedback");
          }
          if (stamp.nanoseconds() <= last_disturbance_stamp_) {
            throw std::runtime_error("Disturbance request replayed; accepted stamps must increase");
          }
          if (angle_offset_.active()) {throw std::runtime_error("Torque pulses and angle offsets are mutually exclusive");}
          for (size_t i = 0; i < axes_.size(); ++i) {
            check_axis(i);
            if (axes_[i].state->phase != Motor::ENABLED || axes_[i].state->status != 1) {
              throw std::runtime_error("Disturbance requires both motors freshly enabled");
            }
          }
          // Validate without mutating the running pulse. A rejected request never
          // extends a deadline, replaces a pulse, or changes the accepted id.
          auto candidate = disturbance_;
          const auto id = candidate.start(request->axis, request->torque, request->duration, time);
          auto desired = desired_;
          for (size_t i = 0; i < axes_.size(); ++i) {desired[i].t = output_.friction_torque[i];}
          symmetric_targets(desired, latest(), limits_, settings_, 1, candidate.update(time, true),
            spring_positions(time));
          disturbance_ = candidate;
          last_disturbance_stamp_ = stamp.nanoseconds();
          response->success = true; response->disturbance_id = id;
          response->message = "Bounded disturbance accepted; check disturbance_id and output components";
          RCLCPP_INFO(get_logger(), "Bilateral disturbance %llu: axis %u, %.3f Nm for %.3f s",
            static_cast<unsigned long long>(id), static_cast<unsigned>(request->axis), request->torque, request->duration);
        } catch (const std::exception & error) {
          response->message = error.what();
          RCLCPP_WARN(get_logger(), "Bilateral disturbance rejected: %s", error.what());
        }
      });
    angle_offset_service_ = create_service<AngleOffsetService>("~/angle_offset",
      [this](AngleOffsetService::Request::SharedPtr request, AngleOffsetService::Response::SharedPtr response) {
        try {
          const auto stamp = rclcpp::Time(request->header.stamp);
          const double age = (now() - stamp).seconds();
          if (!std::isfinite(request->valid_for) || request->valid_for <= 0 ||
            request->valid_for > 0.5 || age < -0.05 || age > request->valid_for)
          {
            throw std::invalid_argument("Angle-offset request expired or has invalid timing");
          }
          if (operator_session_.empty() || request->session != operator_session_) {
            throw std::runtime_error("Angle offset requires the current guarded bilateral operator session");
          }
          const double time = dm_motor::steady_seconds();
          if (!disturbance_permitted(time)) {
            throw std::runtime_error("Angle offset requires ACTIVE, a fresh operator heartbeat and NORMAL fully recovered feedback");
          }
          if (stamp.nanoseconds() <= last_angle_offset_stamp_) {
            throw std::runtime_error("Angle-offset request replayed; accepted stamps must increase");
          }
          if (disturbance_.active()) {throw std::runtime_error("Torque pulses and angle offsets are mutually exclusive");}
          for (size_t i = 0; i < axes_.size(); ++i) {
            check_axis(i);
            if (axes_[i].state->phase != Motor::ENABLED || axes_[i].state->status != 1) {
              throw std::runtime_error("Angle offset requires both motors freshly enabled");
            }
            if (!axes_[i].state->simulated && request->transition_time < 2) {
              throw std::invalid_argument("Physical motors require an angle-offset transition of at least 2 seconds");
            }
          }
          auto candidate = angle_offset_;
          const auto id = candidate.start(request->offset, request->transition_time, request->hold_duration, time);
          auto desired = desired_;
          for (size_t i = 0; i < axes_.size(); ++i) {desired[i].t = output_.friction_torque[i];}
          // A monotonic quintic stays between these two endpoints. Runtime checks
          // still validate the new targets against current feedback on every tick.
          const auto synchronized = spring_positions(time);
          symmetric_targets(desired, latest(), limits_, settings_, 1, {}, synchronized);
          symmetric_targets(apply_angle_offset(desired, request->offset, settings_),
            latest(), limits_, settings_, 1, {}, synchronized);
          angle_offset_ = candidate;
          last_angle_offset_stamp_ = stamp.nanoseconds();
          response->success = true; response->offset_id = id;
          response->message = "Virtual angle offset accepted; wait for this id and applied offset to reach the target";
          RCLCPP_INFO(get_logger(), "Bilateral angle offset %llu: %.6f rad, transition %.3f s, hold %.3f s",
            static_cast<unsigned long long>(id), request->offset, request->transition_time, request->hold_duration);
        } catch (const std::exception & error) {
          response->message = error.what();
          RCLCPP_WARN(get_logger(), "Bilateral angle offset rejected: %s", error.what());
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
  void reset_compensation()
  {
    disturbance_.cancel();
    angle_offset_.cancel();
    for (auto & compensation : compensators_) {if (compensation) {compensation->reset();}}
    for (auto & filter : peer_velocity_filters_) {filter.reset();}
    compensation_clamped_ = {};
  }

  bool disturbance_permitted(double time) const
  {
    return phase_ == State::ACTIVE && !operator_session_.empty() &&
           time - heartbeat_ >= 0 && time - heartbeat_ <= operator_timeout_ &&
           recovery_.quality == Quality::Normal && recovery_.scale == 1 &&
           pairs_.pair() && pairs_.age(time) <= settings_.feedback_age;
  }

  // Read only while stopped; an explicit START always reloads the shared file.
  void load_compensation()
  {
    compensators_ = {}; compensation_sources_ = {}; compensation_messages_ = {};
    compensation_clamped_ = {}; models_checked_ = true;
    last_models_check_ = dm_motor::steady_seconds();
    try {
      if (system_parameters_file_.empty()) {throw std::runtime_error("No shared system_parameters_file configured");}
      const dm_motor::SystemParameters shared(system_parameters_file_);
      for (size_t i = 0; i < axes_.size(); ++i) {
        try {
          if (!axes_[i].state) {throw std::runtime_error("Waiting for motor identity");}
          const auto & state = *axes_[i].state;
          compensators_[i] = shared.compensator(axes_[i].name, state);
          if (!compensators_[i]) {
            throw std::runtime_error(std::string("No matching ") +
              (state.simulated ? "simulation" : "hardware") + " friction calibration; disable compensation or calibrate this axis");
          }
          const auto description = shared.motor(axes_[i].name);
          const auto calibration = (*description)[state.simulated ? "simulation" : "hardware"]["calibrations"]["mechanical"];
          compensation_sources_[i] = calibration["source"] ?
            dm_motor::motor_parameters_detail::scalar<std::string>(calibration["source"]) : system_parameters_file_;
          compensation_messages_[i] =
            "Matched friction model; measured-velocity assistance, bounded to its calibration domain";
          if (settings_.extra_viscous[i] > 0) {
            compensation_messages_[i] += "; explicit extra viscosity " +
              std::to_string(settings_.extra_viscous[i]) + " Nm s/rad, speed ceiling " +
              std::to_string(settings_.extra_viscous_velocity_limit) + " rad/s; separate from calibration";
          }
        } catch (const std::exception & error) {
          compensators_[i].reset(); compensation_messages_[i] = error.what();
        }
      }
    } catch (const std::exception & error) {
      for (auto & message : compensation_messages_) {message = error.what();}
    }
  }

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
    if (settings_.extra_viscous[i] > 0 && settings_.extra_viscous_velocity_limit >
      dm_motor::resolve_limits(limits(state).mapping, limits(state).command).v)
    {
      throw std::runtime_error(axis.name + ": extra viscosity speed ceiling exceeds driver capabilities");
    }
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
    if (active()) {
      const auto & previous = limits_[i];
      const auto & identity = start_identities_[i];
      if (state.motor_id != identity.motor_id || state.master_id != identity.master_id ||
        state.simulated != identity.simulated || state.direction != previous.direction || state.software_offset != previous.offset ||
        state.pmax != previous.mapping.p || state.vmax != previous.mapping.v || state.tmax != previous.mapping.t ||
        state.position_min != previous.position_min || state.position_max != previous.position_max ||
        state.position_limit != previous.command.p || state.velocity_limit != previous.command.v ||
        state.torque_limit != previous.command.t || state.kp_limit != previous.command.kp || state.kd_limit != previous.command.kd)
      {
        throw std::runtime_error(axis.name + ": motor identity, coordinates or command limits changed; explicit restart required");
      }
      if (state.rejected_commands != axis.rejected) {
        throw std::runtime_error(axis.name + ": driver rejected a command during bilateral control");
      }
      if (state.target_active && state.active_source != client_) {
        throw std::runtime_error(axis.name + ": another command source replaced the bilateral target: " + state.active_source);
      }
      if (phase_ == State::ACTIVE && dm_motor::steady_seconds() - phase_since_ > validity_ && !state.target_active) {
        throw std::runtime_error(axis.name + ": bilateral target is no longer active");
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
      if (!active() && latest_[i] && (sample.sequence <= latest_[i]->sequence || sample.time < latest_[i]->time)) {
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
      start_identities_[i] = *axis.state;
      axis.rejected = axis.state->rejected_commands;
      axis.configuration_checked = false;
      bandwidth_checks_[i].reset(required_velocity_bandwidth_);
    }
    if (axes_[0].state->simulated != axes_[1].state->simulated) {
      throw std::runtime_error("Do not couple simulated and physical motors");
    }
    if (settings_.control_mode == ControlMode::DifferentialTorque &&
      !axes_[0].state->simulated && !allow_differential_hardware_) {
      throw std::runtime_error("Differential torque requires simulated motors or explicit allow_differential_hardware at startup");
    }
    load_compensation();
    if (settings_.friction_compensation_enabled) {
      for (size_t i = 0; i < axes_.size(); ++i) {
        if (!compensators_[i]) {throw std::runtime_error(axes_[i].name + ": " + compensation_messages_[i]);}
      }
    }
    if (!pairs_.select(time, settings_.feedback_age, settings_.pair_skew)) {
      if (!pairs_.pair() || pairs_.age(time) > settings_.feedback_age || pairs_.skew() > settings_.pair_skew) {
        throw std::runtime_error("No fresh disabled feedback pair; no motor was enabled");
      }
    }
    for (size_t i = 0; i < 2; ++i) {axes_[i].origin = (*pairs_.pair())[i].position;}
    const auto desired = reference_targets(false);
    const auto validation_samples = settings_.control_mode != ControlMode::Native ?
      latest() : *pairs_.pair();
    symmetric_targets(desired, validation_samples, limits_, settings_, 1, {}, spring_positions(time));
    output_ = {}; reset_compensation(); recovery_.reset(); pair_count_ = 0; pair_waiting_ = false;
    bandwidth_verified_ = false; bandwidth_post_read_ = 0; bandwidth_poll_after_ = 0;
    last_disturbance_stamp_ = 0;
    last_angle_offset_stamp_ = 0;
    phase_ = State::ACQUIRING; phase_since_ = time; fault_stop_ = false; enabled_pair_after_ = 0;
    message_ = "Acquiring non-exclusive motor sessions and verifying driver configuration";
    try {
      for (auto & axis : axes_) {request(axis, Request::LEASE, Request::ACQUIRE);}
    } catch (const std::exception & error) {begin_stop(error.what(), true); throw;}
  }

  std::array<double, 2> origins() const {return {axes_[0].origin, axes_[1].origin};}
  std::array<Sample, 2> latest() const {return {*latest_[0], *latest_[1]};}
  double control_velocity(size_t i, double measured) const
  {
    if (settings_.control_mode == ControlMode::Native && settings_.max_torque > 0) {return measured;}
    // Feedback decoding returns the lower edge of a 12-bit native velocity bin.
    // Its midpoint removes the directional bias; raw feedback and protection checks stay unchanged.
    const auto & axis = limits_[i];
    return std::clamp(measured + axis.direction * axis.mapping.v / 4095.0, -axis.mapping.v, axis.mapping.v);
  }
  std::array<dm_motor::Command, 2> reference_targets(bool update_filters)
  {
    if (settings_.control_mode != ControlMode::Native) {
      auto reference = latest();
      for (size_t i = 0; i < reference.size(); ++i) {
        reference[i].velocity = update_filters ? peer_velocity_filters_[i].update(
          control_velocity(i, reference[i].velocity), reference[i].time) : 0;
      }
      // The reference follows both measured positions. It is not a fixed world
      // center, and no position prediction is added to this control mode.
      auto targets = shared_reference_targets(reference, origins(), settings_);
      return targets;
    }
    auto pair = *pairs_.pair();
    if (!update_filters) {return relative_targets(pair, origins(), settings_);}
    for (size_t i = 0; i < pair.size(); ++i) {
      pair[i].velocity = peer_velocity_filters_[i].update(
        control_velocity(i, pair[i].velocity), pair[i].time);
    }
    return relative_targets(pair, latest(), origins(), settings_);
  }

  std::optional<CommonTimePositions> spring_positions(double time) const
  {
    if (settings_.control_mode != ControlMode::DifferentialTorque) {return std::nullopt;}
    // Align only positions, using received history. This does not predict from
    // noisy compact velocity or replace the latest velocity used for damping.
    // A wide interpolation hole is not evidence of synchronous movement.
    std::string rejection;
    // At 500 Hz each interpolation bracket is at most 8 ms. A newer wide
    // receive gap may use an earlier common point, but every contributing
    // endpoint must still satisfy the independent feedback-age bound.
    auto synchronized = pairs_.common_time_positions(time, settings_.feedback_age,
      std::min(settings_.feedback_age, 4.0 / rate_), &rejection);
    if (!synchronized) {
      throw std::runtime_error("Differential spring requires fresh common-time position history: " + rejection +
        "; feedback_sequences=" + std::to_string(latest_[0] ? latest_[0]->sequence : 0) + "/" +
        std::to_string(latest_[1] ? latest_[1]->sequence : 0));
    }
    return synchronized;
  }
  void request(Axis & axis, uint8_t kind, uint8_t operation)
  {
    dm_motor::send_motor_request(axis, kind, operation, client_, request_id_, now());
  }

  bool verify_velocity_bandwidth(double time)
  {
    if (required_velocity_bandwidth_ == 0) {return true;}
    // Both axes remain disabled. Serialize maintenance reads so that the CAN
    // reservation never overlaps two controller register operations.
    for (size_t i = 0; i < axes_.size(); ++i) {
      auto & check = bandwidth_checks_[i];
      if (check.verified()) {continue;}
      if (time < bandwidth_poll_after_ || std::any_of(axes_.begin(), axes_.end(),
          [](const Axis & axis) {return axis.pending.has_value();})) {return false;}
      if (!disabled(axes_[0]) || !disabled(axes_[1])) {
        throw std::runtime_error("V_BW verification requires both motors freshly disabled");
      }
      auto & axis = axes_[i];
      Request request;
      request.header.stamp = now(); request.client_id = client_; request.request_id = ++request_id_;
      request.valid_for = 0.5; request.token = axis.token;
      if (check.operation()) {
        request.kind = Request::OPERATION_RESULT; request.operation_id = check.operation();
      } else {
        request.kind = Request::REGISTERS; request.addresses = {VelocityBandwidthCheck::address};
        request.write = false;
      }
      axis.pending = request; axis.pending_since = time;
      bandwidth_poll_after_ = time + 0.01;
      axis.control->publish(request);
      message_ = "Reading actual V_BW before enabling: " + axis.name;
      return false;
    }
    if (!bandwidth_post_read_) {
      bandwidth_post_read_ = time;
      for (size_t i = 0; i < axes_.size(); ++i) {
        bandwidth_post_counts_[i] = axes_[i].state->feedback_count;
      }
      pairs_.reset(time);
      return false;
    }
    for (size_t i = 0; i < axes_.size(); ++i) {
      if (!disabled(axes_[i]) || axes_[i].state->feedback_count <= bandwidth_post_counts_[i] ||
        !axes_[i].state->activity_available || !latest_[i] ||
        latest_[i]->time <= bandwidth_post_read_) {return false;}
      if (std::abs(axes_[i].state->velocity) > 0.05 || std::abs(latest_[i]->velocity) > 0.05) {
        // Stay disabled and retry on later tick callbacks within the original
        // ACQUIRING deadline. Discard every pair/count predating this rejection.
        message_ = "Waiting for both motors stationary after V_BW verification";
        bandwidth_post_read_ = time;
        for (size_t j = 0; j < axes_.size(); ++j) {
          bandwidth_post_counts_[j] = axes_[j].state->feedback_count;
        }
        pairs_.reset(time);
        return false;
      }
    }
    if (!pairs_.select(time, settings_.feedback_age, settings_.pair_skew)) {return false;}
    const auto & pair = *pairs_.pair();
    if (pair[0].time <= bandwidth_post_read_ || pair[1].time <= bandwidth_post_read_) {return false;}
    for (size_t i = 0; i < axes_.size(); ++i) {axes_[i].origin = pair[i].position;}
    bandwidth_verified_ = true;
    return true;
  }

  void receive_reply(size_t i, const Reply & reply)
  {
    auto & axis = axes_[i];
    if (reply.client_id != client_) {return;}
    if (reply.kind == Request::CAPABILITIES && reply.success) {
      driver_configurations_[i] = reply.configuration;
      configuration_received_[i] = dm_motor::steady_seconds();
    }
    if (reply.kind == Request::CAPABILITIES && reply.request_id == preview_requests_[i]) {return;}
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
        !std::isfinite(value.control_rate) || value.control_rate < rate_ ||
        !std::isfinite(value.watchdog_timeout) || value.watchdog_timeout <= std::max(settings_.feedback_stop, 2 / rate_) ||
        value.watchdog_timeout > 0.2 || !std::isfinite(value.state_rate) || value.state_rate < 2 / settings_.feedback_stop)
      {
        begin_stop(axis.name + ": incompatible driver rate, timeout, watchdog or state rate; driver control_rate must cover bilateral rate", true);
      } else if (!std::isfinite(value.estimated_torque_limit) || value.estimated_torque_limit < 0 ||
        (value.host_protection_enabled && value.estimated_torque_limit > 0 &&
        (settings_.max_torque == 0 || value.estimated_torque_limit < settings_.max_torque)))
      {
        std::ostringstream reason;
        reason << axis.name << ": driver estimated_torque_limit=" << value.estimated_torque_limit
               << " is incompatible with bilateral application torque budget=" << settings_.max_torque;
        if (settings_.max_torque == 0) {
          reason << " (no application budget); use the bilateral driver profile or set a matching positive budget";
        }
        begin_stop(reason.str(), true);
      } else {axis.configuration_checked = true;}
    } else if (pending.kind == Request::REGISTERS || pending.kind == Request::OPERATION_RESULT) {
      // STOP can race with a late, otherwise valid read result. It must never
      // complete arming or prime the next START.
      if (phase_ != State::ACQUIRING) {return;}
      try {
        if (pending.kind == Request::REGISTERS) {
          bandwidth_checks_[i].queued(reply.operation_ids);
        } else if (bandwidth_checks_[i].observe(
            pending.operation_id, reply.state, reply.address, reply.raw_value))
        {
          RCLCPP_INFO(get_logger(), "%s: fresh V_BW readback %.3f Hz verified for this START",
            axis.name.c_str(), bandwidth_checks_[i].value());
        }
      } catch (const std::exception & error) {begin_stop(axis.name + ": " + error.what(), true);}
    }
  }

  void begin_stop(const std::string & reason, bool fault)
  {
    operator_session_.clear();
    reset_compensation(); output_ = {};
    bandwidth_verified_ = false;
    for (auto & check : bandwidth_checks_) {check.cancel();}
    if (phase_ == State::STOPPING) {return;}
    fault_stop_ = fault; phase_ = State::STOPPING; phase_since_ = dm_motor::steady_seconds(); last_stop_ = 0;
    message_ = reason; stop_warned_ = false;
    for (auto & axis : axes_) {
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
      confirmed = confirmed && motor_stopped_after(
        axis, now(), time, phase_since_, settings_.feedback_stop);
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
      target.kp = commands[i].kp; target.kd = commands[i].kd; target.torque = commands[i].t;
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
          }) && verify_velocity_bandwidth(time))
        {
          phase_ = State::ENABLING; phase_since_ = time; message_ = "Enabling both motors with neutral MIT targets";
          for (auto & axis : axes_) {request(axis, Request::OPERATION, Request::ENABLE);}
        }
        return;
      }
      if (phase_ == State::ENABLING) {
        for (size_t i = 0; i < 2; ++i) {
          if (axes_[i].state->phase == Motor::ENABLED) {
            Target neutral; neutral.mode = 1; neutral.position = neutral_mit_target(limits_[i]).p;
            dm_motor::send_motor_target(axes_[i], neutral, client_, validity_, now());
          }
        }
        if (std::all_of(axes_.begin(), axes_.end(), [](const Axis & axis) {
            return !axis.pending && axis.state->phase == Motor::ENABLED && axis.state->status == 1;
          }))
        {
          if (!enabled_pair_after_) {enabled_pair_after_ = time; pairs_.reset(time);}
          if (pairs_.select(time, settings_.feedback_age, settings_.pair_skew) &&
            (settings_.control_mode != ControlMode::DifferentialTorque ||
            pairs_.common_time_positions(time, settings_.feedback_age,
              std::min(settings_.feedback_age, 4.0 / rate_))))
          {
            phase_ = State::ACTIVE; phase_since_ = time; recovery_.reset();
            message_ = (settings_.control_mode == ControlMode::DifferentialTorque ?
              std::string("Common-time differential host spring with native damping") :
              settings_.control_mode == ControlMode::SharedReference ?
              std::string("Shared-reference bilateral coupling") : std::string("Relative-coordinate bilateral coupling")) +
              (settings_.coupling_mode == CouplingMode::ALeadsB ? "; Unilateral A leads B, no B reaction on A" : "") +
              (settings_.friction_compensation_enabled ? " with both-axis friction compensation" :
              "; friction compensation disabled");
            desired_ = reference_targets(false);
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
      double angle_offset = 0;
      if (angle_offset_.active()) {
        if (!disturbance_permitted(time)) {
          throw std::runtime_error("Angle-offset feedback/session degraded; stopping both motors before clearing the offset");
        }
        if (angle_offset_.expired(time)) {
          begin_stop("Angle-offset hold completed; stopping both motors before clearing the offset", false);
          return;
        }
        angle_offset = angle_offset_.update(time);
      }
      // Cancellation is permanent for this pulse, including recovery with a nonzero gain scale.
      const auto disturbance = disturbance_.update(time, disturbance_permitted(time));
      if (updated) {++pair_count_;}
      if (recovery_.quality != previous_quality) {
        RCLCPP_INFO(get_logger(), "Bilateral feedback quality %u; recovery scale %.3f",
          static_cast<unsigned>(recovery_.quality), recovery_.scale);
      }
      if (recovery_.scale == 0) {
        reset_compensation();
        auto neutral = desired_;
        for (size_t i = 0; i < 2; ++i) {neutral[i] = neutral_mit_target(limits_[i]);}
        output_ = symmetric_targets(neutral, latest(), limits_, settings_, 0);
      } else {
        desired_ = reference_targets(true);
        auto desired = apply_angle_offset(desired_, angle_offset, settings_);
        for (size_t i = 0; i < axes_.size(); ++i) {
          desired[i].t = 0;
          compensation_clamped_[i] = false;
          if (settings_.friction_compensation_enabled && settings_.friction_compensation_fraction > 0) {
            if (!compensators_[i]) {throw std::runtime_error(axes_[i].name + ": friction model unavailable during control");}
            double cap = std::min(settings_.friction_torque_limit,
              dm_motor::resolve_limits(limits_[i].mapping, limits_[i].command).t);
            if (settings_.max_torque > 0) {cap = std::min(cap, 0.5 * settings_.max_torque);}
            const double velocity = control_velocity(i, latest_[i]->velocity);
            compensation_clamped_[i] = velocity < compensators_[i]->velocity_min() || velocity > compensators_[i]->velocity_max();
            desired[i].t = compensators_[i]->step(velocity, dt, cap, settings_.friction_compensation_fraction,
              settings_.extra_viscous[i], settings_.extra_viscous_velocity_limit);
          }
        }
        output_ = symmetric_targets(desired, latest(), limits_, settings_, recovery_.scale, disturbance,
          spring_positions(time));
      }
      send_targets(output_.targets);
    } catch (const std::exception & error) {
      if (phase_ == State::STOPPING) {RCLCPP_ERROR(get_logger(), "Bilateral STOP retry: %s", error.what());}
      else {begin_stop(error.what(), true);}
    }
  }

  void publish_state()
  {
    const double time = dm_motor::steady_seconds();
    // Driver identity can finish initializing after the first disabled state,
    // and calibration can save a new model while this controller stays idle.
    if (!active() && phase_ != State::STOPPING && (!models_checked_ || time - last_models_check_ >= 1.0) &&
      disabled(axes_[0]) && disabled(axes_[1])) {load_compensation();}
    if (!active() && phase_ != State::STOPPING) {
      for (size_t i = 0; i < axes_.size(); ++i) {
        if (disabled(axes_[i]) && time - preview_sent_[i] >= 1.0 && axes_[i].control->get_subscription_count()) {
          Request request;
          request.header.stamp = now(); request.client_id = client_; request.request_id = ++request_id_;
          request.valid_for = 0.5; request.kind = Request::CAPABILITIES;
          preview_requests_[i] = request.request_id; preview_sent_[i] = time;
          axes_[i].control->publish(request);  // Read-only driver configuration; no session or enable.
        }
      }
    }
    if ((phase_ == State::IDLE || phase_ == State::FAULT) && latest_[0] && latest_[1] &&
      disabled(axes_[0]) && disabled(axes_[1]))
    {
      pairs_.select(time, settings_.feedback_age, settings_.pair_skew);
    }
    State state;
    state.header.stamp = now(); state.phase = phase_; state.quality = static_cast<uint8_t>(recovery_.quality);
    state.message = message_; state.motor_a = axes_[0].name; state.motor_b = axes_[1].name;
    state.controller_source = client_;
    state.disturbance_remaining = disturbance_.remaining(time);
    state.disturbance_active = state.disturbance_remaining > 0 && disturbance_permitted(time);
    if (!state.disturbance_active) {state.disturbance_remaining = 0;}
    state.disturbance_axis = disturbance_.axis(); state.disturbance_id = disturbance_.id();
    state.disturbance_torque = state.disturbance_active ? disturbance_.torque() : 0;
    state.disturbance_torque_a = output_.disturbance_torque[0];
    state.disturbance_torque_b = output_.disturbance_torque[1];
    state.angle_offset_active = angle_offset_.active();
    state.angle_offset_target = angle_offset_.target(); state.angle_offset_applied = angle_offset_.applied();
    state.angle_offset_ramping = angle_offset_.ramping(); state.angle_offset_id = angle_offset_.id();
    state.angle_offset_remaining = angle_offset_.remaining(time);
    state.direction = settings_.direction; state.motion_limits_enabled = settings_.motion_limits_enabled;
    state.system_parameters_file = system_parameters_file_;
    state.compensation_active = phase_ == State::ACTIVE && settings_.friction_compensation_enabled &&
      settings_.friction_compensation_fraction > 0 && recovery_.scale > 0;
    state.compensation_ready_a = compensators_[0].has_value(); state.compensation_ready_b = compensators_[1].has_value();
    state.compensation_source_a = compensation_sources_[0]; state.compensation_source_b = compensation_sources_[1];
    state.compensation_message_a = compensation_messages_[0]; state.compensation_message_b = compensation_messages_[1];
    if (compensators_[0]) {
      state.compensation_model_a = compensators_[0]->model();
      state.compensation_velocity_min_a = compensators_[0]->velocity_min();
      state.compensation_velocity_max_a = compensators_[0]->velocity_max();
    }
    if (compensators_[1]) {
      state.compensation_model_b = compensators_[1]->model();
      state.compensation_velocity_min_b = compensators_[1]->velocity_min();
      state.compensation_velocity_max_b = compensators_[1]->velocity_max();
    }
    state.compensation_clamped_a = compensation_clamped_[0]; state.compensation_clamped_b = compensation_clamped_[1];
    state.friction_torque_a = output_.friction_torque[0]; state.friction_torque_b = output_.friction_torque[1];
    state.quantization_torque_a = output_.quantization_torque[0]; state.quantization_torque_b = output_.quantization_torque[1];
    state.application_torque_limited = settings_.max_torque > 0;
    state.protocol_saturated_a = output_.protocol_saturated[0]; state.protocol_saturated_b = output_.protocol_saturated[1];
    for (size_t i = 0; i < axes_.size(); ++i) {
      const bool known = driver_configurations_[i].has_value() &&
        dm_motor::motor_state_fresh(axes_[i], now(), settings_.feedback_stop) &&
        (active() || time - configuration_received_[i] < 2.5);
      (i == 0 ? state.driver_configuration_known_a : state.driver_configuration_known_b) = known;
      if (driver_configurations_[i]) {
        (i == 0 ? state.driver_configuration_a : state.driver_configuration_b) = *driver_configurations_[i];
      }
    }
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
    state.virtual_position_error = state.position_error + state.angle_offset_applied;
    state.pair_count = pair_count_; state.degradation_count = recovery_.degradation_count;
    state.recovery_pairs = recovery_.fresh_pairs; state.recovery_scale = recovery_.scale;
    state.gain_scale = output_.gain_scale; state.decoded_kp = output_.decoded_kp; state.decoded_kd = output_.decoded_kd;
    state.estimated_torque_a = output_.estimated_torque[0]; state.estimated_torque_b = output_.estimated_torque[1];
    state.raw_torque_a = output_.raw_torque[0]; state.raw_torque_b = output_.raw_torque[1];
    for (size_t i = 0; i < 2; ++i) {
      auto & target = i == 0 ? state.target_a : state.target_b;
      const auto & command = output_.targets[i];
      target.mode = 1; target.position = command.p; target.velocity = command.v;
      target.kp = command.kp; target.kd = command.kd; target.torque = command.t;
    }
    state.command_count = command_count_;
    state.control_mode = control_mode_;
    state.required_velocity_bandwidth = required_velocity_bandwidth_;
    state.velocity_bandwidth_verified = bandwidth_verified_;
    state.motor_a_velocity_bandwidth = bandwidth_checks_[0].value();
    state.motor_b_velocity_bandwidth = bandwidth_checks_[1].value();
    publisher_->publish(state);
  }

  std::array<Axis, 2> axes_;
  std::array<std::optional<Sample>, 2> latest_;
  std::array<PeerVelocityFilter, 2> peer_velocity_filters_;
  std::array<AxisLimits, 2> limits_;
  std::array<Motor, 2> start_identities_;
  std::array<std::optional<Configuration>, 2> driver_configurations_;
  std::array<double, 2> configuration_received_{}, preview_sent_{};
  std::array<uint64_t, 2> preview_requests_{};
  std::array<VelocityBandwidthCheck, 2> bandwidth_checks_;
  std::array<uint64_t, 2> bandwidth_post_counts_{};
  double required_velocity_bandwidth_{0}, bandwidth_post_read_{0}, bandwidth_poll_after_{0};
  bool bandwidth_verified_{false};
  std::array<std::optional<dm_motor::FrictionCompensator>, 2> compensators_;
  std::array<std::string, 2> compensation_sources_, compensation_messages_;
  std::array<bool, 2> compensation_clamped_{};
  std::string system_parameters_file_, control_mode_;
  bool models_checked_{false};
  double last_models_check_{0};
  std::array<rclcpp::Subscription<Feedback>::SharedPtr, 2> feedback_;
  FeedbackPairs pairs_;
  Recovery recovery_;
  DisturbancePulse disturbance_;
  AngleOffset angle_offset_;
  int64_t last_disturbance_stamp_{0};
  int64_t last_angle_offset_stamp_{0};
  Settings settings_;
  Output output_;
  std::array<dm_motor::Command, 2> desired_;
  std::string client_, message_{"Idle; explicit start required"}, operator_session_;
  uint8_t phase_{State::IDLE};
  uint64_t request_id_{0}, command_count_{0}, pair_count_{0};
  double rate_{0}, validity_{0}, operator_timeout_{0}, heartbeat_{0}, last_tick_{0}, phase_since_{0};
  double enabled_pair_after_{0}, last_stop_{0};
  bool fault_stop_{false}, stop_warned_{false}, pair_waiting_{false};
  bool allow_differential_hardware_{false};
  rclcpp::Publisher<State>::SharedPtr publisher_;
  rclcpp::Service<Trigger>::SharedPtr start_, stop_;
  rclcpp::Service<Control>::SharedPtr operator_;
  rclcpp::Service<Disturbance>::SharedPtr disturbance_service_;
  rclcpp::Service<AngleOffsetService>::SharedPtr angle_offset_service_;
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
  std::signal(SIGPIPE, SIG_IGN);
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
