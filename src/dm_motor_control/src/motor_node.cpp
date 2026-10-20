#include "dm_motor_control/runtime.hpp"
#include "dm_motor_control/can_transport.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <dm_motor_interfaces/msg/motor_request.hpp>
#include <dm_motor_interfaces/msg/motor_reply.hpp>
#include <dm_motor_interfaces/msg/motor_task_event.hpp>
#include <dm_motor_interfaces/msg/motor_command.hpp>
#include <dm_motor_interfaces/msg/motor_setpoint.hpp>
#include <dm_motor_interfaces/msg/motor_feedback.hpp>
#include <condition_variable>
#include <deque>
#include <dm_motor_interfaces/msg/motor_state.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <variant>

namespace dm_motor
{
namespace
{
namespace api = dm_motor_interfaces;
using Request = api::msg::MotorRequest;
using Reply = api::msg::MotorReply;
struct Task
{
  using Goal = api::msg::MotorTask;
  using Feedback = api::msg::MotorTaskFeedback;
  using Result = api::msg::MotorTaskResult;
};
struct GoalHandle
{
  std::shared_ptr<const Task::Goal> goal;
  std::string client_id;
  uint64_t task_id{0};
  bool cancel_requested{false};
  std::function<void(api::msg::MotorTaskEvent)> emit;
  auto get_goal() const {return goal;}
  bool is_canceling() const {return cancel_requested;}
  void publish_feedback(const std::shared_ptr<Task::Feedback> & feedback)
  {
    api::msg::MotorTaskEvent event;
    event.state = event.RUNNING; event.feedback = *feedback; send(std::move(event));
  }
  void finish(uint8_t state, const std::shared_ptr<Task::Result> & result)
  {
    api::msg::MotorTaskEvent event;
    event.state = state; event.result = *result; send(std::move(event));
  }
  void succeed(const std::shared_ptr<Task::Result> & result) {finish(api::msg::MotorTaskEvent::SUCCEEDED, result);}
  void abort(const std::shared_ptr<Task::Result> & result) {finish(api::msg::MotorTaskEvent::FAILED, result);}
  void canceled(const std::shared_ptr<Task::Result> & result) {finish(api::msg::MotorTaskEvent::CANCELED, result);}
  void send(api::msg::MotorTaskEvent event)
  {
    event.client_id = client_id; event.task_id = task_id; emit(std::move(event));
  }
};
using FeedbackPublication = std::pair<rclcpp::Publisher<api::msg::MotorFeedback>::SharedPtr,
  api::msg::MotorFeedback>;
using TaskPublication = std::pair<rclcpp::Publisher<api::msg::MotorTaskEvent>::SharedPtr,
  api::msg::MotorTaskEvent>;
using ReplyPublication = std::pair<rclcpp::Publisher<Reply>::SharedPtr, Reply>;
using TelemetryPublication = std::variant<FeedbackPublication, TaskPublication, ReplyPublication>;
struct CachedControl
{
  Request request;
  Reply reply;
};
struct InputTarget
{
  api::msg::MotorSetpoint::ConstSharedPtr message;
  std::string source;
};
struct SourceOrder
{
  int64_t stamp{0};
  uint64_t sequence{0};
  double received{0};
};
struct Keepalive
{
  double deadline{0};
};
std::string publisher_key(const rclcpp::MessageInfo & info)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (auto byte : info.get_rmw_message_info().publisher_gid.data) {
    stream << std::setw(2) << static_cast<unsigned>(byte);
  }
  return stream.str();
}
constexpr double pi = 3.14159265358979323846;

void finite(double value, const char * name)
{
  if (!std::isfinite(value)) {throw std::invalid_argument(std::string(name) + " must be finite");}
}
void positive(double value, const char * name)
{
  finite(value, name);
  if (value <= 0) {throw std::invalid_argument(std::string(name) + " must be positive");}
}
Mode mode(uint8_t value)
{
  if (value < 1 || value > 4) {throw std::invalid_argument("mode must be 1..4");}
  return static_cast<Mode>(value);
}
bool identifier(const std::string & value)
{
  if (value.empty() || value.size() > 64 ||
    !((value[0] >= 'a' && value[0] <= 'z') || value[0] == '_'))
  {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
           (c >= '0' && c <= '9') || c == '_';
  });
}
std::string new_token()
{
  std::random_device random;
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (int i = 0; i < 8; ++i) {stream << std::setw(8) << random();}
  return stream.str();
}
Target from_message(const api::msg::MotorTarget & msg)
{
  return {mode(msg.mode), msg.position, msg.velocity, msg.kp, msg.kd,
    msg.torque, msg.velocity_limit, msg.current_ratio};
}
api::msg::MotorTarget to_message(const Target & target)
{
  api::msg::MotorTarget msg;
  msg.mode = static_cast<uint8_t>(target.mode);
  msg.position = target.position; msg.velocity = target.velocity;
  msg.kp = target.kp; msg.kd = target.kd; msg.torque = target.torque;
  msg.velocity_limit = target.velocity_limit; msg.current_ratio = target.current_ratio;
  return msg;
}
struct Axis
{
  MotorConfig config;
  Bus * bus{nullptr};
  CanTransport * transport{nullptr};
  std::string endpoint, profile, backend, fault_group;
  double direction{1}, offset{0};
  double lower{-12.5}, upper{12.5};
  bool explicit_bounds{false};
  bool host_protection{false};
  double measured_velocity{0}, measured_torque{0}, estimated_torque{0};
  double mos_stop{0}, rotor_stop{0};
  int violations{0}, violation_limit{3};
  std::map<std::string, Keepalive> keepalives;
  std::map<std::string, SourceOrder> source_order;
  std::string active_session, active_source;
  uint64_t sequence{0}, rejected{0};
  uint64_t applied_sequence{0};
  double accepted_target_age_ms{0}, applied_target_age_ms{0};
  std::shared_ptr<const InputTarget> latest_target;
  double target_deadline{0};
  bool streamed_target{false};
  bool held_target{false};
  int64_t stamp{0};
  Target requested, active, applied;
  bool applied_target_active{false};
  std::shared_ptr<GoalHandle> task;
  double task_start{0}, task_duration{0}, task_start_position{0};
  double settled_since{-1}, task_feedback_time{0};
  bool task_stopping{false}, task_success{false};
  std::string task_message;
  double feedback_window{0}, feedback_rate{0};
  uint64_t window_count{0};
  uint64_t protection_count{0};
  rclcpp::Publisher<api::msg::MotorState>::SharedPtr publisher;
  rclcpp::Publisher<api::msg::MotorFeedback>::SharedPtr feedback_publisher;
  rclcpp::Subscription<api::msg::MotorSetpoint>::SharedPtr setpoint;
  rclcpp::Subscription<api::msg::MotorSetpoint>::SharedPtr target_subscription;
  rclcpp::Subscription<api::msg::MotorCommand>::SharedPtr legacy;
  rclcpp::Subscription<Request>::SharedPtr control_subscription;
  rclcpp::Publisher<Reply>::SharedPtr reply_publisher;
  rclcpp::Publisher<api::msg::MotorTaskEvent>::SharedPtr task_publisher;
  std::map<uint8_t, std::function<void(Request::SharedPtr, Reply::SharedPtr)>> handlers;
  std::map<std::string, CachedControl> command_history;
};
}

class MotorNode final : public rclcpp::Node
{
public:
  MotorNode() : Node("motor_controller", rclcpp::NodeOptions().start_parameter_services(false))
  {
    rate_ = declare_parameter<double>("control_rate", 200.0);
    state_rate_limit_ = declare_parameter<double>("state_rate", 0.0);
    state_rate_ = state_rate_limit_;
    if (state_rate_ == 0) {state_rate_ = rate_;}
    finite(rate_, "control_rate"); finite(state_rate_, "state_rate");
    if (rate_ < 20 || rate_ > 1000 || state_rate_ < 1 || state_rate_ > rate_) {
      throw std::invalid_argument("control_rate must be 20..1000; state_rate 1..control_rate or 0");
    }
    max_task_duration_ = declare_parameter<double>("max_task_duration", 60.0);
    positive(max_task_duration_, "max_task_duration");
    if (max_task_duration_ > 3600) {throw std::invalid_argument("max_task_duration must be <=3600");}
    if (!declare_parameter<std::vector<std::string>>(
        "motor_names", std::vector<std::string>{}).empty())
    {
      throw std::invalid_argument("Each motor_node controls exactly one motor; use bus.launch.py for multiple nodes");
    }
    configure_single();
    target_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::ExecutorOptions target_options;
    target_options.context = get_node_base_interface()->get_context();
    target_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(target_options);
    target_executor_->add_callback_group(target_callback_group_, get_node_base_interface());
    const auto state_qos = rclcpp::QoS(1).reliable().durability_volatile();
    for (auto & axis : axes_) {create_axis_interfaces(*axis, state_qos);}
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", state_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
    parameters_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() != "control_rate" ||
            parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            !std::isfinite(parameter.as_double()) ||
            parameter.as_double() < 20 || parameter.as_double() > 1000)
          {
            result.reason = "Only control_rate may change at runtime; requires a double in 20..1000 Hz.";
            return result;
          }
        }
        std::lock_guard<std::mutex> guard(mutex_);
        for (const auto & parameter : parameters) {
          for (const auto & axis : axes_) {
            for (const double timeout : {axis->config.command_timeout, axis->config.feedback_timeout,
              axis->config.watchdog})
            {
              if (timeout < 2.0 / parameter.as_double()) {
                result.reason = "Configured command/feedback/watchdog timeouts must allow at least two control periods.";
                return result;
              }
            }
          }
        }
        for (const auto & parameter : parameters) {
          const double requested = parameter.as_double();
          const double state_rate = state_rate_limit_ == 0 ? requested : std::min(state_rate_limit_, requested);
          auto timer = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / state_rate)), [this] {publish_state();});
          state_timer_->cancel();
          state_timer_ = std::move(timer);
          rate_ = requested; state_rate_ = state_rate;
          last_tick_ = 0; period_ = 0; max_period_ = 0;
          period_count_ = 0; period_index_ = 0; percentiles_.fill(0);
          measured_rate_ = 0; rate_window_started_ = 0;
          RCLCPP_INFO(get_logger(), "Control frequency changed to %.1f Hz", rate_);
        }
        result.successful = true;
        return result;
      });
    state_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / state_rate_)), [this] {publish_state();});
    diagnostics_timer_ = create_wall_timer(std::chrono::seconds(1), [this] {publish_diagnostics();});
    running_ = true;
    try {
      feedback_thread_ = std::thread([this] {feedback_loop();});
      target_thread_ = std::thread([this] {
        try {target_executor_->spin();}
        catch (const std::exception & error) {
          if (running_ && rclcpp::ok(get_node_base_interface()->get_context())) {
            std::lock_guard<std::mutex> guard(mutex_);
            for (auto & bus : buses_) {
              bus->transport_failed(std::string("Target input executor failed; relaunch required: ") + error.what());
            }
            RCLCPP_ERROR(get_logger(), "Target topic executor failed: %s", error.what());
          }
        }
      });
      control_thread_ = std::thread([this] {control_loop();});
    } catch (...) {
      running_ = false;
      target_executor_->cancel();
      feedback_condition_.notify_all();
      if (target_thread_.joinable()) {target_thread_.join();}
      if (feedback_thread_.joinable()) {feedback_thread_.join();}
      if (control_thread_.joinable()) {control_thread_.join();}
      throw;
    }
    RCLCPP_INFO(get_logger(), "Independent motor node, remote CAN transport, %.1f Hz; explicit enable required", rate_);
  }

  ~MotorNode() override {stop();}

  void stop()
  {
    if (!running_.exchange(false)) {return;}
    target_executor_->cancel();
    feedback_condition_.notify_all();
    if (target_thread_.joinable()) {target_thread_.join();}
    if (control_thread_.joinable()) {control_thread_.join();}
    if (feedback_thread_.joinable()) {feedback_thread_.join();}
    std::lock_guard<std::mutex> guard(mutex_);
    for (auto & axis : axes_) {
      end_task(*axis, false, "Driver shutting down");
      try {axis->bus->disable(axis->config.name, steady_seconds());}
      catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Shutdown STOP %s failed: %s",
          axis->config.name.c_str(), error.what());
      }
    }
    const double deadline = steady_seconds() + 1.0;
    bool disabled = false;
    while (steady_seconds() < deadline && axes_.front()->transport->ready()) {
      for (auto & bus : buses_) {
        try {bus->tick(steady_seconds());}
        catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Shutdown I/O failed: %s", error.what());
        }
      }
      disabled = std::all_of(axes_.begin(), axes_.end(), [](const auto & axis) {
        auto state = axis->bus->snapshot(axis->config.name, steady_seconds());
        return state.feedback_age >= 0 && state.feedback_age < axis->config.feedback_timeout &&
               state.feedback.status == 0;
      });
      if (disabled) {break;}
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (!disabled) {
      RCLCPP_ERROR(get_logger(), "Shutdown disable unconfirmed; hardware watchdog/estop required");
    }
  }

private:
  template<typename T>
  T parameter(const std::string & prefix, const std::string & name, const T & value)
  {
    return declare_parameter<T>(prefix + name, value);
  }

  MotorConfig motor_config(const std::string & prefix, const std::string & name)
  {
    MotorConfig result;
    result.name = name;
    const int id = parameter<int>(prefix, "motor_id", 1);
    const int master = parameter<int>(prefix, "master_id", 0);
    if (id < 1 || id > 15 || master < 0 || master > 0x7ff) {
      throw std::invalid_argument("motor_id must be 1..15; master_id must be 0..2047");
    }
    result.id = static_cast<uint8_t>(id); result.master = static_cast<uint16_t>(master);
    const int desired_mode = parameter<int>(prefix, "mode", 1);
    if (desired_mode < 1 || desired_mode > 4) {throw std::invalid_argument("mode must be 1..4");}
    result.mode = mode(static_cast<uint8_t>(desired_mode));
    result.configure_mode_on_startup = parameter<bool>(prefix, "configure_mode_on_startup", false);
    result.command_timeout = parameter<double>(prefix, "command_timeout", 0.25);
    result.feedback_timeout = parameter<double>(prefix, "feedback_timeout", 0.1);
    result.watchdog = parameter<double>(prefix, "watchdog_timeout", 0.2);
    for (const double timeout : {result.command_timeout, result.feedback_timeout, result.watchdog}) {
      if (!std::isfinite(timeout) || timeout < 2.0 / rate_ || timeout > 2.0) {
        throw std::invalid_argument("timeouts must be >= two periods and <=2 seconds");
      }
    }
    result.limits = {parameter<double>(prefix, "position_limit", 0.0),
      parameter<double>(prefix, "velocity_limit", 0.0),
      parameter<double>(prefix, "torque_limit", 0.0),
      parameter<double>(prefix, "kp_limit", 500.0), parameter<double>(prefix, "kd_limit", max_kd)};
    return result;
  }

  std::unique_ptr<Axis> axis_config(
    const std::string & prefix, MotorConfig config, const std::string & endpoint)
  {
    auto axis = std::make_unique<Axis>();
    axis->config = std::move(config);
    axis->endpoint = endpoint;
    axis->profile = parameter<std::string>(prefix, "profile", "j4310_v12");
    axis->fault_group = parameter<std::string>(prefix, "fault_group", "");
    if (!axis->fault_group.empty() && !identifier(axis->fault_group)) {
      throw std::invalid_argument("fault_group must be empty or a ROS identifier");
    }
    if (axis->profile != "j4310_v12" && axis->profile != "j4310_v11") {
      throw std::invalid_argument("Unverified profile: only j4310_v11/j4310_v12 are supported");
    }
    axis->direction = parameter<double>(prefix, "direction", 1.0);
    axis->host_protection = parameter<bool>(prefix, "host_protection_enabled", false);
    if (axis->direction != -1 && axis->direction != 1) {
      throw std::invalid_argument("direction must be -1 or 1");
    }
    axis->offset = parameter<double>(prefix, "software_offset", 0.0);
    finite(axis->offset, "software_offset");
    const double unset = std::numeric_limits<double>::max();
    axis->lower = parameter<double>(prefix, "position_min", -unset);
    axis->upper = parameter<double>(prefix, "position_max", unset);
    finite(axis->lower, "position_min"); finite(axis->upper, "position_max");
    axis->explicit_bounds = axis->lower != -unset || axis->upper != unset;
    if (axis->lower >= axis->upper) {throw std::invalid_argument("position_min must be < position_max");}
    axis->measured_velocity = parameter<double>(prefix, "measured_velocity_limit", 0.0);
    axis->measured_torque = parameter<double>(prefix, "measured_torque_limit", 0.0);
    axis->estimated_torque = parameter<double>(prefix, "estimated_torque_limit", 0.0);
    axis->mos_stop = parameter<double>(prefix, "mos_temperature_stop", 0.0);
    axis->rotor_stop = parameter<double>(prefix, "rotor_temperature_stop", 0.0);
    for (double value : {axis->measured_velocity, axis->measured_torque, axis->estimated_torque,
        axis->mos_stop, axis->rotor_stop})
    {
      finite(value, "feedback protection");
      if (value < 0) {throw std::invalid_argument("feedback protection must be >=0 (0 disables)");}
    }
    axis->violation_limit = parameter<int>(prefix, "protection_samples", 3);
    if (axis->violation_limit < 1 || axis->violation_limit > 100) {
      throw std::invalid_argument("protection_samples must be 1..100");
    }
    parameter<bool>(prefix, "legacy_control", false);  // Accepted for old configurations; command input is now always open.
    axis->requested.mode = axis->config.mode;
    axis->active.mode = axis->config.mode;
    axis->applied.mode = axis->config.mode;
    return axis;
  }

  void configure_single()
  {
    const auto name = parameter<std::string>("", "motor_name", "motor");
    if (!identifier(name)) {throw std::invalid_argument("motor_name must be a ROS identifier");}
    auto config = motor_config("", name);
    if (config.configure_mode_on_startup) {
      RCLCPP_INFO(get_logger(), "Startup mode configuration enabled: desired mode %u; "
        "fresh disabled feedback and independent RAM readback required; no Flash save",
        static_cast<unsigned>(config.mode));
    }
    auto axis = axis_config("", config, "");
    const auto bus_namespace = parameter<std::string>("", "can_bus_namespace", "/can/main");
    if (bus_namespace.empty() || bus_namespace.front() != '/') {
      throw std::invalid_argument("can_bus_namespace must be an absolute ROS namespace");
    }
    const bool fd = parameter<bool>("", "can_fd", false);
    const auto client_name = std::string(get_fully_qualified_name());
    auto remote = std::make_unique<CanTransport>(*this, bus_namespace, client_name,
      config.id, config.master, fd, rate_, config.feedback_timeout, axis->fault_group);
    axis->transport = remote.get();
    auto bus = std::make_unique<Bus>(std::move(remote), std::vector<MotorConfig>{config}, false);
    axis->bus = bus.get(); axis->backend = "can_bus";
    axes_.push_back(std::move(axis)); buses_.push_back(std::move(bus));
  }

  double session_deadline(const Axis & axis, const std::string & token) const
  {
    const auto found = axis.keepalives.find(token);
    return found == axis.keepalives.end() ? 0 : found->second.deadline;
  }

  void check_order(Axis & axis, const std::string & source, int64_t stamp, uint64_t sequence, double time)
  {
    for (auto it = axis.source_order.begin(); it != axis.source_order.end();) {
      if (time - it->second.received > 5.0) {it = axis.source_order.erase(it);}
      else {++it;}
    }
    const auto previous = axis.source_order.find(source);
    if (stamp <= axis.stamp || (previous != axis.source_order.end() &&
      (stamp <= previous->second.stamp || (sequence && sequence <= previous->second.sequence))))
    {
      throw std::invalid_argument("stale or replayed target from this publisher");
    }
    if (previous == axis.source_order.end() && axis.source_order.size() >= 256) {
      throw std::runtime_error("target publisher history capacity reached");
    }
  }

  void require_ready(const Axis & axis) const
  {
    const auto state = axis.bus->snapshot(axis.config.name, steady_seconds());
    if (!axis.transport->ready() || !axis.bus->ready() || !state.absolute_position_valid) {
      throw std::runtime_error("bus/absolute position not ready: " + axis.transport->error() +
        axis.bus->initialization_error() + state.error);
    }
  }

  void reset_targets(Axis & axis)
  {
    axis.sequence = 0;
    axis.applied_sequence = 0;
    axis.accepted_target_age_ms = 0; axis.applied_target_age_ms = 0;
    std::atomic_store(&axis.latest_target, std::shared_ptr<const InputTarget>{});
    axis.streamed_target = false;
    axis.held_target = false;
    axis.applied_target_active = false;
    axis.stamp = now().nanoseconds();
    axis.requested = {}; axis.active = {}; axis.applied = {};
    axis.requested.mode = axis.bus->snapshot(axis.config.name, steady_seconds()).mode;
    axis.active.mode = axis.requested.mode;
    axis.applied.mode = axis.requested.mode;
    axis.source_order.clear();
    axis.active_session.clear(); axis.active_source.clear();
  }

  void end_task(Axis & axis, bool success, const std::string & reason)
  {
    if (!axis.task) {return;}
    auto result = std::make_shared<Task::Result>();
    result->success = success; result->message = reason;
    result->final_position = joint_position(axis, axis.bus->snapshot(axis.config.name, steady_seconds()));
    if (axis.task->is_canceling()) {axis.task->canceled(result);}
    else if (success) {axis.task->succeed(result);}
    else {axis.task->abort(result);}
    axis.task.reset(); axis.task_stopping = false;
  }

  void stop_axis(Axis & axis, const std::string & reason)
  {
    end_task(axis, false, reason);
    reset_targets(axis);
    axis.bus->disable(axis.config.name, steady_seconds());
  }

  double joint_position(const Axis & axis, const MotorSnapshot & state) const
  {
    return axis.direction * state.absolute_position + axis.offset;
  }

  std::pair<double, double> joint_bounds(const Axis & axis, const MotorSnapshot & state) const
  {
    if (axis.explicit_bounds) {return {axis.lower, axis.upper};}
    const double limit = position_command_limit(state.mode, state.mapping, axis.config.limits);
    return {axis.offset - limit, axis.offset + limit};
  }

  Target native_target(const Axis & axis, Target target) const
  {
    if (target.mode != Mode::Velocity) {
      target.position = axis.direction * (target.position - axis.offset);
    }
    target.velocity *= axis.direction;
    target.torque *= axis.direction;
    return target;
  }

  void check_target(const Axis & axis, const Target & target, const MotorSnapshot & state) const
  {
    if (target.mode != state.mode) {throw std::invalid_argument("target mode does not match confirmed mode");}
    for (const auto & item : std::array<std::pair<const char *, double>, 7>{{
        {"position", target.position}, {"velocity", target.velocity}, {"kp", target.kp},
        {"kd", target.kd}, {"torque", target.torque},
        {"velocity_limit", target.velocity_limit}, {"current_ratio", target.current_ratio}}})
    {
      if (!std::isfinite(item.second)) {
        throw std::invalid_argument(std::string(item.first) + " outside configured range: must be finite");
      }
    }
    const auto bounds = joint_bounds(axis, state);
    if (target.mode != Mode::Velocity && (target.position < bounds.first || target.position > bounds.second)) {
      throw std::invalid_argument("position outside configured joint bounds");
    }
    if (target.mode == Mode::Mit) {
      if (target.velocity_limit != 0 || target.current_ratio != 0) {
        throw std::invalid_argument("MIT does not accept velocity_limit/current_ratio");
      }
      const auto native = native_target(axis, target);
      validate_command({native.position, native.velocity, native.kp, native.kd, native.torque},
        state.mapping, state.limits);
    } else {
      if (target.kp != 0 || target.kd != 0 || target.torque != 0 ||
        (target.mode != Mode::Velocity && target.velocity != 0))
      {
        throw std::invalid_argument("fields not applicable to selected mode must be zero");
      }
      if (target.mode == Mode::Velocity) {
        if (target.position != 0 || target.velocity_limit != 0 || target.current_ratio != 0 ||
          std::abs(target.velocity) > state.limits.v)
        {
          throw std::invalid_argument("invalid velocity-mode fields or velocity outside configured range");
        }
      } else {
        const auto native = native_target(axis, target);
        if (std::abs(native.position) > position_command_limit(state.mode, state.mapping, axis.config.limits) ||
          target.velocity_limit < 0 ||
          target.velocity_limit > state.limits.v)
        {
          throw std::invalid_argument("position or velocity_limit outside configured range");
        }
        if (target.mode == Mode::PositionVelocity && target.current_ratio != 0) {
          throw std::invalid_argument("position velocity mode does not accept current_ratio");
        }
        if (target.mode == Mode::PositionForce &&
          (target.current_ratio < 0 || target.current_ratio > 1 || target.velocity_limit > 100))
        {
          throw std::invalid_argument("position force requires current_ratio 0..1 and velocity_limit <=100");
        }
      }
    }
  }

  void submit(Axis & axis, const Target & target, double time)
  {
    const auto state = axis.bus->snapshot(axis.config.name, time);
    check_target(axis, target, state);
    const auto native = native_target(axis, target);
    if (axis.host_protection && native.mode == Mode::Mit && axis.estimated_torque > 0) {
      const double estimate = native.kp * (native.position - state.absolute_position) +
        native.kd * (native.velocity - state.feedback.v) + native.torque;
      if (std::abs(estimate) > axis.estimated_torque) {
        throw std::invalid_argument("estimated combined MIT torque exceeds configured limit");
      }
    }
    axis.bus->command(axis.config.name, native, time);
    axis.active = target;
  }

  template<typename Response>
  void rejection(Response & response, const std::exception & error)
  {
    response.success = false; response.message = error.what();
    RCLCPP_WARN(get_logger(), "Operation rejected: %s", error.what());
  }

  template<typename Message>
  void accept_target(Axis & axis, const Message & msg, const std::string & source, bool held, double time)
  {
    require_ready(axis);
    const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
    const double age = (now() - stamp).seconds();
    if (!std::isfinite(msg.valid_for) || msg.valid_for <= 0 ||
      msg.valid_for > axis.config.command_timeout || age > msg.valid_for || age < -0.05)
    {
      throw std::invalid_argument("stale, replayed or future target timestamp/sequence/validity");
    }
    check_order(axis, source, stamp.nanoseconds(), msg.sequence, time);
    if (held && session_deadline(axis, msg.token) <= time) {
      throw std::invalid_argument("held target requires a live non-exclusive keepalive session; use streaming targets without a token");
    }
    const auto target = from_message(msg.target);
    submit(axis, target, time);
    end_task(axis, false, "Superseded by a newer manual target");
    axis.source_order[source] = {stamp.nanoseconds(), msg.sequence, time};
    axis.requested = target; axis.sequence = msg.sequence;
    axis.active_source = msg.client_id.empty() ? source : msg.client_id;
    axis.active_session = held ? msg.token : "";
    axis.accepted_target_age_ms = age * 1000;
    axis.streamed_target = true; axis.held_target = held;
    axis.target_deadline = time + std::max(0.0, msg.valid_for - std::max(0.0, age));
  }

  void create_axis_interfaces(Axis & axis, const rclcpp::QoS & state_qos)
  {
    axis.publisher = create_publisher<api::msg::MotorState>(axis.endpoint + "state", state_qos);
    axis.feedback_publisher = create_publisher<api::msg::MotorFeedback>(
      axis.endpoint + "feedback", rclcpp::QoS(4096).reliable().durability_volatile());
    axis.bus->set_feedback_callback(
      [this, &axis](const std::string &, const MotorSnapshot & sample, double received) {
        api::msg::MotorFeedback msg;
        msg.motor_id = axis.config.id; msg.feedback_count = sample.feedback_count;
        msg.receive_time = received;
        msg.position = joint_position(axis, sample);
        msg.velocity = axis.direction * sample.feedback.v;
        msg.torque = axis.direction * sample.feedback.t;
        msg.native_position = sample.feedback.p; msg.absolute_position = sample.absolute_position;
        msg.absolute_position_valid = sample.absolute_position_valid; msg.position_wraps = sample.position_wraps;
        msg.mos_temperature = sample.feedback.mos; msg.rotor_temperature = sample.feedback.rotor;
        msg.target_active = sample.phase == Phase::Enabled && axis.applied_target_active;
        msg.target_position = axis.applied.position; msg.target_velocity = axis.applied.velocity;
        msg.target_torque = axis.applied.torque;
        msg.applied_target_sequence = axis.applied_sequence;
        msg.target_accept_latency_ms = axis.applied_target_age_ms;
        pending_feedback_.emplace_back(axis.feedback_publisher, std::move(msg));
      });
    const auto receive_target = [this, &axis](api::msg::MotorSetpoint::ConstSharedPtr msg,
        const rclcpp::MessageInfo & info) {
        std::shared_ptr<const InputTarget> input = std::make_shared<InputTarget>(
          InputTarget{std::move(msg), publisher_key(info)});
        const auto previous = std::atomic_exchange(&axis.latest_target, std::move(input));
        if (previous && previous->message->request_id) {
          target_reply(axis, *previous->message, false, "Superseded by a newer target before the control cycle.");
        }
      };
    rclcpp::SubscriptionOptions target_options;
    target_options.callback_group = target_callback_group_;
    axis.target_subscription = create_subscription<api::msg::MotorSetpoint>(
      axis.endpoint + "target", rclcpp::QoS(1).reliable().durability_volatile(), receive_target, target_options);
    const auto command_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    axis.setpoint = create_subscription<api::msg::MotorSetpoint>(
      axis.endpoint + "setpoint", command_qos, receive_target, target_options);
    axis.legacy = create_subscription<api::msg::MotorCommand>(axis.endpoint + "command", command_qos,
      [this, &axis](const api::msg::MotorCommand & msg, const rclcpp::MessageInfo & info) {
        std::lock_guard<std::mutex> guard(mutex_);
        try {
          require_ready(axis);
          const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
          const double age = (now() - stamp).seconds();
          if (age > axis.config.command_timeout || age < -0.05) {
            throw std::invalid_argument("stale, replayed or future command timestamp");
          }
          const auto source = publisher_key(info);
          const double time = steady_seconds();
          check_order(axis, source, stamp.nanoseconds(), 0, time);
          Target target{Mode::Mit, msg.position, msg.velocity, msg.kp, msg.kd, msg.torque, 0, 0};
          submit(axis, target, time);
          end_task(axis, false, "Superseded by a newer command");
          const auto previous = std::atomic_exchange(&axis.latest_target, std::shared_ptr<const InputTarget>{});
          if (previous && previous->message->request_id) {
            target_reply(axis, *previous->message, false, "Superseded by a newer command.");
          }
          axis.source_order[source] = {stamp.nanoseconds(), 0, time};
          axis.requested = target; axis.sequence = 0;
          axis.active_source = source; axis.active_session.clear(); axis.held_target = false;
          axis.streamed_target = true;
          axis.target_deadline = time + axis.config.command_timeout - std::max(0.0, age);
        } catch (const std::exception & error) {
          ++axis.rejected;
          axis.bus->fail(axis.config.name, error.what());
          RCLCPP_ERROR(get_logger(), "Legacy target rejected; STOP latched: %s", error.what());
        }
      });
    axis.handlers[Request::LEASE] =
      [this, &axis](const Request::SharedPtr req, Reply::SharedPtr response) {
        std::lock_guard<std::mutex> guard(mutex_);
        const double time = steady_seconds();
        try {
          require_ready(axis);
          if (req->operation == Request::ACQUIRE) {
            if (req->owner.empty() || req->owner.size() > 128) {throw std::invalid_argument("owner must be 1..128 characters");}
            if (!std::isfinite(req->duration) || req->duration < 0.3 || req->duration > 5) {
              throw std::invalid_argument("lease duration must be 0.3..5 seconds");
            }
            if (axis.keepalives.size() >= 256) {
              throw std::runtime_error("keepalive session capacity reached");
            }
            response->token = new_token();
            axis.keepalives[response->token] = {time + req->duration};
          } else {
            if (req->token.empty() || session_deadline(axis, req->token) <= time) {
              throw std::runtime_error("unknown or expired keepalive session");
            }
            if (req->operation == Request::RENEW) {
              if (!std::isfinite(req->duration) || req->duration < 0.3 || req->duration > 5) {
                throw std::invalid_argument("lease duration must be 0.3..5 seconds");
              }
              axis.keepalives.at(req->token).deadline = time + req->duration;
              response->token = req->token;
            } else if (req->operation == Request::RELEASE) {
              if (axis.active_session == req->token) {stop_axis(axis, "Active source keepalive released");}
              axis.keepalives.erase(req->token);
            } else {throw std::invalid_argument("unknown control lease operation");}
          }
          response->success = true;
          response->remaining = std::max(0.0, session_deadline(axis, response->token) - time);
          response->message = "Non-exclusive keepalive accepted; no control ownership acquired. Release stops only this session's active output.";
        } catch (const std::exception & error) {rejection(*response, error);}
      };
    axis.handlers[Request::OPERATION] =
      [this, &axis](const Request::SharedPtr req, Reply::SharedPtr response) {
        std::lock_guard<std::mutex> guard(mutex_);
        const double time = steady_seconds();
        try {
          if (req->operation == Request::DISABLE) {stop_axis(axis, "STOP requested");}
          else {
            require_ready(axis);
            if (axis.task) {throw std::runtime_error("cancel task before maintenance");}
            if (req->operation == Request::ENABLE) {
              reset_targets(axis); axis.bus->enable(axis.config.name, time);
              if (session_deadline(axis, req->token) > time) {axis.active_session = req->token;}
              axis.active_source = req->client_id;
            } else if (req->operation == Request::RESET_HOST_FAULT) {
              axis.bus->reset(axis.config.name, time); reset_targets(axis);
            } else if (req->operation == Request::SET_MODE) {
              response->operation_id = axis.bus->set_mode(axis.config.name, mode(req->mode), time);
              reset_targets(axis);
            } else if (req->operation == Request::CLEAR_MOTOR_FAULT) {
              response->operation_id = axis.bus->clear_motor_fault(axis.config.name, time);
            } else if (req->operation == Request::SET_DEVICE_ZERO) {
              response->operation_id = axis.bus->set_device_zero(axis.config.name, time);
              reset_targets(axis);
            } else if (req->operation == Request::SAVE_PARAMETERS) {
              response->operation_id = axis.bus->save_parameters(axis.config.name, time);
            } else if (req->operation == Request::SET_SOFTWARE_ZERO) {
              const auto state = axis.bus->snapshot(axis.config.name, time);
              finite(req->value, "software zero value");
              if (state.phase != Phase::Disabled || state.feedback.status != 0 ||
                state.feedback_age < 0 || state.feedback_age > axis.config.feedback_timeout ||
                std::abs(state.feedback.v) > 0.1)
              {
                throw std::runtime_error("software zero requires fresh disabled, near-zero-speed feedback");
              }
              const double change = req->value - joint_position(axis, state);
              axis.offset += change; axis.lower += change; axis.upper += change;
              reset_targets(axis);
            } else {throw std::invalid_argument("unknown motor operation");}
          }
          response->success = true;
          response->message = response->operation_id ?
            "Queued; poll get_operation for CONFIRMED/REJECTED/UNKNOWN." :
            "Request sent, not yet confirmed. Check state.phase/status.";
        } catch (const std::exception & error) {rejection(*response, error);}
      };
    axis.handlers[Request::REGISTERS] =
      [this, &axis](const Request::SharedPtr req, Reply::SharedPtr response) {
        std::lock_guard<std::mutex> guard(mutex_);
        try {
          require_ready(axis);
          const double time = steady_seconds();
          if (req->addresses.empty() || req->addresses.size() > 16 ||
            (req->write && req->raw_values.size() != req->addresses.size()) ||
            (!req->write && !req->raw_values.empty()))
          {
            throw std::invalid_argument("1..16 addresses required; writes require one raw word per address");
          }
          for (size_t i = 0; i < req->addresses.size(); ++i) {
            const auto & catalog = register_catalog();
            const auto item = std::find_if(catalog.begin(), catalog.end(), [&](const RegisterInfo & entry) {
              return entry.rid == req->addresses[i];
            });
            if (item == catalog.end()) {throw std::invalid_argument("unknown/unpublished register address");}
            if (!req->write) {continue;}
            if (!item->writable) {throw std::invalid_argument("register is read-only");}
            if (!raw_register_write_allowed(item->rid)) {
              throw std::invalid_argument("register requires specialized maintenance; raw write prohibited");
            }
            double value = req->raw_values[i];
            if (!item->integer) {
              float decoded;
              std::memcpy(&decoded, &req->raw_values[i], sizeof(decoded));
              value = decoded;
            }
            finite(value, "register value");
            if ((item->minimum && (value < *item->minimum ||
              (!item->minimum_inclusive && value == *item->minimum))) ||
              (item->maximum && (value > *item->maximum ||
              (!item->maximum_inclusive && value == *item->maximum))))
            {
              throw std::invalid_argument("register value outside published range");
            }
          }
          for (size_t i = 0; i < req->addresses.size(); ++i) {
            response->operation_ids.push_back(req->write ?
              axis.bus->write_register(axis.config.name, req->addresses[i], req->raw_values[i], time) :
              axis.bus->read_register(axis.config.name, req->addresses[i], time));
          }
          response->success = true;
          response->message = "Queued ordered non-atomic batch; poll every operation ID.";
        } catch (const std::exception & error) {
          rejection(*response, error);
          if (!response->operation_ids.empty()) {
            response->message += "; earlier items were queued; inspect returned operation IDs";
          }
        }
      };
    axis.handlers[Request::OPERATION_RESULT] =
      [this, &axis](const Request::SharedPtr req, Reply::SharedPtr response) {
        std::lock_guard<std::mutex> guard(mutex_);
        try {
          const auto result = axis.bus->operation(req->operation_id);
          if (result.motor != axis.config.name) {throw std::invalid_argument("operation belongs to another motor");}
          response->success = true; response->state = static_cast<uint8_t>(result.state);
          response->address = result.rid; response->raw_value = result.raw_value; response->message = result.message;
        } catch (const std::exception & error) {rejection(*response, error);}
      };
    axis.handlers[Request::CAPABILITIES] =
      [this, &axis](const Request::SharedPtr, Reply::SharedPtr response) {
        std::lock_guard<std::mutex> guard(mutex_);
        const auto state = axis.bus->snapshot(axis.config.name, steady_seconds());
        response->success = true;
        response->ready = axis.transport->ready() && axis.bus->ready() && state.absolute_position_valid;
        response->name = axis.config.name;
        auto & configuration = response->configuration;
        configuration.control_rate = rate_; configuration.state_rate = state_rate_;
        configuration.command_timeout = axis.config.command_timeout;
        configuration.feedback_timeout = axis.config.feedback_timeout;
        configuration.watchdog_timeout = axis.config.watchdog;
        configuration.max_task_duration = max_task_duration_;
        configuration.host_protection_enabled = axis.host_protection;
        configuration.configure_mode_on_startup = axis.config.configure_mode_on_startup;
        configuration.position_min = axis.lower; configuration.position_max = axis.upper;
        configuration.estimated_torque_limit = axis.estimated_torque;
        configuration.measured_torque_limit = axis.measured_torque;
        configuration.measured_velocity_limit = axis.measured_velocity;
        configuration.mos_temperature_stop = axis.mos_stop; configuration.rotor_temperature_stop = axis.rotor_stop;
        configuration.protection_samples = axis.violation_limit; configuration.fault_group = axis.fault_group;
        response->control_rate = rate_;
        response->profile = axis.profile; response->backend = axis.transport->backend(); response->modes = {1, 2, 3, 4};
        response->simulated = axis.transport->simulated(); response->legacy_control = true;
        response->evidence = "J4310-2EC V1.1/V1.2 manuals 2026-09-14; hardware certification is per deployment.";
        response->message = axis.transport->error() + axis.bus->initialization_error() + state.error;
        for (const auto & item : register_catalog()) {
          api::msg::RegisterDescriptor descriptor;
          descriptor.address = item.rid; descriptor.name = item.name;
          descriptor.integer = item.integer; descriptor.writable = item.writable;
          descriptor.raw_write_allowed = raw_register_write_allowed(item.rid);
          descriptor.unit = item.unit; descriptor.has_minimum = item.minimum.has_value();
          descriptor.has_maximum = item.maximum.has_value();
          descriptor.minimum = item.minimum.value_or(0); descriptor.maximum = item.maximum.value_or(0);
          descriptor.minimum_inclusive = item.minimum_inclusive;
          descriptor.maximum_inclusive = item.maximum_inclusive;
          response->registers.push_back(std::move(descriptor));
        }
      };
    axis.handlers[Request::LEGACY_ENABLE] =
      [this, &axis](const Request::SharedPtr req, Reply::SharedPtr response) {
        std::lock_guard<std::mutex> guard(mutex_);
        try {
          if (!req->data) {
            stop_axis(axis, "STOP requested");
          } else {
            require_ready(axis);
            reset_targets(axis);
            axis.bus->enable(axis.config.name, steady_seconds());
            axis.active_source = req->client_id;
          }
          response->success = true;
          response->message = "Request sent, not yet confirmed. Check state.phase/status.";
        } catch (const std::exception & error) {rejection(*response, error);}
      };
    axis.handlers[Request::LEGACY_RESET] =
      [this, &axis](const Request::SharedPtr, Reply::SharedPtr response) {
        std::lock_guard<std::mutex> guard(mutex_);
        try {
          axis.bus->reset(axis.config.name, steady_seconds()); reset_targets(axis);
          response->success = true; response->message = "Host stop latch reset; motor remains disabled.";
        } catch (const std::exception & error) {rejection(*response, error);}
      };
    axis.task_publisher = create_publisher<api::msg::MotorTaskEvent>(
      axis.endpoint + "task_events", rclcpp::QoS(256).reliable().durability_volatile());
    axis.handlers[Request::TASK_START] =
      [this, &axis](const Request::SharedPtr request, Reply::SharedPtr response) {
        std::lock_guard<std::mutex> guard(mutex_);
        try {
          auto definition = std::make_shared<Task::Goal>(request->task);
          definition->token = request->token;
          validate_task(axis, *definition);
          auto goal = std::make_shared<GoalHandle>();
          goal->goal = definition; goal->client_id = request->client_id;
          goal->task_id = request->request_id;
          goal->emit = [this, publisher = axis.task_publisher](api::msg::MotorTaskEvent event) {
              event.header.stamp = now();
              queue_publication(TaskPublication{publisher, std::move(event)});
            };
          const auto previous = std::atomic_exchange(
            &axis.latest_target, std::shared_ptr<const InputTarget>{});
          if (previous && previous->message->request_id) {
            target_reply(axis, *previous->message, false, "Manual target superseded by task start.");
          }
          end_task(axis, false, "Superseded by a newer task");
          axis.task = goal; axis.task_start = steady_seconds();
          axis.active_session = session_deadline(axis, request->token) > axis.task_start ? request->token : "";
          axis.active_source = request->client_id;
          axis.streamed_target = false;
          axis.held_target = false;
          axis.task_start_position = joint_position(axis, axis.bus->snapshot(axis.config.name, axis.task_start));
          axis.task_duration = goal->get_goal()->duration;
          if (goal->get_goal()->kind == Task::Goal::TRAJECTORY) {
            axis.task_duration = goal->get_goal()->times.back();
          }
          axis.task_stopping = false; axis.settled_since = -1;
          axis.requested = from_message(goal->get_goal()->target);
          response->success = true; response->message = "Task accepted; follow task_events.";
        } catch (const std::exception & error) {rejection(*response, error);}
      };
    axis.handlers[Request::TASK_CANCEL] =
      [this, &axis](const Request::SharedPtr request, Reply::SharedPtr response) {
        std::lock_guard<std::mutex> guard(mutex_);
        try {
          if (!axis.task || (request->task_id && axis.task->task_id != request->task_id))
          {
            throw std::runtime_error("cancel requires the matching active task ID, or zero for any active task");
          }
          axis.task->cancel_requested = true;
          response->success = true; response->message = "Task cancellation accepted; wait for disabled result.";
        } catch (const std::exception & error) {rejection(*response, error);}
      };
    axis.handlers[Request::CONTROL_RATE] =
      [this, &axis](const Request::SharedPtr request, Reply::SharedPtr response) {
        {
          std::lock_guard<std::mutex> guard(mutex_);
          require_ready(axis);
          if (axis.task) {throw std::runtime_error("stop the active task before changing control frequency");}
        }
        const auto result = set_parameters_atomically({rclcpp::Parameter("control_rate", request->value)});
        response->success = result.successful;
        response->message = result.successful ? "Control frequency updated." : result.reason;
        response->control_rate = get_parameter("control_rate").as_double();
      };
    axis.reply_publisher = create_publisher<Reply>(
      axis.endpoint + "control_result", rclcpp::QoS(256).reliable().durability_volatile());
    axis.control_subscription = create_subscription<Request>(
      axis.endpoint + "control", rclcpp::QoS(64).reliable().durability_volatile(),
      [this, &axis](Request::ConstSharedPtr request) {process_control(axis, *request);});
  }

  void process_control(Axis & axis, const Request & request)
  {
    auto response = std::make_shared<Reply>();
    response->client_id = request.client_id; response->request_id = request.request_id;
    response->kind = request.kind;
    const bool stop = (request.kind == Request::OPERATION && request.operation == Request::DISABLE) ||
      (request.kind == Request::LEGACY_ENABLE && !request.data);
    try {
      if (!stop) {
        const double age = (now() - rclcpp::Time(request.header.stamp)).seconds();
        if (request.client_id.empty() || request.client_id.size() > 128 || !request.request_id ||
          !std::isfinite(request.valid_for) || request.valid_for <= 0 || request.valid_for > 1 ||
          age > request.valid_for || age < -0.05)
        {
          throw std::invalid_argument("control requires client ID, sequence and fresh timestamp/validity");
        }
        for (auto it = axis.command_history.begin(); it != axis.command_history.end();) {
          if ((now() - rclcpp::Time(it->second.reply.header.stamp)).seconds() > 2) {
            it = axis.command_history.erase(it);
          } else {++it;}
        }
        const auto previous = axis.command_history.find(request.client_id);
        if (previous == axis.command_history.end() && axis.command_history.size() >= 256) {
          throw std::runtime_error("control client history capacity reached; no command executed");
        }
        if (previous != axis.command_history.end() && request.request_id <= previous->second.reply.request_id) {
          if (request.request_id == previous->second.reply.request_id) {
            if (request != previous->second.request) {
              throw std::invalid_argument("control request ID reused with a different payload");
            }
            queue_publication(ReplyPublication{axis.reply_publisher, previous->second.reply});
            return;
          }
          throw std::invalid_argument("stale control request sequence");
        }
      }
      const auto handler = axis.handlers.find(request.kind);
      if (handler == axis.handlers.end()) {throw std::invalid_argument("unsupported control command kind");}
      handler->second(std::make_shared<Request>(request), response);
    } catch (const std::exception & error) {rejection(*response, error);}
    response->header.stamp = now();
    const auto cached = axis.command_history.find(request.client_id);
    if (!request.client_id.empty() && (cached != axis.command_history.end() || axis.command_history.size() < 256) &&
      (cached == axis.command_history.end() || request.request_id > cached->second.reply.request_id))
    {
      axis.command_history[request.client_id] = CachedControl{request, *response};
    }
    queue_publication(ReplyPublication{axis.reply_publisher, std::move(*response)});
  }

  void target_reply(Axis & axis, const api::msg::MotorSetpoint & msg, bool success, const std::string & message)
  {
    if (msg.client_id.empty() || !msg.request_id) {return;}
    Reply reply;
    reply.header.stamp = now(); reply.client_id = msg.client_id; reply.request_id = msg.request_id;
    reply.kind = Request::TARGET; reply.success = success; reply.message = message;
    reply.token = msg.token;
    reply.target_sequence = msg.sequence;
    queue_publication(ReplyPublication{axis.reply_publisher, std::move(reply)});
  }

  void queue_publication(TelemetryPublication publication)
  {
    {
      std::lock_guard<std::mutex> guard(feedback_mutex_);
      if (feedback_queue_.size() == 65536) {
        feedback_queue_.pop_front();
        RCLCPP_ERROR(get_logger(), "Telemetry backlog overflow; inspect feedback, command and task timeouts");
      }
      feedback_queue_.push_back(std::move(publication));
    }
    feedback_condition_.notify_one();
  }

  void validate_task(const Axis & axis, const Task::Goal & goal) const
  {
    const double time = steady_seconds();
    require_ready(axis);
    const auto state = axis.bus->snapshot(axis.config.name, time);
    if (state.phase != Phase::Enabled || state.feedback_age < 0 ||
      state.feedback_age > axis.config.feedback_timeout)
    {
      throw std::runtime_error("task requires confirmed enabled, fresh motor feedback");
    }
    positive(goal.duration, "task duration");
    if (goal.duration > max_task_duration_) {throw std::invalid_argument("task exceeds max_task_duration");}
    positive(goal.max_velocity, "max_velocity"); positive(goal.max_acceleration, "max_acceleration");
    positive(goal.tolerance, "tolerance"); positive(goal.settling_time, "settling_time");
    finite(goal.amplitude, "amplitude"); finite(goal.frequency, "frequency"); finite(goal.ramp_time, "ramp_time");
    if (goal.settling_time > goal.duration || goal.ramp_time < 0 || goal.ramp_time > goal.duration / 2) {
      throw std::invalid_argument("settling_time/ramp_time incompatible with duration");
    }
    const auto target = from_message(goal.target);
    check_target(axis, target, state);
    if (goal.max_velocity > state.limits.v) {throw std::invalid_argument("task max_velocity exceeds motor limit");}
    if (goal.kind < Task::Goal::MOVE || goal.kind > Task::Goal::TRAJECTORY) {
      throw std::invalid_argument("unknown task kind");
    }
    if (goal.kind != Task::Goal::SINE && (goal.amplitude != 0 || goal.frequency != 0)) {
      throw std::invalid_argument("amplitude/frequency only apply to SINE");
    }
    const bool position_task = goal.kind != Task::Goal::VELOCITY && goal.kind != Task::Goal::TORQUE;
    if (position_task && (target.mode == Mode::Velocity ||
      (target.mode == Mode::Mit && target.kp <= 0)))
    {
      throw std::invalid_argument("position task needs position mode or MIT kp>0");
    }
    if (goal.kind == Task::Goal::VELOCITY &&
      (target.mode != Mode::Velocity && target.mode != Mode::Mit))
    {
      throw std::invalid_argument("velocity task needs velocity or MIT mode");
    }
    if (goal.kind == Task::Goal::VELOCITY &&
      (std::abs(target.velocity) > goal.max_velocity ||
      (target.mode == Mode::Mit && (target.kp != 0 || target.kd <= 0 || target.torque != 0)) ||
      goal.ramp_time <= 0 || std::abs(target.velocity) / goal.ramp_time > goal.max_acceleration))
    {
      throw std::invalid_argument("velocity task requires bounded ramp and pure velocity target");
    }
    if (goal.kind == Task::Goal::TORQUE &&
      (target.mode != Mode::Mit || target.kp != 0 || target.kd != 0 || target.velocity != 0 ||
      goal.ramp_time <= 0))
    {
      throw std::invalid_argument("torque task requires pure MIT torque with nonzero ramp_time");
    }
    const double start = joint_position(axis, state);
    if (goal.kind == Task::Goal::MOVE || goal.kind == Task::Goal::RAMP) {
      const double movement = goal.duration - goal.settling_time;
      if (movement <= 0 || 1.875 * std::abs(target.position - start) / movement > goal.max_velocity ||
        5.774 * std::abs(target.position - start) / (movement * movement) > goal.max_acceleration)
      {
        throw std::invalid_argument("move duration violates quintic velocity/acceleration bounds");
      }
    }
    if (goal.kind == Task::Goal::SINE) {
      positive(goal.frequency, "frequency");
      if (std::abs(target.position - start) > goal.tolerance ||
        std::abs(goal.amplitude) * pi * goal.frequency > goal.max_velocity ||
        2 * std::abs(goal.amplitude) * pi * pi * goal.frequency * goal.frequency > goal.max_acceleration)
      {
        throw std::invalid_argument("sine must start near current position and satisfy motion bounds");
      }
      auto extreme = target; extreme.position += goal.amplitude;
      check_target(axis, extreme, state);
    }
    if (goal.kind == Task::Goal::HOLD && std::abs(target.position - start) > goal.tolerance) {
      throw std::invalid_argument("hold target must be near current position; use MOVE first");
    }
    if (position_task && target.velocity != 0) {
      throw std::invalid_argument("position task derives velocity; supplied velocity must be zero");
    }
    if (position_task && target.mode != Mode::Mit &&
      (target.velocity_limit <= 0 || target.velocity_limit < goal.max_velocity))
    {
      throw std::invalid_argument("position mode velocity_limit must cover positive task max_velocity");
    }
    if (goal.kind == Task::Goal::TRAJECTORY) {
      if (goal.times.size() < 2 || goal.times.size() > 256 ||
        goal.positions.size() != goal.times.size() || goal.times.front() != 0 ||
        goal.times.back() + goal.settling_time > goal.duration ||
        std::abs(goal.positions.front() - start) > goal.tolerance)
      {
        throw std::invalid_argument("trajectory needs 2..256 matched points, t0=0, bounded time, current first point");
      }
      for (size_t i = 0; i < goal.times.size(); ++i) {
        finite(goal.times[i], "trajectory time"); finite(goal.positions[i], "trajectory position");
        auto point = target; point.position = goal.positions[i]; check_target(axis, point, state);
        if (i > 0) {
          const double dt = goal.times[i] - goal.times[i - 1];
          const double distance = std::abs(goal.positions[i] - goal.positions[i - 1]);
          if (dt <= 0 || 1.875 * distance / dt > goal.max_velocity ||
            5.774 * distance / (dt * dt) > goal.max_acceleration)
          {
            throw std::invalid_argument("trajectory times/bounds invalid for rest-to-rest quintic segments");
          }
        }
      }
    } else if (!goal.times.empty() || !goal.positions.empty()) {
      throw std::invalid_argument("trajectory arrays only apply to TRAJECTORY tasks");
    }
  }

  void quintic(Target & target, double from, double to, double elapsed, double duration)
  {
    const double u = std::clamp(elapsed / duration, 0.0, 1.0);
    const double blend = u * u * u * (10 + u * (-15 + 6 * u));
    const double derivative = 30 * u * u * (1 - u) * (1 - u) / duration;
    target.position = from + (to - from) * blend;
    if (target.mode == Mode::Mit) {target.velocity = (to - from) * derivative;}
  }

  void task_tick(Axis & axis, const MotorSnapshot & state, double time)
  {
    if (!axis.task) {return;}
    if (axis.task_stopping) {
      if (state.feedback.status == 0 && state.feedback_age >= 0 &&
        state.feedback_age < axis.config.feedback_timeout)
      {
        end_task(axis, axis.task_success, axis.task_message);
      } else if (time - axis.task_start > axis.task_duration + 2.0) {
        end_task(axis, false, "STOP unconfirmed; hardware watchdog required");
      }
      return;
    }
    if (state.phase == Phase::Fault) {end_task(axis, false, state.error); return;}
    if (axis.task->is_canceling()) {
      axis.task_stopping = true; axis.task_success = false; axis.task_message = "Canceled; disabled feedback confirmed";
      axis.bus->disable(axis.config.name, time); reset_targets(axis); return;
    }
    const auto goal = axis.task->get_goal();
    const double elapsed = time - axis.task_start;
    auto target = from_message(goal->target);
    if (goal->kind == Task::Goal::MOVE || goal->kind == Task::Goal::RAMP) {
      quintic(target, axis.task_start_position, target.position, elapsed, goal->duration - goal->settling_time);
    } else if (goal->kind == Task::Goal::SINE) {
      const double omega = 2 * pi * goal->frequency;
      target.position += goal->amplitude * (1 - std::cos(omega * elapsed)) / 2;
      if (target.mode == Mode::Mit) {target.velocity = goal->amplitude * omega * std::sin(omega * elapsed) / 2;}
    } else if (goal->kind == Task::Goal::VELOCITY || goal->kind == Task::Goal::TORQUE) {
      const double envelope = std::clamp(std::min(elapsed, goal->duration - elapsed) / goal->ramp_time, 0.0, 1.0);
      if (goal->kind == Task::Goal::VELOCITY) {target.velocity *= envelope;}
      else {target.torque *= envelope;}
    } else if (goal->kind == Task::Goal::TRAJECTORY) {
      const auto end = std::upper_bound(goal->times.begin(), goal->times.end(), elapsed);
      if (end == goal->times.end()) {target.position = goal->positions.back(); target.velocity = 0;}
      else {
        const size_t index = static_cast<size_t>(std::distance(goal->times.begin(), end));
        quintic(target, goal->positions[index - 1], goal->positions[index],
          elapsed - goal->times[index - 1], goal->times[index] - goal->times[index - 1]);
      }
    }
    const double error = target.position - joint_position(axis, state);
    const bool positioning = goal->kind == Task::Goal::MOVE || goal->kind == Task::Goal::RAMP ||
      goal->kind == Task::Goal::TRAJECTORY;
    if (positioning && std::abs(error) <= goal->tolerance &&
      (goal->kind == Task::Goal::TRAJECTORY ? elapsed >= axis.task_duration :
      elapsed >= goal->duration - goal->settling_time))
    {
      if (axis.settled_since < 0) {axis.settled_since = time;}
    } else {axis.settled_since = -1;}
    const double allowed_duration = goal->kind == Task::Goal::TRAJECTORY ?
      axis.task_duration + goal->settling_time : goal->duration;
    if (elapsed >= allowed_duration) {
      axis.task_success = !positioning ||
        (axis.settled_since >= 0 && time - axis.settled_since >= goal->settling_time - 1 / rate_);
      axis.task_message = axis.task_success ? "Task completed; disabled feedback confirmed" :
        "Task exceeded final tracking tolerance/settling requirement: position_error=" +
        std::to_string(error) + " rad, tolerance=" + std::to_string(goal->tolerance) +
        " rad, required_settling=" + std::to_string(goal->settling_time) + " s; disabled";
      axis.task_stopping = true;
      axis.bus->disable(axis.config.name, time); reset_targets(axis);
      return;
    }
    submit(axis, target, time);
    if (time - axis.task_feedback_time >= 0.05) {
      auto feedback = std::make_shared<Task::Feedback>();
      feedback->elapsed = elapsed; feedback->progress = std::min(1.0, elapsed / allowed_duration);
      feedback->position_error = error; feedback->phase = static_cast<uint8_t>(state.phase);
      axis.task->publish_feedback(feedback); axis.task_feedback_time = time;
    }
  }

  void control_loop()
  {
    auto next = std::chrono::steady_clock::now();
    while (running_) {
      std::chrono::steady_clock::duration period;
      std::vector<std::pair<rclcpp::Publisher<api::msg::MotorFeedback>::SharedPtr,
        api::msg::MotorFeedback>> feedback;
      {
        std::lock_guard<std::mutex> guard(mutex_);
        period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / rate_));
        const double time = steady_seconds();
        ++control_cycle_count_;
        if (rate_window_started_ == 0) {
          rate_window_started_ = time; rate_window_count_ = control_cycle_count_;
        } else if (time - rate_window_started_ >= 1) {
          measured_rate_ = (control_cycle_count_ - rate_window_count_) / (time - rate_window_started_);
          rate_window_started_ = time; rate_window_count_ = control_cycle_count_;
        }
        period_ = last_tick_ > 0 ? time - last_tick_ : 0;
        max_period_ = std::max(max_period_, period_);
        if (period_ > 0) {
          periods_[period_index_] = period_;
          period_index_ = (period_index_ + 1) % periods_.size();
          period_count_ = std::min(period_count_ + 1, periods_.size());
        }
        if (period_ > 1.5 / rate_) {++deadline_misses_;}
        last_tick_ = time;
        for (auto & axis : axes_) {
          try {
            if (!axis->transport->error().empty()) {
              axis->bus->transport_failed(axis->transport->error());
              throw std::runtime_error(axis->transport->error());
            }
            if (!axis->transport->ready()) {continue;}
            axis->backend = axis->transport->backend();
            auto state = axis->bus->snapshot(axis->config.name, time);
            axis->transport->set_quiet(
              (state.phase == Phase::Disabled || state.phase == Phase::Fault) &&
              (state.feedback_age < 0 || state.feedback.status != 1));
            axis->transport->set_fault(state.host_fault);
            if (axis->bus->ready()) {
              if (!axis->explicit_bounds) {
                const auto bounds = joint_bounds(*axis, state);
                axis->lower = bounds.first;
                axis->upper = bounds.second;
              }
              if (!axis->active_session.empty() && session_deadline(*axis, axis->active_session) <= time) {
                stop_axis(*axis, "Active source keepalive lease expired");
              }
              for (auto it = axis->keepalives.begin(); it != axis->keepalives.end();) {
                if (it->second.deadline <= time) {it = axis->keepalives.erase(it);}
                else {++it;}
              }
              if (period_ > axis->config.feedback_timeout &&
                (state.phase == Phase::Enabled || state.phase == Phase::Enabling))
              {
                axis->bus->fail(axis->config.name, "control loop stalled beyond feedback timeout");
              }
              const auto input = std::atomic_exchange(
                &axis->latest_target, std::shared_ptr<const InputTarget>{});
              if (input) {
                const auto & incoming = input->message;
                try {
                  accept_target(*axis, *incoming, input->source, incoming->hold_until_lease_expiry, time);
                  target_reply(*axis, *incoming, true, "Target accepted at the control-cycle boundary.");
                }
                catch (const std::exception & error) {
                  ++axis->rejected;
                  const auto phase = axis->bus->snapshot(axis->config.name, time).phase;
                  if (!incoming->hold_until_lease_expiry &&
                    (phase == Phase::Enabled || phase == Phase::Enabling))
                  {axis->bus->fail(axis->config.name, error.what());}
                  target_reply(*axis, *incoming, false, error.what());
                  RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Topic target rejected: %s", error.what());
                }
              }
              if (axis->streamed_target && !axis->held_target && state.phase == Phase::Enabled &&
                time > axis->target_deadline && !axis->task)
              {
                axis->bus->fail(axis->config.name, "command publisher timeout: setpoint validity expired");
                axis->streamed_target = false;
              }
              if (axis->host_protection && state.phase == Phase::Enabled && state.feedback_age >= 0 &&
                state.feedback_age < axis->config.feedback_timeout &&
                state.feedback_count != axis->protection_count)
              {
                axis->protection_count = state.feedback_count;
                const double position = joint_position(*axis, state);
                const bool violation = position < axis->lower || position > axis->upper ||
                  (axis->measured_velocity > 0 && std::abs(state.feedback.v) > axis->measured_velocity) ||
                  (axis->measured_torque > 0 && std::abs(state.feedback.t) > axis->measured_torque) ||
                  (axis->mos_stop > 0 && state.feedback.mos >= axis->mos_stop) ||
                  (axis->rotor_stop > 0 && state.feedback.rotor >= axis->rotor_stop);
                axis->violations = violation ? axis->violations + 1 : 0;
                if (axis->violations >= axis->violation_limit) {
                  axis->bus->fail(axis->config.name, "measured feedback protection exceeded");
                }
              } else if (state.phase != Phase::Enabled) {axis->violations = 0;}
              task_tick(*axis, axis->bus->snapshot(axis->config.name, time), time);
              if (axis->held_target && axis->streamed_target && !axis->task &&
                state.phase == Phase::Enabled && session_deadline(*axis, axis->active_session) > time)
              {
                axis->bus->command(axis->config.name, native_target(*axis, axis->active), time);
              }
            }
          } catch (const std::exception & error) {
            axis->bus->fail(axis->config.name, error.what()); end_task(*axis, false, error.what());
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Axis control failed: %s", error.what());
          }
        }
        for (auto & bus : buses_) {
          if (!axes_.front()->transport->ready()) {continue;}
          try {
            bus->tick(time);
            for (auto & axis : axes_) {
              if (axis->bus != bus.get()) {continue;}
              axis->applied_target_active = (axis->streamed_target || axis->task) &&
                bus->snapshot(axis->config.name, time).phase == Phase::Enabled;
              if (axis->applied_target_active) {
                axis->applied = axis->active;
                axis->applied_sequence = axis->task ? 0 : axis->sequence;
                axis->applied_target_age_ms = axis->task ? 0 : axis->accepted_target_age_ms;
              }
            }
          }
          catch (const std::exception & error) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Bus I/O failed: %s", error.what());
            for (auto & axis : axes_) {
              if (axis->bus == bus.get()) {
                axis->bus->fail(axis->config.name, error.what()); end_task(*axis, false, error.what());
              }
            }
            for (const auto & origin : axes_) {
              const auto state = origin->bus->snapshot(origin->config.name, time);
              if (state.phase != Phase::Fault || origin->fault_group.empty()) {continue;}
              for (const auto & axis : axes_) {
                if (axis.get() != origin.get() && axis->fault_group == origin->fault_group) {
                  axis->bus->fail(axis->config.name, "fault group " + origin->fault_group +
                    " stopped by " + origin->config.name);
                }
              }
            }
          }
        }
        feedback.swap(pending_feedback_);
      }
      if (!feedback.empty()) {
        {
          std::lock_guard<std::mutex> guard(feedback_mutex_);
          for (auto & sample : feedback) {
            if (feedback_queue_.size() == 65536) {
              feedback_queue_.pop_front();
              RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                "Feedback publisher backlog overflow; oldest sample dropped, inspect feedback sequence gaps");
            }
            feedback_queue_.push_back(std::move(sample));
          }
        }
        feedback_condition_.notify_one();
      }
      next += period;
      if (next < std::chrono::steady_clock::now()) {next = std::chrono::steady_clock::now();}
      std::this_thread::sleep_until(next);
    }
  }

  void feedback_loop()
  {
    while (running_) {
      decltype(feedback_queue_) samples;
      {
        std::unique_lock<std::mutex> guard(feedback_mutex_);
        feedback_condition_.wait(guard, [this] {return !running_ || !feedback_queue_.empty();});
        if (!running_) {return;}
        samples.swap(feedback_queue_);
      }
      try {
        for (const auto & sample : samples) {
          if (!running_) {return;}
          std::visit([](const auto & publication) {
            publication.first->publish(publication.second);
          }, sample);
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Feedback publication failed; inspect sample counters: %s", error.what());
      }
    }
  }

  void publish_state()
  {
    std::unique_lock<std::mutex> guard(mutex_);
    const double time = steady_seconds();
    std::vector<std::pair<rclcpp::Publisher<api::msg::MotorState>::SharedPtr, api::msg::MotorState>> publications;
    publications.reserve(axes_.size());
    sensor_msgs::msg::JointState joints;
    joints.header.stamp = now();
    for (auto & axis : axes_) {
      const auto state = axis->bus->snapshot(axis->config.name, time);
      api::msg::MotorState msg;
      msg.header.stamp = now(); msg.name = axis->config.name;
      msg.motor_id = axis->config.id; msg.master_id = axis->config.master;
      msg.phase = static_cast<uint8_t>(state.phase); msg.mode = static_cast<uint8_t>(state.mode);
      msg.simulated = axis->transport->simulated();
      msg.ready = axis->transport->ready() && axis->bus->ready() && state.absolute_position_valid;
      const bool local_fresh = state.feedback_age >= 0 &&
        state.feedback_age <= axis->config.feedback_timeout;
      msg.maintenance_available = msg.ready && local_fresh &&
        (state.phase == Phase::Disabled || state.phase == Phase::Fault) &&
        state.feedback.status != 1 && axis->transport->maintenance_available();
      msg.activity_available = msg.ready && local_fresh && axis->transport->activity_available();
      msg.feedback_fresh = axis->transport->ready() &&
        state.feedback_age >= 0 && state.feedback_age <= axis->config.feedback_timeout;
      msg.status = state.feedback.status; msg.position = joint_position(*axis, state);
      msg.host_fault = state.host_fault; msg.device_fault = state.device_fault;
      msg.status_text = motor_status_text(state.feedback.status);
      msg.host_protection_enabled = axis->host_protection;
      msg.velocity = axis->direction * state.feedback.v; msg.torque = axis->direction * state.feedback.t;
      msg.native_position = state.feedback.p; msg.native_velocity = state.feedback.v; msg.native_torque = state.feedback.t;
      msg.absolute_position = state.absolute_position;
      msg.absolute_position_valid = state.absolute_position_valid;
      msg.position_wraps = state.position_wraps;
      msg.direction = axis->direction; msg.software_offset = axis->offset;
      const auto bounds = joint_bounds(*axis, state);
      msg.position_min = bounds.first; msg.position_max = bounds.second;
      msg.mos_temperature = state.feedback.mos; msg.rotor_temperature = state.feedback.rotor;
      msg.feedback_age = state.feedback_age; msg.feedback_count = state.feedback_count;
      msg.error = state.error.empty() ? axis->transport->error() : state.error;
      msg.pmax = state.mapping.p; msg.vmax = state.mapping.v; msg.tmax = state.mapping.t;
      msg.position_limit = position_command_limit(state.mode, state.mapping, axis->config.limits);
      msg.velocity_limit = state.limits.v; msg.torque_limit = state.limits.t;
      msg.kp_limit = state.limits.kp; msg.kd_limit = state.limits.kd;
      msg.period_ms = period_ * 1000; msg.max_period_ms = max_period_ * 1000; msg.deadline_misses = deadline_misses_;
      msg.control_rate = rate_; msg.actual_control_rate = measured_rate_;
      msg.control_cycle_count = control_cycle_count_;
      msg.control_owner = "";
      msg.active_source = axis->active_source;
      msg.lease_remaining = std::max(0.0, session_deadline(*axis, axis->active_session) - time);
      msg.task_active = axis->task != nullptr; msg.rejected_commands = axis->rejected;
      msg.requested_target = to_message(axis->requested); msg.active_target = to_message(axis->active);
      msg.target_active = state.phase == Phase::Enabled && (axis->streamed_target || axis->task);
      msg.accepted_target_sequence = axis->sequence;
      msg.applied_target_sequence = axis->applied_sequence;
      msg.target_accept_latency_ms = axis->applied_target_age_ms;
      if (!msg.target_active) {
        Target inactive;
        inactive.mode = state.mode;
        msg.active_target = to_message(inactive);
      }
      msg.period_p50_ms = percentiles_[0] * 1000;
      msg.period_p95_ms = percentiles_[1] * 1000;
      msg.period_p99_ms = percentiles_[2] * 1000;
      msg.fault_group = axis->fault_group;
      msg.feedback_receive_time = state.feedback_age < 0 ? -1 : time - state.feedback_age;
      if (time - axis->feedback_window >= 1) {
        axis->feedback_rate = axis->feedback_window > 0 ?
          (state.feedback_count - axis->window_count) / (time - axis->feedback_window) : 0;
        axis->window_count = state.feedback_count; axis->feedback_window = time;
      }
      msg.feedback_rate = axis->feedback_rate;
      publications.emplace_back(axis->publisher, msg);
      if (msg.feedback_fresh && msg.absolute_position_valid) {
        joints.name.push_back(axis->config.name); joints.position.push_back(msg.position);
        joints.velocity.push_back(msg.velocity); joints.effort.push_back(msg.torque);
      }
    }
    guard.unlock();
    for (auto & publication : publications) {publication.first->publish(publication.second);}
    joint_pub_->publish(joints);
  }

  void publish_diagnostics()
  {
    std::unique_lock<std::mutex> guard(mutex_);
    diagnostic_msgs::msg::DiagnosticArray array;
    if (period_count_ > 0) {
      std::vector<double> samples(periods_.begin(), periods_.begin() + period_count_);
      std::sort(samples.begin(), samples.end());
      for (size_t index = 0; index < percentiles_.size(); ++index) {
        const double percentile = std::array<double, 3>{0.5, 0.95, 0.99}[index];
        percentiles_[index] = samples[static_cast<size_t>(std::ceil(percentile * samples.size())) - 1];
      }
    }
    array.header.stamp = now();
    for (const auto & axis : axes_) {
      const auto state = axis->bus->snapshot(axis->config.name, steady_seconds());
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = get_fully_qualified_name() + std::string("/") + axis->config.name;
      status.hardware_id = axis->profile + ":" + std::to_string(axis->config.id);
      status.level = state.phase == Phase::Fault ? status.ERROR :
        (!axis->bus->ready() || !state.absolute_position_valid ||
        state.feedback_age < 0 || state.feedback_age > axis->config.feedback_timeout ?
        status.WARN : status.OK);
      status.message = state.error.empty() ? (axis->bus->ready() ? "Ready" : "Initializing") : state.error;
      for (const auto & value : std::vector<std::pair<std::string, std::string>>{
          {"backend", axis->backend}, {"feedback_hz", std::to_string(axis->feedback_rate)},
          {"control_rate", std::to_string(rate_)}, {"actual_control_rate", std::to_string(measured_rate_)},
          {"feedback_age", std::to_string(state.feedback_age)},
          {"absolute_position", std::to_string(state.absolute_position)},
          {"absolute_position_valid", state.absolute_position_valid ? "true" : "false"},
          {"position_wraps", std::to_string(state.position_wraps)},
          {"deadline_misses", std::to_string(deadline_misses_)},
          {"period_p50_ms", std::to_string(percentiles_[0] * 1000)},
          {"period_p95_ms", std::to_string(percentiles_[1] * 1000)},
          {"period_p99_ms", std::to_string(percentiles_[2] * 1000)},
          {"rejected_commands", std::to_string(axis->rejected)}, {"active_source", axis->active_source}})
      {
        diagnostic_msgs::msg::KeyValue item;
        item.key = value.first; item.value = value.second; status.values.push_back(item);
      }
      array.status.push_back(status);
    }
    guard.unlock();
    diagnostics_pub_->publish(array);
  }

  std::vector<std::unique_ptr<Bus>> buses_;
  std::vector<std::unique_ptr<Axis>> axes_;
  std::mutex mutex_;
  std::atomic<bool> running_{false};
  std::thread control_thread_;
  double rate_{200}, state_rate_{200}, max_task_duration_{60};
  double last_tick_{0}, period_{0}, max_period_{0};
  std::vector<std::pair<rclcpp::Publisher<api::msg::MotorFeedback>::SharedPtr,
    api::msg::MotorFeedback>> pending_feedback_;
  std::deque<TelemetryPublication> feedback_queue_;
  std::mutex feedback_mutex_;
  std::condition_variable feedback_condition_;
  std::thread feedback_thread_;
  rclcpp::CallbackGroup::SharedPtr target_callback_group_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> target_executor_;
  std::thread target_thread_;
  double state_rate_limit_{0}, measured_rate_{0}, rate_window_started_{0};
  uint64_t control_cycle_count_{0}, rate_window_count_{0};
  uint64_t deadline_misses_{0};
  std::array<double, 2048> periods_{};
  size_t period_index_{0}, period_count_{0};
  std::array<double, 3> percentiles_{};
  rclcpp::TimerBase::SharedPtr state_timer_, diagnostics_timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_;
};
}  // namespace dm_motor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<dm_motor::MotorNode> node;
  int result = 0;
  try {
    node = std::make_shared<dm_motor::MotorNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_controller"), "%s", error.what());
    result = 1;
  }
  if (node) {node->stop();}
  rclcpp::shutdown();
  return result;
}
