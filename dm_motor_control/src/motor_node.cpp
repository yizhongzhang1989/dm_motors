#include "dm_motor_control/core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

#include <dm_motor_interfaces/msg/motor_command.hpp>
#include <dm_motor_interfaces/msg/motor_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace dm_motor
{
class MotorNode final : public rclcpp::Node
{
public:
  MotorNode() : Node("motor_controller")
  {
    const auto id = declare_parameter<int>("motor_id", 1);
    const auto master = declare_parameter<int>("master_id", 0);
    if (id < 1 || id > 15 || master < 0 || master > 0x7ff) {
      throw std::invalid_argument("motor_id must be 1..15; master_id must be 0..2047");
    }
    id_ = static_cast<uint8_t>(id);
    master_ = static_cast<uint16_t>(master);
    const auto backend = declare_parameter<std::string>("backend", "sim");
    const auto port = declare_parameter<std::string>("port", "");
    const auto rate = declare_parameter<double>("control_rate", 200.0);
    auto state_rate = declare_parameter<double>("state_rate", 0.0);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 0.1);
    const auto command_timeout = declare_parameter<double>("command_timeout", 0.25);
    limits_ = {declare_parameter<double>("position_limit", 0.0),
      declare_parameter<double>("velocity_limit", 0.0),
      declare_parameter<double>("torque_limit", 0.0),
      declare_parameter<double>("kp_limit", 500.0),
      declare_parameter<double>("kd_limit", 5.0)};
    if (!std::isfinite(rate) || rate < 20 || rate > 1000) {
      throw std::invalid_argument("control_rate must be 20..1000 Hz");
    }
    if (state_rate == 0) {state_rate = rate;}
    if (!std::isfinite(state_rate) || state_rate < 1 || state_rate > rate) {
      throw std::invalid_argument("state_rate must be 0 (follow control_rate) or 1..control_rate Hz");
    }
    if (!std::isfinite(feedback_timeout_) || feedback_timeout_ < 2 / rate ||
      feedback_timeout_ > 1.0 || !std::isfinite(command_timeout) ||
      command_timeout < 2 / rate || command_timeout > 2.0)
    {
      throw std::invalid_argument("timeouts must exceed two control periods and be <= 1/2 seconds");
    }
    command_timeout_ = command_timeout;
    nominal_period_ = 1.0 / rate;
    simulated_ = backend == "sim";
    if (simulated_) {
      transport_ = std::make_unique<SimTransport>(id_, master_, mapping_);
    } else if (backend == "serial") {
      if (port.empty()) {throw std::invalid_argument("serial backend requires explicit port");}
      transport_ = std::make_unique<SerialTransport>(port);
      mapping_ = initialize_serial(*transport_, id_, master_);
    } else {throw std::invalid_argument("backend must be sim or serial");}
    controller_ = std::make_unique<Controller>(*transport_, id_, master_, mapping_, limits_,
      command_timeout_, feedback_timeout_);
    limits_ = controller_->limits();
    RCLCPP_INFO(get_logger(), "Motor %u, backend=%s, %.1f Hz; starts disabled",
      id_, backend.c_str(), rate);
    state_pub_ = create_publisher<dm_motor_interfaces::msg::MotorState>(
      "state", rclcpp::QoS(1).reliable().durability_volatile());
    command_sub_ = create_subscription<dm_motor_interfaces::msg::MotorCommand>(
      "command", rclcpp::QoS(1).best_effort().durability_volatile(),
      [this](const dm_motor_interfaces::msg::MotorCommand & msg) {
        if (controller_->phase() != Phase::Enabled) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Command rejected: motor not confirmed enabled");
          return;
        }
        try {
          const rclcpp::Time stamp(msg.header.stamp, get_clock()->get_clock_type());
          const auto age = (now() - stamp).seconds();
          if (stamp.nanoseconds() <= command_stamp_ || age > command_timeout_ || age < -0.05) {
            throw std::invalid_argument("stale, replayed or future command timestamp");
          }
          controller_->command({msg.position, msg.velocity, msg.kp, msg.kd, msg.torque},
            steady_seconds());
          command_stamp_ = stamp.nanoseconds();
        } catch (const std::exception & error) {
          controller_->fail(error.what());
          RCLCPP_ERROR(get_logger(), "Command rejected and stop latched: %s", error.what());
        }
      });
    enable_srv_ = create_service<std_srvs::srv::SetBool>("enable",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr response) {
        try {
          if (req->data) {
            controller_->enable(steady_seconds());
            command_stamp_ = now().nanoseconds();
          } else {controller_->disable(steady_seconds());}
          response->success = true;
          response->message = "Request sent, not yet confirmed. Check state.phase/status.";
        } catch (const std::exception & error) {
          if (!req->data || controller_->phase() == Phase::Enabling) {
            controller_->fail(error.what());
          }
          response->success = false;
          response->message = error.what();
          RCLCPP_ERROR(get_logger(), "Enable/disable request failed: %s", error.what());
        }
      });
    reset_srv_ = create_service<std_srvs::srv::Trigger>("reset",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        try {
          controller_->reset(steady_seconds());
          response->success = true;
          response->message = "Host stop latch reset; motor remains disabled.";
        } catch (const std::exception & error) {
          response->success = false;
          response->message = error.what();
          RCLCPP_WARN(get_logger(), "Reset rejected: %s", error.what());
        }
      });
    tick_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(nominal_period_)), [this] {tick();});
    state_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / state_rate)), [this] {publish_state();});
  }
  void stop()
  {
    if (!controller_) {return;}
    try {
      const auto previous_count = controller_->feedback_count();
      controller_->disable(steady_seconds());
      const auto deadline = steady_seconds() + feedback_timeout_;
      while (steady_seconds() < deadline) {
        controller_->tick(steady_seconds());
        if (controller_->feedback_count() > previous_count &&
          controller_->fresh(steady_seconds()) && controller_->feedback().status == 0)
        {
          RCLCPP_INFO(get_logger(), "Shutdown: disabled feedback confirmed");
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      RCLCPP_ERROR(get_logger(), "Shutdown disable not confirmed; hardware watchdog is required");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Shutdown disable failed: %s", error.what());
    }
  }
private:
  void tick()
  {
    const auto time = steady_seconds();
    if (last_tick_ > 0) {
      period_ = time - last_tick_;
      max_period_ = std::max(max_period_, period_);
      if (period_ > 1.5 * nominal_period_) {++deadline_misses_;}
      if (period_ > feedback_timeout_ &&
        (controller_->phase() == Phase::Enabled || controller_->phase() == Phase::Enabling))
      {
        controller_->fail("control loop stalled beyond feedback timeout");
        RCLCPP_ERROR(get_logger(), "Stop latched: control loop stalled beyond feedback timeout");
      }
    }
    last_tick_ = time;
    const auto old_phase = controller_->phase();
    try {
      controller_->tick(time);
    } catch (const std::exception & error) {
      controller_->fail(error.what());
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Control I/O failed: %s", error.what());
      // Receive failure must not prevent an independent best-effort disable transmission.
      try {controller_->disable(time);}
      catch (const std::exception & stop_error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
          "Disable transmission failed: %s", stop_error.what());
      }
    }
    if (old_phase != controller_->phase() && controller_->phase() == Phase::Fault) {
      RCLCPP_ERROR(get_logger(), "Stop latched: %s", controller_->error().c_str());
    }
  }
  void publish_state()
  {
    dm_motor_interfaces::msg::MotorState msg;
    msg.header.stamp = now();
    msg.motor_id = id_;
    msg.master_id = master_;
    msg.phase = static_cast<uint8_t>(controller_->phase());
    msg.simulated = simulated_;
    msg.feedback_fresh = controller_->fresh(steady_seconds());
    const auto & fb = controller_->feedback();
    msg.status = fb.status;
    msg.position = fb.p; msg.velocity = fb.v; msg.torque = fb.t;
    msg.mos_temperature = fb.mos; msg.rotor_temperature = fb.rotor;
    msg.feedback_age = controller_->age(steady_seconds());
    msg.feedback_count = controller_->feedback_count();
    msg.error = controller_->error();
    msg.pmax = mapping_.p; msg.vmax = mapping_.v; msg.tmax = mapping_.t;
    msg.position_limit = limits_.p; msg.velocity_limit = limits_.v; msg.torque_limit = limits_.t;
    msg.kp_limit = limits_.kp; msg.kd_limit = limits_.kd;
    msg.period_ms = period_ * 1000; msg.max_period_ms = max_period_ * 1000;
    msg.deadline_misses = deadline_misses_;
    state_pub_->publish(msg);
  }
  uint8_t id_{1};
  uint16_t master_{0};
  bool simulated_{true};
  Mapping mapping_;
  Limits limits_;
  double nominal_period_{0}, feedback_timeout_{0}, command_timeout_{0};
  double last_tick_{0}, period_{0}, max_period_{0};
  uint64_t deadline_misses_{0};
  int64_t command_stamp_{0};
  std::unique_ptr<Transport> transport_;
  std::unique_ptr<Controller> controller_;
  rclcpp::Publisher<dm_motor_interfaces::msg::MotorState>::SharedPtr state_pub_;
  rclcpp::Subscription<dm_motor_interfaces::msg::MotorCommand>::SharedPtr command_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr tick_timer_, state_timer_;
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
