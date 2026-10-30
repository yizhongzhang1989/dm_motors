#include "dm_motor_control/simulation/device.hpp"
#include "common/config.hpp"
#include "common/process_lock.hpp"

#include <can_bus/msg/frame.hpp>
#include <can_bus/msg/status.hpp>
#include <dm_motor_interfaces/msg/external_torque.hpp>
#include <dm_motor_interfaces/msg/simulation_state.hpp>
#include <dm_motor_interfaces/srv/simulation_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>

namespace dm_motor::simulation
{
using Wire = can_bus::msg::Frame;
using State = dm_motor_interfaces::msg::SimulationState;
using Force = dm_motor_interfaces::msg::ExternalTorque;
using Control = dm_motor_interfaces::srv::SimulationControl;

class SimMotorNode final : public rclcpp::Node
{
public:
  SimMotorNode() : Node("simulated_device"),
    config_(declare_parameter<std::string>("config_file", common::default_config_path()))
  {
    if (get_parameter("use_sim_time").as_bool()) {
      throw std::invalid_argument("Virtual device uses a wall-time physical clock; use_sim_time must be false");
    }
    bus_ = parameter<std::string>("can_bus_namespace");
    if (bus_.empty() || bus_.front() != '/' || bus_.back() == '/' || bus_.find("//") != std::string::npos) {
      throw std::invalid_argument("can_bus_namespace must be an absolute namespace without trailing slash");
    }
    DeviceParameters parameters;
    parameters.motor.name = parameter<std::string>("motor_name");
    const int id = parameter<int>("motor_id"), master = parameter<int>("master_id"), mode = parameter<int>("mode");
    if (id < 1 || id > 15 || master < 0 || master > 0x7fe || mode < 1 || mode > 4) {
      throw std::invalid_argument("motor_id 1..15, master_id 0..0x7fe, mode 1..4 required");
    }
    parameters.motor.id = id; parameters.motor.master = master; parameters.motor.mode = static_cast<dm_motor::Mode>(mode);
    parameters.motor.mapping = {parameter<double>("mapping_position"), parameter<double>("mapping_velocity"),
      parameter<double>("mapping_torque")};
    auto & physics = parameters.physics;
    physics.inertia = parameter<double>("inertia");
    physics.viscous_friction = parameter<double>("viscous_friction");
    physics.coulomb_friction = parameter<double>("coulomb_friction");
    physics.static_friction = parameter<double>("static_friction");
    physics.friction_model = parameter<std::string>("friction_model");
    physics.stribeck_velocity = parameter<double>("stribeck_velocity");
    physics.friction_negative_scale = parameter<double>("friction_negative_scale");
    physics.gravity_torque = parameter<double>("gravity_torque");
    physics.gravity_offset = parameter<double>("gravity_offset");
    physics.joint_stiffness = parameter<double>("joint_stiffness");
    physics.load_torque = parameter<double>("load_torque");
    physics.max_torque = parameter<double>("max_torque");
    physics.no_load_speed = parameter<double>("no_load_speed");
    physics.torque_time_constant = parameter<double>("torque_time_constant");
    physics.lower_limit = parameter<double>("lower_limit");
    physics.upper_limit = parameter<double>("upper_limit");
    physics.contact_stiffness = parameter<double>("contact_stiffness");
    physics.contact_damping = parameter<double>("contact_damping");
    physics.ambient_temperature = parameter<double>("ambient_temperature");
    physics.heat_capacity = parameter<double>("heat_capacity");
    physics.thermal_resistance = parameter<double>("thermal_resistance");
    physics.winding_resistance = parameter<double>("winding_resistance");
    physics.torque_constant = parameter<double>("torque_constant");
    physics.initial_position = parameter<double>("initial_position");
    physics.initial_velocity = parameter<double>("initial_velocity");
    parameters.integration_step = parameter<double>("integration_step");
    parameters.bus_voltage = parameter<double>("bus_voltage");
    parameters.velocity_kp = parameter<double>("velocity_kp");
    parameters.velocity_ki = parameter<double>("velocity_ki");
    parameters.position_kp = parameter<double>("position_kp");
    parameters.position_ki = parameter<double>("position_ki");
    parameters.anchor_stiffness = parameter<double>("anchor_stiffness");
    parameters.persistence_file = parameter<std::string>("persistence_file");
    const double rate = parameter<double>("physics_rate"), state_rate = parameter<double>("state_rate");
    if (!std::isfinite(rate) || rate < 100 || rate > 2000 ||
      !std::isfinite(state_rate) || state_rate < 1 || state_rate > rate)
    {
      throw std::invalid_argument("physics_rate must be 100..2000; state_rate 1..physics_rate");
    }
    model_ = std::make_unique<VirtualMotor>(parameters, dm_motor::steady_seconds());
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    ownership_.acquire("virtual_motor:" + std::string(domain ? domain : "0") + ":" +
      bus_ + ":" + std::to_string(model_->parameters().motor.id));
    rx_ = create_publisher<Wire>(bus_ + "/wire_rx", rclcpp::QoS(512).reliable());
    state_ = create_publisher<State>("simulation/state", rclcpp::QoS(1).reliable());
    bus_state_ = create_subscription<can_bus::msg::Status>(
      bus_ + "/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](const can_bus::msg::Status & message) {
        last_bus_status_ = dm_motor::steady_seconds();
        bus_confirmed_ = message.ready && message.backend == "virtual" && message.port.empty();
        if (!bus_confirmed_) {
          error_ = "Simulation requires a ready topic-only virtual CAN bus; physical backends are prohibited";
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error_.c_str());
        } else if (bus_epoch_ && bus_epoch_ != message.epoch) {
          model_->inject_fault(13);
          error_ = "Virtual bus epoch changed; device communication fault latched";
          RCLCPP_ERROR(get_logger(), "%s", error_.c_str());
        }
        bus_epoch_ = message.epoch;
      });
    tx_ = create_subscription<Wire>(
      bus_ + "/wire_tx", rclcpp::QoS(512).reliable(),
      [this](const Wire & message) {receive(message);});
    force_ = create_subscription<Force>(
      "simulation/external_torque", rclcpp::QoS(1).reliable(),
      [this](const Force & message, const rclcpp::MessageInfo & info) {force(message, info);});
    service_ = create_service<Control>("simulation/control",
      [this](Control::Request::SharedPtr request, Control::Response::SharedPtr response) {
        try {
          require_virtual();
          const double time = dm_motor::steady_seconds();
          model_->advance(time);
          if ((request->operation == Control::Request::SET_ANCHOR ||
            request->operation == Control::Request::CLEAR_ANCHOR) &&
            request->check_anchor_revision && request->anchor_revision != model_->anchor().revision)
          {
            throw std::runtime_error("Anchor changed; refresh its state before retrying");
          }
          switch (request->operation) {
            case Control::Request::INJECT_FAULT: model_->inject_fault(request->fault_status); break;
            case Control::Request::PAUSE_FEEDBACK: paused_ = request->data; break;
            case Control::Request::POWER_CYCLE: model_->power_cycle(); break;
            case Control::Request::SET_VOLTAGE: model_->set_voltage(request->value); break;
            case Control::Request::CLEAR_EXTERNAL_TORQUE:
              model_->external_torque(0, 0.001, time);
              model_->clear_anchor(); external_source_.clear(); break;
            case Control::Request::SET_ANCHOR: model_->set_anchor(request->value); break;
            case Control::Request::CLEAR_ANCHOR: model_->clear_anchor(); break;
            default: throw std::invalid_argument("Unknown simulation operation");
          }
          error_.clear();
          response->success = true;
          response->message = "Simulation operation applied; motor driver state remains authoritative for control.";
          publish_state();
        } catch (const std::exception & error) {
          response->message = error.what();
          reject(error.what());
        }
      });
    physics_timer_ = create_wall_timer(std::chrono::duration<double>(1 / rate), [this] {
        try {model_->advance(dm_motor::steady_seconds());}
        catch (const std::exception & error) {reject(error.what()); model_->inject_fault(13);}
      });
    state_timer_ = create_wall_timer(std::chrono::duration<double>(1 / state_rate), [this] {publish_state();});
    RCLCPP_INFO(get_logger(), "Virtual physical motor id=%d on %s; waiting for virtual bus, no physical CAN access",
      id, bus_.c_str());
  }

private:
  template<typename T>
  T parameter(const std::string & name)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, config_.get<T>("packages.dm_motor_control.nodes.sim_motor_node." + name), descriptor);
  }

  void require_virtual() const
  {
    if (!bus_confirmed_ || dm_motor::steady_seconds() - last_bus_status_ > 0.5) {
      throw std::runtime_error("Virtual bus is not confirmed or status is stale; no device traffic permitted");
    }
  }

  void reject(const std::string & message)
  {
    ++rejected_;
    error_ = message;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Virtual motor: %s", message.c_str());
  }

  void receive(const Wire & message)
  {
    if (message.extended || message.rtr || message.error || message.channel != 0 || message.length != message.data.size() ||
      message.data.size() > 8 || message.can_id > 0x7ff)
    {
      return;  // Other device protocols may share the virtual CAN bus.
    }
    const auto & motor = model_->parameters().motor;
    const bool parameter_frame = message.can_id == 0x7ff && message.data.size() >= 4 &&
      message.data[0] == motor.id && message.data[1] == 0;
    if (!parameter_frame && message.can_id != dm_motor::control_id(motor.id, motor.mode)) {return;}
    try {
      require_virtual();
      dm_motor::Frame frame;
      frame.id = message.can_id; frame.length = message.length; frame.fd = message.fd; frame.brs = message.brs;
      std::copy(message.data.begin(), message.data.end(), frame.data.begin());
      const auto time = dm_motor::steady_seconds();
      const auto replies = model_->receive(frame, time);
      if (paused_) {return;}
      for (const auto & reply : replies) {
        Wire output;
        output.header.stamp = now(); output.can_id = reply.id; output.channel = reply.channel;
        output.fd = reply.fd; output.brs = reply.brs; output.length = reply.length;
        output.received_steady = static_cast<uint64_t>(time * 1e9);
        output.data.assign(reply.data.begin(), reply.data.begin() + reply.length);
        rx_->publish(output);
      }
    } catch (const std::exception & error) {reject(error.what());}
  }

  void force(const Force & message, const rclcpp::MessageInfo & info)
  {
    try {
      require_virtual();
      const double time = dm_motor::steady_seconds();
      const auto stamp = rclcpp::Time(message.header.stamp).nanoseconds();
      const double age = (now() - rclcpp::Time(message.header.stamp)).seconds();
      if (!std::isfinite(message.valid_for) || message.valid_for <= 0 || message.valid_for > 5 ||
        age < -0.05 || age >= message.valid_for || message.source.size() > 128)
      {
        throw std::invalid_argument("External torque requires a fresh timestamp and valid_for in (0,5]");
      }
      std::ostringstream key;
      for (auto byte : info.get_rmw_message_info().publisher_gid.data) {
        key << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(byte);
      }
      const auto source = key.str();
      for (auto it = ordering_.begin(); it != ordering_.end();) {
        if (time - it->second.received > 5) {it = ordering_.erase(it);}
        else {++it;}
      }
      const auto previous = ordering_.find(source);
      if (previous != ordering_.end() && (stamp <= previous->second.stamp ||
        (message.sequence && message.sequence <= previous->second.sequence)))
      {
        throw std::invalid_argument("External torque timestamp/sequence replayed");
      }
      if (previous == ordering_.end() && ordering_.size() >= 64) {
        throw std::runtime_error("External torque publisher capacity reached");
      }
      const double validity = message.valid_for - std::max(0.0, age);
      if (message.spring) {
        if (message.torque != 0) {throw std::invalid_argument("Spring and direct torque cannot be combined");}
        model_->external_spring(message.position, message.stiffness, message.damping,
          validity, time);
      } else {
        model_->external_torque(message.torque, validity, time);
      }
      ordering_[source] = {stamp, message.sequence, time};
      external_source_ = message.source.empty() ? source : message.source;
      error_.clear();
    } catch (const std::exception & error) {reject(error.what());}
  }

  void publish_state()
  {
    State output;
    const auto & physical = model_->state();
    output.header.stamp = now(); output.motor_id = model_->parameters().motor.id;
    output.can_bus_namespace = bus_;
    output.virtual_bus_confirmed = bus_confirmed_ && dm_motor::steady_seconds() - last_bus_status_ <= 0.5;
    output.feedback_paused = paused_; output.enabled = model_->status() == 1; output.status = model_->status();
    output.simulation_time = model_->time(); output.integration_steps = physical.integration_steps;
    output.position = physical.position; output.encoder_position = model_->encoder_position();
    output.velocity = physical.velocity; output.motor_torque = physical.motor_torque;
    output.transient_torque = model_->transient_torque();
    output.anchor_torque = model_->anchor_torque();
    output.external_torque = output.transient_torque + output.anchor_torque;
    const auto & anchor = model_->anchor();
    output.anchor_active = anchor.active; output.anchor_revision = anchor.revision;
    output.anchor_position = anchor.position; output.anchor_bearing = anchor.bearing;
    output.anchor_stiffness = model_->parameters().anchor_stiffness;
    output.gravity_torque = physical.gravity_torque; output.friction_torque = physical.friction_torque;
    output.contact_torque = physical.contact_torque; output.net_torque = physical.net_torque;
    output.load_torque = physical.load_torque;
    output.mechanical_energy = physical.mechanical_energy;
    output.rotor_temperature = physical.rotor_temperature; output.mos_temperature = physical.mos_temperature;
    output.bus_voltage = model_->voltage(); output.external_remaining = model_->external_remaining();
    output.external_source = external_source_; output.accepted_commands = model_->commands();
    output.rejected_commands = rejected_; output.error = error_;
    state_->publish(output);
  }

  common::Config config_;
  common::ProcessLock ownership_;
  std::unique_ptr<VirtualMotor> model_;
  std::string bus_, error_, external_source_;
  double last_bus_status_{0};
  uint64_t bus_epoch_{0}, rejected_{0};
  bool bus_confirmed_{false}, paused_{false};
  struct Order {int64_t stamp; uint64_t sequence; double received;};
  std::map<std::string, Order> ordering_;
  rclcpp::Publisher<Wire>::SharedPtr rx_;
  rclcpp::Publisher<State>::SharedPtr state_;
  rclcpp::Subscription<Wire>::SharedPtr tx_;
  rclcpp::Subscription<can_bus::msg::Status>::SharedPtr bus_state_;
  rclcpp::Subscription<Force>::SharedPtr force_;
  rclcpp::Service<Control>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr physics_timer_, state_timer_;
};
}  // namespace dm_motor::simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<dm_motor::simulation::SimMotorNode>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("simulated_device"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
