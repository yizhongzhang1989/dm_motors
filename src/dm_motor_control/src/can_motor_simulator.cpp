#include "dm_motor_control/runtime.hpp"

#include <can_bus/msg/frame.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>

namespace dm_motor
{
class MotorSimulator final : public rclcpp::Node
{
public:
  MotorSimulator() : Node("motor_simulator")
  {
    const auto bus = declare_parameter<std::string>("can_bus_namespace", "/can/main");
    const auto names = declare_parameter<std::vector<std::string>>(
      "motor_names", std::vector<std::string>{"motor"});
    std::vector<MotorConfig> configs;
    std::vector<uint8_t> identities;
    for (const auto & name : names) {
      MotorConfig config;
      config.name = name;
      const auto prefix = "motors." + name + ".";
      const int id = declare_parameter<int>(prefix + "motor_id", 1);
      const int master = declare_parameter<int>(prefix + "master_id", 0);
      const int mode = declare_parameter<int>(prefix + "mode", 1);
      if (id < 1 || id > 15 || master < 0 || master > 0x7ff || mode < 1 || mode > 4) {
        throw std::invalid_argument("Invalid simulated motor ID/master/mode");
      }
      config.id = static_cast<uint8_t>(id); config.master = static_cast<uint16_t>(master);
      if (std::find(identities.begin(), identities.end(), config.id) != identities.end()) {
        throw std::invalid_argument("Duplicate simulated motor ID");
      }
      identities.push_back(config.id);
      config.mode = static_cast<Mode>(mode);
      configs.push_back(config);
    }
    transport_ = make_sim_transport(configs);
    for (const auto & config : configs) {
      fault_services_.push_back(create_service<std_srvs::srv::SetBool>(
        "inject_fault_" + config.name,
        [this, config](std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response) {
          auto * simulator = dynamic_cast<SimBusTransport *>(transport_.get());
          if (!simulator) {
            response->success = false; response->message = "Simulation backend unavailable";
            RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
            return;
          }
          simulator->inject_fault(config.id, request->data ? 0xb : 0);
          response->success = true;
          response->message = "Simulation only: MOS fault injection changed.";
        }));
    }
    publisher_ = create_publisher<can_bus::msg::Frame>(
      bus + "/wire_rx", rclcpp::QoS(512).reliable().durability_volatile());
    subscription_ = create_subscription<can_bus::msg::Frame>(
      bus + "/wire_tx", rclcpp::QoS(512).reliable().durability_volatile(),
      [this](const can_bus::msg::Frame & msg) {
        if (msg.extended || msg.rtr || msg.error || msg.can_id > 0x7ff || msg.data.size() > 8 ||
          msg.length != msg.data.size() || msg.channel != 0)
        {
          return;
        }
        Frame frame;
        frame.id = static_cast<uint16_t>(msg.can_id);
        frame.length = static_cast<uint8_t>(msg.data.size());
        frame.fd = msg.fd; frame.brs = msg.brs;
        std::copy(msg.data.begin(), msg.data.end(), frame.data.begin());
        try {
          transport_->send(frame);
          for (const auto & received : transport_->receive()) {
            can_bus::msg::Frame reply;
            reply.header.stamp = now(); reply.can_id = received.id;
            reply.channel = received.channel; reply.fd = msg.fd; reply.brs = msg.brs;
            reply.length = received.length;
            reply.data.assign(received.data.begin(), received.data.begin() + received.length);
            publisher_->publish(reply);
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "Simulated device rejected frame: %s", error.what());
        }
      });
  }
private:
  std::unique_ptr<Transport> transport_;
  rclcpp::Publisher<can_bus::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_bus::msg::Frame>::SharedPtr subscription_;
  std::vector<rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr> fault_services_;
};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<dm_motor::MotorSimulator>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("motor_simulator"), "%s", error.what()); result = 1;
  }
  rclcpp::shutdown();
  return result;
}
