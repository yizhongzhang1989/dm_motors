#pragma once
#include "dm_motor_control/core.hpp"
#include <rclcpp/rclcpp.hpp>
namespace dm_motor
{
class CanTransport final : public Transport
{
public:
  CanTransport(rclcpp::Node & node, const std::string & bus_namespace,
    const std::string & client_name, uint8_t motor_id, uint16_t master_id,
    bool fd, double rate, double timeout, const std::string & fault_group);
  ~CanTransport() override;
  void send(const Frame &) override;
  std::vector<Frame> receive() override;
  bool ready() const;
  std::string error() const;
  std::string backend() const;
  bool simulated() const;
  bool maintenance_available() const override;
  bool activity_available() const override;
  bool prepare_maintenance() override;
  bool maintenance_blocked() const override;
  void set_quiet(bool);
  void set_fault(bool);
  void spin_shutdown();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
