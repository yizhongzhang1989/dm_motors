#pragma once
#include "dm_motor_control/core.hpp"
#include <functional>
#include <rclcpp/rclcpp.hpp>
namespace dm_motor
{
class CanTransport final : public Transport
{
public:
  CanTransport(rclcpp::Node & node, const std::string & bus_namespace,
    const std::string & client_name, uint8_t motor_id, uint16_t master_id,
    bool fd, double rate, double timeout, const std::string & fault_group, bool require_virtual = false,
    bool allow_native_position_observation = false);
  ~CanTransport() override;
  void send(const Frame &) override;
  std::vector<Frame> receive() override;
  // Notification only: runs outside the transport lock and must not perform I/O.
  // The receiver remains the caller's single serialized runtime owner. A copied
  // notification may finish after replacement; capture lifetime-safe state.
  void set_receive_notification(std::function<void()> notification);
  bool ready() const;
  std::string error() const;
  std::string backend() const;
  bool simulated() const;
  bool maintenance_available() const override;
  bool activity_available() const override;
  bool prepare_maintenance() override;
  bool maintenance_blocked() const override;
  bool native_position_observation_supported() const override;
  bool try_send_native_position_read(const Frame &, uint64_t read_id, uint64_t expires_steady) override;
  uint64_t bus_epoch() const;
  std::vector<PositionReadDispatch> take_position_read_dispatches() override;
  std::vector<PositionReadRemoval> take_position_read_removals() override;
  void cancel_position_read(uint64_t read_id) override;
  void set_quiet(bool);
  void set_fault(bool);
  void spin_shutdown();
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
