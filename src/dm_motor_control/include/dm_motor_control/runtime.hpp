#pragma once

#include "dm_motor_control/core.hpp"
#include <functional>

namespace dm_motor
{
enum class Mode : uint8_t {Mit = 1, PositionVelocity = 2, Velocity = 3, PositionForce = 4};
struct Target
{
  Mode mode{Mode::Mit};
  double position{0}, velocity{0}, kp{0}, kd{0}, torque{0}, velocity_limit{0},
    current_ratio{0};
};
struct RegisterInfo
{
  uint8_t rid;
  std::string name;
  bool integer;
  bool writable;
  std::string unit;
  std::optional<double> minimum, maximum;
  bool minimum_inclusive{true}, maximum_inclusive{true};
};
const std::vector<RegisterInfo> & register_catalog();
bool raw_register_write_allowed(uint8_t rid);
struct MotorConfig
{
  std::string name;
  uint8_t id{1};
  uint16_t master{0};
  Mode mode{Mode::Mit};
  bool configure_mode_on_startup{false};
  Mapping mapping{};
  Limits limits{};
  double command_timeout{0.25}, feedback_timeout{0.1}, watchdog{0.2};
};
struct MotorSnapshot
{
  Phase phase{Phase::Disabled};
  Mode mode{Mode::Mit};
  Mapping mapping{};
  // Mapping-resolved limits; float-position command range uses position_command_limit.
  Limits limits{};
  Feedback feedback{};
  double absolute_position{0};
  bool absolute_position_valid{false};
  int64_t position_wraps{0};
  double feedback_age{-1};
  uint64_t feedback_count{0};
  std::string error;
  bool host_fault{false};
  bool device_fault{false};
};
std::string motor_status_text(uint8_t status);
enum class OperationState : uint8_t {Pending, Confirmed, Rejected, Unknown};
struct RegisterOperation
{
  uint64_t id{0};
  OperationState state{OperationState::Pending};
  std::string motor;
  uint8_t rid{0};
  uint32_t raw_value{0};
  std::string message;
};
// Encoders use native CAN payload lengths. SerialTransport explicitly applies
// official u2can eight-byte zero-padding compatibility to four-byte payloads.
Frame target_frame(uint8_t id, const Target & target, const Mapping & mapping,
  const Limits & limits = {});
uint16_t control_id(uint8_t id, Mode mode);
double position_command_limit(Mode mode, const Mapping & mapping, const Limits & limits);
class Bus
{
public:
  Bus(std::unique_ptr<Transport> transport, std::vector<MotorConfig> motors, bool simulated);
  ~Bus();
  void tick(double now);
  MotorSnapshot snapshot(const std::string & name, double now) const;
  void set_feedback_callback(
    std::function<void(const std::string &, const MotorSnapshot &, double)> callback);
  void enable(const std::string & name, double now);
  void disable(const std::string & name, double now);
  void reset(const std::string & name, double now);
  void fail(const std::string & name, const std::string & reason);
  void transport_failed(const std::string & reason);
  void command(const std::string & name, const Target &, double now);
  // Maintenance is serialized and requires every motor disabled with fresh status.
  // The wire protocol cannot unambiguously separate arbitrary online reads from feedback.
  // Firmware RW descriptors do not imply generic-service permission: IDs, mapping,
  // bitrate, watchdog, mode and high-risk commissioning values are protected.
  uint64_t read_register(const std::string & name, uint8_t rid, double now);
  uint64_t write_register(const std::string & name, uint8_t rid, uint32_t raw, double now);
  uint64_t save_parameters(const std::string & name, double now);
  uint64_t set_mode(const std::string & name, Mode mode, double now);
  uint64_t clear_motor_fault(const std::string & name, double now);
  uint64_t set_device_zero(const std::string & name, double now);
  RegisterOperation operation(uint64_t id) const;
  const std::vector<MotorConfig> & configs() const;
  bool ready() const;
  std::string initialization_error() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
class SimBusTransport final : public Transport
{
public:
  explicit SimBusTransport(const std::vector<MotorConfig> & motors);
  ~SimBusTransport() override;
  void send(const Frame &) override;
  std::vector<Frame> receive() override;
  void inject_fault(uint8_t id, uint8_t status);
  void drop_replies(bool drop);
  void inject_frame(const Frame & frame);
  void set_position(uint8_t id, double position);
  void power_cycle(uint8_t id);
  uint32_t register_value(uint8_t id, uint8_t rid) const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
std::unique_ptr<Transport> make_sim_transport(const std::vector<MotorConfig> & motors);
// The interface must already be configured externally. fd selects FD framing;
// BRS remains an explicit per-frame flag. No network settings are changed here.
std::unique_ptr<Transport> make_socketcan_transport(const std::string & interface, bool fd = false);
}  // namespace dm_motor
