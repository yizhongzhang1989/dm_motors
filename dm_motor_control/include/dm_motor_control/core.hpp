#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace dm_motor
{
struct Frame
{
  uint16_t id{0};
  std::array<uint8_t, 8> data{};
};
struct Mapping {double p{12.5}, v{30.0}, t{10.0};};
struct Command {double p{0}, v{0}, kp{0}, kd{0}, t{0};};
struct Feedback
{
  uint8_t status{0};
  double p{0}, v{0}, t{0};
  uint8_t mos{0}, rotor{0};
};
struct Limits
{
  // Zero p/v/t selects the motor's full mapping, not a fixed model-specific cap.
  double p{0.0}, v{0.0}, t{0.0}, kp{500.0}, kd{5.0};
};
void validate_mapping(const Mapping & mapping);
Limits resolve_limits(const Mapping & mapping, Limits limits);
void validate_command(const Command & command, const Mapping & mapping, const Limits & limits);
Frame mit_frame(uint8_t id, const Command & command, const Mapping & mapping);
Command decode_mit(const Frame & frame, const Mapping & mapping);
Frame special_frame(uint8_t id, uint8_t opcode);
Frame query_frame(uint8_t id);
Frame feedback_frame(uint8_t id, uint16_t master, const Feedback & feedback, const Mapping & mapping);
std::optional<Feedback> decode_feedback(
  const Frame & frame, uint8_t id, uint16_t master, const Mapping & mapping);
std::array<uint8_t, 30> serial_packet(const Frame & frame);
class SerialParser
{
public:
  std::vector<Frame> feed(const uint8_t * data, size_t length);
private:
  std::vector<uint8_t> buffer_;
};
class Transport
{
public:
  virtual ~Transport() = default;
  virtual void send(const Frame & frame) = 0;
  virtual std::vector<Frame> receive() = 0;
};
class SerialTransport final : public Transport
{
public:
  explicit SerialTransport(const std::string & port);
  ~SerialTransport() override;
  void send(const Frame & frame) override;
  std::vector<Frame> receive() override;
private:
  int fd_{-1};
  SerialParser parser_;
};
class SimTransport final : public Transport
{
public:
  SimTransport(uint8_t id, uint16_t master, Mapping mapping);
  void send(const Frame & frame) override;
  std::vector<Frame> receive() override;
private:
  uint8_t id_;
  uint16_t master_;
  Mapping mapping_;
  Feedback feedback_;
  std::vector<Frame> pending_;
  double last_time_{0};
};
// Hardware initialization is performed only while disabled, outside the control loop.
Mapping initialize_serial(Transport & transport, uint8_t id, uint16_t master);
enum class Phase : uint8_t {Disabled, Enabling, Enabled, Stopping, Fault};
class Controller
{
public:
  Controller(Transport & transport, uint8_t id, uint16_t master, Mapping mapping,
    Limits limits, double command_timeout, double feedback_timeout);
  void tick(double now);
  void enable(double now);
  void disable(double now);
  void reset(double now);
  void command(const Command & command, double now);
  void fail(const std::string & reason);
  Phase phase() const {return phase_;}
  const Feedback & feedback() const {return feedback_;}
  const std::string & error() const {return error_;}
  const Limits & limits() const {return limits_;}
  bool fresh(double now) const;
  double age(double now) const;
  uint64_t feedback_count() const {return feedback_count_;}
private:
  Transport & transport_;
  uint8_t id_;
  uint16_t master_;
  Mapping mapping_;
  Limits limits_;
  double command_timeout_, feedback_timeout_;
  Phase phase_{Phase::Disabled};
  Feedback feedback_;
  Command target_;
  double last_feedback_{-1}, last_command_{-1}, transition_{0}, last_stop_{-1};
  uint64_t feedback_count_{0};
  std::string error_;
};
double steady_seconds();
}  // namespace dm_motor
