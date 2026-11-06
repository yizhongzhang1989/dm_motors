#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace dm_motor_bilateral_control
{
// A queued register request is not a readback. Only its independently polled,
// confirmed result can satisfy this check. Transport request IDs are checked by
// MotorSession before a result reaches this class.
class VelocityBandwidthCheck
{
public:
  static constexpr uint8_t address = 0x20;

  static void validate(double required)
  {
    if (!std::isfinite(required) || required < 0 || required >= 500 ||
      (required > 0 && (static_cast<float>(required) <= 0 || static_cast<float>(required) >= 500)))
    {
      throw std::invalid_argument("required_velocity_bandwidth must be 0 (unchecked) or in (0, 500) Hz");
    }
  }

  void reset(double required)
  {
    validate(required);
    required_ = required; active_ = required > 0;
    operation_ = 0; value_ = 0; verified_ = false;
  }
  void cancel() {active_ = false; operation_ = 0; value_ = 0; verified_ = false;}
  uint64_t operation() const {return operation_;}
  double value() const {return value_;}
  bool verified() const {return active_ && verified_;}

  void queued(const std::vector<uint64_t> & operations)
  {
    if (!active_ || verified_ || operation_ || operations.size() != 1 || !operations[0]) {
      throw std::runtime_error("V_BW read requires exactly one new operation ID");
    }
    operation_ = operations[0];
  }

  bool observe(uint64_t operation, uint8_t state, uint8_t result_address, uint32_t raw)
  {
    if (!active_ || verified_ || !operation_ || operation != operation_) {
      throw std::runtime_error("V_BW result does not belong to the current read");
    }
    if (state == 0) {return false;}  // MotorReply::PENDING
    if (state != 1 || result_address != address) {  // MotorReply::CONFIRMED
      throw std::runtime_error("V_BW register read was not independently confirmed");
    }
    float measured;
    static_assert(sizeof(measured) == sizeof(raw));
    std::memcpy(&measured, &raw, sizeof(measured));
    if (!std::isfinite(measured) || measured != static_cast<float>(required_)) {
      throw std::runtime_error("Actual V_BW=" + std::to_string(measured) +
        " Hz differs from required " + std::to_string(required_) +
        " Hz; use the configured bilateral driver profile");
    }
    value_ = measured; verified_ = true;
    return true;
  }

private:
  double required_{0}, value_{0};
  uint64_t operation_{0};
  bool active_{false}, verified_{false};
};
}  // namespace dm_motor_bilateral_control
