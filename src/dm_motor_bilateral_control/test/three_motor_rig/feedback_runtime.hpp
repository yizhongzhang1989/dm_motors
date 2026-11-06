#pragma once

#include <dm_motor_control/runtime.hpp>
#include <dm_motor_control/simulation/device.hpp>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

namespace three_motor_rig
{
// Reuse production startup anchoring and compact-position unwrapping. This
// transport has no socket, ROS, hardware or external command destination.
class FeedbackRuntime
{
  class MemoryTransport final : public dm_motor::Transport
  {
  public:
    explicit MemoryTransport(std::array<dm_motor::simulation::VirtualMotor, 3> & devices)
    : devices_(devices) {}
    void send(const dm_motor::Frame & frame) override
    {
      if (!bootstrap) {++discarded_commands; return;}
      // Startup/enable is performed on the actual three disabled VirtualMotors.
      // Their physical clock remains zero throughout the independent host setup.
      for (auto & device : devices_) {
        const auto replies = device.receive(frame, 0);
        frames.insert(frames.end(), replies.begin(), replies.end());
      }
    }
    std::vector<dm_motor::Frame> receive() override
    {
      std::vector<dm_motor::Frame> result;
      result.swap(frames);
      return result;
    }
    bool bootstrap{true};
    unsigned discarded_commands{0};
    std::vector<dm_motor::Frame> frames;
  private:
    std::array<dm_motor::simulation::VirtualMotor, 3> & devices_;
  };

public:
  explicit FeedbackRuntime(std::array<dm_motor::simulation::VirtualMotor, 3> & devices)
  {
    std::vector<dm_motor::MotorConfig> configs;
    for (size_t i = 0; i < devices.size(); ++i) {
      auto config = devices[i].parameters().motor;
      config.name = "axis_" + std::to_string(i + 1);
      names_[i] = config.name;
      configs.push_back(config);
    }
    auto transport = std::make_unique<MemoryTransport>(devices);
    transport_ = transport.get();
    bus_ = std::make_unique<dm_motor::Bus>(std::move(transport), configs, true);
    unsigned step = 0;
    while (!bus_->ready() && step < 3000) {bus_->tick(++step * .001);}
    if (!bus_->ready()) {
      throw std::runtime_error("Offline feedback Bus startup failed: " + bus_->initialization_error());
    }
    // Refresh all disabled feedback before enabling, exactly as runtime requires.
    for (unsigned i = 0; i < 3; ++i) {bus_->tick(++step * .001);}
    for (const auto & name : names_) {bus_->enable(name, step * .001);}
    bus_->poll_feedback(++step * .001);
    epoch_ = step * .001;
    for (const auto & name : names_) {
      const auto state = bus_->snapshot(name, epoch_);
      if (state.phase != dm_motor::Phase::Enabled || !state.absolute_position_valid) {
        throw std::runtime_error("Offline feedback Bus did not enable with a valid absolute anchor");
      }
    }
    transport_->bootstrap = false;
  }

  dm_motor::MotorSnapshot receive(size_t axis, dm_motor::Frame frame, double received)
  {
    // A fixed epoch maps the fixture clock onto Bus's completed startup clock.
    // Production unwrapping uses the original transport receive timestamp.
    frame.received_steady = static_cast<uint64_t>(std::llround((epoch_ + received) * 1e9));
    transport_->frames.push_back(frame);
    bus_->poll_feedback(epoch_ + received);
    const auto state = bus_->snapshot(names_.at(axis), epoch_ + received);
    if (!state.absolute_position_valid || state.host_fault || state.device_fault ||
      state.phase != dm_motor::Phase::Enabled || transport_->discarded_commands)
    {
      throw std::runtime_error("Offline feedback Bus rejected sample: " + state.error);
    }
    return state;
  }

private:
  std::array<std::string, 3> names_;
  MemoryTransport * transport_{};
  std::unique_ptr<dm_motor::Bus> bus_;
  double epoch_{};
};
}  // namespace three_motor_rig
