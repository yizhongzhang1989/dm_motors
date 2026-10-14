#include "dm_motor_control/core.hpp"

#include <cmath>
#include <stdexcept>

namespace dm_motor
{
Controller::Controller(Transport & transport, uint8_t id, uint16_t master, Mapping mapping,
  Limits limits, double command_timeout, double feedback_timeout)
: transport_(transport), id_(id), master_(master), mapping_(mapping),
  limits_(resolve_limits(mapping, limits)),
  command_timeout_(command_timeout), feedback_timeout_(feedback_timeout)
{
  if (id < 1 || id > 15 || master > 0x7ff) {throw std::invalid_argument("invalid motor/master ID");}
  const double values[] = {command_timeout, feedback_timeout};
  for (double value : values) {
    if (!std::isfinite(value) || value <= 0) {throw std::invalid_argument("timeouts must be positive");}
  }
}
double Controller::age(double now) const
{
  return last_feedback_ < 0 ? -1 : now - last_feedback_;
}
bool Controller::fresh(double now) const
{
  return last_feedback_ >= 0 && age(now) <= feedback_timeout_;
}
void Controller::fail(const std::string & reason)
{
  if (phase_ != Phase::Fault) {
    error_ = reason;
    last_stop_ = -1;
  }
  phase_ = Phase::Fault;
  target_ = {};
}
void Controller::enable(double now)
{
  if (phase_ != Phase::Disabled || !fresh(now) || feedback_.status != 0) {
    throw std::runtime_error("enable requires fresh disabled feedback and no latched fault");
  }
  target_ = {};
  last_command_ = now;
  transition_ = now;
  phase_ = Phase::Enabling;
  transport_.send(mit_frame(id_, target_, mapping_));
  transport_.send(special_frame(id_, 0xfc));
}
void Controller::disable(double now)
{
  target_ = {};
  if (phase_ != Phase::Fault) {phase_ = Phase::Stopping; transition_ = now;}
  transport_.send(special_frame(id_, 0xfd));
  last_stop_ = now;
}
void Controller::reset(double now)
{
  if (phase_ != Phase::Fault || !fresh(now) || feedback_.status != 0) {
    throw std::runtime_error("reset requires a latched fault and fresh disabled feedback");
  }
  error_.clear();
  target_ = {};
  phase_ = Phase::Disabled;
}
void Controller::command(const Command & c, double now)
{
  if (phase_ != Phase::Enabled) {
    throw std::runtime_error("command rejected: motor is not confirmed enabled");
  }
  validate_command(c, mapping_, limits_);
  target_ = c;
  last_command_ = now;
}
void Controller::tick(double now)
{
  for (const auto & f : transport_.receive()) {
    auto fb = decode_feedback(f, id_, master_, mapping_);
    if (!fb) {continue;}
    feedback_ = *fb;
    last_feedback_ = now;
    ++feedback_count_;
    if (fb->status > 1) {fail("motor fault code " + std::to_string(fb->status));}
    else if (phase_ == Phase::Enabling && fb->status == 1) {phase_ = Phase::Enabled;}
    else if (phase_ == Phase::Stopping && fb->status == 0) {phase_ = Phase::Disabled;}
    else if (phase_ == Phase::Disabled && fb->status == 1) {fail("unexpected external enable");}
    else if (phase_ == Phase::Enabled && fb->status == 0) {fail("motor unexpectedly disabled");}
  }
  if (phase_ == Phase::Enabled || phase_ == Phase::Enabling) {
    if (!fresh(now)) {fail("feedback timeout");}
    else if (now - last_command_ > command_timeout_) {fail("command publisher timeout");}
    else if (phase_ == Phase::Enabling && now - transition_ > feedback_timeout_) {
      fail("enable acknowledgement timeout");
    }
  } else if (phase_ == Phase::Stopping && now - transition_ > feedback_timeout_) {
    fail("disable acknowledgement timeout");
  }
  if (phase_ == Phase::Fault) {
    if (last_stop_ < 0 || now - last_stop_ >= 0.05) {
      transport_.send(special_frame(id_, 0xfd));
      last_stop_ = now;
    } else {transport_.send(query_frame(id_));}
  } else if (phase_ == Phase::Enabled) {
    transport_.send(mit_frame(id_, target_, mapping_));
  } else {
    transport_.send(query_frame(id_));
  }
}
}  // namespace dm_motor
