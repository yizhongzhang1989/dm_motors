#pragma once

#include <dm_motor_control/core.hpp>
#include <array>
#include <deque>
#include <optional>
#include <string>

namespace dm_motor_bilateral_control
{
enum class ControlMode {Native, SharedReference, DifferentialTorque};
enum class CouplingMode {Bilateral, ALeadsB};

struct Settings
{
  int direction{1};
  ControlMode control_mode{ControlMode::Native};
  // ALeadsB is differential-only: A has no host spring and uses its own
  // filtered velocity reference. B keeps the bilateral reference. Shared torque budgets
  // are forbidden so B's demand cannot rescale A's native damping.
  CouplingMode coupling_mode{CouplingMode::Bilateral};
  double stiffness{50}, coupling_damping{0.995}, local_damping{0.005};
  // Zero disables the application's total PD/feedforward budget, not driver limits.
  double max_torque{0};
  bool friction_compensation_enabled{false};
  double friction_compensation_fraction{0.6}, friction_torque_limit{0.2};
  // Optional controller gains, separate from the qualified calibration model.
  // Nonzero gains require an explicit speed ceiling and measured-velocity input.
  std::array<double, 2> extra_viscous{};
  double extra_viscous_velocity_limit{0};
  bool motion_limits_enabled{false};
  double max_feedback_torque{1}, max_travel{0.3}, warning_temperature{45}, max_temperature{55};
  double feedback_age{0.02}, pair_skew{0.005}, feedback_stop{0.08}, recovery_time{0.1};
  void validate() const;
};

// Settings are effective coupling coefficients; these are the actual unscaled MIT gains.
struct NativeGains {double kp, kd;};
NativeGains native_gains(const Settings & settings);

struct Sample
{
  double time{0}, position{0}, velocity{0}, torque{0};
  uint64_t sequence{0};
};

// Positions reconstructed at one time from already received samples. This is
// deliberately not a Sample: no interpolated velocity/torque enters damping or
// friction compensation. Bounds are enforced by FeedbackPairs' factory below.
struct CommonTimePositions
{
  double time{0};
  std::array<double, 2> position{}, bracket_gap{}, latest_age{};
};

// Only velocity feedforward is filtered; local MIT damping stays intact.
// ALeadsB uses A's own filtered velocity for A's feedforward reference.
class PeerVelocityFilter
{
public:
  // The first sample establishes the clock with zero output; duplicate times do not advance it.
  double update(double velocity, double acquisition_time);
  void reset();
private:
  std::optional<double> acquisition_time_;
  double filtered_{0};
};

// One bounded pulse, timed by the monotonic controller clock. Cancellation never queues a retry.
class DisturbancePulse
{
public:
  uint64_t start(uint8_t axis, double torque, double duration, double now);
  std::array<double, 2> update(double now, bool permitted);
  void cancel();
  double remaining(double now) const;
  bool active() const {return active_;}
  uint8_t axis() const {return axis_;}
  double torque() const {return active_ ? torque_ : 0;}
  uint64_t id() const {return id_;}
private:
  bool active_{false};
  uint8_t axis_{0};
  uint64_t id_{0};
  double torque_{0}, deadline_{0};
  std::optional<double> last_time_;
};

// A virtual relative reference. Expiration never clears an applied offset while enabled:
// the caller must stop both motors, then cancel. A new START never resumes a canceled offset.
class AngleOffset
{
public:
  uint64_t start(double offset, double transition_time, double hold_duration, double now);
  double update(double now);
  bool expired(double now) const;
  double remaining(double now) const;
  void cancel();
  bool active() const {return active_;}
  bool ramping() const {return ramping_;}
  double target() const {return target_;}
  double applied() const {return applied_;}
  uint64_t id() const {return id_;}
private:
  void check_time(double now) const;
  bool active_{false}, ramping_{false};
  uint64_t id_{0};
  double target_{0}, applied_{0}, started_{0}, transition_{0}, deadline_{0};
  std::optional<double> last_time_;
};

struct AxisLimits
{
  dm_motor::Mapping mapping;
  dm_motor::Limits command;
  double direction{1}, offset{0}, position_min{-12.5}, position_max{12.5};
  void validate() const;
};

struct Output
{
  std::array<dm_motor::Command, 2> targets;
  std::array<double, 2> estimated_torque{}, raw_torque{};
  std::array<double, 2> friction_torque{}, quantization_torque{}, disturbance_torque{};
  // DifferentialTorque only: keep the host spring separate from friction/pulses.
  std::array<double, 2> coupling_torque{};
  double coupling_scale{1};
  // Demand exceeds the encoding range or differential spring headroom; not measured motor output.
  std::array<bool, 2> protocol_saturated{};
  double gain_scale{0}, decoded_kp{0}, decoded_kd{0};
};

std::array<dm_motor::Command, 2> relative_targets(
  const std::array<Sample, 2> & pair, const std::array<double, 2> & origins, const Settings & settings);
// Predict toward each local acquisition time, limited to +/-2 ms without changing sample times.
std::array<dm_motor::Command, 2> relative_targets(
  const std::array<Sample, 2> & pair, const std::array<Sample, 2> & latest,
  const std::array<double, 2> & origins, const Settings & settings);
// SharedReference or DifferentialTorque: positions/times are latest received samples. Each velocity is
// already bin-centered and filtered on that axis' acquisition clock. No prediction.
// ALeadsB changes only A's velocity reference; its unused position retains the
// common center so B's differential spring and angle-offset geometry stay intact.
std::array<dm_motor::Command, 2> shared_reference_targets(
  const std::array<Sample, 2> & latest_reference,
  const std::array<double, 2> & origins, const Settings & settings);
// Shift only positions so e_virtual = e_raw + offset; peer velocity filtering/local MIT gains stay intact.
std::array<dm_motor::Command, 2> apply_angle_offset(
  const std::array<dm_motor::Command, 2> & targets, double offset, int direction);
// Native shifts by the full offset, shared/differential modes by half; effective stiffness is unchanged.
std::array<dm_motor::Command, 2> apply_angle_offset(
  const std::array<dm_motor::Command, 2> & targets, double offset, const Settings & settings);
// The quintic is monotonic, so endpoint checks cover the entire reference segment at this feedback snapshot.
// Moving feedback is validated again through symmetric_targets on every control tick.
void validate_angle_offset_path(
  const std::array<dm_motor::Command, 2> & targets, const std::array<Sample, 2> & latest,
  const std::array<AxisLimits, 2> & limits, const Settings & settings, double offset);
// desired[i].t is friction-only feedforward in joint coordinates; disturbance is separately budgeted.
// Only DifferentialTorque consumes spring_positions. Omission retains the old
// latest-position calculation for offline baselines/neutral validation; an active
// hardware caller must explicitly obtain a valid common-time result or stop.
Output symmetric_targets(
  const std::array<dm_motor::Command, 2> & desired, const std::array<Sample, 2> & latest,
  const std::array<AxisLimits, 2> & limits, const Settings & settings, double recovery_scale,
  const std::array<double, 2> & disturbance = {},
  const std::optional<CommonTimePositions> & spring_positions = std::nullopt);
void check_motion(const Sample & sample, double origin, double temperature, const Settings & settings);

class FeedbackPairs
{
public:
  void push(size_t axis, const Sample & sample);
  void reset(double after = 0);
  bool select(double now, double age, double skew);
  // Prefer min(latest times); if that time has no valid bracket, search older
  // received sample endpoints for the latest jointly valid time. Normal-path
  // results are unchanged. Search is bounded by the two retained histories.
  // Never extrapolate, consume a pair, or silently return unaligned positions.
  // No valid intersection returns nullopt; invalid bounds/latest future time throw.
  // Every contributing bracket endpoint must satisfy maximum_age. Repeated
  // calls may hold the same common time only while these bounds remain valid.
  // Exact sample endpoints need no interpolation and retain gap=0 semantics.
  // Optional rejection text retains the preferred time's first failure reason;
  // it is cleared on success. Axis numbers in diagnostics are one-based.
  std::optional<CommonTimePositions> common_time_positions(
    double now, double maximum_age, double maximum_bracket_gap,
    std::string * rejection_reason = nullptr) const;
  const std::optional<std::array<Sample, 2>> & pair() const {return pair_;}
  double age(double now) const;
  double skew() const;
private:
  std::array<std::deque<Sample>, 2> history_;
  std::array<double, 2> used_{};
  std::optional<std::array<Sample, 2>> pair_;
};

enum class Quality : uint8_t {Normal = 0, Degraded = 1, Recovering = 2};
struct Recovery
{
  Quality quality{Quality::Normal};
  double scale{1};
  uint32_t fresh_pairs{0};
  uint64_t degradation_count{0};
  void reset();
  void update(double pair_age, bool new_pair, double dt, double period, const Settings & settings);
};
}  // namespace dm_motor_bilateral_control
