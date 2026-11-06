#pragma once
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

namespace dm_motor
{
// Software identity only. Lease credentials stay internal and are never
// published. Target values/sequences and lease renewal deadlines are not identity.
struct PositionObservationIdentity
{
  std::string source, lease;
  uint64_t task{0}, task_instance{0}, bus_epoch{0};
  bool operator==(const PositionObservationIdentity & other) const
  {
    return source == other.source && lease == other.lease && task == other.task &&
           task_instance == other.task_instance && bus_epoch == other.bus_epoch;
  }
};

class PositionObservationSchedule
{
public:
  static constexpr uint64_t period_ns = 8000000;  // First version: 125 Hz only.
  bool active() const {return active_;}
  bool matches(const PositionObservationIdentity & identity) const
  {return active_ && identity_ == identity;}
  uint64_t epoch() const {return epoch_;}
  uint64_t missed_slots() const {return missed_;}
  uint64_t completed_count() const {return completed_;}
  void clear() {active_ = false; next_due_ = 0;}
  void bind(PositionObservationIdentity identity, uint64_t now)
  {
    if (matches(identity)) {return;}
    if (identity.source.empty() || !identity.bus_epoch) {clear(); return;}
    identity_ = std::move(identity); active_ = true; ++epoch_; next_due_ = now;
  }
  bool take_slot(uint64_t now, uint64_t valid_until = std::numeric_limits<uint64_t>::max())
  {
    if (!active_ || now < next_due_) {return false;}
    const uint64_t skipped = (now - next_due_) / period_ns;
    missed_ += skipped; next_due_ += (skipped + 1) * period_ns;
    // Keep the cadence, but never start a20ms read at a known source deadline.
    // A same-identity renewal can make a later slot eligible without a new epoch.
    if (valid_until <= now || valid_until - now <= 20000000) {
      ++missed_; return false;
    }
    return true;  // At most one attempt, including after a long scheduling gap.
  }
  void missed() {++missed_;}  // Due but busy/locked/context-invalid, no new read.
  void completed() {++completed_;}  // Successful valid80 only, not cancellations.
  uint64_t next_id() {return ++id_;}
private:
  PositionObservationIdentity identity_;
  bool active_{false};
  uint64_t epoch_{0}, next_due_{0}, id_{0}, missed_{0}, completed_{0};
};
}  // namespace dm_motor
