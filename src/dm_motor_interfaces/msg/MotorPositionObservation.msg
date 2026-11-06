# Independent, optional p_m (register 80) observation during native MIT control.
# This is not compact feedback, a command ACK, or an absolute-position anchor.
uint8 COMPLETE=1
uint8 TIMEOUT=2
uint8 CANCELLED=3
uint8 AMBIGUOUS=4
std_msgs/Header header
uint8 state
bool valid
bool session_locked
# False until firmware sampling semantics are independently established.
bool device_sample_time_known
string message
string source_id
uint8 motor_id
uint64 task_id
uint64 observation_id
# Changes with control source, lease identity, task, or observation context.
# No lease credential is published.
uint64 observation_epoch
uint64 bus_epoch
string profile
float64 direction
float64 software_offset
float64 mapping_p
float64 mapping_v
float64 mapping_t
uint32 raw_value
float64 native_position
# Host monotonic nanoseconds, not device sample/execution time. TX is recorded
# after the broker transport send returns; RX is host parser acquisition.
uint64 requested_steady
uint64 tx_steady
uint64 rx_steady
uint64 completed_steady
# Cumulative node counters. Busy slots are skipped; they are never caught up.
uint64 missed_slots
uint64 completed_count
