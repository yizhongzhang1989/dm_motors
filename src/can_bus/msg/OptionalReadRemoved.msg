# Broker evidence, never a CAN transaction or a generic enqueue acknowledgement.
# An authenticated registered STOP actually removed this still-queued priority3
# native register80 read before Router::drain selected it for transmission.
# Other rejection/expiry/fault/maintenance paths must never publish this event.
std_msgs/Header header
string token
uint64 epoch
uint64 sequence
uint64 stop_sequence
uint64 sent_steady
uint64 removed_steady
can_bus/Frame frame
