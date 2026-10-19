std_msgs/Header header
uint32 can_id
uint8 channel
bool extended
bool rtr
bool error
bool fd
bool brs
uint8 length
uint8[<=64] data
# Host monotonic acquisition nanoseconds; preserved when routing physical RX.
uint64 received_steady
