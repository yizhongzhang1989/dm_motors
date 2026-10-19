std_msgs/Header header
string client_id
uint64 request_id
string token
uint64 sequence
float64 valid_for
MotorTarget target
# false: continually refreshed stream; true: hold only while the owning lease is live.
bool hold_until_lease_expiry
