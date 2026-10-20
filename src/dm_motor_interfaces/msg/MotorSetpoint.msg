std_msgs/Header header
string client_id
uint64 request_id
string token
uint64 sequence
float64 valid_for
MotorTarget target
# No exclusive lease/token required for streams; latest accepted source wins.
# false: continually refreshed stream; true: hold while this non-exclusive keepalive session is live.
bool hold_until_lease_expiry
