string token
uint64 epoch
uint64 sequence
builtin_interfaces/Duration valid_for
# 0 ordinary; 1 exact registered STOP; 2 maintenance; 3 optional motor position80 read.
# Priority3 requires inactive classical8-byte0x7ff [owned_nonzero_id,0,0x33,80,0,0,0,0].
# STOP cancels queued non-STOP traffic. Ordinary frames precede optional reads;
# every client retains its existing rate limit and at most one frame per drain.
uint8 priority
bool active
uint64 sent_steady
can_bus/Frame frame
