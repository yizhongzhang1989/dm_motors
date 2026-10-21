# Extra simulation-only input; never a real motor command.
# Output-shaft native coordinates, Nm. Positive increases native joint angle.
# No magnitude clamp; values must remain finite. Expiry returns the force to zero.
# Last valid source wins. Refresh while holding a force.
std_msgs/Header header
string source
uint64 sequence
float64 torque
float64 valid_for
# Optional compliant hand: evaluated at each physics substep, never a motor target.
# A false spring preserves the direct torque input above.
bool spring
float64 position
float64 stiffness
float64 damping
