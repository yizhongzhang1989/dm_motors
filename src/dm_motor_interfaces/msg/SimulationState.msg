std_msgs/Header header
uint8 motor_id
string can_bus_namespace
bool virtual_bus_confirmed
bool feedback_paused
bool enabled
uint8 status
float64 simulation_time
uint64 integration_steps
# Physical shaft angle never changes on device zero; encoder_position does.
float64 position
float64 encoder_position
float64 velocity
float64 motor_torque
float64 external_torque
float64 gravity_torque
float64 friction_torque
float64 contact_torque
float64 net_torque
# Constant load and linear joint spring, excluding gravity/contact/external inputs.
float64 load_torque
float64 mechanical_energy
float64 rotor_temperature
float64 mos_temperature
float64 bus_voltage
float64 external_remaining
string external_source
uint64 accepted_commands
uint64 rejected_commands
string error
# Persistent environment spring, independent of expiring hand/direct force inputs.
bool anchor_active
uint64 anchor_revision
float64 anchor_position
float64 anchor_bearing
float64 anchor_stiffness
float64 anchor_torque
float64 transient_torque
# Optional LuGre friction storage. These are simulated physical states, not
# quantities measured or inferred from the real motor's compact feedback.
float64 bristle_displacement
float64 bristle_energy
# Mechanical energy plus bristle energy; equal to mechanical_energy otherwise.
float64 total_storage
