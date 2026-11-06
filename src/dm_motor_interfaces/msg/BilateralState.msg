uint8 IDLE=0
uint8 ACQUIRING=1
uint8 ENABLING=2
uint8 ACTIVE=3
uint8 STOPPING=4
uint8 FAULT=5
uint8 NORMAL=0
uint8 DEGRADED=1
uint8 RECOVERING=2
std_msgs/Header header
uint8 phase
uint8 quality
string message
string motor_a
string motor_b
int32 direction
bool stop_confirmed
bool motion_limits_enabled
bool operator_guarded
bool pair_waiting
bool paired
bool commands_ready
float64 origin_a
float64 origin_b
float64 position_error
float64 relative_velocity
float64 pair_age
float64 pair_skew
uint64 pair_count
uint64 degradation_count
uint32 recovery_pairs
float64 recovery_scale
float64 gain_scale
float64 decoded_kp
float64 decoded_kd
float64 estimated_torque_a
float64 estimated_torque_b
float64 raw_torque_a
float64 raw_torque_b
MotorTarget target_a
MotorTarget target_b
uint64 command_count
# Friction models are resolved independently for each namespace and hardware/simulation identity.
string system_parameters_file
bool compensation_ready_a
bool compensation_ready_b
bool compensation_active
string compensation_model_a
string compensation_model_b
string compensation_source_a
string compensation_source_b
string compensation_message_a
string compensation_message_b
float64 compensation_velocity_min_a
float64 compensation_velocity_min_b
float64 compensation_velocity_max_a
float64 compensation_velocity_max_b
bool compensation_clamped_a
bool compensation_clamped_b
# Requested friction-only feedforward. estimated_torque above includes decoded PD plus feedforward.
float64 friction_torque_a
float64 friction_torque_b
# Zero application budget leaves MIT gains intact; protocol and driver checks remain.
bool application_torque_limited
# Estimated demand exceeds TMAX, not a measured device saturation flag.
bool protocol_saturated_a
bool protocol_saturated_b
bool driver_configuration_known_a
bool driver_configuration_known_b
MotorConfiguration driver_configuration_a
MotorConfiguration driver_configuration_b
# Known MIT position/velocity encoding residual correction, separate from friction.
float64 quantization_torque_a
float64 quantization_torque_b
# Exact MotorSetpoint.client_id / MotorState.active_source owned by this controller process.
string controller_source
# Explicit bounded pulse; canceled on STOP, lost session/heartbeat or feedback degradation.
# Axis and id identify the last accepted pulse; torque/remaining are zero when inactive.
bool disturbance_active
uint8 disturbance_axis
float64 disturbance_torque
float64 disturbance_remaining
uint64 disturbance_id
# Independent pulse components in the latest output, before torque-field quantization.
# target.torque = friction_torque + quantization_torque + disturbance_torque for each axis.
float64 disturbance_torque_a
float64 disturbance_torque_b
# Virtual relative-angle reference only; original position_error remains the raw physical relative error.
# A positive applied offset shifts A negative and B positive in the mapped direction;
# shared_reference distributes half of the offset to each reference.
# This closes the virtual gap, not necessarily the physical gap or a fixed arithmetic midpoint.
bool angle_offset_active
float64 angle_offset_target
# Value used in the latest controller output, radians; cleared only within the stop/reset lifecycle.
float64 angle_offset_applied
bool angle_offset_ramping
uint64 angle_offset_id
# Seconds left in transition plus hold; expiry requests a dual STOP rather than a reverse step.
float64 angle_offset_remaining
# position_error + angle_offset_applied; independent from raw encoder positions and captured origins.
float64 virtual_position_error
string control_mode
# Required V_BW in Hz; 0 means no startup register requirement.
float64 required_velocity_bandwidth
# True only after both current START reads and post-read disabled feedback were verified.
bool velocity_bandwidth_verified
# Current START readbacks, or 0 when unknown/canceled; not a continuous register monitor.
float64 motor_a_velocity_bandwidth
float64 motor_b_velocity_bandwidth
