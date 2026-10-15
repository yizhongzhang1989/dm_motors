# MIT units: rad, rad/s, Nm/rad, Nm/(rad/s), Nm.
# Publish continuously while enabled; old commands are never restored on enable.
std_msgs/Header header
float64 position
float64 velocity
float64 kp
float64 kd
float64 torque
