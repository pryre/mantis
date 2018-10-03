std_msgs/Header header

string airframe_type
int16 pwm_min
int16 pwm_max

float64 base_arm_length
#float64 motor_kv
#float64 rpm_thrust_m
#float64 rpm_thrust_c
float64 motor_thrust_max
float64 motor_drag_max

# Body information
mantis_msgs/BodyInertial[] bodies

# Joint information
dh_parameters/JointDescription[] joints
