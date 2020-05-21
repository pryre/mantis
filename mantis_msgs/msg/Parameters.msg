std_msgs/Header header

# Airframe name is the layout design of the multirotor base
# Supported options:
#	- "quad_x4"
#	- "quad_p4"
#	- "hex_x6"
#	- "hex_p6"
#	- "octo_x8"
#	- "octo_p8"
string airframe_name

uint16 pwm_min
uint16 pwm_max

float64 base_arm_length
float64 base_motor_cant
#float64 motor_kv
#float64 rpm_thrust_m
#float64 rpm_thrust_c
float64 motor_thrust_max
float64 motor_drag_max

# Body information
mantis_msgs/BodyInertial[] bodies

# Joint information
dh_parameters/JointDescription[] joints
