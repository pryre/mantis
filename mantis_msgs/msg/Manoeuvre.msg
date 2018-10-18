std_msgs/Header header

uint8 MODE_DISCRETE = 0
uint8 MODE_CONTINUOUS = 1

uint8 mode

float64 nominal_velocity
float64 nominal_rate
float64 nominal_joint_rate

duration c_duration

string[] joint_names

mantis_msgs/Movement[] movements
