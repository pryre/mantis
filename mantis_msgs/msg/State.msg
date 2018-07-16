std_msgs/Header header
string child_frame_id

# Time stamp to represent the parameter configuration used when the state was generated
time configuration_stamp

# Base States (Inertial frame pose, body frame twist, body frame accel)
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel accel

# Manipulator States
float64[] r
float64[] rd
float64[] rdd

# Subsystem States
float64 battery_voltage
bool flight_ready
