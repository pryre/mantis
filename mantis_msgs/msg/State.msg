# Mantis State Message

# Bitfield Constants
uint16 SENSOR_POSITION = 1
uint16 SENSOR_ORIENTATION = 2
uint16 SENSOR_VELOCITY_LINEAR = 4
uint16 SENSOR_VELOCITY_ANGULAR = 8
uint16 SENSOR_ACCEL_LINEAR = 16
uint16 SENSOR_ACCEL_ANGULAR = 32

uint16 SENSOR_JOINT_POSITION = 64
uint16 SENSOR_JOINT_VELOCITY = 128
uint16 SENSOR_JOINT_ACCEL = 256

uint16 SENSOR_VOLTAGE = 512

uint16 SENSOR_SAFETY = 16384 #Functionallity disabled, unavailable in current mavros
uint16 SENSOR_ARMING = 32768

# Header Information
std_msgs/Header header
string child_frame_id

# Time stamp to represent the parameter configuration used when the state was generated
time configuration_stamp

# Bitfields
uint16 status_updated_fields
uint16 status_sensor_health

# Base States (Inertial frame pose, body frame twist, body frame accel)
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Accel accel

# Manipulator States (Same order as joints listed by the parameters)
float64[] r
float64[] rd
float64[] rdd

# Subsystem States
float64 battery_voltage
#bool system_safety_disengaged
bool system_armed
