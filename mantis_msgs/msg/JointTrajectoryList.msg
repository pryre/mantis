std_msgs/Header header

# Defines the number of joints that the list contains
# also defines the number of points for each joint
uint64 num_joints
uint64 num_points

# Listing of joint interpolation points
# length should be: num_joints*num_points
float64[] joint_interp_points
