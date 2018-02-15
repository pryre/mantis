# mantis_paths
This node will generate a velocity-dependant path in 3D space.

## Usage
`roslaunch mantis_paths generate.launch path_name:=fig8.yaml`

## Output
The path output uses the message type `nav_msgs/Path` to define a list of poses for a robot to follow.

Each pose is timestamped relative to the `header.stamp` in the `Path` message. As such, each pose represents where the robot should be at a specific time. Add the pose's stamp to the path's stamp to get the desired pose of the robot in real time.

The difference in pose timestamps represents can be interpolated for greater tracking accuracy, and the difference in pose timestamps can be used to derive the desired velocity (assumes that the velocity is in the direction of the line p(t-1) to p(t).

## Generating a Path
A path is generated from parameters (loaded with the launch file from the `.yaml` files) by first setting a starting point, then making lines and arcs from one location to the next. Examples can be found in the launch file.

The parameters are set as follows:

#### start/position
A set of XYZ coordinates for the path to start at.

#### start/alpha
The starting orientation of the path in radians.

#### path/sX
A description of a path segment.

## Line segments
A line segment must have type `"line"` and can have the following properties:

#### velocity
The speed for the robot to travel along this segment.

#### length
The distance this segment will trace in XY space in meters.

#### height
A height modifier for the end of the line segment in meters. Setting this to a positive or negative value will make the end location higher or lower than the the starting location respectively.

## Arc segments
A arc segment must have type `"arc"` and can have the following properties:

#### velocity
The speed for the robot to travel along this segment.

#### hold_time
If the velocity for an arc is set to zero, the `hold_time` parameter will be read. This causes the robot to be instructed to hold the current location, however, changes in desired heading can still be made.

#### height
A height modifier for the end of the line segment in meters. Setting this to a positive or negative value will make the end location higher or lower than the the starting location respectively.

#### alpha
The angle of the arc segment to complete in radians. If this is set to

## The "theta" parameter
Both arc and line segments can have the theta parameter. It is an overide for the desired heading of the robot. This allows for the path to arc or change direction with the robot set to face a different direction.

If the theta parameter is not set, the current heading will be set using the previous robot heading (or alpha for the starting point) to set the new robot heading. This means that if the robot is instructed to complete and arc with no theta set, the robots heading will be rotated relatively to it's starting rotation by the `alpha` parameter of the arc over the entire duration of the arc. Example: If the robot is at -pi/2, and the arc is a rotation of +pi/4, the robot will perform a slerp rotation from -pi/2 to -pi/4 over the duration of the arc.

By having a very short hold of time for an arc segment, you can cause an nearly-instantaenous change in the current heading of the robot.
