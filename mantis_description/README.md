# Mantis Description
Collection of resouces that are used throughout the rest of the meta-package. Includes launch files, meshes, demo scripts, etc.

## Launch File Summary

### Working

Note: you will likely have to configure the flight control SITL, [robin](https://www.github.com/qutas/robin) correctly before use. A set of "ready to go" parameters has been included in `mantis_description/config/robin_sim.params`, and can be loaded into the STIL flight controller using MAVROS.

#### Simulation
All-in-one launch files
- mantis_sim_quad_full_feed.launch
- mantis_sim_quad_full_id.launch

Controller/various subsystem launch files:
- mantis_sim_controller_feed.launch
- mantis_sim_controller_id.launch

Gazebo launch files
- mantis_sim_hex.launch
- mantis_sim_quad.launch

Flight controller and MAVROS connections:
- `robin_sim_udp.launch`
- `robin_sim_udp_2.launch`

#### Real World

System:
- `mantis_gcs_controller_feed.launch`
- `mantis_gcs_quad.launch`
- `mantis_onboard_controller_joints.launch`

MAVROS connections:
- `robin_telem.launch`
- `robin_udp.launch`
- `robin_usb.launch`
- `robin_wifi.launch`

### Unknown Working State
- `clutter_sim_nav.launch`
- `slung_load_sim_quad.launch`
- `marker_detect.launch`
- `marker_sim_detect.launch`


## Additional Examples

### Pipe tracking Demo
```sh
roscore
roslaunch mantis_description mantis_sim_quad_full_id.launch
rosrun mantis_planner bends_pipe
roslaunch mantis_planner plan_loader.launch move:=bends
```
