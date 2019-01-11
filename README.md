# mantis
ROS meta-package for a aerial manipulators.

## Running the full SITL environment

### For a Single UAV


#### All-in-One`
1. `roscore`
2. `roslaunch mantis_description mantis_sim_quad_full_feed.launch`
6. `roslaunch mantis_planner plan_loader.launch move:=home`

#### Individual Terminals
1. `roscore`
2. `roslaunch mantis_description mantis_sim_quad.launch`
3. `roslaunch mantis_description robin_sim_udp.launch`
4. `roslaunch mantis_description mantis_sim_controller_feed.launch`
6. `roslaunch mantis_planner plan_loader.launch move:=home`

### For Multiple UAVs

#### Common
1. `roscore`
2. `roslaunch mantis_gazebo mantis_sim_gazebo.launch`

#### Per-UAV
Change `mantis_uav_X`, `spawn_x`, and `spawn_y` in the following to match your UAV:
1. `roslaunch mantis_description mantis_sim_quad.launch model_name:=mantis_uav_X spawn_x:=0.0 spawn_y:=0.0 gazebo:=false`
2. `roslaunch mantis_description robin_sim_udp.launch model_name:=mantis_uav_X`
3. `roslaunch mantis_description mantis_sim_controller_feed.launch model_name:=mantis_uav_X`
5. `roslaunch mantis_planner plan_loader.launch model_name:=mantis_uav_X move:=home`
