<!-- This Source Code Form is subject to the terms of the Mozilla Public
   - License, v. 2.0. If a copy of the MPL was not distributed with this
   - file, You can obtain one at https://mozilla.org/MPL/2.0/. -->

# A-Star Planner Injection
#### Launch Simulator
roslaunch mantis_description mantis_sim_quad_full_feed.launch

#### Generate Obstacle
roslaunch uavusr_emulator emulator.launch
rosrun topic_tools relay /grid/real /grid

#### Launch Path Planner
roslaunch mantis_description clutter_sim_nav.launch

#### Load in High-Level Path Plan
roslaunch mantis_planner plan_loader.launch move:=square_scan planner:=manoeuvre_astar
