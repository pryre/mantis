# mantis
ROS meta-package for a aerial manipulators.

## Package Summary
The following packages make up the MM-UAV framework. Although not directly listed, many of the packages are managed through Dynamic Reconfigure, and allow for a large amount of runtime tuning / customisation.

### Core Packages
These packages provide the core functionality of the MM-UAV framwork. There is an emphasis on providing a centralised mechanism for distributing parameter and state information to the control and add-on packages.

- `mantis_description`: Collection of resouces that are used throughout the rest of the meta-package. Includes launch files, meshes, demo scripts, etc.
- `mantis_params`: Acts as a centrailised parameter server (and client for other nodes). Interfaces with the ROS parameter server, and can be used to reload MM-UAV configurations "live".
- `mantis_state`: Acts as a centrailised parameter state (and client for other nodes). Collects state information from standard ROS sources (e.g. PoseStamped, TwistStamped, JointState) and outputs a syncronised "current system state" message. Also syncrhonises with a parameter client to allow controllers, etc., to ensure that the current state matches the current parameter configuration.
- `mantis_kinematics`: This package serves two functions:
  - Provides a solver library that can be queiried from another node to get details like "What is the current end effector transform?". This is syncronised with both the param and state clients.
    - **This library currently lacks the functionality to correctly calculate the Coriolis terms**, and as such, these values are ignored. This seems to work fine under the assumption of "slow" (read: not highly dynamic) movements.
  - Provides a forward kinematic node that outputs the current TF tree, such that it can be viewed in RVIZ with the robot_description pacakage.
- `mantis_msgs`: A collection of common messages specific to this framework.
- `mantis_gazebo`: A collection of gazebo plugins that are used to interface with ROS.

### Guidance
The guidance nodes are used to provide the "current referrence" for the control system. This may not be ideal for all controllers, but allows for some additional modularity.
- `mantis_guidance_full`: Provides a full reference state for the system (base and all joints), and can be used to perform tasks such as end-effector tracking. This node provides a minimal UI through dynamic reconfigure. **You will probably want to use this node for guidance tasks**.
- `mantis_guidance_base`: Similar to `mantis_guidance_full`, but only considers the reference for the robot base.
- `mantis_guidance_joints`: Similar to `mantis_guidance_full`, but only considers the reference for the manipulator joints.

### Controllers
The following decoupled controllers are presented as different options to achieve the task of controlling the system. All of the presented controllers work to control attitude. The additional high-level position controller, `mavel`, performs the task of generating a desired thrust vector.
- `mantis_controller_acro`: A naive decoupled multirotor controller (simplified from methods used in projects like PX4)
- `mantis_controller_id`: An inverse dynamics controller (technically a CTC method). **This method is the most accurate if good inertial measurements are provided**.
- `mantis_controller_feed`: A hybrid controller that calculates additional thrust values that would be requried to provide manipulator compensation
- `mantis_controller_joints`: Spawns a set of decoupled joint controllers, one for each manipulator joint.

### Add-on Packages
Other packages that are used in demonstrations, etc. May not be working as expected.
- `mantis_planner`: A set of tools that allow for high-level path planning to occur.
- `rqt_mantis_commander`: An RQT GUI that allows a user to issue "Go To" commands to the system.
- `mantis_target_detector_ball`: Simple image processer to detect a ball target.
- `mantis_target_detector_baton`: Simple image processer to detect a batton target.
- `mantis_tools`: Not actually a ROS package, but a loose workspace for other assorted tools. Avoid using these, as they may not be up to date or even correct.


## Setup
The framework can be configured and installed with a combination `rosinstall` and `catkin_make`.

#### ROS Binary Packages
(Unfortunately this listing is not yet complete, please resolve issues as usual during catkin build later on).

#### QUTAS Packages
This framework build off a set of base packages from the QUTAS Flight Stack. Please refer [here](https://github.com/qutas/info/wiki/UAV-Setup-Guides-(QUTAS-Flight-Stack)) for instructions on getting these packages.

Required QUTAS package groups:
- `qfs_base`
- `qfs_fc`

Recommended QUTAS package groups:
- `qfs_extra`
- `qfs_viz`

#### Mantis (and Other) Packages
```sh
cd ~/catkin_ws/
curl https://raw.githubusercontent.com/pryre/mantis/master/mantis_tools/mantis.rosinstall > /tmp/mantis.rosinstall
wstool merge -t src /tmp/mantis.rosinstall
```

#### Build the Framework:
```sh
cd ~/catkin_ws/
catkin_make
```

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
