# iRC ROS Bringup

This contains the basic, non-MoveIt2 launch files and ROS2 controller configs. 

When you want to use just a ReBeL arm the launch files here should suffice. 

## Install
### ReBeL standalone
 - nothing besides this repository required
### Mobile Platform
In case you are planning to use a mobile platform look at `irc_ros_navigation2` as well.
 - For the mobile platforms laser scanners install the following packages (Add them to your workspace folder)
   - [Sick300 ROS2](https://github.com/ajtudela/sicks300_2)
   - [IRA Laser Tools (humble PR)](https://github.com/nakai-omer/ira_laser_tools/tree/humble)
   - [rqt_robot_steering](https://github.com/ros-visualization/rqt_robot_steering) (Also in the ubuntu repos)

## Usage
You can start the robot driver as follows:

``` console
$ ros2 launch irc_ros_bringup rebel.launch.py rebel_version:=00 gripper:=schmalz_ecbpmi use_rviz:=false
```

Manual joint states then can be send in different ways. For actual applications please refer to the MoveIt pkg.

### Control via command line
Found [here](https://git.nilsschulte.de/nils/dynamixel_ros2_control). Change values accordingly.

#### Rebel 6DOF
``` console
$ ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {stamp: {sec: '"$(date '+%s + 6' | bc)"'}}, joint_names: ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0}}]}' --once
```

#### Rebel 4DOF
``` console
$ ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory '{header: {stamp: {sec: '"$(date '+%s + 6' | bc)"'}}, joint_names: ["joint1", "joint2", "joint3", "joint4"], points: [{positions: [0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 0}}]}' --once
```

#### Mobile platform (x linear and z angular)
``` console
$ ros2 topic pub /cpr_platform_controller/cmd_vel geometry_msgs/msg/TwistStamped '{header: {stamp: {sec: '"$(date '+%s + 6' | bc)"'}}, twist: { linear: {x: 0.0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}}' --once
```
or
``` console
$ ros2 topic pub /cpr_platform_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.0}}' --once
```

### ROS actions
``` console
$ ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory [...]
```

### Control via rqt_robot_steering (Mobile platform only)
Note: Currently only works with `geometry_msgs/msg/Twist`, while we want `TwistStamped` instead, PR is [here](https://github.com/ros-visualization/rqt_robot_steering/pull/14). In the meantime the type is set to `Twist` in the controller config file.
 - Select topic as `/cpr_platform_controller/cmd_vel`
 - W,A,S,D for directions
 - Space to reset speed to 0

## Launch files
Parameters are not all implemented yet, this is for future reference!
  - `rebel_base.launch.py` (Standalone ReBeL 6DOF/4DOF)
    - `use_rviz` Whether to start rviz (default: true)
    - `rviz_file` The rviz launch file, gets overwritten by nav2 launch files (default: `rebel.rviz`) (TODO)
    - `robot_name` (default: `igus_rebel_6dof`)
    - `robot_urdf` Option to overwrite the URDF/XACRO with a custom one, e.g. containing end-effectors, reduced joint ranges, ... (default: `irc_ros_description/config/[robot_name].urdf.xacro`)
    - TODO: xacro parameters for overwriting
    - `robot_controller_config` Option to overwrite the loaded controller file (default: `irc_ros_bringup/config/controller_[robot_name].yaml`)
    - `rebel_version` Whether the ReBeL is a pre, 00 or 01 model (default: `01`)
    - `gripper` Which gripper to load. Currently supports `none`, `schmalz_ecbpmi` and `ext_dio_gripper` (default: `none`)
    - `use_simtime` (default: false) TODO
    - `fake_robot` (default: false) TODO
    - `namespace` Namespace (TODO: test this)
  - `cpr_platform.launch.py` (Mobile Platform)
    - `use_rviz` Whether to start rviz (default: true)
    - `rviz_file` The rviz launch file, gets overwritten by nav2 launch files (default: `platform.rviz`)
    - `use_rqt_robot_steering` Start the robot steering tool (default: true)
    - `platform_name` (default: `cpr_platform_medium`)
    - `platform_urdf` TODO
    - TODO: xacro parameters for overwriting
    - `platform_controller_config` (default: irc_ros_bringup/config/controller_[plaform_name].yaml`)
    - `use_simtime` (default: false) TODO
    - `fake_robot` (default: false) TODO
    - `use_laser_scanners` Whether to start the Sick S300 laser scanner nodes (default: true)
    - `namespace` Namespace TODO
  - `rebel_on_platform.launch.py`
    - All of the above TODO

## Controller files
  - `controller_igus_rebel_*dof.yaml` implements position control via a JointTrajectoryController
  - `controller_cpr_platform_medium.yaml` implements velocity control via a DiffDriveController. While the position commands are more precise for repeatability, the use of velocity commands causes smoother and straighter driving.

## TODO
 - CRI specific controller which only sends the goal position without interpolation et al.?
   - Alternatively wait for a new EmbeddedControl software update to support continuous goal sending
