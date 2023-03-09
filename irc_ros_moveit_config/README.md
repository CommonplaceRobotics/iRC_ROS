# iRC MoveIt2 package

**This is very much WIP. While it appears to run fine the current state is insufficiently tested to let it run without being very cautios**

## Usage
Start MoveIt with rviz with the following command:

``` bash
$ LC_NUMERIC=en_US.UTF-8 ros2 launch irc_ros_moveit_config rebel.launch.py gripper:="ext_dio_gripper"
```

Only gripper and use_rviz parameters are currently supported. The gripper argument changes which controllers are loaded, but the robot model is not channged at the moment.

## TODO
 - Which parts of the launch file can be included from bringup or the moveit share?
 - How easy is it to make this modular?
 - Especially regarding community provided moveit files for other kinematics
 - How extensive should the launch files be? Include warehouse, db, ... stuff as well?
 - turn srdf into a xacro file?
 - Better urdf loading to include parameters