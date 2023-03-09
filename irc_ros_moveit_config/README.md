# iRC MoveIt2 package

**This is very much WIP. While it appears to run fine the current state is insufficiently tested to let it run without being very cautios**

## Usage
Start MoveIt with rviz with the following command:

``` bash
$ LC_NUMERIC=en_US.UTF-8 ros2 launch irc_ros_moveit_config rebel.launch.py gripper:="ext_dio_gripper"
```

Only gripper and use_rviz parameters are currently supported. The gripper argument changes which controllers are loaded, but the robot model is not channged at the moment.