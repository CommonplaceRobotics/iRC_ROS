# iRC ROS Examples

This package showcases how to implement Igus Robot Control ROS2 for different usecases.


## Pick&Place (MoveIt)
![](doc/pick_and_place.gif)

This simple example showcases point-to-point, linear and axis movements, as well as how to interact with the gripper via its controller's service.

The process runs in an infinite loop.

For further MoveIt commands see [here](https://moveit.picknik.ai/humble/doc/examples/examples.html#using-moveit-directly-through-the-c-api).

### Usage
There are two ways to start this example:

``` bash
# Start MoveIt and the example node together
$ LC_NUMERIC=en_US.UTF-8 ros2 launch irc_ros_examples pick_and_place.launch.py

# Alternatively first start moveit with the right gripper parameter
$ LC_NUMERIC=en_US.UTF-8 ros2 launch irc_ros_moveit_config rebel.launch.py gripper:="ext_dio_gripper"

# Then, once the startup is complete, run:
$ ros2 run irc_ros_examples pick_and_place
```

## Pick&Place-Vacuum (MoveIt)
![](doc/pick_and_place_vacuum.gif)

Uses the Schmalz ECBPMI vacuum gripper for a vertical pick and place application.
The example showcases how to use the ECBPMI Controller with service calls.
The process picks up small objects out of a tray with uniform distances between the different slots and places them in an identical tray at another position.
Each tray is defined by their own coordinate frame, which requires a tf2 transform as MoveIt only works with the frame_id set via poseReferenceFrame in the pick_and_place_base.hpp.

The process runs in an infinite loop.

### Usage
There are two ways to start this example:

``` bash
# Start MoveIt and the example node together
$ LC_NUMERIC=en_US.UTF-8 ros2 launch irc_ros_examples pick_and_place_vacuum.launch.py

# Alternatively first start moveit with the right gripper parameter
$ LC_NUMERIC=en_US.UTF-8 ros2 launch irc_ros_moveit_config rebel.launch.py gripper:="schmalz_ecbpmi"

# Publish the tfs to tray_1 and tray_2
$ ros2 run tf2_ros static_transform_publisher [x] [y] [z] [r] [p] [y] base_link tray_1
$ ros2 run tf2_ros static_transform_publisher [x] [y] [z] [r] [p] [y] base_link tray_2

# Then, once the startup is complete, run:
$ ros2 run irc_ros_examples pick_and_place_vacuum
```

## Basic Navigation (Nav2 Simple Commander)
**WORK IN PROGRESS EXAMPLE**
This example should showcase simple movements with the mobile platform over the [Nav2 Python API](https://navigation.ros.org/commander_api/index.html). It is also useful for testing Nav2 parameters.

The platform should drive over the four corners of a square and end where it started. While it achieves that goal already, it is rather slow due to unnecessary turning and detours.

## BaristaBot (MoveIt + Nav2 + more)
**WORK IN PROGRESS EXAMPLE**
This example should showcase controlling a ReBeL via MoveItPy and the platform with Nav2 Simple Commander. The process is supposed to be a coffee brewing and delivery process, right now the Nav2 part does not work reliably and the pick and place locations are hardcoded, making it realiant on precise positioning of the platform.

**The example requires MoveItPy. See [here](https://github.com/ros-planning/moveit2/issues/2014#issuecomment-1532053275) how to install it**

### Links
 - https://navigation.ros.org/plugin_tutorials/docs/writing_new_bt_plugin.html
 - https://navigation.ros.org/tutorials/docs/using_groot.html
