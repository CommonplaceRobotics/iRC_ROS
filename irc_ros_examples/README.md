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
**Only a collection of ideas for now**
 - Coffee delivery showcase application
 - IoT Coffeemachine, [official way](https://api-docs.home-connect.com/quickstart) or [reverse engineered way](https://github.com/osresearch/hcpy)
   - Second way is available offline, but maybe not a good thing to use something reverse engineered if we publish this?
 - Gripper(s)
   - "Dumb", form-fitting gripper for a specific cup,
   - Normal two-finger gripper
 - Coffee Cup localisation (marker or image recognition ?)
   - Camera (Webcam, Depth imaging like Oak-D, ...)
 - Test and film the process
 - Nav2 Behaviour Trees
 - Web interface for ordering


#### Questions
 - Can the CM fail during the process?
 - Different cups for different drinks possible?
 - Predefined cup position or detection?
 - Feedback for gripping success/failure
 - Positions hardcoded, markers, image detection, ... ?
 - Keep out zones?

### Links
 - https://navigation.ros.org/plugin_tutorials/docs/writing_new_bt_plugin.html
 - https://navigation.ros.org/tutorials/docs/using_groot.html
