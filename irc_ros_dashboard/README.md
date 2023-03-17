# iRC ROS Dashboard

This project contains a barebone dashboard with a text user interface. It is mostly intended for debugging purposes. Once the interfaces work and all desired functionality is achievable it may be superseded by a nicer RQt/Rviz plugin/Web interface. 

The ROS+TUI idea was inspired by [RTUI](https://github.com/eduidl/rtui).

## Usage
Requires [textual](https://github.com/Textualize/textual).

Define which modules to plot in the controllers .yaml configuration. The dashboard will then connect to the corrosponding controller interfaces and show the data in a table. In the future it will be possible to also send commands from there and showing more in-depth information, this is currently work in progress.

## Structure
Information is provided by DashboardController, DioController, JointStates and standard ros2_control controllers:

![The dataflow to and from the dashboard](doc/dashboard.png)

### Dashboard commands

![The dataflow of sending commands to the dashboard](doc/dashboard_process.png)