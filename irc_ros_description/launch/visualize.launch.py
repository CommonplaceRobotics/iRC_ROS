# Based on the visualize_franka script licensed under the Apache license which can be found here:
# https://github.com/frankaemika/franka_ros2/blob/develop/franka_description/launch/visualize_franka.launch.py

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Equals the name of the file in irc_ros_description/urdf/ except the file ending
    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="igus_rebel_6dof",
        description="Which robot to use",
    )

    # Required to concatenate name with .urdf.xacro
    xacro_filename_arg = DeclareLaunchArgument(
        "xacro_filename",
        default_value=[LaunchConfiguration("robot_name"), ".urdf.xacro"],
        description="Name of the .urdf.xacro file to use",
    )

    # Fully qualified path to the urdf/xacro file, can also be used directly
    # for files outside the irc_ros_description package.
    xacro_path_arg = DeclareLaunchArgument(
        "xacro_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("irc_ros_description"),
                "urdf",
                LaunchConfiguration("xacro_filename"),
            ]
        ),
        description="Path to the .urdf.xacro file to use",
    )

    rebel_version_arg = DeclareLaunchArgument(
        "rebel_version",
        default_value="01",
        choices=["pre", "00", "01"],
        description="If visualizing an igus ReBel, chose which version to use",
    )

    gripper_arg = DeclareLaunchArgument(
        "gripper",
        default_value="none",
        choices=["none", "schmalz_ecbpmi", "ext_dio_gripper"],
        description="Which gripper to attach to the flange",
    )

    xacro_path = LaunchConfiguration("xacro_path")
    # Parameters for rebel.urdf.xacro
    rebel_version = LaunchConfiguration("rebel_version")
    gripper = LaunchConfiguration("gripper")

    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_path,
            " rebel_version:=",
            rebel_version,
            " gripper:=",
            gripper,
        ]
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("irc_ros_description"), "rviz", "rebel.rviz"]
    )

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_file],
    )

    description = LaunchDescription()

    # Launch args
    description.add_action(robot_name_arg)
    description.add_action(xacro_filename_arg)
    description.add_action(xacro_path_arg)
    description.add_action(rebel_version_arg)
    description.add_action(gripper_arg)

    # Nodes
    description.add_action(robot_state_publisher_node)
    description.add_action(joint_state_publisher_gui_node)

    # Visualization
    description.add_action(rviz_node)

    return description
