from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    irc_ros_moveit_dir = PathJoinSubstitution(
        [
            FindPackageShare("irc_ros_moveit_config"),
            "launch",
        ]
    )

    moveit_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([irc_ros_moveit_dir, "/rebel.launch.py"]),
        launch_arguments={
            "gripper": "ext_dio_gripper",
        }.items(),
    )

    pick_and_place_node = Node(
        package="irc_ros_examples",
        executable="pick_and_place",
        name="pick_and_place",
    )

    ld = LaunchDescription()
    ld.add_entity(moveit_node)
    ld.add_entity(pick_and_place_node)

    return ld
